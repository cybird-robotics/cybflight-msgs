//! Wire-format packed structs for UART transport (ESP bridge).
//!
//! Separate from channel types (which use nalgebra). All downlink types carry
//! the original `Instant` timestamp as `timestamp_us: u64` (microseconds since boot).

/// Wire-format message identifiers.
pub mod msg_id {
    // Downlink (STM32 → ground)
    pub const IMU: u8 = 1;
    pub const VEHICLE_ATTITUDE: u8 = 2;
    pub const RC_INPUT: u8 = 3;
    pub const RC_LINK_STATUS: u8 = 4;
    pub const DSHOT_TELEMETRY: u8 = 5;
    pub const OCP_SOLVER_OUTPUT: u8 = 6;
    pub const IMU_2: u8 = 7;
    pub const GPS_FIX: u8 = 8;
    pub const MAG_EXT: u8 = 9;
    pub const MAG_INT: u8 = 10;
    pub const BARO_1: u8 = 11;
    pub const BARO_2: u8 = 12;
    pub const ATTITUDE_CONTROL_SETPOINT: u8 = 13;
    pub const MANUAL_CONTROL_SETPOINT: u8 = 14;
    pub const ARM_DISARM: u8 = 15;

    // Uplink (ground → STM32)
    pub const POSE: u8 = 128;
    pub const TIME_SYNC: u8 = 129;
    pub const PING_RESP: u8 = 130;

    // Downlink (STM32 → ground)  — time sync
    pub const PING: u8 = 16;
}

/// Maximum wire payload size in bytes.
pub const MAX_WIRE_PAYLOAD: usize = 64;
/// Frame header size: [msg_id: u8][seq: u8].
pub const FRAME_HEADER_SIZE: usize = 2;
/// Maximum raw (pre-COBS) frame size.
pub const MAX_FRAME_SIZE: usize = FRAME_HEADER_SIZE + MAX_WIRE_PAYLOAD;

/// Frame header preceding every wire message.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct FrameHeader {
    pub msg_id: u8,
    pub seq: u8,
}

/// Trait for wire-format message types.
///
/// # Safety
/// Implementors must be `#[repr(C, packed)]` with no padding.
pub trait WireMessage: Copy + Sized {
    const MSG_ID: u8;
    const SIZE: usize = core::mem::size_of::<Self>();

    fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < Self::SIZE {
            return None;
        }
        Some(unsafe { core::ptr::read_unaligned(bytes.as_ptr() as *const Self) })
    }

    fn as_bytes(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self as *const Self as *const u8, Self::SIZE) }
    }
}

// ---------------------------------------------------------------------------
// Downlink types (STM32 → ground)
// ---------------------------------------------------------------------------

/// Wire IMU data — 36 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireImu {
    pub timestamp_us: u64,
    pub accel: [f32; 3],
    pub gyro: [f32; 3],
    pub temp: f32,
}

impl WireMessage for WireImu {
    const MSG_ID: u8 = msg_id::IMU;
}

impl WireImu {
    pub fn from_msg(msg: &crate::Imu) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            accel: [msg.accel_m_s2.x, msg.accel_m_s2.y, msg.accel_m_s2.z],
            gyro: [msg.gyro_rad_s.x, msg.gyro_rad_s.y, msg.gyro_rad_s.z],
            temp: msg.temp_c,
        }
    }
}

/// Wire vehicle attitude — 24 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireVehicleAttitude {
    pub timestamp_us: u64,
    pub orientation: [f32; 4], // [x, y, z, w]
}

impl WireMessage for WireVehicleAttitude {
    const MSG_ID: u8 = msg_id::VEHICLE_ATTITUDE;
}

impl WireVehicleAttitude {
    pub fn from_msg(msg: &crate::VehicleAttitude) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            orientation: [
                msg.orientation.i,
                msg.orientation.j,
                msg.orientation.k,
                msg.orientation.w,
            ],
        }
    }
}

/// Wire RC input — 41 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireRcInput {
    pub timestamp_us: u64,
    pub channels: [u16; 16],
    pub channel_count: u8,
}

impl WireMessage for WireRcInput {
    const MSG_ID: u8 = msg_id::RC_INPUT;
}

impl WireRcInput {
    pub fn from_msg(msg: &crate::RcInput) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            channels: msg.channels,
            channel_count: msg.channel_count,
        }
    }
}

/// Wire RC link status — 13 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireRcLinkStatus {
    pub timestamp_us: u64,
    pub rssi_dbm: i16,
    pub link_quality: u8,
    pub snr: i8,
    pub rf_mode: u8,
}

impl WireMessage for WireRcLinkStatus {
    const MSG_ID: u8 = msg_id::RC_LINK_STATUS;
}

impl WireRcLinkStatus {
    pub fn from_msg(msg: &crate::RcLinkStatus) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            rssi_dbm: msg.rssi_dbm,
            link_quality: msg.link_quality,
            snr: msg.snr,
            rf_mode: msg.rf_mode,
        }
    }
}

/// Wire DShot telemetry — 32 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireDshotTelemetry {
    pub timestamp_us: u64,
    pub erpm: [u32; 4],
    pub raw: [u16; 4],
}

impl WireMessage for WireDshotTelemetry {
    const MSG_ID: u8 = msg_id::DSHOT_TELEMETRY;
}

impl WireDshotTelemetry {
    pub fn from_msg(msg: &crate::DshotTelemetry) -> Self {
        let mut erpm = [0u32; 4];
        let mut raw = [0u16; 4];
        for (i, m) in msg.motors.iter().enumerate() {
            erpm[i] = match m.value {
                crate::dshot::TelemetryValue::Erpm(e) => e,
                _ => 0,
            };
            raw[i] = m.raw.unwrap_or(0);
        }
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            erpm,
            raw,
        }
    }
}

/// Wire OCP solver output — 37 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireOcpSolverOutput {
    pub timestamp_us: u64,
    pub cmd: [f32; 4],
    pub iterations: i32,
    pub converged: u8,
    pub solve_time_us: u64,
}

impl WireMessage for WireOcpSolverOutput {
    const MSG_ID: u8 = msg_id::OCP_SOLVER_OUTPUT;
}

impl WireOcpSolverOutput {
    pub fn from_msg(msg: &crate::OcpSolverOutput) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            cmd: [
                msg.command[0],
                msg.command[1],
                msg.command[2],
                msg.command[3],
            ],
            iterations: msg.iterations,
            converged: msg.converged as u8,
            solve_time_us: msg.solve_time_us,
        }
    }
}

/// Wire GPS fix — 48 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireGpsFix {
    pub timestamp_us: u64,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_mm: i32,
    pub ground_speed_mm_s: u32,
    pub heading_mot_1e5: i32,
    pub fix_type: u8,
    pub num_sv: u8,
    pub h_acc_mm: u32,
    pub v_acc_mm: u32,
    pub pdop: u16,
}

impl WireMessage for WireGpsFix {
    const MSG_ID: u8 = msg_id::GPS_FIX;
}

impl WireGpsFix {
    pub fn from_msg(msg: &crate::GpsFix) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            lat_deg: msg.lat_deg,
            lon_deg: msg.lon_deg,
            alt_msl_mm: msg.alt_msl_mm,
            ground_speed_mm_s: msg.ground_speed_mm_s,
            heading_mot_1e5: msg.heading_mot_1e5,
            fix_type: msg.fix_type,
            num_sv: msg.num_sv,
            h_acc_mm: msg.h_acc_mm,
            v_acc_mm: msg.v_acc_mm,
            pdop: msg.pdop,
        }
    }
}

/// Wire magnetometer sample — 24 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireMagSample {
    pub timestamp_us: u64,
    pub field_ut: [f32; 3],
    pub temp_c: f32,
}

impl WireMessage for WireMagSample {
    const MSG_ID: u8 = msg_id::MAG_EXT;
}

impl WireMagSample {
    pub fn from_msg(msg: &crate::MagSample) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            field_ut: [msg.field_ut.x, msg.field_ut.y, msg.field_ut.z],
            temp_c: msg.temp_c,
        }
    }
}

/// Wire barometer sample — 16 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireBaroSample {
    pub timestamp_us: u64,
    pub pressure_pa: f32,
    pub temp_c: f32,
}

impl WireMessage for WireBaroSample {
    const MSG_ID: u8 = msg_id::BARO_1;
}

impl WireBaroSample {
    pub fn from_msg(msg: &crate::BaroSample) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            pressure_pa: msg.pressure_pa,
            temp_c: msg.temp_c,
        }
    }
}

/// Wire attitude control setpoint — 52 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireAttitudeControlSetpoint {
    pub timestamp_us: u64,
    pub collective_thrust_n: f32,
    pub attitude_quaternion: [f32; 4],
    pub body_rate_rad_s: [f32; 3],
    pub torque_n_m: [f32; 3],
}

impl WireMessage for WireAttitudeControlSetpoint {
    const MSG_ID: u8 = msg_id::ATTITUDE_CONTROL_SETPOINT;
}

impl WireAttitudeControlSetpoint {
    pub fn from_msg(msg: &crate::AttitudeControlSetpoint) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            collective_thrust_n: msg.collective_thrust_n,
            attitude_quaternion: [
                msg.attitude_quaternion.i,
                msg.attitude_quaternion.j,
                msg.attitude_quaternion.k,
                msg.attitude_quaternion.w,
            ],
            body_rate_rad_s: [
                msg.body_rate_rad_s.x,
                msg.body_rate_rad_s.y,
                msg.body_rate_rad_s.z,
            ],
            torque_n_m: [msg.torque_n_m.x, msg.torque_n_m.y, msg.torque_n_m.z],
        }
    }
}

/// Wire manual control setpoint — 24 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireManualControlSetpoint {
    pub timestamp_us: u64,
    pub thrust: f32,
    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,
}

impl WireMessage for WireManualControlSetpoint {
    const MSG_ID: u8 = msg_id::MANUAL_CONTROL_SETPOINT;
}

impl WireManualControlSetpoint {
    pub fn from_msg(msg: &crate::ManualControlSetpoint) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            thrust: msg.thrust,
            roll_rate: msg.roll_rate,
            pitch_rate: msg.pitch_rate,
            yaw_rate: msg.yaw_rate,
        }
    }
}

/// Wire arm/disarm state — 9 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireArmDisarm {
    pub timestamp_us: u64,
    pub armed: u8,
}

impl WireMessage for WireArmDisarm {
    const MSG_ID: u8 = msg_id::ARM_DISARM;
}

impl WireArmDisarm {
    pub fn from_msg(msg: &crate::ArmDisarm) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            armed: msg.armed as u8,
        }
    }
}

// ---------------------------------------------------------------------------
// Uplink types (ground → STM32)
// ---------------------------------------------------------------------------

/// Wire pose from ground station — 44 bytes.
///
/// Carries per-frame UTC timestamps for latency decomposition:
/// - `capture_time_us`: VICON camera capture time (gs_time - GetLatencyTotal())
/// - `gs_send_time_us`: GS PC time just before UDP send
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WirePose {
    pub capture_time_us: i64,
    pub gs_send_time_us: i64,
    pub position: [f32; 3],
    pub orientation: [f32; 4], // [x, y, z, w]
}

impl WireMessage for WirePose {
    const MSG_ID: u8 = msg_id::POSE;
}

impl WirePose {
    /// Convert to a `ViconPose` channel message using the local RX timestamp.
    pub fn to_vicon_pose(&self, rx_time: embassy_time::Instant) -> crate::ViconPose {
        let q = nalgebra::Quaternion::new(
            self.orientation[3], // w
            self.orientation[0], // x (i)
            self.orientation[1], // y (j)
            self.orientation[2], // z (k)
        );
        crate::ViconPose {
            timestamp: rx_time,
            position: nalgebra::Vector3::new(self.position[0], self.position[1], self.position[2]),
            orientation: nalgebra::UnitQuaternion::new_normalize(q),
            capture_time_utc_us: self.capture_time_us,
            gs_send_time_utc_us: self.gs_send_time_us,
        }
    }
}

/// PTP follow-up from ESP32 — 16 bytes.
///
/// Carries two NTP timestamps for clock offset computation:
/// - `esp_send_ntp_us`: ESP32 NTP time captured before UART write (primary estimate)
/// - `prev_tx_complete_ntp_us`: ESP32 NTP time captured after previous write completed
///   (refined estimate — eliminates UART delay estimation)
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireTimeSync {
    pub esp_send_ntp_us: i64,
    pub prev_tx_complete_ntp_us: i64,
}

impl WireMessage for WireTimeSync {
    const MSG_ID: u8 = msg_id::TIME_SYNC;
}

/// Ping request from STM32 — 12 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WirePing {
    pub ping_id: u32,
    pub stm32_send_us: u64,
}

impl WireMessage for WirePing {
    const MSG_ID: u8 = msg_id::PING;
}

/// Ping response from GS — 28 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WirePingResp {
    pub ping_id: u32,
    pub stm32_send_us: u64,
    pub gs_recv_time_us: i64,
    pub gs_send_time_us: i64,
}

impl WireMessage for WirePingResp {
    const MSG_ID: u8 = msg_id::PING_RESP;
}
