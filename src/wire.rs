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
    /// Fleet state: timestamped position + velocity of every drone in the
    /// swarm, broadcast by the ground station to each drone at 10 Hz so it
    /// knows where its neighbors are. See `WireFleetState`.
    pub const FLEET_STATE: u8 = 131;
    /// Peer pose: one drone's ESKF position as absolute geodetic lat/lon/alt +
    /// liveness status, forwarded **directly** leader→chaser between ESP bridges
    /// (bypassing the GS) for low-latency gimbal tracking. See `WirePeerPose`.
    pub const PEER_POSE: u8 = 132;

    pub const VEHICLE_ODOMETRY: u8 = 18;
    pub const POSITION_CONTROL_SETPOINT: u8 = 19;
    pub const ACTUATOR_MOTORS: u8 = 20;
    pub const GNSS_STATUS: u8 = 21;
    pub const MISSION_STATUS: u8 = 22;
    pub const ESTIMATOR_STATUS: u8 = 23;
    pub const MOTOR_STATE_TELEMETRY: u8 = 24;
    pub const POWER_STATUS: u8 = 25;
    /// Aggregate ESKF / failsafe / sensor-health snapshot. Mirrors
    /// the live `health` shell command and the blackbox `/health`
    /// topic. Downlinked at 1 Hz by `comm::esp_bridge`.
    pub const SYSTEM_HEALTH: u8 = 26;
    /// Flat snapshot of `GpsHealth` + the latest NAV-PVT accuracy
    /// fields. Mirrors the live `gpshealth` shell command and the
    /// blackbox `/gps_health` topic. Downlinked at 1 Hz.
    pub const GPS_HEALTH: u8 = 27;

    // Downlink (STM32 → ground)  — time sync
    pub const PING: u8 = 16;
    pub const TIME_SYNC_STATUS: u8 = 17;
}

/// Maximum wire payload size in bytes.
pub const MAX_WIRE_PAYLOAD: usize = 128;
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

/// Peer-pose status flags (`WirePeerPose::status`).
pub const PEER_FLAG_CONVERGED: u8 = 0x01;
pub const PEER_FLAG_3D_FIX: u8 = 0x02;

/// One drone's pose for direct peer-to-peer (leader→chaser) sharing — 21 bytes.
///
/// Carries the sender's **fused ESKF** position re-expressed as absolute geodetic
/// lat/lon/alt (so the receiver reprojects it into its own local ENU regardless
/// of where the sender anchored its frame — the origin cancels in
/// `leader − self`), plus liveness flags. Sent ~100 Hz directly between ESP
/// bridges; the receiver gates gimbal tracking on the flags + frame freshness.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WirePeerPose {
    /// Sender capture time, UTC microseconds (for end-to-end age / staleness).
    pub timestamp_us: u64,
    /// Latitude, degrees × 1e7.
    pub lat_1e7: i32,
    /// Longitude, degrees × 1e7.
    pub lon_1e7: i32,
    /// Altitude MSL, millimetres.
    pub alt_mm: i32,
    /// Bit0 `PEER_FLAG_CONVERGED` (ESKF converged); bit1 `PEER_FLAG_3D_FIX` (≥3D GNSS fix).
    pub status: u8,
}

impl WireMessage for WirePeerPose {
    const MSG_ID: u8 = msg_id::PEER_POSE;
}

const _: () = assert!(core::mem::size_of::<WirePeerPose>() <= MAX_WIRE_PAYLOAD);

impl WirePeerPose {
    /// Build from geodetic radians + altitude (m MSL) + liveness flags.
    pub fn new(
        timestamp_us: u64,
        lat_rad: f64,
        lon_rad: f64,
        alt_m: f32,
        converged: bool,
        has_3d_fix: bool,
    ) -> Self {
        const RAD2DEG: f64 = 180.0 / core::f64::consts::PI;
        Self {
            timestamp_us,
            lat_1e7: (lat_rad * RAD2DEG * 1e7) as i32,
            lon_1e7: (lon_rad * RAD2DEG * 1e7) as i32,
            alt_mm: (alt_m * 1000.0) as i32,
            status: (converged as u8) | ((has_3d_fix as u8) << 1),
        }
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

#[cfg(feature = "embedded")]
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

#[cfg(feature = "embedded")]
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

#[cfg(feature = "embedded")]
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

#[cfg(feature = "embedded")]
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

#[cfg(feature = "embedded")]
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

/// Wire motor state telemetry — 56 bytes.
///
/// Per-motor angular velocity (`omega`, rad/s) and angular acceleration
/// (`omega_dot`, rad/s²). `raw` carries the upstream raw sample (e.g. the
/// pre-filter eRPM-derived value); flattened to `0.0` when absent on the
/// channel side (mirrors `WireDshotTelemetry`).
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireMotorStateTelemetry {
    pub timestamp_us: u64,
    pub omega: [f32; 4],
    pub omega_dot: [f32; 4],
    pub raw: [f32; 4],
}

impl WireMessage for WireMotorStateTelemetry {
    const MSG_ID: u8 = msg_id::MOTOR_STATE_TELEMETRY;
}

#[cfg(feature = "embedded")]
impl WireMotorStateTelemetry {
    pub fn from_msg(msg: &crate::MotorStateTelemetry) -> Self {
        let mut omega = [0.0f32; 4];
        let mut omega_dot = [0.0f32; 4];
        let mut raw = [0.0f32; 4];
        for (i, m) in msg.motors.iter().enumerate() {
            omega[i] = m.omega;
            omega_dot[i] = m.omega_dot;
            raw[i] = m.raw.unwrap_or(0.0);
        }
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            omega,
            omega_dot,
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

#[cfg(feature = "embedded")]
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

#[cfg(feature = "embedded")]
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

#[cfg(feature = "embedded")]
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

#[cfg(feature = "embedded")]
impl WireBaroSample {
    pub fn from_msg(msg: &crate::BaroSample) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            pressure_pa: msg.pressure_pa,
            temp_c: msg.temp_c,
        }
    }
}

/// Wire battery / current monitor status — 20 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WirePowerStatus {
    pub timestamp_us: u64,
    /// Battery voltage in centivolts (0.01 V).
    pub voltage_cv: u16,
    /// Current draw in centiamps (0.01 A).
    pub current_ca: i32,
    /// Milliamp-hours consumed since power-on.
    pub mah_drawn: u32,
    /// Detected cell count (0 = no battery).
    pub cell_count: u8,
    pub _pad: u8,
}

impl WireMessage for WirePowerStatus {
    const MSG_ID: u8 = msg_id::POWER_STATUS;
}

#[cfg(feature = "embedded")]
impl WirePowerStatus {
    pub fn from_msg(msg: &crate::PowerStatus) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            voltage_cv: msg.voltage_cv,
            current_ca: msg.current_ca,
            mah_drawn: msg.mah_drawn,
            cell_count: msg.cell_count,
            _pad: 0,
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

#[cfg(feature = "embedded")]
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

#[cfg(feature = "embedded")]
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

#[cfg(feature = "embedded")]
impl WireArmDisarm {
    pub fn from_msg(msg: &crate::ArmDisarm) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            armed: msg.armed as u8,
        }
    }
}

/// Wire vehicle odometry (ESKF output) — 60 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireVehicleOdometry {
    pub timestamp_us: u64,
    pub position: [f32; 3],
    pub orientation: [f32; 4], // [x, y, z, w]
    pub velocity: [f32; 3],
}

impl WireMessage for WireVehicleOdometry {
    const MSG_ID: u8 = msg_id::VEHICLE_ODOMETRY;
}

#[cfg(feature = "embedded")]
impl WireVehicleOdometry {
    pub fn from_msg(msg: &crate::VehicleOdometry) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            position: [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ],
            orientation: [
                msg.pose.orientation.i,
                msg.pose.orientation.j,
                msg.pose.orientation.k,
                msg.pose.orientation.w,
            ],
            velocity: [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
        }
    }
}

/// Wire position control setpoint — 36 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WirePositionControlSetpoint {
    pub timestamp_us: u64,
    pub position: [f32; 3],
    pub velocity: [f32; 3],
    pub yaw: f32,
}

impl WireMessage for WirePositionControlSetpoint {
    const MSG_ID: u8 = msg_id::POSITION_CONTROL_SETPOINT;
}

#[cfg(feature = "embedded")]
impl WirePositionControlSetpoint {
    pub fn from_msg(msg: &crate::PositionControlSetpoint) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            position: [msg.position.x, msg.position.y, msg.position.z],
            velocity: [msg.velocity.x, msg.velocity.y, msg.velocity.z],
            yaw: msg.yaw,
        }
    }
}

/// Wire actuator motor commands — 24 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireActuatorMotors {
    pub timestamp_us: u64,
    pub motor_commands: [f32; 4], // [0.0, 1.0] per motor
}

impl WireMessage for WireActuatorMotors {
    const MSG_ID: u8 = msg_id::ACTUATOR_MOTORS;
}

#[cfg(feature = "embedded")]
impl WireActuatorMotors {
    pub fn from_msg(msg: &crate::ActuatorMotors) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            motor_commands: [
                msg.motor_commands[0].value(),
                msg.motor_commands[1].value(),
                msg.motor_commands[2].value(),
                msg.motor_commands[3].value(),
            ],
        }
    }
}

/// Wire GNSS/INS status (APX-15) — 102 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireGnssStatus {
    pub timestamp_us: u64, // 8

    // From #49 (InsFullNav) — status bytes
    pub imu_alignment: u8, // 1
    pub gps_quality: u8,   // 1

    // From #50 (InsRmsInfo) — position, velocity, attitude RMS
    pub north_pos_rms: f32, // 4
    pub east_pos_rms: f32,  // 4
    pub down_pos_rms: f32,  // 4
    pub north_vel_rms: f32, // 4
    pub east_vel_rms: f32,  // 4
    pub down_vel_rms: f32,  // 4
    pub roll_rms: f32,      // 4
    pub pitch_rms: f32,     // 4
    pub heading_rms: f32,   // 4

    // From #12 (PositionSigmaInfo)
    pub sigma_east: f32,  // 4
    pub sigma_north: f32, // 4
    pub sigma_up: f32,    // 4

    // From #1 (PositionTime)
    pub num_svs: u8,    // 1
    pub pos_flags1: u8, // 1
    pub pos_flags2: u8, // 1

    // From #38 (PositionTypeInfo)
    pub solution_flags: u8,    // 1
    pub rtk_condition: u8,     // 1
    pub correction_age: f32,   // 4
    pub network_flags: u8,     // 1
    pub position_fix_type: u8, // 1

    // From #9 (PdopInfo)
    pub pdop: f32, // 4
    pub hdop: f32, // 4
    pub vdop: f32, // 4
    pub tdop: f32, // 4

    // From #35 (ReceivedBaseInfo)
    pub base_id: u16, // 2

    // From #40 (LbandStatus)
    pub lband_snr: f32,       // 4
    pub lband_hp_xp_mode: u8, // 1
    pub lband_vbs_mode: u8,   // 1

    // From #10 (ClockInfo)
    pub clock_flags: u8,      // 1
    pub clock_offset_ms: f32, // 4
    pub freq_offset_ppm: f32, // 4
}

impl WireMessage for WireGnssStatus {
    const MSG_ID: u8 = msg_id::GNSS_STATUS;
}

#[cfg(feature = "embedded")]
impl WireGnssStatus {
    pub fn from_msg(msg: &crate::GnssStatus) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            imu_alignment: msg.imu_alignment,
            gps_quality: msg.gps_quality,
            north_pos_rms: msg.north_pos_rms,
            east_pos_rms: msg.east_pos_rms,
            down_pos_rms: msg.down_pos_rms,
            north_vel_rms: msg.north_vel_rms,
            east_vel_rms: msg.east_vel_rms,
            down_vel_rms: msg.down_vel_rms,
            roll_rms: msg.roll_rms,
            pitch_rms: msg.pitch_rms,
            heading_rms: msg.heading_rms,
            sigma_east: msg.sigma_east,
            sigma_north: msg.sigma_north,
            sigma_up: msg.sigma_up,
            num_svs: msg.num_svs,
            pos_flags1: msg.pos_flags1,
            pos_flags2: msg.pos_flags2,
            solution_flags: msg.solution_flags,
            rtk_condition: msg.rtk_condition,
            correction_age: msg.correction_age,
            network_flags: msg.network_flags,
            position_fix_type: msg.position_fix_type,
            pdop: msg.pdop,
            hdop: msg.hdop,
            vdop: msg.vdop,
            tdop: msg.tdop,
            base_id: msg.base_id,
            lband_snr: msg.lband_snr,
            lband_hp_xp_mode: msg.lband_hp_xp_mode,
            lband_vbs_mode: msg.lband_vbs_mode,
            clock_flags: msg.clock_flags,
            clock_offset_ms: msg.clock_offset_ms,
            freq_offset_ppm: msg.freq_offset_ppm,
        }
    }
}

/// Wire mission status — 25 bytes.
///
/// `state` encodes `MissionState` (0=Idle, 1=Planning, 2=Executing).
/// `tau_s` is elapsed trajectory time; for Idle/Planning both `tau_s` and
/// `total_duration_s` are 0. `target_position` is the position the outer
/// loop is currently tracking (trajectory sample at τ₀ while Executing,
/// hover setpoint otherwise).
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireMissionStatus {
    pub timestamp_us: u64,
    pub state: u8,
    pub tau_s: f32,
    pub total_duration_s: f32,
    pub target_position: [f32; 3],

    // ── Solve diagnostics (see `crate::SolveDiagnostics`) ──
    /// `SolverStatus` as a byte, or `crate::SOLVE_STATUS_NONE = 0xFF`
    /// when no solve has completed yet (Planning heartbeat / Idle).
    pub solve_status: u8,
    /// BFGS outer iterations consumed.
    pub solve_iterations: u16,
    /// Wall-clock solve duration in milliseconds.
    pub solve_time_ms: u16,
    /// Number of polynomial pieces in the published trajectory.
    pub solve_num_pieces: u8,
    /// Pre-BFGS initial time allocation (sum of seed segment times).
    pub solve_init_duration_s: f32,
    /// Final optimized cost value.
    pub solve_final_cost: f32,
    /// Peak ‖v‖ sampled across the published trajectory (m/s).
    pub solve_peak_vel_m_s: f32,
    /// Planner-side reject reason — see `crate::SOLVE_REJECT_*`. Zero on
    /// the happy path; non-zero identifies why a Planning → Idle
    /// transition fired without the mission reaching Executing.
    pub solve_reject_reason: u8,
}

impl WireMessage for WireMissionStatus {
    const MSG_ID: u8 = msg_id::MISSION_STATUS;
}

#[cfg(feature = "embedded")]
impl WireMissionStatus {
    pub fn from_msg(msg: &crate::MissionStatus) -> Self {
        Self {
            timestamp_us: msg.timestamp.as_micros(),
            state: msg.state,
            tau_s: msg.tau_s,
            total_duration_s: msg.total_duration_s,
            target_position: [
                msg.target_position.x,
                msg.target_position.y,
                msg.target_position.z,
            ],
            solve_status: msg.solve.status,
            solve_iterations: msg.solve.iterations,
            solve_time_ms: msg.solve.solve_time_ms,
            solve_num_pieces: msg.solve.num_pieces,
            solve_init_duration_s: msg.solve.init_duration_s,
            solve_final_cost: msg.solve.final_cost,
            solve_peak_vel_m_s: msg.solve.peak_vel_m_s,
            solve_reject_reason: msg.solve.reject_reason,
        }
    }
}

/// Wire ESKF state-estimator status — 80 bytes.
///
/// Reports filter health for the firmware-side ESKF: lifecycle phase,
/// fault bitflags, attitude (Euler), per-channel covariance traces, last
/// NIS values, cumulative gate-rejection counters, and time-since-last
/// position-update for staleness watchdogs. Cadence ~5 Hz from the
/// firmware (the ESP bridge subdivides its 100 Hz TX tick).
///
/// `phase` mirrors `EstimatorPhase` ordinal: 0 = AwaitingExteroceptive,
/// 1 = Converging, 2 = Running.
///
/// `fault_flags` mirrors the firmware-side `EskfFault` bitset; consumers
/// should treat unrecognised bits as opaque "something is wrong".
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireEstimatorStatus {
    pub timestamp_us: u64,
    pub fault_flags: u32,
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,
    /// Sum of position-covariance diagonal entries [m²].
    pub pos_cov_trace: f32,
    /// Sum of velocity-covariance diagonal entries [m²/s²].
    pub vel_cov_trace: f32,
    /// Sum of gyro-bias covariance diagonal entries [rad²/s²].
    pub gyro_bias_cov_trace: f32,
    pub last_nis_pos: f32,
    pub last_nis_vel: f32,
    pub last_nis_att: f32,
    pub nan_resets: u32,
    pub gate_rejects_pos: u32,
    pub gate_rejects_vel: u32,
    pub gate_rejects_att: u32,
    pub gate_rejects_baro: u32,
    pub gate_rejects_mag: u32,
    /// `f32::INFINITY` until the first update arrives.
    pub seconds_since_last_pos_update: f32,
    pub phase: u8,
    /// AHRS supervisor mode: 0 = Normal, 1 = AttitudeRateFallback, 2 = Disarmed.
    pub ahrs_mode: u8,
    pub _pad: [u8; 2],
}

impl WireMessage for WireEstimatorStatus {
    const MSG_ID: u8 = msg_id::ESTIMATOR_STATUS;
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

#[cfg(feature = "embedded")]
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

/// Maximum number of drones a single `WireFleetState` frame carries. Sized so
/// the whole message fits the 128-byte wire payload (12-byte header + 4×28).
pub const MAX_FLEET: usize = 4;

/// `WireFleetEntry.flags` bit: the estimate is fresh (within the GCS staleness
/// threshold). When clear, `position`/`velocity` are meaningless and the slot
/// should be ignored.
pub const FLEET_FLAG_VALID: u8 = 0b0000_0001;

/// One drone's shared state within a `WireFleetState`. The drone's identity is
/// its **index** in the `entries` array (stable fleet order) — there is no
/// per-entry id. 28 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireFleetEntry {
    /// Bitset; see `FLEET_FLAG_VALID`. A zeroed entry (`flags == 0`) is an
    /// absent/stale drone and must be ignored by the receiver.
    pub flags: u8,
    pub _pad: u8,
    /// Age of this estimate at `gs_send_time_us`, in milliseconds, saturating
    /// at `u16::MAX` (~65 s). Absolute capture time ≈ `gs_send_time_us − age_ms×1000`.
    pub age_ms: u16,
    /// Position in metres, in the drone's world frame (delivered verbatim from
    /// the source `WireVehicleOdometry`; cross-drone frame consistency is the
    /// estimator's responsibility).
    pub position: [f32; 3],
    /// Velocity in metres/second.
    pub velocity: [f32; 3],
}

/// Fleet state broadcast from the ground station to each drone at 10 Hz —
/// 124 bytes.
///
/// Carries the timestamped position+velocity of every drone in the swarm so
/// each one knows where its neighbours are. The frame is self-contained: a
/// lost datagram just means one stale 100 ms cycle. `entries[i]` is drone `i`
/// in fleet order; `self_idx` tells the recipient which entry is itself.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireFleetState {
    /// UTC microseconds at GCS send. Lets the receiver compute end-to-end
    /// latency (`local_utc_now − gs_send_time_us`) and each entry's absolute
    /// capture time (`gs_send_time_us − age_ms×1000`).
    pub gs_send_time_us: i64,
    /// The recipient's own index in `entries`, so it can skip itself.
    /// `0xFF` if the recipient isn't among the first `MAX_FLEET` drones.
    pub self_idx: u8,
    /// Number of populated entries = `min(fleet_len, MAX_FLEET)`.
    pub count: u8,
    pub _pad: [u8; 2],
    pub entries: [WireFleetEntry; MAX_FLEET],
}

impl WireMessage for WireFleetState {
    const MSG_ID: u8 = msg_id::FLEET_STATE;
}

// The frame must fit the wire payload budget — enforced at compile time.
const _: () = assert!(core::mem::size_of::<WireFleetState>() <= MAX_WIRE_PAYLOAD);
const _: () = assert!(core::mem::size_of::<WireFleetEntry>() == 28);

/// Time sync diagnostics downlink — 25 bytes.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireTimeSyncStatus {
    pub synced: u8,
    pub offset_us: i64,
    pub ping_rtt_us: u64,
    pub ping_clock_err_us: i64,
}

impl WireMessage for WireTimeSyncStatus {
    const MSG_ID: u8 = msg_id::TIME_SYNC_STATUS;
}

// ── Health snapshots ──────────────────────────────────────────────────────
//
// Both types mirror the live shell (`health` / `gpshealth`) and the
// matching blackbox topics. The firmware's `comm::esp_bridge` snapshots
// them at 1 Hz off the same atomics the shell reads, so a wire frame and
// a contemporaneous BB record agree by construction. The GCS dashboard
// renders these in a view-only TUI pane.
//
// Stable on the wire — never renumber. Field additions go at the end of
// the struct (and are zero-filled on older firmware via the `from_bytes`
// length check).

/// Aggregate ESKF / failsafe / sensor-health snapshot — 76 bytes.
///
/// Booleans are `u8` to keep the layout fully POD. `last_*_update_age_ms`
/// is pre-subtracted from "now" on the firmware side; `u32::MAX` encodes
/// "never accepted". `_pad` keeps the trailing struct on a 4-byte
/// boundary; readers must ignore.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireSystemHealth {
    pub timestamp_us: u64,
    pub estimator_faults: u32,
    pub nan_resets: u32,
    pub gate_rejects_pos: u32,
    pub gate_rejects_vel: u32,
    pub gate_rejects_att: u32,
    /// Age in ms of the last accepted pos/vel/att update. `u32::MAX` =
    /// never. Disambiguates POS_STALE-from-no-data vs
    /// POS_STALE-from-rejection: a frozen value means upstream stopped
    /// delivering; a recent one means the guard is rejecting fresh
    /// measurements.
    pub last_pos_update_age_ms: u32,
    pub last_vel_update_age_ms: u32,
    pub last_att_update_age_ms: u32,
    pub last_nis_pos: f32,
    pub last_nis_vel: f32,
    pub last_nis_att: f32,
    pub failsafe_active: u8,
    pub estimator_ready: u8,
    pub eskf_degraded: u8,
    pub eskf_severe_fault: u8,
    pub attitude_health: u8,
    pub _pad: [u8; 3],
}

impl WireMessage for WireSystemHealth {
    const MSG_ID: u8 = msg_id::SYSTEM_HEALTH;
}

// ── GpsHealth state / err_kind encoding ───────────────────────────────────
//
// Stable on disk; never renumber.

pub const GPS_STATE_NOT_CONFIGURED: u8 = 0;
pub const GPS_STATE_INITIALIZING: u8 = 1;
pub const GPS_STATE_UART_INIT_FAILED: u8 = 2;
pub const GPS_STATE_INIT_TIMED_OUT: u8 = 3;
pub const GPS_STATE_INIT_FAILED: u8 = 4;
pub const GPS_STATE_WAITING: u8 = 5;
pub const GPS_STATE_LOCKED: u8 = 6;
pub const GPS_STATE_READ_ERROR: u8 = 7;

pub const GPS_ERR_NONE: u8 = 0;
pub const GPS_ERR_IO: u8 = 1;
pub const GPS_ERR_BAD_CHECKSUM: u8 = 2;
pub const GPS_ERR_NAK: u8 = 3;
pub const GPS_ERR_TIMEOUT: u8 = 4;

/// Flat snapshot of `GpsHealth` + the latest NAV-PVT accuracy fields —
/// 44 bytes.
///
/// Variant fields the active state doesn't define are zero. See
/// [`GPS_STATE_*`](GPS_STATE_NOT_CONFIGURED) and
/// [`GPS_ERR_*`](GPS_ERR_NONE) for the encoding tables.
///
/// ## Inter-PVT cadence on this wire
///
/// Earlier revisions carried `last_fix_age_ms` and `nav_pvt_age_ms`
/// — snapshot ages computed as `<emit time> - <last arrival>`. They
/// were dropped because plotting them in a GCS dashboard at the
/// wire's 1 Hz cadence aliases against the GPS ~2-5 Hz NAV-PVT
/// period and produces a sawtooth that misreads as receiver jitter.
/// The replacement fields:
///
/// - `nav_pvt_arrival_us`: raw `Instant`-microseconds of the most
///   recent NAV-PVT. The GCS computes liveness/staleness as
///   `timestamp_us - nav_pvt_arrival_us` from a single record —
///   that's an honest "as-of-now" age that doesn't depend on the
///   wire sampling cadence.
/// - `nav_pvt_max_interval_recent_us`: firmware-side health metric
///   for dropouts. Largest inter-arrival gap among the most recent
///   16 NAV-PVTs. Threshold input for "did the receiver miss a
///   frame?" alarms.
/// - `nav_pvt_mean_interval_recent_us`: ROS `topic hz`-style mean
///   period over the same window. Threshold input for "is the
///   typical rate what we expect?" alarms; a TUI converts to Hz
///   via `Hz = 1_000_000 / value`. Note: the wire alone is too
///   slow (1 Hz) to compute meaningful inter-PVT stats TUI-side
///   when GPS runs at 2–5 Hz, so both aggregates are computed at
///   the source rather than left for the dashboard to derive.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireGpsHealth {
    pub timestamp_us: u64,
    /// `Instant`-microseconds when the firmware GPS task parsed the
    /// most recent NAV-PVT. `0` when no NAV-PVT has ever been
    /// received. Use `timestamp_us - nav_pvt_arrival_us` for the
    /// per-record staleness.
    pub nav_pvt_arrival_us: u64,
    pub h_acc_mm: u32,
    pub v_acc_mm: u32,
    pub s_acc_mm_s: u32,
    /// Worst-case inter-arrival interval among the most recent 16
    /// NAV-PVTs, in microseconds. `u32::MAX` until at least two
    /// PVTs have been parsed since boot. Compare against the
    /// configured measurement period to detect a jittering pipeline.
    pub nav_pvt_max_interval_recent_us: u32,
    /// Mean inter-arrival interval over the same 16-PVT window, in
    /// microseconds. `u32::MAX` until at least two PVTs have been
    /// parsed. Display as Hz via `1_000_000 / value`.
    pub nav_pvt_mean_interval_recent_us: u32,
    pub state: u8,
    pub err_kind: u8,
    pub fix_type: u8,
    pub num_sv: u8,
    pub diff_soln: u8,
    pub carr_soln: u8,
    pub gnss_fix_ok: u8,
    pub _pad: u8,
}

impl WireMessage for WireGpsHealth {
    const MSG_ID: u8 = msg_id::GPS_HEALTH;
}
