//! Wire-format packed structs for UART transport (ESP bridge).
//!
//! Separate from channel types (which use nalgebra). All downlink types carry
//! the original `Instant` timestamp as `timestamp_us: u64` (microseconds since boot).

/// Wire-format message identifiers.
pub mod msg_id {
    /// Reserved — never assign. A downlink UDP datagram whose first byte is 0
    /// is an ESP-bridge batch envelope, not a frame (see `crate::udp_batch`).
    pub const RESERVED_BATCH_MARKER: u8 = 0;
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
    /// Fleet-wide session ENU origin (the RTK base station's antenna
    /// reference point, WGS84 ellipsoidal height), pushed by the ground
    /// station to every drone at 1 Hz. Never persisted on the drone — every
    /// boot waits for it again. See [`WireGnssOriginSet`].
    pub const GNSS_ORIGIN_SET: u8 = 133;
    /// Radio link status reported by the datalink bridge itself (not
    /// forwarded from the ground station): kind, state, RSSI, reconnect
    /// count. The ESP32 bridge sends it at 5 Hz. See [`WireDatalinkStatus`].
    pub const DATALINK_STATUS: u8 = 134;

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
    /// Which airframe this firmware believes it is: the baked
    /// `airframe.name`, the vehicle YAML stem it was built from, and the
    /// compiled-in position source. Static for the life of a boot, so it
    /// is downlinked slowly (~0.2 Hz) purely as a self-announcement.
    ///
    /// Exists so the ground station can *verify* a binding it otherwise
    /// has to assume: poses are routed to a drone by IP, and nothing
    /// checks that the board at that IP runs the firmware for the
    /// airframe whose mocap subject is being sent there. A mis-flashed
    /// board silently flies another machine's mass, inertia and gains
    /// while tracking another body's poses. See [`WireVehicleIdentity`].
    pub const VEHICLE_IDENTITY: u8 = 28;
    /// Per-drone echo of the ENU origin it is (or is not yet) flying in,
    /// plus its latched HOME. Downlinked at 1 Hz. See [`WireGnssOrigin`].
    pub const GNSS_ORIGIN: u8 = 29;
    /// Drone ↔ ground-station datalink status as the flight controller
    /// sees it (radio kind/state/RSSI from the last bridge report, plus
    /// FC-side uplink age and ping RTT). Downlinked at 1 Hz. A separate
    /// message rather than more fields on `SystemHealth`, whose readers
    /// reject a short payload. See [`WireDatalinkHealth`].
    pub const DATALINK_HEALTH: u8 = 30;

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
    /// Height in the sender's origin datum (WGS84 ellipsoidal), millimetres.
    pub alt_mm: i32,
    /// Bit0 `PEER_FLAG_CONVERGED` (ESKF converged); bit1 `PEER_FLAG_3D_FIX` (≥3D GNSS fix).
    pub status: u8,
}

impl WireMessage for WirePeerPose {
    const MSG_ID: u8 = msg_id::PEER_POSE;
}

const _: () = assert!(core::mem::size_of::<WirePeerPose>() <= MAX_WIRE_PAYLOAD);

impl WirePeerPose {
    /// Build from geodetic radians + height (m, origin datum) + liveness flags.
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

/// `WireGnssOriginSet::flags` bit 0: the origin fields are meaningful.
pub const ORIGIN_SET_FLAG_VALID: u8 = 0x01;

/// Ground station → drone: the session's shared ENU origin — 21 bytes.
///
/// The origin is the RTK base station's antenna reference point, decoded
/// by the ground station from RTCM 1005/1006 (ECEF → WGS84) or given on
/// its command line. The base is portable, so this is a per-session value:
/// the drone latches it once per boot and never stores it.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireGnssOriginSet {
    /// Latitude [deg].
    pub lat_deg: f64,
    /// Longitude [deg].
    pub lon_deg: f64,
    /// WGS84 ellipsoidal height [m]. NOT mean sea level.
    pub h_ell_m: f32,
    /// Bit0 [`ORIGIN_SET_FLAG_VALID`].
    pub flags: u8,
}

impl WireMessage for WireGnssOriginSet {
    const MSG_ID: u8 = msg_id::GNSS_ORIGIN_SET;
}

const _: () = assert!(core::mem::size_of::<WireGnssOriginSet>() <= MAX_WIRE_PAYLOAD);

/// `kind` of a datalink status: which radio carries the drone ↔ ground link.
pub mod datalink_kind {
    /// No datalink status known.
    pub const NONE: u8 = 0;
    /// WiFi STA through the ESP32 bridge (`cybesp-bridge`).
    pub const WIFI: u8 = 1;
    /// LoRa (EBYTE E22 via cybcomm) — reserved for the future driver.
    pub const LORA: u8 = 2;
}

/// `state` of a WiFi datalink — mirrors `cybesp-bridge`'s `LinkStatus`.
pub mod datalink_state {
    pub const BOOTING: u8 = 0;
    pub const CONNECTING: u8 = 1;
    pub const CONNECTED: u8 = 2;
    pub const DISCONNECTED: u8 = 3;
    /// No status report received (downlink only).
    pub const UNKNOWN: u8 = 0xFF;
}

/// `rssi_dbm` sentinel: not associated / not measured.
pub const RSSI_UNKNOWN: i8 = i8::MIN;

/// Datalink status reported by the bridge — 9 bytes, ESP32 → STM32.
///
/// The one frame (besides TIME_SYNC) the bridge originates rather than
/// forwards, so it proves the bridge is alive even when nothing arrives
/// from the ground. Append-only: fields go at the end (readers accept
/// longer payloads).
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireDatalinkStatus {
    /// [`datalink_kind`].
    pub kind: u8,
    /// [`datalink_state`] for WiFi.
    pub state: u8,
    /// RSSI of the associated access point from its last beacon [dBm];
    /// [`RSSI_UNKNOWN`] when not associated.
    pub rssi_dbm: i8,
    /// WiFi disconnects since the bridge booted (saturating).
    pub reconnects: u16,
    /// Bridge uptime [ms], wrapping. A drop means the bridge rebooted.
    pub bridge_uptime_ms: u32,
}

impl WireMessage for WireDatalinkStatus {
    const MSG_ID: u8 = msg_id::DATALINK_STATUS;
}

const _: () = assert!(core::mem::size_of::<WireDatalinkStatus>() <= MAX_WIRE_PAYLOAD);

impl WireDatalinkStatus {
    /// RSSI in dBm, `None` for [`RSSI_UNKNOWN`].
    pub fn rssi(&self) -> Option<i8> {
        let r = self.rssi_dbm;
        (r != RSSI_UNKNOWN).then_some(r)
    }
}

impl WireGnssOriginSet {
    /// A VALID-flagged origin from degrees + ellipsoidal height.
    pub fn new(lat_deg: f64, lon_deg: f64, h_ell_m: f32) -> Self {
        Self { lat_deg, lon_deg, h_ell_m, flags: ORIGIN_SET_FLAG_VALID }
    }

    /// `(lat_deg, lon_deg, h_ell_m)` when the VALID flag is set and every
    /// field is finite and in range; `None` otherwise. The single
    /// validation point for both the firmware decoder and host tests.
    pub fn validated(&self) -> Option<(f64, f64, f32)> {
        // Copy packed fields before use (unaligned reads are UB by reference).
        let (lat, lon, h, flags) = (self.lat_deg, self.lon_deg, self.h_ell_m, self.flags);
        if flags & ORIGIN_SET_FLAG_VALID == 0 {
            return None;
        }
        if !(lat.is_finite() && lon.is_finite() && h.is_finite()) {
            return None;
        }
        if !((-90.0..=90.0).contains(&lat) && (-180.0..=180.0).contains(&lon)) {
            return None;
        }
        // Ellipsoidal heights on Earth's surface span roughly −110..+9000 m;
        // anything far outside is a unit or datum mistake, not a site.
        if !(-1000.0..=10_000.0).contains(&h) {
            return None;
        }
        Some((lat, lon, h))
    }
}

/// `WireGnssOrigin::state` values.
pub mod origin_state {
    /// No origin latched yet (no ground-station origin, or no RTK fix).
    pub const WAITING: u8 = 0;
    /// Origin latched; the ENU frame is fixed for this boot.
    pub const LATCHED: u8 = 1;
    /// The seed RTK fix lay farther from the offered origin than
    /// `gps_origin_max_dist_m`; still waiting.
    pub const REJECTED_TOO_FAR: u8 = 2;
    /// A later ground-station origin disagrees with the latched one by
    /// more than `gps_origin_mismatch_m`. Sticky until reboot.
    pub const MISMATCH: u8 = 3;
}

/// `WireGnssOrigin::source` values.
pub mod origin_source {
    pub const NONE: u8 = 0;
    /// Ground-station uplink ([`WireGnssOriginSet`]).
    pub const UPLINK: u8 = 1;
    /// Vehicle YAML `origin:` (offline / fixed-site override).
    pub const BAKED_YAML: u8 = 2;
    /// Legacy: the drone's own first RTK-fixed PVT.
    pub const FIRST_FIX: u8 = 3;
}

/// Drone → ground station: the origin this drone latched, its state, and
/// the HOME it latched at arming — 43 bytes. Downlinked at 1 Hz.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireGnssOrigin {
    pub timestamp_us: u64,
    /// Latched origin latitude [deg]; zero until latched.
    pub lat_deg: f64,
    /// Latched origin longitude [deg]; zero until latched.
    pub lon_deg: f64,
    /// Latched origin WGS84 ellipsoidal height [m]; zero until latched.
    pub h_ell_m: f32,
    /// See [`origin_state`].
    pub state: u8,
    /// See [`origin_source`].
    pub source: u8,
    /// HOME (latched at the arm edge) in this drone's ENU frame [m].
    pub home_enu: [f32; 3],
    /// 1 when `home_enu` is meaningful (armed with a latched home).
    pub home_valid: u8,
}

impl WireMessage for WireGnssOrigin {
    const MSG_ID: u8 = msg_id::GNSS_ORIGIN;
}

const _: () = assert!(core::mem::size_of::<WireGnssOrigin>() <= MAX_WIRE_PAYLOAD);

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

/// Wire OCP solver output — 41 bytes. `policy_time_us` was APPENDED
/// (2026-09-08): an old GCS still parses the 37-byte prefix of new
/// frames, but a new GCS drops old 37-byte frames (`from_bytes` length
/// check) — pair the GCS update with the firmware that sends it.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireOcpSolverOutput {
    pub timestamp_us: u64,
    pub cmd: [f32; 4],
    pub iterations: i32,
    pub converged: u8,
    pub solve_time_us: u64,
    pub policy_time_us: u32,
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
            policy_time_us: msg.policy_time_us,
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

/// Datalink health as the flight controller sees it — 25 bytes, 1 Hz.
///
/// Same conventions as [`WireSystemHealth`]: ages pre-subtracted on the
/// firmware side, `u32::MAX` = never; [`RSSI_UNKNOWN`] = no RSSI;
/// `state` = [`datalink_state::UNKNOWN`] when no bridge report arrived.
/// Link-agnostic: a LoRa build fills it with `kind = LORA`.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireDatalinkHealth {
    pub timestamp_us: u64,
    /// [`datalink_kind`] of the last status report (`NONE` if none).
    pub kind: u8,
    pub state: u8,
    pub rssi_dbm: i8,
    pub reconnects: u16,
    /// Age of the last radio status report [ms] — is the bridge alive?
    pub status_age_ms: u32,
    /// Age of the last ground-station-originated frame [ms] — is the
    /// ground still getting through?
    pub uplink_age_ms: u32,
    /// Last ground-station ping round trip [ms]; `u32::MAX` = none yet.
    pub ping_rtt_ms: u32,
}

impl WireMessage for WireDatalinkHealth {
    const MSG_ID: u8 = msg_id::DATALINK_HEALTH;
}

const _: () = assert!(core::mem::size_of::<WireDatalinkHealth>() <= MAX_WIRE_PAYLOAD);

/// Maximum length of an airframe name / vehicle stem on the wire.
/// Names are ASCII `[A-Za-z0-9_-]` (enforced by the vehicle-YAML loader),
/// NUL-padded here.
pub const IDENT_LEN: usize = 24;

/// `pos_source` encoding in [`WireVehicleIdentity`].
pub mod pos_source {
    /// Firmware did not declare one (no `est_pos_*` feature).
    pub const UNKNOWN: u8 = 0;
    /// Motion capture — this drone NEEDS an external pose stream to fly.
    pub const MOCAP: u8 = 1;
    /// GNSS — this drone must NOT be sent mocap poses.
    pub const GPS: u8 = 2;
}

/// Self-announced firmware identity — 64 bytes.
///
/// Every field is fixed at build time, so this never changes during a
/// boot; it is downlinked slowly and exists only to let the ground
/// station check its roster against reality.
///
/// `pos_source` is included because the routing decision depends on it,
/// not just on the name: flashing the *outdoor* (GPS) build of the right
/// airframe and flying it indoors gives a firmware that never subscribes
/// to the pose channel at all, which the name alone would not reveal.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct WireVehicleIdentity {
    pub timestamp_us: u64,
    /// `airframe.name` from the vehicle YAML — the physical machine.
    /// All-NUL means the YAML declared none.
    pub airframe_name: [u8; IDENT_LEN],
    /// Vehicle YAML stem this firmware was baked from. Distinguishes the
    /// indoor and outdoor builds of one airframe.
    pub vehicle: [u8; IDENT_LEN],
    /// See [`pos_source`].
    pub pos_source: u8,
    pub _pad: [u8; 7],
}

impl WireMessage for WireVehicleIdentity {
    const MSG_ID: u8 = msg_id::VEHICLE_IDENTITY;
}

impl WireVehicleIdentity {
    /// Copy `s` into a NUL-padded fixed field, truncating if it does not
    /// fit. Truncation is silent here because the vehicle-YAML loader is
    /// the place that can produce a good error message about it.
    pub fn pack_ident(s: &str) -> [u8; IDENT_LEN] {
        let mut out = [0u8; IDENT_LEN];
        let b = s.as_bytes();
        let n = if b.len() > IDENT_LEN { IDENT_LEN } else { b.len() };
        out[..n].copy_from_slice(&b[..n]);
        out
    }

    /// Decode a NUL-padded field back to `&str`, dropping anything past
    /// the first NUL. Returns `None` if the bytes are not valid UTF-8.
    pub fn unpack_ident(field: &[u8; IDENT_LEN]) -> Option<&str> {
        let end = field.iter().position(|b| *b == 0).unwrap_or(IDENT_LEN);
        core::str::from_utf8(&field[..end]).ok()
    }

    /// The airframe this firmware was built for, or `None` if the vehicle
    /// YAML declared no `airframe.name`.
    pub fn airframe(&self) -> Option<&str> {
        Self::unpack_ident(&self.airframe_name).filter(|s| !s.is_empty())
    }

    /// The vehicle YAML stem, or `None` if unset/undecodable.
    pub fn vehicle_stem(&self) -> Option<&str> {
        Self::unpack_ident(&self.vehicle).filter(|s| !s.is_empty())
    }
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

#[cfg(test)]
mod identity_tests {
    use super::*;

    #[test]
    fn identity_is_64_bytes_and_fits_a_frame() {
        assert_eq!(core::mem::size_of::<WireVehicleIdentity>(), 64);
        assert!(core::mem::size_of::<WireVehicleIdentity>() <= MAX_WIRE_PAYLOAD);
    }

    #[test]
    fn ident_round_trips_and_truncates() {
        let f = WireVehicleIdentity::pack_ident("hunter_01");
        assert_eq!(WireVehicleIdentity::unpack_ident(&f), Some("hunter_01"));

        // Exactly-full field has no NUL terminator; unpack must still stop
        // at the field boundary rather than run off the end.
        let full = "a".repeat(IDENT_LEN);
        let f = WireVehicleIdentity::pack_ident(&full);
        assert_eq!(WireVehicleIdentity::unpack_ident(&f), Some(full.as_str()));

        let f = WireVehicleIdentity::pack_ident(&"b".repeat(IDENT_LEN + 9));
        assert_eq!(
            WireVehicleIdentity::unpack_ident(&f),
            Some("b".repeat(IDENT_LEN).as_str())
        );
    }

    #[test]
    fn absent_airframe_reads_as_none() {
        let id = WireVehicleIdentity {
            timestamp_us: 0,
            airframe_name: [0u8; IDENT_LEN],
            vehicle: WireVehicleIdentity::pack_ident("sim_baseline"),
            pos_source: pos_source::UNKNOWN,
            _pad: [0u8; 7],
        };
        assert_eq!(id.airframe(), None);
        assert_eq!(id.vehicle_stem(), Some("sim_baseline"));
    }
}

#[cfg(test)]
mod origin_wire_tests {
    use super::*;

    #[test]
    fn origin_message_sizes() {
        assert_eq!(core::mem::size_of::<WireGnssOriginSet>(), 21);
        assert_eq!(core::mem::size_of::<WireGnssOrigin>(), 43);
    }

    #[test]
    fn origin_set_round_trips_through_bytes() {
        let w = WireGnssOriginSet::new(47.397_741_9, 8.545_593_8, 540.25);
        let back = WireGnssOriginSet::from_bytes(w.as_bytes()).unwrap();
        let (lat, lon, h) = back.validated().unwrap();
        assert_eq!(lat, 47.397_741_9);
        assert_eq!(lon, 8.545_593_8);
        assert_eq!(h, 540.25);
    }

    #[test]
    fn origin_set_validation_rejects_bad_input() {
        let mut w = WireGnssOriginSet::new(47.0, 8.0, 500.0);
        w.flags = 0;
        assert!(w.validated().is_none(), "VALID flag clear");
        assert!(WireGnssOriginSet::new(f64::NAN, 8.0, 500.0).validated().is_none());
        assert!(WireGnssOriginSet::new(91.0, 8.0, 500.0).validated().is_none());
        assert!(WireGnssOriginSet::new(47.0, 181.0, 500.0).validated().is_none());
        assert!(WireGnssOriginSet::new(47.0, 8.0, f32::INFINITY).validated().is_none());
        assert!(WireGnssOriginSet::new(47.0, 8.0, 50_000.0).validated().is_none());
        assert!(WireGnssOriginSet::new(-33.9, 151.2, -20.0).validated().is_some());
    }
}

#[cfg(test)]
mod datalink_wire_tests {
    use super::*;

    #[test]
    fn datalink_message_sizes() {
        assert_eq!(core::mem::size_of::<WireDatalinkStatus>(), 9);
        assert_eq!(core::mem::size_of::<WireDatalinkHealth>(), 25);
    }

    #[test]
    fn datalink_status_round_trips_and_decodes_rssi() {
        let w = WireDatalinkStatus {
            kind: datalink_kind::WIFI,
            state: datalink_state::CONNECTED,
            rssi_dbm: -62,
            reconnects: 3,
            bridge_uptime_ms: 123_456,
        };
        let back = WireDatalinkStatus::from_bytes(w.as_bytes()).unwrap();
        let (k, s, n, up) = (back.kind, back.state, back.reconnects, back.bridge_uptime_ms);
        assert_eq!((k, s, n, up), (datalink_kind::WIFI, datalink_state::CONNECTED, 3, 123_456));
        assert_eq!(back.rssi(), Some(-62));
        let off = WireDatalinkStatus { rssi_dbm: RSSI_UNKNOWN, ..w };
        assert_eq!(off.rssi(), None);
        // Appended fields must not break today's reader.
        let mut longer = w.as_bytes().to_vec();
        longer.extend_from_slice(&[0xAA; 4]);
        assert!(WireDatalinkStatus::from_bytes(&longer).is_some());
    }

    #[test]
    fn datalink_health_round_trips() {
        let w = WireDatalinkHealth {
            timestamp_us: 7,
            kind: datalink_kind::WIFI,
            state: datalink_state::DISCONNECTED,
            rssi_dbm: RSSI_UNKNOWN,
            reconnects: 2,
            status_age_ms: 180,
            uplink_age_ms: u32::MAX,
            ping_rtt_ms: 12,
        };
        let back = WireDatalinkHealth::from_bytes(w.as_bytes()).unwrap();
        let (st, up, rtt) = (back.state, back.uplink_age_ms, back.ping_rtt_ms);
        assert_eq!((st, up, rtt), (datalink_state::DISCONNECTED, u32::MAX, 12));
    }
}
