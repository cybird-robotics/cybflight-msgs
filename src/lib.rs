#![no_std]

pub mod wire;

#[cfg(feature = "embedded")]
pub mod dshot;

#[cfg(feature = "embedded")]
use embassy_time::Instant;
#[cfg(feature = "embedded")]
use nalgebra::{SVector, UnitQuaternion, Vector3};

#[cfg(feature = "embedded")]
pub trait Message: defmt::Format + Clone + Sync + 'static {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct Imu {
    pub timestamp: Instant,
    pub accel_m_s2: Vector3<f32>,
    pub gyro_rad_s: Vector3<f32>,
    pub temp_c: f32,
}

#[cfg(feature = "embedded")]
impl Message for Imu {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct VehicleAttitude {
    pub timestamp: Instant,
    pub orientation: UnitQuaternion<f32>,
}

#[cfg(feature = "embedded")]
impl Message for VehicleAttitude {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct Pose {
    pub position: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
}

#[cfg(feature = "embedded")]
impl Message for Pose {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct Twist {
    pub linear: Vector3<f32>,
    pub angular: Vector3<f32>,
}

#[cfg(feature = "embedded")]
impl Message for Twist {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct VehicleOdometry {
    pub timestamp: Instant,
    pub pose: Pose,
    pub twist: Twist,
}

#[cfg(feature = "embedded")]
impl Message for VehicleOdometry {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct ViconPose {
    pub timestamp: Instant,
    pub position: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
    /// VICON camera capture time (UTC microseconds since UNIX epoch).
    pub capture_time_utc_us: i64,
    /// GS PC send time (UTC microseconds since UNIX epoch).
    pub gs_send_time_utc_us: i64,
}

#[cfg(feature = "embedded")]
impl Message for ViconPose {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct RcInput {
    pub timestamp: Instant,
    /// Channel values in PWM microseconds [988..2012].
    pub channels: [u16; 16],
    /// Number of valid channels.
    pub channel_count: u8,
}

#[cfg(feature = "embedded")]
impl Message for RcInput {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct RcLinkStatus {
    pub timestamp: Instant,
    /// RSSI in dBm (negative value).
    pub rssi_dbm: i16,
    /// Link quality percentage [0..100].
    pub link_quality: u8,
    /// Signal-to-noise ratio in dB.
    pub snr: i8,
    /// RF mode index (protocol-specific).
    pub rf_mode: u8,
}

#[cfg(feature = "embedded")]
impl Message for RcLinkStatus {}

/// NMPC position setpoint.  Published by the RC-input-to-setpoint converter
/// (not yet implemented); the NMPC driver falls back to a hardcoded hover
/// target until a publisher exists.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct NmpcSetpoint {
    pub timestamp: Instant,
    /// Target position in world frame [m].
    pub position: Vector3<f32>,
}

#[cfg(feature = "embedded")]
impl Message for NmpcSetpoint {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct DshotMotorTelemetry {
    pub value: dshot::TelemetryValue,
    pub raw: Option<u16>,
}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct DshotTelemetry {
    pub timestamp: Instant,
    pub motors: [DshotMotorTelemetry; 4],
}
#[cfg(feature = "embedded")]
impl Message for DshotTelemetry {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct GpsFix {
    pub timestamp: Instant,
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

#[cfg(feature = "embedded")]
impl Message for GpsFix {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct MagSample {
    pub timestamp: Instant,
    pub field_ut: Vector3<f32>,
    pub temp_c: f32,
}

#[cfg(feature = "embedded")]
impl Message for MagSample {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct BaroSample {
    pub timestamp: Instant,
    pub pressure_pa: f32,
    pub temp_c: f32,
}

#[cfg(feature = "embedded")]
impl Message for BaroSample {}

#[cfg(feature = "embedded")]
const OCP_OUTPUT_SIZE: usize = 4;
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct OcpSolverOutput {
    pub timestamp: Instant,
    pub command: SVector<f32, OCP_OUTPUT_SIZE>,
    pub iterations: i32,
    pub converged: bool,
    pub solve_time_us: u64,
}
#[cfg(feature = "embedded")]
impl Message for OcpSolverOutput {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct AttitudeControlSetpoint {
    pub timestamp: Instant,
    pub collective_thrust_n: f32,
    pub attitude_quaternion: UnitQuaternion<f32>,
    pub body_rate_rad_s: Vector3<f32>,
    pub torque_n_m: Vector3<f32>,
}

#[cfg(feature = "embedded")]
impl Message for AttitudeControlSetpoint {}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct PositionControlSetpoint {
    pub timestamp: Instant,
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub yaw: f32,
}

#[cfg(feature = "embedded")]
impl Message for PositionControlSetpoint {}

/// NMPC control output consumed by the inner loop.
///
/// Carries both collective thrust + body rate (for rate PID path) and
/// per-motor thrusts (for direct motor control path). The `direct_motor_control`
/// flag tells the inner loop which path to use.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct NmpcControlOutput {
    pub timestamp: Instant,
    /// Collective thrust along body z [N].
    pub collective_thrust_n: f32,
    /// Desired body rate [rad/s] from NMPC predicted next state.
    pub body_rate_rad_s: Vector3<f32>,
    /// Desired attitude quaternion (for geometric controller).
    pub attitude_quaternion: UnitQuaternion<f32>,
    /// Per-motor thrusts [N] (used when direct_motor_control is true).
    pub motor_thrusts: [f32; 4],
    /// Whether the NMPC solver converged.
    pub converged: bool,
    /// true = use motor_thrusts directly (bypass rate PID + mixer),
    /// false = use collective_thrust + body_rate through rate PID + mixer.
    pub direct_motor_control: bool,
}

#[cfg(feature = "embedded")]
impl Message for NmpcControlOutput {}

#[cfg(feature = "embedded")]
#[derive(Debug, Copy, Clone, defmt::Format)]
pub struct NormalizedThrottle(f32);

#[cfg(feature = "embedded")]
impl NormalizedThrottle {
    pub fn new(value: f32) -> Option<Self> {
        if (0.0..=1.0).contains(&value) {
            Some(Self(value))
        } else {
            None
        }
    }

    pub fn new_saturating(value: f32) -> Self {
        Self(value.clamp(0.0, 1.0))
    }

    pub fn value(&self) -> f32 {
        self.0
    }
}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct ActuatorMotors {
    pub timestamp: Instant,
    pub motor_commands: [NormalizedThrottle; 4],
}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct ArmDisarm {
    pub timestamp: Instant,
    pub armed: bool,
}

/// Manual control setpoint: thrust + body rates from RC sticks.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct ManualControlSetpoint {
    pub timestamp: Instant,
    /// Normalized thrust [0, 1].
    pub thrust: f32,
    /// Roll rate command (rad/s).
    pub roll_rate: f32,
    /// Pitch rate command (rad/s).
    pub pitch_rate: f32,
    /// Yaw rate command (rad/s).
    pub yaw_rate: f32,
}

#[cfg(feature = "embedded")]
impl Message for ManualControlSetpoint {}

/// Stablized control setpoint: thrust + angles from RC sticks.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct StablizedControlSetpoint {
    pub timestamp: Instant,
    /// Normalized thrust [0, 1].
    pub thrust: f32,
    /// Roll command (rad).
    pub roll: f32,
    /// Pitch command (rad).
    pub pitch: f32,
    /// Yaw command (rad).
    pub yaw: f32,
}

#[cfg(feature = "embedded")]
impl Message for StablizedControlSetpoint {}

// ── Navigation ───────────────────────────────────────────────────────────────

/// Maximum number of target positions in a waypoint sequence.
///
/// Must be `<= cybflight_core::trajectory_planning::MAX_PIECES` — the planner
/// uses the last target as the tail and all preceding targets as intermediate
/// waypoints, producing `num_targets` polynomial pieces.
pub const MAX_NAV_WAYPOINTS: usize = 16;

/// Navigation command type: single position or waypoint sequence.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub enum NavigationCommandType {
    /// Go to a single position. Planner decides if trajectory is needed.
    GoToPosition {
        position: Vector3<f32>,
        yaw: f32,
    },
    /// Follow a waypoint sequence through all points.
    WaypointSequence {
        waypoints: [Vector3<f32>; MAX_NAV_WAYPOINTS],
        num_waypoints: u8,
        yaw: f32,
    },
}

/// Navigation command from ground station or RC interpreter.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct NavigationCommand {
    pub timestamp: Instant,
    pub command: NavigationCommandType,
}

#[cfg(feature = "embedded")]
impl Message for NavigationCommand {}

