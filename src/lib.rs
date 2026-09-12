#![no_std]

pub mod wire;

/// Downlink datagram envelope (bridge → ground station). See [`udp_batch`].
pub mod udp_batch;

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

/// One neighbour drone's shared state, decoded from a `WireFleetEntry`.
#[cfg(feature = "embedded")]
#[derive(Clone, Copy, defmt::Format)]
pub struct NeighborState {
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    /// Age of the estimate at the GS send instant.
    pub age: embassy_time::Duration,
    /// False when the GS had no fresh estimate for this slot — ignore the pose.
    pub valid: bool,
}

/// Snapshot of the whole fleet's state as received from the ground station.
/// `states[i]` is drone `i` (fleet order); `self_idx` is this drone's own slot.
///
/// Safe access for consumers (avoidance / formation): use [`Self::message_age`]
/// / [`Self::is_stale`] to reject a snapshot whose *whole* fleet-state frame is
/// too old (GS broadcaster stopped or the link dropped), and iterate only
/// [`Self::neighbors`] — which already excludes this drone, invalid slots, and
/// per-entry-stale slots.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct NeighborStates {
    /// Local RX instant — the basis for whole-message staleness.
    pub timestamp: Instant,
    /// UTC microseconds at GS send (for end-to-end latency).
    pub gs_send_time_utc_us: i64,
    pub self_idx: u8,
    pub count: u8,
    pub states: [NeighborState; wire::MAX_FLEET],
}

#[cfg(feature = "embedded")]
impl NeighborStates {
    /// Decode a received `WireFleetState`. `rx_time` is the local RX instant.
    pub fn from_wire(w: &wire::WireFleetState, rx_time: Instant) -> Self {
        let mut states = [NeighborState {
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            age: embassy_time::Duration::from_ticks(0),
            valid: false,
        }; wire::MAX_FLEET];
        // Copy packed fields/array by value before use (UB to reference them).
        let count = w.count;
        let entries = w.entries;
        for (i, slot) in states.iter_mut().enumerate() {
            if i >= count as usize {
                break;
            }
            let e = entries[i];
            let flags = e.flags;
            let pos = e.position;
            let vel = e.velocity;
            let age_ms = e.age_ms;
            *slot = NeighborState {
                position: Vector3::new(pos[0], pos[1], pos[2]),
                velocity: Vector3::new(vel[0], vel[1], vel[2]),
                age: embassy_time::Duration::from_millis(age_ms as u64),
                valid: flags & wire::FLEET_FLAG_VALID != 0,
            };
        }
        Self {
            timestamp: rx_time,
            gs_send_time_utc_us: w.gs_send_time_us,
            self_idx: w.self_idx,
            count,
            states,
        }
    }

    /// How long since this fleet-state frame was received.
    pub fn message_age(&self, now: Instant) -> embassy_time::Duration {
        now.checked_duration_since(self.timestamp)
            .unwrap_or(embassy_time::Duration::from_ticks(0))
    }

    /// True if the whole frame is older than `max_age` — treat all neighbours
    /// as unavailable (the GS stopped broadcasting or the link dropped).
    pub fn is_stale(&self, now: Instant, max_age: embassy_time::Duration) -> bool {
        self.message_age(now) > max_age
    }

    /// This drone's own slot, if the GS included it.
    pub fn self_state(&self) -> Option<&NeighborState> {
        let i = self.self_idx as usize;
        (i < self.count as usize).then(|| &self.states[i])
    }

    /// Iterate the *usable* neighbours: populated, `valid`, not this drone, and
    /// individually no older than `max_entry_age`. Returns nothing once the
    /// frame itself is stale (`is_stale`), so a single guard covers both the
    /// whole-link-down and per-drone-dropout cases.
    pub fn neighbors(
        &self,
        now: Instant,
        max_msg_age: embassy_time::Duration,
        max_entry_age: embassy_time::Duration,
    ) -> impl Iterator<Item = (usize, &NeighborState)> {
        let frame_ok = !self.is_stale(now, max_msg_age);
        let self_idx = self.self_idx as usize;
        let count = self.count as usize;
        self.states
            .iter()
            .enumerate()
            .take(count)
            .filter(move |(i, s)| {
                frame_ok && *i != self_idx && s.valid && s.age <= max_entry_age
            })
    }
}

#[cfg(feature = "embedded")]
impl Message for NeighborStates {}

/// A peer drone's pose decoded from [`wire::WirePeerPose`] (direct leader→chaser
/// link). lat/lon in **radians**, alt in metres MSL — ready to reproject into the
/// local ENU via `LlhOrigin::llh_to_enu`.
#[cfg(feature = "embedded")]
#[derive(Clone, Copy, defmt::Format)]
pub struct PeerPose {
    /// Local RX instant — basis for staleness.
    pub rx: Instant,
    /// Sender capture time, UTC microseconds.
    pub timestamp_us: u64,
    pub lat_rad: f64,
    pub lon_rad: f64,
    pub alt_m: f32,
    /// Sender's ESKF has converged.
    pub converged: bool,
    /// Sender has a ≥3D GNSS fix.
    pub has_3d_fix: bool,
}

#[cfg(feature = "embedded")]
impl PeerPose {
    pub fn from_wire(w: &wire::WirePeerPose, rx: Instant) -> Self {
        const DEG2RAD: f64 = core::f64::consts::PI / 180.0;
        // Copy packed fields before use (unaligned reads are UB by reference).
        let lat_1e7 = w.lat_1e7;
        let lon_1e7 = w.lon_1e7;
        let alt_mm = w.alt_mm;
        let status = w.status;
        let timestamp_us = w.timestamp_us;
        Self {
            rx,
            timestamp_us,
            lat_rad: (lat_1e7 as f64) * 1e-7 * DEG2RAD,
            lon_rad: (lon_1e7 as f64) * 1e-7 * DEG2RAD,
            alt_m: (alt_mm as f32) * 1e-3,
            converged: status & wire::PEER_FLAG_CONVERGED != 0,
            has_3d_fix: status & wire::PEER_FLAG_3D_FIX != 0,
        }
    }

    /// How long since this pose was received.
    pub fn age(&self, now: Instant) -> embassy_time::Duration {
        now.checked_duration_since(self.rx)
            .unwrap_or(embassy_time::Duration::from_ticks(0))
    }

    /// Usable for tracking: sender converged + has a 3D fix + fresh.
    pub fn usable(&self, now: Instant, max_age: embassy_time::Duration) -> bool {
        self.converged && self.has_3d_fix && self.age(now) <= max_age
    }
}

#[cfg(feature = "embedded")]
impl Message for PeerPose {}

/// Ground-station session origin (msg 133), decoded and validated.
///
/// Published by the ESP-bridge RX task on `sensors::GNSS_ORIGIN_CMD` and
/// consumed by the GPS estimator, which latches it as its ENU origin.
#[cfg(feature = "embedded")]
#[derive(Clone, Copy, defmt::Format)]
pub struct GnssOriginCmd {
    /// Local RX instant.
    pub rx: Instant,
    pub lat_rad: f64,
    pub lon_rad: f64,
    /// WGS84 ellipsoidal height [m].
    pub h_ell_m: f32,
}

#[cfg(feature = "embedded")]
impl GnssOriginCmd {
    /// `None` when the VALID flag is clear or a field is non-finite or out
    /// of range (see [`wire::WireGnssOriginSet::validated`]).
    pub fn from_wire(w: &wire::WireGnssOriginSet, rx: Instant) -> Option<Self> {
        let (lat_deg, lon_deg, h_ell_m) = w.validated()?;
        Some(Self {
            rx,
            lat_rad: lat_deg.to_radians(),
            lon_rad: lon_deg.to_radians(),
            h_ell_m,
        })
    }

    /// How long since this command was received.
    pub fn age(&self, now: Instant) -> embassy_time::Duration {
        now.saturating_duration_since(self.rx)
    }
}

#[cfg(feature = "embedded")]
impl Message for GnssOriginCmd {}

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
#[derive(Clone, Copy, defmt::Format)]
pub struct MotorDynamicsTelemetry {
    pub omega: f32,
    pub omega_dot: f32,
    pub raw: Option<f32>,
}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct MotorStateTelemetry {
    pub timestamp: Instant,
    pub motors: [MotorDynamicsTelemetry; 4],
}
#[cfg(feature = "embedded")]
impl Message for MotorStateTelemetry {}


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
#[derive(Clone, defmt::Format)]
pub struct PowerStatus {
    pub timestamp: Instant,
    /// Battery voltage in centivolts (0.01 V).
    pub voltage_cv: u16,
    /// Current draw in centiamps (0.01 A).
    pub current_ca: i32,
    /// Milliamp-hours consumed since power-on.
    pub mah_drawn: u32,
    /// Detected cell count (0 = no battery).
    pub cell_count: u8,
}

#[cfg(feature = "embedded")]
impl Message for PowerStatus {}

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
    /// Wall time of the learned cost policy's observation + inference
    /// this tick [µs]. 0 when no policy is baked/enabled (and always 0
    /// under `outer_mpc_full`). Runs off the command's critical path —
    /// after the publish — so this is budget telemetry, not latency.
    pub policy_time_us: u32,
}
#[cfg(feature = "embedded")]
impl Message for OcpSolverOutput {}

/// Width of the learned cost policy's output (`cybflight_core`'s
/// `mpc::cost_adapt::NZ`): 8 stage + 8 terminal state residuals + 3
/// body-rate input weights.
#[cfg(feature = "embedded")]
pub const MPC_COST_NZ: usize = 19;

/// One outer-loop tick of the learned cost adaptation
/// (docs/learned_mpc_cost_deploy.md): the policy output `z`, the
/// effective weights it produced, and its wall-clock cost. Without this
/// record a flight cannot be compared against the sim probes.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct MpcCostAdapt {
    pub timestamp: Instant,
    /// Raw policy output after the `[−1, 1]` clamp, before `gain`.
    pub z: [f32; MPC_COST_NZ],
    /// Effective weights after `gain` + the log map: stage contour, lag,
    /// and the three body-rate input weights.
    pub w_contour: f32,
    pub w_lag: f32,
    pub w_rate: [f32; 3],
    /// `mpc_learned_gain` at this tick.
    pub gain: f32,
    /// Observation + forward-pass wall time.
    pub policy_time_us: u32,
}
#[cfg(feature = "embedded")]
impl Message for MpcCostAdapt {}

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

/// ESKF-estimated IMU biases — gyro (rad/s) and accel (m/s^2).
///
/// Decimated mirror of the per-predict-tick `ESKF_GYRO_BIAS` /
/// `ESKF_ACCEL_BIAS` Signals in `crate::estimation`. Published at
/// ~10 Hz on `ESTIMATOR_BIAS_TELEM`; biases drift at seconds-scale
/// so the channel rate is set well below the 1 kHz predict rate to
/// keep blackbox bandwidth bounded. Used post-flight for
/// bias-corrected accel reconstruction in INDI sysid fits.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct EstimatorBias {
    pub timestamp: Instant,
    pub gyro_bias_rad_s: Vector3<f32>,
    pub accel_bias_m_s2: Vector3<f32>,
}

#[cfg(feature = "embedded")]
impl Message for EstimatorBias {}

/// Per-solve diagnostics carried alongside [`MissionStatus`].
///
/// Populated once by the mission planner when it finalizes a trajectory
/// and then propagated verbatim through every Executing-state tick so the
/// ground station can inspect the *last* solve at any point during flight.
/// For Planning-state heartbeats and Idle (no solve ever completed),
/// `status == SOLVE_STATUS_NONE` and the rest of the fields are zero.
#[cfg(feature = "embedded")]
#[derive(Clone, Copy, defmt::Format)]
pub struct SolveDiagnostics {
    /// `SolverStatus` as a byte. See [`SOLVE_STATUS_*`](SOLVE_STATUS_CONVERGENCE).
    pub status: u8,
    /// BFGS outer iterations consumed by the solve.
    pub iterations: u16,
    /// Wall-clock solve duration in milliseconds.
    pub solve_time_ms: u16,
    /// Number of polynomial pieces in the published trajectory.
    pub num_pieces: u8,
    /// Pre-BFGS sum of segment times (initial time allocation). Compare
    /// against `total_duration_s` to see how much the optimizer actually
    /// compressed the trajectory — ratio ≈ 1 means BFGS made no progress.
    pub init_duration_s: f32,
    /// Final optimized cost value.
    pub final_cost: f32,
    /// Peak ‖v‖ sampled across the published trajectory (m/s).
    pub peak_vel_m_s: f32,
    /// Planner-side reject reason. `SOLVE_REJECT_NONE` on the happy path;
    /// any non-zero value is a breadcrumb explaining why a Planning →
    /// Idle transition occurred without reaching Executing.
    pub reject_reason: u8,
}

/// Sentinel: no solve has completed yet (Planning heartbeat / Idle state).
pub const SOLVE_STATUS_NONE: u8 = 0xFF;
/// `SolverStatus::Convergence`.
pub const SOLVE_STATUS_CONVERGENCE: u8 = 0;
/// `SolverStatus::Stop`.
pub const SOLVE_STATUS_STOP: u8 = 1;
/// `SolverStatus::MaxIterations`.
pub const SOLVE_STATUS_MAX_ITERATIONS: u8 = 2;
/// `SolverStatus::InvalidValue`.
pub const SOLVE_STATUS_INVALID_VALUE: u8 = 3;
/// `SolverStatus::TimeExceeded`.
pub const SOLVE_STATUS_TIME_EXCEEDED: u8 = 4;

/// No reject — the happy path, or a state other than a fresh reject
/// transition.
pub const SOLVE_REJECT_NONE: u8 = 0;
/// Wall-clock `SOLVE_BUDGET` elapsed before the solver converged.
pub const SOLVE_REJECT_BUDGET_EXCEEDED: u8 = 1;
/// `IS_ARMED` cleared mid-solve — pilot disarmed before the plan landed.
pub const SOLVE_REJECT_DISARMED: u8 = 2;
/// Solver converged to a numerically-invalid iterate: non-finite cost,
/// `dur` out of `[MIN, MAX]`, or `InvalidValue` status.
pub const SOLVE_REJECT_INVALID: u8 = 3;
/// Optimised duration is ≥ 90% of the initial time allocation, meaning
/// BFGS failed to meaningfully compress the seed trajectory.
pub const SOLVE_REJECT_UNDER_COMPRESSED: u8 = 4;
/// Pilot released the mission switch during the solve.
pub const SOLVE_REJECT_USER_ABORT: u8 = 5;
/// `MISSION_STATE` was changed externally (failsafe, abort handler) while
/// the solver was running — trajectory discarded.
pub const SOLVE_REJECT_STATE_CHANGED: u8 = 6;
/// Race detected inside the publish critical section (abort flag flipped
/// or state changed between the check and the slot write).
pub const SOLVE_REJECT_PUBLISH_RACE: u8 = 7;
/// Offline profile's timestamps are not strictly increasing from > 0.
pub const SOLVE_REJECT_INVALID_TIMES: u8 = 8;
/// Offline profile is structurally unusable for this solve (e.g. the live
/// head is too far from a home-relative mission's first waypoint).
pub const SOLVE_REJECT_PROFILE_INVALID: u8 = 9;
/// A home-relative mission was requested with no HOME latched.
pub const SOLVE_REJECT_NO_HOME: u8 = 10;

#[cfg(feature = "embedded")]
impl SolveDiagnostics {
    /// All-zero / no-solve-yet sentinel. Use for Planning heartbeats and
    /// any other publish site that has no solver result to report.
    pub const NONE: Self = Self {
        status: SOLVE_STATUS_NONE,
        iterations: 0,
        solve_time_ms: 0,
        num_pieces: 0,
        init_duration_s: 0.0,
        final_cost: 0.0,
        peak_vel_m_s: 0.0,
        reject_reason: SOLVE_REJECT_NONE,
    };
}

#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct MissionStatus {
    pub timestamp: Instant,
    pub state: u8,
    pub tau_s: f32,
    pub total_duration_s: f32,
    pub target_position: Vector3<f32>,
    /// Diagnostics from the most recent solve. See [`SolveDiagnostics`].
    pub solve: SolveDiagnostics,
}

#[cfg(feature = "embedded")]
impl Message for MissionStatus {}

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

/// Per-tick tracking error reported directly by whichever controller
/// just ran. Each source publishes only the fields it computes; the
/// remaining fields are zero. Distinguished by [`source`](Self::source):
///
/// - [`TRACKING_ERROR_SOURCE_CASCADE`] (0): cascade outer loop. Fills
///   `pos_err` and `vel_err` (raw `reference - actual`, *before* the
///   controller's `p_err_max`/`v_err_max` clamp). Pair with the
///   controller's known clamp limits at decode time to spot
///   saturation. Attitude / rate err are zero — cascade hands
///   attitude downstream to the geometric controller, and rate-
///   tracking is INDI's domain.
/// - [`TRACKING_ERROR_SOURCE_MPC`] (1): MPC outer loop. Fills `pos_err`,
///   `vel_err` (raw `xref - x` since MPC enforces error envelope
///   through cost weights, not clamps), and `attitude_err` — the
///   tilt-prioritizing 3-vec from
///   `cybflight_core::mpc::model_utils::attitude_error`. Body-rate err
///   is zero — MPC commands rates downstream rather than tracking
///   them.
/// - [`TRACKING_ERROR_SOURCE_INDI`] (2): INDI inner loop. Fills
///   `body_rate_err` (= `rate_ref - gyro_corrected`) only. Position /
///   velocity / attitude err are zero — INDI sees neither.
///
/// All errors are in the **world frame** for pos/vel, **body frame**
/// for attitude (tilt-prio Euler-like 3-vec) and rate. Sign convention:
/// `error = reference - actual`, so a positive `pos_err.x` means the
/// reference is *ahead* of the vehicle along world-x.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct TrackingError {
    pub timestamp: Instant,
    pub pos_err: Vector3<f32>,
    pub vel_err: Vector3<f32>,
    pub attitude_err: Vector3<f32>,
    pub body_rate_err: Vector3<f32>,
    pub source: u8,
}

#[cfg(feature = "embedded")]
impl Message for TrackingError {}

pub const TRACKING_ERROR_SOURCE_CASCADE: u8 = 0;
pub const TRACKING_ERROR_SOURCE_MPC: u8 = 1;
pub const TRACKING_ERROR_SOURCE_INDI: u8 = 2;

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
    GoToPosition { position: Vector3<f32>, yaw: f32 },
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

// ── GNSS Status (APX-15 INS) ───────────────────────────────────────────────

/// Comprehensive GNSS/INS status from APX-15 for go/no-go decision making.
///
/// Carries quality indicators from GSOF messages #49, #50, #12, #1, #38,
/// #9, #35, #40, #10 needed to determine whether the INS solution is
/// usable for flight.
#[cfg(feature = "embedded")]
#[derive(Clone, defmt::Format)]
pub struct GnssStatus {
    pub timestamp: Instant,

    // From #49 (InsFullNav) — status bytes
    pub imu_alignment: u8,
    pub gps_quality: u8,

    // From #50 (InsRmsInfo) — position, velocity, attitude RMS
    pub north_pos_rms: f32,
    pub east_pos_rms: f32,
    pub down_pos_rms: f32,
    pub north_vel_rms: f32,
    pub east_vel_rms: f32,
    pub down_vel_rms: f32,
    pub roll_rms: f32,
    pub pitch_rms: f32,
    pub heading_rms: f32,

    // From #12 (PositionSigmaInfo)
    pub sigma_east: f32,
    pub sigma_north: f32,
    pub sigma_up: f32,

    // From #1 (PositionTime)
    pub num_svs: u8,
    pub pos_flags1: u8,
    pub pos_flags2: u8,

    // From #38 (PositionTypeInfo)
    pub solution_flags: u8,
    pub rtk_condition: u8,
    pub correction_age: f32,
    pub network_flags: u8,
    pub position_fix_type: u8,

    // From #9 (PdopInfo)
    pub pdop: f32,
    pub hdop: f32,
    pub vdop: f32,
    pub tdop: f32,

    // From #35 (ReceivedBaseInfo)
    pub base_id: u16,

    // From #40 (LbandStatus)
    pub lband_snr: f32,
    pub lband_hp_xp_mode: u8,
    pub lband_vbs_mode: u8,

    // From #10 (ClockInfo)
    pub clock_flags: u8,
    pub clock_offset_ms: f32,
    pub freq_offset_ppm: f32,
}

#[cfg(feature = "embedded")]
impl Message for GnssStatus {}
