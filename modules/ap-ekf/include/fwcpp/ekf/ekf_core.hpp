#pragma once

// Port of AP_NavEKF3's state vector, strapdown INS mechanization, and
// covariance time-propagation. CPP-052, PHASE 1 of a large multi-phase
// epic - see this file's own "OUT OF SCOPE" section below before assuming
// anything not listed here exists.
//
// Upstream: AP_NavEKF3/AP_NavEKF3_core.h (state_elements struct, ~line
// 566; Matrix24/Vector24 typedefs, ~line 531-558) and
// AP_NavEKF3_core.cpp (NavEKF3_core::UpdateStrapdownEquationsNED(), ~line
// 743; NavEKF3_core::CovariancePrediction(), ~line 1008;
// NavEKF3_core::CovarianceInit(), ~line 573; NavEKF3_core::ConstrainStates(),
// ~line 2016; NavEKF3_core::ConstrainVariances(), ~line 1877;
// NavEKF3_core::calcEarthRateNED(), ~line 2066; NavEKF3_core::
// InitialGyroBiasUncertainty(), AP_NavEKF3_GyroBias.cpp ~line 20) - all
// read directly from the pinned Plane-4.7.0 upstream worktree.
//
// ============================================================================
// SCALE WARNING (reproduced from ticket CPP-052 - read before extending
// this file): upstream's real AP_NavEKF3 is ~17,000 lines: a genuine
// 24-state extended Kalman filter with multi-IMU/multi-lane management,
// GPS/baro/compass/optical-flow/rangefinder fusion, innovation-consistency
// gating, and a Gaussian Sum Filter yaw estimator. This file is
// deliberately NOT that. It is the first of a multi-phase epic.
// ============================================================================
//
// WHAT THIS PHASE BUILDS:
//   - The real 24-element state vector (state_elements, upstream
//     AP_NavEKF3_core.h:566-575), all 24 conceptual slots present.
//   - The real 24x24 covariance matrix P: upstream's traced (not invented)
//     initial values (CovarianceInit()) and its real time-propagation
//     (CovariancePrediction() - the process-noise Jacobian for the
//     quaternion/velocity/position/gyro-bias/accel-bias block is
//     transcribed verbatim from upstream's auto-generated symbolic
//     expressions, see ekf_core.cpp's own banner).
//   - UpdateStrapdownEquationsNED()'s real mechanization: quaternion
//     update from bias-corrected delta-angle (with earth-rotation-rate
//     compensation), trapezoidal velocity/position integration from
//     bias-corrected, gravity-compensated delta-velocity.
//
// WHAT THIS PHASE DOES NOT BUILD (named per-file, matching the ticket's
// acceptance criterion - this is the real roadmap for phase 2+, not a
// commitment to any particular next order):
//   - AP_NavEKF3_PosVelFusion.cpp - GPS/baro/external-nav position and
//     velocity fusion (~2073 lines).
//   - AP_NavEKF3_MagFusion.cpp - 3-axis and yaw-only magnetometer fusion,
//     declination fusion (~1544 lines).
//   - AP_NavEKF3_OptFlowFusion.cpp - optical flow fusion.
//   - AP_NavEKF3_RngBcnFusion.cpp - range beacon fusion.
//   - AP_NavEKF3_Measurements.cpp - multi-IMU/multi-sensor sample
//     buffering, the delayed fusion time horizon, and inactiveBias[]
//     per-IMU bias bookkeeping (~1555 lines). This port has one IMU and no
//     delay buffer - see SIMPLIFICATION notes below.
//   - AP_NavEKF3.cpp - multi-core/multi-lane management, lane switching,
//     core arbitration (~2173 lines).
//   - AP_NavEKF3_Control.cpp - health/mode logic, inhibitMagStates/
//     inhibitWindStates/inhibitDelAngBiasStates/inhibitDelVelBiasStates
//     control-flow, badIMUdata vibration detection, onGround/dvelBiasAxis
//     ground-alignment gating. Phase 1 hardcodes the permanent-no-fusion
//     equivalents of these flags - see SIMPLIFICATION notes below.
//   - The GSF (Gaussian Sum Filter) yaw estimator.
//   - AP_NavEKF3_Outputs.cpp - output-buffer/complementary-filter
//     blending between the fusion time horizon and "now".
//   - Innovation gating / consistency checks (no fusion exists to gate).
//   - Wiring this estimator into Plane as an alternative to AhrsDcm - a
//     separate, later integration decision once fusion exists.
//
// SIMPLIFICATIONS MADE IN THIS PHASE (each is a real, documented
// divergence from upstream behavior, not a bug - "port fixes bugs, not
// upstream" / "register every divergence"):
//
//   1. inhibitMagStates and inhibitWindStates are permanently TRUE (no
//      compass/airspeed fusion exists in this phase to ever clear them -
//      upstream sets these in AP_NavEKF3_Control.cpp, out of scope). Per
//      upstream's OWN logic for this condition (ConstrainVariances(),
//      ~line 1970-1994), this means P[16..21] (mag) and P[22..23] (wind)
//      are zeroed every predict cycle - covariance_init() sets their real
//      upstream initial values once, and the very first covariance_
//      prediction() call zeros them again, exactly reproducing upstream's
//      real behavior for a configuration with no magnetometer/airspeed
//      fusion enabled. This is NOT a shortcut - computing their would-be
//      correlation growth with the 0-9 block would be wasted work upstream
//      itself throws away in this condition.
//
//   2. inhibitDelAngBiasStates and inhibitDelVelBiasStates are permanently
//      FALSE (gyro/accel bias states keep accumulating real process-noise
//      uncertainty every cycle, matching normal in-flight operation) - but
//      the per-axis ground-alignment observability gate
//      (dvelBiasAxisInhibit[], tied to onGround/prevTnb, CovariancePrediction
//      ~line 158-173) is NOT reproduced (it depends on an onGround flag
//      this port's EkfCore has no source for in phase 1). Accel bias
//      variance can therefore grow slightly more freely on the ground than
//      upstream's real ground-alignment gate would allow - harmless for
//      the pure free-integrating-INS behavior this phase targets, and
//      exactly the kind of thing phase 2 (which will need onGround/mode
//      context anyway for real fusion gating) should revisit.
//
//   3. badIMUdata (vibration detection, AP_NavEKF3_Control.cpp) is
//      permanently FALSE - no vibration-detection subsystem in this port.
//      Accel process noise always uses the normal acc_noise parameter,
//      never the elevated BAD_IMU_DATA_ACC_P_NSE fallback.
//
//   4. Single-IMU bias correction: upstream's correctDeltaAngle()/
//      correctDeltaVelocity() (AP_NavEKF3_core.cpp ~line 726) subtract
//      inactiveBias[gyro_index]/[accel_index] - a per-IMU bias bookkeeping
//      array that exists to let multiple simultaneous IMU instances share
//      one set of EKF bias *states* while independently tracking their own
//      raw sensor offsets (AP_NavEKF3_Measurements.cpp, out of scope: no
//      multi-IMU affinity/blending in this port). With one IMU, upstream's
//      inactiveBias[active_index] IS this filter's own stateStruct.
//      gyro_bias/accel_bias - so update_strapdown_equations_ned()
//      subtracts state.gyro_bias/state.accel_bias directly. Bit-identical
//      to upstream's real single-IMU behavior, not an approximation of it.
//
//   5. No fusion time-horizon delay buffer (AP_NavEKF3_Measurements.cpp's
//      storedIMU FIFO, ~150ms typical). Upstream runs strapdown mechanics
//      on a delayed IMU sample so it lines up with delayed GPS/baro/mag
//      measurements at fusion time; with no fusion in this phase there is
//      no other measurement stream to align with, so "the current sample"
//      IS "the delayed sample" trivially. Phase 2 (any real fusion type)
//      will need to reintroduce this buffer.
//
//   6. dtEkfAvg (upstream: a running low-pass-filtered average IMU
//      timestep, itself computed in AP_NavEKF3_Measurements.cpp from the
//      downsampled IMU rate, out of scope) is an explicit caller-supplied
//      input here (ADR-0012: no singleton AP::ins().get_loop_rate_hz()
//      read). This mirrors AhrsDcm's own explicit-input pattern (this
//      class does no polling/averaging of its own; the caller - whoever
//      owns the real IMU driver - is expected to maintain it, e.g. as a
//      simple exponential filter of consecutive dt samples, matching
//      upstream's own `dtEkfAvg = 0.98f * dtEkfAvg + 0.02f * dtNow`).
//
//   7. Earth-rotation-rate compensation (calcEarthRateNED(), upstream
//      home-latitude-dependent) is an explicit `earth_rate_ned` field the
//      caller sets via calc_earth_rate_ned(latitude_rad) once a home
//      location is known (ADR-0012: no AP::ahrs().get_home() singleton
//      read). Defaults to zero (no compensation) until the caller sets
//      it - matching upstream behavior before a home location exists.
//
//   8. MagTableConstrain() (World Magnetic Model table lookup,
//      ConstrainStates()'s alternate earth-magfield clamp path) is not
//      ported - this phase always takes upstream's simpler unconditional
//      +-1 Gauss clamp branch (have_table_earth_field is always false
//      here, matching a fresh port with no WMM table wired in).
//
//   9. Terrain state / rangefinder-ground clamp (ConstrainStates()'s final
//      `if (!inhibitGndState) terrainState = MAX(...)` line) is excluded
//      entirely - terrainState is a separate scalar outside the 24-element
//      state_elements vector and belongs with AP_NavEKF3_RngBcnFusion.cpp/
//      terrain-following, out of scope here.
//
// NO SINGLETONS, EXPLICIT INPUTS INSTEAD (ADR-0012), matching AhrsDcm's
// GyroSample/AccelSample pattern exactly (see ap-ahrs/ahrs_dcm.hpp):
//   - GyroSample/AccelSample REPLACE AP::ins() reads.
//   - earth_rate_ned REPLACES AP::ahrs().get_home()-derived latitude.
//   - dt_ekf_avg is a per-call parameter, not a polled loop rate.
//   - Noise/limit parameters (gyr_noise, acc_noise, gps_horiz_vel_noise,
//     etc.) are plain public fields defaulted to the real upstream
//     Plane-4.7.0 parameter defaults (AP_NavEKF3.cpp's APM_BUILD_ArduPlane
//     block) - same "AP_Param not wired in yet" treatment as AhrsDcm's
//     kp_yaw.
//
// ftype: mirrors upstream's AP_Math/ftype.h HAL_WITH_EKF_DOUBLE switch via
// this port's own FWCPP_EKF_DOUBLE CMake option (top-level CMakeLists.txt,
// already declared - this ticket is its first consumer). Kept LOCAL to
// this module (fwcpp::ekf::ftype), not hoisted into ap-math alongside
// postype_t, deliberately: five parallel agents are editing disjoint
// modules this round, and ap-math is not part of this ticket's scope.
// Phase 2+ may want to hoist it into ap-math as a shared primitive
// (upstream's own ftype.h is shared AP_Math, after all) once that can be
// coordinated outside a multi-agent parallel session.
//
// ============================================================================
// CPP-056, PHASE 2 (this ticket): the first real measurement update.
// Everything above this point is phase 1 (CPP-052), unmodified. Read
// FuseVelPosNED() in full (AP_NavEKF3_PosVelFusion.cpp ~line 694-1181)
// before extending anything below - it was read in full for this phase,
// not skimmed from the ticket's own summary of it.
// ============================================================================
//
// WHAT THIS PHASE BUILDS:
//   - fuse_direct_state_observation(): the reusable "fuse one direct
//     observation of a single state element" primitive, transcribed
//     line-by-line from FuseVelPosNED()'s obsIndex loop body (~line
//     1024-1163) for the case that matters for GPS vel/pos: real GPS-
//     derived aiding, i.e. upstream's PV_AidingMode == AID_ABSOLUTE (see
//     "NOT PORTED FROM FuseVelPosNED" below for exactly what that
//     excludes and why it's the right cut).
//   - fuse_gps_velocity() / fuse_gps_position(): the GPS-specific
//     obsIndex 0-2 / 3-4 callers, including the real R_OBS "no reported
//     accuracy" noise-variance formula.
//   - GpsSample: a new, EKF-specific explicit input struct (see below for
//     why ahrs::GpsSample, used by AhrsDcm, is NOT reused as-is).
//   - Four real inhibitDelAngBiasStates/inhibitDelVelBiasStates/
//     inhibitMagStates/inhibitWindStates bool fields (see "CORRECTION TO
//     THE CPP-056 TICKET'S OWN PREMISE" below).
//
// CORRECTION TO THE CPP-056 TICKET'S OWN PREMISE: the ticket asserts
// phase 1 "already has real inhibitDelAngBiasStates/
// inhibitDelVelBiasStates/inhibitMagStates/inhibitWindStates fields".
// Verified directly against the phase-1 source (ekf_core.cpp) before
// writing any phase-2 code: this is FALSE. Phase 1 has no such fields -
// the equivalent permanently-true/permanently-false behavior is baked
// directly into constrain_variances()'s and covariance_prediction()'s
// control flow (unconditional branches, described only in prose
// comments). Since fuse_direct_state_observation() genuinely needs these
// as real, gateable fields (upstream's own Kfusion computation branches
// on them per-call, and a future phase may want to flip inhibitMagStates
// once mag fusion exists), this phase adds them as real public bool
// fields below, defaulted to reproduce phase 1's existing hardcoded
// behavior exactly (inhibit_del_ang_bias_states=false,
// inhibit_del_vel_bias_states=false, inhibit_mag_states=true,
// inhibit_wind_states=true) - a behavior-preserving correction, not a
// new simplification. constrain_variances()/covariance_prediction()
// themselves are NOT modified to read these fields dynamically (that
// would mean touching the delicate, independently-verified transcribed
// Jacobian block in ekf_core.cpp for no behavioral gain at today's fixed
// defaults) - a named gap: a future phase that actually wants to flip
// inhibit_mag_states at runtime must also wire it into those two
// functions's currently-hardcoded branches.
//
// NOT PORTED FROM FuseVelPosNED() (named per the ticket's own acceptance
// criterion - each is a real upstream mechanism, verified present in the
// ~694-1181 read, deliberately left out of fuse_direct_state_observation
// because it belongs to a mode/feature this port doesn't have yet):
//   - The `PV_AidingMode == AID_NONE` "poorObservability" gyro-bias-
//     Kalman-gain gate and "horizInhibit" delta-velocity-bias gate
//     (~line 1051-1084). Both are guarded by `PV_AidingMode == AID_NONE`
//     (upstream's fake-position/fake-velocity "hold attitude without
//     real aiding" mode) and are UNCONDITIONALLY SKIPPED whenever
//     PV_AidingMode == AID_ABSOLUTE, which is the real mode real GPS
//     fusion runs under - so omitting them here is not an approximation,
//     it's the literal AID_ABSOLUTE-mode behavior of upstream's own
//     code, verified by reading the guard conditions directly.
//   - `dvelBiasAxisInhibit[]` (per-axis delta-velocity-bias inhibit tied
//     to onGround/ground-alignment, AP_NavEKF3_Control.cpp) - already a
//     named phase-1 gap (see simplification 2 above); this phase's
//     inhibit_del_vel_bias_states gate is all-or-nothing across x/y/z,
//     same as phase 1.
//   - `badIMUdata` gating of the delta-velocity-bias Kalman gain - always
//     false, per phase-1 simplification 3.
//   - `treatWindStatesAsTruth` narrowing the wind-state Kalman gain gate
//     (`!inhibitWindStates && !treatWindStatesAsTruth`) - no such field
//     exists in this port (no optical-flow/const-position-hold subsystem
//     that would ever set it); moot in this phase anyway since
//     inhibit_wind_states is permanently true.
//   - `gpsNoiseScaler` (satellite-count-based R_OBS scaling, applied
//     inside the obsIndex loop at ~line 1027/1032) - named in the ticket
//     as part of SelectVelPosFusion()'s orchestration, out of scope here.
//   - The innovation-consistency test-ratio gates (velTestRatio/
//     posTestRatio, ~line 811-928) that decide fuseVelData/fusePosData
//     in the first place, and faultStatus.bad_* bookkeeping - only the
//     healthyFusion negative-variance guard is in scope (see ticket).
//     fuse_gps_velocity()/fuse_gps_position() therefore have NO way to
//     reject a bad GPS fix beyond that one guard - a REAL, NAMED GAP:
//     upstream's real innovation gating is what lets the filter refuse a
//     glitching GPS fix rather than being corrupted by it, and this
//     phase cannot do that yet.
//   - The GPS-reported-accuracy branch (`gpsSpdAccuracy > 0.0f` /
//     `gpsPosAccuracy > 0.0f`, ~line 736-756) and the ExtNav branches -
//     only the "no reported accuracy" R_OBS formula is ported (see
//     GpsSample's own comment for why the struct has no accuracy
//     fields).
//   - The delay-buffer / observation-time-horizon machinery
//     (AP_NavEKF3_Measurements.cpp's storedGPS/imuDataDelayed ring
//     buffers) that lets upstream fuse a GPS sample against the state as
//     of the time that sample was actually valid. fuse_gps_velocity()/
//     fuse_gps_position() fuse a GpsSample against the CURRENT state,
//     assuming it is already time-aligned - a real, disclosed
//     simplification, not upstream's real behavior. Any future use of
//     this against genuinely asynchronous sensor timing needs this
//     buffer reintroduced first.
//   - Height/baro fusion (obsIndex==5, selectHeightForFusion()) - no
//     baro sensor model exists in this port.
//   - ResetVelocity()/ResetPosition()/initial alignment, and
//     SelectVelPosFusion()'s full orchestration (timeout detection,
//     glitch-radius handling, the GPS-vertical-velocity-vs-baro aliasing
//     cross-check) - this phase calls the fusion functions directly with
//     an already-known-good GpsSample; it does not decide WHEN to call
//     them or what to do on a stale/bad fix.
//
// GpsSample vs. ahrs::GpsSample (ADR-0012 explicit-input check, per the
// ticket): read ap-ahrs/ahrs_dcm.hpp's GpsSample directly before adding a
// new struct. It carries ground_speed_ms/ground_course_deg/velocity_ned/
// num_sats/has_fix/has_3d_fix - built for AhrsDcm's drift-correction use,
// which never fuses absolute position. It has NO position field at all
// (DCM has no position state to correct). This phase's GPS position
// fusion needs a local-NE position observation, which ahrs::GpsSample
// cannot represent - so it is not "reusable as-is", and not a subset/
// superset relationship either (ahrs::GpsSample's ground_speed_ms/
// ground_course_deg polar form vs. this phase's NED velocity vector
// don't correspond 1:1). This phase therefore defines its own
// fwcpp::ekf::GpsSample below rather than extending or wrapping
// ahrs::GpsSample - a genuinely different explicit input, not a
// duplicate of an existing one.

#include <array>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::ekf {

#if FWCPP_EKF_DOUBLE
using ftype = double;
#else
using ftype = float;
#endif

using Vector2F = fwcpp::math::Vector2<ftype>;
using Vector3F = fwcpp::math::Vector3<ftype>;
using Matrix3F = fwcpp::math::Matrix3<ftype>;
using QuaternionF = fwcpp::math::QuaternionT<ftype>;

// upstream: sq(T), AP_Math/AP_Math.h - trivial, not worth a shared header
// dependency for one line.
template <typename T>
[[nodiscard]] constexpr T sq(T x) { return x * x; }

// upstream: NavEKF3_core::calcEarthRateNED(), AP_NavEKF3_core.cpp ~line
// 2066. earthRate constant: AP_NavEKF3_core.h:49, `#define earthRate
// 0.000072921f` - an explicitly float-precision literal even in a
// double-ftype build (upstream's own choice, reproduced exactly via the
// float() narrowing below rather than widened to full double precision).
[[nodiscard]] inline Vector3F calc_earth_rate_ned(ftype latitude_rad) {
    constexpr ftype kEarthRate = static_cast<ftype>(0.000072921f);
    return Vector3F(kEarthRate * std::cos(latitude_rad), ftype(0),
                     -kEarthRate * std::sin(latitude_rad));
}

// upstream: imu_elements' delAng/delAngDT fields (AP_NavEKF3_core.h ~589-
// 591), reshaped into AhrsDcm's GyroSample/AccelSample split rather than
// upstream's single combined imu_elements struct - see file banner's
// "NO SINGLETONS" section.
struct GyroSample {
    Vector3F delta_angle;      // upstream: imuDataDelayed.delAng, rad
    ftype delta_angle_dt = 0;  // upstream: imuDataDelayed.delAngDT, s
};

struct AccelSample {
    Vector3F delta_velocity;      // upstream: imuDataDelayed.delVel, m/s
    ftype delta_velocity_dt = 0;  // upstream: imuDataDelayed.delVelDT, s
};

// CPP-056 phase 2. upstream: gps_elements (AP_NavEKF3_core.h), the subset
// FuseVelPosNED()'s obsIndex 0-4 path actually reads (gpsDataDelayed.vel,
// and the local-NE position AP_NavEKF3_PosVelFusion.cpp ~line 574-580
// computes as `EKF_origin.get_distance_NE_ftype(gpsloc)` from the raw
// lat/lng fix). See this file's banner ("GpsSample vs. ahrs::GpsSample")
// for why this is a new struct, not a reuse of ap-ahrs's GpsSample.
//
// position_ne is ALREADY the local-NE projection, in metres relative to
// whatever local origin the caller's EKF instance uses - the raw-fix-to-
// local-NE projection itself (upstream: Location::get_distance_NE_ftype(),
// tied to the EKF_origin/home-location singleton) is explicitly out of
// scope for this phase (see banner) - the caller must have already done
// that conversion, matching this port's general explicit-input
// convention (ADR-0012).
struct GpsSample {
    Vector3F velocity_ned;  // upstream: gpsDataDelayed.vel, NED m/s
    Vector2F position_ne;   // upstream: velPosObs[3]/[4] source value, local NE metres
};

// upstream: state_elements, AP_NavEKF3_core.h:566-575. Field order and
// element-count comments match upstream's own byte-for-byte (quat 0..3,
// velocity 4..6, position 7..9, gyro_bias 10..12, accel_bias 13..15,
// earth_magfield 16..18, body_magfield 19..21, wind_vel 22..23 = 24
// total) - read directly from upstream, not assumed from the ticket's own
// summary of it.
struct StateVector {
    QuaternionF quat;               // 0..3   - local NED earth frame -> body frame
    Vector3F velocity;               // 4..6   - NED earth frame, m/s
    Vector3F position;               // 7..9   - NED earth frame, m
    Vector3F gyro_bias;              // 10..12 - body frame delta angle IMU bias, rad
    Vector3F accel_bias;             // 13..15 - body frame delta velocity IMU bias, m/s
    Vector3F earth_magfield;         // 16..18 - PHASE 1: zero-initialized and carried, never fused (see file banner)
    Vector3F body_magfield;          // 19..21 - PHASE 1: zero-initialized and carried, never fused
    Vector2F wind_vel;                // 22..23 - PHASE 1: zero-initialized and carried, never fused
};

// upstream: Matrix24, AP_NavEKF3_core.h:558 (`typedef ftype
// Matrix24[24][24]`) - a plain fixed 24x24, not this port's own VectorN.
using Matrix24 = std::array<std::array<ftype, 24>, 24>;

// EkfCore: the strapdown-INS-plus-covariance-prediction core described in
// this file's banner. NOT a drop-in AhrsDcm replacement (see banner's
// "explicitly out of scope" list) - a standalone estimator with no fusion,
// health/mode logic, or Plane wiring.
class EkfCore {
public:
    StateVector state{};
    Matrix24 P{};  // upstream: P (member of NavEKF3_core), covariance matrix

    // upstream: earthRateNED (member), set once via calcEarthRateNED() when
    // a home location becomes known - see file banner's simplification 7.
    // Zero (no earth-rotation compensation) until the caller sets it.
    Vector3F earth_rate_ned{};

    // upstream: prevTnb (member) - nav-to-body DCM from the previous
    // strapdown step, used both for earth-rate compensation and for
    // rotating the current step's body-frame delta-velocity into nav
    // frame. Starts as the identity (matches a freshly-constructed
    // identity-quaternion state).
    Matrix3F prev_tnb{Vector3F(ftype(1), ftype(0), ftype(0)), Vector3F(ftype(0), ftype(1), ftype(0)),
                       Vector3F(ftype(0), ftype(0), ftype(1))};

    // upstream: velDotNED/velDotNEDfilt (members) - computed by
    // UpdateStrapdownEquationsNED() for launch-detection/GPS-variance
    // consumers, neither of which exists in this phase. Exposed for
    // inspection/testing but not otherwise consumed here.
    Vector3F vel_dot_ned{};
    Vector3F vel_dot_ned_filt{};

    // upstream: PV_AidingMode != AID_NONE - collapsed to one bool since
    // this phase has exactly two states (aiding is always false: no
    // fusion of any kind exists to ever set it true). Exposed (not
    // hardcoded away) so phase 2 has an obvious place to start driving it
    // from real fusion-health logic.
    bool aiding = false;

    // upstream: vertVelVarClipCounter (member), ConstrainVariances()'s
    // persistent counter for the "vertical velocity variance has
    // collapsed" reset.
    std::uint32_t vert_vel_var_clip_counter = 0;

    // CPP-056 phase 2: real inhibitDelAngBiasStates/inhibitDelVelBiasStates/
    // inhibitMagStates/inhibitWindStates fields (AP_NavEKF3_core.h members,
    // set by AP_NavEKF3_Control.cpp - out of scope). Defaults reproduce
    // phase 1's existing hardcoded behavior exactly - see this file's
    // banner "CORRECTION TO THE CPP-056 TICKET'S OWN PREMISE" for why
    // these did not already exist and why constrain_variances()/
    // covariance_prediction() are not (yet) wired to read them.
    bool inhibit_del_ang_bias_states = false;  // upstream: inhibitDelAngBiasStates
    bool inhibit_del_vel_bias_states = false;  // upstream: inhibitDelVelBiasStates
    bool inhibit_mag_states = true;            // upstream: inhibitMagStates
    bool inhibit_wind_states = true;           // upstream: inhibitWindStates

    // --- Noise/limit parameters. Defaults are upstream's real
    // Plane-4.7.0 AP_NavEKF3.cpp APM_BUILD_ArduPlane parameter-default
    // block (traced directly, not invented) - "AP_Param not wired in
    // yet", same treatment as AhrsDcm's kp_yaw. ---
    ftype gyr_noise = static_cast<ftype>(1.5e-02f);          // GYRO_P_NSE_DEFAULT
    ftype acc_noise = static_cast<ftype>(3.5e-01f);          // ACC_P_NSE_DEFAULT
    ftype gyro_bias_process_noise = static_cast<ftype>(1.0e-03f);   // GBIAS_P_NSE_DEFAULT
    ftype accel_bias_process_noise = static_cast<ftype>(2.0e-02f);  // ABIAS_P_NSE_DEFAULT
    ftype gps_horiz_vel_noise = static_cast<ftype>(0.5f);    // VELNE_M_NSE_DEFAULT
    ftype gps_vert_vel_noise = static_cast<ftype>(0.7f);     // VELD_M_NSE_DEFAULT
    ftype gps_horiz_pos_noise = static_cast<ftype>(0.5f);    // POSNE_M_NSE_DEFAULT
    ftype baro_alt_noise = static_cast<ftype>(3.0f);         // ALT_M_NSE_DEFAULT
    ftype mag_noise = static_cast<ftype>(0.05f);             // MAG_M_NSE_DEFAULT (init value only - see banner simplification 1)
    ftype acc_bias_lim = static_cast<ftype>(1.0f);           // AP_GROUPINFO("ACC_BIAS_LIM", ..., 1.0f)

    // upstream: NavEKF3_core::InitialGyroBiasUncertainty(),
    // AP_NavEKF3_GyroBias.cpp - a fixed 2.5 deg/sec, not vehicle-specific
    // despite its own doc comment claiming otherwise (read directly:
    // `return 2.5f;`, unconditional).
    ftype initial_gyro_bias_uncertainty_deg_s = static_cast<ftype>(2.5f);

    // upstream: NavEKF3_core::CovarianceInit(), AP_NavEKF3_core.cpp ~line
    // 573. Requires state.quat already set (e.g. from an initial attitude
    // estimate) - matches upstream reading stateStruct.quat directly, not
    // a parameter to CovarianceInit() there either. dt_ekf_avg: see file
    // banner's simplification 6.
    void covariance_init(ftype dt_ekf_avg);

    // upstream: NavEKF3_core::UpdateStrapdownEquationsNED(),
    // AP_NavEKF3_core.cpp ~line 743, PLUS the ConstrainStates() call at
    // its end (~line 793 upstream: `ConstrainStates();`). dt_ekf_avg is
    // needed here too, for ConstrainStates()'s gyro/accel bias limits
    // (`GYRO_BIAS_LIMIT*dtEkfAvg`, `_accBiasLim*dtEkfAvg`).
    void update_strapdown_equations_ned(const GyroSample& gyro, const AccelSample& accel, ftype dt_ekf_avg);

    // upstream: NavEKF3_core::CovariancePrediction(Vector3F*
    // rotVarVecPtr), AP_NavEKF3_core.cpp ~line 1008, PLUS the
    // ConstrainVariances() call at its end (~line 783). `rot_var_vec`
    // non-null reproduces upstream's quaternion-covariance-RESET path
    // (upstream's only caller: CovarianceInit()); null is the normal
    // per-tick prediction path (upstream's only caller: UpdateFilter()).
    void covariance_prediction(const GyroSample& gyro, const AccelSample& accel, ftype dt_ekf_avg,
                                const Vector3F* rot_var_vec = nullptr);

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s obsIndex loop body,
    // AP_NavEKF3_PosVelFusion.cpp ~line 1024-1163 - see this file's
    // banner for exactly what is/isn't reproduced. `state_index` is the
    // observed state's own index (upstream: `stateIndex = 4 + obsIndex`,
    // computed by the caller here rather than derived from an obsIndex,
    // since this primitive is meant to be reusable beyond GPS obsIndex
    // 0-4 too - e.g. a future height-fusion phase's obsIndex==5, state
    // index 9). `innovation` is upstream's innovVelPos[obsIndex]
    // (observed convention: state MINUS observation, matching upstream's
    // own `stateStruct.velocity[i] - velPosObs[i]` - NOT the more common
    // observation-minus-prediction sign). `obs_variance` is upstream's
    // R_OBS[obsIndex]. `dt_ekf_avg` is required only because this
    // primitive calls the existing constrain_variances(ftype) - upstream
    // doesn't need to pass it because ConstrainVariances() reads its own
    // dtEkfAvg member directly; this port's explicit-input convention
    // (ADR-0012) has no such member to read, hence the extra parameter
    // versus the ticket's own suggested 3-argument signature.
    //
    // Returns whether the update was applied (upstream's local
    // `healthyFusion` bool) - false means the negative-variance guard
    // fired and P/state were left untouched, matching upstream's own
    // "skip the update" branch exactly (see banner: this is the ONLY
    // fusion-health check in scope for this phase, not full innovation-
    // consistency gating).
    bool fuse_direct_state_observation(int state_index, ftype innovation, ftype obs_variance, ftype dt_ekf_avg);

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s obsIndex 0-2 path
    // (innovation ~line 1011-1014; R_OBS "no reported accuracy" branch
    // ~line 741-742) - fuses the 3 GPS velocity axes sequentially against
    // `gps.velocity_ned`, using the CURRENT state for each axis's
    // innovation (matching upstream: obsIndex 1's innovation is computed
    // AFTER obsIndex 0's state correction has already been applied,
    // since correlated off-diagonal terms mean fusing velN can shift the
    // velE/velD state estimate too). Returns the number of axes (0-3)
    // whose healthyFusion guard passed - phase 1/this phase has no
    // faultStatus.bad_nvel/bad_evel/bad_dvel bookkeeping to report this
    // through instead.
    int fuse_gps_velocity(const GpsSample& gps, ftype dt_ekf_avg);

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s obsIndex 3-4 path
    // (innovation ~line 1016; R_OBS "no reported accuracy" branch ~line
    // 750-756). Fuses the 2 GPS horizontal position axes sequentially
    // against `gps.position_ne`. Returns the number of axes (0-2) fused.
    int fuse_gps_position(const GpsSample& gps, ftype dt_ekf_avg);

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s R_OBS[0]/[1]/[2] "no
    // reported accuracy" formula, AP_NavEKF3_PosVelFusion.cpp ~line
    // 740-742 - `sq(constrain_ftype(noise, 0.05f, 5.0f)) +
    // sq(accScale*accNavMag)`, accNavMag = velDotNEDfilt.length()
    // (AP_NavEKF3_core.cpp ~line 772; this port's own vel_dot_ned_filt,
    // populated by update_strapdown_equations_ned()). Public (not a
    // fuse_gps_velocity() implementation detail) so tests can verify the
    // exact formula independently, per the ticket's verification
    // standard.
    [[nodiscard]] ftype gps_horiz_vel_obs_variance() const;
    [[nodiscard]] ftype gps_vert_vel_obs_variance() const;

    // CPP-056 phase 2. upstream: FuseVelPosNED()'s R_OBS[3]/[4] "no
    // reported accuracy" formula, ~line 750-756 -
    // `sq(constrain_ftype(_gpsHorizPosNoise, 0.1f, 10.0f)) +
    // sq(gpsPosVarAccScale*accNavMag)`.
    [[nodiscard]] ftype gps_horiz_pos_obs_variance() const;

private:
    void constrain_states(ftype dt_ekf_avg);   // upstream: NavEKF3_core::ConstrainStates()
    void constrain_variances(ftype dt_ekf_avg); // upstream: NavEKF3_core::ConstrainVariances()

    // CPP-056 phase 2. upstream: NavEKF3_core::updateStateIndexLim(),
    // AP_NavEKF3_Control.cpp ~line 190-208 - the same nested inhibit-flag
    // logic, transcribed directly, reused here to bound
    // fuse_direct_state_observation()'s KHP/healthyFusion/P-update loops
    // exactly as upstream bounds them (not the full 0..23 every call).
    [[nodiscard]] int state_index_lim() const;

    // upstream: NavEKF3_core::ForceSymmetry(), AP_NavEKF3_core.cpp ~line
    // 1862 - averages P[i][j]/P[j][i] for i in 1..lim. Did not exist in
    // phase 1 (nothing needed it before fusion existed).
    void force_symmetry(int lim);

    // CPP-056 phase 2 helper (no direct upstream counterpart - upstream's
    // statesArray is a flat float[24] alias over the same memory as
    // stateStruct, so `statesArray[i] -= Kfusion[i]*innovation` IS its
    // state update; this port's StateVector is a structured type, so the
    // flat-index correction has to be unpacked field-by-field here
    // instead). Applies `state -= kfusion*innovation` across all 24
    // conceptual slots unconditionally, then re-normalizes the
    // quaternion (upstream: `stateStruct.quat.normalize()`,
    // unconditional, right after the state update loop). Safe to apply
    // unconditionally to inhibited slots (16..23 when mag/wind
    // inhibited) because kfusion[i] is exactly 0.0 there by construction
    // (fuse_direct_state_observation() only ever writes a nonzero value
    // into an index it has confirmed is not inhibited) - not a shortcut,
    // provably identical to bounding this loop at state_index_lim() too.
    void apply_state_correction(const std::array<ftype, 24>& kfusion, ftype innovation);
};

} // namespace fwcpp::ekf
