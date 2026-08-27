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

private:
    void constrain_states(ftype dt_ekf_avg);   // upstream: NavEKF3_core::ConstrainStates()
    void constrain_variances(ftype dt_ekf_avg); // upstream: NavEKF3_core::ConstrainVariances()
};

} // namespace fwcpp::ekf
