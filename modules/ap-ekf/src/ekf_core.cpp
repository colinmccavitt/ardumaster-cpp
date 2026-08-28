// Implementation of fwcpp::ekf::EkfCore - see ekf_core.hpp for the full
// scope/exclusions banner. This file holds the two functions with
// upstream-literal-sensitive or algebraically dense bodies:
// update_strapdown_equations_ned() and covariance_prediction() (which
// includes the transcribed CovariancePrediction() Jacobian block and the
// ConstrainVariances()/ConstrainStates() equivalents).
//
// COVARIANCE PREDICTION TRANSCRIPTION NOTE: NavEKF3_core::
// CovariancePrediction() (upstream AP_NavEKF3_core.cpp ~line 1008-1803,
// ~800 lines) is not hand-derived anywhere in ArduPilot - its own upstream
// comment ("intermediate calculations") and the PS0..PS222 naming pattern
// are the signature of a symbolic-math-tool-generated (sympy-style)
// Jacobian expansion for the 10x10 quaternion/velocity/position
// process-noise covariance block, plus its correlation with the
// gyro-bias/accel-bias states. Given that provenance, this port
// transcribes the PS-intermediate values and the resulting nextP[i][j]
// (i,j in 0..9) formulas VERBATIM from the upstream text (variable names
// kept identical on purpose - PS0..PS222, dax/day/daz/dax_b.../dvx_b -
// specifically so this block stays mechanically diffable against
// AP_NavEKF3_core.cpp line-by-line, rather than "cleaned up" into this
// port's usual naming style, which would make an audit far harder for
// code this numerically sensitive). This is the one deliberate exception
// in this port to writing every port "in its own idioms" - verbatim
// transcription IS the safer idiom for auto-generated algebra.
//
// `P` (read side, old covariance) and `nextP` (write side, the local
// scratch upstream itself uses - upstream literally aliases its own KHP
// scratch buffer as `nextP`, see its own comment "save memory by using
// KHP as nextP") are named to match upstream exactly for the same
// diffability reason: `P` here is simply this object's member (read
// through unqualified, same as upstream's own member access), and
// `nextP` is a fresh local Matrix24 - no macro tricks, just upstream's
// own two-buffer naming reproduced directly in C++.
//
// One genuine simplification vs. the unrolled upstream source: upstream
// unrolls the row-0..9-vs-column-10..23 correlation update as separate
// blocks per column because its codegen tool doesn't parametrize over
// column index. This port instead runs a small loop over columns 10..15
// (verified algebraically, using P's symmetry, to be IDENTICAL to each
// of upstream's unrolled column-10 through column-15 blocks - not a new
// formula).
//
// CPP-065 UPDATE (phase 11): columns 16..23 (mag/wind) are NO LONGER
// skipped - they are now transcribed VERBATIM (not via the column-10..15
// loop's generalized-and-verified approach) directly below that loop,
// gated by state_index_lim() exactly as upstream gates them by
// stateIndexLim, since covariance_prediction() and constrain_variances()
// are now wired to the real runtime inhibit_mag_states/inhibit_wind_states
// flags (see this ticket's own banner in covariance_prediction() and in
// constrain_variances()) rather than treating those states as permanently
// inhibited. At this port's real default settings (both flags true),
// state_index_lim() stays 15 and none of that new code executes - so the
// prior phases' "permanently inhibited" behavior is exactly preserved by
// default.
//
// LITERAL PRECISION NOTE: this port writes every literal in the
// transcribed formulas as an explicit `ftype(...)` (e.g. `ftype(0.25)`),
// rather than reproducing upstream's own mix of F-suffixed
// (always-float) and bare (build-flag-dependent) literals. For every
// literal that is exactly representable in IEEE-754 float (0.25, 0.5,
// 1.0, 2.0, 5.0, 100.0, ...) this is bit-identical to upstream regardless
// of ftype - no divergence. The one place this could matter is
// safety-clamp threshold constants that are NOT exactly float-representable
// (e.g. 0.175 in ConstrainVariances' gyro-bias-variance ceiling) - under
// upstream's own project-wide `-fsingle-precision-constant` flag those
// bare literals are typed float even in a double-EKF build, then widened;
// this port's `ftype(0.175)` instead uses full double precision when
// FWCPP_EKF_DOUBLE is on. The difference is a coarse safety bound moving
// by roughly 1e-9 relative - not a physics formula - and is a deliberate,
// documented precision choice (favoring accuracy over bug-for-bug
// reproduction of a build-flag artifact), not an oversight.

#include <fwcpp/ekf/ekf_core.hpp>

#include <algorithm>

namespace fwcpp::ekf {

namespace {

#if FWCPP_EKF_DOUBLE
constexpr ftype kPosXyStateLimit = static_cast<ftype>(50.0e6);  // EK3_POSXY_STATE_LIMIT, HAL_WITH_EKF_DOUBLE branch
#else
constexpr ftype kPosXyStateLimit = static_cast<ftype>(1.0e6);   // EK3_POSXY_STATE_LIMIT, float branch
#endif

constexpr ftype kGravityMss = static_cast<ftype>(9.80665f);        // AP_Math/definitions.h GRAVITY_MSS
constexpr ftype kAccelBiasLimScaler = static_cast<ftype>(0.2f);    // AP_NavEKF3_core.h ACCEL_BIAS_LIM_SCALER
constexpr ftype kGyroBiasLimit = static_cast<ftype>(0.5f);         // AP_NavEKF3_core.h GYRO_BIAS_LIMIT
constexpr ftype kBadImuDataAccPNse = static_cast<ftype>(5.0f);     // AP_NavEKF3_core.h BAD_IMU_DATA_ACC_P_NSE
constexpr ftype kVelStateMinVariance = static_cast<ftype>(1e-4);   // AP_NavEKF3_core.h VEL_STATE_MIN_VARIANCE
constexpr ftype kPosStateMinVariance = static_cast<ftype>(1e-4);   // AP_NavEKF3_core.h POS_STATE_MIN_VARIANCE
constexpr ftype kMinSafeStateVar = static_cast<ftype>(5e-9);       // ConstrainVariances() local minSafeStateVar
constexpr std::uint32_t kEkfTargetRateHz = 83;                     // uint32_t(1.0/EKF_TARGET_DT), EKF_TARGET_DT=0.012
constexpr std::uint32_t kVertVelVarClipCountLim = 5 * kEkfTargetRateHz; // VERT_VEL_VAR_CLIP_COUNT_LIM

// CPP-056 phase 2. upstream: AP_NavEKF3.h ~line 486-488 - `const float`
// members of NavEKF3 (NOT AP_Param-tunable, unlike gps_horiz_vel_noise
// etc. above), read directly from the pinned Plane-4.7.0 source.
constexpr ftype kGpsNeVelVarAccScale = static_cast<ftype>(0.05f);  // AP_NavEKF3.h gpsNEVelVarAccScale
constexpr ftype kGpsDVelVarAccScale = static_cast<ftype>(0.07f);   // AP_NavEKF3.h gpsDVelVarAccScale
constexpr ftype kGpsPosVarAccScale = static_cast<ftype>(0.05f);    // AP_NavEKF3.h gpsPosVarAccScale

// CPP-058 phase 4. upstream: AP_NavEKF3.h:493 - `const uint16_t
// posRetryTimeUseVel_ms = 10000;`, a real, hardcoded upstream constant
// (NOT an AP_Param), verified directly. Same "not user-tunable"
// treatment as kGpsNeVelVarAccScale etc. above - a file-local constant,
// not a public field. See ekf_core.hpp's "CPP-058, PHASE 4" banner for
// why this single threshold is applied to both position and velocity
// timeout in this port.
constexpr ftype kGpsFusionTimeoutS = static_cast<ftype>(10.0);  // posRetryTimeUseVel_ms

// CPP-062 phase 8. upstream: AP_NavEKF3.h:495 - `const uint16_t
// hgtRetryTimeMode0_ms = 10000;`, verified directly to be a TEXTUALLY
// SEPARATE upstream constant from posRetryTimeUseVel_ms above (AP_NavEKF3.h:
// 493) - both real, hardcoded (NOT AP_Param) constants that simply happen to
// share the same 10000ms value today. See ekf_core.hpp's "CPP-062, PHASE 8"
// banner "A REAL CONSTANT-IDENTITY CHECK" for the full verification
// (including confirming, at the real height-timeout-selection call site,
// that Mode0 - "with vertical velocity measurement" - is genuinely the
// applicable branch for this port's always-has-GPS-velocity-aiding
// assumption). Deliberately NOT a reuse of kGpsFusionTimeoutS.
constexpr ftype kBaroFusionTimeoutS = static_cast<ftype>(10.0);  // hgtRetryTimeMode0_ms

// CPP-059 phase 5. upstream: AP_NavEKF3.h:500 - `const float
// magVarRateScale = 0.005f;`, a real, hardcoded upstream constant (NOT an
// AP_Param), verified directly. Same "not user-tunable" treatment as
// kGpsFusionTimeoutS etc. above - a file-local constant, not a public
// field.
constexpr ftype kMagVarRateScale = static_cast<ftype>(0.005f);  // magVarRateScale

// CPP-065 phase 11. upstream: AP_NavEKF3_core.h:109 - #define
// WIND_VEL_VARIANCE_MAX 400.0f, verified directly. Used by
// ConstrainVariances()'s real per-state-group wind-variance clamp (see
// constrain_variances() below).
constexpr ftype kWindVelVarianceMax = static_cast<ftype>(400.0);  // WIND_VEL_VARIANCE_MAX

[[nodiscard]] ftype clamp(ftype v, ftype lo, ftype hi) {
    return fwcpp::math::constrain_value(v, lo, hi);
}

void zero_row_col(Matrix24& m, int idx) {
    for (int k = 0; k < 24; ++k) {
        m[static_cast<std::size_t>(idx)][static_cast<std::size_t>(k)] = 0;
        m[static_cast<std::size_t>(k)][static_cast<std::size_t>(idx)] = 0;
    }
}

void zero_rows_cols(Matrix24& m, int first, int last) {
    for (int i = first; i <= last; ++i) {
        zero_row_col(m, i);
    }
}

} // namespace

// upstream: NavEKF3_core::UpdateStrapdownEquationsNED(), AP_NavEKF3_core.cpp
// ~line 743, PLUS the ConstrainStates() call at its end. See ekf_core.hpp
// banner simplification 4 for the bias-correction divergence and
// simplification 6/7 for dt_ekf_avg/earth_rate_ned.
void EkfCore::update_strapdown_equations_ned(const GyroSample& gyro, const AccelSample& accel, ftype dt_ekf_avg) {
    // upstream: correctDeltaAngle()/correctDeltaVelocity(), AP_NavEKF3_core.cpp
    // ~line 726-732 - `delAng -= inactiveBias[gyro_index].gyro_bias *
    // (delAngDT / dtEkfAvg)`. Single-IMU equivalent: state.gyro_bias IS the
    // active IMU's bias (see hpp banner simplification 4).
    const Vector3F del_ang_corrected =
        gyro.delta_angle - state.gyro_bias * (gyro.delta_angle_dt / dt_ekf_avg);
    const Vector3F del_vel_corrected =
        accel.delta_velocity - state.accel_bias * (accel.delta_velocity_dt / dt_ekf_avg);

    // Quaternion update from delta-angle, earth-rotation-rate compensated,
    // via the axis-angle rotation upstream calls `quat.rotate(v)` -
    // upstream: `stateStruct.quat.rotate(delAngCorrected - prevTnb *
    // earthRateNED * imuDataDelayed.delAngDT)`. rotate()/from_axis_angle()
    // are ported here as an inline local computation (not added to
    // ap-math/quaternion.hpp - CPP-009's slice explicitly excluded
    // from_axis_angle/rotate(Vector3), see that file's banner), matching
    // upstream's real formula (AP_Math/quaternion.cpp ~line 454-491)
    // exactly: `rotate(v)` builds `r` via `from_axis_angle(v)` then does
    // `(*this) *= r`.
    const Vector3F rotation_vec = del_ang_corrected - prev_tnb * earth_rate_ned * gyro.delta_angle_dt;
    QuaternionF delta_quat;
    {
        const ftype theta = rotation_vec.length();
        if (fwcpp::math::is_zero(theta)) {
            delta_quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
        } else {
            const Vector3F axis = rotation_vec / theta;
            const ftype st2 = std::sin(ftype(0.5) * theta);
            delta_quat = QuaternionF(std::cos(ftype(0.5) * theta), axis.x * st2, axis.y * st2, axis.z * st2);
        }
    }
    state.quat *= delta_quat;
    state.quat.normalize();

    // Body delta-velocity -> nav frame using the PREVIOUS step's Tnb
    // (upstream's own comment: "use the nav frame from previous time step
    // as the delta velocities have been rotated into that frame"), then
    // gravity-compensate.
    Vector3F del_vel_nav = prev_tnb.mul_transpose(del_vel_corrected);
    del_vel_nav.z += kGravityMss * accel.delta_velocity_dt;

    // Recompute prevTnb from the NEW quaternion, for use next call.
    state.quat.inverse().rotation_matrix(prev_tnb);

    vel_dot_ned = del_vel_nav / accel.delta_velocity_dt;
    vel_dot_ned_filt = vel_dot_ned * ftype(0.05) + vel_dot_ned_filt * ftype(0.95);

    // upstream: "if we are not aiding, then limit the horizontal magnitude
    // of acceleration to prevent large manoeuvre transients disturbing the
    // attitude" - real strapdown-adjacent behavior (not a fusion feature),
    // and always active in this phase since `aiding` is permanently false
    // (see hpp banner).
    const ftype acc_nav_mag_horiz = vel_dot_ned_filt.xy().length();
    if (!aiding && acc_nav_mag_horiz > ftype(5.0)) {
        const ftype gain = ftype(5.0) / acc_nav_mag_horiz;
        del_vel_nav.x *= gain;
        del_vel_nav.y *= gain;
    }

    const Vector3F last_velocity = state.velocity;
    state.velocity += del_vel_nav;
    // Trapezoidal integration for position.
    state.position += (state.velocity + last_velocity) * (accel.delta_velocity_dt * ftype(0.5));

    constrain_states(dt_ekf_avg);
}

// upstream: NavEKF3_core::ConstrainStates(), AP_NavEKF3_core.cpp ~line
// 2016. See hpp banner simplifications 8/9 for the two excluded branches.
void EkfCore::constrain_states(ftype dt_ekf_avg) {
    for (std::uint8_t i = 0; i < 4; ++i) {
        state.quat[i] = clamp(state.quat[i], ftype(-1.0), ftype(1.0));
    }
    state.velocity.x = clamp(state.velocity.x, ftype(-5.0e2), ftype(5.0e2));
    state.velocity.y = clamp(state.velocity.y, ftype(-5.0e2), ftype(5.0e2));
    state.velocity.z = clamp(state.velocity.z, ftype(-5.0e2), ftype(5.0e2));
    state.position.x = clamp(state.position.x, -kPosXyStateLimit, kPosXyStateLimit);
    state.position.y = clamp(state.position.y, -kPosXyStateLimit, kPosXyStateLimit);
    state.position.z = clamp(state.position.z, ftype(-4.0e4), ftype(1.0e4));
    const ftype gyro_bias_lim = kGyroBiasLimit * dt_ekf_avg;
    state.gyro_bias.x = clamp(state.gyro_bias.x, -gyro_bias_lim, gyro_bias_lim);
    state.gyro_bias.y = clamp(state.gyro_bias.y, -gyro_bias_lim, gyro_bias_lim);
    state.gyro_bias.z = clamp(state.gyro_bias.z, -gyro_bias_lim, gyro_bias_lim);
    const ftype accel_bias_lim = acc_bias_lim * dt_ekf_avg;
    state.accel_bias.x = clamp(state.accel_bias.x, -accel_bias_lim, accel_bias_lim);
    state.accel_bias.y = clamp(state.accel_bias.y, -accel_bias_lim, accel_bias_lim);
    state.accel_bias.z = clamp(state.accel_bias.z, -accel_bias_lim, accel_bias_lim);
    // Simplified branch only - see hpp banner simplification 8.
    state.earth_magfield.x = clamp(state.earth_magfield.x, ftype(-1.0), ftype(1.0));
    state.earth_magfield.y = clamp(state.earth_magfield.y, ftype(-1.0), ftype(1.0));
    state.earth_magfield.z = clamp(state.earth_magfield.z, ftype(-1.0), ftype(1.0));
    state.body_magfield.x = clamp(state.body_magfield.x, ftype(-0.5), ftype(0.5));
    state.body_magfield.y = clamp(state.body_magfield.y, ftype(-0.5), ftype(0.5));
    state.body_magfield.z = clamp(state.body_magfield.z, ftype(-0.5), ftype(0.5));
    state.wind_vel.x = clamp(state.wind_vel.x, ftype(-100.0), ftype(100.0));
    state.wind_vel.y = clamp(state.wind_vel.y, ftype(-100.0), ftype(100.0));
    // Terrain state clamp excluded - see hpp banner simplification 9.
}

void EkfCore::covariance_init(ftype dt_ekf_avg) {
    P = {};

    // upstream: rot_vec_var.x = rot_vec_var.y = rot_vec_var.z = sq(0.1f) -
    // initial angle uncertainty, rad^2, rotated into body frame by the
    // reset path below via CovariancePrediction(&rot_vec_var).
    const Vector3F rot_vec_var(sq(ftype(0.1)), sq(ftype(0.1)), sq(ftype(0.1)));
    const GyroSample zero_gyro{};
    const AccelSample zero_accel{};
    covariance_prediction(zero_gyro, zero_accel, dt_ekf_avg, &rot_vec_var);

    // velocities
    P[4][4] = sq(gps_horiz_vel_noise);
    P[5][5] = P[4][4];
    P[6][6] = sq(gps_vert_vel_noise);
    // positions
    P[7][7] = sq(gps_horiz_pos_noise);
    P[8][8] = P[7][7];
    P[9][9] = sq(baro_alt_noise);
    // gyro delta angle biases
    P[10][10] = sq(fwcpp::math::radians(initial_gyro_bias_uncertainty_deg_s * dt_ekf_avg));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    // delta velocity biases
    P[13][13] = sq(kAccelBiasLimScaler * acc_bias_lim * dt_ekf_avg);
    P[14][14] = P[13][13];
    P[15][15] = P[13][13];
    // earth magnetic field - real upstream init value; permanently zeroed
    // again by the very next covariance_prediction() call in this phase
    // (mag states inhibited - see hpp banner simplification 1).
    P[16][16] = sq(mag_noise);
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    // body magnetic field
    P[19][19] = sq(mag_noise);
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
    // wind velocities
    P[22][22] = 0;
    P[23][23] = 0;
}

// upstream: NavEKF3_core::CovariancePrediction(Vector3F* rotVarVecPtr),
// AP_NavEKF3_core.cpp ~line 1008-1803. See this file's own banner for the
// transcription approach, and ekf_core.hpp's banner for the
// inhibitMagStates/inhibitWindStates/inhibitDelAngBiasStates/
// inhibitDelVelBiasStates/badIMUdata simplifications baked in below.
void EkfCore::covariance_prediction(const GyroSample& gyro, const AccelSample& accel, ftype dt_ekf_avg,
                                     const Vector3F* rot_var_vec) {
    const ftype dt = clamp(ftype(0.5) * (gyro.delta_angle_dt + accel.delta_velocity_dt), ftype(0.5) * dt_ekf_avg,
                            ftype(2.0) * dt_ekf_avg);

    // CPP-065 phase 11. upstream: AP_NavEKF3_core.cpp ~line 1038-1040 -
    // "use filtered height rate to increase wind process noise when
    // climbing or descending. Filter height rate using a 10 second time
    // constant filter": `alpha = 0.1f*dt; hgtRate = hgtRate*(1.0f-alpha) -
    // stateStruct.velocity.z*alpha`. Runs unconditionally, on every call
    // (including the covariance_init() reset call, matching upstream's own
    // CovarianceInit() -> CovariancePrediction(&rotVarVec) call, verified
    // directly at AP_NavEKF3_core.cpp ~line 582 - the quatCovResetOnly
    // special-casing below only narrows the daxVar/dayVar/dazVar branch,
    // not this filter). See ekf_core.hpp's hgt_rate member comment for why
    // this is a real persistent filtered quantity, not a raw
    // state.velocity.z alias.
    {
        const ftype alpha = ftype(0.1) * dt;
        hgt_rate = hgt_rate * (ftype(1) - alpha) - state.velocity.z * alpha;
    }

    // processNoiseVariance[0..5] map to state indices 10..15 (gyro bias,
    // accel bias) - always active in this phase (simplification 2).
    // Indices 6..13 (mag/wind, state 16..23) - CPP-065 phase 11: now
    // populated (magEarthVar/magBodyVar/windVelVar) whenever the
    // corresponding runtime inhibit flag is clear, matching upstream's
    // real AP_NavEKF3_core.cpp ~line 1087-1115 exactly (see the two `if`
    // blocks below). At this port's real defaults (inhibit_mag_states=
    // true, inhibit_wind_states=true) both stay exactly 0, same as before
    // this ticket - a behavior-preserving change at default settings.
    std::array<ftype, 14> process_noise_variance{};
    {
        const ftype d_ang_bias_var = sq(sq(dt) * clamp(gyro_bias_process_noise, ftype(0), ftype(1)));
        for (int i = 0; i <= 2; ++i) process_noise_variance[static_cast<std::size_t>(i)] = d_ang_bias_var;
    }
    {
        const ftype d_vel_bias_var = sq(sq(dt) * clamp(accel_bias_process_noise, ftype(0), ftype(1)));
        for (int i = 3; i <= 5; ++i) process_noise_variance[static_cast<std::size_t>(i)] = d_vel_bias_var;
    }

    // CPP-065 phase 11. upstream: AP_NavEKF3_core.cpp ~line 1087-1092:
    // `if (!inhibitMagStates) { magEarthVar = sq(dt*constrain_ftype(
    // _magEarthProcessNoise,0,1)); magBodyVar = sq(dt*constrain_ftype(
    // _magBodyProcessNoise,0,1)); for(i=6;i<=8;i++) processNoiseVariance[i]
    // = magEarthVar; for(i=9;i<=11;i++) processNoiseVariance[i] =
    // magBodyVar; }`. Verified directly, transcribed exactly.
    //
    // EXCLUDED (real upstream mechanisms immediately preceding this block
    // in the real source, deliberately not ported - matching this port's
    // established exclusion pattern for the same machinery elsewhere):
    //   - `lastInhibitMagStates`-edge-triggered `needMagBodyVarReset`/
    //     `needEarthBodyVarReset` and the `zeroCols`/`zeroRows`/
    //     `FuseDeclination(radians(20.0f))` reset they trigger - ties to
    //     yaw-realignment machinery, already excluded since phases 5/6
    //     (ekf_core.hpp's "CPP-059, PHASE 5" banner).
    if (!inhibit_mag_states) {
        const ftype mag_earth_var = sq(dt * clamp(mag_earth_process_noise, ftype(0), ftype(1)));
        const ftype mag_body_var = sq(dt * clamp(mag_body_process_noise, ftype(0), ftype(1)));
        for (int i = 6; i <= 8; ++i) process_noise_variance[static_cast<std::size_t>(i)] = mag_earth_var;
        for (int i = 9; i <= 11; ++i) process_noise_variance[static_cast<std::size_t>(i)] = mag_body_var;
    }

    // CPP-065 phase 11. upstream: AP_NavEKF3_core.cpp ~line 1094-1116:
    // `if (!inhibitWindStates) { ... if (newTreatWindStatesAsTruth) {...}
    // else { ... windVelVar = sq(dt*constrain_ftype(_windVelProcessNoise,
    // 0,1)*(1+constrain_ftype(_wndVarHgtRateScale,0,1)*fabsF(hgtRate)));
    // if (!tasDataDelayed.allowFusion) { windVelVar *= 10.0f; } for(i=12;
    // i<=13;i++) processNoiseVariance[i] = windVelVar; } }`. Verified
    // directly, transcribed exactly for the real, active branch.
    //
    // EXCLUDED (real upstream mechanisms, deliberately not ported):
    //   - `treatWindStatesAsTruth`/`isDragFusionDeadReckoning`/
    //     `windStateIsObservable` - already-established exclusion (phase
    //     2, reconfirmed phase 9/10) - no such fields exist in this port
    //     (no optical-flow/const-position-hold subsystem that would ever
    //     set them); this port always takes the real "else" (normal)
    //     branch below.
    //   - `tasDataDelayed.allowFusion`-gated 10x wind-noise scaling for a
    //     failed airspeed sensor - no airspeed-sensor-health state
    //     modeled (already-established phase-9 exclusion pattern).
    if (!inhibit_wind_states) {
        const ftype wind_vel_var =
            sq(dt * clamp(wind_vel_process_noise, ftype(0), ftype(1)) *
               (ftype(1) + clamp(wind_var_hgt_rate_scale, ftype(0), ftype(1)) * std::abs(hgt_rate)));
        for (int i = 12; i <= 13; ++i) process_noise_variance[static_cast<std::size_t>(i)] = wind_vel_var;
    }

    const ftype dvx = accel.delta_velocity.x;
    const ftype dvy = accel.delta_velocity.y;
    const ftype dvz = accel.delta_velocity.z;
    const ftype dax = gyro.delta_angle.x;
    const ftype day = gyro.delta_angle.y;
    const ftype daz = gyro.delta_angle.z;
    const ftype q0 = state.quat[0];
    const ftype q1 = state.quat[1];
    const ftype q2 = state.quat[2];
    const ftype q3 = state.quat[3];
    const ftype dax_b = state.gyro_bias.x;
    const ftype day_b = state.gyro_bias.y;
    const ftype daz_b = state.gyro_bias.z;
    const ftype dvx_b = state.accel_bias.x;
    const ftype dvy_b = state.accel_bias.y;
    const ftype dvz_b = state.accel_bias.z;

    ftype daxVar, dayVar, dazVar;
    const bool quatCovResetOnly = (rot_var_vec != nullptr);
    if (quatCovResetOnly) {
        const Matrix3F R_ef(Vector3F(rot_var_vec->x, ftype(0), ftype(0)), Vector3F(ftype(0), rot_var_vec->y, ftype(0)),
                             Vector3F(ftype(0), ftype(0), rot_var_vec->z));
        Matrix3F Tnb;
        state.quat.inverse().rotation_matrix(Tnb);
        const Matrix3F R_bf = Tnb * R_ef * Tnb.transposed();
        daxVar = R_bf.a.x;
        dayVar = R_bf.b.y;
        dazVar = R_bf.c.z;
        zero_rows_cols(P, 0, 3);
    } else {
        const ftype _gyrNoise = clamp(gyr_noise, ftype(0), ftype(1));
        daxVar = dayVar = dazVar = sq(dt * _gyrNoise);
    }
    // badIMUdata always false in this phase - simplification 3.
    const ftype _accNoise = clamp(acc_noise, ftype(0), kBadImuDataAccPNse);
    const ftype dvxVar = sq(dt * _accNoise);
    const ftype dvyVar = dvxVar;
    const ftype dvzVar = dvxVar;

    // `P` below (unqualified) is this object's own member - the read
    // side, holding the covariance as of the START of this call. `nextP`
    // is a fresh local scratch - the write side - matching upstream's own
    // two-buffer naming (see file banner).
    Matrix24 nextP{};

    // ---- BEGIN verbatim transcription of upstream's "intermediate
    // calculations" block (AP_NavEKF3_core.cpp ~line 1188-1405) ----
    const ftype PS0 = sq(q1);
    const ftype PS1 = ftype(0.25) * daxVar;
    const ftype PS2 = sq(q2);
    const ftype PS3 = ftype(0.25) * dayVar;
    const ftype PS4 = sq(q3);
    const ftype PS5 = ftype(0.25) * dazVar;
    const ftype PS6 = ftype(0.5) * q1;
    const ftype PS7 = ftype(0.5) * q2;
    const ftype PS8 = PS7 * P[10][11];
    const ftype PS9 = ftype(0.5) * q3;
    const ftype PS10 = PS9 * P[10][12];
    const ftype PS11 = ftype(0.5) * dax - ftype(0.5) * dax_b;
    const ftype PS12 = ftype(0.5) * day - ftype(0.5) * day_b;
    const ftype PS13 = ftype(0.5) * daz - ftype(0.5) * daz_b;
    const ftype PS14 = PS10 - PS11 * P[1][10] - PS12 * P[2][10] - PS13 * P[3][10] + PS6 * P[10][10] + PS8 + P[0][10];
    const ftype PS15 = PS6 * P[10][11];
    const ftype PS16 = PS9 * P[11][12];
    const ftype PS17 = -PS11 * P[1][11] - PS12 * P[2][11] - PS13 * P[3][11] + PS15 + PS16 + PS7 * P[11][11] + P[0][11];
    const ftype PS18 = PS6 * P[10][12];
    const ftype PS19 = PS7 * P[11][12];
    const ftype PS20 = -PS11 * P[1][12] - PS12 * P[2][12] - PS13 * P[3][12] + PS18 + PS19 + PS9 * P[12][12] + P[0][12];
    const ftype PS21 = PS12 * P[1][2];
    const ftype PS22 = -PS13 * P[1][3];
    const ftype PS23 = -PS11 * P[1][1] - PS21 + PS22 + PS6 * P[1][10] + PS7 * P[1][11] + PS9 * P[1][12] + P[0][1];
    const ftype PS24 = -PS11 * P[1][2];
    const ftype PS25 = PS13 * P[2][3];
    const ftype PS26 = -PS12 * P[2][2] + PS24 - PS25 + PS6 * P[2][10] + PS7 * P[2][11] + PS9 * P[2][12] + P[0][2];
    const ftype PS27 = PS11 * P[1][3];
    const ftype PS28 = -PS12 * P[2][3];
    const ftype PS29 = -PS13 * P[3][3] - PS27 + PS28 + PS6 * P[3][10] + PS7 * P[3][11] + PS9 * P[3][12] + P[0][3];
    const ftype PS30 = PS11 * P[0][1];
    const ftype PS31 = PS12 * P[0][2];
    const ftype PS32 = PS13 * P[0][3];
    const ftype PS33 = -PS30 - PS31 - PS32 + PS6 * P[0][10] + PS7 * P[0][11] + PS9 * P[0][12] + P[0][0];
    const ftype PS34 = ftype(0.5) * q0;
    const ftype PS35 = q2 * q3;
    const ftype PS36 = q0 * q1;
    const ftype PS37 = q1 * q3;
    const ftype PS38 = q0 * q2;
    const ftype PS39 = q1 * q2;
    const ftype PS40 = q0 * q3;
    const ftype PS41 = 2 * PS2;
    const ftype PS42 = 2 * PS4 - 1;
    const ftype PS43 = PS41 + PS42;
    const ftype PS44 = -PS11 * P[1][13] - PS12 * P[2][13] - PS13 * P[3][13] + PS6 * P[10][13] + PS7 * P[11][13] + PS9 * P[12][13] + P[0][13];
    const ftype PS45 = PS37 + PS38;
    const ftype PS46 = -PS11 * P[1][15] - PS12 * P[2][15] - PS13 * P[3][15] + PS6 * P[10][15] + PS7 * P[11][15] + PS9 * P[12][15] + P[0][15];
    const ftype PS47 = 2 * PS46;
    const ftype PS48 = dvy - dvy_b;
    const ftype PS49 = PS48 * q0;
    const ftype PS50 = dvz - dvz_b;
    const ftype PS51 = PS50 * q1;
    const ftype PS52 = dvx - dvx_b;
    const ftype PS53 = PS52 * q3;
    const ftype PS54 = PS49 - PS51 + 2 * PS53;
    const ftype PS55 = 2 * PS29;
    const ftype PS56 = -PS39 + PS40;
    const ftype PS57 = -PS11 * P[1][14] - PS12 * P[2][14] - PS13 * P[3][14] + PS6 * P[10][14] + PS7 * P[11][14] + PS9 * P[12][14] + P[0][14];
    const ftype PS58 = 2 * PS57;
    const ftype PS59 = PS48 * q2;
    const ftype PS60 = PS50 * q3;
    const ftype PS61 = PS59 + PS60;
    const ftype PS62 = 2 * PS23;
    const ftype PS63 = PS50 * q2;
    const ftype PS64 = PS48 * q3;
    const ftype PS65 = -PS64;
    const ftype PS66 = PS63 + PS65;
    const ftype PS67 = 2 * PS33;
    const ftype PS68 = PS50 * q0;
    const ftype PS69 = PS48 * q1;
    const ftype PS70 = PS52 * q2;
    const ftype PS71 = PS68 + PS69 - 2 * PS70;
    const ftype PS72 = 2 * PS26;
    const ftype PS73 = -PS11 * P[1][4] - PS12 * P[2][4] - PS13 * P[3][4] + PS6 * P[4][10] + PS7 * P[4][11] + PS9 * P[4][12] + P[0][4];
    const ftype PS74 = 2 * PS0;
    const ftype PS75 = PS42 + PS74;
    const ftype PS76 = PS39 + PS40;
    const ftype PS77 = 2 * PS44;
    const ftype PS78 = PS51 - PS53;
    const ftype PS79 = -PS70;
    const ftype PS80 = PS68 + 2 * PS69 + PS79;
    const ftype PS81 = -PS35 + PS36;
    const ftype PS82 = PS52 * q1;
    const ftype PS83 = PS60 + PS82;
    const ftype PS84 = PS52 * q0;
    const ftype PS85 = PS63 - 2 * PS64 + PS84;
    const ftype PS86 = -PS11 * P[1][5] - PS12 * P[2][5] - PS13 * P[3][5] + PS6 * P[5][10] + PS7 * P[5][11] + PS9 * P[5][12] + P[0][5];
    const ftype PS87 = PS41 + PS74 - 1;
    const ftype PS88 = PS35 + PS36;
    const ftype PS89 = 2 * PS63 + PS65 + PS84;
    const ftype PS90 = -PS37 + PS38;
    const ftype PS91 = PS59 + PS82;
    const ftype PS92 = PS69 + PS79;
    const ftype PS93 = PS49 - 2 * PS51 + PS53;
    const ftype PS94 = -PS11 * P[1][6] - PS12 * P[2][6] - PS13 * P[3][6] + PS6 * P[6][10] + PS7 * P[6][11] + PS9 * P[6][12] + P[0][6];
    const ftype PS95 = sq(q0);
    const ftype PS96 = -PS34 * P[10][11];
    const ftype PS97 = PS11 * P[0][11] - PS12 * P[3][11] + PS13 * P[2][11] - PS19 + PS9 * P[11][11] + PS96 + P[1][11];
    const ftype PS98 = PS13 * P[0][2];
    const ftype PS99 = PS12 * P[0][3];
    const ftype PS100 = PS11 * P[0][0] - PS34 * P[0][10] - PS7 * P[0][12] + PS9 * P[0][11] + PS98 - PS99 + P[0][1];
    const ftype PS101 = PS11 * P[0][2];
    const ftype PS102 = PS101 + PS13 * P[2][2] + PS28 - PS34 * P[2][10] - PS7 * P[2][12] + PS9 * P[2][11] + P[1][2];
    const ftype PS103 = PS9 * P[10][11];
    const ftype PS104 = PS7 * P[10][12];
    const ftype PS105 = PS103 - PS104 + PS11 * P[0][10] - PS12 * P[3][10] + PS13 * P[2][10] - PS34 * P[10][10] + P[1][10];
    const ftype PS106 = -PS34 * P[10][12];
    const ftype PS107 = PS106 + PS11 * P[0][12] - PS12 * P[3][12] + PS13 * P[2][12] + PS16 - PS7 * P[12][12] + P[1][12];
    const ftype PS108 = PS11 * P[0][3];
    const ftype PS109 = PS108 - PS12 * P[3][3] + PS25 - PS34 * P[3][10] - PS7 * P[3][12] + PS9 * P[3][11] + P[1][3];
    const ftype PS110 = PS13 * P[1][2];
    const ftype PS111 = PS12 * P[1][3];
    const ftype PS112 = PS110 - PS111 + PS30 - PS34 * P[1][10] - PS7 * P[1][12] + PS9 * P[1][11] + P[1][1];
    const ftype PS113 = PS11 * P[0][13] - PS12 * P[3][13] + PS13 * P[2][13] - PS34 * P[10][13] - PS7 * P[12][13] + PS9 * P[11][13] + P[1][13];
    const ftype PS114 = PS11 * P[0][15] - PS12 * P[3][15] + PS13 * P[2][15] - PS34 * P[10][15] - PS7 * P[12][15] + PS9 * P[11][15] + P[1][15];
    const ftype PS115 = 2 * PS114;
    const ftype PS116 = 2 * PS109;
    const ftype PS117 = PS11 * P[0][14] - PS12 * P[3][14] + PS13 * P[2][14] - PS34 * P[10][14] - PS7 * P[12][14] + PS9 * P[11][14] + P[1][14];
    const ftype PS118 = 2 * PS117;
    const ftype PS119 = 2 * PS112;
    const ftype PS120 = 2 * PS100;
    const ftype PS121 = 2 * PS102;
    const ftype PS122 = PS11 * P[0][4] - PS12 * P[3][4] + PS13 * P[2][4] - PS34 * P[4][10] - PS7 * P[4][12] + PS9 * P[4][11] + P[1][4];
    const ftype PS123 = 2 * PS113;
    const ftype PS124 = PS11 * P[0][5] - PS12 * P[3][5] + PS13 * P[2][5] - PS34 * P[5][10] - PS7 * P[5][12] + PS9 * P[5][11] + P[1][5];
    const ftype PS125 = PS11 * P[0][6] - PS12 * P[3][6] + PS13 * P[2][6] - PS34 * P[6][10] - PS7 * P[6][12] + PS9 * P[6][11] + P[1][6];
    const ftype PS126 = -PS34 * P[11][12];
    const ftype PS127 = -PS10 + PS11 * P[3][12] + PS12 * P[0][12] + PS126 - PS13 * P[1][12] + PS6 * P[12][12] + P[2][12];
    const ftype PS128 = PS11 * P[3][3] + PS22 - PS34 * P[3][11] + PS6 * P[3][12] - PS9 * P[3][10] + PS99 + P[2][3];
    const ftype PS129 = PS13 * P[0][1];
    const ftype PS130 = PS108 + PS12 * P[0][0] - PS129 - PS34 * P[0][11] + PS6 * P[0][12] - PS9 * P[0][10] + P[0][2];
    const ftype PS131 = PS6 * P[11][12];
    const ftype PS132 = -PS103 + PS11 * P[3][11] + PS12 * P[0][11] - PS13 * P[1][11] + PS131 - PS34 * P[11][11] + P[2][11];
    const ftype PS133 = PS11 * P[3][10] + PS12 * P[0][10] - PS13 * P[1][10] + PS18 - PS9 * P[10][10] + PS96 + P[2][10];
    const ftype PS134 = PS12 * P[0][1];
    const ftype PS135 = -PS13 * P[1][1] + PS134 + PS27 - PS34 * P[1][11] + PS6 * P[1][12] - PS9 * P[1][10] + P[1][2];
    const ftype PS136 = PS11 * P[2][3];
    const ftype PS137 = -PS110 + PS136 + PS31 - PS34 * P[2][11] + PS6 * P[2][12] - PS9 * P[2][10] + P[2][2];
    const ftype PS138 = PS11 * P[3][13] + PS12 * P[0][13] - PS13 * P[1][13] - PS34 * P[11][13] + PS6 * P[12][13] - PS9 * P[10][13] + P[2][13];
    const ftype PS139 = PS11 * P[3][15] + PS12 * P[0][15] - PS13 * P[1][15] - PS34 * P[11][15] + PS6 * P[12][15] - PS9 * P[10][15] + P[2][15];
    const ftype PS140 = 2 * PS139;
    const ftype PS141 = 2 * PS128;
    const ftype PS142 = PS11 * P[3][14] + PS12 * P[0][14] - PS13 * P[1][14] - PS34 * P[11][14] + PS6 * P[12][14] - PS9 * P[10][14] + P[2][14];
    const ftype PS143 = 2 * PS142;
    const ftype PS144 = 2 * PS135;
    const ftype PS145 = 2 * PS130;
    const ftype PS146 = 2 * PS137;
    const ftype PS147 = PS11 * P[3][4] + PS12 * P[0][4] - PS13 * P[1][4] - PS34 * P[4][11] + PS6 * P[4][12] - PS9 * P[4][10] + P[2][4];
    const ftype PS148 = 2 * PS138;
    const ftype PS149 = PS11 * P[3][5] + PS12 * P[0][5] - PS13 * P[1][5] - PS34 * P[5][11] + PS6 * P[5][12] - PS9 * P[5][10] + P[2][5];
    const ftype PS150 = PS11 * P[3][6] + PS12 * P[0][6] - PS13 * P[1][6] - PS34 * P[6][11] + PS6 * P[6][12] - PS9 * P[6][10] + P[2][6];
    const ftype PS151 = PS106 - PS11 * P[2][10] + PS12 * P[1][10] + PS13 * P[0][10] - PS15 + PS7 * P[10][10] + P[3][10];
    const ftype PS152 = PS12 * P[1][1] + PS129 + PS24 - PS34 * P[1][12] - PS6 * P[1][11] + PS7 * P[1][10] + P[1][3];
    const ftype PS153 = -PS101 + PS13 * P[0][0] + PS134 - PS34 * P[0][12] - PS6 * P[0][11] + PS7 * P[0][10] + P[0][3];
    const ftype PS154 = PS104 - PS11 * P[2][12] + PS12 * P[1][12] + PS13 * P[0][12] - PS131 - PS34 * P[12][12] + P[3][12];
    const ftype PS155 = -PS11 * P[2][11] + PS12 * P[1][11] + PS126 + PS13 * P[0][11] - PS6 * P[11][11] + PS8 + P[3][11];
    const ftype PS156 = -PS11 * P[2][2] + PS21 - PS34 * P[2][12] - PS6 * P[2][11] + PS7 * P[2][10] + PS98 + P[2][3];
    const ftype PS157 = PS111 - PS136 + PS32 - PS34 * P[3][12] - PS6 * P[3][11] + PS7 * P[3][10] + P[3][3];
    const ftype PS158 = -PS11 * P[2][13] + PS12 * P[1][13] + PS13 * P[0][13] - PS34 * P[12][13] - PS6 * P[11][13] + PS7 * P[10][13] + P[3][13];
    const ftype PS159 = -PS11 * P[2][15] + PS12 * P[1][15] + PS13 * P[0][15] - PS34 * P[12][15] - PS6 * P[11][15] + PS7 * P[10][15] + P[3][15];
    const ftype PS160 = 2 * PS159;
    const ftype PS161 = 2 * PS157;
    const ftype PS162 = -PS11 * P[2][14] + PS12 * P[1][14] + PS13 * P[0][14] - PS34 * P[12][14] - PS6 * P[11][14] + PS7 * P[10][14] + P[3][14];
    const ftype PS163 = 2 * PS162;
    const ftype PS164 = 2 * PS152;
    const ftype PS165 = 2 * PS153;
    const ftype PS166 = 2 * PS156;
    const ftype PS167 = -PS11 * P[2][4] + PS12 * P[1][4] + PS13 * P[0][4] - PS34 * P[4][12] - PS6 * P[4][11] + PS7 * P[4][10] + P[3][4];
    const ftype PS168 = 2 * PS158;
    const ftype PS169 = -PS11 * P[2][5] + PS12 * P[1][5] + PS13 * P[0][5] - PS34 * P[5][12] - PS6 * P[5][11] + PS7 * P[5][10] + P[3][5];
    const ftype PS170 = -PS11 * P[2][6] + PS12 * P[1][6] + PS13 * P[0][6] - PS34 * P[6][12] - PS6 * P[6][11] + PS7 * P[6][10] + P[3][6];
    const ftype PS171 = 2 * PS45;
    const ftype PS172 = 2 * PS56;
    const ftype PS173 = 2 * PS61;
    const ftype PS174 = 2 * PS66;
    const ftype PS175 = 2 * PS71;
    const ftype PS176 = 2 * PS54;
    const ftype PS177 = -PS171 * P[13][15] + PS172 * P[13][14] + PS173 * P[1][13] + PS174 * P[0][13] + PS175 * P[2][13] - PS176 * P[3][13] + PS43 * P[13][13] + P[4][13];
    const ftype PS178 = -PS171 * P[15][15] + PS172 * P[14][15] + PS173 * P[1][15] + PS174 * P[0][15] + PS175 * P[2][15] - PS176 * P[3][15] + PS43 * P[13][15] + P[4][15];
    const ftype PS179 = -PS171 * P[3][15] + PS172 * P[3][14] + PS173 * P[1][3] + PS174 * P[0][3] + PS175 * P[2][3] - PS176 * P[3][3] + PS43 * P[3][13] + P[3][4];
    const ftype PS180 = -PS171 * P[14][15] + PS172 * P[14][14] + PS173 * P[1][14] + PS174 * P[0][14] + PS175 * P[2][14] - PS176 * P[3][14] + PS43 * P[13][14] + P[4][14];
    const ftype PS181 = -PS171 * P[1][15] + PS172 * P[1][14] + PS173 * P[1][1] + PS174 * P[0][1] + PS175 * P[1][2] - PS176 * P[1][3] + PS43 * P[1][13] + P[1][4];
    const ftype PS182 = -PS171 * P[0][15] + PS172 * P[0][14] + PS173 * P[0][1] + PS174 * P[0][0] + PS175 * P[0][2] - PS176 * P[0][3] + PS43 * P[0][13] + P[0][4];
    const ftype PS183 = -PS171 * P[2][15] + PS172 * P[2][14] + PS173 * P[1][2] + PS174 * P[0][2] + PS175 * P[2][2] - PS176 * P[2][3] + PS43 * P[2][13] + P[2][4];
    const ftype PS184 = 4 * dvyVar;
    const ftype PS185 = 4 * dvzVar;
    const ftype PS186 = -PS171 * P[4][15] + PS172 * P[4][14] + PS173 * P[1][4] + PS174 * P[0][4] + PS175 * P[2][4] - PS176 * P[3][4] + PS43 * P[4][13] + P[4][4];
    const ftype PS187 = 2 * PS177;
    const ftype PS188 = 2 * PS182;
    const ftype PS189 = 2 * PS181;
    const ftype PS190 = 2 * PS81;
    const ftype PS191 = 2 * PS183;
    const ftype PS192 = 2 * PS179;
    const ftype PS193 = 2 * PS76;
    const ftype PS194 = PS43 * dvxVar;
    const ftype PS195 = PS75 * dvyVar;
    const ftype PS196 = -PS171 * P[5][15] + PS172 * P[5][14] + PS173 * P[1][5] + PS174 * P[0][5] + PS175 * P[2][5] - PS176 * P[3][5] + PS43 * P[5][13] + P[4][5];
    const ftype PS197 = 2 * PS88;
    const ftype PS198 = PS87 * dvzVar;
    const ftype PS199 = 2 * PS90;
    const ftype PS200 = -PS171 * P[6][15] + PS172 * P[6][14] + PS173 * P[1][6] + PS174 * P[0][6] + PS175 * P[2][6] - PS176 * P[3][6] + PS43 * P[6][13] + P[4][6];
    const ftype PS201 = 2 * PS83;
    const ftype PS202 = 2 * PS78;
    const ftype PS203 = 2 * PS85;
    const ftype PS204 = 2 * PS80;
    const ftype PS205 = PS190 * P[14][15] - PS193 * P[13][14] + PS201 * P[2][14] - PS202 * P[0][14] + PS203 * P[3][14] - PS204 * P[1][14] + PS75 * P[14][14] + P[5][14];
    const ftype PS206 = PS190 * P[13][15] - PS193 * P[13][13] + PS201 * P[2][13] - PS202 * P[0][13] + PS203 * P[3][13] - PS204 * P[1][13] + PS75 * P[13][14] + P[5][13];
    const ftype PS207 = PS190 * P[0][15] - PS193 * P[0][13] + PS201 * P[0][2] - PS202 * P[0][0] + PS203 * P[0][3] - PS204 * P[0][1] + PS75 * P[0][14] + P[0][5];
    const ftype PS208 = PS190 * P[1][15] - PS193 * P[1][13] + PS201 * P[1][2] - PS202 * P[0][1] + PS203 * P[1][3] - PS204 * P[1][1] + PS75 * P[1][14] + P[1][5];
    const ftype PS209 = PS190 * P[15][15] - PS193 * P[13][15] + PS201 * P[2][15] - PS202 * P[0][15] + PS203 * P[3][15] - PS204 * P[1][15] + PS75 * P[14][15] + P[5][15];
    const ftype PS210 = PS190 * P[2][15] - PS193 * P[2][13] + PS201 * P[2][2] - PS202 * P[0][2] + PS203 * P[2][3] - PS204 * P[1][2] + PS75 * P[2][14] + P[2][5];
    const ftype PS211 = PS190 * P[3][15] - PS193 * P[3][13] + PS201 * P[2][3] - PS202 * P[0][3] + PS203 * P[3][3] - PS204 * P[1][3] + PS75 * P[3][14] + P[3][5];
    const ftype PS212 = 4 * dvxVar;
    const ftype PS213 = PS190 * P[5][15] - PS193 * P[5][13] + PS201 * P[2][5] - PS202 * P[0][5] + PS203 * P[3][5] - PS204 * P[1][5] + PS75 * P[5][14] + P[5][5];
    const ftype PS214 = 2 * PS89;
    const ftype PS215 = 2 * PS91;
    const ftype PS216 = 2 * PS92;
    const ftype PS217 = 2 * PS93;
    const ftype PS218 = PS190 * P[6][15] - PS193 * P[6][13] + PS201 * P[2][6] - PS202 * P[0][6] + PS203 * P[3][6] - PS204 * P[1][6] + PS75 * P[6][14] + P[5][6];
    const ftype PS219 = -PS197 * P[14][15] + PS199 * P[13][15] - PS214 * P[2][15] + PS215 * P[3][15] + PS216 * P[0][15] + PS217 * P[1][15] + PS87 * P[15][15] + P[6][15];
    const ftype PS220 = -PS197 * P[14][14] + PS199 * P[13][14] - PS214 * P[2][14] + PS215 * P[3][14] + PS216 * P[0][14] + PS217 * P[1][14] + PS87 * P[14][15] + P[6][14];
    const ftype PS221 = -PS197 * P[13][14] + PS199 * P[13][13] - PS214 * P[2][13] + PS215 * P[3][13] + PS216 * P[0][13] + PS217 * P[1][13] + PS87 * P[13][15] + P[6][13];
    const ftype PS222 = -PS197 * P[6][14] + PS199 * P[6][13] - PS214 * P[2][6] + PS215 * P[3][6] + PS216 * P[0][6] + PS217 * P[1][6] + PS87 * P[6][15] + P[6][6];
    // ---- END verbatim PS-intermediate transcription ----

    // ---- BEGIN verbatim transcription of the dense nextP[i][j], i,j in
    // 0..9 (AP_NavEKF3_core.cpp ~line 1190-1477) ----
    nextP[0][0] = PS0 * PS1 - PS11 * PS23 - PS12 * PS26 - PS13 * PS29 + PS14 * PS6 + PS17 * PS7 + PS2 * PS3 + PS20 * PS9 + PS33 + PS4 * PS5;
    nextP[0][1] = -PS1 * PS36 + PS11 * PS33 - PS12 * PS29 + PS13 * PS26 - PS14 * PS34 + PS17 * PS9 - PS20 * PS7 + PS23 + PS3 * PS35 - PS35 * PS5;
    nextP[1][1] = PS1 * PS95 + PS100 * PS11 + PS102 * PS13 - PS105 * PS34 - PS107 * PS7 - PS109 * PS12 + PS112 + PS2 * PS5 + PS3 * PS4 + PS9 * PS97;
    nextP[0][2] = -PS1 * PS37 + PS11 * PS29 + PS12 * PS33 - PS13 * PS23 - PS14 * PS9 - PS17 * PS34 + PS20 * PS6 + PS26 - PS3 * PS38 + PS37 * PS5;
    nextP[1][2] = PS1 * PS40 + PS100 * PS12 + PS102 - PS105 * PS9 + PS107 * PS6 + PS109 * PS11 - PS112 * PS13 - PS3 * PS40 - PS34 * PS97 - PS39 * PS5;
    nextP[2][2] = PS0 * PS5 + PS1 * PS4 + PS11 * PS128 + PS12 * PS130 + PS127 * PS6 - PS13 * PS135 - PS132 * PS34 - PS133 * PS9 + PS137 + PS3 * PS95;
    nextP[0][3] = PS1 * PS39 - PS11 * PS26 + PS12 * PS23 + PS13 * PS33 + PS14 * PS7 - PS17 * PS6 - PS20 * PS34 + PS29 - PS3 * PS39 - PS40 * PS5;
    nextP[1][3] = -PS1 * PS38 + PS100 * PS13 - PS102 * PS11 + PS105 * PS7 - PS107 * PS34 + PS109 + PS112 * PS12 - PS3 * PS37 + PS38 * PS5 - PS6 * PS97;
    nextP[2][3] = -PS1 * PS35 - PS11 * PS137 + PS12 * PS135 - PS127 * PS34 + PS128 + PS13 * PS130 - PS132 * PS6 + PS133 * PS7 + PS3 * PS36 - PS36 * PS5;
    nextP[3][3] = PS0 * PS3 + PS1 * PS2 - PS11 * PS156 + PS12 * PS152 + PS13 * PS153 + PS151 * PS7 - PS154 * PS34 - PS155 * PS6 + PS157 + PS5 * PS95;

    if (quatCovResetOnly) {
        for (int row = 0; row <= 3; ++row) {
            P[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)] =
                clamp(nextP[static_cast<std::size_t>(row)][static_cast<std::size_t>(row)], ftype(0.0), ftype(1.0));
            for (int col = 0; col < row; ++col) {
                P[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                    P[static_cast<std::size_t>(col)][static_cast<std::size_t>(row)] =
                        nextP[static_cast<std::size_t>(col)][static_cast<std::size_t>(row)];
            }
        }
        return;
    }

    nextP[0][4] = PS43 * PS44 - PS45 * PS47 - PS54 * PS55 + PS56 * PS58 + PS61 * PS62 + PS66 * PS67 + PS71 * PS72 + PS73;
    nextP[1][4] = PS113 * PS43 - PS115 * PS45 - PS116 * PS54 + PS118 * PS56 + PS119 * PS61 + PS120 * PS66 + PS121 * PS71 + PS122;
    nextP[2][4] = PS138 * PS43 - PS140 * PS45 - PS141 * PS54 + PS143 * PS56 + PS144 * PS61 + PS145 * PS66 + PS146 * PS71 + PS147;
    nextP[3][4] = PS158 * PS43 - PS160 * PS45 - PS161 * PS54 + PS163 * PS56 + PS164 * PS61 + PS165 * PS66 + PS166 * PS71 + PS167;
    nextP[4][4] = -PS171 * PS178 + PS172 * PS180 + PS173 * PS181 + PS174 * PS182 + PS175 * PS183 - PS176 * PS179 + PS177 * PS43 + PS184 * sq(PS56) + PS185 * sq(PS45) + PS186 + sq(PS43) * dvxVar;
    nextP[0][5] = PS47 * PS81 + PS55 * PS85 + PS57 * PS75 - PS62 * PS80 - PS67 * PS78 + PS72 * PS83 - PS76 * PS77 + PS86;
    nextP[1][5] = PS115 * PS81 + PS116 * PS85 + PS117 * PS75 - PS119 * PS80 - PS120 * PS78 + PS121 * PS83 - PS123 * PS76 + PS124;
    nextP[2][5] = PS140 * PS81 + PS141 * PS85 + PS142 * PS75 - PS144 * PS80 - PS145 * PS78 + PS146 * PS83 - PS148 * PS76 + PS149;
    nextP[3][5] = PS160 * PS81 + PS161 * PS85 + PS162 * PS75 - PS164 * PS80 - PS165 * PS78 + PS166 * PS83 - PS168 * PS76 + PS169;
    nextP[4][5] = PS172 * PS195 + PS178 * PS190 + PS180 * PS75 - PS185 * PS45 * PS81 - PS187 * PS76 - PS188 * PS78 - PS189 * PS80 + PS191 * PS83 + PS192 * PS85 - PS193 * PS194 + PS196;
    nextP[5][5] = PS185 * sq(PS81) + PS190 * PS209 - PS193 * PS206 + PS201 * PS210 - PS202 * PS207 + PS203 * PS211 - PS204 * PS208 + PS205 * PS75 + PS212 * sq(PS76) + PS213 + sq(PS75) * dvyVar;
    nextP[0][6] = PS46 * PS87 + PS55 * PS91 - PS58 * PS88 + PS62 * PS93 + PS67 * PS92 - PS72 * PS89 + PS77 * PS90 + PS94;
    nextP[1][6] = PS114 * PS87 + PS116 * PS91 - PS118 * PS88 + PS119 * PS93 + PS120 * PS92 - PS121 * PS89 + PS123 * PS90 + PS125;
    nextP[2][6] = PS139 * PS87 + PS141 * PS91 - PS143 * PS88 + PS144 * PS93 + PS145 * PS92 - PS146 * PS89 + PS148 * PS90 + PS150;
    nextP[3][6] = PS159 * PS87 + PS161 * PS91 - PS163 * PS88 + PS164 * PS93 + PS165 * PS92 - PS166 * PS89 + PS168 * PS90 + PS170;
    nextP[4][6] = -PS171 * PS198 + PS178 * PS87 - PS180 * PS197 - PS184 * PS56 * PS88 + PS187 * PS90 + PS188 * PS92 + PS189 * PS93 - PS191 * PS89 + PS192 * PS91 + PS194 * PS199 + PS200;
    nextP[5][6] = PS190 * PS198 - PS195 * PS197 - PS197 * PS205 + PS199 * PS206 + PS207 * PS216 + PS208 * PS217 + PS209 * PS87 - PS210 * PS214 + PS211 * PS215 - PS212 * PS76 * PS90 + PS218;
    nextP[6][6] = PS184 * sq(PS88) - PS197 * PS220 + PS199 * PS221 + PS212 * sq(PS90) -
                  PS214 * (-PS197 * P[2][14] + PS199 * P[2][13] - PS214 * P[2][2] + PS215 * P[2][3] + PS216 * P[0][2] + PS217 * P[1][2] + PS87 * P[2][15] + P[2][6]) +
                  PS215 * (-PS197 * P[3][14] + PS199 * P[3][13] - PS214 * P[2][3] + PS215 * P[3][3] + PS216 * P[0][3] + PS217 * P[1][3] + PS87 * P[3][15] + P[3][6]) +
                  PS216 * (-PS197 * P[0][14] + PS199 * P[0][13] - PS214 * P[0][2] + PS215 * P[0][3] + PS216 * P[0][0] + PS217 * P[0][1] + PS87 * P[0][15] + P[0][6]) +
                  PS217 * (-PS197 * P[1][14] + PS199 * P[1][13] - PS214 * P[1][2] + PS215 * P[1][3] + PS216 * P[0][1] + PS217 * P[1][1] + PS87 * P[1][15] + P[1][6]) +
                  PS219 * PS87 + PS222 + sq(PS87) * dvzVar;
    nextP[0][7] = -PS11 * P[1][7] - PS12 * P[2][7] - PS13 * P[3][7] + PS6 * P[7][10] + PS7 * P[7][11] + PS73 * dt + PS9 * P[7][12] + P[0][7];
    nextP[1][7] = PS11 * P[0][7] - PS12 * P[3][7] + PS122 * dt + PS13 * P[2][7] - PS34 * P[7][10] - PS7 * P[7][12] + PS9 * P[7][11] + P[1][7];
    nextP[2][7] = PS11 * P[3][7] + PS12 * P[0][7] - PS13 * P[1][7] + PS147 * dt - PS34 * P[7][11] + PS6 * P[7][12] - PS9 * P[7][10] + P[2][7];
    nextP[3][7] = -PS11 * P[2][7] + PS12 * P[1][7] + PS13 * P[0][7] + PS167 * dt - PS34 * P[7][12] - PS6 * P[7][11] + PS7 * P[7][10] + P[3][7];
    nextP[4][7] = -PS171 * P[7][15] + PS172 * P[7][14] + PS173 * P[1][7] + PS174 * P[0][7] + PS175 * P[2][7] - PS176 * P[3][7] + PS186 * dt + PS43 * P[7][13] + P[4][7];
    nextP[5][7] = PS190 * P[7][15] - PS193 * P[7][13] + PS201 * P[2][7] - PS202 * P[0][7] + PS203 * P[3][7] - PS204 * P[1][7] + PS75 * P[7][14] + P[5][7] + dt * (PS190 * P[4][15] - PS193 * P[4][13] + PS201 * P[2][4] - PS202 * P[0][4] + PS203 * P[3][4] - PS204 * P[1][4] + PS75 * P[4][14] + P[4][5]);
    nextP[6][7] = -PS197 * P[7][14] + PS199 * P[7][13] - PS214 * P[2][7] + PS215 * P[3][7] + PS216 * P[0][7] + PS217 * P[1][7] + PS87 * P[7][15] + P[6][7] + dt * (-PS197 * P[4][14] + PS199 * P[4][13] - PS214 * P[2][4] + PS215 * P[3][4] + PS216 * P[0][4] + PS217 * P[1][4] + PS87 * P[4][15] + P[4][6]);
    nextP[7][7] = P[4][7] * dt + P[7][7] + dt * (P[4][4] * dt + P[4][7]);
    nextP[0][8] = -PS11 * P[1][8] - PS12 * P[2][8] - PS13 * P[3][8] + PS6 * P[8][10] + PS7 * P[8][11] + PS86 * dt + PS9 * P[8][12] + P[0][8];
    nextP[1][8] = PS11 * P[0][8] - PS12 * P[3][8] + PS124 * dt + PS13 * P[2][8] - PS34 * P[8][10] - PS7 * P[8][12] + PS9 * P[8][11] + P[1][8];
    nextP[2][8] = PS11 * P[3][8] + PS12 * P[0][8] - PS13 * P[1][8] + PS149 * dt - PS34 * P[8][11] + PS6 * P[8][12] - PS9 * P[8][10] + P[2][8];
    nextP[3][8] = -PS11 * P[2][8] + PS12 * P[1][8] + PS13 * P[0][8] + PS169 * dt - PS34 * P[8][12] - PS6 * P[8][11] + PS7 * P[8][10] + P[3][8];
    nextP[4][8] = -PS171 * P[8][15] + PS172 * P[8][14] + PS173 * P[1][8] + PS174 * P[0][8] + PS175 * P[2][8] - PS176 * P[3][8] + PS196 * dt + PS43 * P[8][13] + P[4][8];
    nextP[5][8] = PS190 * P[8][15] - PS193 * P[8][13] + PS201 * P[2][8] - PS202 * P[0][8] + PS203 * P[3][8] - PS204 * P[1][8] + PS213 * dt + PS75 * P[8][14] + P[5][8];
    nextP[6][8] = -PS197 * P[8][14] + PS199 * P[8][13] - PS214 * P[2][8] + PS215 * P[3][8] + PS216 * P[0][8] + PS217 * P[1][8] + PS87 * P[8][15] + P[6][8] + dt * (-PS197 * P[5][14] + PS199 * P[5][13] - PS214 * P[2][5] + PS215 * P[3][5] + PS216 * P[0][5] + PS217 * P[1][5] + PS87 * P[5][15] + P[5][6]);
    nextP[7][8] = P[4][8] * dt + P[7][8] + dt * (P[4][5] * dt + P[5][7]);
    nextP[8][8] = P[5][8] * dt + P[8][8] + dt * (P[5][5] * dt + P[5][8]);
    nextP[0][9] = -PS11 * P[1][9] - PS12 * P[2][9] - PS13 * P[3][9] + PS6 * P[9][10] + PS7 * P[9][11] + PS9 * P[9][12] + PS94 * dt + P[0][9];
    nextP[1][9] = PS11 * P[0][9] - PS12 * P[3][9] + PS125 * dt + PS13 * P[2][9] - PS34 * P[9][10] - PS7 * P[9][12] + PS9 * P[9][11] + P[1][9];
    nextP[2][9] = PS11 * P[3][9] + PS12 * P[0][9] - PS13 * P[1][9] + PS150 * dt - PS34 * P[9][11] + PS6 * P[9][12] - PS9 * P[9][10] + P[2][9];
    nextP[3][9] = -PS11 * P[2][9] + PS12 * P[1][9] + PS13 * P[0][9] + PS170 * dt - PS34 * P[9][12] - PS6 * P[9][11] + PS7 * P[9][10] + P[3][9];
    nextP[4][9] = -PS171 * P[9][15] + PS172 * P[9][14] + PS173 * P[1][9] + PS174 * P[0][9] + PS175 * P[2][9] - PS176 * P[3][9] + PS200 * dt + PS43 * P[9][13] + P[4][9];
    nextP[5][9] = PS190 * P[9][15] - PS193 * P[9][13] + PS201 * P[2][9] - PS202 * P[0][9] + PS203 * P[3][9] - PS204 * P[1][9] + PS218 * dt + PS75 * P[9][14] + P[5][9];
    nextP[6][9] = -PS197 * P[9][14] + PS199 * P[9][13] - PS214 * P[2][9] + PS215 * P[3][9] + PS216 * P[0][9] + PS217 * P[1][9] + PS222 * dt + PS87 * P[9][15] + P[6][9];
    nextP[7][9] = P[4][9] * dt + P[7][9] + dt * (P[4][6] * dt + P[6][7]);
    nextP[8][9] = P[5][9] * dt + P[8][9] + dt * (P[5][6] * dt + P[6][8]);
    nextP[9][9] = P[6][9] * dt + P[9][9] + dt * (P[6][6] * dt + P[6][9]);
    // ---- END verbatim dense-block transcription ----

    // ---- Correlation with gyro/accel-bias states (columns 10..15) - a
    // small loop, algebraically verified identical to upstream's unrolled
    // column-10..15 blocks (each of the row-0..9 formulas below matches
    // the corresponding upstream nextP[row][10..15] expression exactly,
    // using P's symmetry: e.g. upstream's PS14 == this loop's row-0
    // formula evaluated at j=10). See this file's own banner. ----
    for (int j = 10; j <= 15; ++j) {
        const auto jj = static_cast<std::size_t>(j);
        const ftype p0j = P[0][jj], p1j = P[1][jj], p2j = P[2][jj], p3j = P[3][jj];
        const ftype p10j = P[10][jj], p11j = P[11][jj], p12j = P[12][jj];
        const ftype p13j = P[13][jj], p14j = P[14][jj], p15j = P[15][jj];
        const ftype p4j = P[4][jj], p5j = P[5][jj], p6j = P[6][jj];
        nextP[0][jj] = -PS11 * p1j - PS12 * p2j - PS13 * p3j + PS6 * p10j + PS7 * p11j + PS9 * p12j + p0j;
        nextP[1][jj] = PS11 * p0j - PS12 * p3j + PS13 * p2j - PS34 * p10j - PS7 * p12j + PS9 * p11j + p1j;
        nextP[2][jj] = PS11 * p3j + PS12 * p0j - PS13 * p1j - PS34 * p11j + PS6 * p12j - PS9 * p10j + p2j;
        nextP[3][jj] = -PS11 * p2j + PS12 * p1j + PS13 * p0j - PS34 * p12j - PS6 * p11j + PS7 * p10j + p3j;
        nextP[4][jj] = -PS171 * p15j + PS172 * p14j + PS173 * p1j + PS174 * p0j + PS175 * p2j - PS176 * p3j + PS43 * p13j + p4j;
        nextP[5][jj] = PS190 * p15j - PS193 * p13j + PS201 * p2j - PS202 * p0j + PS203 * p3j - PS204 * p1j + PS75 * p14j + p5j;
        nextP[6][jj] = -PS197 * p14j + PS199 * p13j - PS214 * p2j + PS215 * p3j + PS216 * p0j + PS217 * p1j + PS87 * p15j + p6j;
        nextP[7][jj] = p4j * dt + P[7][jj];
        nextP[8][jj] = p5j * dt + P[8][jj];
        nextP[9][jj] = p6j * dt + P[9][jj];
        nextP[jj][jj] = P[jj][jj];
    }
    // Bias-bias correlations (10..15 vs 10..15) have no dynamics of their
    // own - identity pass-through, matching upstream's unrolled
    // nextP[10][11]=P[10][11]-style entries exactly.
    for (int i = 10; i <= 15; ++i) {
        for (int j = i + 1; j <= 15; ++j) {
            nextP[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
        }
    }

    // upstream's own real stateIndexLim, reused directly as this port's
    // established gate (CPP-056's state_index_lim()) rather than a new one.
    const int lim = state_index_lim();

    // CPP-065 phase 11: the mag/wind (16..23) extension of the dense
    // Jacobian block above. VERBATIM-TRANSCRIBED from upstream
    // AP_NavEKF3_core.cpp ~line 1571-1739 (`if (stateIndexLim > 15) { ...
    // if (stateIndexLim > 21) { ... } }`), reusing the EXACT SAME
    // PS-coefficient variables already verbatim-transcribed above for the
    // 0-15 block (verified directly, spot-checked far more than this
    // port's usual 2-4 checks given this touches the previously-untouched
    // dense block - see this ticket's commit message for the full
    // verification account) - no new PS-coefficients needed, only extended
    // column ranges. Gated by `lim` exactly as upstream gates by
    // stateIndexLim, with the 22-23 block correctly nested inside the >15
    // block, matching upstream's real nesting. At this port's real
    // defaults (inhibit_mag_states=true, inhibit_wind_states=true,
    // state_index_lim()==15), `lim > 15` is false and NONE of this new
    // code executes - a behavior-preserving change at default settings.
    if (lim > 15) {
        nextP[0][16] = -PS11 * P[1][16] - PS12 * P[2][16] - PS13 * P[3][16] + PS6 * P[10][16] + PS7 * P[11][16] + PS9 * P[12][16] + P[0][16];
        nextP[1][16] = PS11 * P[0][16] - PS12 * P[3][16] + PS13 * P[2][16] - PS34 * P[10][16] - PS7 * P[12][16] + PS9 * P[11][16] + P[1][16];
        nextP[2][16] = PS11 * P[3][16] + PS12 * P[0][16] - PS13 * P[1][16] - PS34 * P[11][16] + PS6 * P[12][16] - PS9 * P[10][16] + P[2][16];
        nextP[3][16] = -PS11 * P[2][16] + PS12 * P[1][16] + PS13 * P[0][16] - PS34 * P[12][16] - PS6 * P[11][16] + PS7 * P[10][16] + P[3][16];
        nextP[4][16] = -PS171 * P[15][16] + PS172 * P[14][16] + PS173 * P[1][16] + PS174 * P[0][16] + PS175 * P[2][16] - PS176 * P[3][16] + PS43 * P[13][16] + P[4][16];
        nextP[5][16] = PS190 * P[15][16] - PS193 * P[13][16] + PS201 * P[2][16] - PS202 * P[0][16] + PS203 * P[3][16] - PS204 * P[1][16] + PS75 * P[14][16] + P[5][16];
        nextP[6][16] = -PS197 * P[14][16] + PS199 * P[13][16] - PS214 * P[2][16] + PS215 * P[3][16] + PS216 * P[0][16] + PS217 * P[1][16] + PS87 * P[15][16] + P[6][16];
        nextP[7][16] = P[4][16] * dt + P[7][16];
        nextP[8][16] = P[5][16] * dt + P[8][16];
        nextP[9][16] = P[6][16] * dt + P[9][16];
        nextP[10][16] = P[10][16];
        nextP[11][16] = P[11][16];
        nextP[12][16] = P[12][16];
        nextP[13][16] = P[13][16];
        nextP[14][16] = P[14][16];
        nextP[15][16] = P[15][16];
        nextP[16][16] = P[16][16];
        nextP[0][17] = -PS11 * P[1][17] - PS12 * P[2][17] - PS13 * P[3][17] + PS6 * P[10][17] + PS7 * P[11][17] + PS9 * P[12][17] + P[0][17];
        nextP[1][17] = PS11 * P[0][17] - PS12 * P[3][17] + PS13 * P[2][17] - PS34 * P[10][17] - PS7 * P[12][17] + PS9 * P[11][17] + P[1][17];
        nextP[2][17] = PS11 * P[3][17] + PS12 * P[0][17] - PS13 * P[1][17] - PS34 * P[11][17] + PS6 * P[12][17] - PS9 * P[10][17] + P[2][17];
        nextP[3][17] = -PS11 * P[2][17] + PS12 * P[1][17] + PS13 * P[0][17] - PS34 * P[12][17] - PS6 * P[11][17] + PS7 * P[10][17] + P[3][17];
        nextP[4][17] = -PS171 * P[15][17] + PS172 * P[14][17] + PS173 * P[1][17] + PS174 * P[0][17] + PS175 * P[2][17] - PS176 * P[3][17] + PS43 * P[13][17] + P[4][17];
        nextP[5][17] = PS190 * P[15][17] - PS193 * P[13][17] + PS201 * P[2][17] - PS202 * P[0][17] + PS203 * P[3][17] - PS204 * P[1][17] + PS75 * P[14][17] + P[5][17];
        nextP[6][17] = -PS197 * P[14][17] + PS199 * P[13][17] - PS214 * P[2][17] + PS215 * P[3][17] + PS216 * P[0][17] + PS217 * P[1][17] + PS87 * P[15][17] + P[6][17];
        nextP[7][17] = P[4][17] * dt + P[7][17];
        nextP[8][17] = P[5][17] * dt + P[8][17];
        nextP[9][17] = P[6][17] * dt + P[9][17];
        nextP[10][17] = P[10][17];
        nextP[11][17] = P[11][17];
        nextP[12][17] = P[12][17];
        nextP[13][17] = P[13][17];
        nextP[14][17] = P[14][17];
        nextP[15][17] = P[15][17];
        nextP[16][17] = P[16][17];
        nextP[17][17] = P[17][17];
        nextP[0][18] = -PS11 * P[1][18] - PS12 * P[2][18] - PS13 * P[3][18] + PS6 * P[10][18] + PS7 * P[11][18] + PS9 * P[12][18] + P[0][18];
        nextP[1][18] = PS11 * P[0][18] - PS12 * P[3][18] + PS13 * P[2][18] - PS34 * P[10][18] - PS7 * P[12][18] + PS9 * P[11][18] + P[1][18];
        nextP[2][18] = PS11 * P[3][18] + PS12 * P[0][18] - PS13 * P[1][18] - PS34 * P[11][18] + PS6 * P[12][18] - PS9 * P[10][18] + P[2][18];
        nextP[3][18] = -PS11 * P[2][18] + PS12 * P[1][18] + PS13 * P[0][18] - PS34 * P[12][18] - PS6 * P[11][18] + PS7 * P[10][18] + P[3][18];
        nextP[4][18] = -PS171 * P[15][18] + PS172 * P[14][18] + PS173 * P[1][18] + PS174 * P[0][18] + PS175 * P[2][18] - PS176 * P[3][18] + PS43 * P[13][18] + P[4][18];
        nextP[5][18] = PS190 * P[15][18] - PS193 * P[13][18] + PS201 * P[2][18] - PS202 * P[0][18] + PS203 * P[3][18] - PS204 * P[1][18] + PS75 * P[14][18] + P[5][18];
        nextP[6][18] = -PS197 * P[14][18] + PS199 * P[13][18] - PS214 * P[2][18] + PS215 * P[3][18] + PS216 * P[0][18] + PS217 * P[1][18] + PS87 * P[15][18] + P[6][18];
        nextP[7][18] = P[4][18] * dt + P[7][18];
        nextP[8][18] = P[5][18] * dt + P[8][18];
        nextP[9][18] = P[6][18] * dt + P[9][18];
        nextP[10][18] = P[10][18];
        nextP[11][18] = P[11][18];
        nextP[12][18] = P[12][18];
        nextP[13][18] = P[13][18];
        nextP[14][18] = P[14][18];
        nextP[15][18] = P[15][18];
        nextP[16][18] = P[16][18];
        nextP[17][18] = P[17][18];
        nextP[18][18] = P[18][18];
        nextP[0][19] = -PS11 * P[1][19] - PS12 * P[2][19] - PS13 * P[3][19] + PS6 * P[10][19] + PS7 * P[11][19] + PS9 * P[12][19] + P[0][19];
        nextP[1][19] = PS11 * P[0][19] - PS12 * P[3][19] + PS13 * P[2][19] - PS34 * P[10][19] - PS7 * P[12][19] + PS9 * P[11][19] + P[1][19];
        nextP[2][19] = PS11 * P[3][19] + PS12 * P[0][19] - PS13 * P[1][19] - PS34 * P[11][19] + PS6 * P[12][19] - PS9 * P[10][19] + P[2][19];
        nextP[3][19] = -PS11 * P[2][19] + PS12 * P[1][19] + PS13 * P[0][19] - PS34 * P[12][19] - PS6 * P[11][19] + PS7 * P[10][19] + P[3][19];
        nextP[4][19] = -PS171 * P[15][19] + PS172 * P[14][19] + PS173 * P[1][19] + PS174 * P[0][19] + PS175 * P[2][19] - PS176 * P[3][19] + PS43 * P[13][19] + P[4][19];
        nextP[5][19] = PS190 * P[15][19] - PS193 * P[13][19] + PS201 * P[2][19] - PS202 * P[0][19] + PS203 * P[3][19] - PS204 * P[1][19] + PS75 * P[14][19] + P[5][19];
        nextP[6][19] = -PS197 * P[14][19] + PS199 * P[13][19] - PS214 * P[2][19] + PS215 * P[3][19] + PS216 * P[0][19] + PS217 * P[1][19] + PS87 * P[15][19] + P[6][19];
        nextP[7][19] = P[4][19] * dt + P[7][19];
        nextP[8][19] = P[5][19] * dt + P[8][19];
        nextP[9][19] = P[6][19] * dt + P[9][19];
        nextP[10][19] = P[10][19];
        nextP[11][19] = P[11][19];
        nextP[12][19] = P[12][19];
        nextP[13][19] = P[13][19];
        nextP[14][19] = P[14][19];
        nextP[15][19] = P[15][19];
        nextP[16][19] = P[16][19];
        nextP[17][19] = P[17][19];
        nextP[18][19] = P[18][19];
        nextP[19][19] = P[19][19];
        nextP[0][20] = -PS11 * P[1][20] - PS12 * P[2][20] - PS13 * P[3][20] + PS6 * P[10][20] + PS7 * P[11][20] + PS9 * P[12][20] + P[0][20];
        nextP[1][20] = PS11 * P[0][20] - PS12 * P[3][20] + PS13 * P[2][20] - PS34 * P[10][20] - PS7 * P[12][20] + PS9 * P[11][20] + P[1][20];
        nextP[2][20] = PS11 * P[3][20] + PS12 * P[0][20] - PS13 * P[1][20] - PS34 * P[11][20] + PS6 * P[12][20] - PS9 * P[10][20] + P[2][20];
        nextP[3][20] = -PS11 * P[2][20] + PS12 * P[1][20] + PS13 * P[0][20] - PS34 * P[12][20] - PS6 * P[11][20] + PS7 * P[10][20] + P[3][20];
        nextP[4][20] = -PS171 * P[15][20] + PS172 * P[14][20] + PS173 * P[1][20] + PS174 * P[0][20] + PS175 * P[2][20] - PS176 * P[3][20] + PS43 * P[13][20] + P[4][20];
        nextP[5][20] = PS190 * P[15][20] - PS193 * P[13][20] + PS201 * P[2][20] - PS202 * P[0][20] + PS203 * P[3][20] - PS204 * P[1][20] + PS75 * P[14][20] + P[5][20];
        nextP[6][20] = -PS197 * P[14][20] + PS199 * P[13][20] - PS214 * P[2][20] + PS215 * P[3][20] + PS216 * P[0][20] + PS217 * P[1][20] + PS87 * P[15][20] + P[6][20];
        nextP[7][20] = P[4][20] * dt + P[7][20];
        nextP[8][20] = P[5][20] * dt + P[8][20];
        nextP[9][20] = P[6][20] * dt + P[9][20];
        nextP[10][20] = P[10][20];
        nextP[11][20] = P[11][20];
        nextP[12][20] = P[12][20];
        nextP[13][20] = P[13][20];
        nextP[14][20] = P[14][20];
        nextP[15][20] = P[15][20];
        nextP[16][20] = P[16][20];
        nextP[17][20] = P[17][20];
        nextP[18][20] = P[18][20];
        nextP[19][20] = P[19][20];
        nextP[20][20] = P[20][20];
        nextP[0][21] = -PS11 * P[1][21] - PS12 * P[2][21] - PS13 * P[3][21] + PS6 * P[10][21] + PS7 * P[11][21] + PS9 * P[12][21] + P[0][21];
        nextP[1][21] = PS11 * P[0][21] - PS12 * P[3][21] + PS13 * P[2][21] - PS34 * P[10][21] - PS7 * P[12][21] + PS9 * P[11][21] + P[1][21];
        nextP[2][21] = PS11 * P[3][21] + PS12 * P[0][21] - PS13 * P[1][21] - PS34 * P[11][21] + PS6 * P[12][21] - PS9 * P[10][21] + P[2][21];
        nextP[3][21] = -PS11 * P[2][21] + PS12 * P[1][21] + PS13 * P[0][21] - PS34 * P[12][21] - PS6 * P[11][21] + PS7 * P[10][21] + P[3][21];
        nextP[4][21] = -PS171 * P[15][21] + PS172 * P[14][21] + PS173 * P[1][21] + PS174 * P[0][21] + PS175 * P[2][21] - PS176 * P[3][21] + PS43 * P[13][21] + P[4][21];
        nextP[5][21] = PS190 * P[15][21] - PS193 * P[13][21] + PS201 * P[2][21] - PS202 * P[0][21] + PS203 * P[3][21] - PS204 * P[1][21] + PS75 * P[14][21] + P[5][21];
        nextP[6][21] = -PS197 * P[14][21] + PS199 * P[13][21] - PS214 * P[2][21] + PS215 * P[3][21] + PS216 * P[0][21] + PS217 * P[1][21] + PS87 * P[15][21] + P[6][21];
        nextP[7][21] = P[4][21] * dt + P[7][21];
        nextP[8][21] = P[5][21] * dt + P[8][21];
        nextP[9][21] = P[6][21] * dt + P[9][21];
        nextP[10][21] = P[10][21];
        nextP[11][21] = P[11][21];
        nextP[12][21] = P[12][21];
        nextP[13][21] = P[13][21];
        nextP[14][21] = P[14][21];
        nextP[15][21] = P[15][21];
        nextP[16][21] = P[16][21];
        nextP[17][21] = P[17][21];
        nextP[18][21] = P[18][21];
        nextP[19][21] = P[19][21];
        nextP[20][21] = P[20][21];
        nextP[21][21] = P[21][21];

        if (lim > 21) {
            nextP[0][22] = -PS11 * P[1][22] - PS12 * P[2][22] - PS13 * P[3][22] + PS6 * P[10][22] + PS7 * P[11][22] + PS9 * P[12][22] + P[0][22];
            nextP[1][22] = PS11 * P[0][22] - PS12 * P[3][22] + PS13 * P[2][22] - PS34 * P[10][22] - PS7 * P[12][22] + PS9 * P[11][22] + P[1][22];
            nextP[2][22] = PS11 * P[3][22] + PS12 * P[0][22] - PS13 * P[1][22] - PS34 * P[11][22] + PS6 * P[12][22] - PS9 * P[10][22] + P[2][22];
            nextP[3][22] = -PS11 * P[2][22] + PS12 * P[1][22] + PS13 * P[0][22] - PS34 * P[12][22] - PS6 * P[11][22] + PS7 * P[10][22] + P[3][22];
            nextP[4][22] = -PS171 * P[15][22] + PS172 * P[14][22] + PS173 * P[1][22] + PS174 * P[0][22] + PS175 * P[2][22] - PS176 * P[3][22] + PS43 * P[13][22] + P[4][22];
            nextP[5][22] = PS190 * P[15][22] - PS193 * P[13][22] + PS201 * P[2][22] - PS202 * P[0][22] + PS203 * P[3][22] - PS204 * P[1][22] + PS75 * P[14][22] + P[5][22];
            nextP[6][22] = -PS197 * P[14][22] + PS199 * P[13][22] - PS214 * P[2][22] + PS215 * P[3][22] + PS216 * P[0][22] + PS217 * P[1][22] + PS87 * P[15][22] + P[6][22];
            nextP[7][22] = P[4][22] * dt + P[7][22];
            nextP[8][22] = P[5][22] * dt + P[8][22];
            nextP[9][22] = P[6][22] * dt + P[9][22];
            nextP[10][22] = P[10][22];
            nextP[11][22] = P[11][22];
            nextP[12][22] = P[12][22];
            nextP[13][22] = P[13][22];
            nextP[14][22] = P[14][22];
            nextP[15][22] = P[15][22];
            nextP[16][22] = P[16][22];
            nextP[17][22] = P[17][22];
            nextP[18][22] = P[18][22];
            nextP[19][22] = P[19][22];
            nextP[20][22] = P[20][22];
            nextP[21][22] = P[21][22];
            nextP[22][22] = P[22][22];
            nextP[0][23] = -PS11 * P[1][23] - PS12 * P[2][23] - PS13 * P[3][23] + PS6 * P[10][23] + PS7 * P[11][23] + PS9 * P[12][23] + P[0][23];
            nextP[1][23] = PS11 * P[0][23] - PS12 * P[3][23] + PS13 * P[2][23] - PS34 * P[10][23] - PS7 * P[12][23] + PS9 * P[11][23] + P[1][23];
            nextP[2][23] = PS11 * P[3][23] + PS12 * P[0][23] - PS13 * P[1][23] - PS34 * P[11][23] + PS6 * P[12][23] - PS9 * P[10][23] + P[2][23];
            nextP[3][23] = -PS11 * P[2][23] + PS12 * P[1][23] + PS13 * P[0][23] - PS34 * P[12][23] - PS6 * P[11][23] + PS7 * P[10][23] + P[3][23];
            nextP[4][23] = -PS171 * P[15][23] + PS172 * P[14][23] + PS173 * P[1][23] + PS174 * P[0][23] + PS175 * P[2][23] - PS176 * P[3][23] + PS43 * P[13][23] + P[4][23];
            nextP[5][23] = PS190 * P[15][23] - PS193 * P[13][23] + PS201 * P[2][23] - PS202 * P[0][23] + PS203 * P[3][23] - PS204 * P[1][23] + PS75 * P[14][23] + P[5][23];
            nextP[6][23] = -PS197 * P[14][23] + PS199 * P[13][23] - PS214 * P[2][23] + PS215 * P[3][23] + PS216 * P[0][23] + PS217 * P[1][23] + PS87 * P[15][23] + P[6][23];
            nextP[7][23] = P[4][23] * dt + P[7][23];
            nextP[8][23] = P[5][23] * dt + P[8][23];
            nextP[9][23] = P[6][23] * dt + P[9][23];
            nextP[10][23] = P[10][23];
            nextP[11][23] = P[11][23];
            nextP[12][23] = P[12][23];
            nextP[13][23] = P[13][23];
            nextP[14][23] = P[14][23];
            nextP[15][23] = P[15][23];
            nextP[16][23] = P[16][23];
            nextP[17][23] = P[17][23];
            nextP[18][23] = P[18][23];
            nextP[19][23] = P[19][23];
            nextP[20][23] = P[20][23];
            nextP[21][23] = P[21][23];
            nextP[22][23] = P[22][23];
            nextP[23][23] = P[23][23];
        }
    }

    // Add process noise to the gyro/accel-bias diagonal (states 10..15)
    // and, per CPP-065 phase 11, the mag/wind diagonal (states 16..23)
    // when active. upstream: AP_NavEKF3_core.cpp ~line 1744-1748: `if
    // (stateIndexLim > 9) { for (i=10;i<=stateIndexLim;i++) nextP[i][i] +=
    // processNoiseVariance[i-10]; } }` - transcribed as a single bound
    // change (15 -> lim) rather than adding the `stateIndexLim > 9` outer
    // gate, since lim is never below 9 in this port and the loop already
    // naturally does nothing when lim==9 (10 <= 9 is false).
    for (int i = 10; i <= lim; ++i) {
        nextP[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
            nextP[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] +
            process_noise_variance[static_cast<std::size_t>(i - 10)];
    }

    // Position-variance-collapse guard (upstream: "if the total position
    // variance exceeds 1e4 (100m), then stop covariance growth"). CPP-065
    // phase 11: inner loop bound extended from the hardcoded 15 to `lim`,
    // matching upstream's own `for (j=0;j<=stateIndexLim;j++)`.
    if ((P[7][7] + P[8][8]) > ftype(1e4)) {
        for (int i = 7; i <= 8; ++i) {
            for (int j = 0; j <= lim; ++j) {
                nextP[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                    P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
                nextP[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] =
                    P[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)];
            }
        }
    }

    // Symmetric copy-back. CPP-065 phase 11: bound extended from the
    // hardcoded 15 to `lim`, matching upstream's own `for (row=0;
    // row<=stateIndexLim;row++)` - states 16..23 are now genuinely
    // populated by the block above when active, instead of being left for
    // constrain_variances() to zero every call.
    //
    // EXCLUDED (a real, separate upstream mechanism immediately following
    // this loop in the real source, AP_NavEKF3_core.cpp ~line 1793-1802:
    // `if (!inhibitDelVelBiasStates) { for (index=0;index<3;index++) { if
    // (dvelBiasAxisInhibit[index]) { zeroRows/zeroCols(P, stateIndex,
    // stateIndex); P[stateIndex][stateIndex] = dvelBiasAxisVarPrev[index];
    // } } }` - a per-axis delta-velocity-bias covariance reset tied to
    // `dvelBiasAxisInhibit[]`/ground-alignment axis inhibiting, distinct
    // from ConstrainVariances()'s own accel-bias handling. Already a named
    // phase-1/2 gap (ekf_core.hpp: "dvelBiasAxisInhibit[]... already a
    // named phase-1 gap" / "this phase's inhibit_del_vel_bias_states gate
    // is all-or-nothing across x/y/z") - this port has no per-axis
    // inhibiting to trigger it, so it is correctly absent rather than
    // approximated. Named freshly here since this is the first phase to
    // touch this exact region of covariance_prediction() since that gap
    // was originally disclosed.
    for (int row = 0; row <= lim; ++row) {
        const auto r = static_cast<std::size_t>(row);
        P[r][r] = nextP[r][r];
        for (int col = 0; col < row; ++col) {
            const auto c = static_cast<std::size_t>(col);
            P[r][c] = P[c][r] = nextP[c][r];
        }
    }

    constrain_variances(dt_ekf_avg);
    if (vert_vel_var_clip_counter > 0) {
        --vert_vel_var_clip_counter;
    }
}

// upstream: NavEKF3_core::ConstrainVariances(), AP_NavEKF3_core.cpp ~line
// 1877. See hpp banner simplification 2 (bias states never inhibited, no
// per-axis ground-alignment gate). CPP-065 phase 11: the mag/wind
// (16..21 / 22..23) blocks below now match upstream's real, runtime-
// gated if/else structure (per this port's covariance_prediction() and
// hpp banners) instead of the old phase-1 unconditional-zeroing
// simplification.
void EkfCore::constrain_variances(ftype dt_ekf_avg) {
    for (int i = 0; i <= 3; ++i) P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
        clamp(P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], ftype(0.0), ftype(1.0));
    for (int i = 4; i <= 5; ++i) P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
        clamp(P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], kVelStateMinVariance, ftype(1.0e3));

    // Vertical velocity variance-collapse guard (badIMUdata always false -
    // simplification 3).
    if (P[6][6] < kVelStateMinVariance) {
        P[6][6] = kVelStateMinVariance;
        vert_vel_var_clip_counter += kEkfTargetRateHz;
        if (vert_vel_var_clip_counter > kVertVelVarClipCountLim) {
            zero_row_col(P, 6);
            P[6][6] = sq(gps_vert_vel_noise);
            vert_vel_var_clip_counter = 0;
        }
    }

    for (int i = 7; i <= 9; ++i) P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
        clamp(P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], kPosStateMinVariance, ftype(1.0e6));

    // Gyro bias (10..12) - never inhibited in this phase (simplification 2).
    for (int i = 10; i <= 12; ++i) P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
        clamp(P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], ftype(0.0), sq(ftype(0.175) * dt_ekf_avg));

    // Accel bias (13..15) - never inhibited in this phase.
    {
        ftype max_state_var = 0;
        bool reset_required = false;
        for (int i = 13; i <= 15; ++i) {
            const ftype v = P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)];
            if (v > max_state_var) {
                max_state_var = v;
            } else if (v < kMinSafeStateVar) {
                reset_required = true;
            }
        }
        const ftype min_allowed_state_var = std::max(ftype(0.01) * max_state_var, kMinSafeStateVar);
        for (int i = 13; i <= 15; ++i) {
            P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
                clamp(P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], min_allowed_state_var, sq(ftype(10.0) * dt_ekf_avg));
        }
        if (reset_required) {
            zero_rows_cols(P, 13, 15);
            P[13][13] = sq(kAccelBiasLimScaler * acc_bias_lim * dt_ekf_avg);
            P[14][14] = P[13][13];
            P[15][15] = P[13][13];
            state.accel_bias.zero();
        }
    }

    // CPP-065 phase 11. upstream: AP_NavEKF3_core.cpp ~line 1974-1985:
    // `if (!inhibitMagStates) { for(i=16;i<=18;i++) P[i][i] =
    // constrain_ftype(P[i][i],0.0f,0.01f); for(i=19;i<=21;i++) P[i][i] =
    // constrain_ftype(P[i][i],0.0f,0.01f); } else { zeroCols(P,16,21);
    // zeroRows(P,16,21); }` - verified directly, transcribed exactly.
    // Replaces this port's old unconditional zeroing (a phase-1
    // simplification that is no longer accurate now that
    // covariance_prediction() genuinely populates these states when
    // active - see that function's own CPP-065 banner). At this port's
    // real default (inhibit_mag_states=true) this always takes the
    // `else` branch, i.e. exactly the old unconditional-zeroing behavior
    // - a behavior-preserving change at default settings.
    if (!inhibit_mag_states) {
        for (int i = 16; i <= 18; ++i)
            P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
                clamp(P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], ftype(0.0), ftype(0.01));  // earth magnetic field
        for (int i = 19; i <= 21; ++i)
            P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
                clamp(P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], ftype(0.0), ftype(0.01));  // body magnetic field
    } else {
        zero_rows_cols(P, 16, 21);
    }

    // CPP-065 phase 11. upstream: AP_NavEKF3_core.cpp ~line 1987-1995:
    // `if (!inhibitWindStates) { if (treatWindStatesAsTruth) {
    // P[23][23]=P[22][22]=0.0f; } else { for(i=22;i<=23;i++) P[i][i] =
    // constrain_ftype(P[i][i],0.0f,WIND_VEL_VARIANCE_MAX); } } else {
    // zeroCols(P,22,23); zeroRows(P,22,23); }` - verified directly.
    // `treatWindStatesAsTruth` is the already-established exclusion
    // (phase 2, reconfirmed phase 9/10 - no such field exists in this
    // port, see covariance_prediction()'s own process-noise banner
    // above), so this always takes the `clamp` path when wind states are
    // active - the ticket's own specified simplification. At this port's
    // real default (inhibit_wind_states=true) this always takes the
    // `else` branch, i.e. exactly the old unconditional-zeroing behavior
    // - a behavior-preserving change at default settings.
    if (!inhibit_wind_states) {
        for (int i = 22; i <= 23; ++i)
            P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
                clamp(P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)], ftype(0.0), kWindVelVarianceMax);
    } else {
        zero_rows_cols(P, 22, 23);
    }
}

// ============================================================================
// CPP-056 PHASE 2: GPS velocity/position fusion. See ekf_core.hpp's
// "CPP-056, PHASE 2" banner section for the full scope/exclusions
// discussion - only implementation-level notes live here.
// ============================================================================

// upstream: NavEKF3_core::updateStateIndexLim(), AP_NavEKF3_Control.cpp
// ~line 190-208 - transcribed directly (same nested structure, same
// literal thresholds).
int EkfCore::state_index_lim() const {
    if (inhibit_wind_states) {
        if (inhibit_mag_states) {
            if (inhibit_del_vel_bias_states) {
                return inhibit_del_ang_bias_states ? 9 : 12;
            }
            return 15;
        }
        return 21;
    }
    return 23;
}

// upstream: NavEKF3_core::ForceSymmetry(), AP_NavEKF3_core.cpp ~line
// 1862 - verbatim (`0.5*(P[i][j]+P[j][i])` averaging), bounded to `lim`
// exactly as upstream bounds it to stateIndexLim.
void EkfCore::force_symmetry(int lim) {
    for (int i = 1; i <= lim; ++i) {
        for (int j = 0; j <= i - 1; ++j) {
            const auto ii = static_cast<std::size_t>(i);
            const auto jj = static_cast<std::size_t>(j);
            const ftype temp = ftype(0.5) * (P[ii][jj] + P[jj][ii]);
            P[ii][jj] = temp;
            P[jj][ii] = temp;
        }
    }
}

// See ekf_core.hpp's declaration comment for why this exists (structured
// StateVector vs. upstream's flat statesArray alias) and why looping over
// all 24 conceptual slots unconditionally is provably identical to
// bounding at state_index_lim().
void EkfCore::apply_state_correction(const std::array<ftype, 24>& kfusion, ftype innovation) {
    std::array<ftype, 24> delta{};
    for (int i = 0; i < 24; ++i) {
        delta[static_cast<std::size_t>(i)] = kfusion[static_cast<std::size_t>(i)] * innovation;
    }
    state.quat[0] -= delta[0];
    state.quat[1] -= delta[1];
    state.quat[2] -= delta[2];
    state.quat[3] -= delta[3];
    state.velocity.x -= delta[4];
    state.velocity.y -= delta[5];
    state.velocity.z -= delta[6];
    state.position.x -= delta[7];
    state.position.y -= delta[8];
    state.position.z -= delta[9];
    state.gyro_bias.x -= delta[10];
    state.gyro_bias.y -= delta[11];
    state.gyro_bias.z -= delta[12];
    state.accel_bias.x -= delta[13];
    state.accel_bias.y -= delta[14];
    state.accel_bias.z -= delta[15];
    state.earth_magfield.x -= delta[16];
    state.earth_magfield.y -= delta[17];
    state.earth_magfield.z -= delta[18];
    state.body_magfield.x -= delta[19];
    state.body_magfield.y -= delta[20];
    state.body_magfield.z -= delta[21];
    state.wind_vel.x -= delta[22];
    state.wind_vel.y -= delta[23];
    // upstream: `stateStruct.quat.normalize()`, unconditional, right
    // after the state update loop (~line 1131).
    state.quat.normalize();
}

// upstream: FuseVelPosNED()'s obsIndex loop body, AP_NavEKF3_PosVelFusion.cpp
// ~line 1024-1163. Verified line-by-line against that source (not
// approximated) - see ekf_core.hpp's banner for exactly which upstream
// sub-branches (poorObservability/horizInhibit/dvelBiasAxisInhibit/
// badIMUdata/treatWindStatesAsTruth/gpsNoiseScaler) are real upstream
// mechanisms this port's current feature set never exercises, and are
// therefore correctly absent rather than approximated away.
bool EkfCore::fuse_direct_state_observation(int state_index, ftype innovation, ftype obs_variance, ftype dt_ekf_avg) {
    const auto si = static_cast<std::size_t>(state_index);
    const int lim = state_index_lim();

    // upstream: `varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] +
    // R_OBS[obsIndex]; SK = 1.0f/varInnovVelPos[obsIndex];` (~line
    // 1029-1030).
    const ftype var_innov = P[si][si] + obs_variance;
    const ftype sk = ftype(1) / var_innov;

    // upstream: `for (i=0;i<=9;i++) Kfusion[i] = P[i][stateIndex]*SK;`
    // (~line 1031-1033) - always active, no inhibit flag gates the
    // quaternion/velocity/position block.
    std::array<ftype, 24> kfusion{};
    for (int i = 0; i <= 9; ++i) {
        const auto ii = static_cast<std::size_t>(i);
        kfusion[ii] = P[ii][si] * sk;
    }

    // upstream: `if (!inhibitDelAngBiasStates) { for(i=10;i<=12;i++)
    // Kfusion[i]=P[i][stateIndex]*SK; }` (~line 1036-1067), WITHOUT the
    // nested `PV_AidingMode == AID_NONE` poorObservability narrowing -
    // see hpp banner for why that narrowing does not apply here.
    if (!inhibit_del_ang_bias_states) {
        for (int i = 10; i <= 12; ++i) {
            const auto ii = static_cast<std::size_t>(i);
            kfusion[ii] = P[ii][si] * sk;
        }
    }

    // upstream: `if (!horizInhibit && !inhibitDelVelBiasStates &&
    // !badIMUdata) { for(i=13;i<=15;i++) if(!dvelBiasAxisInhibit[i-13])
    // Kfusion[i]=P[i][stateIndex]*SK; }` (~line 1073-1084) - horizInhibit
    // is PV_AidingMode==AID_NONE-only (does not apply here), badIMUdata
    // is permanently false (phase-1 simplification 3), and
    // dvelBiasAxisInhibit[] is an already-named phase-1 gap (no
    // ground-alignment axis narrowing modeled) - so this reduces to the
    // single inhibit_del_vel_bias_states gate.
    if (!inhibit_del_vel_bias_states) {
        for (int i = 13; i <= 15; ++i) {
            const auto ii = static_cast<std::size_t>(i);
            kfusion[ii] = P[ii][si] * sk;
        }
    }

    // upstream: `if (!inhibitMagStates) { for(i=16;i<=21;i++)
    // Kfusion[i]=P[i][stateIndex]*SK; }` (~line 1087-1092).
    if (!inhibit_mag_states) {
        for (int i = 16; i <= 21; ++i) {
            const auto ii = static_cast<std::size_t>(i);
            kfusion[ii] = P[ii][si] * sk;
        }
    }

    // upstream: `if (!inhibitWindStates && !treatWindStatesAsTruth) {
    // Kfusion[22]=P[22][stateIndex]*SK; Kfusion[23]=P[23][stateIndex]*SK;
    // }` (~line 1095-1100) - treatWindStatesAsTruth has no equivalent in
    // this port (see hpp banner); moot since inhibit_wind_states is
    // permanently true in this phase regardless.
    if (!inhibit_wind_states) {
        kfusion[22] = P[22][si] * sk;
        kfusion[23] = P[23][si] * sk;
    }

    // upstream: `for(i=0;i<=stateIndexLim;i++) for(j=0;j<=stateIndexLim;j++)
    // KHP[i][j] = Kfusion[i]*P[stateIndex][j];` (~line 1103-1108), then
    // the healthyFusion negative-variance guard (~line 1109-1115) - the
    // ONLY fusion-health check in this phase's scope (see hpp banner: no
    // innovation-consistency gating).
    Matrix24 khp{};
    for (int i = 0; i <= lim; ++i) {
        const auto ii = static_cast<std::size_t>(i);
        for (int j = 0; j <= lim; ++j) {
            const auto jj = static_cast<std::size_t>(j);
            khp[ii][jj] = kfusion[ii] * P[si][jj];
        }
    }
    for (int i = 0; i <= lim; ++i) {
        const auto ii = static_cast<std::size_t>(i);
        if (khp[ii][ii] > P[ii][ii]) {
            // upstream: `healthyFusion = false;` -> skip the update
            // entirely, P and state are left untouched (~line 1163's
            // else branch, minus the faultStatus bookkeeping this port
            // doesn't model).
            return false;
        }
    }

    // upstream: `P[i][j] = P[i][j] - KHP[i][j];` (~line 1116-1121), then
    // ForceSymmetry() + ConstrainVariances() (~line 1123-1124).
    for (int i = 0; i <= lim; ++i) {
        const auto ii = static_cast<std::size_t>(i);
        for (int j = 0; j <= lim; ++j) {
            const auto jj = static_cast<std::size_t>(j);
            P[ii][jj] -= khp[ii][jj];
        }
    }
    force_symmetry(lim);
    constrain_variances(dt_ekf_avg);

    // upstream: `for(i=0;i<=stateIndexLim;i++) statesArray[i] -=
    // Kfusion[i]*innovVelPos[obsIndex]; stateStruct.quat.normalize();`
    // (~line 1127-1131) - see apply_state_correction()'s own comment for
    // why this port applies it across all 24 conceptual slots rather
    // than bounding at `lim`.
    apply_state_correction(kfusion, innovation);
    return true;
}

// upstream: FuseVelPosNED()'s R_OBS[0]/[1] "no reported accuracy" branch,
// AP_NavEKF3_PosVelFusion.cpp ~line 740-741: `R_OBS[0] =
// sq(constrain_ftype(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) +
// sq(frontend->gpsNEVelVarAccScale * accNavMag); R_OBS[1] = R_OBS[0];`
ftype EkfCore::gps_horiz_vel_obs_variance() const {
    const ftype acc_nav_mag = vel_dot_ned_filt.length();
    return sq(clamp(gps_horiz_vel_noise, ftype(0.05), ftype(5.0))) + sq(kGpsNeVelVarAccScale * acc_nav_mag);
}

// upstream: R_OBS[2], ~line 742: `R_OBS[2] =
// sq(constrain_ftype(frontend->_gpsVertVelNoise, 0.05f, 5.0f)) +
// sq(frontend->gpsDVelVarAccScale * accNavMag);`
ftype EkfCore::gps_vert_vel_obs_variance() const {
    const ftype acc_nav_mag = vel_dot_ned_filt.length();
    return sq(clamp(gps_vert_vel_noise, ftype(0.05), ftype(5.0))) + sq(kGpsDVelVarAccScale * acc_nav_mag);
}

// upstream: R_OBS[3]/[4] "no reported accuracy" branch, ~line 754-757:
// `const ftype posErr = frontend->gpsPosVarAccScale * accNavMag; R_OBS[3]
// = sq(constrain_ftype(frontend->_gpsHorizPosNoise, 0.1f, 10.0f)) +
// sq(posErr); R_OBS[4] = R_OBS[3];`
ftype EkfCore::gps_horiz_pos_obs_variance() const {
    const ftype acc_nav_mag = vel_dot_ned_filt.length();
    const ftype pos_err = kGpsPosVarAccScale * acc_nav_mag;
    return sq(clamp(gps_horiz_pos_noise, ftype(0.1), ftype(10.0))) + sq(pos_err);
}

// CPP-057 phase 3. upstream: velTestRatio, AP_NavEKF3_PosVelFusion.cpp
// ~line 875-901 - `velTestRatio = innovVelSumSq / (varVelSum *
// sq(MAX(0.01*_gpsVelInnovGate, 1.0)))`, summed over N/E/D (this port's
// imax==2 case unconditionally - see ekf_core.hpp's "CPP-057, PHASE 3"
// banner for why that's the real AID_ABSOLUTE-equivalent behavior here,
// not an approximation). R_OBS_DATA_CHECKS[0..2] == R_OBS[0..2] for this
// port exactly (see banner's "CORRECTION TO NOTHING" note) - reuses
// gps_horiz_vel_obs_variance()/gps_vert_vel_obs_variance() directly
// rather than a second, duplicate formula.
ftype EkfCore::gps_vel_test_ratio(const GpsSample& gps) const {
    const ftype r_obs_horiz = gps_horiz_vel_obs_variance();
    const ftype r_obs_vert = gps_vert_vel_obs_variance();

    const ftype innov_n = state.velocity.x - gps.velocity_ned.x;
    const ftype innov_e = state.velocity.y - gps.velocity_ned.y;
    const ftype innov_d = state.velocity.z - gps.velocity_ned.z;
    const ftype innov_sum_sq = sq(innov_n) + sq(innov_e) + sq(innov_d);

    const ftype var_sum = (P[4][4] + r_obs_horiz) + (P[5][5] + r_obs_horiz) + (P[6][6] + r_obs_vert);
    const ftype gate = std::max(ftype(0.01) * gps_vel_innov_gate_pct, ftype(1.0));

    return innov_sum_sq / (var_sum * sq(gate));
}

// CPP-057 phase 3. upstream: posTestRatio, ~line 806-816 -
// `posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) /
// (sq(MAX(0.01*_gpsPosInnovGate, 1.0)) * (varInnovVelPos[3] +
// varInnovVelPos[4]))`. R_OBS_DATA_CHECKS[3]/[4] == R_OBS[3]/[4]
// unconditionally upstream (~line 774) - reuses
// gps_horiz_pos_obs_variance() directly, no duplicate formula.
ftype EkfCore::gps_pos_test_ratio(const GpsSample& gps) const {
    const ftype r_obs = gps_horiz_pos_obs_variance();

    const ftype innov_n = state.position.x - gps.position_ne.x;
    const ftype innov_e = state.position.y - gps.position_ne.y;
    const ftype innov_sum_sq = sq(innov_n) + sq(innov_e);

    const ftype var_sum = (P[7][7] + r_obs) + (P[8][8] + r_obs);
    const ftype gate = std::max(ftype(0.01) * gps_pos_innov_gate_pct, ftype(1.0));

    return innov_sum_sq / (var_sum * sq(gate));
}

// CPP-058 phase 4. upstream: NavEKF3_core::ResetVelocity(resetDataSource),
// AP_NavEKF3_PosVelFusion.cpp lines 14-89 - reduced to the state+
// covariance portion of the real AID_ABSOLUTE/GPS-source branch (see
// ekf_core.hpp's "CPP-058, PHASE 4" banner for the full scope reduction
// and what's excluded). Directly overwrites state.velocity.x/y from the
// GpsSample (upstream: `stateStruct.velocity.x  = gps_corrected.vel.x;
// stateStruct.velocity.y  = gps_corrected.vel.y;`, ~line 54-55, minus the
// antenna-offset correction - GpsSample is already-corrected per
// CPP-056's convention) and re-seeds P[4][4]/P[5][5] (upstream: `P[5][5]
// = P[4][4] = sq(MAX(frontend->_gpsHorizVelNoise,gpsSpdAccuracy));`,
// ~line 57 - degenerates to sq(gps_horiz_vel_noise) once the
// gpsSpdAccuracy reported-accuracy term is excluded, per CPP-056's own
// established exclusion of that branch, verified directly by reading the
// real MAX() call, not assumed). Stamps last_vel_pass_time_s = now_s,
// matching upstream's own `lastVelPassTime_ms = imuSampleTime_ms;` at
// ResetVelocity()'s end (~line 74) - a reset counts as "just passed", so
// the timeout clock restarts from here.
void EkfCore::reset_velocity(const GpsSample& gps, ftype now_s) {
    zero_rows_cols(P, 4, 5);  // upstream: zeroRows(P,4,5); zeroCols(P,4,5);
    state.velocity.x = gps.velocity_ned.x;
    state.velocity.y = gps.velocity_ned.y;
    P[4][4] = P[5][5] = sq(gps_horiz_vel_noise);
    last_vel_pass_time_s = now_s;
}

// CPP-058 phase 4. upstream: NavEKF3_core::ResetPosition(resetDataSource),
// AP_NavEKF3_PosVelFusion.cpp lines 94-186 - same reduction as
// reset_velocity() above, but see ekf_core.hpp's "CPP-058, PHASE 4"
// banner "CORRECTIONS/FINDINGS" #1 for a real divergence found and
// deliberately NOT followed here: upstream's actual timeout-triggered
// call site (FuseVelPosNED(), ~line 844-856) immediately re-overrides
// ResetPosition()'s own P[7][7]/P[8][8] value with
// sq(0.5*_gpsGlitchRadiusMax) right after calling it. This function
// intentionally reproduces ResetPosition() ITSELF in isolation (this
// ticket's literal, narrower scope, per its own "lines ~14-183"
// pointer), not that further caller-side override, which belongs to the
// still-excluded full "reset-on-timeout sequence" orchestration named
// since CPP-057. Overwrites state.position.x/y directly from the
// GpsSample's already-projected local-NE position (upstream:
// `stateStruct.position.xy() = EKF_origin.get_distance_NE_ftype
// (gpsloc);` plus a velocity*tdiff time-alignment correction, ~line
// 139-143 - both excluded, see banner) and re-seeds P[7][7]/P[8][8]
// (upstream: `P[7][7] = P[8][8] =
// sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));`, ~line 146 -
// degenerates to sq(gps_horiz_pos_noise) once gpsPosAccuracy is
// excluded, same reasoning as reset_velocity() above). Stamps
// last_pos_pass_time_s = now_s, matching upstream's own
// `lastGpsPosPassTime_ms = imuSampleTime_ms;` at ResetPosition()'s end
// (~line 184).
void EkfCore::reset_position(const GpsSample& gps, ftype now_s) {
    zero_rows_cols(P, 7, 8);  // upstream: zeroRows(P,7,8); zeroCols(P,7,8);
    state.position.x = gps.position_ne.x;
    state.position.y = gps.position_ne.y;
    P[7][7] = P[8][8] = sq(gps_horiz_pos_noise);
    last_pos_pass_time_s = now_s;
}

// upstream: FuseVelPosNED()'s obsIndex 0-2 path. Innovation formula
// (~line 1011): `innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] -
// velPosObs[obsIndex];` - recomputed fresh before each axis's fusion
// call so a correlated correction from an earlier axis in this same call
// is reflected, exactly matching upstream's sequential-fusion loop
// (state is not a snapshot taken once at the top of the loop).
//
// CPP-057 phase 3: gated by gps_vel_test_ratio(), computed ONCE here
// using the state/P as they stand at entry - matching upstream, where
// the test block (~line 875-901) runs once, before the separate per-axis
// sequential fusion loop it gates (~line 1005-1014). Failing the gate
// (ratio >= 1.0) skips the WHOLE velocity vector for this cycle: no axis
// is fused, P/state are left completely untouched, matching upstream's
// own `else { fuseVelData = false; }` (~line 928).
//
// CPP-058 phase 4: on a gate failure, checks the elapsed time since
// last_vel_pass_time_s against kGpsFusionTimeoutS (posRetryTimeUseVel_ms
// = 10.0s) BEFORE returning 0 - if timed out, calls reset_velocity()
// instead of simply skipping (upstream: `if (PV_AidingMode ==
// AID_ABSOLUTE && velTimeout) { ResetVelocity(...); fuseVelData = false;
// ...}`, ~line 917-924 - this port has no aiding-mode state machine, so
// AID_ABSOLUTE is unconditionally this port's only mode, see banner). A
// reset is NOT a fusion (matches upstream's own `fuseVelData = false`
// right after the reset call, ~line 921), so this still returns 0 either
// way on the failing branch. On a passing gate, last_vel_pass_time_s is
// only stamped when n_fused > 0 - see this file's ekf_core.hpp banner
// "CPP-058, PHASE 4" section "A REAL, NAMED DIVERGENCE" for why that is
// slightly stricter than upstream's own gate-pass-only condition.
int EkfCore::fuse_gps_velocity(const GpsSample& gps, ftype dt_ekf_avg, ftype now_s) {
    if (gps_vel_test_ratio(gps) >= ftype(1.0)) {
        if ((now_s - last_vel_pass_time_s) >= kGpsFusionTimeoutS) {
            reset_velocity(gps, now_s);
        }
        return 0;
    }

    int n_fused = 0;
    const ftype r_obs_horiz = gps_horiz_vel_obs_variance();
    const ftype r_obs_vert = gps_vert_vel_obs_variance();

    if (fuse_direct_state_observation(4, state.velocity.x - gps.velocity_ned.x, r_obs_horiz, dt_ekf_avg)) {
        ++n_fused;
    }
    if (fuse_direct_state_observation(5, state.velocity.y - gps.velocity_ned.y, r_obs_horiz, dt_ekf_avg)) {
        ++n_fused;
    }
    if (fuse_direct_state_observation(6, state.velocity.z - gps.velocity_ned.z, r_obs_vert, dt_ekf_avg)) {
        ++n_fused;
    }
    if (n_fused > 0) {
        last_vel_pass_time_s = now_s;
    }
    return n_fused;
}

// upstream: FuseVelPosNED()'s obsIndex 3-4 path. Innovation formula
// (~line 1016): `innovVelPos[obsIndex] = stateStruct.position[obsIndex-3]
// - velPosObs[obsIndex];`.
//
// CPP-057 phase 3: gated by gps_pos_test_ratio(), computed ONCE here
// using the state/P as they stand at entry, same reasoning as
// fuse_gps_velocity() above. Failing the gate (ratio >= 1.0) skips both
// position axes for this cycle, P/state left completely untouched,
// matching upstream's own `else { fusePosData = false; }` (~line 859).
//
// CPP-058 phase 4: same timeout/reset wiring as fuse_gps_velocity()
// above, using reset_position()/last_pos_pass_time_s instead - see that
// function's doc comment for the full detail. Upstream's real
// timeout-triggered call site for position (~line 845-865) additionally
// requires `(!velAiding || gpsGoodToAlign)` and offers a second,
// independent `posVarianceIsTooLarge` trigger - neither is modeled here,
// named explicitly in this file's ekf_core.hpp "CPP-058, PHASE 4" banner
// (CORRECTIONS/FINDINGS #2): this port's reset fires on elapsed-time
// timeout alone.
int EkfCore::fuse_gps_position(const GpsSample& gps, ftype dt_ekf_avg, ftype now_s) {
    if (gps_pos_test_ratio(gps) >= ftype(1.0)) {
        if ((now_s - last_pos_pass_time_s) >= kGpsFusionTimeoutS) {
            reset_position(gps, now_s);
        }
        return 0;
    }

    int n_fused = 0;
    const ftype r_obs = gps_horiz_pos_obs_variance();

    if (fuse_direct_state_observation(7, state.position.x - gps.position_ne.x, r_obs, dt_ekf_avg)) {
        ++n_fused;
    }
    if (fuse_direct_state_observation(8, state.position.y - gps.position_ne.y, r_obs, dt_ekf_avg)) {
        ++n_fused;
    }
    if (n_fused > 0) {
        last_pos_pass_time_s = now_s;
    }
    return n_fused;
}

// CPP-067 phase 13. upstream: NavEKF3_core::readGpsData()'s
// `storedGPS.push(gpsDataNew);` (AP_NavEKF3_Measurements.cpp ~line 733).
// See ekf_core.hpp's "CPP-067, PHASE 13" banner and push_gps_sample()'s
// own doc comment for the full scope discussion - a thin pass-through,
// all the real behavior lives in ObsBuffer::push() (ekf_buffer.hpp).
void EkfCore::push_gps_sample(const GpsSample& sample) {
    gps_buffer.push(sample);
}

// CPP-067 phase 13. upstream: NavEKF3_core::SelectVelPosFusion()'s
// `gpsDataToFuse = storedGPS.recall(gpsDataDelayed,
// imuDataDelayed.time_ms) && !waitingForGpsChecks;` (AP_NavEKF3_
// PosVelFusion.cpp ~line 534) - see ekf_core.hpp's "CPP-067, PHASE 13"
// banner and recall_gps_sample()'s own doc comment for the full
// now_s-vs-imuDataDelayed.time_ms discussion and why this is ONE combined
// recall primitive rather than two independently-recalling functions.
//
// `now_s` is converted to milliseconds the same way GpsSample::
// set_time_s() converts a timestamp INTO the buffer (clamp negative to
// zero before the cast, avoiding an implementation-defined/UB-adjacent
// negative-float-to-uint32_t conversion - `now_s` is never legitimately
// negative in this port's own elapsed-simulated-time convention, so this
// is a defensive clamp, not a real case this port expects to hit) -
// ObsBuffer::recall()'s own 100ms window and dt arithmetic
// (ekf_buffer.hpp) then decide the match, exactly as it would for
// upstream's real millisecond-resolution imuDataDelayed.time_ms query.
bool EkfCore::recall_gps_sample(GpsSample& out, ftype now_s) {
    const ftype clamped_now_s = now_s > ftype(0) ? now_s : ftype(0);
    const std::uint32_t now_ms = static_cast<std::uint32_t>(clamped_now_s * ftype(1000));
    return gps_buffer.recall(out, now_ms);
}

// ============================================================================
// CPP-059 PHASE 5: 3-axis magnetometer fusion. See ekf_core.hpp's
// "CPP-059, PHASE 5" banner for the full scope/exclusions/corrections
// discussion - only implementation-level transcription notes live here.
//
// VERBATIM TRANSCRIPTION NOTE (same deliberate exception as
// covariance_prediction()'s PS0..PS222 block, see this file's own top
// banner): SH_MAG/var_innov_mag/H_MAG/SK_MX/SK_MY/SK_MZ below are dense,
// auto-generated (Matlab symbolic toolbox, per upstream's own
// FuseMagnetometer() doc comment) Jacobian/Kalman-gain algebra, transcribed
// verbatim from AP_NavEKF3_MagFusion.cpp ~line 473-836. Names are kept as
// close to upstream as this port's member-vs-local split allows (SH_MAG,
// H_MAG, SK_MX/SK_MY/SK_MZ, magN/magE/magD/magXbias/magYbias/magZbias) for
// the same mechanical-diffability reason covariance_prediction() gives -
// NOT "cleaned up" into this port's usual naming style. Literal precision
// follows this file's own "LITERAL PRECISION NOTE" (top of file): every
// literal is written as an explicit ftype(...) rather than reproducing
// upstream's bare 2.0f-style literals - bit-identical to upstream for
// every literal in this block (all are exactly representable in IEEE-754
// float: 1, 2).
//
// One implementation-only (non-behavioral) simplification versus the
// unrolled upstream source: upstream declares H_MAG once outside the
// obsIndex loop and explicitly zeros indices 0..stateIndexLim at the top
// of each axis's branch before setting the ones it needs (~line 591,
// 651, 713). This port instead value-initializes a fresh
// std::array<ftype,24> H_MAG{} inside the loop body each iteration -
// provably identical, since every index upstream's code ever READS
// (0,1,2,3,16,17,18, and the per-axis H_MAG_unit_index) is unconditionally
// WRITTEN in every branch below, including the explicit `= 0.0f`/`=
// ftype(0)` assignments upstream makes for the two mag-bias slots that
// don't apply to the current axis (e.g. H_MAG[20]/H_MAG[21] = 0 in the X
// branch) - so which indices happen to hold stale values from a previous
// iteration is never observable either way.

// CPP-060 phase 6. upstream: magTestRatio[i] = sq(innovMag[i]) /
// (sq(MAX(0.01f*(ftype)frontend->_magInnovGate, 1.0f)) * varInnovMag[i]),
// AP_NavEKF3_MagFusion.cpp ~line 571-573 - reads this object's own
// stored innov_mag/var_innov_mag (populated by fuse_magnetometer() itself
// before this is ever called, see that function's body) rather than
// recomputing anything from a fresh sample. See ekf_core.hpp's
// "CPP-060, PHASE 6" banner for the full derivation and the real
// MAG_I_GATE_DEFAULT=300 default.
Vector3F EkfCore::mag_test_ratio() const {
    const ftype gate = std::max(ftype(0.01) * mag_innov_gate_pct, ftype(1.0));
    const ftype gate_sq = sq(gate);
    return Vector3F(sq(innov_mag.x) / (gate_sq * var_innov_mag.x),
                    sq(innov_mag.y) / (gate_sq * var_innov_mag.y),
                    sq(innov_mag.z) / (gate_sq * var_innov_mag.z));
}

bool EkfCore::fuse_magnetometer(const MagSample& mag, const GyroSample& gyro, ftype dt_ekf_avg) {
    // create aliases for state to make code easier to read (upstream:
    // identical aliases, ~line 481-490).
    const ftype q0 = state.quat[0];
    const ftype q1 = state.quat[1];
    const ftype q2 = state.quat[2];
    const ftype q3 = state.quat[3];
    const ftype magN = state.earth_magfield.x;
    const ftype magE = state.earth_magfield.y;
    const ftype magD = state.earth_magfield.z;
    const ftype magXbias = state.body_magfield.x;
    const ftype magYbias = state.body_magfield.y;
    const ftype magZbias = state.body_magfield.z;

    // rotate predicted earth components into body axes and calculate
    // predicted measurements (upstream ~line 492-511 - verified this
    // round to build the DCM in the same form as any other DCM
    // construction in this port, per the ticket's own instruction; it is
    // NOT bit-for-bit the same expression as, e.g., Matrix3::
    // from_quaternion() would produce because upstream hand-expands it
    // here rather than calling a shared helper - transcribed exactly as
    // upstream writes it, not substituted for an existing DCM helper).
    const Matrix3F DCM(q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, ftype(2) * (q1 * q2 + q0 * q3),
                        ftype(2) * (q1 * q3 - q0 * q2), ftype(2) * (q1 * q2 - q0 * q3),
                        q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, ftype(2) * (q2 * q3 + q0 * q1),
                        ftype(2) * (q1 * q3 + q0 * q2), ftype(2) * (q2 * q3 - q0 * q1),
                        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

    const Vector3F mag_pred(DCM[0][0] * magN + DCM[0][1] * magE + DCM[0][2] * magD + magXbias,
                             DCM[1][0] * magN + DCM[1][1] * magE + DCM[1][2] * magD + magYbias,
                             DCM[2][0] * magN + DCM[2][1] * magE + DCM[2][2] * magD + magZbias);

    // upstream: `innovMag = MagPred - magDataDelayed.mag;` (~line 514).
    innov_mag = mag_pred - mag.mag;

    // scale magnetometer observation error with total angular rate to
    // allow for timing errors (upstream ~line 517, `frontend->_magNoise`/
    // `frontend->magVarRateScale` - this port's mag_noise field/
    // kMagVarRateScale constant, see ekf_core.hpp banner and this file's
    // anonymous namespace above).
    const ftype R_MAG = sq(clamp(mag_noise, ftype(0.01), ftype(0.5))) +
                         sq(kMagVarRateScale * gyro.delta_angle.length() / gyro.delta_angle_dt);

    // calculate common expressions used to calculate observation jacobians
    // and innovation variance for each component (upstream ~line 520-529).
    const std::array<ftype, 9> SH_MAG{
        ftype(2) * magD * q3 + ftype(2) * magE * q2 + ftype(2) * magN * q1,
        ftype(2) * magD * q0 - ftype(2) * magE * q1 + ftype(2) * magN * q2,
        ftype(2) * magD * q1 + ftype(2) * magE * q0 - ftype(2) * magN * q3,
        sq(q3),
        sq(q2),
        sq(q1),
        sq(q0),
        ftype(2) * magN * q0,
        ftype(2) * magE * q3,
    };

    // Calculate the innovation variance for each axis (upstream ~line
    // 532-546 X, 548-558 Y, 560-570 Z) - verbatim transcription, see this
    // function's own banner. Each axis's "badly conditioned" check aborts
    // the WHOLE fusion call immediately (upstream: CovarianceInit() then
    // an early `return;`) - see ekf_core.hpp banner for why this is a
    // real, distinctive divergence from GPS fusion's simple per-axis skip.
    var_innov_mag.x = (P[19][19] + R_MAG + P[1][19]*SH_MAG[0] - P[2][19]*SH_MAG[1] + P[3][19]*SH_MAG[2] - P[16][19]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + (ftype(2)*q0*q3 + ftype(2)*q1*q2)*(P[19][17] + P[1][17]*SH_MAG[0] - P[2][17]*SH_MAG[1] + P[3][17]*SH_MAG[2] - P[16][17]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][17]*(ftype(2)*q0*q3 + ftype(2)*q1*q2) - P[18][17]*(ftype(2)*q0*q2 - ftype(2)*q1*q3) + P[0][17]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) - (ftype(2)*q0*q2 - ftype(2)*q1*q3)*(P[19][18] + P[1][18]*SH_MAG[0] - P[2][18]*SH_MAG[1] + P[3][18]*SH_MAG[2] - P[16][18]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][18]*(ftype(2)*q0*q3 + ftype(2)*q1*q2) - P[18][18]*(ftype(2)*q0*q2 - ftype(2)*q1*q3) + P[0][18]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)*(P[19][0] + P[1][0]*SH_MAG[0] - P[2][0]*SH_MAG[1] + P[3][0]*SH_MAG[2] - P[16][0]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][0]*(ftype(2)*q0*q3 + ftype(2)*q1*q2) - P[18][0]*(ftype(2)*q0*q2 - ftype(2)*q1*q3) + P[0][0]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + P[17][19]*(ftype(2)*q0*q3 + ftype(2)*q1*q2) - P[18][19]*(ftype(2)*q0*q2 - ftype(2)*q1*q3) + SH_MAG[0]*(P[19][1] + P[1][1]*SH_MAG[0] - P[2][1]*SH_MAG[1] + P[3][1]*SH_MAG[2] - P[16][1]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][1]*(ftype(2)*q0*q3 + ftype(2)*q1*q2) - P[18][1]*(ftype(2)*q0*q2 - ftype(2)*q1*q3) + P[0][1]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) - SH_MAG[1]*(P[19][2] + P[1][2]*SH_MAG[0] - P[2][2]*SH_MAG[1] + P[3][2]*SH_MAG[2] - P[16][2]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][2]*(ftype(2)*q0*q3 + ftype(2)*q1*q2) - P[18][2]*(ftype(2)*q0*q2 - ftype(2)*q1*q3) + P[0][2]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + SH_MAG[2]*(P[19][3] + P[1][3]*SH_MAG[0] - P[2][3]*SH_MAG[1] + P[3][3]*SH_MAG[2] - P[16][3]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][3]*(ftype(2)*q0*q3 + ftype(2)*q1*q2) - P[18][3]*(ftype(2)*q0*q2 - ftype(2)*q1*q3) + P[0][3]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P[19][16] + P[1][16]*SH_MAG[0] - P[2][16]*SH_MAG[1] + P[3][16]*SH_MAG[2] - P[16][16]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[17][16]*(ftype(2)*q0*q3 + ftype(2)*q1*q2) - P[18][16]*(ftype(2)*q0*q2 - ftype(2)*q1*q3) + P[0][16]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + P[0][19]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2));
    if (var_innov_mag.x < R_MAG) {
        // upstream: "the calculation is badly conditioned, so we cannot
        // perform fusion on this step - we reset the covariance matrix
        // and try again next measurement" (~line 542-546).
        covariance_init(dt_ekf_avg);
        return false;
    }

    var_innov_mag.y = (P[20][20] + R_MAG + P[0][20]*SH_MAG[2] + P[1][20]*SH_MAG[1] + P[2][20]*SH_MAG[0] - P[17][20]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (ftype(2)*q0*q3 - ftype(2)*q1*q2)*(P[20][16] + P[0][16]*SH_MAG[2] + P[1][16]*SH_MAG[1] + P[2][16]*SH_MAG[0] - P[17][16]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][16]*(ftype(2)*q0*q3 - ftype(2)*q1*q2) + P[18][16]*(ftype(2)*q0*q1 + ftype(2)*q2*q3) - P[3][16]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + (ftype(2)*q0*q1 + ftype(2)*q2*q3)*(P[20][18] + P[0][18]*SH_MAG[2] + P[1][18]*SH_MAG[1] + P[2][18]*SH_MAG[0] - P[17][18]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][18]*(ftype(2)*q0*q3 - ftype(2)*q1*q2) + P[18][18]*(ftype(2)*q0*q1 + ftype(2)*q2*q3) - P[3][18]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)*(P[20][3] + P[0][3]*SH_MAG[2] + P[1][3]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[17][3]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][3]*(ftype(2)*q0*q3 - ftype(2)*q1*q2) + P[18][3]*(ftype(2)*q0*q1 + ftype(2)*q2*q3) - P[3][3]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) - P[16][20]*(ftype(2)*q0*q3 - ftype(2)*q1*q2) + P[18][20]*(ftype(2)*q0*q1 + ftype(2)*q2*q3) + SH_MAG[2]*(P[20][0] + P[0][0]*SH_MAG[2] + P[1][0]*SH_MAG[1] + P[2][0]*SH_MAG[0] - P[17][0]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][0]*(ftype(2)*q0*q3 - ftype(2)*q1*q2) + P[18][0]*(ftype(2)*q0*q1 + ftype(2)*q2*q3) - P[3][0]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + SH_MAG[1]*(P[20][1] + P[0][1]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[2][1]*SH_MAG[0] - P[17][1]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][1]*(ftype(2)*q0*q3 - ftype(2)*q1*q2) + P[18][1]*(ftype(2)*q0*q1 + ftype(2)*q2*q3) - P[3][1]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + SH_MAG[0]*(P[20][2] + P[0][2]*SH_MAG[2] + P[1][2]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[17][2]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][2]*(ftype(2)*q0*q3 - ftype(2)*q1*q2) + P[18][2]*(ftype(2)*q0*q1 + ftype(2)*q2*q3) - P[3][2]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P[20][17] + P[0][17]*SH_MAG[2] + P[1][17]*SH_MAG[1] + P[2][17]*SH_MAG[0] - P[17][17]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[16][17]*(ftype(2)*q0*q3 - ftype(2)*q1*q2) + P[18][17]*(ftype(2)*q0*q1 + ftype(2)*q2*q3) - P[3][17]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) - P[3][20]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2));
    if (var_innov_mag.y < R_MAG) {
        covariance_init(dt_ekf_avg);
        return false;
    }

    var_innov_mag.z = (P[21][21] + R_MAG + P[0][21]*SH_MAG[1] - P[1][21]*SH_MAG[2] + P[3][21]*SH_MAG[0] + P[18][21]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + (ftype(2)*q0*q2 + ftype(2)*q1*q3)*(P[21][16] + P[0][16]*SH_MAG[1] - P[1][16]*SH_MAG[2] + P[3][16]*SH_MAG[0] + P[18][16]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][16]*(ftype(2)*q0*q2 + ftype(2)*q1*q3) - P[17][16]*(ftype(2)*q0*q1 - ftype(2)*q2*q3) + P[2][16]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) - (ftype(2)*q0*q1 - ftype(2)*q2*q3)*(P[21][17] + P[0][17]*SH_MAG[1] - P[1][17]*SH_MAG[2] + P[3][17]*SH_MAG[0] + P[18][17]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][17]*(ftype(2)*q0*q2 + ftype(2)*q1*q3) - P[17][17]*(ftype(2)*q0*q1 - ftype(2)*q2*q3) + P[2][17]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)*(P[21][2] + P[0][2]*SH_MAG[1] - P[1][2]*SH_MAG[2] + P[3][2]*SH_MAG[0] + P[18][2]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][2]*(ftype(2)*q0*q2 + ftype(2)*q1*q3) - P[17][2]*(ftype(2)*q0*q1 - ftype(2)*q2*q3) + P[2][2]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + P[16][21]*(ftype(2)*q0*q2 + ftype(2)*q1*q3) - P[17][21]*(ftype(2)*q0*q1 - ftype(2)*q2*q3) + SH_MAG[1]*(P[21][0] + P[0][0]*SH_MAG[1] - P[1][0]*SH_MAG[2] + P[3][0]*SH_MAG[0] + P[18][0]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][0]*(ftype(2)*q0*q2 + ftype(2)*q1*q3) - P[17][0]*(ftype(2)*q0*q1 - ftype(2)*q2*q3) + P[2][0]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) - SH_MAG[2]*(P[21][1] + P[0][1]*SH_MAG[1] - P[1][1]*SH_MAG[2] + P[3][1]*SH_MAG[0] + P[18][1]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][1]*(ftype(2)*q0*q2 + ftype(2)*q1*q3) - P[17][1]*(ftype(2)*q0*q1 - ftype(2)*q2*q3) + P[2][1]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + SH_MAG[0]*(P[21][3] + P[0][3]*SH_MAG[1] - P[1][3]*SH_MAG[2] + P[3][3]*SH_MAG[0] + P[18][3]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][3]*(ftype(2)*q0*q2 + ftype(2)*q1*q3) - P[17][3]*(ftype(2)*q0*q1 - ftype(2)*q2*q3) + P[2][3]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P[21][18] + P[0][18]*SH_MAG[1] - P[1][18]*SH_MAG[2] + P[3][18]*SH_MAG[0] + P[18][18]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[16][18]*(ftype(2)*q0*q2 + ftype(2)*q1*q3) - P[17][18]*(ftype(2)*q0*q1 - ftype(2)*q2*q3) + P[2][18]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2)) + P[2][21]*(SH_MAG[7] + SH_MAG[8] - ftype(2)*magD*q2));
    if (var_innov_mag.z < R_MAG) {
        covariance_init(dt_ekf_avg);
        return false;
    }

    // CPP-060 phase 6: the real per-axis magTestRatio/magHealth gate
    // upstream computes right here (~line 571-582) - see ekf_core.hpp's
    // "CPP-060, PHASE 6" banner for the full derivation, the real
    // MAG_I_GATE_DEFAULT=300 default, and why a gate failure below is a
    // bare `return false;` with NO covariance_init() call - a THIRD,
    // distinct outcome from the two covariance-reset abort paths above
    // (badly-conditioned axis; healthyFusion guard failure later in this
    // function) - unlike either of those, this leaves state/P completely
    // untouched.
    const Vector3F test_ratio = mag_test_ratio();
    const bool mag_health = (test_ratio.x < ftype(1.0)) && (test_ratio.y < ftype(1.0)) &&
                             (test_ratio.z < ftype(1.0));
    if (!mag_health) {
        // upstream ~line 579-582: `if (!magHealth) { return; }` - a bare
        // return, no CovarianceInit(). Skip the ENTIRE 3-axis fusion
        // call for this cycle; state/P are left exactly as they were at
        // entry to this function.
        return false;
    }

    const int lim = state_index_lim();

    for (int obs_index = 0; obs_index <= 2; ++obs_index) {
        std::array<ftype, 24> H_MAG{};  // see this function's own banner for why value-init replaces upstream's explicit per-axis zeroing
        int H_MAG_unit_index = 0;
        std::array<ftype, 24> kfusion{};
        ftype innovation = 0;

        if (obs_index == 0) {
            // upstream ~line 591-644.
            H_MAG[0] = SH_MAG[7] + SH_MAG[8] - ftype(2) * magD * q2;
            H_MAG[1] = SH_MAG[0];
            H_MAG[2] = -SH_MAG[1];
            H_MAG[3] = SH_MAG[2];
            H_MAG[16] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
            H_MAG[17] = ftype(2) * q0 * q3 + ftype(2) * q1 * q2;
            H_MAG[18] = ftype(2) * q1 * q3 - ftype(2) * q0 * q2;
            H_MAG[19] = ftype(1);
            H_MAG_unit_index = 19;
            innovation = innov_mag.x;

            const std::array<ftype, 5> SK_MX{
                ftype(1) / var_innov_mag.x,
                SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6],
                SH_MAG[7] + SH_MAG[8] - ftype(2) * magD * q2,
                ftype(2) * q0 * q2 - ftype(2) * q1 * q3,
                ftype(2) * q0 * q3 + ftype(2) * q1 * q2,
            };

            // upstream: kalman_mask construction (~line 613-631) - see
            // ekf_core.hpp banner for the dvelBiasAxisInhibit[] exclusion
            // (this port sets bits 13-15 together, gated by the single
            // existing inhibit_del_vel_bias_states flag).
            std::uint32_t kalman_mask = (1u << 10) - 1;
            if (!inhibit_del_ang_bias_states) {
                kalman_mask |= (1u << 10) | (1u << 11) | (1u << 12);
            }
            if (!inhibit_del_vel_bias_states) {
                kalman_mask |= (1u << 13) | (1u << 14) | (1u << 15);
            }
            if (!inhibit_mag_states) {
                kalman_mask |= (1u << 16) | (1u << 17) | (1u << 18) | (1u << 19) | (1u << 20) | (1u << 21);
            }
            if (!inhibit_wind_states) {
                kalman_mask |= (1u << 22) | (1u << 23);
            }

            // upstream ~line 637-642 - NOT bounded at stateIndexLim, see
            // this file's ekf_core.hpp banner for why that's exact, not a
            // divergence.
            for (int i = 0; i < 24; ++i) {
                if ((kalman_mask & (1u << i)) == 0) {
                    continue;
                }
                const auto ii = static_cast<std::size_t>(i);
                kfusion[ii] = SK_MX[0] * (P[ii][19] + P[ii][1] * SH_MAG[0] - P[ii][2] * SH_MAG[1] +
                                           P[ii][3] * SH_MAG[2] + P[ii][0] * SK_MX[2] - P[ii][16] * SK_MX[1] +
                                           P[ii][17] * SK_MX[4] - P[ii][18] * SK_MX[3]);
            }
        } else if (obs_index == 1) {
            // upstream ~line 652-705.
            H_MAG[0] = SH_MAG[2];
            H_MAG[1] = SH_MAG[1];
            H_MAG[2] = SH_MAG[0];
            H_MAG[3] = ftype(2) * magD * q2 - SH_MAG[8] - SH_MAG[7];
            H_MAG[16] = ftype(2) * q1 * q2 - ftype(2) * q0 * q3;
            H_MAG[17] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
            H_MAG[18] = ftype(2) * q0 * q1 + ftype(2) * q2 * q3;
            H_MAG[20] = ftype(1);
            H_MAG_unit_index = 20;
            innovation = innov_mag.y;

            const std::array<ftype, 5> SK_MY{
                ftype(1) / var_innov_mag.y,
                SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6],
                SH_MAG[7] + SH_MAG[8] - ftype(2) * magD * q2,
                ftype(2) * q0 * q3 - ftype(2) * q1 * q2,
                ftype(2) * q0 * q1 + ftype(2) * q2 * q3,
            };

            std::uint32_t kalman_mask = (1u << 10) - 1;
            if (!inhibit_del_ang_bias_states) {
                kalman_mask |= (1u << 10) | (1u << 11) | (1u << 12);
            }
            if (!inhibit_del_vel_bias_states) {
                kalman_mask |= (1u << 13) | (1u << 14) | (1u << 15);
            }
            if (!inhibit_mag_states) {
                kalman_mask |= (1u << 16) | (1u << 17) | (1u << 18) | (1u << 19) | (1u << 20) | (1u << 21);
            }
            if (!inhibit_wind_states) {
                kalman_mask |= (1u << 22) | (1u << 23);
            }

            // upstream ~line 698-703.
            for (int i = 0; i < 24; ++i) {
                if ((kalman_mask & (1u << i)) == 0) {
                    continue;
                }
                const auto ii = static_cast<std::size_t>(i);
                kfusion[ii] = SK_MY[0] * (P[ii][20] + P[ii][0] * SH_MAG[2] + P[ii][1] * SH_MAG[1] +
                                           P[ii][2] * SH_MAG[0] - P[ii][3] * SK_MY[2] - P[ii][17] * SK_MY[1] -
                                           P[ii][16] * SK_MY[3] + P[ii][18] * SK_MY[4]);
            }
        } else {
            // upstream ~line 714-765.
            H_MAG[0] = SH_MAG[1];
            H_MAG[1] = -SH_MAG[2];
            H_MAG[2] = SH_MAG[7] + SH_MAG[8] - ftype(2) * magD * q2;
            H_MAG[3] = SH_MAG[0];
            H_MAG[16] = ftype(2) * q0 * q2 + ftype(2) * q1 * q3;
            H_MAG[17] = ftype(2) * q2 * q3 - ftype(2) * q0 * q1;
            H_MAG[18] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            H_MAG[21] = ftype(1);
            H_MAG_unit_index = 21;
            innovation = innov_mag.z;

            const std::array<ftype, 5> SK_MZ{
                ftype(1) / var_innov_mag.z,
                SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6],
                SH_MAG[7] + SH_MAG[8] - ftype(2) * magD * q2,
                ftype(2) * q0 * q1 - ftype(2) * q2 * q3,
                ftype(2) * q0 * q2 + ftype(2) * q1 * q3,
            };

            std::uint32_t kalman_mask = (1u << 10) - 1;
            if (!inhibit_del_ang_bias_states) {
                kalman_mask |= (1u << 10) | (1u << 11) | (1u << 12);
            }
            if (!inhibit_del_vel_bias_states) {
                kalman_mask |= (1u << 13) | (1u << 14) | (1u << 15);
            }
            if (!inhibit_mag_states) {
                kalman_mask |= (1u << 16) | (1u << 17) | (1u << 18) | (1u << 19) | (1u << 20) | (1u << 21);
            }
            if (!inhibit_wind_states) {
                kalman_mask |= (1u << 22) | (1u << 23);
            }

            // upstream ~line 760-765.
            for (int i = 0; i < 24; ++i) {
                if ((kalman_mask & (1u << i)) == 0) {
                    continue;
                }
                const auto ii = static_cast<std::size_t>(i);
                kfusion[ii] = SK_MZ[0] * (P[ii][21] + P[ii][0] * SH_MAG[1] - P[ii][1] * SH_MAG[2] +
                                           P[ii][3] * SH_MAG[0] + P[ii][2] * SK_MZ[2] + P[ii][18] * SK_MZ[1] +
                                           P[ii][16] * SK_MZ[4] - P[ii][17] * SK_MZ[3]);
            }
        }

        // correct the covariance P = (I - K*H)*P = P - K*H*P, taking
        // advantage of H_MAG's known-zero elements (upstream ~line
        // 773-791) - shared across all 3 axes, matching upstream's own
        // shared post-if/else-if block exactly.
        Matrix24 khp{};
        for (int i = 0; i <= lim; ++i) {
            const auto ii = static_cast<std::size_t>(i);
            for (int j = 0; j <= lim; ++j) {
                const auto jj = static_cast<std::size_t>(j);
                ftype res = 0;
                res += (kfusion[ii] * H_MAG[0]) * P[0][jj];
                res += (kfusion[ii] * H_MAG[1]) * P[1][jj];
                res += (kfusion[ii] * H_MAG[2]) * P[2][jj];
                res += (kfusion[ii] * H_MAG[3]) * P[3][jj];
                res += (kfusion[ii] * H_MAG[16]) * P[16][jj];
                res += (kfusion[ii] * H_MAG[17]) * P[17][jj];
                res += (kfusion[ii] * H_MAG[18]) * P[18][jj];
                // one value in H is always 1, and the others not
                // mentioned here are zero, so we can skip that H product.
                res += kfusion[ii] * P[static_cast<std::size_t>(H_MAG_unit_index)][jj];
                khp[ii][jj] = res;
            }
        }

        // Check that we are not going to drive any variances negative and
        // skip the update if so (upstream ~line 793-798).
        bool healthy_fusion = true;
        for (int i = 0; i <= lim; ++i) {
            const auto ii = static_cast<std::size_t>(i);
            if (khp[ii][ii] > P[ii][ii]) {
                healthy_fusion = false;
            }
        }

        if (!healthy_fusion) {
            // upstream ~line 824-830: record the bad axis (faultStatus -
            // not modeled in this port, see banner), then CovarianceInit()
            // and an early return - the SAME real, distinctive
            // full-covariance-reset-and-abort behavior as the
            // badly-conditioned checks above, not GPS fusion's simple
            // per-axis skip.
            covariance_init(dt_ekf_avg);
            return false;
        }

        // update the covariance matrix (upstream ~line 800-801).
        for (int i = 0; i <= lim; ++i) {
            const auto ii = static_cast<std::size_t>(i);
            for (int j = 0; j <= lim; ++j) {
                const auto jj = static_cast<std::size_t>(j);
                P[ii][jj] -= khp[ii][jj];
            }
        }

        // force the covariance matrix to be symmetrical and limit the
        // variances to prevent ill-conditioning (upstream ~line 804-805).
        force_symmetry(lim);
        constrain_variances(dt_ekf_avg);

        // correct the state vector (upstream ~line 807-810,
        // `statesArray[j] -= Kfusion[j]*innovMag[obsIndex];
        // stateStruct.quat.normalize();` - see apply_state_correction()'s
        // own comment for why this port applies it across all 24
        // conceptual slots rather than bounding at `lim`, same reasoning
        // as fuse_direct_state_observation() above).
        //
        // EXCLUDED: `if (have_table_earth_field && frontend->
        // _mag_ef_limit > 0) MagTableConstrain();` (upstream ~line
        // 812-815) - see ekf_core.hpp banner, no WMM table in this port.
        apply_state_correction(kfusion, innovation);
    }

    return true;
}

// CPP-068 phase 14. upstream: NavEKF3_core::readMagData()'s
// `storedMag.push(magDataNew);` (AP_NavEKF3_Measurements.cpp ~line 377).
// See ekf_core.hpp's "CPP-068, PHASE 14" banner and push_mag_sample()'s
// own doc comment for the full scope discussion - a thin pass-through,
// identical in shape to push_gps_sample() above, all the real behavior
// lives in ObsBuffer::push() (ekf_buffer.hpp).
void EkfCore::push_mag_sample(const MagSample& sample) {
    mag_buffer.push(sample);
}

// CPP-068 phase 14. upstream: NavEKF3_core::SelectMagFusion()'s
// `magDataToFuse = storedMag.recall(magDataDelayed,
// imuDataDelayed.time_ms);` (AP_NavEKF3_MagFusion.cpp ~line 411) - see
// ekf_core.hpp's "CPP-068, PHASE 14" banner and recall_mag_sample()'s own
// doc comment for the full now_s-vs-imuDataDelayed.time_ms discussion.
// `now_s` is converted to milliseconds identically to recall_gps_sample()
// above (clamp negative to zero before the cast, matching MagSample::
// set_time_s()'s own convention) - ObsBuffer::recall()'s own 100ms window
// and dt arithmetic (ekf_buffer.hpp) then decide the match.
bool EkfCore::recall_mag_sample(MagSample& out, ftype now_s) {
    const ftype clamped_now_s = now_s > ftype(0) ? now_s : ftype(0);
    const std::uint32_t now_ms = static_cast<std::uint32_t>(clamped_now_s * ftype(1000));
    return mag_buffer.recall(out, now_ms);
}

// ============================================================================
// CPP-062 PHASE 8: baro height fusion. See ekf_core.hpp's "CPP-062, PHASE 8"
// banner for the full scope/exclusions/corrections discussion - only
// implementation-level notes live here.
// ============================================================================

// upstream: selectHeightForFusion()'s baro branch, AP_NavEKF3_PosVelFusion.cpp
// ~line 1376-1377 - `posDownObsNoise = sq(constrain_ftype(frontend->
// _baroAltNoise, 0.1f, 100.0f))`. Reuses the phase-1 baro_alt_noise field
// directly (see ekf_core.hpp banner).
ftype EkfCore::baro_hgt_obs_variance() const { return sq(clamp(baro_alt_noise, ftype(0.1), ftype(100.0))); }

// upstream: hgtTestRatio, AP_NavEKF3_PosVelFusion.cpp ~line 929-934 -
// `innovVelPos[5] = stateStruct.position.z - velPosObs[5]; varInnovVelPos[5]
// = P[9][9] + R_OBS_DATA_CHECKS[5]; hgtTestRatio = sq(innovVelPos[5]) /
// (sq(MAX(0.01*_hgtInnovGate,1.0)) * varInnovVelPos[5]);`. See ekf_core.hpp
// banner's sign-convention derivation for why `state.position.z +
// baro_altitude_m` is exactly upstream's `position.z - velPosObs[5]` here
// (velPosObs[5] = -baro_altitude_m). R_OBS_DATA_CHECKS[5] == R_OBS[5] ==
// posDownObsNoise unconditionally upstream (~line 774: `for (i=3;i<=5;i++)
// R_OBS_DATA_CHECKS[i] = R_OBS[i];`) - reuses baro_hgt_obs_variance()
// directly, no duplicate formula, same reasoning as gps_pos_test_ratio()
// reusing gps_horiz_pos_obs_variance().
ftype EkfCore::hgt_test_ratio(ftype baro_altitude_m) const {
    const ftype r_obs = baro_hgt_obs_variance();
    const ftype innov = state.position.z + baro_altitude_m;
    const ftype var_innov = P[9][9] + r_obs;
    const ftype gate = std::max(ftype(0.01) * hgt_innov_gate_pct, ftype(1.0));
    return sq(innov) / (sq(gate) * var_innov);
}

// upstream: NavEKF3_core::ResetHeight(), AP_NavEKF3_PosVelFusion.cpp lines
// 287-355 - reduced to ONLY state.position.z + P[9][9], see ekf_core.hpp's
// "CPP-062, PHASE 8" banner ("A REAL DIVERGENCE FOUND IN ResetHeight()'S OWN
// BODY" / "DELIBERATELY NOT REPRODUCED") for the real velocity.z/P[6][6]
// touch upstream additionally makes and why this port does not reproduce it.
// Overwrites state.position.z directly from the given baro_altitude_m
// (upstream: `stateStruct.position.z = -hgtMea;`) and re-seeds P[9][9] to
// baro_hgt_obs_variance() (upstream: `P[9][9] = posDownObsNoise;`). Stamps
// last_hgt_pass_time_s = now_s, matching upstream's own `lastHgtPassTime_ms
// = imuSampleTime_ms;` at ResetHeight()'s own timeout-clearing line (~line
// 316-317).
void EkfCore::reset_height(ftype baro_altitude_m, ftype now_s) {
    zero_rows_cols(P, 9, 9);  // upstream: zeroRows(P,9,9); zeroCols(P,9,9);
    state.position.z = -baro_altitude_m;
    P[9][9] = baro_hgt_obs_variance();
    last_hgt_pass_time_s = now_s;
}

// upstream: FuseVelPosNED()'s obsIndex==5 path. Innovation formula (~line
// 929): `innovVelPos[5] = stateStruct.position.z - velPosObs[5];` - see
// hgt_test_ratio()'s own comment for the sign-convention derivation, reused
// identically here.
//
// Gated by hgt_test_ratio(), computed ONCE here using the state/P as they
// stand at entry, same reasoning as fuse_gps_velocity()/fuse_gps_position()
// (CPP-057). Failing the gate (ratio >= 1.0) skips fusion entirely for this
// cycle, P/state left completely untouched, matching upstream's own `else {
// fuseHgtData = false; }` (~line 979).
//
// On a gate failure, checks the elapsed time since last_hgt_pass_time_s
// against kBaroFusionTimeoutS (hgtRetryTimeMode0_ms = 10.0s, a SEPARATE
// constant from fuse_gps_velocity()/fuse_gps_position()'s own
// kGpsFusionTimeoutS - see ekf_core.hpp banner "A REAL CONSTANT-IDENTITY
// CHECK") BEFORE returning false - if timed out, calls reset_height()
// instead of simply skipping (upstream: `if (hgtTimeout) { ResetHeight();
// fuseHgtData = false; }`, ~line 975-977). A reset is NOT a fusion (matches
// upstream's own `fuseHgtData = false` right after the reset call), so this
// still returns false either way on the failing branch.
bool EkfCore::fuse_baro_height(ftype baro_altitude_m, ftype dt_ekf_avg, ftype now_s) {
    if (hgt_test_ratio(baro_altitude_m) >= ftype(1.0)) {
        if ((now_s - last_hgt_pass_time_s) >= kBaroFusionTimeoutS) {
            reset_height(baro_altitude_m, now_s);
        }
        return false;
    }

    const ftype r_obs = baro_hgt_obs_variance();
    const ftype innovation = state.position.z + baro_altitude_m;
    const bool applied = fuse_direct_state_observation(9, innovation, r_obs, dt_ekf_avg);
    if (applied) {
        last_hgt_pass_time_s = now_s;
    }
    return applied;
}

// CPP-069 phase 15. upstream: NavEKF3_core::readBaroData()'s
// `storedBaro.push(baroDataNew);` (AP_NavEKF3_Measurements.cpp ~line 799).
// See ekf_core.hpp's "CPP-069, PHASE 15" banner and push_baro_sample()'s
// own doc comment for the full scope discussion - a thin pass-through,
// identical in shape to push_gps_sample()/push_mag_sample() above, all the
// real behavior lives in ObsBuffer::push() (ekf_buffer.hpp).
void EkfCore::push_baro_sample(const BaroSample& sample) {
    baro_buffer.push(sample);
}

// CPP-069 phase 15. upstream: selectHeightForFusion()'s
// `baroDataToFuse = storedBaro.recall(baroDataDelayed,
// imuDataDelayed.time_ms);` (AP_NavEKF3_PosVelFusion.cpp ~line 1207) - see
// ekf_core.hpp's "CPP-069, PHASE 15" banner and recall_baro_sample()'s own
// doc comment for the full now_s-vs-imuDataDelayed.time_ms discussion.
// `now_s` is converted to milliseconds identically to recall_gps_sample()/
// recall_mag_sample() above (clamp negative to zero before the cast,
// matching BaroSample::set_time_s()'s own convention) - ObsBuffer::
// recall()'s own 100ms window and dt arithmetic (ekf_buffer.hpp) then
// decide the match.
bool EkfCore::recall_baro_sample(BaroSample& out, ftype now_s) {
    const ftype clamped_now_s = now_s > ftype(0) ? now_s : ftype(0);
    const std::uint32_t now_ms = static_cast<std::uint32_t>(clamped_now_s * ftype(1000));
    return baro_buffer.recall(out, now_ms);
}

// ============================================================================
// CPP-063 PHASE 9: true airspeed / wind velocity fusion. See ekf_core.hpp's
// "CPP-063, PHASE 9" banner for the full scope/exclusions/corrections
// discussion - only implementation-level notes live here.
// ============================================================================

// upstream: tasTestRatio, AP_NavEKF3_AirDataFusion.cpp ~line 108-110 - see
// ekf_core.hpp banner "tas_test_ratio() IS MAG-SHAPED" for why this reads
// this object's own stored innov_vtas/var_innov_vtas (populated by
// fuse_airspeed() itself) rather than recomputing fresh.
ftype EkfCore::tas_test_ratio() const {
    const ftype gate = std::max(ftype(0.01) * tas_innov_gate_pct, ftype(1.0));
    return sq(innov_vtas) / (sq(gate) * var_innov_vtas);
}

// upstream: NavEKF3_core::FuseAirspeed(), AP_NavEKF3_AirDataFusion.cpp lines
// ~20-156 - verified line-by-line against that source (not approximated).
// See ekf_core.hpp's "CPP-063, PHASE 9" banner for the full derivation of
// every simplification applied below (allowFusion always true,
// airDataFusionWindOnly provably always false, treatWindStatesAsTruth not
// modeled, dvelBiasAxisInhibit[] already collapsed) and for "THE REAL,
// THREE-WAY OUTCOME SHAPE" this function reproduces.
bool EkfCore::fuse_airspeed(ftype true_airspeed_m_s, ftype dt_ekf_avg) {
    // upstream ~line 26-30: copy required states to local variable names.
    const ftype vn = state.velocity.x;
    const ftype ve = state.velocity.y;
    const ftype vd = state.velocity.z;
    const ftype vwn = state.wind_vel.x;
    const ftype vwe = state.wind_vel.y;

    // upstream ~line 33: `VtasPred = norm((ve-vwe),(vn-vwn),vd)`.
    const ftype VtasPred = Vector3F(ve - vwe, vn - vwn, vd).length();

    // upstream ~line 34/155: the ENTIRE rest of the function - including
    // the trailing ForceSymmetry()/ConstrainVariances() calls - is wrapped
    // in `if (VtasPred > 1.0f) { ... }`. Below that threshold, NOTHING
    // happens: not a failure mode with any state/covariance consequence,
    // see banner "THE REAL, THREE-WAY OUTCOME SHAPE" outcome 1.
    if (!(VtasPred > ftype(1.0))) {
        return false;
    }

    // upstream ~line 37: `innovVtas = VtasPred - tasDataDelayed.tas;`.
    innov_vtas = VtasPred - true_airspeed_m_s;

    // upstream ~line 40-42: observation jacobians SH_TAS[0..2].
    const ftype sh_tas0 = ftype(1) / VtasPred;
    const ftype sh_tas1 = (sh_tas0 * (ftype(2) * ve - ftype(2) * vwe)) * ftype(0.5);
    const ftype sh_tas2 = (sh_tas0 * (ftype(2) * vn - ftype(2) * vwn)) * ftype(0.5);

    // upstream ~line 43-48: H_TAS[4]/[5]/[6]/[22]/[23] - all other indices
    // are 0 (value-initialized below, same convention as fuse_magnetometer()'s
    // H_MAG).
    std::array<ftype, 24> H_TAS{};
    H_TAS[4] = sh_tas2;
    H_TAS[5] = sh_tas1;
    H_TAS[6] = vd * sh_tas0;
    H_TAS[22] = -sh_tas2;
    H_TAS[23] = -sh_tas1;

    // upstream: tasDataDelayed.tasVariance = sq(MAX(_easNoise*EAS2TAS,
    // 0.5f)) (readAirSpdData(), already out of scope - see banner). EAS2TAS
    // assumed 1.0 (banner "EAS2TAS - NOT MODELED", reusing TECS's own
    // precedent).
    const ftype tas_variance = sq(std::max(eas_noise, ftype(0.5)));

    // upstream ~line 49: dense, auto-generated Kalman-gain-denominator
    // expression spanning P[4..6][4..6,22,23] - transcribed verbatim, same
    // disclosed exception as CovariancePrediction()/FuseMagnetometer()'s own
    // dense blocks (see banner).
    const ftype temp =
        (tas_variance +
         sh_tas2 * (P[4][4] * sh_tas2 + P[5][4] * sh_tas1 - P[22][4] * sh_tas2 - P[23][4] * sh_tas1 +
                    P[6][4] * vd * sh_tas0) +
         sh_tas1 * (P[4][5] * sh_tas2 + P[5][5] * sh_tas1 - P[22][5] * sh_tas2 - P[23][5] * sh_tas1 +
                    P[6][5] * vd * sh_tas0) -
         sh_tas2 * (P[4][22] * sh_tas2 + P[5][22] * sh_tas1 - P[22][22] * sh_tas2 - P[23][22] * sh_tas1 +
                    P[6][22] * vd * sh_tas0) -
         sh_tas1 * (P[4][23] * sh_tas2 + P[5][23] * sh_tas1 - P[22][23] * sh_tas2 - P[23][23] * sh_tas1 +
                    P[6][23] * vd * sh_tas0) +
         vd * sh_tas0 * (P[4][6] * sh_tas2 + P[5][6] * sh_tas1 - P[22][6] * sh_tas2 - P[23][6] * sh_tas1 +
                          P[6][6] * vd * sh_tas0));

    // upstream ~line 54-60: badly-conditioned check - mirrors phase 5's
    // mag-fusion badly-conditioned-axis failure mode exactly (banner "THE
    // REAL, THREE-WAY OUTCOME SHAPE" outcome 2). Unlike phase 5/6's mag
    // fusion, there is no SECOND (healthyFusion) covariance-reset guard
    // anywhere in FuseAirspeed() - verified directly (banner "THE REAL,
    // DISTINCTIVE ABSENCE").
    if (temp < tas_variance) {
        covariance_init(dt_ekf_avg);
        return false;
    }
    const ftype sk_tas0 = ftype(1) / temp;
    const ftype sk_tas1 = sh_tas1;  // upstream ~line 61: `SK_TAS[1] = SH_TAS[1];`

    // upstream ~line 63-90: kalman_mask construction. See banner "A NOTABLE
    // STRUCTURAL CONSEQUENCE" - after this port's already-established
    // exclusions, this is algebraically identical in structure to
    // fuse_magnetometer()'s own kalman_mask block (same four inhibit flags,
    // same nesting order) - reused directly, not re-derived.
    std::uint32_t kalman_mask = (1u << 10) - 1;
    if (!inhibit_del_ang_bias_states) {
        kalman_mask |= (1u << 10) | (1u << 11) | (1u << 12);
    }
    if (!inhibit_del_vel_bias_states) {
        kalman_mask |= (1u << 13) | (1u << 14) | (1u << 15);
    }
    if (!inhibit_mag_states) {
        kalman_mask |= (1u << 16) | (1u << 17) | (1u << 18) | (1u << 19) | (1u << 20) | (1u << 21);
    }
    if (!inhibit_wind_states) {
        kalman_mask |= (1u << 22) | (1u << 23);
    }

    // upstream ~line 92-97: NOT bounded at stateIndexLim, same as
    // fuse_magnetometer()'s own Kfusion loop (banner: verified equal to
    // bounding at state_index_lim() for every inhibit-flag combination,
    // reusing phase 5/6's own proof rather than re-deriving it).
    std::array<ftype, 24> kfusion{};
    for (int i = 0; i < 24; ++i) {
        if ((kalman_mask & (1u << i)) == 0) {
            continue;
        }
        const auto ii = static_cast<std::size_t>(i);
        kfusion[ii] = sk_tas0 * (P[ii][4] * sh_tas2 - P[ii][22] * sh_tas2 + P[ii][5] * sk_tas1 -
                                  P[ii][23] * sk_tas1 + P[ii][6] * vd * sh_tas0);
    }

    // upstream ~line 105: `varInnovVtas = 1.0f/SK_TAS[0];` - exactly `temp`
    // (see banner "tas_test_ratio() IS MAG-SHAPED").
    var_innov_vtas = temp;

    // upstream ~line 108-113: `tasTestRatio = ...; isConsistent =
    // (tasTestRatio < 1.0f) || badIMUdata;` - badIMUdata already-established
    // permanently false (phase 1 simplification 3), dropped.
    const bool is_consistent = tas_test_ratio() < ftype(1.0);

    const int lim = state_index_lim();

    // upstream ~line 117-149: `if (tasDataDelayed.allowFusion &&
    // (isConsistent || (tasTimeout && posTimeout))) { ... }` - allowFusion
    // always true (banner), the `tasTimeout && posTimeout` forced-fusion
    // disjunct explicitly out of scope (banner) - reduces to `if
    // (is_consistent)`.
    if (is_consistent) {
        // upstream ~line 128-138: covariance update P -= K*H*P, taking
        // advantage of H_TAS's known-zero elements - same structure as
        // fuse_magnetometer()'s own KHP loop, bounded at lim.
        Matrix24 khp{};
        for (int i = 0; i <= lim; ++i) {
            const auto ii = static_cast<std::size_t>(i);
            for (int j = 0; j <= lim; ++j) {
                const auto jj = static_cast<std::size_t>(j);
                ftype res = 0;
                res += (kfusion[ii] * H_TAS[4]) * P[4][jj];
                res += (kfusion[ii] * H_TAS[5]) * P[5][jj];
                res += (kfusion[ii] * H_TAS[6]) * P[6][jj];
                res += (kfusion[ii] * H_TAS[22]) * P[22][jj];
                res += (kfusion[ii] * H_TAS[23]) * P[23][jj];
                khp[ii][jj] = res;
            }
        }
        for (int i = 0; i <= lim; ++i) {
            const auto ii = static_cast<std::size_t>(i);
            for (int j = 0; j <= lim; ++j) {
                const auto jj = static_cast<std::size_t>(j);
                P[ii][jj] -= khp[ii][jj];
            }
        }

        // upstream ~line 120-125: `statesArray[j] -= Kfusion[j]*innovVtas;
        // stateStruct.quat.normalize();` - reuses apply_state_correction(),
        // no new state-correction code needed (same reuse as
        // fuse_baro_height()).
        apply_state_correction(kfusion, innov_vtas);
    }

    // upstream ~line 153-154: `ForceSymmetry(); ConstrainVariances();` -
    // OUTSIDE the `isConsistent` if-block but INSIDE the outer
    // `if (VtasPred > 1.0f)` block, so these run UNCONDITIONALLY here,
    // regardless of whether the gate passed or failed - see banner "THE
    // REAL, THREE-WAY OUTCOME SHAPE" outcome 3 for the real, verified
    // consequence of this on a gate failure.
    force_symmetry(lim);
    constrain_variances(dt_ekf_avg);

    return is_consistent;
}

// See ekf_core.hpp's "CPP-070, PHASE 16" banner and push_tas_sample()'s
// own doc comment for the full scope discussion - a thin pass-through,
// identical in shape to push_gps_sample()/push_mag_sample()/
// push_baro_sample() above, all the real behavior lives in
// ObsBuffer::push() (ekf_buffer.hpp).
void EkfCore::push_tas_sample(const TasSample& sample) {
    tas_buffer.push(sample);
}

// CPP-070 phase 16. upstream: readAirSpdData()'s `tasDataToFuse =
// storedTAS.recall(tasDataDelayed,imuDataDelayed.time_ms);`
// (AP_NavEKF3_Measurements.cpp ~line 882) - see ekf_core.hpp's "CPP-070,
// PHASE 16" banner and recall_tas_sample()'s own doc comment for the full
// now_s-vs-imuDataDelayed.time_ms discussion. `now_s` is converted to
// milliseconds identically to recall_gps_sample()/recall_mag_sample()/
// recall_baro_sample() above (clamp negative to zero before the cast,
// matching TasSample::set_time_s()'s own convention) - ObsBuffer::
// recall()'s own 100ms window and dt arithmetic (ekf_buffer.hpp) then
// decide the match.
bool EkfCore::recall_tas_sample(TasSample& out, ftype now_s) {
    const ftype clamped_now_s = now_s > ftype(0) ? now_s : ftype(0);
    const std::uint32_t now_ms = static_cast<std::uint32_t>(clamped_now_s * ftype(1000));
    return tas_buffer.recall(out, now_ms);
}

// ============================================================================
// CPP-071, PHASE 17 (this ticket). See ekf_core.hpp's own "CPP-071,
// PHASE 17" banner (above EkfCore::tick()'s declaration) for the full
// upstream-verification, delay-depth-derivation, and pre-fill-strategy
// discussion. This implementation is a thin, direct transcription of
// upstream's real per-tick sequence (push_youngest_element() then
// get_oldest_element(), unconditionally, every tick) plus the pre-fill
// seeding that sequence needs on this port (see hpp banner) - nothing
// here is a new algorithm.
void EkfCore::tick(const GyroSample& gyro, const AccelSample& accel, ftype dt_ekf_avg) {
    if (!imu_buffer_seeded) {
        // One-time pre-fill: seed every slot with a stationary/level
        // no-op sample shaped by THIS call's dt_ekf_avg - see hpp
        // banner's "PRE-FILL STRATEGY" section for why (avoids the
        // 0/0 = NaN hazard a raw zero-dt default ImuSample would create
        // in update_strapdown_equations_ned()'s
        // `vel_dot_ned = del_vel_nav / accel.delta_velocity_dt`, while
        // still genuinely delaying - not short-circuiting - the pre-fill
        // window, unlike upstream NavEKF2's own reset_history(imuDataNew)
        // convention which seeds with the just-arrived REAL sample
        // instead). Matches ekf_core_test.cpp's own already-verified
        // "stationary vehicle" no-op convention exactly: zero
        // delta_angle, delta_velocity = (0,0,-g*dt) (exactly cancels
        // gravity), both dt fields = dt_ekf_avg.
        GyroSample seed_gyro;
        seed_gyro.delta_angle_dt = dt_ekf_avg;
        AccelSample seed_accel;
        seed_accel.delta_velocity = Vector3F(ftype(0), ftype(0), -kGravityMss * dt_ekf_avg);
        seed_accel.delta_velocity_dt = dt_ekf_avg;
        imu_buffer.reset_history(ImuSample{seed_gyro, seed_accel});
        imu_buffer_seeded = true;
    }

    // upstream: storedIMU.push_youngest_element(imuDataDownSampledNew);
    // immediately followed by imuDataDelayed = storedIMU.
    // get_oldest_element(); - the SAME buffer, same order, every tick,
    // unconditionally (downsampling gate moot per ADR-0012 - see hpp
    // banner). ImuBuffer<T,N> is non-destructive (ekf_buffer.hpp): this
    // read does not consume or remove the element, unlike ObsBuffer::
    // recall()'s destructive semantics used elsewhere in this file for
    // GPS/mag/baro/TAS - see ekf_buffer.hpp's own file banner for that
    // distinction.
    imu_buffer.push_youngest_element(ImuSample{gyro, accel});
    const ImuSample delayed = imu_buffer.get_oldest_element();

    // CPP-073: while ticks_since_seed is still less than
    // kImuBufferCapacity-1, get_oldest_element() above is still returning
    // one of the seeded no-op slots, not a genuinely-pushed real sample -
    // see hpp banner's "CPP-073 ADDENDUM" for the full bug trace. The
    // seeded sample is a true mechanization no-op for attitude/velocity
    // (zero delta-angle, gravity-cancelling delta-velocity), but
    // update_strapdown_equations_ned()'s trapezoidal position integration
    // reads the state's OWN EXISTING velocity every call regardless of
    // the IMU sample content, so a non-zero starting velocity still
    // advances position during these calls unless explicitly held. Snapshot
    // state.position immediately before mechanizing and restore it
    // immediately after, for exactly this window - velocity/attitude/
    // covariance are untouched, already correctly unaffected.
    const bool in_prefill_window = ticks_since_seed < (kImuBufferCapacity - 1);
    const Vector3F position_before_prefill = state.position;

    // The EXISTING, UNCHANGED direct-call functions - called here with
    // the delayed, buffer-sourced sample instead of an immediate one.
    // Their own signatures/behavior are completely untouched by this
    // ticket (see hpp banner) - every existing test/closed-loop phase
    // keeps calling them directly and is unaffected.
    update_strapdown_equations_ned(delayed.gyro, delayed.accel, dt_ekf_avg);
    covariance_prediction(delayed.gyro, delayed.accel, dt_ekf_avg);

    if (in_prefill_window) {
        state.position = position_before_prefill;
    }
    ++ticks_since_seed;

    // upstream: imuDataDelayed's own timestamp concept, advanced by this
    // tick's dt_ekf_avg - see hpp banner's own field doc comment for why
    // this is the closest expressible equivalent under this port's
    // caller-supplied-time convention (ADR-0012), and for what is/isn't
    // wired to consume it yet (nothing, in this ticket).
    delayed_time_s += dt_ekf_avg;
}

} // namespace fwcpp::ekf
