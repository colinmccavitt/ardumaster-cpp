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
// blocks per column (mag/wind columns 16..23 included) because its
// codegen tool doesn't parametrize over column index. This port instead
// runs a small loop over columns 10..15 (verified algebraically, using
// P's symmetry, to be IDENTICAL to each of upstream's unrolled column-10
// through column-15 blocks - not a new formula) and skips columns 16..23
// altogether: those states are permanently "inhibited" in this phase (no
// mag/wind fusion exists - see ekf_core.hpp's simplification 1), and
// upstream's own ConstrainVariances() zeros their entire row/column every
// single cycle when inhibited - so computing their correlation growth
// here would be work upstream itself immediately discards under these
// conditions.
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

    // processNoiseVariance[0..5] map to state indices 10..15 (gyro bias,
    // accel bias) - always active in this phase (simplification 2).
    // Indices 6..13 (mag/wind, state 16..23) are always 0 - permanently
    // inhibited (simplification 1).
    std::array<ftype, 14> process_noise_variance{};
    {
        const ftype d_ang_bias_var = sq(sq(dt) * clamp(gyro_bias_process_noise, ftype(0), ftype(1)));
        for (int i = 0; i <= 2; ++i) process_noise_variance[static_cast<std::size_t>(i)] = d_ang_bias_var;
    }
    {
        const ftype d_vel_bias_var = sq(sq(dt) * clamp(accel_bias_process_noise, ftype(0), ftype(1)));
        for (int i = 3; i <= 5; ++i) process_noise_variance[static_cast<std::size_t>(i)] = d_vel_bias_var;
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

    // Add process noise to the gyro/accel-bias diagonal (states 10..15).
    // Mag/wind (16..23) get none - permanently inhibited, see hpp banner.
    for (int i = 10; i <= 15; ++i) {
        nextP[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] =
            nextP[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] +
            process_noise_variance[static_cast<std::size_t>(i - 10)];
    }

    // Position-variance-collapse guard (upstream: "if the total position
    // variance exceeds 1e4 (100m), then stop covariance growth").
    if ((P[7][7] + P[8][8]) > ftype(1e4)) {
        for (int i = 7; i <= 8; ++i) {
            for (int j = 0; j <= 15; ++j) {
                nextP[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                    P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
                nextP[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] =
                    P[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)];
            }
        }
    }

    // Symmetric copy-back, states 0..15 (16..23 handled by the inhibited-
    // state zeroing in constrain_variances() below, matching upstream's
    // own zeroRows/zeroCols behavior for those states in this
    // configuration - see hpp banner simplification 1).
    for (int row = 0; row <= 15; ++row) {
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
// 1877. See hpp banner simplification 1 (mag/wind permanently inhibited ->
// zeroed) and simplification 2 (bias states never inhibited, no per-axis
// ground-alignment gate).
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

    // Mag (16..21) and wind (22..23) - permanently inhibited in this
    // phase (simplification 1): zeroed every cycle, matching upstream's
    // own inhibited-state branch exactly.
    zero_rows_cols(P, 16, 21);
    zero_rows_cols(P, 22, 23);
}

} // namespace fwcpp::ekf
