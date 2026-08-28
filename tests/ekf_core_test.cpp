// Tests for fwcpp::ekf::EkfCore. CPP-052 phase 1 - pure strapdown INS
// mechanization and covariance time-propagation, no fusion. See
// fwcpp/ekf/ekf_core.hpp for the scope/exclusions banner.
//
// Test strategy (per the ticket's own acceptance criteria - this is the
// only meaningful correctness check available before any fusion exists):
//   1. Constant angular rate -> quaternion after N steps matches the
//      known-analytic single large-angle rotation.
//   2. Constant nav-frame acceleration -> velocity/position match the
//      known-analytic constant-acceleration kinematics (trapezoidal
//      integration is EXACT for a linear velocity ramp, so these
//      assertions use a tight tolerance, not just "approximately").
//   3. A truly stationary vehicle (zero rotation, accelerometer reading
//      that exactly cancels gravity - see note on test 3 below for why
//      that is NOT literally a zero accel vector) leaves the state
//      unchanged.
//   4. covariance_init() + repeated covariance_prediction() grows the
//      active (gyro-bias/accel-bias) diagonal monotonically and boundedly
//      under pure prediction with no fusion, while the permanently
//      inhibited mag/wind diagonal (see hpp banner simplification 1)
//      stays exactly zero - not a bug, the documented behavior of a
//      configuration with no magnetometer/airspeed fusion.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/ekf/ekf_core.hpp>

using namespace fwcpp::ekf;

namespace {
constexpr ftype kGravity = static_cast<ftype>(9.80665);
}

TEST_CASE("update_strapdown_equations_ned: constant angular rate matches analytic rotation", "[ekf_core]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0)); // identity

    const ftype omega_z = ftype(0.5);   // rad/s, constant yaw rate
    const ftype dt = ftype(0.001);      // 1 kHz IMU, small-angle-per-step
    const int steps = 2000;             // T = 2.0 s total
    const ftype dt_ekf_avg = dt;

    GyroSample gyro;
    gyro.delta_angle = Vector3F(ftype(0), ftype(0), omega_z * dt);
    gyro.delta_angle_dt = dt;
    AccelSample accel;
    // Level, stationary-equivalent accel input (see test 3's note) so this
    // test isolates rotation from translation.
    accel.delta_velocity = Vector3F(ftype(0), ftype(0), -kGravity * dt);
    accel.delta_velocity_dt = dt;

    for (int i = 0; i < steps; ++i) {
        ekf.update_strapdown_equations_ned(gyro, accel, dt_ekf_avg);
    }

    // Analytic reference: a constant body-z rotation rate for T seconds
    // produces a pure yaw rotation of omega_z*T radians - computed here
    // independently via QuaternionF::from_euler, not read back from the
    // class under test.
    const ftype total_angle = omega_z * static_cast<ftype>(steps) * dt;
    QuaternionF expected;
    expected.from_euler(ftype(0), ftype(0), total_angle);

    // Repeated small-angle composition accumulates O(dt) error over the
    // whole run - loose-but-meaningful tolerance for a 2000-step
    // integration at dt=1ms.
    REQUIRE(std::abs(static_cast<double>(ekf.state.quat.q1) - static_cast<double>(expected.q1)) < 1e-4);
    REQUIRE(std::abs(static_cast<double>(ekf.state.quat.q4) - static_cast<double>(expected.q4)) < 1e-4);
    REQUIRE(ekf.state.quat.is_unit_length());

    // Velocity/position must be untouched by pure rotation (accel input
    // was the "stationary" case throughout).
    REQUIRE(std::abs(static_cast<double>(ekf.state.velocity.x)) < 1e-6);
    REQUIRE(std::abs(static_cast<double>(ekf.state.velocity.y)) < 1e-6);
    REQUIRE(std::abs(static_cast<double>(ekf.state.velocity.z)) < 1e-6);
}

TEST_CASE("update_strapdown_equations_ned: constant nav-frame acceleration matches analytic kinematics", "[ekf_core]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0)); // identity, stays level (no gyro input)

    const ftype accel_x = ftype(2.0); // m/s^2, constant nav-frame (== body-frame, level) forward accel
    const ftype dt = ftype(0.001);
    const int steps = 1000; // T = 1.0 s
    const ftype dt_ekf_avg = dt;

    GyroSample gyro; // zero rotation - level attitude persists exactly
    gyro.delta_angle_dt = dt;
    AccelSample accel;
    // Body-z component cancels gravity (level attitude => Tbn=I), body-x
    // component is the constant accel under test.
    accel.delta_velocity = Vector3F(accel_x * dt, ftype(0), -kGravity * dt);
    accel.delta_velocity_dt = dt;

    for (int i = 0; i < steps; ++i) {
        ekf.update_strapdown_equations_ned(gyro, accel, dt_ekf_avg);
    }

    const ftype T = static_cast<ftype>(steps) * dt;
    const double expected_velocity_x = static_cast<double>(accel_x) * static_cast<double>(T);
    const double expected_position_x = 0.5 * static_cast<double>(accel_x) * static_cast<double>(T) * static_cast<double>(T);

    // Trapezoidal integration of a linear velocity ramp is exact -
    // tight tolerance (float rounding over 1000 steps only).
    REQUIRE(static_cast<double>(ekf.state.velocity.x) == Catch::Approx(expected_velocity_x).margin(1e-3));
    REQUIRE(static_cast<double>(ekf.state.position.x) == Catch::Approx(expected_position_x).margin(1e-3));
    REQUIRE(std::abs(static_cast<double>(ekf.state.velocity.y)) < 1e-6);
    REQUIRE(std::abs(static_cast<double>(ekf.state.velocity.z)) < 1e-3);
}

TEST_CASE("update_strapdown_equations_ned: a stationary vehicle's IMU reading leaves the state unchanged", "[ekf_core]") {
    // NOTE on why this isn't literally Vector3F(0,0,0): an accelerometer
    // measures specific force, not coordinate acceleration. A vehicle at
    // rest on the ground reads +1g pointing away from the ground (body -Z,
    // NED-down convention) precisely BECAUSE it is NOT in free-fall - a
    // truly zero accelerometer reading is what a vehicle in free-fall
    // reports. UpdateStrapdownEquationsNED's own gravity-compensation term
    // (`delVelNav.z += GRAVITY_MSS*delVelDT`) exists exactly to convert
    // between these two conventions - see the mechanization's real
    // upstream formula, ekf_core.cpp's update_strapdown_equations_ned().
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));

    const ftype dt = ftype(0.001);
    const int steps = 500;
    const ftype dt_ekf_avg = dt;

    GyroSample gyro;
    gyro.delta_angle_dt = dt; // zero delta_angle: no rotation
    AccelSample accel;
    accel.delta_velocity = Vector3F(ftype(0), ftype(0), -kGravity * dt); // cancels gravity exactly - see note above
    accel.delta_velocity_dt = dt;

    for (int i = 0; i < steps; ++i) {
        ekf.update_strapdown_equations_ned(gyro, accel, dt_ekf_avg);
    }

    REQUIRE(static_cast<double>(ekf.state.quat.q1) == Catch::Approx(1.0).margin(1e-9));
    REQUIRE(std::abs(static_cast<double>(ekf.state.quat.q2)) < 1e-12);
    REQUIRE(std::abs(static_cast<double>(ekf.state.quat.q3)) < 1e-12);
    REQUIRE(std::abs(static_cast<double>(ekf.state.quat.q4)) < 1e-12);
    REQUIRE(std::abs(static_cast<double>(ekf.state.velocity.x)) < 1e-9);
    REQUIRE(std::abs(static_cast<double>(ekf.state.velocity.y)) < 1e-9);
    REQUIRE(std::abs(static_cast<double>(ekf.state.velocity.z)) < 1e-6);
    REQUIRE(std::abs(static_cast<double>(ekf.state.position.x)) < 1e-9);
    REQUIRE(std::abs(static_cast<double>(ekf.state.position.y)) < 1e-9);
    REQUIRE(std::abs(static_cast<double>(ekf.state.position.z)) < 1e-6);
}

TEST_CASE("covariance_init: initial diagonal values trace to real upstream defaults", "[ekf_core]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    const ftype dt_ekf_avg = ftype(0.012); // EKF_TARGET_DT
    ekf.covariance_init(dt_ekf_avg);

    // Velocities/positions: sq(Plane-4.7.0 default noise), traced from
    // AP_NavEKF3.cpp's APM_BUILD_ArduPlane block - see ekf_core.hpp.
    REQUIRE(static_cast<double>(ekf.P[4][4]) == Catch::Approx(0.5 * 0.5));
    REQUIRE(static_cast<double>(ekf.P[6][6]) == Catch::Approx(0.7 * 0.7));
    REQUIRE(static_cast<double>(ekf.P[7][7]) == Catch::Approx(0.5 * 0.5));
    REQUIRE(static_cast<double>(ekf.P[9][9]) == Catch::Approx(3.0 * 3.0));

    // Attitude covariance is non-negative and bounded per ConstrainVariances'
    // [0,1] table entry (even though this path bypasses ConstrainVariances
    // itself - see EkfCore::covariance_prediction's quatCovResetOnly early
    // return - the reset path applies its own [0,1] clamp directly).
    for (int i = 0; i < 4; ++i) {
        REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] >= ftype(0.0));
        REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] <= ftype(1.0));
    }

    // Mag/wind: real upstream init values are non-zero (traced), but this
    // phase permanently inhibits mag/wind fusion - see hpp banner
    // simplification 1 - so covariance_init() itself still sets the real
    // upstream values here (nothing wiped yet, no predict() has run).
    REQUIRE(static_cast<double>(ekf.P[16][16]) == Catch::Approx(0.05 * 0.05));
    REQUIRE(static_cast<double>(ekf.P[22][22]) == Catch::Approx(0.0));
}

TEST_CASE("covariance_prediction: active-state diagonal grows monotonically and stays bounded under pure prediction", "[ekf_core]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    const ftype dt_ekf_avg = ftype(0.012);
    ekf.covariance_init(dt_ekf_avg);

    // After covariance_init(), the very first covariance_prediction() call
    // already zeros the permanently-inhibited mag/wind diagonal (see hpp
    // banner simplification 1) - verify that up front.
    GyroSample gyro;
    gyro.delta_angle_dt = dt_ekf_avg;
    AccelSample accel;
    accel.delta_velocity = Vector3F(ftype(0), ftype(0), -kGravity * dt_ekf_avg);
    accel.delta_velocity_dt = dt_ekf_avg;

    ftype prev_gyro_bias_var = ekf.P[10][10];
    ftype prev_accel_bias_var = ekf.P[13][13];

    const int steps = 50;
    for (int i = 0; i < steps; ++i) {
        ekf.covariance_prediction(gyro, accel, dt_ekf_avg);

        // Mag/wind: permanently zero (inhibited) - not a bug, see banner.
        for (int idx = 16; idx <= 23; ++idx) {
            REQUIRE(ekf.P[static_cast<std::size_t>(idx)][static_cast<std::size_t>(idx)] == ftype(0.0));
        }

        // Gyro bias variance (process-noise-driven, unclamped from below
        // by anything but 0) must not decrease - monotonic growth under
        // pure prediction with no fusion to ever reduce it.
        REQUIRE(ekf.P[10][10] >= prev_gyro_bias_var);
        prev_gyro_bias_var = ekf.P[10][10];

        // Accel bias variance grows too, until it saturates at
        // ConstrainVariances' own ceiling (sq(10*dtEkfAvg)) - so this is
        // "monotonic non-decreasing", not strictly increasing forever.
        REQUIRE(ekf.P[13][13] >= prev_accel_bias_var - static_cast<ftype>(1e-9));
        prev_accel_bias_var = ekf.P[13][13];

        // Bounded: every diagonal entry stays finite and within the real
        // upstream ConstrainVariances() ranges - "not exploding".
        REQUIRE(ekf.P[10][10] <= sq(static_cast<ftype>(0.175) * dt_ekf_avg));
        REQUIRE(ekf.P[13][13] <= sq(static_cast<ftype>(10.0) * dt_ekf_avg));
        for (int idx = 0; idx < 24; ++idx) {
            REQUIRE(std::isfinite(static_cast<double>(ekf.P[static_cast<std::size_t>(idx)][static_cast<std::size_t>(idx)])));
            REQUIRE(ekf.P[static_cast<std::size_t>(idx)][static_cast<std::size_t>(idx)] >= ftype(0.0));
        }
    }

    // Not collapsing: gyro bias variance grew from its initial value over
    // 50 steps of pure process-noise injection with nothing to reduce it.
    REQUIRE(ekf.P[10][10] > ftype(0.0));
}

// CPP-065 phase 11: this is the direct proof the ticket's own acceptance
// criterion asks for - with the respective inhibit flag cleared,
// covariance_prediction() now genuinely grows P[16][16] (earth magfield)
// and P[22][22] (wind velocity North) tick-over-tick, where the test
// immediately above this one shows they stay PINNED AT EXACTLY ZERO under
// default (inhibited) settings. Pure prediction, no fusion, mirroring the
// existing test's own "nothing to reduce it" setup - process noise is the
// only thing that can move these diagonals here.
TEST_CASE("covariance_prediction: with inhibit_mag_states/inhibit_wind_states cleared, P[16][16] and "
          "P[22][22] now genuinely grow tick-over-tick (process noise accumulating), where they previously "
          "stayed frozen",
          "[ekf_core]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    const ftype dt_ekf_avg = ftype(0.012);
    ekf.inhibit_mag_states = false;
    ekf.inhibit_wind_states = false;
    ekf.covariance_init(dt_ekf_avg);

    // covariance_init() sets these to real upstream values BEFORE any
    // predict() call: P[16][16] = sq(mag_noise) (nonzero seed - upstream's
    // real earth-field init value), P[22][22] = 0.0 (upstream's real wind
    // init value - verified directly, AP_NavEKF3_core.cpp ~line 610-611,
    // and already reproduced by this port's own covariance_init()).
    const ftype p16_after_init = ekf.P[16][16];
    const ftype p22_after_init = ekf.P[22][22];
    REQUIRE(static_cast<double>(p16_after_init) == Catch::Approx(0.05 * 0.05));
    REQUIRE(static_cast<double>(p22_after_init) == Catch::Approx(0.0));

    GyroSample gyro;
    gyro.delta_angle_dt = dt_ekf_avg;
    AccelSample accel;
    accel.delta_velocity = Vector3F(ftype(0), ftype(0), -kGravity * dt_ekf_avg);
    accel.delta_velocity_dt = dt_ekf_avg;

    ftype prev_p16 = ekf.P[16][16];
    ftype prev_p22 = ekf.P[22][22];
    bool p16_grew_at_least_once = false;
    bool p22_grew_at_least_once = false;

    const int steps = 200;
    for (int i = 0; i < steps; ++i) {
        ekf.covariance_prediction(gyro, accel, dt_ekf_avg);

        // Monotonic non-decreasing - process noise only ever adds, and
        // constrain_variances()'s clamp floor is 0.0, so nothing here can
        // push these diagonals down.
        REQUIRE(ekf.P[16][16] >= prev_p16 - static_cast<ftype>(1e-12));
        REQUIRE(ekf.P[22][22] >= prev_p22 - static_cast<ftype>(1e-12));
        if (ekf.P[16][16] > prev_p16) p16_grew_at_least_once = true;
        if (ekf.P[22][22] > prev_p22) p22_grew_at_least_once = true;
        prev_p16 = ekf.P[16][16];
        prev_p22 = ekf.P[22][22];

        // Bounded by ConstrainVariances' own real ranges (see
        // constrain_variances()'s CPP-065 banner): [0, 0.01] for mag,
        // [0, WIND_VEL_VARIANCE_MAX=400] for wind.
        REQUIRE(ekf.P[16][16] <= ftype(0.01));
        REQUIRE(ekf.P[22][22] <= ftype(400.0));
        REQUIRE(std::isfinite(static_cast<double>(ekf.P[16][16])));
        REQUIRE(std::isfinite(static_cast<double>(ekf.P[22][22])));
    }

    // THE CENTRAL FINDING: both diagonals moved - not frozen at their
    // covariance_init() values the way they would be (and still are, see
    // the test above) at this port's real default (inhibited) settings.
    REQUIRE(p16_grew_at_least_once);
    REQUIRE(p22_grew_at_least_once);
    REQUIRE(ekf.P[16][16] > p16_after_init);
    REQUIRE(ekf.P[22][22] > p22_after_init);

    // P[22][22] started at exactly 0.0 (unlike P[16][16]'s nonzero seed) -
    // confirm it is now strictly positive, the sharpest possible
    // statement that wind process noise is real and active.
    REQUIRE(ekf.P[22][22] > ftype(0.0));
}

// ============================================================================
// CPP-077, PHASE 22: tilt-alignment and gyro-bias-convergence status
// tracking. See fwcpp/ekf/ekf_core.hpp's own "CPP-077, PHASE 22" banner for
// the full scope, branch-decision, and latching-behavior writeup these
// tests verify directly against the real upstream source.
// ============================================================================

TEST_CASE("calc_tilt_error_variance: verbatim formula matches its own closed form at identity attitude",
          "[ekf_core][cpp077]") {
    // At an identity quaternion (q0=1, q1=q2=q3=0), the verbatim upstream
    // formula's PS-intermediates collapse algebraically to
    // tilt_error_variance == 4*(P[1][1] + P[2][2]) - hand-derived directly
    // from the transcribed formula, not assumed. A wrong coefficient or
    // wrong PS-index anywhere in the surviving terms would make this fail.
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[1][1] = ftype(0.01);
    ekf.P[2][2] = ftype(0.02);

    ekf.calc_tilt_error_variance();

    REQUIRE(static_cast<double>(ekf.tilt_error_variance) == Catch::Approx(4.0 * (0.01 + 0.02)));
}

TEST_CASE("calc_tilt_error_variance: at identity attitude, P[0][0]/P[3][3] (w/yaw variance) do not contribute",
          "[ekf_core][cpp077]") {
    // Physically sensible and directly traceable from the transcribed
    // formula: at identity attitude, tilt (roll+pitch) error is carried
    // entirely by the x/y quaternion-error components, not by w or z
    // (yaw) - exactly what a correct transcription of upstream's
    // quaternion_error_propagation()-generated formula should produce.
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[0][0] = ftype(100.0);
    ekf.P[3][3] = ftype(100.0);
    ekf.P[1][1] = ftype(0.0);
    ekf.P[2][2] = ftype(0.0);

    ekf.calc_tilt_error_variance();

    REQUIRE(static_cast<double>(ekf.tilt_error_variance) == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("calc_tilt_error_variance: clamps to sq(radians(30)) per upstream's own constrain_ftype call",
          "[ekf_core][cpp077]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[1][1] = ftype(1.0e6);
    ekf.P[2][2] = ftype(1.0e6);

    ekf.calc_tilt_error_variance();

    const double expected_max = static_cast<double>(sq(fwcpp::math::radians(ftype(30.0))));
    REQUIRE(static_cast<double>(ekf.tilt_error_variance) == Catch::Approx(expected_max));
}

TEST_CASE("EkfCore construction: tilt_error_variance defaults to upstream's own large defensive value, not 0.0",
          "[ekf_core][cpp077]") {
    // See ekf_core.hpp's own "tilt_error_variance DEFAULT VALUE" banner: a
    // raw 0.0 default would read as "already perfectly aligned" before any
    // real covariance has ever been computed - upstream's real reset-all-
    // state function avoids this with `tiltErrorVariance = sq(M_2PI);`;
    // this port reproduces the same value via radians(360.0).
    EkfCore ekf;
    REQUIRE(static_cast<double>(ekf.tilt_error_variance) > 1.0);

    // Directly consequential: check_attitude_alignment_status() on a
    // freshly-constructed EkfCore (before any calc_tilt_error_variance()
    // call) must NOT spuriously report alignment complete.
    ekf.check_attitude_alignment_status();
    REQUIRE_FALSE(ekf.tilt_align_complete);
}

TEST_CASE("check_attitude_alignment_status: tilt_align_complete transitions false->true exactly at the real "
          "tilt_error_variance threshold, then stays latched true even if covariance grows back above it",
          "[ekf_core][cpp077]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    const double threshold = static_cast<double>(sq(fwcpp::math::radians(ftype(5.0))));

    // ABOVE threshold: tilt_error_variance = 4*(P11+P22) = 1.5*threshold.
    const ftype above_each = static_cast<ftype>(1.5 * threshold / 8.0);
    ekf.P[1][1] = above_each;
    ekf.P[2][2] = above_each;
    ekf.calc_tilt_error_variance();
    REQUIRE(static_cast<double>(ekf.tilt_error_variance) > threshold);
    ekf.check_attitude_alignment_status();
    REQUIRE_FALSE(ekf.tilt_align_complete);

    // BELOW threshold: tilt_error_variance = 0.5*threshold - crosses to
    // true.
    const ftype below_each = static_cast<ftype>(0.5 * threshold / 8.0);
    ekf.P[1][1] = below_each;
    ekf.P[2][2] = below_each;
    ekf.calc_tilt_error_variance();
    REQUIRE(static_cast<double>(ekf.tilt_error_variance) < threshold);
    ekf.check_attitude_alignment_status();
    REQUIRE(ekf.tilt_align_complete);

    // LATCH CHECK: grow covariance back above threshold and re-run both
    // functions - a real one-way latch (verified directly against
    // upstream, see ekf_core.hpp banner) must NOT clear tilt_align_complete.
    ekf.P[1][1] = above_each;
    ekf.P[2][2] = above_each;
    ekf.calc_tilt_error_variance();
    REQUIRE(static_cast<double>(ekf.tilt_error_variance) > threshold);
    ekf.check_attitude_alignment_status();
    REQUIRE(ekf.tilt_align_complete);
}

TEST_CASE("check_gyro_cal_status: with inhibit_mag_states (default), only X/Y gyro-bias variance (rotated by "
          "prev_tnb) gates del_ang_bias_learned - Z is ignored, matching upstream's yaw-unobservable-without-a-"
          "yaw-reference branch",
          "[ekf_core][cpp077]") {
    EkfCore ekf; // inhibit_mag_states defaults true; prev_tnb defaults identity
    const ftype dt_ekf_avg = ftype(0.012);
    const double max_val = static_cast<double>(sq(fwcpp::math::radians(ftype(0.15) * dt_ekf_avg)));
    const ftype small = static_cast<ftype>(max_val * 0.1);
    const ftype huge = static_cast<ftype>(max_val * 100.0);

    ekf.P[10][10] = small;
    ekf.P[11][11] = small;
    ekf.P[12][12] = huge; // Z - must be ignored by this branch

    ekf.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE(ekf.del_ang_bias_learned);

    // Now put the huge value on X instead - the branch's horizontal check
    // must catch it.
    ekf.P[10][10] = huge;
    ekf.P[12][12] = small;
    ekf.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE_FALSE(ekf.del_ang_bias_learned);
}

TEST_CASE("check_gyro_cal_status: with inhibit_mag_states cleared, all three body-frame axes gate "
          "del_ang_bias_learned directly, matching upstream's compass-in-use branch",
          "[ekf_core][cpp077]") {
    EkfCore ekf;
    ekf.inhibit_mag_states = false;
    const ftype dt_ekf_avg = ftype(0.012);
    const double max_val = static_cast<double>(sq(fwcpp::math::radians(ftype(0.15) * dt_ekf_avg)));
    const ftype small = static_cast<ftype>(max_val * 0.1);
    const ftype huge = static_cast<ftype>(max_val * 100.0);

    // All three axes small -> learned.
    ekf.P[10][10] = small;
    ekf.P[11][11] = small;
    ekf.P[12][12] = small;
    ekf.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE(ekf.del_ang_bias_learned);

    // Z alone huge - this branch DOES check Z (unlike the inhibited branch
    // above) - must now report not-learned.
    ekf.P[12][12] = huge;
    ekf.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE_FALSE(ekf.del_ang_bias_learned);
}

TEST_CASE("check_gyro_cal_status: del_ang_bias_learned is NOT a latch - it toggles both ways as covariance "
          "grows and shrinks, unlike tilt_align_complete",
          "[ekf_core][cpp077]") {
    EkfCore ekf;
    ekf.inhibit_mag_states = false; // simplest branch: direct body-frame check, all 3 axes
    const ftype dt_ekf_avg = ftype(0.012);
    const double max_val = static_cast<double>(sq(fwcpp::math::radians(ftype(0.15) * dt_ekf_avg)));
    const ftype small = static_cast<ftype>(max_val * 0.1);
    const ftype huge = static_cast<ftype>(max_val * 100.0);

    ekf.P[10][10] = ekf.P[11][11] = ekf.P[12][12] = small;
    ekf.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE(ekf.del_ang_bias_learned);

    ekf.P[10][10] = huge; // covariance grows back up
    ekf.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE_FALSE(ekf.del_ang_bias_learned); // real toggle back to false - no latch

    ekf.P[10][10] = small; // shrinks again
    ekf.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE(ekf.del_ang_bias_learned); // toggles back to true again
}

TEST_CASE("check_gyro_cal_status: the two real upstream branches use different comparators at the exact "
          "threshold - the inhibited branch is strict '<', the direct branch is '<='",
          "[ekf_core][cpp077]") {
    const ftype dt_ekf_avg = ftype(0.012);
    const ftype max_val = sq(fwcpp::math::radians(ftype(0.15) * dt_ekf_avg));

    EkfCore ekf_inhibited; // default inhibit_mag_states = true, identity prev_tnb
    ekf_inhibited.P[10][10] = max_val;
    ekf_inhibited.P[11][11] = max_val;
    ekf_inhibited.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE_FALSE(ekf_inhibited.del_ang_bias_learned); // strict '<' - exactly-at-threshold fails

    EkfCore ekf_direct;
    ekf_direct.inhibit_mag_states = false;
    ekf_direct.P[10][10] = max_val;
    ekf_direct.P[11][11] = max_val;
    ekf_direct.P[12][12] = max_val;
    ekf_direct.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE(ekf_direct.del_ang_bias_learned); // '<=' - exactly-at-threshold passes
}

TEST_CASE("check_gyro_cal_status: a genuinely non-identity prev_tnb is really applied, not a no-op - proves "
          "the earth-frame rotation is wired correctly, not merely passed through by an identity default",
          "[ekf_core][cpp077]") {
    EkfCore ekf; // inhibit_mag_states defaults true
    const ftype dt_ekf_avg = ftype(0.012);
    const ftype max_val = sq(fwcpp::math::radians(ftype(0.15) * dt_ekf_avg));
    const ftype small = max_val * ftype(0.1);
    const ftype huge = max_val * ftype(100.0);

    // A row-permutation of the identity: row a picks up P[11][11], row b
    // picks up P[12][12], row c (unused by the horizontal-only check)
    // picks up P[10][10]. With P[10][10] huge but P[11][11]/P[12][12]
    // small, an IDENTITY prev_tnb would fail (temp.x = P[10][10] = huge),
    // but this permuted prev_tnb must pass (temp.x = P[11][11], temp.y =
    // P[12][12], both small; the huge P[10][10] lands in temp.z, which
    // this branch never checks).
    ekf.prev_tnb = Matrix3F(Vector3F(ftype(0), ftype(1), ftype(0)), Vector3F(ftype(0), ftype(0), ftype(1)),
                             Vector3F(ftype(1), ftype(0), ftype(0)));
    ekf.P[10][10] = huge;
    ekf.P[11][11] = small;
    ekf.P[12][12] = small;

    ekf.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE(ekf.del_ang_bias_learned);

    // Sanity: confirm an IDENTITY prev_tnb with the SAME P values really
    // would fail (proves the permuted-matrix pass above is due to the
    // rotation, not some other effect).
    EkfCore ekf_identity;
    ekf_identity.P[10][10] = huge;
    ekf_identity.P[11][11] = small;
    ekf_identity.P[12][12] = small;
    ekf_identity.check_gyro_cal_status(dt_ekf_avg);
    REQUIRE_FALSE(ekf_identity.del_ang_bias_learned);
}
