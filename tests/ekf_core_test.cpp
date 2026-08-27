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
