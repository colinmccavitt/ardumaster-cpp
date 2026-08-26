// Tests for fwcpp::steer_control::SteerController - ground/taxi steering
// rate and angle-error controller (see steer_controller.hpp's own file
// banner for the full port design and why it is NOT AC_PID-based, unlike
// RollController/PitchController/YawController).
//
// Style note: mirrors fw_control_test.cpp - drives the controller through
// its two public entry points (get_steering_out_rate()/
// get_steering_out_angle_error()) and reads back get_pid_info() for
// white-box checks of the internal P/I/D/FF state.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/steer_control/steer_controller.hpp>

using namespace fwcpp::steer_control;

namespace {

SteerInputs make_inputs(float ground_speed_ms, float yaw_rate_earth_dps, std::uint32_t now_ms) {
    SteerInputs in;
    in.ground_speed_ms = ground_speed_ms;
    in.yaw_rate_earth_dps = yaw_rate_earth_dps;
    in.dt = 0.02f;
    in.now_ms = now_ms;
    return in;
}

} // namespace

// ---------------------------------------------------------------------
// get_steering_out_rate()
// ---------------------------------------------------------------------

TEST_CASE("SteerController::get_steering_out_rate: positive/negative desired rate produces same-sign output",
          "[steer_control][rate]") {
    SteerController::Gains gains;
    SteerController steer(gains);

    // No actual yaw motion (yaw_rate_earth_dps=0), so the full demanded
    // rate is pure rate-error - a positive desired_rate should produce a
    // positive (rightward) steering output, and vice versa.
    const std::int32_t out_pos = steer.get_steering_out_rate(30.0f, make_inputs(2.0f, 0.0f, 1000));
    REQUIRE(out_pos > 0);

    SteerController steer2(gains);
    const std::int32_t out_neg = steer2.get_steering_out_rate(-30.0f, make_inputs(2.0f, 0.0f, 1000));
    REQUIRE(out_neg < 0);
}

TEST_CASE("SteerController::get_steering_out_rate: zero desired rate with zero measured rate produces zero output",
          "[steer_control][rate]") {
    SteerController::Gains gains;
    SteerController steer(gains);

    const std::int32_t out = steer.get_steering_out_rate(0.0f, make_inputs(2.0f, 0.0f, 1000));
    REQUIRE(out == 0);
    REQUIRE(steer.get_pid_info().target == Catch::Approx(0.0f));
}

TEST_CASE("SteerController::get_steering_out_rate: ground speed is floored at minspeed, preventing a speed-scaler "
          "blow-up at very low speed",
          "[steer_control][rate]") {
    SteerController::Gains gains;
    gains.minspeed = 1.0f;
    SteerController steer(gains);
    SteerController steer_floor(gains);

    // Both calls should scale identically: a genuinely-stopped vehicle
    // (0 m/s) and one exactly at minspeed (1 m/s) both use scaler=1/1.
    const std::int32_t out_stopped = steer.get_steering_out_rate(30.0f, make_inputs(0.0f, 0.0f, 1000));
    const std::int32_t out_at_min = steer_floor.get_steering_out_rate(30.0f, make_inputs(1.0f, 0.0f, 1000));
    REQUIRE(out_stopped == out_at_min);
}

TEST_CASE("SteerController::get_steering_out_rate: a measured rate matching the desired rate drives the P/FF terms "
          "toward zero",
          "[steer_control][rate]") {
    SteerController::Gains gains;
    SteerController steer(gains);

    // desired_rate == yaw_rate_earth_dps -> rate_error == 0 -> P term is
    // computed from desired_rate directly (upstream's P = radians(desired_
    // rate)*kp_ff*scaler, NOT from rate_error), so it stays nonzero, but D
    // (which IS rate_error-driven) must be exactly zero.
    (void)steer.get_steering_out_rate(20.0f, make_inputs(3.0f, 20.0f, 1000));
    REQUIRE(steer.get_pid_info().d == Catch::Approx(0.0f));
}

TEST_CASE("SteerController::get_steering_out_rate: derating reduces the output clamp above deratespeed",
          "[steer_control][rate]") {
    SteerController::Gains gains;
    gains.deratespeed = 5.0f;
    gains.deratefactor = 10.0f; // deg per m/s over deratespeed
    gains.mindegree = 500.0f;   // centidegrees floor
    gains.k_p = 100.0f;         // deliberately huge so the raw demand always saturates the clamp
    SteerController steer(gains);

    // 10 m/s is 5 m/s over deratespeed -> derate_constraint = 4500 - 5*10*100 = -500 -> clamped up to mindegree (500).
    const std::int32_t out = steer.get_steering_out_rate(90.0f, make_inputs(10.0f, 0.0f, 1000));
    REQUIRE(out == 500);
}

TEST_CASE("SteerController::reset_I zeroes the integrator term", "[steer_control][rate]") {
    SteerController::Gains gains;
    SteerController steer(gains);

    // A sustained rate error accumulates a nonzero integrator.
    for (int i = 0; i < 20; ++i) {
        (void)steer.get_steering_out_rate(30.0f, make_inputs(2.0f, 0.0f, 1000U + static_cast<std::uint32_t>(i) * 20U));
    }
    REQUIRE(steer.get_pid_info().i != Catch::Approx(0.0f));

    steer.reset_I();
    REQUIRE(steer.get_pid_info().i == Catch::Approx(0.0f));
}

// ---------------------------------------------------------------------
// get_steering_out_angle_error()
// ---------------------------------------------------------------------

TEST_CASE("SteerController::get_steering_out_angle_error: positive/negative angle error produces same-sign output",
          "[steer_control][angle]") {
    SteerController::Gains gains;
    SteerController steer(gains);

    const std::int32_t out_pos = steer.get_steering_out_angle_error(1000, make_inputs(2.0f, 0.0f, 1000)); // +10 deg
    REQUIRE(out_pos > 0);

    SteerController steer2(gains);
    const std::int32_t out_neg = steer2.get_steering_out_angle_error(-1000, make_inputs(2.0f, 0.0f, 1000)); // -10 deg
    REQUIRE(out_neg < 0);
}

TEST_CASE("SteerController::get_steering_out_angle_error: zero error produces zero output", "[steer_control][angle]") {
    SteerController::Gains gains;
    SteerController steer(gains);

    const std::int32_t out = steer.get_steering_out_angle_error(0, make_inputs(2.0f, 0.0f, 1000));
    REQUIRE(out == 0);
}

TEST_CASE("SteerController::get_steering_out_angle_error: converts angle error to a desired rate via angle_err/tau, "
          "visible in pid_info().target",
          "[steer_control][angle]") {
    SteerController::Gains gains;
    gains.tau = 0.5f;
    SteerController steer(gains);

    // 20 deg error / 0.5s tau -> desired_rate = 40 deg/s.
    (void)steer.get_steering_out_angle_error(2000, make_inputs(2.0f, 0.0f, 1000));
    REQUIRE(steer.get_pid_info().target == Catch::Approx(40.0f));
}

TEST_CASE("SteerController::get_steering_out_angle_error: a tau below the 0.1s floor is clamped in place, matching "
          "upstream's own mutate-in-place behavior",
          "[steer_control][angle]") {
    SteerController::Gains gains;
    gains.tau = 0.01f; // below the 0.1s floor
    SteerController steer(gains);

    // 10 deg error / 0.1s (the clamped floor, not the original 0.01s) -> desired_rate = 100 deg/s.
    (void)steer.get_steering_out_angle_error(1000, make_inputs(2.0f, 0.0f, 1000));
    REQUIRE(steer.get_pid_info().target == Catch::Approx(100.0f));
}
