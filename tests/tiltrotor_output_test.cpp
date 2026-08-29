#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include <fwcpp/tiltrotor/tiltrotor.hpp>

using fwcpp::tiltrotor::BicopterOutputInputs;
using fwcpp::tiltrotor::BicopterOutputPath;
using fwcpp::tiltrotor::ForwardThrottleInputs;
using fwcpp::tiltrotor::TiltType;
using fwcpp::tiltrotor::TiltrotorGate;
using fwcpp::tiltrotor::TiltrotorSetupInputs;
using fwcpp::tiltrotor::bicopter_constrain_tilt;
using fwcpp::tiltrotor::bicopter_output;
using fwcpp::tiltrotor::bicopter_scale_negative_tilt;
using fwcpp::tiltrotor::fully_fwd;
using fwcpp::tiltrotor::get_forward_throttle;
using fwcpp::tiltrotor::kMaxNumMotors;
using fwcpp::tiltrotor::kServoMax;
using fwcpp::tiltrotor::kThrottleScaledToUnit;
using fwcpp::tiltrotor::resolve_setup;

namespace {

TiltrotorGate enabled_gate(std::uint16_t tilt_mask = 0x3u, TiltType type = TiltType::kBicopter) {
    TiltrotorSetupInputs setup{};
    setup.enable = 1;
    setup.tilt_mask = tilt_mask;
    setup.type = type;
    return TiltrotorGate::from_setup(resolve_setup(setup));
}

TiltrotorGate disabled_gate() {
    return TiltrotorGate::from_setup(resolve_setup(TiltrotorSetupInputs{}));
}

BicopterOutputInputs bicopter_hover() {
    BicopterOutputInputs in{};
    in.type = TiltType::kBicopter;
    in.gate = enabled_gate();
    in.tilt_mask = 0x3u;
    in.in_vtol_mode = true;
    in.current_tilt = 0.0f;
    return in;
}

}  // namespace

TEST_CASE("bicopter_output motor-test and non-bicopter are no-ops", "[tiltrotor][bicopter]") {
    auto in = bicopter_hover();
    in.motor_test_running = true;
    in.throttle_scaled = 50.0f;
    in.tilt_left = 1000.0f;
    const auto motor_test = bicopter_output(in);
    REQUIRE(motor_test.path == BicopterOutputPath::kNone);
    REQUIRE_FALSE(motor_test.write_tilt_servos);
    REQUIRE_FALSE(motor_test.hold_stabilize);
    REQUIRE_FALSE(motor_test.motors_output);

    in = bicopter_hover();
    in.type = TiltType::kContinuous;
    in.tilt_left = 1000.0f;
    const auto not_bicopter = bicopter_output(in);
    REQUIRE(not_bicopter.path == BicopterOutputPath::kNone);
    REQUIRE_FALSE(not_bicopter.write_tilt_servos);
    REQUIRE_FALSE(not_bicopter.motors_output);

    in.type = TiltType::kVectoredYaw;
    REQUIRE(bicopter_output(in).path == BicopterOutputPath::kNone);
}

TEST_CASE("bicopter_output fully_fwd FW path sets -SERVO_MAX", "[tiltrotor][bicopter]") {
    BicopterOutputInputs in{};
    in.type = TiltType::kBicopter;
    in.gate = enabled_gate();
    in.tilt_mask = 0x3u;
    in.in_vtol_mode = false;
    in.current_tilt = 1.0f;
    in.assisted_flight = true;
    in.throttle_scaled = 80.0f;
    in.tilt_left = 1234.0f;
    in.tilt_right = -500.0f;
    REQUIRE(fully_fwd(in.gate, in.tilt_mask, in.current_tilt, in.flap_angle_deg));

    const auto out = bicopter_output(in);
    REQUIRE(out.path == BicopterOutputPath::kFullyFwdFixedWing);
    REQUIRE(out.write_tilt_servos);
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(-kServoMax, 1e-6f));
    REQUIRE_THAT(out.tilt_right, Catch::Matchers::WithinAbs(-kServoMax, 1e-6f));
    REQUIRE_FALSE(out.hold_stabilize);
    REQUIRE_FALSE(out.motors_output);
}

TEST_CASE("bicopter_output fully_fwd uses existing predicate", "[tiltrotor][bicopter]") {
    BicopterOutputInputs in{};
    in.type = TiltType::kBicopter;
    in.gate = enabled_gate();
    in.tilt_mask = 0x3u;
    in.in_vtol_mode = false;
    in.current_tilt = 0.5f;
    REQUIRE_FALSE(fully_fwd(in.gate, in.tilt_mask, in.current_tilt, in.flap_angle_deg));
    REQUIRE(bicopter_output(in).path == BicopterOutputPath::kUnassisted);

    in.tilt_mask = 0u;
    in.current_tilt = 1.0f;
    REQUIRE_FALSE(fully_fwd(in.gate, in.tilt_mask, in.current_tilt, in.flap_angle_deg));
    REQUIRE(bicopter_output(in).path == BicopterOutputPath::kUnassisted);
}

TEST_CASE("bicopter_output assisted vs unassisted motors flags", "[tiltrotor][bicopter]") {
    auto in = bicopter_hover();
    in.assisted_flight = true;
    in.throttle_scaled = 75.0f;
    const auto assisted = bicopter_output(in);
    REQUIRE(assisted.path == BicopterOutputPath::kAssisted);
    REQUIRE(assisted.hold_stabilize);
    REQUIRE_THAT(assisted.hold_stabilize_throttle,
                 Catch::Matchers::WithinAbs(75.0f * kThrottleScaledToUnit, 1e-6f));
    REQUIRE(assisted.motors_output);
    REQUIRE(assisted.motors_output_assisted);
    REQUIRE(assisted.write_tilt_servos);

    in.assisted_flight = false;
    const auto unassisted = bicopter_output(in);
    REQUIRE(unassisted.path == BicopterOutputPath::kUnassisted);
    REQUIRE_FALSE(unassisted.hold_stabilize);
    REQUIRE(unassisted.motors_output);
    REQUIRE_FALSE(unassisted.motors_output_assisted);
}

TEST_CASE("bicopter_output negative tilt uses yaw-angle scale", "[tiltrotor][bicopter]") {
    auto in = bicopter_hover();
    in.tilt_yaw_angle = 9.0f;
    in.tilt_left = -1000.0f;
    in.tilt_right = 1000.0f;
    const auto out = bicopter_output(in);
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(-100.0f, 1e-4f));
    REQUIRE_THAT(out.tilt_right, Catch::Matchers::WithinAbs(1000.0f, 1e-4f));

    REQUIRE_THAT(bicopter_scale_negative_tilt(-1000.0f, 9.0f), Catch::Matchers::WithinAbs(-100.0f, 1e-6f));
    REQUIRE_THAT(bicopter_scale_negative_tilt(1000.0f, 9.0f), Catch::Matchers::WithinAbs(1000.0f, 1e-6f));
}

TEST_CASE("bicopter_output scales by cos(current_tilt * pi/2) then constrains", "[tiltrotor][bicopter]") {
    auto in = bicopter_hover();
    in.current_tilt = 0.5f;
    in.tilt_left = 1000.0f;
    in.tilt_right = -2000.0f;
    in.tilt_yaw_angle = 90.0f;
    const float scaling = std::cos(0.5f * static_cast<float>(M_PI_2));
    const float expected_left = bicopter_constrain_tilt(0.5f, 1000.0f * scaling);
    const float expected_right = bicopter_constrain_tilt(0.5f, -2000.0f * scaling);
    const auto out = bicopter_output(in);
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(expected_left, 1e-4f));
    REQUIRE_THAT(out.tilt_right, Catch::Matchers::WithinAbs(expected_right, 1e-4f));

    in.current_tilt = 0.0f;
    in.tilt_left = 6000.0f;
    in.tilt_right = -6000.0f;
    in.tilt_yaw_angle = 90.0f;
    const auto clamped = bicopter_output(in);
    REQUIRE_THAT(clamped.tilt_left, Catch::Matchers::WithinAbs(kServoMax, 1e-6f));
    REQUIRE_THAT(clamped.tilt_right, Catch::Matchers::WithinAbs(-kServoMax, 1e-6f));
}

TEST_CASE("get_forward_throttle disabled non-vectored zero-range", "[tiltrotor][fwd_thr]") {
    ForwardThrottleInputs in{};
    in.gate = disabled_gate();
    in.is_vectored = true;
    in.tilt_mask = 0x1u;
    in.motors.ok[0] = true;
    in.motors.thrust[0] = 0.5f;
    REQUIRE_FALSE(get_forward_throttle(in).ok);

    in.gate = enabled_gate(0x3u, TiltType::kVectoredYaw);
    in.is_vectored = false;
    REQUIRE_FALSE(get_forward_throttle(in).ok);

    in.is_vectored = true;
    in.thr_lin.spin_min = 0.5f;
    in.thr_lin.spin_max = 0.5f;
    REQUIRE_FALSE(get_forward_throttle(in).ok);

    in.thr_lin.spin_min = 0.9f;
    in.thr_lin.spin_max = 0.1f;
    REQUIRE_FALSE(get_forward_throttle(in).ok);
}

TEST_CASE("get_forward_throttle no tilting motors or failed get_thrust", "[tiltrotor][fwd_thr]") {
    ForwardThrottleInputs in{};
    in.gate = enabled_gate(0x3u, TiltType::kVectoredYaw);
    in.is_vectored = true;
    in.thr_lin.spin_min = 0.1f;
    in.thr_lin.spin_max = 0.9f;
    in.tilt_mask = 0u;
    in.motors.ok[0] = true;
    in.motors.thrust[0] = 1.0f;
    REQUIRE_FALSE(get_forward_throttle(in).ok);

    in.tilt_mask = 0x3u;
    in.motors.ok[0] = false;
    in.motors.ok[1] = false;
    REQUIRE_FALSE(get_forward_throttle(in).ok);
}

TEST_CASE("get_forward_throttle averages tilting motors that report thrust", "[tiltrotor][fwd_thr]") {
    ForwardThrottleInputs in{};
    in.gate = enabled_gate(0x5u, TiltType::kVectoredYaw);
    in.is_vectored = true;
    in.tilt_mask = 0x5u;
    in.thr_lin.spin_min = 0.1f;
    in.thr_lin.spin_max = 0.9f;
    in.motors.ok[0] = true;
    in.motors.thrust[0] = 0.5f;
    in.motors.ok[2] = true;
    in.motors.thrust[2] = 1.0f;
    in.motors.ok[1] = true;
    in.motors.thrust[1] = 0.0f;

    const float range = 0.8f;
    const float a0 = (in.thr_lin.thrust_to_actuator(0.5f) - 0.1f) / range;
    const float a2 = (in.thr_lin.thrust_to_actuator(1.0f) - 0.1f) / range;
    const auto out = get_forward_throttle(in);
    REQUIRE(out.ok);
    REQUIRE_THAT(out.throttle, Catch::Matchers::WithinAbs((a0 + a2) * 0.5f, 1e-6f));
    REQUIRE_THAT(a0, Catch::Matchers::WithinAbs(0.5f, 1e-6f));
    REQUIRE_THAT(a2, Catch::Matchers::WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("get_forward_throttle scan cap is 32", "[tiltrotor][fwd_thr]") {
    REQUIRE(kMaxNumMotors == 32);
    ForwardThrottleInputs in{};
    in.gate = enabled_gate(0x1u, TiltType::kVectoredYaw);
    in.is_vectored = true;
    in.tilt_mask = 0x1u;
    in.thr_lin.spin_min = 0.0f;
    in.thr_lin.spin_max = 1.0f;
    in.motors.ok[0] = true;
    in.motors.thrust[0] = 0.25f;
    const auto out = get_forward_throttle(in);
    REQUIRE(out.ok);
    REQUIRE_THAT(out.throttle, Catch::Matchers::WithinAbs(0.25f, 1e-6f));
}
