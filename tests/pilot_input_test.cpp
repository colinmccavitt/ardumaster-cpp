#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/pilot_input.hpp>
#include <fwcpp/math/scalar.hpp>

using Catch::Approx;
using fwcpp::copter::get_pilot_desired_climb_rate_ms;
using fwcpp::copter::get_pilot_desired_lean_angles_rad;
using fwcpp::copter::get_pilot_desired_throttle;
using fwcpp::copter::get_pilot_desired_yaw_rate_rads;
using fwcpp::copter::get_pilot_speed_dn_ms;
using fwcpp::copter::input_expo;
using fwcpp::copter::rc_input_to_roll_pitch_rad;
using fwcpp::copter::set_accel_throttle_I_from_pilot_throttle;
using fwcpp::copter::pilot::PortStatus;
using fwcpp::copter::pilot::completeness_has;
using fwcpp::copter::pilot::completeness_size;
using fwcpp::copter::pilot::on_main_count;
using fwcpp::copter::pilot::out_of_scope_count;
using fwcpp::copter::pilot::remaining_count;
using fwcpp::copter::pilot::this_slice_count;

TEST_CASE("invalid RC zeros lean and yaw", "[copter][pilot]") {
    const float angle_max = fwcpp::math::radians(45.0f);
    const auto lean =
        get_pilot_desired_lean_angles_rad(false, 1.0f, -1.0f, angle_max, angle_max);
    REQUIRE(lean.roll_rad == 0.0f);
    REQUIRE(lean.pitch_rad == 0.0f);
    REQUIRE(get_pilot_desired_yaw_rate_rads(false, 1.0f, 45.0f, 0.25f) == 0.0f);
}

TEST_CASE("center stick is ~0 lean", "[copter][pilot]") {
    const float angle_max = fwcpp::math::radians(45.0f);
    const auto lean =
        get_pilot_desired_lean_angles_rad(true, 0.0f, 0.0f, angle_max, angle_max);
    REQUIRE(lean.roll_rad == Approx(0.0f).margin(1e-6f));
    REQUIRE(lean.pitch_rad == Approx(0.0f).margin(1e-6f));
}

TEST_CASE("yaw expo vs linear", "[copter][pilot]") {
    const float rate_deg = 45.0f;
    const float stick = 0.5f;
    const float linear = get_pilot_desired_yaw_rate_rads(true, stick, rate_deg, 0.0f);
    const float expoed = get_pilot_desired_yaw_rate_rads(true, stick, rate_deg, 0.5f);
    REQUIRE(linear == Approx(fwcpp::math::radians(rate_deg) * input_expo(stick, 0.0f)));
    REQUIRE(expoed == Approx(fwcpp::math::radians(rate_deg) * input_expo(stick, 0.5f)));
    REQUIRE(linear != Approx(expoed));
    REQUIRE(std::fabs(expoed) < std::fabs(linear));
}

TEST_CASE("throttle mid_stick 500 hover 0.5 stick 500 is ~0.5", "[copter][pilot]") {
    REQUIRE(get_pilot_desired_throttle(500, 500, 0.5f) == Approx(0.5f));
    REQUIRE(get_pilot_desired_throttle(0, 500, 0.5f) == Approx(0.5f));
}

TEST_CASE("throttle hover 0.6 expo path", "[copter][pilot]") {
    const float out = get_pilot_desired_throttle(500, 500, 0.6f);
    const float expo = fwcpp::math::constrain_value(-(0.6f - 0.5f) / 0.375f, -0.5f, 1.0f);
    const float in = 0.5f;
    REQUIRE(expo != Approx(0.0f));
    REQUIRE(out == Approx(in * (1.0f - expo) + expo * in * in * in));
    REQUIRE(out == Approx(0.6f));
}

TEST_CASE("set_accel_throttle_I sign", "[copter][pilot]") {
    REQUIRE(set_accel_throttle_I_from_pilot_throttle(1.0f, 0.5f) < 0.0f);
    REQUIRE(set_accel_throttle_I_from_pilot_throttle(0.0f, 0.5f) > 0.0f);
    REQUIRE(set_accel_throttle_I_from_pilot_throttle(0.5f, 0.5f) == Approx(0.0f));
    REQUIRE(set_accel_throttle_I_from_pilot_throttle(1.0f, 0.5f) == Approx(-0.5f));
    REQUIRE(set_accel_throttle_I_from_pilot_throttle(1.5f, 0.5f) == Approx(-0.5f));
}

TEST_CASE("invalid RC zeros climb rate", "[copter][pilot]") {
    REQUIRE(get_pilot_desired_climb_rate_ms(false, 800.0f, 500.0f, 100, 150.0f, 250.0f) == 0.0f);
}

TEST_CASE("mid stick in deadzone is zero climb rate", "[copter][pilot]") {
    REQUIRE(get_pilot_desired_climb_rate_ms(true, 500.0f, 500.0f, 100, 150.0f, 250.0f) == 0.0f);
    REQUIRE(get_pilot_desired_climb_rate_ms(true, 550.0f, 500.0f, 100, 150.0f, 250.0f) == 0.0f);
    REQUIRE(get_pilot_desired_climb_rate_ms(true, 400.0f, 500.0f, 100, 150.0f, 250.0f) == 0.0f);
    REQUIRE(get_pilot_desired_climb_rate_ms(true, 600.0f, 500.0f, 100, 150.0f, 250.0f) == 0.0f);
}

TEST_CASE("below deadband is negative climb using speed_dn", "[copter][pilot]") {
    // mid=500 dz=100 -> bottom=400; thr=200: 150*(200-400)/400 = -75
    REQUIRE(get_pilot_desired_climb_rate_ms(true, 200.0f, 500.0f, 100, 150.0f, 250.0f) ==
            Approx(-75.0f));
    REQUIRE(get_pilot_speed_dn_ms(0.0f, 250.0f) == Approx(250.0f));
    REQUIRE(get_pilot_desired_climb_rate_ms(true, 200.0f, 500.0f, 100, 0.0f, 250.0f) ==
            Approx(-125.0f));
}

TEST_CASE("above deadband is positive climb using speed_up", "[copter][pilot]") {
    // mid=500 dz=100 -> top=600; thr=800: 250*(800-600)/(1000-600) = 125
    REQUIRE(get_pilot_desired_climb_rate_ms(true, 800.0f, 500.0f, 100, 150.0f, 250.0f) ==
            Approx(125.0f));
}

TEST_CASE("leftover remaining_count", "[copter][pilot][leftover]") {
    REQUIRE(remaining_count() == 3);
    REQUIRE(remaining_count() > 0);
    REQUIRE(this_slice_count() == 3);
    REQUIRE(on_main_count() == 6);
    REQUIRE(out_of_scope_count() == 0);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("get_pilot_desired_lean_angles", PortStatus::kOnMain));
    REQUIRE(completeness_has("get_pilot_desired_yaw_rate", PortStatus::kOnMain));
    REQUIRE(completeness_has("get_pilot_desired_throttle", PortStatus::kOnMain));
    REQUIRE(completeness_has("rc_input_to_roll_pitch_rad", PortStatus::kOnMain));
    REQUIRE(completeness_has("input_expo", PortStatus::kOnMain));
    REQUIRE(completeness_has("set_accel_throttle_I", PortStatus::kOnMain));
    REQUIRE(completeness_has("get_pilot_desired_climb_rate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("get_pilot_speed_dn", PortStatus::kThisSlice));
    REQUIRE(completeness_has("AutoYaw state machine", PortStatus::kRemaining));
    REQUIRE(completeness_has("weathervane", PortStatus::kRemaining));
    REQUIRE(completeness_has("get_pilot_desired_velocity", PortStatus::kRemaining));
}

TEST_CASE("rc_input_to_roll_pitch_rad matches tan/atan mapping", "[copter][pilot]") {
    const float angle_max = fwcpp::math::radians(45.0f);
    float roll = 99.0f;
    float pitch = 99.0f;
    rc_input_to_roll_pitch_rad(0.0f, 0.0f, angle_max, angle_max, roll, pitch);
    REQUIRE(roll == Approx(0.0f).margin(1e-6f));
    REQUIRE(pitch == Approx(0.0f).margin(1e-6f));

    rc_input_to_roll_pitch_rad(1.0f, 0.0f, angle_max, angle_max, roll, pitch);
    REQUIRE(roll == Approx(angle_max).margin(1e-5f));
    REQUIRE(pitch == Approx(0.0f).margin(1e-6f));
}
