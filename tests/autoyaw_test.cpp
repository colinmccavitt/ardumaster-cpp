#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/copter/autoyaw.hpp>
#include <fwcpp/copter/pilot_input.hpp>

using Catch::Approx;
using fwcpp::control::HeadingMode;
using fwcpp::copter::AutoYaw;
using fwcpp::copter::kWeathervaneEnabled;
using fwcpp::copter::pilot::PortStatus;
using fwcpp::copter::pilot::completeness_has;
using fwcpp::copter::pilot::remaining_count;

TEST_CASE("valid input + nonzero yaw rate -> PILOT_RATE", "[copter][autoyaw]") {
    AutoYaw yaw;
    REQUIRE(yaw.mode() == AutoYaw::Mode::LOOK_AT_NEXT_WP);

    const float rate = 0.4f;
    const auto heading = yaw.get_heading(true, true, rate);

    REQUIRE(yaw.mode() == AutoYaw::Mode::PILOT_RATE);
    REQUIRE(heading.heading_mode == HeadingMode::Rate_Only);
    REQUIRE(heading.yaw_rate_rads == Approx(rate));
    REQUIRE(yaw.pilot_yaw_rate_rads() == Approx(rate));
}

TEST_CASE("valid input + zero rate stays previous mode", "[copter][autoyaw]") {
    AutoYaw yaw;
    REQUIRE(yaw.mode() == AutoYaw::Mode::LOOK_AT_NEXT_WP);

    const auto heading = yaw.get_heading(true, true, 0.0f);

    REQUIRE(yaw.mode() == AutoYaw::Mode::LOOK_AT_NEXT_WP);
    REQUIRE(yaw.mode() != AutoYaw::Mode::PILOT_RATE);
    REQUIRE(heading.heading_mode == HeadingMode::Angle_And_Rate);
}

TEST_CASE("invalid input while PILOT_RATE -> HOLD", "[copter][autoyaw]") {
    AutoYaw yaw;
    (void)yaw.get_heading(true, true, 0.5f);
    REQUIRE(yaw.mode() == AutoYaw::Mode::PILOT_RATE);

    const auto heading = yaw.get_heading(false, true, 0.5f);

    REQUIRE(yaw.mode() == AutoYaw::Mode::HOLD);
    REQUIRE(heading.heading_mode == HeadingMode::Rate_Only);
    REQUIRE(heading.yaw_rate_rads == Approx(0.0f));
}

TEST_CASE("use_pilot_yaw disabled while PILOT_RATE -> HOLD", "[copter][autoyaw]") {
    AutoYaw yaw;
    (void)yaw.get_heading(true, true, 0.5f);
    REQUIRE(yaw.mode() == AutoYaw::Mode::PILOT_RATE);

    (void)yaw.get_heading(true, false, 0.5f);
    REQUIRE(yaw.mode() == AutoYaw::Mode::HOLD);
}

TEST_CASE("weathervane not invoked", "[copter][autoyaw]") {
    AutoYaw yaw;
    REQUIRE_FALSE(kWeathervaneEnabled);
    (void)yaw.get_heading(true, true, 0.3f);
    REQUIRE_FALSE(yaw.weathervane_invoked());
}

TEST_CASE("leftover remaining_count==0", "[copter][autoyaw][leftover]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(completeness_has("weathervane", PortStatus::kThisSlice));
    REQUIRE(completeness_has("get_pilot_desired_velocity", PortStatus::kOnMain));
    REQUIRE(completeness_has("AutoYaw state machine", PortStatus::kOnMain));
}

TEST_CASE("get_yaw_out success -> WEATHERVANE + stored rate", "[copter][autoyaw][weathervane]") {
    AutoYaw yaw;
    const float wv_rate = 0.25f;
    yaw.update_weathervane(0.1f, true, true, wv_rate, AutoYaw::Mode::LOOK_AT_NEXT_WP);

    REQUIRE(yaw.weathervane_invoked());
    REQUIRE(yaw.mode() == AutoYaw::Mode::WEATHERVANE);
    REQUIRE(yaw.rate_rads() == Approx(wv_rate));
    REQUIRE_FALSE(yaw.used_default_mode_leftover());
}

TEST_CASE("get_yaw_out fail while WEATHERVANE + last HOLD -> default_mode leftover",
          "[copter][autoyaw][weathervane]") {
    AutoYaw yaw;
    yaw.set_mode(AutoYaw::Mode::HOLD);
    yaw.update_weathervane(0.0f, true, true, 0.2f, AutoYaw::Mode::LOOK_AT_NEXT_WP);
    REQUIRE(yaw.mode() == AutoYaw::Mode::WEATHERVANE);
    REQUIRE(yaw.last_mode() == AutoYaw::Mode::HOLD);

    yaw.update_weathervane(0.0f, true, false, 0.0f, AutoYaw::Mode::LOOK_AT_NEXT_WP);

    REQUIRE(yaw.used_default_mode_leftover());
    REQUIRE(yaw.mode() == AutoYaw::Mode::LOOK_AT_NEXT_WP);
    REQUIRE(yaw.rate_rads() == Approx(0.0f));
}

TEST_CASE("get_yaw_out fail while WEATHERVANE restores last_mode when not HOLD",
          "[copter][autoyaw][weathervane]") {
    AutoYaw yaw;
    yaw.set_mode(AutoYaw::Mode::RATE);
    yaw.update_weathervane(0.1f, true, true, 0.3f, AutoYaw::Mode::LOOK_AT_NEXT_WP);
    REQUIRE(yaw.mode() == AutoYaw::Mode::WEATHERVANE);
    REQUIRE(yaw.last_mode() == AutoYaw::Mode::RATE);

    yaw.update_weathervane(0.1f, true, false, 0.0f, AutoYaw::Mode::LOOK_AT_NEXT_WP);

    REQUIRE(yaw.mode() == AutoYaw::Mode::RATE);
    REQUIRE_FALSE(yaw.used_default_mode_leftover());
    REQUIRE(yaw.rate_rads() == Approx(0.0f));
}
