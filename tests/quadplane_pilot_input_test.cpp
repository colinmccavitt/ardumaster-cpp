#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_pilot_input.hpp>
#include <fwcpp/tailsitter/tailsitter_defaults.hpp>

using Catch::Approx;
using fwcpp::quadplane::DesiredSpoolState;
using fwcpp::quadplane::DesiredYawRateInputs;
using fwcpp::quadplane::HoldHoverInputs;
using fwcpp::quadplane::PilotClimbRateInputs;
using fwcpp::quadplane::PilotLandThrottleInputs;
using fwcpp::quadplane::PilotLeanAngleInputs;
using fwcpp::quadplane::PilotThrottleInputs;
using fwcpp::quadplane::PilotYawRateInputs;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::RudderArming;
using fwcpp::quadplane::StickMixing;
using fwcpp::quadplane::get_desired_yaw_rate_cds;
using fwcpp::quadplane::get_pilot_desired_climb_rate_cms;
using fwcpp::quadplane::get_pilot_desired_lean_angles;
using fwcpp::quadplane::get_pilot_input_yaw_rate_cds;
using fwcpp::quadplane::get_pilot_land_throttle;
using fwcpp::quadplane::get_pilot_throttle;
using fwcpp::quadplane::get_pilot_velocity_z_max_dn_m;
using fwcpp::quadplane::hold_hover;
using fwcpp::quadplane::input_expo;
using fwcpp::quadplane::kCommandModelPilotRateDefault;
using fwcpp::quadplane::kRuddDtGainDefault;
using fwcpp::quadplane::throttle_curve;

TEST_CASE("pilot throttle expo vs linear", "[quadplane][pilot][hover]") {
    PilotThrottleInputs linear{.control_in = 25.0f, .range = 100.0f, .throttle_expo = 0.0f};
    REQUIRE(get_pilot_throttle(linear) == Approx(0.25f));

    PilotThrottleInputs expo{.control_in = 25.0f, .range = 100.0f, .throttle_expo = 0.2f, .throttle_hover = 0.5f};
    const float curved = throttle_curve(0.5f, 0.2f, 0.25f);
    REQUIRE(get_pilot_throttle(expo) == Approx(curved));
    REQUIRE(curved != Approx(0.25f));

    QuadPlane qp{1};
    qp.set_throttle_expo(0.0f);
    REQUIRE(qp.get_pilot_throttle({.control_in = 50.0f, .range = 100.0f}) == Approx(0.5f));
}

TEST_CASE("pilot failsafe lean zeros", "[quadplane][pilot][hover]") {
    PilotLeanAngleInputs in{.roll_control_in = 2000.0f, .pitch_control_in = -1500.0f};
    in.rc_failsafe = true;
    const auto fs = get_pilot_desired_lean_angles(in);
    REQUIRE(fs.roll_out_cd == 0.0f);
    REQUIRE(fs.pitch_out_cd == 0.0f);

    in.rc_failsafe = false;
    in.throttle_counter_active = true;
    const auto thr_fs = get_pilot_desired_lean_angles(in);
    REQUIRE(thr_fs.roll_out_cd == 0.0f);
    REQUIRE(thr_fs.pitch_out_cd == 0.0f);
}

TEST_CASE("pilot circular lean limit", "[quadplane][pilot][hover]") {
    PilotLeanAngleInputs in{
        .roll_control_in = 4500.0f,
        .pitch_control_in = 4500.0f,
        .angle_max_cd = 4500.0f,
        .angle_limit_cd = 4500.0f,
    };
    const auto out = get_pilot_desired_lean_angles(in);
    const float limited = 4500.0f / std::sqrt(2.0f);
    REQUIRE(out.pitch_out_cd == Approx(limited).margin(1e-3f));
    const float expected_roll =
        100.0f * fwcpp::math::degrees(std::atan(std::cos(fwcpp::math::cd_to_rad(limited)) *
                                                std::tan(fwcpp::math::cd_to_rad(limited))));
    REQUIRE(out.roll_out_cd == Approx(expected_roll).margin(1e-3f));
    REQUIRE(std::hypot(out.pitch_out_cd, limited) == Approx(4500.0f).margin(1e-2f));
}

TEST_CASE("pilot land throttle failsafe", "[quadplane][pilot][hover]") {
    PilotLandThrottleInputs in{.rc_failsafe_active = true, .control_in = 80.0f, .range = 100.0f};
    REQUIRE(get_pilot_land_throttle(in) == 0.0f);
    in.rc_failsafe_active = false;
    REQUIRE(get_pilot_land_throttle(in) == Approx(0.8f));
    in.control_in = 150.0f;
    REQUIRE(get_pilot_land_throttle(in) == Approx(1.0f));
}

TEST_CASE("pilot yaw disarm gate", "[quadplane][pilot][hover]") {
    PilotYawRateInputs in{};
    in.rudder_in = -4500.0f;
    in.throttle_input = 0.0f;
    in.does_auto_throttle = false;
    in.rudder_arming = RudderArming::kArmDisarm;
    in.velocity_z_up_cms = 0.0f;
    in.command_model_expo = 0.0f;
    REQUIRE(get_pilot_input_yaw_rate_cds(in) == 0.0f);

    in.throttle_input = 10.0f;
    REQUIRE(get_pilot_input_yaw_rate_cds(in) ==
            Approx(input_expo(-1.0f, 0.0f) * kCommandModelPilotRateDefault * 100.0f));
}

TEST_CASE("pilot stick_mixing none in qrtl", "[quadplane][pilot][hover]") {
    PilotYawRateInputs in{};
    in.rudder_in = 2250.0f;
    in.throttle_input = 50.0f;
    in.stick_mixing = StickMixing::kNone;
    in.mode_is_qrtl = true;
    REQUIRE(get_pilot_input_yaw_rate_cds(in) == 0.0f);

    in.mode_is_qrtl = false;
    REQUIRE(get_pilot_input_yaw_rate_cds(in) != 0.0f);
}

TEST_CASE("pilot tailsitter rudd_dt scale", "[quadplane][pilot][hover]") {
    PilotYawRateInputs in{};
    in.rudder_in = 4500.0f;
    in.throttle_input = 50.0f;
    in.command_model_expo = 0.0f;
    in.in_vtol_mode = false;
    in.tailsitter_enabled = true;
    in.rudd_dt_gain = kRuddDtGainDefault;
    const float scaled = get_pilot_input_yaw_rate_cds(in);
    const float expected = input_expo(1.0f, 0.0f) * (kCommandModelPilotRateDefault * kRuddDtGainDefault * 0.01f) * 100.0f;
    REQUIRE(scaled == Approx(expected));

    in.tailsitter_enabled = false;
    const float unscaled = get_pilot_input_yaw_rate_cds(in);
    REQUIRE(unscaled == Approx(input_expo(1.0f, 0.0f) * kCommandModelPilotRateDefault * 100.0f));
    REQUIRE(scaled == Approx(unscaled * (kRuddDtGainDefault * 0.01f)));
}

TEST_CASE("pilot climb invalid RC is -50", "[quadplane][pilot][hover]") {
    PilotClimbRateInputs in{.has_valid_input = false, .throttle_request = 0.4f};
    REQUIRE(get_pilot_desired_climb_rate_cms(in) == -50.0f);
}

TEST_CASE("pilot dn-zero fallback", "[quadplane][pilot][hover]") {
    REQUIRE(get_pilot_velocity_z_max_dn_m(0.0f, 2.50f) == 2);
    REQUIRE(get_pilot_velocity_z_max_dn_m(3.0f, 2.50f) == 3);
    REQUIRE(get_pilot_velocity_z_max_dn_m(-4.0f, 2.50f) == 4);

    QuadPlane qp{1};
    qp.set_pilot_speed_z_max_up_ms(2.50f);
    qp.set_pilot_speed_z_max_dn_ms(0.0f);
    REQUIRE(qp.get_pilot_velocity_z_max_dn_m() == 2);
}

TEST_CASE("hold_hover tick effects", "[quadplane][pilot][hover]") {
    HoldHoverInputs in{};
    in.target_climb_rate_cms = 250.0f;
    in.pilot_speed_z_max_up_ms = 2.50f;
    in.pilot_speed_z_max_dn_ms = 0.0f;
    in.pilot_accel_z_mss = 2.5f;
    in.yaw.pilot.throttle_input = 50.0f;
    in.yaw.pilot.rudder_in = 0.0f;
    in.yaw.should_weathervane = true;
    in.yaw.weathervane_yaw_rate_cds = 99.0f;

    const auto tick = hold_hover(in);
    REQUIRE(tick.desired_spool == DesiredSpoolState::kThrottleUnlimited);
    REQUIRE(tick.d_max_speed_dn_m == Approx(2.0f));
    REQUIRE(tick.d_max_speed_up_ms == Approx(2.50f));
    REQUIRE(tick.d_max_accel_z_mss == Approx(2.5f));
    REQUIRE(tick.multicopter_attitude_rate_update);
    REQUIRE(tick.run_z_controller);
    REQUIRE(tick.climb_rate_ms == Approx(2.50f));
    REQUIRE(tick.desired_yaw_rate_cds == Approx(0.0f));

    QuadPlane qp{1};
    qp.set_pilot_speed_z_max_up_ms(2.50f);
    qp.set_pilot_speed_z_max_dn_ms(0.0f);
    qp.set_pilot_accel_z_mss(2.5f);
    DesiredYawRateInputs yaw{};
    yaw.pilot.throttle_input = 50.0f;
    const auto wired = qp.hold_hover(100.0f, yaw);
    REQUIRE(wired.desired_spool == DesiredSpoolState::kThrottleUnlimited);
    REQUIRE(wired.climb_rate_ms == Approx(1.0f));
    REQUIRE(wired.run_z_controller);
}

TEST_CASE("pilot desired yaw adds assisted and weathervane", "[quadplane][pilot][hover]") {
    DesiredYawRateInputs in{};
    in.assisted_flight = true;
    in.desired_auto_yaw_rate_cds = 30.0f;
    in.should_weathervane = true;
    in.weathervane_yaw_rate_cds = 20.0f;
    in.pilot.throttle_input = 50.0f;
    in.pilot.rudder_in = 0.0f;
    REQUIRE(get_desired_yaw_rate_cds(in) == Approx(50.0f));
}
