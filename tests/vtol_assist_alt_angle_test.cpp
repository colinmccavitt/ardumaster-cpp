#include <catch2/catch_test_macros.hpp>
#include <fwcpp/vtol_assist/assist_triggers.hpp>
#include <fwcpp/vtol_assist/should_assist.hpp>

using fwcpp::vtol_assist::AssistInputs;
using fwcpp::vtol_assist::AssistState;
using fwcpp::vtol_assist::ShouldAssistGates;
using fwcpp::vtol_assist::ShouldAssistHysteresis;
using fwcpp::vtol_assist::SpeedAssistSample;
using fwcpp::vtol_assist::VtolAssist;
using fwcpp::vtol_assist::evaluate_alt_assist_trigger;
using fwcpp::vtol_assist::evaluate_angle_assist_trigger;
using fwcpp::vtol_assist::inside_angle_error;
using fwcpp::vtol_assist::inside_attitude_envelope;
using fwcpp::vtol_assist::should_assist;

static ShouldAssistGates open_gates() {
    ShouldAssistGates g;
    g.armed_and_safety_off = true;
    g.assist_state = AssistState::kAssistEnabled;
    g.does_auto_throttle = true;
    g.is_flying = true;
    return g;
}

TEST_CASE("alt assist threshold", "[vtol_assist][alt]") {
    VtolAssist assist = VtolAssist::with_defaults();
    assist.set_speed(8.0f);
    assist.set_alt(15);
    REQUIRE(evaluate_alt_assist_trigger(assist, 10.0f));
    REQUIRE_FALSE(evaluate_alt_assist_trigger(assist, 20.0f));
    assist.set_alt(0);
    REQUIRE_FALSE(evaluate_alt_assist_trigger(assist, 5.0f));
    assist.set_speed(0.0f);
    assist.set_alt(15);
    REQUIRE_FALSE(evaluate_alt_assist_trigger(assist, 5.0f));
}

TEST_CASE("angle assist envelope and error", "[vtol_assist][angle]") {
    VtolAssist assist = VtolAssist::with_defaults();
    assist.set_speed(8.0f);
    assist.set_angle(30);
    REQUIRE(inside_attitude_envelope(20.0f, 5.0f, 45.0f, 20.0f, -15.0f));
    REQUIRE(inside_angle_error(25.0f, 5.0f, 25.0f, 5.0f, 30));
    REQUIRE_FALSE(evaluate_angle_assist_trigger(assist, 0, 0, 25.0f, 5.0f, 45.0f, 20.0f, -15.0f));
    REQUIRE(evaluate_angle_assist_trigger(assist, 0, 0, 80.0f, 5.0f, 45.0f, 20.0f, -15.0f));
    assist.set_angle(0);
    REQUIRE_FALSE(evaluate_angle_assist_trigger(assist, 0, 0, 80.0f, 5.0f, 45.0f, 20.0f, -15.0f));
}

TEST_CASE("should_assist alt hysteresis and speed interaction", "[vtol_assist][should_assist]") {
    VtolAssist assist = VtolAssist::with_defaults();
    assist.set_speed(8.0f);
    assist.set_alt(15);
    assist.set_delay(0.5f);
    auto gates = open_gates();
    ShouldAssistHysteresis h;
    AssistInputs in{};
    in.speed.have_airspeed = true;
    in.speed.aspeed = 12.0f;
    in.height_above_ground_m = 10.0f;

    auto r = should_assist(assist, gates, in, 100, h);
    REQUIRE_FALSE(r.requested);
    REQUIRE_FALSE(r.alt_assist);

    r = should_assist(assist, gates, in, 700, h);
    REQUIRE(r.alt_assist);
    REQUIRE(r.requested);
    REQUIRE_FALSE(r.speed_assist);

    in.speed.aspeed = 5.0f;
    r = should_assist(assist, gates, in, 800, h);
    REQUIRE(r.speed_assist);
    REQUIRE(r.alt_assist);
    REQUIRE(r.requested);

    in.height_above_ground_m = 30.0f;
    r = should_assist(assist, gates, in, 900, h);
    REQUIRE(r.speed_assist);
    REQUIRE(r.alt_assist);

    r = should_assist(assist, gates, in, 2000, h);
    REQUIRE(r.speed_assist);
    REQUIRE_FALSE(r.alt_assist);
}

TEST_CASE("should_assist angle hysteresis", "[vtol_assist][should_assist]") {
    VtolAssist assist = VtolAssist::with_defaults();
    assist.set_speed(8.0f);
    assist.set_angle(10);
    assist.set_delay(0.5f);
    auto gates = open_gates();
    ShouldAssistHysteresis h;
    AssistInputs in{};
    in.speed.have_airspeed = true;
    in.speed.aspeed = 12.0f;
    in.roll_limit_deg = 45.0f;
    in.pitch_limit_max_deg = 20.0f;
    in.pitch_limit_min_deg = -15.0f;
    in.ahrs_roll_deg = 80.0f;
    in.ahrs_pitch_deg = 5.0f;

    auto r = should_assist(assist, gates, in, 100, h);
    REQUIRE_FALSE(r.angle_assist);
    r = should_assist(assist, gates, in, 700, h);
    REQUIRE(r.angle_assist);
    REQUIRE(r.angle_assist_first_edge);
    REQUIRE(r.requested);
}
