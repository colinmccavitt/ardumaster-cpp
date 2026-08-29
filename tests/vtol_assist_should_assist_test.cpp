#include <catch2/catch_test_macros.hpp>
#include <fwcpp/vtol_assist/should_assist.hpp>

using fwcpp::vtol_assist::AssistState;
using fwcpp::vtol_assist::FlareMode;
using fwcpp::vtol_assist::ShouldAssistGates;
using fwcpp::vtol_assist::SpeedAssistSample;
using fwcpp::vtol_assist::VtolAssist;
using fwcpp::vtol_assist::should_assist_gates_open;
using fwcpp::vtol_assist::should_assist_slice1;

static ShouldAssistGates open_gates() {
    ShouldAssistGates g;
    g.armed_and_safety_off = true;
    g.assist_state = AssistState::kAssistEnabled;
    g.does_auto_throttle = true;
    g.is_flying = true;
    return g;
}

TEST_CASE("should_assist gates", "[vtol_assist][should_assist]") {
    auto g = open_gates();
    REQUIRE(should_assist_gates_open(g));
    g.armed_and_safety_off = false;
    REQUIRE_FALSE(should_assist_gates_open(g));
    g = open_gates();
    g.flare_mode = FlareMode::kActive;
    REQUIRE_FALSE(should_assist_gates_open(g));
}

TEST_CASE("should_assist slice1 speed trigger", "[vtol_assist][should_assist]") {
    VtolAssist assist = VtolAssist::with_defaults();
    assist.set_speed(8.0f);
    auto gates = open_gates();
    SpeedAssistSample s{};
    s.have_airspeed = true;
    s.aspeed = 5.0f;
    auto r = should_assist_slice1(assist, gates, s);
    REQUIRE(r.speed_assist);
    REQUIRE(r.requested);
    s.aspeed = 12.0f;
    r = should_assist_slice1(assist, gates, s);
    REQUIRE_FALSE(r.speed_assist);
    gates.assist_state = AssistState::kForceEnabled;
    r = should_assist_slice1(assist, gates, s);
    REQUIRE(r.force_assist);
    REQUIRE(r.requested);
}

TEST_CASE("should_assist fly_inverted skips angle hysteresis", "[vtol_assist][should_assist]") {
    using fwcpp::vtol_assist::AssistInputs;
    using fwcpp::vtol_assist::ShouldAssistHysteresis;
    using fwcpp::vtol_assist::should_assist;

    VtolAssist assist = VtolAssist::with_defaults();
    assist.set_speed(8.0f);
    assist.set_angle(30);
    assist.set_delay(0.5f);
    auto gates = open_gates();
    gates.fly_inverted = true;
    ShouldAssistHysteresis h;
    AssistInputs in{};
    in.speed.have_airspeed = true;
    in.speed.aspeed = 12.0f;
    in.ahrs_roll_deg = 80.0f;
    in.ahrs_pitch_deg = 5.0f;
    in.roll_limit_deg = 45.0f;
    in.pitch_limit_max_deg = 20.0f;
    in.pitch_limit_min_deg = -15.0f;

    auto r = should_assist(assist, gates, in, 700, h);
    REQUIRE_FALSE(r.angle_assist);
    REQUIRE_FALSE(r.requested);
}
