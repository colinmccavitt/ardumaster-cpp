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
