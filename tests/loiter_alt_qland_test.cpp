#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_loiter/loiter_alt_qland.hpp>

using fwcpp::q_loiter::GuidedAltFrame;
using fwcpp::q_loiter::LoiterAltQlandEnterAction;
using fwcpp::q_loiter::LoiterAltQlandEnterEffects;
using fwcpp::q_loiter::LoiterAltQlandEnterInputs;
using fwcpp::q_loiter::LoiterAltQlandSwitchAction;
using fwcpp::q_loiter::LoiterAltQlandSwitchInputs;
using fwcpp::q_loiter::loiter_alt_qland_enter;
using fwcpp::q_loiter::loiter_alt_qland_switch;

TEST_CASE("loiter alt qland enter", "[q_loiter][loiter_alt]") {
    LoiterAltQlandEnterEffects effects{};
    LoiterAltQlandEnterInputs in{};
    in.in_vtol_mode = true;
    auto r = loiter_alt_qland_enter(in, effects);
    REQUIRE(r.action == LoiterAltQlandEnterAction::kSwitchQlandImmediate);
    REQUIRE(effects.request_qland_mode);
    in.in_vtol_mode = false;
    effects = {};
    r = loiter_alt_qland_enter(in, effects);
    REQUIRE(r.action == LoiterAltQlandEnterAction::kFwLoiterThenGuided);
    REQUIRE(effects.handle_guided_request);
    in.terrain_enabled = true;
    r = loiter_alt_qland_enter(in, effects);
    REQUIRE(r.guided_alt_frame == GuidedAltFrame::kAboveTerrain);
}

TEST_CASE("loiter alt qland switch", "[q_loiter][loiter_alt]") {
    LoiterAltQlandSwitchInputs in{};
    in.height_above_valid = false;
    in.reached_loiter_target = true;
    REQUIRE(loiter_alt_qland_switch(in) == LoiterAltQlandSwitchAction::kSwitchQland);
    in.height_above_m = -1.0F;
    in.height_above_valid = true;
    REQUIRE(loiter_alt_qland_switch(in) == LoiterAltQlandSwitchAction::kSwitchQland);
    in.height_above_m = 5.0F;
    REQUIRE(loiter_alt_qland_switch(in) == LoiterAltQlandSwitchAction::kNone);
}
