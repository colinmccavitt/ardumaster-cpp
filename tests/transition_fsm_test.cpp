#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane_transition/transition_fsm.hpp>
#include <fwcpp/quadplane_transition/transition_state.hpp>

using fwcpp::quadplane_transition::SltTransition;
using fwcpp::quadplane_transition::TransitionState;
using fwcpp::quadplane_transition::can_transition;

TEST_CASE("slt state discriminants match upstream", "[transition][fsm]") {
    REQUIRE(static_cast<std::uint8_t>(TransitionState::kAirspeedWait) == 0);
    REQUIRE(static_cast<std::uint8_t>(TransitionState::kTimer) == 1);
    REQUIRE(static_cast<std::uint8_t>(TransitionState::kDone) == 2);
}

TEST_CASE("new zero inits to airspeed wait", "[transition][fsm]") {
    const SltTransition fsm = SltTransition::with_defaults();
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    REQUIRE(fsm.get_log_transition_state() == 0);
    REQUIRE_FALSE(fsm.complete());
    REQUIRE(fsm.in_transition());
    REQUIRE_FALSE(fsm.in_forced_transition());
}

TEST_CASE("can_transition allows forward and assist-back paths", "[transition][fsm]") {
    REQUIRE(can_transition(TransitionState::kAirspeedWait, TransitionState::kTimer));
    REQUIRE(can_transition(TransitionState::kAirspeedWait, TransitionState::kDone));
    REQUIRE(can_transition(TransitionState::kTimer, TransitionState::kDone));
    REQUIRE(can_transition(TransitionState::kTimer, TransitionState::kAirspeedWait));
    REQUIRE(can_transition(TransitionState::kDone, TransitionState::kAirspeedWait));
    REQUIRE_FALSE(can_transition(TransitionState::kDone, TransitionState::kTimer));
}

TEST_CASE("set_state rejects illegal moves", "[transition][fsm]") {
    SltTransition fsm = SltTransition::with_defaults();
    REQUIRE(fsm.set_state(TransitionState::kTimer));
    REQUIRE(fsm.state() == TransitionState::kTimer);
    REQUIRE(fsm.set_state(TransitionState::kDone));
    REQUIRE(fsm.complete());
    REQUIRE_FALSE(fsm.set_state(TransitionState::kTimer));
}

TEST_CASE("restart and force_transition_complete", "[transition][fsm]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.enter_timer();
    fsm.force_transition_complete();
    REQUIRE(fsm.complete());
    fsm.restart();
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    REQUIRE(fsm.in_transition());
}
