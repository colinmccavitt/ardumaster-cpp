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
    (void)fsm.force_transition_complete();
    REQUIRE(fsm.complete());
    fsm.restart();
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    REQUIRE(fsm.in_transition());
}

TEST_CASE("force complete stamps pitch and requests assist reset", "[transition][fsm]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.enter_timer();
    const auto effects = fsm.force_transition_complete(5000U, -1200);
    REQUIRE(fsm.complete());
    REQUIRE(effects.assist_reset);
    REQUIRE(effects.last_fw_pitch_stamped);
    REQUIRE(fsm.assist_reset_pending());
    REQUIRE(fsm.last_fw_mode_ms() == 5000U);
    REQUIRE(fsm.last_fw_nav_pitch_cd() == -1200);
    fsm.clear_assist_reset_pending();
    REQUIRE_FALSE(fsm.assist_reset_pending());
}

TEST_CASE("tiltrotor fwd completes timer but not airspeed wait", "[transition][fsm]") {
    SltTransition fsm = SltTransition::with_defaults();
    REQUIRE_FALSE(fsm.try_complete_tiltrotor_fwd(true));
    fsm.enter_timer();
    REQUIRE(fsm.try_complete_tiltrotor_fwd(true));
    REQUIRE(fsm.complete());
}

TEST_CASE("vtol update prepares next transition", "[transition][fsm]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.force_transition_complete();
    fsm.vtol_update(false, true, 0.42f);
    REQUIRE(fsm.state() == TransitionState::kAirspeedWait);
    REQUIRE(fsm.last_throttle() == 0.42f);
    REQUIRE(fsm.assist_reset_pending());
    fsm.clear_assist_reset_pending();
    fsm.vtol_update(true, false, 0.1f);
    REQUIRE(fsm.complete());
}
