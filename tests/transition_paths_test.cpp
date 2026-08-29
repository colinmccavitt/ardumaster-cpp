#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane_transition/transition_paths.hpp>
#include <fwcpp/quadplane_transition/transition_state.hpp>

using fwcpp::quadplane_transition::TransitionPhase;
using fwcpp::quadplane_transition::TransitionState;
using fwcpp::quadplane_transition::active_forward_transition;
using fwcpp::quadplane_transition::kMavVtolStateFw;
using fwcpp::quadplane_transition::kMavVtolStateMc;
using fwcpp::quadplane_transition::kMavVtolStateTransitionToFw;
using fwcpp::quadplane_transition::mav_vtol_state_slt;
using fwcpp::quadplane_transition::show_vtol_view_slt;
using fwcpp::quadplane_transition::transition_phase_from_inputs;

TEST_CASE("active forward and mav vtol mapping", "[transition][paths]") {
    REQUIRE(active_forward_transition(true, TransitionState::kTimer, false));
    REQUIRE_FALSE(active_forward_transition(true, TransitionState::kTimer, true));
    REQUIRE(show_vtol_view_slt(true));
    REQUIRE(mav_vtol_state_slt(TransitionState::kDone, false, false) == kMavVtolStateFw);
    REQUIRE(mav_vtol_state_slt(TransitionState::kTimer, false, false) == kMavVtolStateTransitionToFw);
    REQUIRE(mav_vtol_state_slt(TransitionState::kDone, true, false) == kMavVtolStateMc);
    REQUIRE(transition_phase_from_inputs(TransitionState::kDone, false, false) == TransitionPhase::kAir);
}
