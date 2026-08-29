#include <catch2/catch_test_macros.hpp>

#include <fwcpp/tailsitter/tailsitter_control.hpp>
#include <fwcpp/tailsitter/tailsitter_setup.hpp>

using fwcpp::tailsitter::TailsitterGate;
using fwcpp::tailsitter::TailsitterSetupInputs;
using fwcpp::tailsitter::TailsitterTransitionState;
using fwcpp::tailsitter::TransitionRamp;
using fwcpp::tailsitter::check_input_remapped_roll;
using fwcpp::tailsitter::check_input_remapped_yaw;
using fwcpp::tailsitter::get_transition_angle_vtol;
using fwcpp::tailsitter::in_vtol_transition;
using fwcpp::tailsitter::kTailsitterInputPlane;
using fwcpp::tailsitter::resolve_setup;
using fwcpp::tailsitter::tailsitter_active;

TEST_CASE("tailsitter control active and check_input", "[tailsitter][control]") {
    const auto setup = resolve_setup(TailsitterSetupInputs{.enable = 1});
    const TailsitterGate gate = TailsitterGate::from_setup(setup);
    REQUIRE(tailsitter_active(gate, true, false));
    REQUIRE(tailsitter_active(gate, false, true));
    REQUIRE(!tailsitter_active(gate, false, false));

    const std::int8_t input = static_cast<std::int8_t>(kTailsitterInputPlane);
    REQUIRE(check_input_remapped_roll(100, 200, gate, input, true, false) == 200);
    REQUIRE(check_input_remapped_yaw(100, 200, gate, input, true, false) == -100);
    REQUIRE(check_input_remapped_roll(100, 200, gate, 0, true, false) == 100);
}

TEST_CASE("tailsitter in_vtol_transition and angle fallback", "[tailsitter][control]") {
    TransitionRamp ramp{};
    REQUIRE(get_transition_angle_vtol(ramp) == ramp.angle_fw);
    ramp.angle_vtol = 60;
    REQUIRE(get_transition_angle_vtol(ramp) == 60);

    REQUIRE(in_vtol_transition(true, true, TailsitterTransitionState::kAngleWaitVtol, 500, 0));
    REQUIRE(in_vtol_transition(true, true, TailsitterTransitionState::kDone, 2000, 0));
    REQUIRE(!in_vtol_transition(true, false, TailsitterTransitionState::kDone, 2000, 0));
}
