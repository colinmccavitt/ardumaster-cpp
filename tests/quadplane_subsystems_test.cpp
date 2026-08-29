#include <catch2/catch_test_macros.hpp>

#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_subsystems.hpp>

using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::ThrustType;
using fwcpp::quadplane::TransitionKind;
using fwcpp::quadplane::VtolSubsystemsSetupInputs;
using fwcpp::quadplane::wire_vtol_subsystems_setup;
using fwcpp::quadplane::wire_vtol_subsystems_update;

TEST_CASE("vtol subsystems setup wiring", "[quadplane][subsystems]") {
    QuadPlane qp(1);
    qp.set_tilt_enable(1);
    REQUIRE(qp.setup());
    REQUIRE(qp.subsystems().wired);
    REQUIRE(qp.subsystems().thrust_type == ThrustType::kTiltrotor);
    REQUIRE(qp.subsystems().transition_kind == TransitionKind::kTiltrotor);
    REQUIRE(qp.subsystems().tiltrotor.enabled());
}

TEST_CASE("tailsitter setup assigns tailsitter thrust and transition", "[quadplane][subsystems]") {
    QuadPlane qp(1);
    qp.set_tailsit_enable(1);
    REQUIRE(qp.setup());
    REQUIRE(qp.subsystems().thrust_type == ThrustType::kTailsitter);
    REQUIRE(qp.subsystems().transition_kind == TransitionKind::kTailsitter);
    REQUIRE(qp.subsystems().tailsitter.enabled());
}

TEST_CASE("default setup stays SLT thrust and SLT transition", "[quadplane][subsystems]") {
    QuadPlane qp(1);
    REQUIRE(qp.setup());
    REQUIRE(qp.subsystems().wired);
    REQUIRE(qp.subsystems().thrust_type == ThrustType::kSlt);
    REQUIRE(qp.subsystems().transition_kind == TransitionKind::kSlt);
}

TEST_CASE("tiltrotor update wired on quadplane update", "[quadplane][subsystems]") {
    QuadPlane qp(1);
    qp.set_tilt_enable(1);
    REQUIRE(qp.setup());
    const auto tick = qp.update(fwcpp::quadplane::QuadPlaneUpdateView{.now_ms = 1, .in_vtol_mode = true});
    REQUIRE(tick.ran_tiltrotor_update);
    REQUIRE(qp.subsystems().tiltrotor_update_ticks >= 1);
}

TEST_CASE("wire helpers idle when not enabled", "[quadplane][subsystems]") {
    fwcpp::quadplane::VtolSubsystemsState state{};
    wire_vtol_subsystems_setup(state, VtolSubsystemsSetupInputs{});
    REQUIRE(state.wired);
    REQUIRE(state.thrust_type == ThrustType::kSlt);
    REQUIRE(state.transition_kind == TransitionKind::kSlt);
    REQUIRE_FALSE(state.tailsitter.enabled());
    const auto tick = wire_vtol_subsystems_update(state);
    REQUIRE_FALSE(tick.ran_tiltrotor_update);
}
