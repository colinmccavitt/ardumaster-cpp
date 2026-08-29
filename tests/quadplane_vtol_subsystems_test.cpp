#include <catch2/catch_test_macros.hpp>

#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_frame.hpp>
#include <fwcpp/quadplane/quadplane_vtol_subsystems.hpp>
#include <fwcpp/tiltrotor/tiltrotor_types.hpp>

using fwcpp::quadplane::MotorFrameClass;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::QuadPlaneSetupInputs;
using fwcpp::quadplane::ThrustType;
using fwcpp::quadplane::TransitionKind;
using fwcpp::quadplane::VtolAirframe;
using fwcpp::quadplane::VtolSubsystemWireInputs;
using fwcpp::quadplane::motor_frame_class_as_u8;
using fwcpp::quadplane::wire_vtol_subsystems;
using fwcpp::tiltrotor::TiltType;

TEST_CASE("wire_vtol_subsystems", "[quadplane][vtol_subsystems]") {
    VtolSubsystemWireInputs tilt{};
    tilt.tilt_enable = 0;
    tilt.tilt_mask = 0x3;
    const auto tr = wire_vtol_subsystems(tilt);
    REQUIRE(tr.ok);
    REQUIRE(tr.resolved_tilt_enable > 0);
    REQUIRE(tr.tiltrotor.enabled());
    REQUIRE(tr.thrust_type == ThrustType::kTiltrotor);
    REQUIRE(tr.transition_kind == TransitionKind::kTiltrotor);

    VtolSubsystemWireInputs conflict{};
    conflict.tailsit_enable = 1;
    conflict.tilt_enable = 1;
    conflict.tilt_mask = 1;
    REQUIRE_FALSE(wire_vtol_subsystems(conflict).ok);
}

TEST_CASE("wire_vtol_subsystems SLT when neither enabled", "[quadplane][vtol_subsystems]") {
    const auto slt = wire_vtol_subsystems({});
    REQUIRE(slt.ok);
    REQUIRE(slt.thrust_type == ThrustType::kSlt);
    REQUIRE(slt.transition_kind == TransitionKind::kSlt);
    REQUIRE_FALSE(slt.tailsitter.enabled());
    REQUIRE_FALSE(slt.tiltrotor.enabled());
}

TEST_CASE("QuadPlane setup wires tailsitter tiltrotor gates", "[quadplane][vtol_subsystems]") {
    QuadPlane qp{1};
    qp.set_tilt_enable(0);
    qp.set_tailsit_enable(0);
    qp.set_frame_class(motor_frame_class_as_u8(MotorFrameClass::kTailsitter));
    REQUIRE(qp.setup());
    REQUIRE(qp.tailsitter().enabled());
    REQUIRE(qp.vtol_airframe() == VtolAirframe::kTailsitter);
    REQUIRE(qp.subsystems().thrust_type == ThrustType::kTailsitter);
    REQUIRE(qp.subsystems().transition_kind == TransitionKind::kTailsitter);

    QuadPlane tilt{1};
    QuadPlaneSetupInputs in{};
    in.tilt_mask = 0x1;
    in.tilt_type = TiltType::kContinuous;
    REQUIRE(tilt.setup(in));
    REQUIRE(tilt.tiltrotor().enabled());
    REQUIRE(tilt.vtol_airframe() == VtolAirframe::kTiltrotor);
    REQUIRE(tilt.subsystems().thrust_type == ThrustType::kTiltrotor);
    REQUIRE(tilt.subsystems().transition_kind == TransitionKind::kTiltrotor);
}
