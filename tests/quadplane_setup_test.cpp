#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_defaults.hpp>
#include <fwcpp/quadplane/quadplane_frame.hpp>

using fwcpp::quadplane::MotorFrameClass;
using fwcpp::quadplane::MotorsKind;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::QuadPlaneSetupInputs;
using fwcpp::quadplane::VtolAirframe;
using fwcpp::quadplane::classify_frame;
using fwcpp::quadplane::kQEnableDefault;
using fwcpp::quadplane::kQFrameClassDefault;
using fwcpp::quadplane::motor_frame_class_as_u8;

TEST_CASE("setup core", "[quadplane]") {
    QuadPlane qp;
    REQUIRE(qp.enable() == kQEnableDefault);
    REQUIRE_FALSE(qp.setup());
    QuadPlane on{1};
    REQUIRE(on.setup());
    REQUIRE(on.available());
    REQUIRE(on.setup());
    QuadPlane auto_en{2};
    REQUIRE(auto_en.setup());
    on.set_enable(0);
    REQUIRE(on.available());
}

TEST_CASE("frame class", "[quadplane]") {
    QuadPlane qp{1};
    qp.set_frame_class(motor_frame_class_as_u8(MotorFrameClass::kTailsitter));
    REQUIRE(qp.setup());
    REQUIRE(qp.motors_kind() == MotorsKind::kTailsitter);
    QuadPlane bad{1};
    bad.set_frame_class(motor_frame_class_as_u8(MotorFrameClass::kHeli));
    REQUIRE_FALSE(bad.setup());
    QuadPlane conflict{1};
    conflict.set_tailsit_enable(1);
    conflict.set_tilt_enable(1);
    REQUIRE_FALSE(conflict.setup());
    QuadPlaneSetupInputs armed;
    armed.soft_armed = true;
    QuadPlane arm{1};
    REQUIRE_FALSE(arm.setup(armed));
    const auto sel = classify_frame(kQFrameClassDefault, 0, 0);
    REQUIRE(sel->airframe == VtolAirframe::kMulticopter);
}