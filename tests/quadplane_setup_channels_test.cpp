#include <catch2/catch_test_macros.hpp>

#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_frame.hpp>
#include <fwcpp/quadplane/quadplane_setup_channels.hpp>

using fwcpp::quadplane::AhrsViewRotation;
using fwcpp::quadplane::MotorFrameClass;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::QuadPlaneSetupInputs;
using fwcpp::quadplane::motor_frame_class_as_u8;
using fwcpp::quadplane::setup_default_channels;
using fwcpp::quadplane::wire_setup_channels;

TEST_CASE("setup channel wiring stub", "[quadplane][setup_channels]") {
    fwcpp::quadplane::SetupChannelsSink sink{};
    setup_default_channels(4, sink);
    REQUIRE(sink.motor_default_count == 4);
    REQUIRE(sink.motor_defaults[0].motor_function_index == 0);
    REQUIRE(sink.motor_defaults[0].channel == 5);
    REQUIRE(sink.motor_defaults[3].channel == 8);

    wire_setup_channels(motor_frame_class_as_u8(MotorFrameClass::kHexa), sink);
    REQUIRE(sink.motor_default_count == 6);

    wire_setup_channels(motor_frame_class_as_u8(MotorFrameClass::kTri), sink);
    REQUIRE(sink.tri_frame_param_flags);
    REQUIRE(sink.motor_default_count == 4);
    REQUIRE(sink.motor_defaults[2].motor_function_index == 3);
    REQUIRE(sink.motor_defaults[2].channel == 8);

    wire_setup_channels(motor_frame_class_as_u8(MotorFrameClass::kTailsitter), sink);
    REQUIRE(sink.motor_default_count == 0);

    QuadPlane qp{1};
    QuadPlaneSetupInputs inputs{};
    inputs.ahrs_view.trim_pitch_rad = 0.05f;
    qp.set_tailsit_enable(1);
    REQUIRE(qp.setup(inputs));
    REQUIRE(qp.ahrs_view_inited());
    REQUIRE(qp.ahrs_view().rotation == AhrsViewRotation::kPitch90);
    REQUIRE(qp.ahrs_view().trim_pitch_rad == 0.05f);
    REQUIRE(qp.setup_channels().motor_default_count == 4);

    QuadPlane ts{1};
    ts.set_frame_class(motor_frame_class_as_u8(MotorFrameClass::kTailsitter));
    REQUIRE(ts.setup());
    REQUIRE(ts.setup_channels().motor_default_count == 0);

    QuadPlane quad{1};
    REQUIRE(quad.setup());
    REQUIRE(quad.setup_channels().motor_default_count == 4);
    REQUIRE(quad.setup_channels().motor_defaults[0].channel == 5);
}
