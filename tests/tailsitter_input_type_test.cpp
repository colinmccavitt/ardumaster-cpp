#include <catch2/catch_test_macros.hpp>
#include <fwcpp/tailsitter/tailsitter.hpp>

using fwcpp::tailsitter::InputType;
using fwcpp::tailsitter::TailsitterGate;
using fwcpp::tailsitter::TailsitterInputContext;
using fwcpp::tailsitter::TailsitterSetupInputs;
using fwcpp::tailsitter::input_body_frame_roll;
using fwcpp::tailsitter::input_plane_mode;
using fwcpp::tailsitter::is_control_surface_tailsitter;
using fwcpp::tailsitter::is_vectored;
using fwcpp::tailsitter::kMotorFrameTailsitter;
using fwcpp::tailsitter::kTailsitterInputBfRoll;
using fwcpp::tailsitter::kTailsitterInputPlane;
using fwcpp::tailsitter::resolve_input_type;
using fwcpp::tailsitter::resolve_setup;

TEST_CASE("vectored vs control surface paths", "[tailsitter][input_type]") {
    TailsitterInputContext ctx{};
    ctx.frame_class = kMotorFrameTailsitter;
    ctx.gate = TailsitterGate::from_setup(resolve_setup(TailsitterSetupInputs{.frame_class = kMotorFrameTailsitter}));
    ctx.vectored_hover_gain = 0.5f;
    ctx.tilt_motor_left = true;
    REQUIRE(is_vectored(ctx));
    REQUIRE_FALSE(is_control_surface_tailsitter(ctx));
    REQUIRE(resolve_input_type(ctx) == InputType::kVectoredYaw);

    ctx.tilt_motor_left = false;
    ctx.vectored_hover_gain = 0.5f;
    REQUIRE(is_control_surface_tailsitter(ctx));
    REQUIRE(resolve_input_type(ctx) == InputType::kControlSurfaces);
}

TEST_CASE("input bitmask helpers", "[tailsitter][input_type]") {
    const std::int8_t both = static_cast<std::int8_t>(kTailsitterInputPlane | kTailsitterInputBfRoll);
    REQUIRE(input_plane_mode(both));
    REQUIRE(input_body_frame_roll(both));
}
