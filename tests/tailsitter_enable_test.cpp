#include <catch2/catch_test_macros.hpp>
#include <fwcpp/tailsitter/tailsitter.hpp>

using fwcpp::tailsitter::TailsitterGate;
using fwcpp::tailsitter::TailsitterSetupInputs;
using fwcpp::tailsitter::TiltrotorType;
using fwcpp::tailsitter::kMotorFrameTailsitter;
using fwcpp::tailsitter::resolve_setup;

TEST_CASE("tailsitter enable heuristic", "[tailsitter][enable]") {
    TailsitterSetupInputs in{};
    in.frame_class = kMotorFrameTailsitter;
    const auto setup = resolve_setup(in);
    REQUIRE(setup.enable == 1);
    const auto gate = TailsitterGate::from_setup(setup);
    REQUIRE(gate.enabled());
}

TEST_CASE("tailsitter bicopter blocks heuristic", "[tailsitter][enable]") {
    TailsitterSetupInputs in{};
    in.frame_class = kMotorFrameTailsitter;
    in.tiltrotor_type = TiltrotorType::kBicopter;
    const auto setup = resolve_setup(in);
    REQUIRE(setup.enable == 0);
    REQUIRE_FALSE(TailsitterGate::from_setup(setup).enabled());
}

TEST_CASE("explicit enable zero stays off", "[tailsitter][enable]") {
    TailsitterSetupInputs in{};
    in.enable = 0;
    in.frame_class = kMotorFrameTailsitter;
    const auto setup = resolve_setup(in);
    REQUIRE(setup.enable == 0);
    REQUIRE_FALSE(setup.setup_complete);
}
