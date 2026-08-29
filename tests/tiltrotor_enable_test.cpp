#include <catch2/catch_test_macros.hpp>
#include <fwcpp/tiltrotor/tiltrotor.hpp>

using fwcpp::tiltrotor::TiltType;
using fwcpp::tiltrotor::TiltrotorGate;
using fwcpp::tiltrotor::TiltrotorSetupInputs;
using fwcpp::tiltrotor::resolve_setup;

TEST_CASE("tiltrotor enable heuristic from mask", "[tiltrotor][enable]") {
    const auto setup = resolve_setup(TiltrotorSetupInputs{.tilt_mask = 0x3u});
    REQUIRE(setup.enable == 1);
    REQUIRE(setup.setup_complete);
    REQUIRE(TiltrotorGate::from_setup(setup).enabled());
}

TEST_CASE("tiltrotor bicopter type heuristic", "[tiltrotor][enable]") {
    const auto setup = resolve_setup(TiltrotorSetupInputs{.type = TiltType::kBicopter});
    REQUIRE(setup.enable == 1);
    REQUIRE(TiltrotorGate::from_setup(setup).enabled());
}

TEST_CASE("explicit enable zero stays off", "[tiltrotor][enable]") {
    TiltrotorSetupInputs in{};
    in.enable = 0;
    in.tilt_mask = 0x3u;
    const auto setup = resolve_setup(in);
    REQUIRE(setup.enable == 0);
    REQUIRE_FALSE(setup.setup_complete);
    REQUIRE_FALSE(TiltrotorGate::from_setup(setup).enabled());
}
