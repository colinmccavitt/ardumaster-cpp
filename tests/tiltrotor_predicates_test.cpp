#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <fwcpp/tiltrotor/tiltrotor.hpp>

using fwcpp::tiltrotor::TiltType;
using fwcpp::tiltrotor::TiltrotorGate;
using fwcpp::tiltrotor::TiltrotorSetupInputs;
using fwcpp::tiltrotor::get_fully_forward_tilt;
using fwcpp::tiltrotor::is_continuous_type;
using fwcpp::tiltrotor::is_motor_tilting;
using fwcpp::tiltrotor::is_vectored_type;
using fwcpp::tiltrotor::resolve_setup;
using fwcpp::tiltrotor::tilt_angle_achieved;

TEST_CASE("vectored yaw requires mask and type", "[tiltrotor][predicates]") {
    REQUIRE(is_vectored_type(TiltType::kVectoredYaw, 0x3u));
    REQUIRE_FALSE(is_vectored_type(TiltType::kContinuous, 0x3u));
    REQUIRE_FALSE(is_vectored_type(TiltType::kVectoredYaw, 0u));

    const auto setup = resolve_setup(TiltrotorSetupInputs{.tilt_mask = 0x3u, .type = TiltType::kVectoredYaw});
    REQUIRE(setup.is_vectored);
}

TEST_CASE("motor tilting bitmask", "[tiltrotor][predicates]") {
    REQUIRE(is_motor_tilting(0x5u, 0));
    REQUIRE_FALSE(is_motor_tilting(0x5u, 1));
    REQUIRE(is_motor_tilting(0x5u, 2));
}

TEST_CASE("tilt angle achieved and forward tilt", "[tiltrotor][predicates]") {
    const auto gate = TiltrotorGate::from_setup(resolve_setup(TiltrotorSetupInputs{.tilt_mask = 1u}));
    REQUIRE(tilt_angle_achieved(gate, TiltType::kBinary, false));
    REQUIRE_FALSE(tilt_angle_achieved(gate, TiltType::kContinuous, false));
    REQUIRE(tilt_angle_achieved(gate, TiltType::kContinuous, true));
    REQUIRE(is_continuous_type(TiltType::kContinuous));
    REQUIRE(get_fully_forward_tilt(9.0f) == Catch::Approx(0.9f));
}
