#include <catch2/catch_test_macros.hpp>

#include <fwcpp/tiltrotor/tiltrotor.hpp>

using fwcpp::tiltrotor::TiltType;
using fwcpp::tiltrotor::TiltrotorSetupFlagInputs;
using fwcpp::tiltrotor::TiltrotorSetupInputs;
using fwcpp::tiltrotor::kTiltServoRange;
using fwcpp::tiltrotor::resolve_have_fw_motor;
using fwcpp::tiltrotor::resolve_setup;
using fwcpp::tiltrotor::resolve_setup_flags;
using fwcpp::tiltrotor::resolve_setup_with_flags;

TEST_CASE("tiltrotor enable<=0 skips leftover setup flags", "[tiltrotor][setup]") {
    TiltrotorSetupInputs setup_in{};
    setup_in.enable = 0;
    setup_in.tilt_mask = 0x3u;
    setup_in.type = TiltType::kVectoredYaw;
    const auto setup = resolve_setup(setup_in);
    REQUIRE(setup.enable == 0);
    REQUIRE_FALSE(setup.setup_complete);
    REQUIRE_FALSE(setup.is_vectored);

    TiltrotorSetupFlagInputs flags_in{};
    flags_in.throttle.throttle = true;
    flags_in.throttle.throttle_left = true;
    flags_in.motor_enabled = 0xFu;
    flags_in.num_motors = 4;
    flags_in.tilt_mask = 0x3u;
    flags_in.type = TiltType::kVectoredYaw;

    const auto flags = resolve_setup_flags(setup, flags_in);
    REQUIRE_FALSE(flags.set_thrust_tiltrotor);
    REQUIRE_FALSE(flags.is_vectored);
    REQUIRE_FALSE(flags.have_fw_motor);
    REQUIRE_FALSE(flags.have_vtol_motor);
    REQUIRE_FALSE(flags.disable_yaw_torque);
    REQUIRE_FALSE(flags.bind_thrust_compensation);
    REQUIRE_FALSE(flags.tilt_servo_range.any());
}

TEST_CASE("tiltrotor have_fw_motor excludes bicopter left/right throttle", "[tiltrotor][setup]") {
    REQUIRE(resolve_have_fw_motor({.throttle = true}, TiltType::kBicopter));
    REQUIRE_FALSE(resolve_have_fw_motor({.throttle_left = true}, TiltType::kBicopter));
    REQUIRE_FALSE(resolve_have_fw_motor({.throttle_right = true}, TiltType::kBicopter));
    REQUIRE_FALSE(resolve_have_fw_motor(
        {.throttle_left = true, .throttle_right = true}, TiltType::kBicopter));
    REQUIRE(resolve_have_fw_motor({.throttle_left = true}, TiltType::kContinuous));
    REQUIRE(resolve_have_fw_motor({.throttle_right = true}, TiltType::kVectoredYaw));

    TiltrotorSetupInputs setup_in{};
    setup_in.enable = 1;
    setup_in.type = TiltType::kBicopter;
    const auto setup = resolve_setup(setup_in);

    TiltrotorSetupFlagInputs flags_in{};
    flags_in.throttle.throttle_left = true;
    flags_in.type = TiltType::kBicopter;
    const auto flags = resolve_setup_flags(setup, flags_in);
    REQUIRE(flags.set_thrust_tiltrotor);
    REQUIRE_FALSE(flags.have_fw_motor);
}

TEST_CASE("tiltrotor vtol motor is enabled bit outside tilt_mask", "[tiltrotor][setup]") {
    TiltrotorSetupInputs setup_in{};
    setup_in.enable = 1;
    setup_in.tilt_mask = 0x3u;
    const auto setup = resolve_setup(setup_in);

    TiltrotorSetupFlagInputs flags_in{};
    flags_in.tilt_mask = 0x3u;
    flags_in.num_motors = 4;
    flags_in.motor_enabled = 0x4u;
    REQUIRE(resolve_setup_flags(setup, flags_in).have_vtol_motor);

    flags_in.motor_enabled = 0x3u;
    REQUIRE_FALSE(resolve_setup_flags(setup, flags_in).have_vtol_motor);

    flags_in.motor_enabled = 0x4u;
    flags_in.num_motors = 2;
    REQUIRE_FALSE(resolve_setup_flags(setup, flags_in).have_vtol_motor);
}

TEST_CASE("tiltrotor vectored yaw disables torque and sets five ranges", "[tiltrotor][setup]") {
    TiltrotorSetupInputs setup_in{};
    setup_in.enable = 1;
    setup_in.tilt_mask = 0x3u;
    setup_in.type = TiltType::kVectoredYaw;

    TiltrotorSetupFlagInputs flags_in{};
    flags_in.tilt_mask = 0x3u;
    flags_in.type = TiltType::kVectoredYaw;

    const auto both = resolve_setup_with_flags(setup_in, flags_in);
    REQUIRE(both.setup.enable == 1);
    REQUIRE(both.setup.is_vectored);
    REQUIRE(both.flags.set_thrust_tiltrotor);
    REQUIRE(both.flags.is_vectored);
    REQUIRE(both.flags.disable_yaw_torque);
    REQUIRE(both.flags.bind_thrust_compensation);
    REQUIRE(both.flags.tilt_servo_range.all());
    REQUIRE(kTiltServoRange == 1000);
}

TEST_CASE("tiltrotor non-vectored tilt_mask binds compensate without yaw ranges",
          "[tiltrotor][setup]") {
    TiltrotorSetupInputs setup_in{};
    setup_in.enable = 1;
    setup_in.tilt_mask = 0x5u;
    setup_in.type = TiltType::kContinuous;
    const auto setup = resolve_setup(setup_in);
    REQUIRE_FALSE(setup.is_vectored);

    TiltrotorSetupFlagInputs flags_in{};
    flags_in.tilt_mask = 0x5u;
    flags_in.type = TiltType::kContinuous;

    const auto flags = resolve_setup_flags(setup, flags_in);
    REQUIRE(flags.set_thrust_tiltrotor);
    REQUIRE_FALSE(flags.is_vectored);
    REQUIRE_FALSE(flags.disable_yaw_torque);
    REQUIRE(flags.bind_thrust_compensation);
    REQUIRE_FALSE(flags.tilt_servo_range.any());
}
