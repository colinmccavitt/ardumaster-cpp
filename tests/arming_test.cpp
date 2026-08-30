#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/arming.hpp>
#include <fwcpp/copter/arming_leftover.hpp>

using fwcpp::copter::ArmingCopter;
using fwcpp::copter::PreArmInputs;
using fwcpp::copter::pre_arm_checks;
using fwcpp::copter::arming::PortStatus;
using fwcpp::copter::arming::completeness_has;
using fwcpp::copter::arming::completeness_size;
using fwcpp::copter::arming::on_main_count;
using fwcpp::copter::arming::out_of_scope_count;
using fwcpp::copter::arming::remaining_count;
using fwcpp::copter::arming::this_slice_count;

TEST_CASE("arming leftover catalog this_slice and remaining", "[copter][arming][leftover]") {
    REQUIRE(remaining_count() > 0);
    REQUIRE(this_slice_count() == 4);
    REQUIRE(remaining_count() == 6);
    REQUIRE(out_of_scope_count() == 1);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("pre_arm_checks", PortStatus::kThisSlice));
    REQUIRE(completeness_has("run_pre_arm_checks already_armed gate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("system_initialized check", PortStatus::kThisSlice));
    REQUIRE(completeness_has("interlock/estop conflict", PortStatus::kRemaining));
    REQUIRE(completeness_has("motor interlock enabled", PortStatus::kRemaining));
    REQUIRE(completeness_has("disarm_switch_checks", PortStatus::kRemaining));
    REQUIRE(completeness_has("motors->arming_checks", PortStatus::kRemaining));
    REQUIRE(completeness_has(
        "parameter_checks / gps / baro / board_voltage / alt / rc_throttle_failsafe",
        PortStatus::kRemaining));
    REQUIRE(completeness_has("arm() / disarm()", PortStatus::kRemaining));
    REQUIRE(completeness_has("AP:: singletons", PortStatus::kOutOfScope));
}

TEST_CASE("already armed short-circuits pre_arm", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.motors_armed = true;
    in.system_initialized = false;  // must not be checked when already armed

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.pre_arm_ran);
    REQUIRE(fx.already_armed_short_circuit);
    REQUIRE(fx.passed);
    REQUIRE_FALSE(fx.system_init_checked);
    REQUIRE_FALSE(fx.system_init_failed);
    REQUIRE(fx.set_pre_arm_check_called);
    REQUIRE(fx.set_pre_arm_check_value);
    REQUIRE(arming.pre_arm_check);
}

TEST_CASE("system not initialized fails pre_arm", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.motors_armed = false;
    in.system_initialized = false;
    in.display_failure = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.pre_arm_ran);
    REQUIRE_FALSE(fx.already_armed_short_circuit);
    REQUIRE(fx.system_init_checked);
    REQUIRE(fx.system_init_failed);
    REQUIRE(fx.check_failed_system_init);
    REQUIRE_FALSE(fx.passed);
    REQUIRE(fx.set_pre_arm_check_called);
    REQUIRE_FALSE(fx.set_pre_arm_check_value);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("system initialized passes scaffold pre_arm", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.motors_armed = false;
    in.system_initialized = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.pre_arm_ran);
    REQUIRE_FALSE(fx.already_armed_short_circuit);
    REQUIRE(fx.system_init_checked);
    REQUIRE_FALSE(fx.system_init_failed);
    REQUIRE_FALSE(fx.check_failed_system_init);
    REQUIRE(fx.passed);
    REQUIRE(fx.set_pre_arm_check_called);
    REQUIRE(fx.set_pre_arm_check_value);
    REQUIRE(arming.pre_arm_check);
}
