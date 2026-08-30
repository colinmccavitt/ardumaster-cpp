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
    REQUIRE(this_slice_count() == 7);
    REQUIRE(remaining_count() == 3);
    REQUIRE(out_of_scope_count() == 1);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("pre_arm_checks", PortStatus::kThisSlice));
    REQUIRE(completeness_has("run_pre_arm_checks already_armed gate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("system_initialized check", PortStatus::kThisSlice));
    REQUIRE(completeness_has("interlock/estop conflict", PortStatus::kThisSlice));
    REQUIRE(completeness_has("motor interlock enabled", PortStatus::kThisSlice));
    REQUIRE(completeness_has("disarm_switch_checks", PortStatus::kThisSlice));
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
    in.has_motor_interlock_option = true;
    in.has_motor_estop_option = true;
    in.using_interlock = true;
    in.motor_interlock_switch = true;
    in.has_disarm_switch_option = true;
    in.disarm_switch_high = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.pre_arm_ran);
    REQUIRE(fx.already_armed_short_circuit);
    REQUIRE(fx.passed);
    REQUIRE_FALSE(fx.system_init_checked);
    REQUIRE_FALSE(fx.system_init_failed);
    REQUIRE_FALSE(fx.interlock_estop_conflict_checked);
    REQUIRE_FALSE(fx.motor_interlock_enabled_checked);
    REQUIRE_FALSE(fx.disarm_switch_checked);
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
    in.has_motor_interlock_option = true;
    in.has_motor_estop_option = true;
    in.has_disarm_switch_option = true;
    in.disarm_switch_high = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.pre_arm_ran);
    REQUIRE_FALSE(fx.already_armed_short_circuit);
    REQUIRE(fx.system_init_checked);
    REQUIRE(fx.system_init_failed);
    REQUIRE(fx.check_failed_system_init);
    REQUIRE_FALSE(fx.interlock_estop_conflict_checked);
    REQUIRE_FALSE(fx.motor_interlock_enabled_checked);
    REQUIRE_FALSE(fx.disarm_switch_checked);
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
    REQUIRE(fx.interlock_estop_conflict_checked);
    REQUIRE_FALSE(fx.interlock_estop_conflict_failed);
    REQUIRE(fx.motor_interlock_enabled_checked);
    REQUIRE_FALSE(fx.motor_interlock_enabled_failed);
    REQUIRE(fx.disarm_switch_checked);
    REQUIRE_FALSE(fx.disarm_switch_failed);
    REQUIRE(fx.passed);
    REQUIRE(fx.set_pre_arm_check_called);
    REQUIRE(fx.set_pre_arm_check_value);
    REQUIRE(arming.pre_arm_check);
}

TEST_CASE("interlock plus estop fails conflict check", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.has_motor_interlock_option = true;
    in.has_motor_estop_option = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.interlock_estop_conflict_checked);
    REQUIRE(fx.interlock_estop_conflict_failed);
    REQUIRE(fx.check_failed_interlock_estop);
    REQUIRE(fx.motor_interlock_enabled_checked);
    REQUIRE_FALSE(fx.motor_interlock_enabled_failed);
    REQUIRE(fx.disarm_switch_checked);
    REQUIRE_FALSE(fx.disarm_switch_failed);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("interlock plus arm_emergency_stop fails conflict check", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.has_motor_interlock_option = true;
    in.has_arm_emergency_stop_option = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.interlock_estop_conflict_checked);
    REQUIRE(fx.interlock_estop_conflict_failed);
    REQUIRE(fx.check_failed_interlock_estop);
    REQUIRE_FALSE(fx.passed);
}

TEST_CASE("interlock alone does not fail conflict check", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.has_motor_interlock_option = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.interlock_estop_conflict_checked);
    REQUIRE_FALSE(fx.interlock_estop_conflict_failed);
    REQUIRE_FALSE(fx.check_failed_interlock_estop);
    REQUIRE(fx.passed);
    REQUIRE(arming.pre_arm_check);
}

TEST_CASE("motor interlock enabled fails when switch active", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.using_interlock = true;
    in.motor_interlock_switch = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.motor_interlock_enabled_checked);
    REQUIRE(fx.motor_interlock_enabled_failed);
    REQUIRE(fx.check_failed_motor_interlock);
    REQUIRE_FALSE(fx.interlock_estop_conflict_failed);
    REQUIRE(fx.disarm_switch_checked);
    REQUIRE_FALSE(fx.disarm_switch_failed);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("both interlock conflict and enabled can fail in one call", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.has_motor_interlock_option = true;
    in.has_motor_estop_option = true;
    in.using_interlock = true;
    in.motor_interlock_switch = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.interlock_estop_conflict_checked);
    REQUIRE(fx.interlock_estop_conflict_failed);
    REQUIRE(fx.check_failed_interlock_estop);
    REQUIRE(fx.motor_interlock_enabled_checked);
    REQUIRE(fx.motor_interlock_enabled_failed);
    REQUIRE(fx.check_failed_motor_interlock);
    REQUIRE(fx.disarm_switch_checked);
    REQUIRE_FALSE(fx.disarm_switch_failed);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("disarm switch HIGH fails pre_arm", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.has_disarm_switch_option = true;
    in.disarm_switch_high = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.disarm_switch_checked);
    REQUIRE(fx.disarm_switch_failed);
    REQUIRE(fx.check_failed_disarm_switch);
    REQUIRE_FALSE(fx.interlock_estop_conflict_failed);
    REQUIRE_FALSE(fx.motor_interlock_enabled_failed);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("no disarm switch option passes", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.has_disarm_switch_option = false;
    in.disarm_switch_high = true;  // ignored without option

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.disarm_switch_checked);
    REQUIRE_FALSE(fx.disarm_switch_failed);
    REQUIRE_FALSE(fx.check_failed_disarm_switch);
    REQUIRE(fx.passed);
    REQUIRE(arming.pre_arm_check);
}

TEST_CASE("disarm switch not HIGH passes", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.has_disarm_switch_option = true;
    in.disarm_switch_high = false;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.disarm_switch_checked);
    REQUIRE_FALSE(fx.disarm_switch_failed);
    REQUIRE_FALSE(fx.check_failed_disarm_switch);
    REQUIRE(fx.passed);
    REQUIRE(arming.pre_arm_check);
}

TEST_CASE("disarm switch fail accumulates with interlock fail", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.has_motor_interlock_option = true;
    in.has_motor_estop_option = true;
    in.has_disarm_switch_option = true;
    in.disarm_switch_high = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.interlock_estop_conflict_checked);
    REQUIRE(fx.interlock_estop_conflict_failed);
    REQUIRE(fx.check_failed_interlock_estop);
    REQUIRE(fx.disarm_switch_checked);
    REQUIRE(fx.disarm_switch_failed);
    REQUIRE(fx.check_failed_disarm_switch);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}
