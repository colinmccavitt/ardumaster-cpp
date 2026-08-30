#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/arming.hpp>
#include <fwcpp/copter/arming_leftover.hpp>

using fwcpp::copter::ArmEffects;
using fwcpp::copter::ArmInputs;
using fwcpp::copter::ArmingCopter;
using fwcpp::copter::DisarmEffects;
using fwcpp::copter::DisarmInputs;
using fwcpp::copter::PreArmInputs;
using fwcpp::copter::leftover_arm;
using fwcpp::copter::leftover_disarm;
using fwcpp::copter::pre_arm_checks;
using fwcpp::copter::arming::PortStatus;
using fwcpp::copter::arming::completeness_has;
using fwcpp::copter::arming::completeness_size;
using fwcpp::copter::arming::on_main_count;
using fwcpp::copter::arming::out_of_scope_count;
using fwcpp::copter::arming::remaining_count;
using fwcpp::copter::arming::this_slice_count;

TEST_CASE("arming leftover catalog this_slice and remaining", "[copter][arming][leftover]") {
    REQUIRE(this_slice_count() == 10);
    REQUIRE(remaining_count() == 0);
    REQUIRE(out_of_scope_count() == 2);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("pre_arm_checks", PortStatus::kThisSlice));
    REQUIRE(completeness_has("run_pre_arm_checks already_armed gate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("system_initialized check", PortStatus::kThisSlice));
    REQUIRE(completeness_has("interlock/estop conflict", PortStatus::kThisSlice));
    REQUIRE(completeness_has("motor interlock enabled", PortStatus::kThisSlice));
    REQUIRE(completeness_has("disarm_switch_checks", PortStatus::kThisSlice));
    REQUIRE(completeness_has("motors->arming_checks", PortStatus::kThisSlice));
    REQUIRE(completeness_has(
        "parameter_checks / gps / baro / board_voltage / alt / rc_throttle_failsafe",
        PortStatus::kThisSlice));
    REQUIRE(completeness_has("arm() / disarm()", PortStatus::kThisSlice));
    REQUIRE(completeness_has(
        "arm()/disarm() AHRS/notify/logger/motors/compass/mission body",
        PortStatus::kOutOfScope));
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
    in.motors_arming_checks_ok = false;  // must not be checked when already armed

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.pre_arm_ran);
    REQUIRE(fx.already_armed_short_circuit);
    REQUIRE(fx.passed);
    REQUIRE_FALSE(fx.system_init_checked);
    REQUIRE_FALSE(fx.system_init_failed);
    REQUIRE_FALSE(fx.interlock_estop_conflict_checked);
    REQUIRE_FALSE(fx.motor_interlock_enabled_checked);
    REQUIRE_FALSE(fx.disarm_switch_checked);
    REQUIRE_FALSE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.motors_arming_failed);
    REQUIRE_FALSE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.parameter_chain_ran);
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
    in.motors_arming_checks_ok = false;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.pre_arm_ran);
    REQUIRE_FALSE(fx.already_armed_short_circuit);
    REQUIRE(fx.system_init_checked);
    REQUIRE(fx.system_init_failed);
    REQUIRE(fx.check_failed_system_init);
    REQUIRE_FALSE(fx.interlock_estop_conflict_checked);
    REQUIRE_FALSE(fx.motor_interlock_enabled_checked);
    REQUIRE_FALSE(fx.disarm_switch_checked);
    REQUIRE_FALSE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.parameter_chain_ran);
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
    REQUIRE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.motors_arming_failed);
    REQUIRE_FALSE(fx.early_return_after_gate_checks);
    REQUIRE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.mandatory_checks_ran);
    REQUIRE(fx.parameter_chain_ran);
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
    REQUIRE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.motors_arming_failed);
    REQUIRE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.parameter_chain_ran);
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
    REQUIRE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.parameter_chain_ran);
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
    REQUIRE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.early_return_after_gate_checks);
    REQUIRE(fx.skip_all_checked);
    REQUIRE(fx.parameter_chain_ran);
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
    REQUIRE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.motors_arming_failed);
    REQUIRE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.parameter_chain_ran);
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
    REQUIRE(fx.motors_arming_checked);
    REQUIRE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.parameter_chain_ran);
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
    REQUIRE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.motors_arming_failed);
    REQUIRE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.parameter_chain_ran);
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
    REQUIRE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.early_return_after_gate_checks);
    REQUIRE(fx.skip_all_checked);
    REQUIRE(fx.parameter_chain_ran);
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
    REQUIRE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.early_return_after_gate_checks);
    REQUIRE(fx.skip_all_checked);
    REQUIRE(fx.parameter_chain_ran);
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
    REQUIRE(fx.motors_arming_checked);
    REQUIRE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.parameter_chain_ran);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("motors arming_checks fail alone", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.motors_arming_checks_ok = false;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.disarm_switch_checked);
    REQUIRE_FALSE(fx.disarm_switch_failed);
    REQUIRE(fx.motors_arming_checked);
    REQUIRE(fx.motors_arming_failed);
    REQUIRE(fx.check_failed_motors);
    REQUIRE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.mandatory_checks_ran);
    REQUIRE_FALSE(fx.parameter_chain_ran);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("motors fail plus interlock both set then early return", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.has_motor_interlock_option = true;
    in.has_motor_estop_option = true;
    in.motors_arming_checks_ok = false;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.interlock_estop_conflict_checked);
    REQUIRE(fx.interlock_estop_conflict_failed);
    REQUIRE(fx.check_failed_interlock_estop);
    REQUIRE(fx.motors_arming_checked);
    REQUIRE(fx.motors_arming_failed);
    REQUIRE(fx.check_failed_motors);
    REQUIRE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.parameter_chain_ran);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("motors ok continues with passed true", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.motors_arming_checks_ok = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.motors_arming_checked);
    REQUIRE_FALSE(fx.motors_arming_failed);
    REQUIRE_FALSE(fx.check_failed_motors);
    REQUIRE_FALSE(fx.early_return_after_gate_checks);
    REQUIRE(fx.skip_all_checked);
    REQUIRE(fx.parameter_chain_ran);
    REQUIRE(fx.passed);
    REQUIRE(arming.pre_arm_check);
}

TEST_CASE("skip_all runs mandatory only", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.skip_all_checks = true;
    in.mandatory_checks_ok = true;
    in.parameter_checks_ok = false;  // must not be consulted

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE_FALSE(fx.early_return_after_gate_checks);
    REQUIRE(fx.skip_all_checked);
    REQUIRE(fx.mandatory_checks_ran);
    REQUIRE_FALSE(fx.parameter_chain_ran);
    REQUIRE_FALSE(fx.check_failed_mandatory);
    REQUIRE(fx.passed);
    REQUIRE(arming.pre_arm_check);
}

TEST_CASE("skip_all mandatory fail", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.skip_all_checks = true;
    in.mandatory_checks_ok = false;
    in.parameter_checks_ok = true;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.skip_all_checked);
    REQUIRE(fx.mandatory_checks_ran);
    REQUIRE_FALSE(fx.parameter_chain_ran);
    REQUIRE(fx.check_failed_mandatory);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("parameter chain all ok passes", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.skip_all_checks = false;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.mandatory_checks_ran);
    REQUIRE(fx.parameter_chain_ran);
    REQUIRE_FALSE(fx.check_failed_parameter);
    REQUIRE_FALSE(fx.check_failed_oa);
    REQUIRE_FALSE(fx.check_failed_gcs_failsafe);
    REQUIRE_FALSE(fx.check_failed_winch);
    REQUIRE_FALSE(fx.check_failed_rc_throttle_failsafe);
    REQUIRE_FALSE(fx.check_failed_alt);
    REQUIRE_FALSE(fx.check_failed_ap_arming_pre_arm);
    REQUIRE(fx.passed);
    REQUIRE(arming.pre_arm_check);
}

TEST_CASE("parameter chain one false fails", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.oa_checks_ok = false;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.skip_all_checked);
    REQUIRE(fx.parameter_chain_ran);
    REQUIRE_FALSE(fx.mandatory_checks_ran);
    REQUIRE(fx.check_failed_oa);
    REQUIRE_FALSE(fx.check_failed_parameter);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("early motors fail never reaches parameter chain", "[copter][arming]") {
    ArmingCopter arming{};
    PreArmInputs in{};
    in.motors_arming_checks_ok = false;
    in.skip_all_checks = true;  // would take skip path if reached
    in.parameter_checks_ok = false;

    const auto fx = pre_arm_checks(arming, in);

    REQUIRE(fx.motors_arming_failed);
    REQUIRE(fx.early_return_after_gate_checks);
    REQUIRE_FALSE(fx.skip_all_checked);
    REQUIRE_FALSE(fx.mandatory_checks_ran);
    REQUIRE_FALSE(fx.parameter_chain_ran);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.pre_arm_check);
}

TEST_CASE("arm reentry rejected", "[copter][arming]") {
    ArmingCopter arming{};
    ArmInputs in{};
    in.in_arm_motors = true;
    in.motors_armed = false;
    in.base_arm_ok = true;

    ArmEffects fx{};
    REQUIRE_FALSE(leftover_arm(arming, in, fx));

    REQUIRE(fx.reentry_rejected);
    REQUIRE_FALSE(fx.already_armed_short_circuit);
    REQUIRE_FALSE(fx.base_arm_called);
    REQUIRE_FALSE(fx.armed_success_flags);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.in_arm_motors);
}

TEST_CASE("arm reentry via ArmingCopter guard", "[copter][arming]") {
    ArmingCopter arming{};
    arming.in_arm_motors = true;
    ArmInputs in{};

    const auto fx = leftover_arm(arming, in);

    REQUIRE(fx.reentry_rejected);
    REQUIRE_FALSE(fx.passed);
    REQUIRE(arming.in_arm_motors);  // unchanged on reentry reject
}

TEST_CASE("arm already armed short-circuit", "[copter][arming]") {
    ArmingCopter arming{};
    ArmInputs in{};
    in.motors_armed = true;
    in.base_arm_ok = false;  // must not be consulted

    ArmEffects fx{};
    REQUIRE(leftover_arm(arming, in, fx));

    REQUIRE_FALSE(fx.reentry_rejected);
    REQUIRE(fx.already_armed_short_circuit);
    REQUIRE_FALSE(fx.base_arm_called);
    REQUIRE_FALSE(fx.base_arm_failed);
    REQUIRE_FALSE(fx.arming_failed_notify);
    REQUIRE_FALSE(fx.armed_success_flags);
    REQUIRE(fx.passed);
    REQUIRE_FALSE(arming.in_arm_motors);
}

TEST_CASE("arm base fail notifies", "[copter][arming]") {
    ArmingCopter arming{};
    ArmInputs in{};
    in.base_arm_ok = false;

    ArmEffects fx{};
    REQUIRE_FALSE(leftover_arm(arming, in, fx));

    REQUIRE_FALSE(fx.reentry_rejected);
    REQUIRE_FALSE(fx.already_armed_short_circuit);
    REQUIRE(fx.base_arm_called);
    REQUIRE(fx.base_arm_failed);
    REQUIRE(fx.arming_failed_notify);
    REQUIRE_FALSE(fx.armed_success_flags);
    REQUIRE_FALSE(fx.passed);
    REQUIRE_FALSE(arming.in_arm_motors);
}

TEST_CASE("arm success sets armed flags only", "[copter][arming]") {
    ArmingCopter arming{};
    ArmInputs in{};

    ArmEffects fx{};
    REQUIRE(leftover_arm(arming, in, fx));

    REQUIRE_FALSE(fx.reentry_rejected);
    REQUIRE_FALSE(fx.already_armed_short_circuit);
    REQUIRE(fx.base_arm_called);
    REQUIRE_FALSE(fx.base_arm_failed);
    REQUIRE_FALSE(fx.arming_failed_notify);
    REQUIRE(fx.armed_success_flags);
    REQUIRE(fx.passed);
    REQUIRE_FALSE(arming.in_arm_motors);
}

TEST_CASE("disarm already disarmed short-circuit", "[copter][arming]") {
    ArmingCopter arming{};
    DisarmInputs in{};
    in.motors_armed = false;
    in.method_is_gcs = true;
    in.land_complete = false;  // must not reject when already disarmed
    in.base_disarm_ok = false;

    DisarmEffects fx{};
    REQUIRE(leftover_disarm(arming, in, fx));

    REQUIRE(fx.already_disarmed_short_circuit);
    REQUIRE_FALSE(fx.gcs_flying_rejected);
    REQUIRE_FALSE(fx.rudder_flying_rejected);
    REQUIRE_FALSE(fx.base_disarm_called);
    REQUIRE_FALSE(fx.motors_disarmed_flag);
    REQUIRE(fx.passed);
}

TEST_CASE("disarm gcs flying rejected", "[copter][arming]") {
    ArmingCopter arming{};
    DisarmInputs in{};
    in.motors_armed = true;
    in.do_disarm_checks = true;
    in.method_is_gcs = true;
    in.land_complete = false;

    DisarmEffects fx{};
    REQUIRE_FALSE(leftover_disarm(arming, in, fx));

    REQUIRE_FALSE(fx.already_disarmed_short_circuit);
    REQUIRE(fx.gcs_flying_rejected);
    REQUIRE_FALSE(fx.rudder_flying_rejected);
    REQUIRE_FALSE(fx.base_disarm_called);
    REQUIRE_FALSE(fx.passed);
}

TEST_CASE("disarm gcs flying allowed when checks skipped", "[copter][arming]") {
    ArmingCopter arming{};
    DisarmInputs in{};
    in.do_disarm_checks = false;
    in.method_is_gcs = true;
    in.land_complete = false;

    const auto fx = leftover_disarm(arming, in);

    REQUIRE_FALSE(fx.gcs_flying_rejected);
    REQUIRE(fx.base_disarm_called);
    REQUIRE(fx.motors_disarmed_flag);
    REQUIRE(fx.passed);
}

TEST_CASE("disarm rudder flying rejected", "[copter][arming]") {
    ArmingCopter arming{};
    DisarmInputs in{};
    in.method_is_rudder = true;
    in.has_manual_throttle = false;
    in.land_complete = false;

    DisarmEffects fx{};
    REQUIRE_FALSE(leftover_disarm(arming, in, fx));

    REQUIRE_FALSE(fx.gcs_flying_rejected);
    REQUIRE(fx.rudder_flying_rejected);
    REQUIRE_FALSE(fx.base_disarm_called);
    REQUIRE_FALSE(fx.passed);
}

TEST_CASE("disarm rudder ok with manual throttle while flying", "[copter][arming]") {
    ArmingCopter arming{};
    DisarmInputs in{};
    in.method_is_rudder = true;
    in.has_manual_throttle = true;
    in.land_complete = false;

    const auto fx = leftover_disarm(arming, in);

    REQUIRE_FALSE(fx.rudder_flying_rejected);
    REQUIRE(fx.base_disarm_called);
    REQUIRE(fx.motors_disarmed_flag);
    REQUIRE(fx.passed);
}

TEST_CASE("disarm base fail", "[copter][arming]") {
    ArmingCopter arming{};
    DisarmInputs in{};
    in.base_disarm_ok = false;

    DisarmEffects fx{};
    REQUIRE_FALSE(leftover_disarm(arming, in, fx));

    REQUIRE(fx.base_disarm_called);
    REQUIRE(fx.base_disarm_failed);
    REQUIRE_FALSE(fx.motors_disarmed_flag);
    REQUIRE_FALSE(fx.passed);
}

TEST_CASE("disarm success sets motors_disarmed_flag", "[copter][arming]") {
    ArmingCopter arming{};
    DisarmInputs in{};

    DisarmEffects fx{};
    REQUIRE(leftover_disarm(arming, in, fx));

    REQUIRE_FALSE(fx.already_disarmed_short_circuit);
    REQUIRE_FALSE(fx.gcs_flying_rejected);
    REQUIRE_FALSE(fx.rudder_flying_rejected);
    REQUIRE(fx.base_disarm_called);
    REQUIRE_FALSE(fx.base_disarm_failed);
    REQUIRE(fx.motors_disarmed_flag);
    REQUIRE(fx.passed);
}
