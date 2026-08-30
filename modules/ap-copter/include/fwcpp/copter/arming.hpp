#pragma once

// Copter AP_Arming_Copter pre_arm leftover (CCP-038 slice 5).
// Upstream ArduCopter/AP_Arming_Copter.cpp pre_arm_checks ~8-13,
// run_pre_arm_checks ~17-86: already-armed, system_initialized,
// interlock/E-Stop conflict, motor interlock enabled,
// disarm_switch_checks, motors->arming_checks, early return when
// !passed, then should_skip_all_checks → mandatory_checks else
// parameter/oa/gcs/winch/rc_throttle/alt & AP_Arming::pre_arm_checks
// (thin inject AND-chain; real check bodies not this slice).
// HELI AROT out of scope. arm()/disarm() remain.
//
// Explicit PreArmInputs — no motors / scheduler / GCS / rc objects
// (ADR-0012). Catalog: arming_leftover.hpp.

#include <fwcpp/copter/arming_leftover.hpp>

namespace fwcpp::copter {

// Leftover stand-in for AP_Arming_Copter. No AP_Arming inheritance this
// slice; pre_arm_check mirrors set_pre_arm_check(bool) storage only.
struct ArmingCopter {
    bool pre_arm_check{false};
};

struct PreArmInputs {
    bool motors_armed{false};
    bool system_initialized{true};  // inject; default true so pass path works
    bool display_failure{false};    // record only; no GCS
    // RC aux option presence (inject; no rc().find_channel_for_option).
    bool has_motor_interlock_option{false};
    bool has_motor_estop_option{false};
    bool has_arm_emergency_stop_option{false};
    // copter.ap.using_interlock / motor_interlock_switch injects.
    bool using_interlock{false};
    bool motor_interlock_switch{false};
    // DISARM aux option + AuxSwitchPos::HIGH (inject; no rc find).
    bool has_disarm_switch_option{false};
    bool disarm_switch_high{false};
    // inject result of motors->arming_checks (default ok).
    bool motors_arming_checks_ok{true};
    // Upstream ~71-86: should_skip_all_checks → mandatory only, else
    // parameter & oa & gcs & winch & rc_throttle & alt & AP_Arming::pre_arm.
    // Bodies are injects (scaffold); real helpers remain for later.
    bool skip_all_checks{false};
    bool mandatory_checks_ok{true};
    bool parameter_checks_ok{true};
    bool oa_checks_ok{true};
    bool gcs_failsafe_ok{true};
    bool winch_checks_ok{true};
    bool rc_throttle_failsafe_ok{true};
    bool alt_checks_ok{true};
    bool ap_arming_pre_arm_ok{true};  // base AP_Arming::pre_arm_checks
};

struct PreArmEffects {
    bool already_armed_short_circuit{false};  // motors_armed → return true early
    bool pre_arm_ran{false};
    bool system_init_checked{false};
    bool system_init_failed{false};
    bool check_failed_system_init{false};  // would call check_failed
    bool interlock_estop_conflict_checked{false};
    bool interlock_estop_conflict_failed{false};
    bool check_failed_interlock_estop{false};
    bool motor_interlock_enabled_checked{false};
    bool motor_interlock_enabled_failed{false};
    bool check_failed_motor_interlock{false};
    bool disarm_switch_checked{false};
    bool disarm_switch_failed{false};
    bool check_failed_disarm_switch{false};
    bool motors_arming_checked{false};
    bool motors_arming_failed{false};
    bool check_failed_motors{false};
    // true when !passed after motors block (upstream ~67-69).
    bool early_return_after_gate_checks{false};
    // Upstream ~71-86 skip_all / parameter chain.
    bool skip_all_checked{false};
    bool mandatory_checks_ran{false};
    bool parameter_chain_ran{false};
    bool check_failed_mandatory{false};
    bool check_failed_parameter{false};
    bool check_failed_oa{false};
    bool check_failed_gcs_failsafe{false};
    bool check_failed_winch{false};
    bool check_failed_rc_throttle_failsafe{false};
    bool check_failed_alt{false};
    bool check_failed_ap_arming_pre_arm{false};
    bool passed{false};
    bool set_pre_arm_check_called{false};
    bool set_pre_arm_check_value{false};
};

// Upstream run_pre_arm_checks: already-armed short-circuit, then
// system_initialized, then interlock/E-Stop conflict + motor interlock
// enabled + disarm_switch_checks + motors->arming_checks; early return
// when !passed (~67-69); then skip_all → mandatory else parameter AND-chain
// (~71-86). Real parameter/gps/baro helper bodies not this slice.
[[nodiscard]] inline PreArmEffects run_pre_arm_checks(const PreArmInputs& in = {}) {
    PreArmEffects fx{};
    fx.pre_arm_ran = true;

    if (in.motors_armed) {
        fx.already_armed_short_circuit = true;
        fx.passed = true;
        return fx;
    }

    fx.system_init_checked = true;
    if (!in.system_initialized) {
        fx.system_init_failed = true;
        fx.check_failed_system_init = true;
        // display_failure is accepted but not used (no GCS this slice).
        (void)in.display_failure;
        fx.passed = false;
        return fx;
    }

    bool passed = true;

    // Interlock and E-Stop conflict (~29-37). Continue on failure.
    fx.interlock_estop_conflict_checked = true;
    if (in.has_motor_interlock_option &&
        (in.has_motor_estop_option || in.has_arm_emergency_stop_option)) {
        fx.interlock_estop_conflict_failed = true;
        fx.check_failed_interlock_estop = true;
        (void)in.display_failure;
        passed = false;
    }

    // Motor interlock enabled (~42-45). Continue on failure.
    fx.motor_interlock_enabled_checked = true;
    if (in.using_interlock && in.motor_interlock_switch) {
        fx.motor_interlock_enabled_failed = true;
        fx.check_failed_motor_interlock = true;
        (void)in.display_failure;
        passed = false;
    }

    // disarm_switch_checks (~47-49 / AP_Arming.cpp ~2078-2088).
    // Continue on failure (accumulate).
    fx.disarm_switch_checked = true;
    if (in.has_disarm_switch_option && in.disarm_switch_high) {
        fx.disarm_switch_failed = true;
        fx.check_failed_disarm_switch = true;
        (void)in.display_failure;
        passed = false;
    }

    // motors->arming_checks (~51-56). Then early return if !passed (~67-69).
    fx.motors_arming_checked = true;
    if (!in.motors_arming_checks_ok) {
        fx.motors_arming_failed = true;
        fx.check_failed_motors = true;
        (void)in.display_failure;
        passed = false;
    }
    if (!passed) {
        fx.early_return_after_gate_checks = true;
        fx.passed = false;
        return fx;
    }

    // Upstream ~71-86: should_skip_all_checks → mandatory only, else
    // parameter_checks & oa & gcs_failsafe & winch & rc_throttle_failsafe
    // & alt & AP_Arming::pre_arm_checks (inject AND-chain scaffold).
    fx.skip_all_checked = true;
    if (in.skip_all_checks) {
        fx.mandatory_checks_ran = true;
        if (!in.mandatory_checks_ok) {
            fx.check_failed_mandatory = true;
            (void)in.display_failure;
            fx.passed = false;
        } else {
            fx.passed = true;
        }
        return fx;
    }

    fx.parameter_chain_ran = true;
    if (!in.parameter_checks_ok) {
        fx.check_failed_parameter = true;
    }
    if (!in.oa_checks_ok) {
        fx.check_failed_oa = true;
    }
    if (!in.gcs_failsafe_ok) {
        fx.check_failed_gcs_failsafe = true;
    }
    if (!in.winch_checks_ok) {
        fx.check_failed_winch = true;
    }
    if (!in.rc_throttle_failsafe_ok) {
        fx.check_failed_rc_throttle_failsafe = true;
    }
    if (!in.alt_checks_ok) {
        fx.check_failed_alt = true;
    }
    if (!in.ap_arming_pre_arm_ok) {
        fx.check_failed_ap_arming_pre_arm = true;
    }
    fx.passed = in.parameter_checks_ok && in.oa_checks_ok && in.gcs_failsafe_ok &&
                in.winch_checks_ok && in.rc_throttle_failsafe_ok && in.alt_checks_ok &&
                in.ap_arming_pre_arm_ok;
    return fx;
}

// Upstream set_pre_arm_check(bool): store on ArmingCopter leftover.
inline void set_pre_arm_check(ArmingCopter& arming, bool b) {
    arming.pre_arm_check = b;
}

// Upstream pre_arm_checks: run_pre_arm then set_pre_arm_check(passed).
[[nodiscard]] inline PreArmEffects pre_arm_checks(ArmingCopter& arming,
                                                  const PreArmInputs& in = {}) {
    PreArmEffects fx = run_pre_arm_checks(in);
    set_pre_arm_check(arming, fx.passed);
    fx.set_pre_arm_check_called = true;
    fx.set_pre_arm_check_value = fx.passed;
    return fx;
}

// Free-function wrapper when the caller does not need ArmingCopter storage.
[[nodiscard]] inline PreArmEffects pre_arm_checks(const PreArmInputs& in = {}) {
    ArmingCopter arming{};
    return pre_arm_checks(arming, in);
}

}  // namespace fwcpp::copter
