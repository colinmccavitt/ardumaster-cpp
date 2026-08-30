#pragma once

// Copter AP_Arming_Copter leftover (CCP-038 slice 6 closing).
// Upstream ArduCopter/AP_Arming_Copter.cpp pre_arm_checks ~8-13,
// run_pre_arm_checks ~17-86, arm() entry ~675-695, disarm() entry
// ~790-812. Heavy AHRS/notify/logger/motors/compass/mission bodies
// are out of scope (ADR-0012). HELI AROT out of scope.
//
// Explicit ArmInputs / DisarmInputs — no motors / AHRS / notify /
// logger objects (ADR-0012). Catalog: arming_leftover.hpp.

#include <cstdint>

#include <fwcpp/copter/arming_leftover.hpp>

namespace fwcpp::copter {

// Leftover stand-in for AP_Arming_Copter. No AP_Arming inheritance;
// pre_arm_check mirrors set_pre_arm_check(bool); in_arm_motors mirrors
// the static reentry guard in upstream arm().
struct ArmingCopter {
    bool pre_arm_check{false};
    bool in_arm_motors{false};
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

// --- arm() / disarm() entry leftover (slice 6) ---

struct ArmInputs {
    bool motors_armed{false};
    bool in_arm_motors{false};  // reentry inject (upstream static)
    bool base_arm_ok{true};     // AP_Arming::arm inject
    std::uint8_t method{0};     // optional Method enum stand-in
};

struct ArmEffects {
    bool reentry_rejected{false};
    bool already_armed_short_circuit{false};
    bool base_arm_called{false};
    bool base_arm_failed{false};
    bool arming_failed_notify{false};
    bool armed_success_flags{false};  // would set notify/logger — flag only
    bool passed{false};
};

// Upstream arm() ~675-695 entry only. AHRS/notify loops/logger/motors
// body (~697-786) is kOutOfScope. Returns true iff passed (fx.passed).
[[nodiscard]] inline bool leftover_arm(ArmingCopter& arming, const ArmInputs& in,
                                       ArmEffects& fx) {
    fx = ArmEffects{};
    (void)in.method;

    // exit immediately if already in this function (~679-682)
    if (in.in_arm_motors || arming.in_arm_motors) {
        fx.reentry_rejected = true;
        fx.passed = false;
        return false;
    }
    arming.in_arm_motors = true;

    // return true if already armed (~685-689)
    if (in.motors_armed) {
        fx.already_armed_short_circuit = true;
        fx.passed = true;
        arming.in_arm_motors = false;
        return true;
    }

    // AP_Arming::arm inject (~691-695)
    fx.base_arm_called = true;
    if (!in.base_arm_ok) {
        fx.base_arm_failed = true;
        fx.arming_failed_notify = true;  // AP_Notify::events.arming_failed
        fx.passed = false;
        arming.in_arm_motors = false;
        return false;
    }

    // Success entry complete — heavy body out of scope; flag only.
    fx.armed_success_flags = true;
    fx.passed = true;
    arming.in_arm_motors = false;
    return true;
}

[[nodiscard]] inline ArmEffects leftover_arm(ArmingCopter& arming,
                                             const ArmInputs& in = {}) {
    ArmEffects fx{};
    (void)leftover_arm(arming, in, fx);
    return fx;
}

struct DisarmInputs {
    bool motors_armed{true};
    bool do_disarm_checks{true};
    bool method_is_gcs{false};
    bool land_complete{true};
    bool method_is_rudder{false};
    bool has_manual_throttle{false};
    bool base_disarm_ok{true};
    std::uint8_t method{0};  // optional Method enum stand-in
};

struct DisarmEffects {
    bool already_disarmed_short_circuit{false};
    bool gcs_flying_rejected{false};
    bool rudder_flying_rejected{false};
    bool base_disarm_called{false};
    bool base_disarm_failed{false};
    bool motors_disarmed_flag{false};
    bool passed{false};
};

// Upstream disarm() ~790-812 entry only. Compass/AHRS/mission/logger
// body (~814-856) is kOutOfScope. Returns true iff passed (fx.passed).
[[nodiscard]] inline bool leftover_disarm(ArmingCopter& arming,
                                          const DisarmInputs& in,
                                          DisarmEffects& fx) {
    fx = DisarmEffects{};
    (void)arming;
    (void)in.method;

    // return immediately if already disarmed (~792-795)
    if (!in.motors_armed) {
        fx.already_disarmed_short_circuit = true;
        fx.passed = true;
        return true;
    }

    // do not allow disarm via mavlink if flying (~797-802)
    if (in.do_disarm_checks && in.method_is_gcs && !in.land_complete) {
        fx.gcs_flying_rejected = true;
        fx.passed = false;
        return false;
    }

    // rudder disarm while flying without manual throttle (~804-808)
    if (in.method_is_rudder) {
        if (!in.has_manual_throttle && !in.land_complete) {
            fx.rudder_flying_rejected = true;
            fx.passed = false;
            return false;
        }
    }

    // AP_Arming::disarm inject (~810-812)
    fx.base_disarm_called = true;
    if (!in.base_disarm_ok) {
        fx.base_disarm_failed = true;
        fx.passed = false;
        return false;
    }

    // Success entry — motors->armed(false) as flag; body out of scope.
    fx.motors_disarmed_flag = true;
    fx.passed = true;
    return true;
}

[[nodiscard]] inline DisarmEffects leftover_disarm(ArmingCopter& arming,
                                                   const DisarmInputs& in = {}) {
    DisarmEffects fx{};
    (void)leftover_disarm(arming, in, fx);
    return fx;
}

}  // namespace fwcpp::copter
