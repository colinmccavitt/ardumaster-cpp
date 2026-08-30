#pragma once

// Copter AP_Arming_Copter pre_arm leftover scaffold (CCP-038 slice 1).
// Upstream ArduCopter/AP_Arming_Copter.cpp pre_arm_checks ~8-13,
// run_pre_arm_checks ~17-27 (already-armed + system_initialized only).
//
// Explicit PreArmInputs — no motors / scheduler / GCS objects (ADR-0012).
// Interlock, disarm_switch, motors->arming_checks, parameter/gps/baro/
// board_voltage/alt/rc_throttle_failsafe, and arm()/disarm() remain.
// Catalog: arming_leftover.hpp.

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
};

struct PreArmEffects {
    bool already_armed_short_circuit{false};  // motors_armed → return true early
    bool pre_arm_ran{false};
    bool system_init_checked{false};
    bool system_init_failed{false};
    bool check_failed_system_init{false};  // would call check_failed
    bool passed{false};
    bool set_pre_arm_check_called{false};
    bool set_pre_arm_check_value{false};
};

// Upstream run_pre_arm_checks: already-armed short-circuit, then
// system_initialized. Further checks remain (not run this slice).
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

    // Further checks remaining — treat as passed for this scaffold only.
    fx.passed = true;
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
