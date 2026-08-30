#pragma once

// ModeLoiter leftover scaffold — ArduCopter/mode_loiter.cpp (Plane-4.7.0).
// CCP-040 slice 1: leftover_init flags matching init ~10-38; leftover_run
// is call-site-only (leftover_run_called). No loiter_nav / pos_control /
// attitude_control objects (ADR-0012). precision_loiter / run body /
// POSHOLD / DRIFT are cataloged in mode_loiter_leftover.hpp.
//
// mode_from_mode_num still returns nullptr for LOITER this slice.

#include <fwcpp/copter/mode.hpp>

namespace fwcpp::copter {

class ModeLoiter : public Mode {
public:
    // Injected pos_control->D_is_active(). Default true so D_init is not
    // required without a pos_control object.
    bool leftover_d_is_active{true};

    // Leftover ModeLoiter::init call-site flags (mode_loiter.cpp ~10-38).
    bool leftover_update_simple_mode{false};
    bool leftover_pilot_lean{false};
    bool leftover_loiter_set_pilot_accel{false};
    bool leftover_loiter_init_target{false};
    bool leftover_d_init{false};
    // D_set_max_speed_accel_m + D_set_correction_speed_accel_m.
    bool leftover_d_set_max{false};

    // Leftover ModeLoiter::run entry (mode_loiter.cpp ~80+). This slice:
    // call-site only; spool / loiter_nav / pos / attitude bodies remaining.
    bool leftover_run_called{false};

    ModeLoiter() = default;

    [[nodiscard]] Number mode_number() const override { return Number::LOITER; }
    [[nodiscard]] bool init(bool ignore_checks) override {
        return leftover_init(ignore_checks);
    }
    void run() override { leftover_run(); }
    [[nodiscard]] bool requires_position() const override { return true; }
    [[nodiscard]] bool has_manual_throttle() const override { return false; }

    // Leftover ModeLoiter::init (mode_loiter.cpp ~10-38). ignore_checks
    // unused upstream (always returns true). No loiter_nav / pos_control.
    [[nodiscard]] bool leftover_init(bool /*ignore_checks*/) {
        leftover_update_simple_mode = true;
        leftover_pilot_lean = true;
        leftover_loiter_set_pilot_accel = true;
        leftover_loiter_init_target = true;
        leftover_d_init = false;
        if (!leftover_d_is_active) {
            leftover_d_init = true;
        }
        leftover_d_set_max = true;
        return true;
    }

    // Leftover ModeLoiter::run (mode_loiter.cpp ~80+). Thin entry flag only
    // this slice; remaining for spool / wp / pos / attitude bodies.
    void leftover_run() { leftover_run_called = true; }
};

}  // namespace fwcpp::copter
