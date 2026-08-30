#pragma once

// ModePosHold leftover scaffold — ArduCopter/mode_poshold.cpp (Plane-4.7.0).
// CCP-040 slice 3: leftover_init flags matching init ~71-107; leftover_run
// is call-site-only (leftover_run_called). No loiter_nav / pos_control /
// attitude_control objects (ADR-0012). PosHold run state machine / ModeDrift
// are cataloged in mode_loiter_leftover.hpp.
//
// mode_from_mode_num still returns nullptr for POSHOLD this slice.

#include <fwcpp/copter/mode.hpp>

#include <cstdint>

namespace fwcpp::copter {

// Upstream ModePosHold::RPMode (mode.h). Init only needs LOITER /
// PILOT_OVERRIDE; remaining values reserved for the run machine.
enum class PosHoldRpMode : std::uint8_t {
    PilotOverride = 0,
    Brake = 1,
    BrakeReadyToLoiter = 2,
    BrakeToLoiter = 3,
    Loiter = 4,
    ControllerToPilotOverride = 5,
};

class ModePosHold : public Mode {
public:
    // Injected pos_control->D_is_active(). Default true so D_init is not
    // required without a pos_control object.
    bool leftover_d_is_active{true};
    // Injected copter.ap.land_complete for starting RP mode.
    bool leftover_land_complete{true};

    // Leftover ModePosHold::init call-site flags (mode_poshold.cpp ~71-107).
    // D_set_max_speed_accel_m + D_set_correction_speed_accel_m.
    bool leftover_d_set_max{false};
    bool leftover_d_init{false};
    bool leftover_zero_pilot_lean{false};
    bool leftover_brake_gain{false};
    bool leftover_loiter_clear_pilot_accel{false};
    bool leftover_loiter_init_target{false};
    bool leftover_init_wind_comp{false};
    PosHoldRpMode leftover_roll_mode{PosHoldRpMode::Loiter};
    PosHoldRpMode leftover_pitch_mode{PosHoldRpMode::Loiter};

    // Leftover ModePosHold::run entry (mode_poshold.cpp ~111+). This slice:
    // call-site only; spool / RP machine / loiter_nav / pos / attitude remaining.
    bool leftover_run_called{false};

    ModePosHold() = default;

    [[nodiscard]] Number mode_number() const override { return Number::POSHOLD; }
    [[nodiscard]] bool init(bool ignore_checks) override {
        return leftover_init(ignore_checks);
    }
    void run() override { leftover_run(); }
    [[nodiscard]] bool requires_position() const override { return true; }
    [[nodiscard]] bool has_manual_throttle() const override { return false; }

    // Leftover ModePosHold::init (mode_poshold.cpp ~71-107). ignore_checks
    // unused upstream (always returns true). No loiter_nav / pos_control.
    [[nodiscard]] bool leftover_init(bool /*ignore_checks*/) {
        leftover_d_set_max = true;
        leftover_d_init = false;
        if (!leftover_d_is_active) {
            leftover_d_init = true;
        }
        leftover_zero_pilot_lean = true;
        leftover_brake_gain = true;
        if (leftover_land_complete) {
            leftover_roll_mode = PosHoldRpMode::Loiter;
            leftover_pitch_mode = PosHoldRpMode::Loiter;
        } else {
            leftover_roll_mode = PosHoldRpMode::PilotOverride;
            leftover_pitch_mode = PosHoldRpMode::PilotOverride;
        }
        leftover_loiter_clear_pilot_accel = true;
        leftover_loiter_init_target = true;
        leftover_init_wind_comp = true;
        return true;
    }

    // Leftover ModePosHold::run (mode_poshold.cpp ~111+). Thin entry flag
    // only this slice; remaining for RP / alt-hold / loiter / attitude bodies.
    void leftover_run() { leftover_run_called = true; }
};

}  // namespace fwcpp::copter
