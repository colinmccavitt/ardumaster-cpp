#pragma once

// ModeDrift leftover scaffold — ArduCopter/mode_drift.cpp (Plane-4.7.0).
// CCP-040 slice 4: leftover_init returns true (~49-52); leftover_run thin
// flags through pilot lean / vel_ned / spool / attitude / throttle_assist
// call-sites (~56-145). No real vel-rotate / braker / yaw-schedule math
// (ADR-0012: no motors / pos_control / attitude_control objects).
// ModePosHold::run remains catalog remaining.
//
// mode_from_mode_num still returns nullptr for DRIFT this slice.

#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>

namespace fwcpp::copter {

class ModeDrift : public Mode {
public:
    // Injected motors / ap state for leftover_run spool path.
    bool leftover_throttle_zero{true};
    SpoolState leftover_spool_state{SpoolState::SHUT_DOWN};
    bool leftover_throttle_lower_limit{false};

    // Leftover ModeDrift::run call-site / effect flags.
    bool leftover_run_called{false};
    bool leftover_pilot_lean{false};
    bool leftover_get_vel_ned{false};
    bool leftover_set_desired_spool{false};
    DesiredSpoolState leftover_desired_spool{DesiredSpoolState::SHUT_DOWN};
    bool leftover_reset_yaw_target_and_rate{false};
    bool leftover_reset_I{false};
    bool leftover_reset_I_smoothly{false};
    bool leftover_clear_land_complete{false};
    bool leftover_attitude{false};
    bool leftover_throttle_assist{false};
    bool leftover_set_throttle_out{false};

    ModeDrift() = default;

    [[nodiscard]] Number mode_number() const override { return Number::DRIFT; }
    [[nodiscard]] bool init(bool ignore_checks) override {
        return leftover_init(ignore_checks);
    }
    void run() override { leftover_run(); }
    [[nodiscard]] bool requires_position() const override { return true; }
    [[nodiscard]] bool has_manual_throttle() const override { return false; }

    // Leftover ModeDrift::init (mode_drift.cpp ~49-52). ignore_checks unused
    // upstream (always returns true).
    [[nodiscard]] bool leftover_init(bool /*ignore_checks*/) { return true; }

    // Leftover ModeDrift::run (mode_drift.cpp ~56-145). Thin flags through
    // lean / get_vel_ned / spool desired + switch / attitude / throttle
    // assist + set_throttle_out. Resets run effect flags at entry so a
    // later inject change does not leave stale true flags. Vel-NED rotate,
    // yaw-rate schedule, roll_input LP, braker math remaining (no real
    // bodies this slice).
    void leftover_run() {
        leftover_run_called = false;
        leftover_pilot_lean = false;
        leftover_get_vel_ned = false;
        leftover_set_desired_spool = false;
        leftover_desired_spool = DesiredSpoolState::SHUT_DOWN;
        leftover_reset_yaw_target_and_rate = false;
        leftover_reset_I = false;
        leftover_reset_I_smoothly = false;
        leftover_clear_land_complete = false;
        leftover_attitude = false;
        leftover_throttle_assist = false;
        leftover_set_throttle_out = false;

        leftover_run_called = true;

        // Upstream ~61-63: get_pilot_desired_lean_angles_rad. Body remaining.
        leftover_pilot_lean = true;
        // Upstream ~66: pos_control->get_vel_estimate_NED_ms. Body remaining.
        leftover_get_vel_ned = true;
        // Upstream ~68-107: body-frame vel / yaw schedule / braker math —
        // not ported this slice (no real math).

        // Upstream ~108-114: set_desired_spool_state from throttle_zero.
        leftover_set_desired_spool = true;
        if (leftover_throttle_zero) {
            leftover_desired_spool = DesiredSpoolState::GROUND_IDLE;
        } else {
            leftover_desired_spool = DesiredSpoolState::THROTTLE_UNLIMITED;
        }

        // Upstream ~116-136: spool switch call-site flags only.
        switch (leftover_spool_state) {
            case SpoolState::SHUT_DOWN:
                leftover_reset_yaw_target_and_rate = true;
                leftover_reset_I = true;
                break;

            case SpoolState::GROUND_IDLE:
                leftover_reset_yaw_target_and_rate = true;
                leftover_reset_I_smoothly = true;
                break;

            case SpoolState::THROTTLE_UNLIMITED:
                if (!leftover_throttle_lower_limit) {
                    leftover_clear_land_complete = true;
                }
                break;

            case SpoolState::SPOOLING_UP:
            case SpoolState::SPOOLING_DOWN:
                break;
        }

        // Upstream ~138-144: attitude + get_throttle_assist + set_throttle_out.
        leftover_attitude = true;
        leftover_throttle_assist = true;
        leftover_set_throttle_out = true;
    }
};

}  // namespace fwcpp::copter
