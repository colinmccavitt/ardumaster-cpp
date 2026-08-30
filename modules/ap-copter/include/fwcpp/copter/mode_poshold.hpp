#pragma once

// ModePosHold leftover — ArduCopter/mode_poshold.cpp (Plane-4.7.0).
// CCP-040 slice 5: leftover_run thin body flags through common pilot path +
// get_alt_hold_state switch + attitude / D_update (mode_poshold.cpp ~111-215,
// ~530-534). Slice 3 leftover_init (~71-107) unchanged. Brake / wind_comp RP
// state machine bodies OOS this ticket ("detailed PosHold brake blend later").
// No loiter_nav / pos_control / attitude_control objects (ADR-0012).
// Reuses AltHoldModeState + get_alt_hold_state_D_ms from mode_althold.hpp.
//
// mode_from_mode_num still returns nullptr for POSHOLD this slice.

#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/mode_althold.hpp>

#include <cstdint>

namespace fwcpp::copter {

// Upstream ModePosHold::RPMode (mode.h). Init + alt-hold switch arms set
// LOITER / PILOT_OVERRIDE; brake blend modes reserved (OOS this ticket).
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

    // Injected for leftover_run (mode_poshold.cpp ~111-215 / ~530-534).
    bool leftover_land_complete_maybe{false};
    bool leftover_takeoff_running{false};
    bool leftover_armed{false};
    bool leftover_takeoff_triggered{false};
    bool leftover_auto_armed{false};
    bool leftover_using_interlock{false};
    SpoolState leftover_spool_state{SpoolState::SHUT_DOWN};
    // Climb rate already constrained; fed to get_alt_hold_state_D_ms.
    float leftover_target_climb_rate_ms{0.0f};

    // Leftover ModePosHold::run call-site / effect flags.
    bool leftover_run_called{false};
    bool leftover_update_simple_mode{false};
    bool leftover_pilot_lean{false};
    bool leftover_pilot_yaw{false};
    bool leftover_pilot_climb{false};
    bool leftover_soften_for_landing{false};
    bool leftover_get_alt_hold_state{false};
    AltHoldModeState leftover_poshold_state{AltHoldModeState::MotorStopped};
    bool leftover_reset_I{false};
    bool leftover_reset_yaw_target_and_rate{false};
    bool leftover_reset_I_smoothly{false};
    bool leftover_d_relax{false};
    bool leftover_takeoff_start{false};
    bool leftover_do_pilot_takeoff{false};
    bool leftover_avoidance{false};
    bool leftover_surface_tracking{false};
    bool leftover_d_set_pos_from_climb{false};
    bool leftover_attitude{false};
    bool leftover_d_update_controller{false};

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

    // Leftover ModePosHold::run (mode_poshold.cpp ~111-215 common + alt-hold
    // switch; ~530-534 attitude / D_update). Thin flags through pilot path,
    // soften_for_landing, get_alt_hold_state, switch arms, and post-switch
    // attitude / D_update. Resets run effect flags at entry so a later inject
    // change does not leave stale true flags. Brake / wind_comp / RP roll+
    // pitch state machine bodies OOS (no real math this ticket).
    void leftover_run() {
        leftover_run_called = false;
        leftover_d_set_max = false;
        leftover_update_simple_mode = false;
        leftover_pilot_lean = false;
        leftover_loiter_clear_pilot_accel = false;
        leftover_pilot_yaw = false;
        leftover_pilot_climb = false;
        leftover_soften_for_landing = false;
        leftover_get_alt_hold_state = false;
        leftover_loiter_init_target = false;
        leftover_init_wind_comp = false;
        leftover_reset_I = false;
        leftover_reset_yaw_target_and_rate = false;
        leftover_reset_I_smoothly = false;
        leftover_d_relax = false;
        leftover_takeoff_start = false;
        leftover_do_pilot_takeoff = false;
        leftover_avoidance = false;
        leftover_surface_tracking = false;
        leftover_d_set_pos_from_climb = false;
        leftover_attitude = false;
        leftover_d_update_controller = false;
        leftover_poshold_state = AltHoldModeState::MotorStopped;

        leftover_run_called = true;

        // Upstream ~123-139: D_set_max; clear_pilot_accel; update_simple_mode;
        // lean; yaw; climb + constrain. Bodies remaining; call-site flags only.
        leftover_d_set_max = true;
        leftover_loiter_clear_pilot_accel = true;
        leftover_update_simple_mode = true;
        leftover_pilot_lean = true;
        leftover_pilot_yaw = true;
        leftover_pilot_climb = true;

        // Upstream ~141-144: soften_for_landing if land_complete_maybe.
        if (leftover_land_complete_maybe) {
            leftover_soften_for_landing = true;
        }

        // Upstream ~146-147: get_alt_hold_state_D_ms(target_climb_rate).
        leftover_get_alt_hold_state = true;
        AltHoldStateInputs st;
        st.armed = leftover_armed;
        st.takeoff_running = leftover_takeoff_running;
        st.takeoff_triggered = leftover_takeoff_triggered;
        st.auto_armed = leftover_auto_armed;
        st.land_complete = leftover_land_complete;
        st.using_interlock = leftover_using_interlock;
        st.spool = leftover_spool_state;
        st.target_climb_rate_ms = leftover_target_climb_rate_ms;
        leftover_poshold_state = get_alt_hold_state_D_ms(st).state;

        // Upstream ~149-215: switch arms as leftover flags only.
        switch (leftover_poshold_state) {
            case AltHoldModeState::MotorStopped:
                leftover_reset_I = true;
                leftover_reset_yaw_target_and_rate = true;
                leftover_d_relax = true;
                leftover_loiter_clear_pilot_accel = true;
                leftover_loiter_init_target = true;
                leftover_roll_mode = PosHoldRpMode::PilotOverride;
                leftover_pitch_mode = PosHoldRpMode::PilotOverride;
                leftover_init_wind_comp = true;
                break;

            case AltHoldModeState::Landed_Ground_Idle:
                leftover_loiter_clear_pilot_accel = true;
                leftover_loiter_init_target = true;
                leftover_reset_yaw_target_and_rate = true;
                leftover_init_wind_comp = true;
                [[fallthrough]];

            case AltHoldModeState::Landed_Pre_Takeoff:
                leftover_reset_I_smoothly = true;
                leftover_d_relax = true;
                leftover_roll_mode = PosHoldRpMode::PilotOverride;
                leftover_pitch_mode = PosHoldRpMode::PilotOverride;
                break;

            case AltHoldModeState::Takeoff:
                if (!leftover_takeoff_running) {
                    leftover_takeoff_start = true;
                }
                leftover_avoidance = true;
                leftover_do_pilot_takeoff = true;
                leftover_loiter_clear_pilot_accel = true;
                leftover_loiter_init_target = true;
                leftover_roll_mode = PosHoldRpMode::PilotOverride;
                leftover_pitch_mode = PosHoldRpMode::PilotOverride;
                break;

            case AltHoldModeState::Flying:
                // Upstream ~203-214: avoidance climbrate; surface_tracking;
                // D_set_pos_target_from_climb_rate. No loiter_nav->update here
                // (unlike ModeLoiter). Flags only.
                leftover_avoidance = true;
                leftover_surface_tracking = true;
                leftover_d_set_pos_from_climb = true;
                break;
        }

        // Upstream ~217-528: vel body-frame + wind_comp lean + roll/pitch RP
        // brake/loiter blend state machines — OOS this ticket (catalog
        // "PosHold brake / wind_comp blend").

        // Upstream ~530-534: attitude + D_update_controller after switch.
        leftover_attitude = true;
        leftover_d_update_controller = true;
    }
};

}  // namespace fwcpp::copter
