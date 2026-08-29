#pragma once

#include <fwcpp/q_modes/mode_qstabilize.hpp>
#include <fwcpp/q_modes/q_run_common.hpp>

#include <cstdint>

namespace fwcpp::q_modes {

struct QHoverEnterEffects {
    bool set_d_max_speed_accel{false};
    bool set_d_correction_speed_accel{false};
    float pilot_velocity_z_max_dn_ms{0.0f};
    float pilot_speed_z_max_up_ms{0.0f};
    float pilot_accel_z_mss{0.0f};
    bool set_climb_rate_zero{false};
    bool init_throttle_wait{false};
};

struct QHoverEnterInputs {
    float pilot_velocity_z_max_dn_ms{1.0f};
    float pilot_speed_z_max_up_ms{2.5f};
    float pilot_accel_z_mss{1.0f};
};

struct QHoverEnterResult {
    bool entered{true};
    bool throttle_wait{true};
    float climb_rate_ms{0.0f};
};

[[nodiscard]] inline QHoverEnterResult qhover_enter(const QHoverEnterInputs& in, QHoverEnterEffects& effects) {
    effects = QHoverEnterEffects{};
    effects.set_d_max_speed_accel = true;
    effects.set_d_correction_speed_accel = true;
    effects.pilot_velocity_z_max_dn_ms = in.pilot_velocity_z_max_dn_ms;
    effects.pilot_speed_z_max_up_ms = in.pilot_speed_z_max_up_ms;
    effects.pilot_accel_z_mss = in.pilot_accel_z_mss;
    effects.set_climb_rate_zero = true;
    effects.init_throttle_wait = true;

    QHoverEnterResult out{};
    out.climb_rate_ms = 0.0f;
    return out;
}

[[nodiscard]] inline QHoverEnterResult qhover_enter() {
    QHoverEnterEffects effects{};
    QHoverEnterInputs in{};
    return qhover_enter(in, effects);
}

enum class QHoverRunPhase : std::uint8_t {
    kFwTransitionControllers = 0,
    kThrottleWait = 1,
    kHoldHover = 2,
};

struct QHoverRunInputs {
    bool tailsitter_in_vtol_transition{false};
    bool throttle_wait{false};
    float pilot_climb_rate_cms{0.0f};
};

[[nodiscard]] inline QHoverRunPhase qhover_run_phase(const QHoverRunInputs& in) {
    if (run_delegates_to_fw_controllers(in.tailsitter_in_vtol_transition)) {
        return QHoverRunPhase::kFwTransitionControllers;
    }
    if (in.throttle_wait) {
        return QHoverRunPhase::kThrottleWait;
    }
    return QHoverRunPhase::kHoldHover;
}

struct QHoverRunActions {
    bool check_vtol_recovery{false};
    bool ground_idle_spool{false};
    bool relax_attitude{false};
    bool relax_pos_z{false};
    bool assign_tilt_to_fwd_thr{false};
    bool hold_hover{false};
    float hold_hover_climb_rate_cms{0.0f};
    bool stabilize_fw_surfaces{false};
    bool center_rudder{false};
    bool output_spin_recovery{false};
};

[[nodiscard]] inline QHoverRunActions qhover_run_actions(QHoverRunPhase phase, float pilot_climb_rate_cms = 0.0f) {
    QHoverRunActions out{};
    out.check_vtol_recovery = true;
    switch (phase) {
        case QHoverRunPhase::kFwTransitionControllers:
            break;
        case QHoverRunPhase::kThrottleWait:
            out.ground_idle_spool = true;
            out.relax_attitude = true;
            out.relax_pos_z = true;
            break;
        case QHoverRunPhase::kHoldHover:
            out.assign_tilt_to_fwd_thr = true;
            out.hold_hover = true;
            out.hold_hover_climb_rate_cms = pilot_climb_rate_cms;
            out.stabilize_fw_surfaces = true;
            out.center_rudder = true;
            out.output_spin_recovery = true;
            break;
    }
    return out;
}

struct QHoverRunResult {
    QHoverRunPhase phase{QHoverRunPhase::kHoldHover};
    QHoverRunActions actions{};
    QRunFwSurfaceFollowup fw_followup{};
    bool delegate_mode_run{false};
};

/// Port of ModeQHover::run (assist recovery check, throttle wait vs hold_hover, FW surfaces + spin recovery).
[[nodiscard]] inline QHoverRunResult qhover_run(const QHoverRunInputs& in) {
    QHoverRunResult out{};
    out.phase = qhover_run_phase(in);
    if (out.phase == QHoverRunPhase::kFwTransitionControllers) {
        out.delegate_mode_run = true;
        out.actions.check_vtol_recovery = true;
        return out;
    }
    out.actions = qhover_run_actions(out.phase, in.pilot_climb_rate_cms);
    if (out.actions.stabilize_fw_surfaces) {
        out.fw_followup = q_run_fw_surface_followup();
    }
    return out;
}

[[nodiscard]] inline QStabilizeUpdateResult qhover_update(const QStabilizeUpdateInputs& in) {
    return qstabilize_update(in);
}

}  // namespace fwcpp::q_modes
