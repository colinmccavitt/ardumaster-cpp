#pragma once

#include <fwcpp/q_loiter/q_loiter_defaults.hpp>
#include <fwcpp/q_modes/q_run_common.hpp>

#include <cstdint>

namespace fwcpp::q_loiter {

enum class QLoiterRunPhase : std::uint8_t {
    kAssistRecovery = 0,
    kFwTransitionControllers = 1,
    kThrottleWait = 2,
    kLoiterControl = 3,
};

enum class QLoiterVerticalBranch : std::uint8_t {
    kPilotClimb = 0,
    kGuidedTakeoffHold = 1,
    kQlandDescent = 2,
};

struct QLoiterRunInputs {
    bool assist_vtol_recovery{false};
    bool tailsitter_in_vtol_transition{false};
    bool throttle_wait{false};
    bool motors_armed{true};
    bool should_relax{false};
    std::uint32_t now_ms{0};
    std::uint32_t last_loiter_ms{0};
    bool active_control_is_qland{false};
    bool active_control_is_guided_takeoff{false};
    bool poscontrol_before_land_final{true};
    bool check_land_final_true{false};
};

struct QLoiterRunActions {
    bool delegate_qhover_run{false};
    bool delegate_mode_run{false};
    bool reenter_on_disarm{false};
    bool soften_for_landing{false};
    bool reinit_loiter_target{false};
    bool throttle_unlimited_spool{false};
    bool set_pos_z_limits{false};
    bool loiter_nav_update{false};
    bool assign_tilt_to_fwd_thr{false};
    bool stabilize_fw_surfaces{false};
    bool center_rudder{false};
    bool qland_land_final_transition{false};
    bool qland_descent_rate{false};
    bool qland_check_complete{false};
    bool guided_zero_climb{false};
    bool pilot_climb_rate{false};
    bool run_z_controller{false};
};

[[nodiscard]] inline bool qloiter_should_reinit_target(std::uint32_t now_ms,
                                                       std::uint32_t last_loiter_ms) {
    return now_ms - last_loiter_ms > kLoiterTargetReinitMs;
}

[[nodiscard]] inline QLoiterRunPhase qloiter_run_phase(const QLoiterRunInputs& in) {
    if (in.assist_vtol_recovery) {
        return QLoiterRunPhase::kAssistRecovery;
    }
    if (fwcpp::q_modes::run_delegates_to_fw_controllers(in.tailsitter_in_vtol_transition)) {
        return QLoiterRunPhase::kFwTransitionControllers;
    }
    if (in.throttle_wait) {
        return QLoiterRunPhase::kThrottleWait;
    }
    return QLoiterRunPhase::kLoiterControl;
}

[[nodiscard]] inline QLoiterVerticalBranch qloiter_vertical_branch(const QLoiterRunInputs& in) {
    if (in.active_control_is_qland) {
        return QLoiterVerticalBranch::kQlandDescent;
    }
    if (in.active_control_is_guided_takeoff) {
        return QLoiterVerticalBranch::kGuidedTakeoffHold;
    }
    return QLoiterVerticalBranch::kPilotClimb;
}

[[nodiscard]] inline QLoiterRunActions qloiter_run_actions(QLoiterRunPhase phase,
                                                           const QLoiterRunInputs& in) {
    QLoiterRunActions out{};
    switch (phase) {
        case QLoiterRunPhase::kAssistRecovery:
            out.delegate_qhover_run = true;
            return out;
        case QLoiterRunPhase::kFwTransitionControllers:
            out.delegate_mode_run = true;
            return out;
        case QLoiterRunPhase::kThrottleWait:
            out.reinit_loiter_target = true;
            out.stabilize_fw_surfaces = true;
            return out;
        case QLoiterRunPhase::kLoiterControl:
            break;
    }

    if (!in.motors_armed) {
        out.reenter_on_disarm = true;
    }
    if (in.should_relax) {
        out.soften_for_landing = true;
    }
    if (qloiter_should_reinit_target(in.now_ms, in.last_loiter_ms)) {
        out.reinit_loiter_target = true;
    }
    out.throttle_unlimited_spool = true;
    out.set_pos_z_limits = true;
    out.loiter_nav_update = true;
    out.assign_tilt_to_fwd_thr = true;
    out.stabilize_fw_surfaces = true;
    out.center_rudder = true;
    out.run_z_controller = true;

    switch (qloiter_vertical_branch(in)) {
        case QLoiterVerticalBranch::kQlandDescent:
            if (in.poscontrol_before_land_final && in.check_land_final_true) {
                out.qland_land_final_transition = true;
            }
            out.qland_descent_rate = true;
            out.qland_check_complete = true;
            break;
        case QLoiterVerticalBranch::kGuidedTakeoffHold:
            out.guided_zero_climb = true;
            break;
        case QLoiterVerticalBranch::kPilotClimb:
            out.pilot_climb_rate = true;
            break;
    }
    return out;
}

struct QLoiterRunResult {
    QLoiterRunPhase phase{QLoiterRunPhase::kLoiterControl};
    QLoiterVerticalBranch vertical{QLoiterVerticalBranch::kPilotClimb};
    QLoiterRunActions actions{};
    fwcpp::q_modes::QRunFwSurfaceFollowup fw_followup{};
};

[[nodiscard]] inline QLoiterRunResult qloiter_run(const QLoiterRunInputs& in) {
    QLoiterRunResult out{};
    out.phase = qloiter_run_phase(in);
    out.actions = qloiter_run_actions(out.phase, in);
    if (out.phase == QLoiterRunPhase::kLoiterControl) {
        out.vertical = qloiter_vertical_branch(in);
    }
    if (out.actions.stabilize_fw_surfaces) {
        out.fw_followup = fwcpp::q_modes::q_run_fw_surface_followup();
    }
    return out;
}

}  // namespace fwcpp::q_loiter
