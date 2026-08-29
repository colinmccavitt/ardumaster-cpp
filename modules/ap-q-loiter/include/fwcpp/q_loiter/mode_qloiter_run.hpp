#pragma once

#include <fwcpp/q_loiter/mode_qloiter_precland_run.hpp>
#include <fwcpp/q_loiter/q_loiter_defaults.hpp>
#include <fwcpp/q_modes/q_run_common.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

#include <cstdint>

namespace fwcpp::q_loiter {

using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::PosControlState;

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
    bool ne_is_active{true};
    bool vtol_roll_pitch_limited{false};
    float pilot_climb_rate_cms{0.0F};
    fwcpp::quadplane::PositionControlState poscontrol_state{
        fwcpp::quadplane::PositionControlState::kLandDescend};
    bool disable_ground_effect_comp{false};
    bool poscontrol_is_land_final{false};
    std::uint32_t last_target_loc_set_ms{0};
    bool precland_rel_origin_valid{false};
    float precland_rel_origin_n_m{0.0F};
    float precland_rel_origin_e_m{0.0F};
    float precland_velocity_match_n_ms{0.0F};
    float precland_velocity_match_e_ms{0.0F};
    std::uint32_t precland_last_velocity_match_ms{0};
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
    bool qland_touchdown_expected{false};
    bool guided_zero_climb{false};
    bool pilot_climb_rate{false};
    bool run_z_controller{false};
    bool ground_idle_spool{false};
    bool throttle_out_zero{false};
    bool relax_attitude{false};
    bool relax_pos_z{false};
    bool clear_pilot_accel{false};
    bool ne_init_controller{false};
    bool ne_set_externally_limited{false};
    bool pilot_yaw_rate_time_constant{false};
    bool attitude_euler_input{false};
    bool process_pilot_lean_angles{false};
};

struct QLoiterRunEffects {
    QLoiterPreclandEffects precland{};
    bool set_poscontrol_land_final{false};
};

struct QLoiterRunResult {
    QLoiterRunPhase phase{QLoiterRunPhase::kLoiterControl};
    QLoiterVerticalBranch vertical{QLoiterVerticalBranch::kPilotClimb};
    QLoiterRunActions actions{};
    QLoiterRunEffects effects{};
    fwcpp::q_modes::QRunFwSurfaceFollowup fw_followup{};
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

namespace detail {

[[nodiscard]] inline QLoiterRunActions qloiter_throttle_wait_actions() {
    QLoiterRunActions out{};
    out.ground_idle_spool = true;
    out.throttle_out_zero = true;
    out.relax_attitude = true;
    out.relax_pos_z = true;
    out.clear_pilot_accel = true;
    out.reinit_loiter_target = true;
    out.stabilize_fw_surfaces = true;
    return out;
}

[[nodiscard]] inline QLoiterRunActions qloiter_loiter_control_actions(const QLoiterRunInputs& in) {
    QLoiterRunActions out{};
    if (!in.motors_armed) {
        out.reenter_on_disarm = true;
    }
    if (in.should_relax) {
        out.soften_for_landing = true;
    }
    if (qloiter_should_reinit_target(in.now_ms, in.last_loiter_ms)) {
        out.reinit_loiter_target = true;
        out.clear_pilot_accel = true;
    }
    out.throttle_unlimited_spool = true;
    out.set_pos_z_limits = true;
    out.process_pilot_lean_angles = true;
    if (!in.ne_is_active) {
        out.ne_init_controller = true;
    }
    out.loiter_nav_update = true;
    out.assign_tilt_to_fwd_thr = true;
    if (in.vtol_roll_pitch_limited) {
        out.ne_set_externally_limited = true;
    }
    out.pilot_yaw_rate_time_constant = true;
    out.attitude_euler_input = true;
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
            if (in.poscontrol_is_land_final && !in.disable_ground_effect_comp) {
                out.qland_touchdown_expected = true;
            }
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

[[nodiscard]] inline QLoiterRunActions qloiter_run_actions(QLoiterRunPhase phase,
                                                           const QLoiterRunInputs& in) {
    switch (phase) {
        case QLoiterRunPhase::kAssistRecovery: {
            QLoiterRunActions out{};
            out.delegate_qhover_run = true;
            return out;
        }
        case QLoiterRunPhase::kFwTransitionControllers: {
            QLoiterRunActions out{};
            out.delegate_mode_run = true;
            return out;
        }
        case QLoiterRunPhase::kThrottleWait:
            return qloiter_throttle_wait_actions();
        case QLoiterRunPhase::kLoiterControl:
            return qloiter_loiter_control_actions(in);
    }
    return qloiter_loiter_control_actions(in);
}

}  // namespace detail

[[nodiscard]] inline QLoiterPreclandInputs qloiter_precland_inputs_from(const QLoiterRunInputs& in) {
    QLoiterPreclandInputs pc{};
    pc.now_ms = in.now_ms;
    pc.last_target_loc_set_ms = in.last_target_loc_set_ms;
    pc.last_velocity_match_ms = in.precland_last_velocity_match_ms;
    pc.rel_origin_valid = in.precland_rel_origin_valid;
    pc.rel_origin_n_m = in.precland_rel_origin_n_m;
    pc.rel_origin_e_m = in.precland_rel_origin_e_m;
    pc.velocity_match_n_ms = in.precland_velocity_match_n_ms;
    pc.velocity_match_e_ms = in.precland_velocity_match_e_ms;
    return pc;
}

[[nodiscard]] inline QLoiterRunInputs qloiter_run_view_loiter() {
    QLoiterRunInputs in{};
    in.now_ms = 600;
    in.last_loiter_ms = 0;
    return in;
}

[[nodiscard]] inline QLoiterRunInputs qloiter_run_view_throttle_wait() {
    QLoiterRunInputs in = qloiter_run_view_loiter();
    in.throttle_wait = true;
    return in;
}

[[nodiscard]] inline QLoiterRunInputs qloiter_run_view_qland_descent() {
    QLoiterRunInputs in = qloiter_run_view_loiter();
    in.active_control_is_qland = true;
    in.poscontrol_before_land_final = true;
    in.check_land_final_true = false;
    in.poscontrol_is_land_final = false;
    return in;
}

[[nodiscard]] inline QLoiterRunInputs qloiter_run_view_qland_final() {
    QLoiterRunInputs in = qloiter_run_view_qland_descent();
    in.check_land_final_true = true;
    in.poscontrol_is_land_final = true;
    return in;
}

/// Port of ModeQLoiter::run (assist, precland overrides, throttle wait, loiter + QLAND vertical).
[[nodiscard]] inline QLoiterRunResult qloiter_run(QLoiterRunInputs in, PosControlState& pc) {
    QLoiterRunResult out{};
    out.effects.precland = qloiter_precland_effects(qloiter_precland_inputs_from(in));
    if (out.effects.precland.clear_velocity_match_ms) {
        pc.last_velocity_match_ms = 0;
    }

    out.phase = qloiter_run_phase(in);
    out.actions = detail::qloiter_run_actions(out.phase, in);
    if (out.actions.qland_land_final_transition) {
        pc.state = PositionControlState::kLandFinal;
        out.effects.set_poscontrol_land_final = true;
    }
    if (out.phase == QLoiterRunPhase::kLoiterControl) {
        out.vertical = qloiter_vertical_branch(in);
    }
    if (out.actions.stabilize_fw_surfaces) {
        out.fw_followup = fwcpp::q_modes::q_run_fw_surface_followup();
    }
    return out;
}

[[nodiscard]] inline QLoiterRunResult qloiter_run(const QLoiterRunInputs& in) {
    PosControlState pc{};
    pc.last_velocity_match_ms = in.precland_last_velocity_match_ms;
    pc.state = in.poscontrol_state;
    return qloiter_run(in, pc);
}

}  // namespace fwcpp::q_loiter
