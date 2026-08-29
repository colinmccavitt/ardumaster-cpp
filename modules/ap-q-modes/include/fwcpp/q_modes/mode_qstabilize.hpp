#pragma once

#include <fwcpp/q_modes/q_run_common.hpp>

#include <cstdint>

#include <algorithm>

namespace fwcpp::q_modes {

struct QStabilizeEnterResult {
    bool entered{true};
    bool throttle_wait{false};
};

[[nodiscard]] inline QStabilizeEnterResult qstabilize_enter() {
    QStabilizeEnterResult out{};
    out.throttle_wait = false;
    return out;
}

enum class QStabilizeRunPhase : std::uint8_t {
    kFwTransitionControllers = 0,
    kEscCalibration = 1,
    kNormal = 2,
};

struct QStabilizeRunInputs {
    bool tailsitter_in_vtol_transition{false};
    std::int8_t esc_calibration{0};
    float pilot_throttle_scaled{0.0f};
};

[[nodiscard]] inline QStabilizeRunPhase qstabilize_run_phase(const QStabilizeRunInputs& in) {
    if (run_delegates_to_fw_controllers(in.tailsitter_in_vtol_transition)) {
        return QStabilizeRunPhase::kFwTransitionControllers;
    }
    if (in.esc_calibration != 0) {
        return QStabilizeRunPhase::kEscCalibration;
    }
    return QStabilizeRunPhase::kNormal;
}

struct QStabilizeRunActions {
    bool assign_tilt_to_fwd_thr{false};
    bool hold_stabilize{false};
    float hold_stabilize_throttle{0.0f};
    bool stabilize_fw_surfaces{false};
    bool center_rudder{false};
    bool run_esc_calibration{false};
};

[[nodiscard]] inline QStabilizeRunActions qstabilize_run_actions(QStabilizeRunPhase phase,
                                                                 float pilot_throttle_scaled = 0.0f) {
    QStabilizeRunActions out{};
    switch (phase) {
        case QStabilizeRunPhase::kFwTransitionControllers:
            break;
        case QStabilizeRunPhase::kEscCalibration:
            out.run_esc_calibration = true;
            out.stabilize_fw_surfaces = true;
            break;
        case QStabilizeRunPhase::kNormal:
            out.assign_tilt_to_fwd_thr = true;
            out.hold_stabilize = true;
            out.hold_stabilize_throttle = pilot_throttle_scaled;
            out.stabilize_fw_surfaces = true;
            out.center_rudder = true;
            break;
    }
    return out;
}

struct QStabilizeRunResult {
    QStabilizeRunPhase phase{QStabilizeRunPhase::kNormal};
    QStabilizeRunActions actions{};
    QRunFwSurfaceFollowup fw_followup{};
    bool delegate_mode_run{false};
};

/// Port of ModeQStabilize::run control flow (assign_tilt / ESC cal / hold_stabilize + FW surfaces).
[[nodiscard]] inline QStabilizeRunResult qstabilize_run(const QStabilizeRunInputs& in) {
    QStabilizeRunResult out{};
    out.phase = qstabilize_run_phase(in);
    if (out.phase == QStabilizeRunPhase::kFwTransitionControllers) {
        out.delegate_mode_run = true;
        return out;
    }
    out.actions = qstabilize_run_actions(out.phase, in.pilot_throttle_scaled);
    if (out.actions.stabilize_fw_surfaces) {
        out.fw_followup = q_run_fw_surface_followup();
    }
    return out;
}


struct QStabilizeNavTargets {
    float nav_roll_cd{0.0f};
    float nav_pitch_cd{0.0f};
};

struct LimitedRollPitchInputs {
    float roll_input{0.0f};
    float pitch_input{0.0f};
    float roll_limit_cd{4500.0f};
    float pitch_limit_max_deg{20.0f};
    float pitch_limit_min_deg{25.0f};
    float lean_angle_max_cd{4500.0f};
};

[[nodiscard]] inline QStabilizeNavTargets set_limited_roll_pitch(const LimitedRollPitchInputs& in) {
    QStabilizeNavTargets out{};
    const float angle_max_cd = in.lean_angle_max_cd;
    out.nav_roll_cd = in.roll_input * std::min(in.roll_limit_cd, angle_max_cd);
    if (in.pitch_input > 0.0f) {
        out.nav_pitch_cd =
            in.pitch_input * std::min(in.pitch_limit_max_deg * 100.0f, angle_max_cd);
    } else {
        out.nav_pitch_cd =
            in.pitch_input * std::min(-in.pitch_limit_min_deg * 100.0f, angle_max_cd);
    }
    return out;
}

struct TailsitterRollPitchInputs {
    float roll_input{0.0f};
    float pitch_input{0.0f};
    float tailsitter_max_roll_angle_deg{0.0f};
    float lean_angle_max_cd{4500.0f};
};

struct TailsitterRollPitchResult {
    QStabilizeNavTargets nav{};
    bool apply_vtol_roll_pitch_limit{false};
};

[[nodiscard]] inline TailsitterRollPitchResult set_tailsitter_roll_pitch(const TailsitterRollPitchInputs& in) {
    TailsitterRollPitchResult out{};
    if (in.tailsitter_max_roll_angle_deg > 0.0f) {
        out.nav.nav_roll_cd = in.tailsitter_max_roll_angle_deg * 100.0f * in.roll_input;
    } else {
        out.nav.nav_roll_cd = in.roll_input * in.lean_angle_max_cd;
    }
    out.nav.nav_pitch_cd = in.pitch_input * in.lean_angle_max_cd;
    out.apply_vtol_roll_pitch_limit = true;
    return out;
}

struct QStabilizeUpdateInputs {
    float roll_control_in{0.0f};
    float roll_range{4500.0f};
    float pitch_control_in{0.0f};
    float pitch_range{4500.0f};
    bool tailsitter_active{false};
    bool ignore_fw_angle_limits_in_q_modes{false};
    float roll_limit_cd{4500.0f};
    float pitch_limit_max_deg{20.0f};
    float pitch_limit_min_deg{25.0f};
    float lean_angle_max_cd{4500.0f};
    float tailsitter_max_roll_angle_deg{0.0f};
};

struct QStabilizeUpdateResult {
    QStabilizeNavTargets nav{};
    bool apply_vtol_roll_pitch_limit{false};
};

[[nodiscard]] inline QStabilizeUpdateResult qstabilize_update(const QStabilizeUpdateInputs& in) {
    QStabilizeUpdateResult out{};
    const float roll_input = in.roll_control_in / in.roll_range;
    const float pitch_input = in.pitch_control_in / in.pitch_range;

    if (in.tailsitter_active) {
        const auto ts = set_tailsitter_roll_pitch(
            {roll_input, pitch_input, in.tailsitter_max_roll_angle_deg, in.lean_angle_max_cd});
        out.nav = ts.nav;
        out.apply_vtol_roll_pitch_limit = ts.apply_vtol_roll_pitch_limit;
        return out;
    }

    if (!in.ignore_fw_angle_limits_in_q_modes) {
        out.nav = set_limited_roll_pitch({roll_input, pitch_input, in.roll_limit_cd, in.pitch_limit_max_deg,
                                          in.pitch_limit_min_deg, in.lean_angle_max_cd});
    } else {
        out.nav.nav_roll_cd = roll_input * in.lean_angle_max_cd;
        out.nav.nav_pitch_cd = pitch_input * in.lean_angle_max_cd;
    }
    return out;
}


}  // namespace fwcpp::q_modes
