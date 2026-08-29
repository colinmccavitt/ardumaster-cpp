#pragma once

#include <fwcpp/q_modes/q_run_common.hpp>

#include <cstdint>

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

}  // namespace fwcpp::q_modes
