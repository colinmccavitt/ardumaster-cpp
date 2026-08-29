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
    bool stabilize_fw_surfaces{false};
    bool center_rudder{false};
    bool run_esc_calibration{false};
};

[[nodiscard]] inline QStabilizeRunActions qstabilize_run_actions(QStabilizeRunPhase phase) {
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
            out.stabilize_fw_surfaces = true;
            out.center_rudder = true;
            break;
    }
    return out;
}

}  // namespace fwcpp::q_modes
