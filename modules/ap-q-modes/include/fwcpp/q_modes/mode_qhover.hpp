#pragma once

#include <fwcpp/q_modes/q_run_common.hpp>

#include <cstdint>

namespace fwcpp::q_modes {

struct QHoverEnterResult {
    bool entered{true};
    bool throttle_wait{true};
    float climb_rate_ms{0.0f};
};

[[nodiscard]] inline QHoverEnterResult qhover_enter() {
    QHoverEnterResult out{};
    out.climb_rate_ms = 0.0f;
    return out;
}

enum class QHoverRunPhase : std::uint8_t {
    kFwTransitionControllers = 0,
    kThrottleWait = 1,
    kHoldHover = 2,
};

struct QHoverRunInputs {
    bool tailsitter_in_vtol_transition{false};
    bool throttle_wait{false};
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
    bool stabilize_fw_surfaces{false};
    bool center_rudder{false};
    bool output_spin_recovery{false};
};

[[nodiscard]] inline QHoverRunActions qhover_run_actions(QHoverRunPhase phase) {
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
            out.stabilize_fw_surfaces = true;
            out.center_rudder = true;
            out.output_spin_recovery = true;
            break;
    }
    return out;
}

}  // namespace fwcpp::q_modes
