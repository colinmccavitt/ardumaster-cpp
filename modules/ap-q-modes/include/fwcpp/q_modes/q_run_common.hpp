#pragma once

#include <cstdint>

namespace fwcpp::q_modes {

/// Shared first branch in ModeQ*::run when tailsitter is in FW pull-up of VTOL transition.
[[nodiscard]] inline constexpr bool run_delegates_to_fw_controllers(bool tailsitter_in_vtol_transition) {
    return tailsitter_in_vtol_transition;
}

struct QRunTiming {
    std::uint32_t now_ms{0};
};

/// Tail shared by ModeQStabilize/QHover::run after the VTOL-specific body (upstream stabilize_roll/pitch + rudder).
struct QRunFwSurfaceFollowup {
    bool stabilize_roll{false};
    bool stabilize_pitch{false};
    bool center_rudder{false};
    float rudder_steering{0.0f};
};

[[nodiscard]] inline QRunFwSurfaceFollowup q_run_fw_surface_followup() {
    QRunFwSurfaceFollowup out{};
    out.stabilize_roll = true;
    out.stabilize_pitch = true;
    out.center_rudder = true;
    out.rudder_steering = 0.0f;
    return out;
}

}  // namespace fwcpp::q_modes
