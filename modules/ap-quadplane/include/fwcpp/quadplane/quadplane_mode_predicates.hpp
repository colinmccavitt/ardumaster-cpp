#pragma once

#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

#include <cstdint>

namespace fwcpp::quadplane {

struct InVtolModeInputs {
    bool available{false};
    bool in_vtol_land_sequence{false};
    PositionControlState pos_state{PositionControlState::kNone};
    bool control_is_vtol_mode{false};
    bool control_is_guided_mode{false};
    bool auto_vtol_loiter{false};
    bool mode_is_guided{false};
    bool guided_takeoff{false};
    bool in_vtol_auto{false};
};

[[nodiscard]] inline constexpr bool poscontrol_state_after(PositionControlState state,
                                                             PositionControlState threshold) {
    return static_cast<std::uint8_t>(state) > static_cast<std::uint8_t>(threshold);
}

/// Upstream `QuadPlane::in_vtol_mode()` (`quadplane.cpp`).
[[nodiscard]] inline bool compute_in_vtol_mode(const InVtolModeInputs& in) {
    if (!in.available) {
        return false;
    }
    if (in.in_vtol_land_sequence) {
        return in.pos_state != PositionControlState::kApproach &&
               in.pos_state != PositionControlState::kAirbrake;
    }
    if (in.control_is_vtol_mode) {
        return true;
    }
    if (in.control_is_guided_mode && in.auto_vtol_loiter &&
        poscontrol_state_after(in.pos_state, PositionControlState::kApproach)) {
        return true;
    }
    if (in.mode_is_guided && in.guided_takeoff) {
        return true;
    }
    if (in.in_vtol_auto) {
        if (!in.auto_vtol_loiter || poscontrol_state_after(in.pos_state, PositionControlState::kAirbrake)) {
            return true;
        }
    }
    return false;
}

}  // namespace fwcpp::quadplane
