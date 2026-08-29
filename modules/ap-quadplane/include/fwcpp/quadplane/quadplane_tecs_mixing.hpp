#pragma once

#include <fwcpp/quadplane/quadplane_auto_vtol_mission.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

#include <cstdint>

namespace fwcpp::quadplane {

[[nodiscard]] inline bool in_vtol_land_descent_phase(PositionControlState state) {
    return state == PositionControlState::kLandDescend ||
           state == PositionControlState::kLandFinal ||
           state == PositionControlState::kLandAbort;
}

struct InVtolLandDescentInputs {
    bool control_is_qrtl{false};
    bool in_vtol_auto{false};
    std::uint16_t mission_nav_cmd_id{0};
    bool available{false};
    std::int32_t options{0};
    PositionControlState pos_state{PositionControlState::kNone};
};

[[nodiscard]] inline bool compute_in_vtol_land_descent(const InVtolLandDescentInputs& in) {
    if (in.control_is_qrtl && in_vtol_land_descent_phase(in.pos_state)) {
        return true;
    }
    if (in.in_vtol_auto &&
        is_vtol_land(in.mission_nav_cmd_id, in.available, in.options) &&
        in_vtol_land_descent_phase(in.pos_state)) {
        return true;
    }
    return false;
}

struct ShouldDisableTecsInputs {
    InVtolLandDescentInputs land_descent{};
    bool control_is_guided{false};
    bool auto_vtol_loiter{false};
};

[[nodiscard]] inline bool should_disable_TECS(const ShouldDisableTecsInputs& in) {
    if (compute_in_vtol_land_descent(in.land_descent)) {
        return true;
    }
    if (in.control_is_guided && in.auto_vtol_loiter) {
        return true;
    }
    return false;
}

}  // namespace fwcpp::quadplane
