#pragma once

#include <fwcpp/quadplane_transition/transition_state.hpp>

namespace fwcpp::quadplane_transition {

/// Upstream `SLT_Transition::active_frwd` (inputs injected; no QuadPlane ref).
[[nodiscard]] inline constexpr bool active_forward_transition(
    bool assisted_flight, TransitionState state, bool in_vtol_airbrake) {
    if (!assisted_flight) {
        return false;
    }
    if (!in_forward_transition(state)) {
        return false;
    }
    if (in_vtol_airbrake) {
        return false;
    }
    return true;
}

/// Upstream `SLT_Transition::show_vtol_view` (SLT returns `in_vtol_mode`).
[[nodiscard]] inline constexpr bool show_vtol_view_slt(bool in_vtol_mode) {
    return in_vtol_mode;
}

}  // namespace fwcpp::quadplane_transition
