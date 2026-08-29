#pragma once

#include <cstdint>

namespace fwcpp::quadplane_transition {

/// Upstream `SLT_Transition::State` (`ArduPlane/transition.h`).
enum class TransitionState : std::uint8_t {
    kAirspeedWait = 0,
    kTimer = 1,
    kDone = 2,
};

/// High-level AIR / VTOL / TRANSITION phase (upstream `get_mav_vtol_state`).
enum class TransitionPhase : std::uint8_t {
    kAir,
    kVtol,
    kTransition,
};

[[nodiscard]] inline constexpr TransitionState default_transition_state() {
    return TransitionState::kAirspeedWait;
}

[[nodiscard]] inline constexpr std::uint8_t log_transition_state(TransitionState state) {
    return static_cast<std::uint8_t>(state);
}

[[nodiscard]] inline constexpr bool transition_complete(TransitionState state) {
    return state == TransitionState::kDone;
}

[[nodiscard]] inline constexpr bool in_forward_transition(TransitionState state) {
    return state == TransitionState::kAirspeedWait || state == TransitionState::kTimer;
}


/// MAVLink MAV_VTOL_STATE values used by `get_mav_vtol_state`.
inline constexpr std::uint8_t kMavVtolStateUndefined = 0;
inline constexpr std::uint8_t kMavVtolStateTransitionToFw = 1;
inline constexpr std::uint8_t kMavVtolStateTransitionToMc = 2;
inline constexpr std::uint8_t kMavVtolStateMc = 3;
inline constexpr std::uint8_t kMavVtolStateFw = 4;

[[nodiscard]] inline constexpr TransitionPhase transition_phase_from_inputs(
    TransitionState state, bool in_vtol_mode, bool assisted_flight) {
    if (in_vtol_mode) {
        return TransitionPhase::kVtol;
    }
    if (assisted_flight && in_forward_transition(state)) {
        return TransitionPhase::kTransition;
    }
    if (transition_complete(state)) {
        return TransitionPhase::kAir;
    }
    return TransitionPhase::kTransition;
}

[[nodiscard]] inline constexpr std::uint8_t mav_vtol_state_slt(
    TransitionState state, bool in_vtol_mode, bool transition_to_mc) {
    if (transition_to_mc) {
        return kMavVtolStateTransitionToMc;
    }
    if (in_vtol_mode) {
        return kMavVtolStateMc;
    }
    switch (state) {
        case TransitionState::kAirspeedWait:
        case TransitionState::kTimer:
            return kMavVtolStateTransitionToFw;
        case TransitionState::kDone:
            return kMavVtolStateFw;
    }
    return kMavVtolStateUndefined;
}

}  // namespace fwcpp::quadplane_transition
