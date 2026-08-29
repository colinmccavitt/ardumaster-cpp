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

}  // namespace fwcpp::quadplane_transition
