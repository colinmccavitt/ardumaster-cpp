#pragma once

#include <fwcpp/quadplane_transition/transition_state.hpp>

namespace fwcpp::quadplane_transition {

/// Legal explicit state moves for slice-1 scaffold (full update() later).
[[nodiscard]] inline constexpr bool can_transition(TransitionState from, TransitionState to) {
    if (from == to) {
        return true;
    }
    switch (from) {
        case TransitionState::kAirspeedWait:
            return to == TransitionState::kTimer || to == TransitionState::kDone;
        case TransitionState::kTimer:
            return to == TransitionState::kAirspeedWait || to == TransitionState::kDone;
        case TransitionState::kDone:
            return to == TransitionState::kAirspeedWait;
    }
    return false;
}

/// Separate-lift-thrust transition object, upstream `SLT_Transition` (stub).
class SltTransition {
public:
    [[nodiscard]] static SltTransition with_defaults() { return SltTransition{}; }

    [[nodiscard]] TransitionState state() const { return state_; }
    [[nodiscard]] bool complete() const { return transition_complete(state_); }
    [[nodiscard]] bool in_transition() const { return in_forward_transition(state_); }
    [[nodiscard]] std::uint8_t get_log_transition_state() const {
        return log_transition_state(state_);
    }
    [[nodiscard]] bool in_forced_transition() const { return in_forced_transition_; }

    /// Assign `transition_state` when the move is allowed.
    [[nodiscard]] bool set_state(TransitionState to) {
        if (!can_transition(state_, to)) {
            return false;
        }
        state_ = to;
        return true;
    }

    /// Upstream `SLT_Transition::restart`.
    void restart() { state_ = TransitionState::kAirspeedWait; }

    /// Upstream `SLT_Transition::force_transition_complete` (timers cleared).
    void force_transition_complete() {
        state_ = TransitionState::kDone;
        in_forced_transition_ = false;
        transition_start_ms_ = 0;
        transition_low_airspeed_ms_ = 0;
    }

    /// Enter TIMER (stub; timing in later slices).
    void enter_timer() { set_state(TransitionState::kTimer); }

    [[nodiscard]] std::uint32_t transition_start_ms() const { return transition_start_ms_; }
    [[nodiscard]] std::uint32_t transition_low_airspeed_ms() const {
        return transition_low_airspeed_ms_;
    }

private:
    TransitionState state_{TransitionState::kAirspeedWait};
    bool in_forced_transition_{false};
    std::uint32_t transition_start_ms_{0};
    std::uint32_t transition_low_airspeed_ms_{0};
};

}  // namespace fwcpp::quadplane_transition
