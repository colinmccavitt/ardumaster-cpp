#pragma once

#include <fwcpp/quadplane_transition/transition_state.hpp>
#include <fwcpp/quadplane_transition/transition_timing.hpp>

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

/// Separate-lift-thrust transition object, upstream `SLT_Transition`.
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

    [[nodiscard]] std::uint32_t transition_start_ms() const { return transition_start_ms_; }
    [[nodiscard]] std::uint32_t transition_low_airspeed_ms() const {
        return transition_low_airspeed_ms_;
    }
    [[nodiscard]] std::int16_t transition_time_ms() const { return transition_time_ms_; }
    [[nodiscard]] std::uint32_t timer_duration_ms() const {
        return constrain_transition_time_ms(transition_time_ms_);
    }
    [[nodiscard]] float transition_decel_mss() const { return transition_decel_mss_; }
    [[nodiscard]] std::int16_t transition_fail_timeout_s() const {
        return transition_fail_timeout_s_;
    }
    [[nodiscard]] TransFailAction transition_fail_action() const {
        return trans_fail_action_from_param(transition_fail_action_);
    }
    [[nodiscard]] bool transition_fail_warned() const { return transition_fail_warned_; }
    [[nodiscard]] std::int32_t q_options() const { return q_options_; }
    [[nodiscard]] bool trans_fail_to_fw() const { return trans_fail_to_fw_set(q_options_); }

    void set_transition_time_ms(std::int16_t ms) { transition_time_ms_ = ms; }
    void set_transition_decel_mss(float decel_mss) { transition_decel_mss_ = decel_mss; }
    void set_transition_fail_timeout_s(std::int16_t timeout_s) {
        transition_fail_timeout_s_ = timeout_s;
    }
    void set_transition_fail_action(TransFailAction action) {
        transition_fail_action_ = trans_fail_action_as_i16(action);
    }
    void set_q_options(std::int32_t q_options) { q_options_ = q_options; }

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

    /// Upstream `SLT_Transition::force_transition_complete`.
    void force_transition_complete() {
        state_ = TransitionState::kDone;
        in_forced_transition_ = false;
        transition_start_ms_ = 0;
        transition_low_airspeed_ms_ = 0;
        transition_fail_warned_ = false;
    }

    /// Enter TIMER (does not stamp low-airspeed timer; prefer update_airspeed_wait).
    void enter_timer() { state_ = TransitionState::kTimer; }

    void reset_fail_timer_if_disarmed(std::uint32_t now_ms, bool armed_and_safety_off) {
        if (!armed_and_safety_off) {
            transition_start_ms_ = now_ms;
        }
    }

    TransFailOutcome apply_transition_fail(std::uint32_t now_ms, bool tiltrotor_with_ground_speed) {
        if (state_ != TransitionState::kAirspeedWait) {
            return TransFailOutcome::kContinue;
        }
        const std::int16_t timeout_s = transition_fail_timeout_s_;
        const bool timed_out = transition_start_ms_ != 0 && timeout_s > 0 &&
                               (now_ms - transition_start_ms_) >
                                   static_cast<std::uint32_t>(timeout_s) * 1000U;
        if (!timed_out) {
            transition_fail_warned_ = false;
            return TransFailOutcome::kContinue;
        }
        if (!transition_fail_warned_) {
            transition_fail_warned_ = true;
        }
        if (trans_fail_to_fw() && tiltrotor_with_ground_speed) {
            state_ = TransitionState::kTimer;
            in_forced_transition_ = true;
            return TransFailOutcome::kCompleteToFw;
        }
        switch (transition_fail_action()) {
            case TransFailAction::kQland:
                return TransFailOutcome::kFallbackQland;
            case TransFailAction::kQrtl:
                return TransFailOutcome::kFallbackQrtl;
            case TransFailAction::kWarnOnly:
                return TransFailOutcome::kWarnOnly;
        }
        return TransFailOutcome::kWarnOnly;
    }

    void update_airspeed_wait(std::uint32_t now_ms, bool have_airspeed, float aspeed,
                              float airspeed_min, bool assisted_flight) {
        if (transition_start_ms_ == 0) {
            transition_start_ms_ = now_ms;
        }
        transition_low_airspeed_ms_ = now_ms;
        if (have_airspeed && aspeed > airspeed_min && !assisted_flight) {
            state_ = TransitionState::kTimer;
        }
    }

    void update_timer(std::uint32_t now_ms, bool tilt_fwd_complete) {
        const std::uint32_t trans_time_ms = timer_duration_ms();
        const std::uint32_t transition_timer_ms = now_ms - transition_low_airspeed_ms_;
        if (transition_timer_ms > trans_time_ms && tilt_fwd_complete) {
            force_transition_complete();
        }
    }

    void apply_assist_back(std::uint32_t now_ms, bool should_assist) {
        if (should_assist && !in_forced_transition_) {
            state_ = TransitionState::kAirspeedWait;
            if (transition_start_ms_ == 0) {
                transition_start_ms_ = now_ms;
            }
        }
    }

    void update_forward_timing(std::uint32_t now_ms, bool have_airspeed, float aspeed,
                               float airspeed_min, bool should_assist, bool tilt_fwd_complete) {
        apply_assist_back(now_ms, should_assist);
        switch (state_) {
            case TransitionState::kAirspeedWait:
                update_airspeed_wait(now_ms, have_airspeed, aspeed, airspeed_min, should_assist);
                break;
            case TransitionState::kTimer:
                update_timer(now_ms, tilt_fwd_complete);
                break;
            case TransitionState::kDone:
                break;
        }
    }

    [[nodiscard]] float stopping_distance_m(float ground_speed_squared_m) const {
        return fwcpp::quadplane_transition::stopping_distance_m(ground_speed_squared_m,
                                                                transition_decel_mss_);
    }

    [[nodiscard]] float back_transition_time_s(float ground_speed_ms) const {
        return fwcpp::quadplane_transition::back_transition_time_s(ground_speed_ms,
                                                                     transition_decel_mss_);
    }

private:
    TransitionState state_{TransitionState::kAirspeedWait};
    bool in_forced_transition_{false};
    std::uint32_t transition_start_ms_{0};
    std::uint32_t transition_low_airspeed_ms_{0};
    std::int16_t transition_time_ms_{kQTransitionMsDefault};
    float transition_decel_mss_{kQTransDecelDefault};
    std::int16_t transition_fail_timeout_s_{kQTransFailDefault};
    std::int16_t transition_fail_action_{kQTransFailActDefault};
    bool transition_fail_warned_{false};
    std::int32_t q_options_{0};
};

}  // namespace fwcpp::quadplane_transition
