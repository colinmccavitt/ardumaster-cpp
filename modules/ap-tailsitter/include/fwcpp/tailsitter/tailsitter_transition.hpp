#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include <fwcpp/quadplane_transition/transition_state.hpp>
#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_transition_ramp.hpp>

namespace fwcpp::tailsitter {

enum class CompleteReason : std::uint8_t {
    kDisarmed,
    kPitch,
    kRollError,
    kTimeout,
    kZeroThrottle,
};

enum class TailsitterTransitionState : std::uint8_t {
    kAngleWaitFw = 0,
    kAngleWaitVtol = 1,
    kDone = 2,
};

struct TransitionCompleteSample {
    bool armed_and_safety_off{true};
    std::int32_t pitch_cd{0};
    std::int32_t roll_cd{0};
    std::int32_t roll_limit_cd{4500};
    std::uint32_t now_ms{0};
    bool is_vectored{false};
    float pilot_throttle{0.5f};
    float groundspeed_ms{10.0f};
    bool fly_inverted{false};
};

struct TransitionFwUpdate {
    bool use_synthetic_airspeed{false};
    bool assisted_flight{false};
    std::optional<std::int32_t> nav_pitch_cd{};
    std::optional<std::int32_t> nav_roll_cd{};
    std::optional<float> throttle{};
    bool start_fw_limit{false};
    std::optional<CompleteReason> completed{};
};

struct TransitionVtolUpdate {
    bool still_waiting{false};
    bool assisted_flight{false};
    bool start_vtol_limit{false};
    std::optional<CompleteReason> completed{};
};

[[nodiscard]] inline constexpr std::int32_t roll_error_limit_cd(std::int32_t roll_limit_cd) {
    const std::int32_t extra = roll_limit_cd + kRollErrorMarginCd;
    return extra > kRollErrorFloorCd ? extra : kRollErrorFloorCd;
}

[[nodiscard]] inline std::int32_t roll_abs_cd(std::int32_t roll_cd, bool fly_inverted) {
    const std::int32_t abs_roll = std::abs(roll_cd);
    if (fly_inverted) {
        return kInvertedRollCd - abs_roll;
    }
    return abs_roll;
}

[[nodiscard]] inline bool roll_past_error(std::int32_t roll_cd, std::int32_t roll_limit_cd,
                                          bool fly_inverted) {
    return roll_abs_cd(roll_cd, fly_inverted) > roll_error_limit_cd(roll_limit_cd);
}

[[nodiscard]] inline float fw_timeout_ms(std::int8_t angle_fw, float initial_pitch_cd, float rate_fw) {
    return (static_cast<float>(angle_fw) + initial_pitch_cd * 0.01f) / rate_fw * kTransitionTimeoutScale;
}

[[nodiscard]] inline float vtol_timeout_ms(std::int8_t angle_vtol, float initial_pitch_cd,
                                           float rate_vtol) {
    return (static_cast<float>(angle_vtol) - initial_pitch_cd * 0.01f) / rate_vtol *
           kTransitionTimeoutScale;
}

[[nodiscard]] inline bool elapsed_past_timeout(std::uint32_t now_ms, std::uint32_t start_ms,
                                               float timeout_ms) {
    const float elapsed = static_cast<float>(now_ms - start_ms);
    return elapsed > timeout_ms;
}

struct TailsitterTransition {
    TailsitterTransitionState state{TailsitterTransitionState::kDone};
    TransitionRamp ramp{};
    std::uint32_t vtol_transition_start_ms{0};
    float vtol_transition_initial_pitch{0.0f};
    std::uint32_t fw_transition_start_ms{0};
    float fw_transition_initial_pitch{0.0f};
    std::uint32_t last_vtol_mode_ms{0};
    std::uint32_t vtol_limit_start_ms{0};
    std::uint32_t fw_limit_start_ms{0};

    [[nodiscard]] constexpr bool complete() const {
        return state == TailsitterTransitionState::kDone;
    }

    [[nodiscard]] constexpr bool active_frwd() const {
        return state == TailsitterTransitionState::kAngleWaitFw;
    }

    [[nodiscard]] static float constrain_pitch_param(float pitch_cd) {
        const float limit = static_cast<float>(kPitchCdLimit);
        return std::clamp(pitch_cd, -limit, limit);
    }

    void restart(std::uint32_t now_ms, float attitude_target_pitch_cd) {
        state = TailsitterTransitionState::kAngleWaitFw;
        fw_transition_start_ms = now_ms;
        fw_transition_initial_pitch = constrain_pitch_param(attitude_target_pitch_cd);
    }

    void force_transition_complete(std::uint32_t now_ms, std::int32_t nav_pitch_cd) {
        state = TailsitterTransitionState::kDone;
        vtol_transition_start_ms = now_ms;
        vtol_transition_initial_pitch = constrain_pitch_param(static_cast<float>(nav_pitch_cd));
        fw_limit_start_ms = 0;
    }

    [[nodiscard]] std::optional<CompleteReason> transition_fw_complete(
        const TransitionCompleteSample& sample) const {
        if (!sample.armed_and_safety_off) {
            return CompleteReason::kDisarmed;
        }
        if (ramp.angle_complete(TransitionKind::kToFw, sample.pitch_cd)) {
            return CompleteReason::kPitch;
        }
        if (roll_past_error(sample.roll_cd, sample.roll_limit_cd, false)) {
            return CompleteReason::kRollError;
        }
        if (elapsed_past_timeout(
                sample.now_ms, fw_transition_start_ms,
                fw_timeout_ms(ramp.angle_fw, fw_transition_initial_pitch, ramp.rate_fw))) {
            return CompleteReason::kTimeout;
        }
        return std::nullopt;
    }

    [[nodiscard]] std::optional<CompleteReason> transition_vtol_complete(
        const TransitionCompleteSample& sample) const {
        if (!sample.armed_and_safety_off) {
            return CompleteReason::kDisarmed;
        }
        if (sample.is_vectored && sample.pilot_throttle < kVtolZeroThrottle &&
            sample.groundspeed_ms < kVtolZeroGroundspeedMs) {
            return CompleteReason::kZeroThrottle;
        }
        if (ramp.angle_complete(TransitionKind::kToVtol, sample.pitch_cd)) {
            return CompleteReason::kPitch;
        }
        if (roll_past_error(sample.roll_cd, sample.roll_limit_cd, sample.fly_inverted)) {
            return CompleteReason::kRollError;
        }
        if (elapsed_past_timeout(
                sample.now_ms, vtol_transition_start_ms,
                vtol_timeout_ms(ramp.get_transition_angle_vtol(), vtol_transition_initial_pitch,
                                ramp.rate_vtol))) {
            return CompleteReason::kTimeout;
        }
        return std::nullopt;
    }

    TransitionFwUpdate update(const TransitionCompleteSample& sample, bool inverted, float hover,
                              float current_throttle) {
        const bool use_synthetic_airspeed = state != TailsitterTransitionState::kDone;
        if (state != TailsitterTransitionState::kAngleWaitFw) {
            return TransitionFwUpdate{.use_synthetic_airspeed = use_synthetic_airspeed};
        }
        if (const auto reason = transition_fw_complete(sample)) {
            state = TailsitterTransitionState::kDone;
            const bool start_fw_limit = sample.armed_and_safety_off;
            if (start_fw_limit) {
                fw_limit_start_ms = sample.now_ms;
            }
            TransitionFwUpdate out{};
            out.use_synthetic_airspeed = use_synthetic_airspeed;
            out.start_fw_limit = start_fw_limit;
            out.completed = reason;
            return out;
        }
        const std::uint32_t dt = sample.now_ms - fw_transition_start_ms;
        TransitionFwUpdate out{};
        out.use_synthetic_airspeed = use_synthetic_airspeed;
        out.assisted_flight = true;
        out.nav_pitch_cd = ramp.pitch_cd(TransitionKind::kToFw, fw_transition_initial_pitch, dt,
                                         inverted);
        out.nav_roll_cd = 0;
        out.throttle = ramp.throttle(TransitionKind::kToFw, hover, 0.0f, current_throttle);
        return out;
    }

    TransitionVtolUpdate vtol_update(const TransitionCompleteSample& sample,
                                     float attitude_target_pitch_cd) {
        const std::uint32_t now = sample.now_ms;
        if (now - last_vtol_mode_ms > kLastVtolModeMs) {
            state = TailsitterTransitionState::kAngleWaitVtol;
        }
        last_vtol_mode_ms = now;

        if (state == TailsitterTransitionState::kAngleWaitVtol) {
            if (const auto reason = transition_vtol_complete(sample)) {
                const bool start_vtol_limit = sample.armed_and_safety_off;
                if (start_vtol_limit) {
                    vtol_limit_start_ms = now;
                }
                restart(now, attitude_target_pitch_cd);
                TransitionVtolUpdate out{};
                out.still_waiting = false;
                out.assisted_flight = true;
                out.start_vtol_limit = start_vtol_limit;
                out.completed = reason;
                return out;
            }
            return TransitionVtolUpdate{.still_waiting = true, .assisted_flight = true};
        }
        restart(now, attitude_target_pitch_cd);
        return TransitionVtolUpdate{};
    }

    [[nodiscard]] bool show_vtol_view(bool in_vtol_mode) const {
        if (in_vtol_mode && state == TailsitterTransitionState::kAngleWaitVtol) {
            return false;
        }
        if (!in_vtol_mode && state == TailsitterTransitionState::kAngleWaitFw) {
            return true;
        }
        return in_vtol_mode;
    }

    [[nodiscard]] std::uint8_t get_mav_vtol_state(bool in_vtol_mode) const {
        using fwcpp::quadplane_transition::kMavVtolStateFw;
        using fwcpp::quadplane_transition::kMavVtolStateMc;
        using fwcpp::quadplane_transition::kMavVtolStateTransitionToFw;
        using fwcpp::quadplane_transition::kMavVtolStateTransitionToMc;
        using fwcpp::quadplane_transition::kMavVtolStateUndefined;
        switch (state) {
            case TailsitterTransitionState::kAngleWaitVtol:
                return kMavVtolStateTransitionToMc;
            case TailsitterTransitionState::kDone:
                return kMavVtolStateFw;
            case TailsitterTransitionState::kAngleWaitFw:
                return in_vtol_mode ? kMavVtolStateMc : kMavVtolStateTransitionToFw;
        }
        return kMavVtolStateUndefined;
    }

    [[nodiscard]] bool is_in_fw_flight(bool enabled, bool in_vtol_mode) const {
        return enabled && !in_vtol_mode && complete();
    }

    [[nodiscard]] bool allow_weathervane(bool in_vtol_transition) const {
        return !in_vtol_transition && vtol_limit_start_ms == 0;
    }

    [[nodiscard]] bool allow_stick_mixing(bool in_vtol_transition) const {
        if (in_vtol_transition) {
            return false;
        }
        if (complete() && fw_limit_start_ms != 0) {
            return false;
        }
        return true;
    }

    void set_fw_roll_pitch(std::int32_t& nav_pitch_cd, std::int32_t& nav_roll_cd, std::uint32_t now_ms,
                           bool in_vtol_transition) {
        if (in_vtol_transition) {
            const std::uint32_t dt = now_ms - vtol_transition_start_ms;
            nav_pitch_cd = ramp.pitch_cd(TransitionKind::kToVtol, vtol_transition_initial_pitch, dt,
                                         false);
            nav_roll_cd = 0;
        } else if (complete()) {
            vtol_transition_start_ms = now_ms;
            vtol_transition_initial_pitch = constrain_pitch_param(static_cast<float>(nav_pitch_cd));
        }
    }
};

}  // namespace fwcpp::tailsitter
