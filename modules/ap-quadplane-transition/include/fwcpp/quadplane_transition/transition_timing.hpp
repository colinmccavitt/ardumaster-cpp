#pragma once

#include <cstdint>

namespace fwcpp::quadplane_transition {

inline constexpr std::int16_t kQTransitionMsDefault = 5000;
inline constexpr std::int16_t kQTransitionMsMin = 500;
inline constexpr std::int16_t kQTransitionMsMax = 30000;
inline constexpr float kQTransDecelDefault = 2.0f;
inline constexpr std::int16_t kQTransFailDefault = 0;
inline constexpr std::int16_t kQTransFailActDefault = 0;
inline constexpr std::int32_t kQOptionsTransFailToFw = 1 << 19;

inline constexpr std::uint8_t kModeQstabilize = 17;
inline constexpr std::uint8_t kModeQland = 20;
inline constexpr std::uint8_t kModeQrtl = 21;
inline constexpr std::uint8_t kModeReasonVtolFailedTransition = 23;

enum class TransFailAction : std::int16_t {
    kWarnOnly = -1,
    kQland = 0,
    kQrtl = 1,
};

enum class TransFailOutcome : std::uint8_t {
    kContinue,
    kWarnOnly,
    kFallbackQland,
    kFallbackQrtl,
    kCompleteToFw,
};

[[nodiscard]] inline constexpr TransFailAction trans_fail_action_from_param(std::int16_t v) {
    switch (v) {
        case 0:
            return TransFailAction::kQland;
        case 1:
            return TransFailAction::kQrtl;
        default:
            return TransFailAction::kWarnOnly;
    }
}

[[nodiscard]] inline constexpr std::int16_t trans_fail_action_as_i16(TransFailAction action) {
    return static_cast<std::int16_t>(action);
}

[[nodiscard]] inline constexpr bool trans_fail_outcome_requests_q_fallback(TransFailOutcome outcome) {
    return outcome == TransFailOutcome::kFallbackQland || outcome == TransFailOutcome::kFallbackQrtl;
}

[[nodiscard]] inline constexpr std::uint8_t trans_fail_outcome_fallback_mode(TransFailOutcome outcome) {
    switch (outcome) {
        case TransFailOutcome::kFallbackQland:
            return kModeQland;
        case TransFailOutcome::kFallbackQrtl:
            return kModeQrtl;
        default:
            return 0;
    }
}

[[nodiscard]] inline constexpr bool trans_fail_to_fw_set(std::int32_t q_options) {
    return (q_options & kQOptionsTransFailToFw) != 0;
}

[[nodiscard]] inline constexpr std::uint32_t constrain_transition_time_ms(std::int16_t ms) {
    const std::int16_t v =
        ms < kQTransitionMsMin ? kQTransitionMsMin
                               : (ms > kQTransitionMsMax ? kQTransitionMsMax : ms);
    return static_cast<std::uint32_t>(v);
}

[[nodiscard]] inline constexpr float stopping_distance_m(float ground_speed_squared_m,
                                                         float transition_decel_mss) {
    return ground_speed_squared_m / (2.0f * transition_decel_mss);
}

[[nodiscard]] inline constexpr float back_transition_time_s(float ground_speed_ms,
                                                            float transition_decel_mss) {
    return ground_speed_ms / transition_decel_mss;
}

}  // namespace fwcpp::quadplane_transition
