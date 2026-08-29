#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/tailsitter/tailsitter_defaults.hpp>

namespace fwcpp::tailsitter {

enum class TransitionKind {
    kToFw,
    kToVtol,
};

struct TransitionRamp {
    std::int8_t angle_fw{kTransitionAngleFwDefault};
    std::int8_t angle_vtol{kTransitionAngleVtolDefault};
    float rate_fw{kTransitionRateFwDefault};
    float rate_vtol{kTransitionRateVtolDefault};
    float throttle_vtol{kTransitionThrottleVtolDefault};
    float throttle_scale_max{2.0f};
    float gain_scaling_min{0.4f};

    [[nodiscard]] constexpr std::int8_t get_transition_angle_vtol() const {
        return angle_vtol == 0 ? angle_fw : angle_vtol;
    }

    [[nodiscard]] static constexpr float pitch_delta_cd(float rate_deg_s, std::uint32_t dt_ms) {
        return rate_deg_s * static_cast<float>(dt_ms) * 0.1f;
    }

    [[nodiscard]] static std::int32_t constrain_pitch_cd(float v) {
        const float limit = static_cast<float>(kPitchCdLimit);
        const float c = std::clamp(v, -limit, limit);
        return static_cast<std::int32_t>(c);
    }

    [[nodiscard]] std::int32_t pitch_cd(TransitionKind kind, float initial_pitch_cd, std::uint32_t dt_ms,
                                        bool inverted) const {
        float raw = initial_pitch_cd;
        switch (kind) {
            case TransitionKind::kToFw: {
                const float sign = inverted ? -1.0f : 1.0f;
                raw = initial_pitch_cd - pitch_delta_cd(rate_fw, dt_ms) * sign;
                break;
            }
            case TransitionKind::kToVtol:
                raw = initial_pitch_cd + pitch_delta_cd(rate_vtol, dt_ms);
                break;
        }
        return constrain_pitch_cd(raw);
    }

    [[nodiscard]] bool angle_complete(TransitionKind kind, std::int32_t pitch_cd) const {
        const std::int8_t angle =
            (kind == TransitionKind::kToFw) ? angle_fw : get_transition_angle_vtol();
        const std::uint32_t threshold =
            static_cast<std::uint32_t>(std::abs(static_cast<int>(angle))) * 100u;
        const std::uint32_t abs_pitch = static_cast<std::uint32_t>(std::abs(pitch_cd));
        return abs_pitch > threshold;
    }

    [[nodiscard]] float throttle(TransitionKind kind, float hover, float cruise_pct,
                                 float current) const {
        switch (kind) {
            case TransitionKind::kToVtol:
                if (throttle_vtol < 0.0f) {
                    return std::max(hover, cruise_pct * 0.01f);
                }
                return std::min(throttle_vtol * 0.01f, 1.0f);
            case TransitionKind::kToFw:
                return std::max(hover, current);
        }
        return hover;
    }
};

}  // namespace fwcpp::tailsitter
