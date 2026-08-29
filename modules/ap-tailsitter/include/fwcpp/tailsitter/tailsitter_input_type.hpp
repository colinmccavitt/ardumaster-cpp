#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_enable.hpp>
#include <fwcpp/tailsitter/tailsitter_types.hpp>

namespace fwcpp::tailsitter {

inline constexpr float kFltEpsilon = 1.19209290e-7f;

[[nodiscard]] inline constexpr bool is_zero(float v) {
    return std::fabs(v) < kFltEpsilon;
}

struct TailsitterInputContext {
    TailsitterGate gate{};
    std::uint8_t frame_class{0};
    std::uint16_t motor_mask{0};
    float vectored_hover_gain{0.0f};
    bool tilt_motor_left{false};
    bool tilt_motor_right{false};
    std::int8_t input_type{kTailsitInputDefault};
};

[[nodiscard]] inline bool is_vectored(const TailsitterInputContext& ctx) {
    return ctx.frame_class == kMotorFrameTailsitter && !is_zero(ctx.vectored_hover_gain) &&
           (ctx.tilt_motor_left || ctx.tilt_motor_right);
}

[[nodiscard]] inline bool is_control_surface_tailsitter(const TailsitterInputContext& ctx) {
    return ctx.frame_class == kMotorFrameTailsitter &&
           (is_zero(ctx.vectored_hover_gain) || !ctx.tilt_motor_left);
}

[[nodiscard]] inline std::optional<InputType> resolve_input_type(const TailsitterInputContext& ctx) {
    if (!ctx.gate.enabled()) {
        return std::nullopt;
    }
    if (is_vectored(ctx)) {
        return InputType::kVectoredYaw;
    }
    if (is_control_surface_tailsitter(ctx)) {
        return InputType::kControlSurfaces;
    }
    return std::nullopt;
}

[[nodiscard]] inline bool input_plane_mode(std::int8_t input_type) {
    return (static_cast<std::uint8_t>(input_type) & kTailsitterInputPlane) != 0u;
}

[[nodiscard]] inline bool input_body_frame_roll(std::int8_t input_type) {
    return (static_cast<std::uint8_t>(input_type) & kTailsitterInputBfRoll) != 0u;
}

}  // namespace fwcpp::tailsitter
