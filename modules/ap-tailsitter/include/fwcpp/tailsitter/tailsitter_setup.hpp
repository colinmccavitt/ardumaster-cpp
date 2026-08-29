#pragma once

#include <cstdint>
#include <optional>

#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_types.hpp>

namespace fwcpp::tailsitter {

struct TailsitterSetupInputs {
    std::optional<std::int8_t> enable;
    std::uint8_t frame_class{0};
    std::uint16_t motor_mask{0};
    TiltrotorType tiltrotor_type{TiltrotorType::kNone};
};

struct TailsitterSetupResult {
    std::int8_t enable{kTailsitEnableDefault};
    bool setup_complete{false};
};

[[nodiscard]] inline constexpr bool setup_heuristic_applies(const TailsitterSetupInputs& in) {
    if (in.enable.has_value()) {
        return false;
    }
    const bool frame_or_mask =
        (in.frame_class == kMotorFrameTailsitter) || (in.motor_mask != 0u);
    const bool not_bicopter = in.tiltrotor_type != TiltrotorType::kBicopter;
    return frame_or_mask && not_bicopter;
}

[[nodiscard]] inline TailsitterSetupResult resolve_setup(const TailsitterSetupInputs& in) {
    TailsitterSetupResult out{};
    out.enable = in.enable.value_or(kTailsitEnableDefault);
    if (setup_heuristic_applies(in)) {
        out.enable = 1;
    }
    if (out.enable <= 0) {
        out.setup_complete = false;
        return out;
    }
    out.setup_complete = true;
    return out;
}

}  // namespace fwcpp::tailsitter
