#pragma once

#include <cstdint>
#include <optional>

#include <fwcpp/tiltrotor/tiltrotor_defaults.hpp>
#include <fwcpp/tiltrotor/tiltrotor_types.hpp>

namespace fwcpp::tiltrotor {

struct TiltrotorSetupResult {
    std::int8_t enable{kTiltEnableDefault};
    bool setup_complete{false};
    bool is_vectored{false};
};

[[nodiscard]] inline constexpr bool setup_heuristic_applies(const TiltrotorSetupInputs& in) {
    if (in.enable.has_value()) {
        return false;
    }
    return (in.tilt_mask != 0u) || (in.type == TiltType::kBicopter);
}

[[nodiscard]] inline TiltrotorSetupResult resolve_setup(const TiltrotorSetupInputs& in) {
    TiltrotorSetupResult out{};
    out.enable = in.enable.value_or(kTiltEnableDefault);
    if (setup_heuristic_applies(in)) {
        out.enable = 1;
    }
    if (out.enable <= 0) {
        out.setup_complete = false;
        return out;
    }
    out.is_vectored = (in.tilt_mask != 0u) && (in.type == TiltType::kVectoredYaw);
    out.setup_complete = true;
    return out;
}

}  // namespace fwcpp::tiltrotor
