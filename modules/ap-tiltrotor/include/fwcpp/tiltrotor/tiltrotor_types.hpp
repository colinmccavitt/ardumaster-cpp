#pragma once

#include <cstdint>
#include <optional>

namespace fwcpp::tiltrotor {

enum class TiltType : std::int8_t {
    kContinuous = 0,
    kBinary = 1,
    kVectoredYaw = 2,
    kBicopter = 3,
};

struct TiltrotorSetupInputs {
    std::optional<std::int8_t> enable;
    std::uint16_t tilt_mask{0};
    TiltType type{TiltType::kContinuous};
};

}  // namespace fwcpp::tiltrotor
