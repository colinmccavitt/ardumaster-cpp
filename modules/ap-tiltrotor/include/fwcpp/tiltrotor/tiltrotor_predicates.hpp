#pragma once

#include <cstdint>

#include <fwcpp/tiltrotor/tiltrotor_defaults.hpp>
#include <fwcpp/tiltrotor/tiltrotor_enable.hpp>
#include <fwcpp/tiltrotor/tiltrotor_types.hpp>

namespace fwcpp::tiltrotor {

[[nodiscard]] inline constexpr bool is_motor_tilting(std::uint16_t tilt_mask, std::uint8_t motor) {
    return (tilt_mask & (1u << motor)) != 0u;
}

[[nodiscard]] inline constexpr bool is_vectored_type(TiltType type, std::uint16_t tilt_mask) {
    return tilt_mask != 0u && type == TiltType::kVectoredYaw;
}

[[nodiscard]] inline constexpr bool tilt_angle_achieved(const TiltrotorGate& gate, TiltType type,
                                                         bool angle_achieved) {
    return !gate.enabled() || (type != TiltType::kContinuous) || angle_achieved;
}

[[nodiscard]] inline constexpr float get_fully_forward_tilt(float flap_angle_deg) {
    return 1.0f - (flap_angle_deg * kDegreesPerTiltUnit);
}

[[nodiscard]] inline constexpr bool is_continuous_type(TiltType type) {
    return type == TiltType::kContinuous;
}

}  // namespace fwcpp::tiltrotor
