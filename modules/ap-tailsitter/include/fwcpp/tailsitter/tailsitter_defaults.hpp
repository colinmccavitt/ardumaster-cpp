#pragma once

#include <cstdint>

namespace fwcpp::tailsitter {

inline constexpr std::uint8_t kMotorFrameTailsitter = 10;

inline constexpr std::int8_t kTailsitEnableDefault = 0;
inline constexpr float kVectoredHoverGainDefault = 0.5f;
inline constexpr std::int8_t kTailsitInputDefault = 0;
inline constexpr std::uint16_t kTailsitMotmxDefault = 0;
inline constexpr std::int8_t kTransitionAngleFwDefault = 45;
inline constexpr std::int8_t kTransitionAngleVtolDefault = 0;

inline constexpr std::uint8_t kTailsitterInputPlane = 1u << 0;
inline constexpr std::uint8_t kTailsitterInputBfRoll = 1u << 1;

inline constexpr std::uint16_t kTailsitterGsclThrottle = 1u << 0;
inline constexpr std::uint16_t kTailsitterGsclAttThr = 1u << 1;
inline constexpr std::uint16_t kTailsitterGsclDiskTheory = 1u << 2;
inline constexpr std::uint16_t kTailsitterGsclAltitude = 1u << 3;

}  // namespace fwcpp::tailsitter
