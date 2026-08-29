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


inline constexpr float kTransitionRateFwDefault = 50.0f;
inline constexpr float kTransitionRateVtolDefault = 50.0f;
inline constexpr float kTransitionThrottleVtolDefault = -1.0f;
inline constexpr float kTransitionTimeoutScale = 1500.0f;
inline constexpr std::int32_t kPitchCdLimit = 8500;
inline constexpr std::uint32_t kLastVtolModeMs = 1000;
inline constexpr std::int32_t kRollErrorFloorCd = 4500;
inline constexpr std::int32_t kRollErrorMarginCd = 500;
inline constexpr std::int32_t kInvertedRollCd = 18000;
inline constexpr float kVtolZeroThrottle = 0.05f;
inline constexpr float kVtolZeroGroundspeedMs = 1.0f;

inline constexpr float kVectoredForwardGainDefault = 0.0f;
inline constexpr float kVectoredHoverPowerDefault = 2.5f;
inline constexpr float kThrottleScaleMaxDefault = 2.0f;
inline constexpr float kGainScalingMinDefault = 0.4f;
inline constexpr float kVtolRollScaleDefault = 1.0f;
inline constexpr float kVtolPitchScaleDefault = 1.0f;
inline constexpr float kVtolYawScaleDefault = 1.0f;

}  // namespace fwcpp::tailsitter
