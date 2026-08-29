#pragma once

#include <cstdint>

#include <fwcpp/tiltrotor/tiltrotor_types.hpp>

namespace fwcpp::tiltrotor {

inline constexpr std::int8_t kTiltEnableDefault = 0;
inline constexpr std::uint16_t kTiltMaskDefault = 0;
inline constexpr std::int16_t kTiltRateUpDefaultDps = 40;
inline constexpr std::int16_t kTiltRateDownDefaultDps = 0;
inline constexpr std::int8_t kTiltMaxAngleDefaultDeg = 45;
inline constexpr TiltType kTiltTypeDefault = TiltType::kContinuous;
inline constexpr float kTiltYawAngleDefault = 0.0f;
inline constexpr float kTiltFixAngleDefault = 0.0f;
inline constexpr float kTiltFixGainDefault = 0.0f;
inline constexpr float kTiltFlapAngleDefaultDeg = 0.0f;

inline constexpr float kDegreesPerTiltUnit = 1.0f / 90.0f;

inline constexpr float kFastTiltMinRateDps = 90.0f;
inline constexpr float kServoMotorTiltScale = 1000.0f;
inline constexpr float kThrottleScaledToUnit = 0.01f;
inline constexpr std::uint32_t kTiltDelayMs = 3000;
inline constexpr std::int32_t kQOptionDisarmedTilt = 1 << 10;
inline constexpr float kTiltElevonScale = 4500.0f;
inline constexpr float kServoMax = 4500.0f;
inline constexpr std::uint8_t kMaxNumMotors = 32;
inline constexpr float kVectoringThrottleScaleMin = 0.5f;
inline constexpr float kVectoringThrottleScaleMax = 2.0f;
inline constexpr float kVectoringAvgRollFactor = 0.5f;
inline constexpr std::uint32_t kTransitionYawLockMs = 100u;
inline constexpr float kGravityMss = 9.80665f;
inline constexpr std::int32_t kNavRollTransitionThresholdCd = 1000;
inline constexpr float kAirspeedMinTransitionMs = 5.0f;

}  // namespace fwcpp::tiltrotor
