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

}  // namespace fwcpp::tiltrotor
