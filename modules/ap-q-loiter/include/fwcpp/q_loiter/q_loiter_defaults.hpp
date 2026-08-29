#pragma once

#include <cstdint>

namespace fwcpp::q_loiter {

inline constexpr std::uint32_t kLoiterTargetReinitMs = 500;
inline constexpr std::uint32_t kPreclandOverrideTimeoutMs = 250;
inline constexpr float kQrtlAltDefaultM = 15.0F;

}  // namespace fwcpp::q_loiter
