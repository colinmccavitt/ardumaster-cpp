#pragma once

#include <cstdint>

namespace fwcpp::q_loiter {

inline constexpr std::uint8_t kModeLoiterAltQlandNumber = 25;

[[nodiscard]] inline constexpr const char* loiter_alt_qland_name() { return "Loiter to QLand"; }
[[nodiscard]] inline constexpr const char* loiter_alt_qland_name4() { return "L2QL"; }

}  // namespace fwcpp::q_loiter
