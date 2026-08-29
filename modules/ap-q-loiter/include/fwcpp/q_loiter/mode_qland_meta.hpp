#pragma once

#include <cstdint>

namespace fwcpp::q_loiter {

inline constexpr std::uint8_t kModeQlandNumber = 20;

struct QLandModeFlags {
    bool is_vtol_mode{true};
    bool is_vtol_man_mode{false};
    bool does_auto_throttle{true};
};

[[nodiscard]] inline constexpr QLandModeFlags qland_mode_flags() {
    return QLandModeFlags{};
}

[[nodiscard]] inline constexpr const char* qland_name() { return "QLand"; }
[[nodiscard]] inline constexpr const char* qland_name4() { return "QLND"; }

}  // namespace fwcpp::q_loiter
