#pragma once

#include <cstdint>

namespace fwcpp::qrtl {

inline constexpr std::uint8_t kModeQrtlNumber = 21;

struct QrtlModeFlags {
    bool is_vtol_mode{true};
    bool is_vtol_man_mode{false};
    bool does_auto_throttle{true};
    bool pre_arm_checks{false};
};

[[nodiscard]] inline constexpr QrtlModeFlags qrtl_mode_flags() {
    return QrtlModeFlags{};
}

[[nodiscard]] inline constexpr const char* qrtl_name() { return "QRTL"; }
[[nodiscard]] inline constexpr const char* qrtl_name4() { return "QRTL"; }

}  // namespace fwcpp::qrtl
