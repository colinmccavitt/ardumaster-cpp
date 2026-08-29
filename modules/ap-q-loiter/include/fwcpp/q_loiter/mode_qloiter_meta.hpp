#pragma once

#include <cstdint>

namespace fwcpp::q_loiter {

inline constexpr std::uint8_t kModeQloiterNumber = 19;

struct QLoiterModeFlags {
    bool is_vtol_mode{true};
    bool is_vtol_man_mode{true};
    bool does_auto_throttle{true};
    bool supports_vtol_systemid{true};
};

[[nodiscard]] inline constexpr QLoiterModeFlags qloiter_mode_flags() {
    return QLoiterModeFlags{};
}

[[nodiscard]] inline constexpr const char* qloiter_name() { return "QLoiter"; }
[[nodiscard]] inline constexpr const char* qloiter_name4() { return "QLOT"; }

}  // namespace fwcpp::q_loiter
