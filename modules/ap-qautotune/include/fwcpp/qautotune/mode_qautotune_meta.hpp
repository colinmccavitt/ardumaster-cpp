#pragma once

#include <fwcpp/qautotune/qautotune_defaults.hpp>

#include <cstdint>

namespace fwcpp::qautotune {

struct QAutotuneModeFlags {
    bool is_vtol_mode{true};
    bool is_vtol_man_mode{true};
};

[[nodiscard]] inline constexpr QAutotuneModeFlags qautotune_mode_flags() {
    return QAutotuneModeFlags{};
}

[[nodiscard]] inline constexpr std::uint8_t qautotune_mode_number() {
    return kModeQautotuneNumber;
}

[[nodiscard]] inline constexpr const char* qautotune_name() { return "QAutotune"; }
[[nodiscard]] inline constexpr const char* qautotune_name4() { return "QATN"; }

}  // namespace fwcpp::qautotune
