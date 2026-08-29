#pragma once

#include <cstdint>

namespace fwcpp::q_modes {

enum class QModeNumber : std::uint8_t {
    kQstabilize = 17,
    kQhover = 18,
    kQacro = 23,
};

struct QModeFlags {
    bool is_vtol_mode{true};
    bool is_vtol_man_throttle{true};
    bool is_vtol_man_mode{true};
    bool allows_throttle_nudging{true};
};

[[nodiscard]] inline constexpr QModeFlags qstabilize_flags() { return QModeFlags{}; }
[[nodiscard]] inline constexpr QModeFlags qhover_flags() { return qstabilize_flags(); }

[[nodiscard]] inline constexpr QModeFlags qacro_flags() {
    QModeFlags f = qstabilize_flags();
    f.is_vtol_man_mode = false;
    return f;
}

[[nodiscard]] inline constexpr const char* qmode_name(QModeNumber mode) {
    switch (mode) {
        case QModeNumber::kQstabilize:
            return "QStabilize";
        case QModeNumber::kQhover:
            return "QHOVER";
        case QModeNumber::kQacro:
            return "QACRO";
    }
    return "";
}

[[nodiscard]] inline constexpr const char* qmode_name4(QModeNumber mode) {
    switch (mode) {
        case QModeNumber::kQstabilize:
            return "QSTB";
        case QModeNumber::kQhover:
            return "QHOV";
        case QModeNumber::kQacro:
            return "QACR";
    }
    return "";
}

}  // namespace fwcpp::q_modes
