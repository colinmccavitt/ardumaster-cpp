#pragma once

#include <cstddef>
#include <cstdint>

namespace fwcpp::q_loiter {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct QLoiterPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr QLoiterPortItem kQLoiterCompleteness[] = {
    {"QLOITER mode Number 19", PortStatus::kThisSlice, "mode_qloiter_meta.hpp"},
    {"QLAND mode Number 20", PortStatus::kThisSlice, "mode_qland_meta.hpp"},
    {"LOITER_ALT_QLAND Number 25", PortStatus::kThisSlice, "mode_loiter_alt_qland_meta.hpp"},
    {"QLOITER _enter loiter init", PortStatus::kThisSlice, "mode_qloiter_enter.hpp"},
    {"QLAND _enter poscontrol descend", PortStatus::kThisSlice, "mode_qland_enter.hpp"},
    {"LoiterAltQLand VTOL fast path", PortStatus::kThisSlice, "loiter_alt_qland.hpp enter"},
    {"LoiterAltQLand switch_qland", PortStatus::kThisSlice, "loiter_alt_qland.hpp switch"},
    {"QLOITER run phase gate", PortStatus::kThisSlice, "mode_qloiter_run.hpp"},
    {"QLAND run delegates QLOITER", PortStatus::kThisSlice, "mode_qland_run.hpp"},
    {"QLOITER QLAND vertical branch", PortStatus::kThisSlice, "mode_qloiter_run.hpp"},
    {"completeness catalog", PortStatus::kThisSlice, "this table"},
    {"QLOITER update delegate QStabilize", PortStatus::kRemaining, "mode_qloiter.cpp update"},
    {"QLOITER run loiter_nav body", PortStatus::kRemaining, "AC_WPNav loiter wiring"},
    {"QLOITER precland overrides", PortStatus::kRemaining, "AC_PRECLAND 250ms path"},
    {"QLOITER systemid att offset", PortStatus::kRemaining, "AP_PLANE_SYSTEMID_ENABLED"},
    {"LoiterAltQLand navigate hook", PortStatus::kRemaining, "ModeLoiter navigate"},
    {"LoiterAltQLand handle_guided WP", PortStatus::kRemaining, "set_guided_WP caller"},
    {"QLAND landing gear IC engine cut", PortStatus::kRemaining, "optional compile flags"},
    {"Plane / QuadPlane refs", PortStatus::kOutOfScope, "ADR-0012 caller applies"},
};

[[nodiscard]] inline constexpr std::size_t q_loiter_completeness_size() {
    return sizeof(kQLoiterCompleteness) / sizeof(kQLoiterCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kQLoiterCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kQLoiterCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}

}  // namespace fwcpp::q_loiter
