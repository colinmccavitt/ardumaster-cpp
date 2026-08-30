#pragma once

// CCP-040 leftover completeness catalog — ArduCopter ModeLoiter / ModePosHold /
// ModeDrift. Nested under fwcpp::copter::loiter so remaining_count() does not
// collide with mode_leftover.hpp / althold / arming leftovers.
//
// Slice 1: ModeLoiter::init leftover scaffold on this slice; run / POSHOLD /
// DRIFT remaining. Fence/avoidance ticket-OOS. precision_loiter OOS
// (AC_PRECLAND). ADR-0012: no AP:: singletons.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter::loiter {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct PortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr PortItem kCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"ModeLoiter::init", PortStatus::kThisSlice,
     "mode_loiter.cpp ~10-38; leftover_init flags; no loiter_nav / pos_control"},
    {"ModeLoiter::run", PortStatus::kRemaining,
     "mode_loiter.cpp ~80-188; leftover_run_called only; spool/wp/pos/attitude remaining"},
    {"ModePosHold", PortStatus::kRemaining, "mode_poshold.cpp; not started"},
    {"ModeDrift", PortStatus::kRemaining, "mode_drift.cpp; not started"},
    {"precision_loiter", PortStatus::kOutOfScope,
     "mode_loiter.cpp do_precision_loiter / precision_loiter_xy; AC_PRECLAND"},
    {"fence / avoidance", PortStatus::kOutOfScope, "ticket OOS; no AC_Fence / AC_Avoid"},
    {"AP:: singletons", PortStatus::kOutOfScope, "ADR-0012 explicit context"},
};

[[nodiscard]] inline constexpr std::size_t completeness_size() {
    return sizeof(kCompleteness) / sizeof(kCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kCompleteness) {
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

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

}  // namespace fwcpp::copter::loiter
