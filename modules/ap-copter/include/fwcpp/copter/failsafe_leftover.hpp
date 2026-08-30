#pragma once

// CCP-042 leftover completeness catalog — ArduCopter failsafe / events /
// crash_check (Plane-4.7.0). Nested under fwcpp::copter::failsafe so
// remaining_count() does not collide with copter_leftover / land_detector.
//
// Slice 1: leftover catalog + leftover_failsafe_radio_check thin gate
// (armed && radio_failsafe inject → leftover_set_mode_rtl_or_land flags).
// GCS announce as flags (ADR-0012). events override ladder, GCS failsafe,
// crash_check, and failsafe.cpp CPU watchdog remain.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter::failsafe {

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
    {"leftover_failsafe_radio_check", PortStatus::kThisSlice,
     "armed && radio_failsafe inject → leftover_set_mode_rtl_or_land + GCS flags"},
    {"leftover_set_mode_rtl_or_land", PortStatus::kThisSlice,
     "events.cpp ~389 thin flags; no set_mode / land-with-pause body"},
    {"failsafe_enable call site", PortStatus::kOnMain,
     "CCP-035 init_ardupilot leftover failsafe_enable flag"},
    {"ModeRTL / ModeLand", PortStatus::kOnMain,
     "CCP-036 mode leftovers; land_detector CCP-041 scaffold"},
    {"failsafe_radio_on_event override ladder", PortStatus::kRemaining,
     "events.cpp ~13-79; FS_THR_ENABLE + continue-landing/auto/guided"},
    {"failsafe_gcs_check / failsafe_gcs_on_event", PortStatus::kRemaining,
     "events.cpp ~125+; heartbeat age edge + FS_GCS_ENABLE table"},
    {"do_failsafe_action / battery / terrain / deadreckon", PortStatus::kRemaining,
     "events.cpp action dispatcher + other failsafe sources"},
    {"failsafe.cpp CPU watchdog", PortStatus::kRemaining,
     "failsafe_enable/disable/check; 2s lockup → output_min / disarm"},
    {"crash_check / thrust_loss / yaw_imbalance", PortStatus::kRemaining,
     "crash_check.cpp; events/crash_check remaining"},
    {"ModeBrake failsafe path", PortStatus::kRemaining,
     "mode_brake.cpp; BRAKE_OR_LAND action"},
    {"GCS / Notify / logger objects", PortStatus::kOutOfScope,
     "ADR-0012; announce_failsafe + notify as bool flags"},
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

}  // namespace fwcpp::copter::failsafe
