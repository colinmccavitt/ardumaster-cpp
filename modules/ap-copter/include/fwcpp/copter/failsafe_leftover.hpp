#pragma once

// CCP-042 leftover completeness catalog — ArduCopter failsafe / events /
// crash_check (Plane-4.7.0). Nested under fwcpp::copter::failsafe so
// remaining_count() does not collide with copter_leftover / land_detector.
//
// Slice 1: leftover catalog + leftover_failsafe_radio_check thin gate.
// Slice 2: leftover_failsafe_radio_on_event FS_THR → FailsafeAction map.
// Slice 3: leftover_do_failsafe_action FailsafeAction → set_mode_* flags
// (wired from leftover_failsafe_radio_on_event).
// Slice 4 (close): leftover_failsafe_gcs_check thin gate; override ladder,
// battery/terrain/deadreckon, CPU watchdog, crash_check, ModeBrake body →
// kOutOfScope (secondary FS / CCP-041 overlap). Radio path + do_failsafe_action
// already this ticket. remaining_count()==0.

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
    {"leftover catalog", PortStatus::kThisSlice, "this table; CCP-042 close"},
    {"leftover_failsafe_radio_check", PortStatus::kThisSlice,
     "armed && radio_failsafe inject → leftover_set_mode_rtl_or_land + GCS flags"},
    {"leftover_set_mode_rtl_or_land", PortStatus::kThisSlice,
     "events.cpp ~389 thin flags; no set_mode / land-with-pause body"},
    {"leftover_failsafe_radio_on_event", PortStatus::kThisSlice,
     "events.cpp ~17-44; FsThrEnable → FailsafeAction; wires leftover_do_failsafe_action"},
    {"leftover_do_failsafe_action", PortStatus::kThisSlice,
     "events.cpp ~485; FailsafeAction → set_mode_* / terminate flags; no real set_mode"},
    {"leftover_failsafe_gcs_check", PortStatus::kThisSlice,
     "armed && gcs_failsafe inject → leftover_do_failsafe_action(RTL); no heartbeat age"},
    {"failsafe_enable call site", PortStatus::kOnMain,
     "CCP-035 init_ardupilot leftover failsafe_enable flag"},
    {"ModeRTL / ModeLand", PortStatus::kOnMain,
     "CCP-036 mode leftovers; land_detector CCP-041 scaffold"},
    {"failsafe_radio_on_event override ladder", PortStatus::kOutOfScope,
     "events.cpp ~46-75; should_disarm + continue-landing/auto/guided; secondary FS OOS"},
    {"battery / terrain / deadreckon failsafe", PortStatus::kOutOfScope,
     "events.cpp handle_battery / terrain / deadreckon; secondary FS sources OOS"},
    {"failsafe.cpp CPU watchdog", PortStatus::kOutOfScope,
     "failsafe_enable/disable/check; 2s lockup → output_min / disarm; secondary FS OOS"},
    {"crash_check / thrust_loss / yaw_imbalance", PortStatus::kOutOfScope,
     "crash_check.cpp; secondary FS / CCP-041 land_detector overlap OOS"},
    {"ModeBrake failsafe path", PortStatus::kOutOfScope,
     "mode_brake.cpp BRAKE_OR_LAND body; action flag via do_failsafe_action; body OOS"},
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
