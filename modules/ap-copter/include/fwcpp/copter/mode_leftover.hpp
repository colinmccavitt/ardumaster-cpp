#pragma once

// CCP-036 leftover completeness catalog — ArduCopter Mode base and set_mode.
// Separate from copter_leftover.hpp (CCP-035 vehicle loop). remaining_count()
// is the kRemaining count and is intentionally > 0 after this slice.
//
// ADR-0012: no AP:: singletons, no exceptions, no flight-path alloc.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter {

enum class ModePortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct ModePortItem {
    const char* name;
    ModePortStatus status;
    const char* note;
};

inline constexpr ModePortItem kModeCompleteness[] = {
    {"leftover catalog", ModePortStatus::kThisSlice, "this table"},
    {"Mode::Number", ModePortStatus::kThisSlice,
     "mode.h enum values including gaps at 8/10/12"},
    {"ModeReason", ModePortStatus::kThisSlice,
     "AP_Vehicle/ModeReason.h UNKNOWN=0 through FENCE_REENABLE=55"},
    {"Mode base virtuals", ModePortStatus::kThisSlice,
     "mode_number, init, exit, run, requires_position, has_manual_throttle, "
     "allows_entry_in_rc_failsafe; takeoff_stop no-op"},
    {"mode_from_mode_num stabilize+althold", ModePortStatus::kThisSlice,
     "STABILIZE and ALT_HOLD only; other Number values nullptr"},
    {"set_mode checks", ModePortStatus::kThisSlice,
     "already-in, GCS gate, unknown, ignore_checks, throttle-too-high, "
     "position, alt, rc_failsafe, init, exit+switch"},
    {"stabilize_run", ModePortStatus::kThisSlice,
     "mode_stabilize.hpp; ModeStabilize::run via input_euler_angle; CCP-039 s1"},
    {"AUTO_RTL", ModePortStatus::kRemaining,
     "set_mode special case; not a true mode, AUTO in disguise"},
    {"acro_run", ModePortStatus::kRemaining,
     "mode_acro.cpp; needs get_pilot_desired_rates; CCP-039 later slice"},
    {"althold_run", ModePortStatus::kRemaining,
     "mode_althold.cpp; needs climb_rate leftover; CCP-039 later slice"},
    {"remaining mode bodies", ModePortStatus::kRemaining,
     "AUTO/RTL/LAND/... run/init; ACRO/ALTHOLD catalogued separately"},
    {"FLTMODE_GCSBLOCK param", ModePortStatus::kRemaining,
     "gcs_mode_enabled is injected; param bitmask lookup stays remaining"},
    {"fence recovery", ModePortStatus::kRemaining,
     "AP_FENCE_ENABLED DISABLE_MODE_CHANGE while breached"},
    {"update_flight_mode FAST_TASK", ModePortStatus::kRemaining,
     "mode.cpp ~497-508; CCP-035 leftover, not this slice"},
    {"Write_Mode/notify", ModePortStatus::kRemaining,
     "logger.Write_Mode, notify_flight_mode, AP_Notify user_mode_change"},
    {"Drift-as-manual-throttle", ModePortStatus::kRemaining,
     "MODE_DRIFT_ENABLED forces user_throttle true in throttle-too-high"},
    {"set_accel_throttle_I", ModePortStatus::kRemaining,
     "exit_mode manual-to-auto I transfer; CCP-037"},
    {"HELI runup/flybar", ModePortStatus::kOutOfScope,
     "rotor_runup_complete, flybar passthrough, collective ramp"},
    {"AP:: singletons", ModePortStatus::kOutOfScope, "ADR-0012 explicit context"},
    {"AP_Notify sounds", ModePortStatus::kOutOfScope,
     "user_mode_change happy noise; already-in INITIALISED STABILIZE yaw_rate_tc"},
};

[[nodiscard]] inline constexpr std::size_t mode_completeness_size() {
    return sizeof(kModeCompleteness) / sizeof(kModeCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t mode_count_status(ModePortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kModeCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool mode_completeness_has(const char* name, ModePortStatus status) {
    for (const auto& item : kModeCompleteness) {
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

[[nodiscard]] inline constexpr std::size_t mode_on_main_count() {
    return mode_count_status(ModePortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t mode_this_slice_count() {
    return mode_count_status(ModePortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return mode_count_status(ModePortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t mode_out_of_scope_count() {
    return mode_count_status(ModePortStatus::kOutOfScope);
}

}  // namespace fwcpp::copter
