#pragma once

// CCP-038 leftover completeness catalog — ArduCopter AP_Arming_Copter.
// Nested under fwcpp::copter::arming so remaining_count() does not collide
// with copter_leftover.hpp / mode_leftover.hpp in fwcpp::copter.
//
// Slice 2: interlock/E-Stop conflict + motor interlock enabled on top of
// slice 1 already-armed / system_initialized. disarm_switch / motors /
// gps / baro / arm / disarm bodies remain. ADR-0012: no AP:: singletons,
// no GCS, no exceptions.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter::arming {

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
    {"pre_arm_checks", PortStatus::kThisSlice,
     "AP_Arming_Copter.cpp ~8-13; run_pre_arm then set_pre_arm_check(passed)"},
    {"run_pre_arm_checks already_armed gate", PortStatus::kThisSlice,
     "AP_Arming_Copter.cpp ~19-22; motors_armed short-circuit return true"},
    {"system_initialized check", PortStatus::kThisSlice,
     "AP_Arming_Copter.cpp ~24-27; inject system_initialized; check_failed flag"},
    {"interlock/estop conflict", PortStatus::kThisSlice,
     "AP_Arming_Copter.cpp ~29-37; MOTOR_INTERLOCK vs MOTOR_ESTOP/ARM_EMERGENCY_STOP"},
    {"motor interlock enabled", PortStatus::kThisSlice,
     "AP_Arming_Copter.cpp ~42-45; using_interlock && motor_interlock_switch"},
    {"disarm_switch_checks", PortStatus::kRemaining,
     "AP_Arming_Copter.cpp ~47-49; AP_Arming::disarm_switch_checks"},
    {"motors->arming_checks", PortStatus::kRemaining,
     "AP_Arming_Copter.cpp ~51-56; motors failure_msg path"},
    {"parameter_checks / gps / baro / board_voltage / alt / rc_throttle_failsafe",
     PortStatus::kRemaining,
     "AP_Arming_Copter.cpp ~77-86 and check helpers; skip_all / mandatory remaining"},
    {"arm() / disarm()", PortStatus::kRemaining,
     "AP_Arming_Copter.h arm/disarm overrides; bodies not this slice"},
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

}  // namespace fwcpp::copter::arming
