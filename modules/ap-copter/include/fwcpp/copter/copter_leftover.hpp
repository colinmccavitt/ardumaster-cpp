#pragma once

// CCP-035 leftover completeness catalog — ArduCopter vehicle loop
// (Copter.cpp / Copter.h / system.cpp). Slice 8 lands
// update_land_and_crash_detectors as a leftover tick. remaining_count() > 0
// is expected after this slice.
//
// ADR-0012: no AP:: singletons, no AP_Param var_info, no exceptions.
// Subsystem objects are injected as inputs on later leftover ticks.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct CopterPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr CopterPortItem kCopterCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"Copter::scheduler_tasks[]", PortStatus::kOnMain,
     "scheduler_tasks.hpp; gated rows stay with gate string"},
    {"Copter::get_scheduler_tasks", PortStatus::kOnMain, "MASK_LOG_PM view"},
    {"Copter::rc_loop", PortStatus::kOnMain,
     "rc_loop.hpp; always read_radio then read_mode_switch"},
    {"RC_Channels::read_mode_switch", PortStatus::kOnMain,
     "NoValidInput / NoChannel / Read; inject has_valid_input + channel"},
    {"Copter::motors_output / motors_output_main", PortStatus::kOnMain,
     "motors_output.hpp; AFS skip, arming delay, interlock, drive, push"},
    {"Copter::read_AHRS", PortStatus::kOnMain, "read_ahrs.hpp; skip_ins_update"},
    {"Copter::throttle_loop", PortStatus::kOnMain,
     "throttle_loop.hpp; always mix, auto_armed, gnd-effect, ekf-terrain; no heli"},
    {"Copter::init_ardupilot", PortStatus::kRemaining, "system.cpp init"},
    {"Copter::run_rate_controller_main", PortStatus::kOnMain,
     "run_rate_controller.hpp; set_dt_s + rate_controller_run iff !rate thread"},
    {"Copter::read_inertia", PortStatus::kOnMain,
     "read_inertia.hpp; pos_control estimates, lat/lng, alt-above-home"},
    {"Copter::check_ekf_reset", PortStatus::kOnMain,
     "check_ekf_reset.hpp; yaw-reset + primary-core leftover flags"},
    {"Copter::update_flight_mode", PortStatus::kOnMain,
     "update_flight_mode.hpp; surface-tracking + landed-gain + EKF method + run"},
    {"Copter::update_home_from_EKF", PortStatus::kOnMain,
     "update_home_from_ekf.hpp; inflight copy_alt_from; SmartRTL leftover"},
    {"Copter::update_land_and_crash_detectors", PortStatus::kThisSlice,
     "dispatcher + disarmed/landed; crash_check/AND-gate remaining"},
    {"Copter::update_rangefinder_terrain_offset", PortStatus::kRemaining, "FAST_TASK body"},
    {"Copter::update_batt_compass", PortStatus::kRemaining, "10 Hz leftover"},
    {"Copter::update_altitude", PortStatus::kRemaining, "10 Hz leftover"},
    {"Copter::run_nav_updates", PortStatus::kRemaining, "50 Hz leftover"},
    {"Copter::update_throttle_hover", PortStatus::kRemaining, "100 Hz leftover"},
    {"Copter::three_hz_loop", PortStatus::kRemaining, "3 Hz leftover"},
    {"Copter::loop_rate_logging", PortStatus::kRemaining, "HAL_LOGGING_ENABLED leftover"},
    {"Copter::ten_hz_logging_loop", PortStatus::kRemaining, "HAL_LOGGING_ENABLED leftover"},
    {"Copter::twentyfive_hz_logging", PortStatus::kRemaining, "HAL_LOGGING_ENABLED leftover"},
    {"Copter::one_hz_loop", PortStatus::kRemaining, "1 Hz leftover"},
    {"Copter::ap_value", PortStatus::kRemaining, "packed ap bools"},
    {"Copter::init_simple_bearing", PortStatus::kRemaining, "simple-mode leftover"},
    {"Copter::update_simple_mode", PortStatus::kRemaining, "simple-mode leftover"},
    {"Copter::update_super_simple_bearing", PortStatus::kRemaining, "simple-mode leftover"},
    {"Copter::auto_disarm_check", PortStatus::kRemaining, "motors.cpp leftover"},
    {"Copter::standby_update", PortStatus::kRemaining, "standby.cpp leftover"},
    {"Copter::lost_vehicle_check", PortStatus::kRemaining, "motors.cpp leftover"},
    {"Copter::takeoff_check", PortStatus::kRemaining, "takeoff leftover"},
    {"Copter::get_wp_distance_m", PortStatus::kRemaining, "GCS helper leftover"},
    {"Copter::update_auto_armed", PortStatus::kRemaining, "system.cpp leftover"},
    {"Copter::startup_INS_ground", PortStatus::kRemaining, "system.cpp leftover"},
    {"Copter::allocate_motors", PortStatus::kRemaining, "system.cpp leftover"},
    {"AP:: singletons", PortStatus::kOutOfScope, "ADR-0012 explicit Copter context"},
    {"AP_Param var_info", PortStatus::kOutOfScope, "inject params via setters"},
    {"scripting / external control", PortStatus::kOutOfScope,
     "set_target_* / register_custom_mode; no scripting in this port"},
    {"GCS / MAVLink / logger objects", PortStatus::kOutOfScope, "no GCS in this port"},
};

[[nodiscard]] inline constexpr std::size_t copter_completeness_size() {
    return sizeof(kCopterCompleteness) / sizeof(kCopterCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kCopterCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kCopterCompleteness) {
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

}  // namespace fwcpp::copter
