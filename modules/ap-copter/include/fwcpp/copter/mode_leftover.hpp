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
    {"Mode::Number", ModePortStatus::kOnMain,
     "mode.h enum values including gaps at 8/10/12"},
    {"ModeReason", ModePortStatus::kOnMain,
     "AP_Vehicle/ModeReason.h UNKNOWN=0 through FENCE_REENABLE=55"},
    {"Mode base virtuals", ModePortStatus::kOnMain,
     "mode_number, init, exit, run, requires_position, has_manual_throttle, "
     "allows_entry_in_rc_failsafe; takeoff_stop no-op"},
    {"mode_from_mode_num stabilize+althold", ModePortStatus::kOnMain,
     "STABILIZE, ALT_HOLD, AUTO, RTL; AUTO_RTL stays nullptr (not a true mode); "
     "LAND stays nullptr"},
    {"set_mode checks", ModePortStatus::kOnMain,
     "already-in, GCS gate, unknown, ignore_checks, throttle-too-high, "
     "position, alt, rc_failsafe, init, exit+switch"},
    {"stabilize_run", ModePortStatus::kOnMain,
     "mode_stabilize.hpp; ModeStabilize::run via input_euler_angle; CCP-039 s1"},
    {"AUTO_RTL", ModePortStatus::kOnMain,
     "set_mode special case; not a true mode, AUTO in disguise; injected jumps"},
    {"acro_run", ModePortStatus::kOnMain,
     "mode_acro.hpp; CCP-039 landed run() on main"},
    {"althold_run", ModePortStatus::kOnMain,
     "mode_althold.hpp; CCP-039 landed run() on main"},
    {"remaining mode bodies", ModePortStatus::kRemaining,
     "ModeRTL::run rest; ModeLand init/run; other modes; auto_takeoff.run body; "
     "land_run_normal_or_precland body; land_run_horizontal_control body; "
     "ModeGuided::run body"},
    {"ModeAuto::init", ModePortStatus::kOnMain,
     "mode_auto.cpp auto_init leftover; mission_present / landed takeoff gate; "
     "no wp_nav/mission objects; precland remaining"},
    {"ModeAuto::exit", ModePortStatus::kOnMain,
     "mode_auto.cpp ~71-81; mission.stop if running; auto_RTL clear; "
     "camera_mount.set_mode_to_default remaining"},
    {"ModeAuto::run", ModePortStatus::kOnMain,
     "mode_auto.cpp ~85-98; waiting_to_start + origin leftover; "
     "injected has_origin; start_or_resume / mis_change_check_init leftovers"},
    {"ModeAuto::run else-path", ModePortStatus::kOnMain,
     "mode_auto.cpp ~99-113; change detector restart + mission.update leftover; "
     "injected mission_changed / submode_is_wp / restart_nav_ok"},
    {"ModeAuto::run SubMode switch", ModePortStatus::kOnMain,
     "mode_auto.cpp ~116-164; leftover dispatch flags only; no *_run bodies; "
     "NAV_PAYLOAD_PLACE omitted; nav_guided gated by nav_guided_or_scripting"},
    {"ModeAuto::run auto_RTL landing-sequence", ModePortStatus::kOnMain,
     "mode_auto.cpp ~166-174; clear auto_RTL when not landing/return/complete; "
     "leftover Write_Mode AUTO_RTL_EXIT; logged number is AUTO after clear"},
    {"ModeAuto::takeoff_run", ModePortStatus::kOnMain,
     "mode_auto.cpp ~1075-1083; leftover set_auto_armed when "
     "allow_takeoff_without_raising_throttle; leftover auto_takeoff_run; "
     "no auto_takeoff.run body / Option enum"},
    {"ModeAuto::wp_run", ModePortStatus::kOnMain,
     "mode_auto.cpp ~1087-1107; leftover make_safe_ground_handling when "
     "disarmed_or_landed; else leftover spool/wp_nav/pos/attitude flags; "
     "no motors / wp_nav / pos_control / attitude objects"},
    {"ModeAuto::land_run", ModePortStatus::kOnMain,
     "mode_auto.cpp ~1111-1125; leftover make_safe_ground_handling when "
     "disarmed_or_landed; else leftover spool + land_run_normal_or_precland; "
     "no motors / precland / land_run_normal_or_precland body"},
    {"ModeAuto::rtl_run", ModePortStatus::kOnMain,
     "mode_auto.cpp ~1129-1133; leftover ModeRTL::run(false) flag; "
     "does not call ModeRTL::init or run"},
    {"ModeAuto::loiter_run", ModePortStatus::kOnMain,
     "mode_auto.cpp ~1162-1180; leftover same flags as wp_run; "
     "no motors / wp_nav / pos / attitude objects"},
    {"ModeAuto::circle_run", ModePortStatus::kOnMain,
     "mode_auto.cpp ~1135-1148; leftover circle_nav update_ms + pos/attitude; "
     "no circle_nav object; no spool/disarmed"},
    {"ModeAuto::loiter_to_alt_run", ModePortStatus::kOnMain,
     "leftover through climb flags; land_run_horizontal_control body remaining"},
    {"ModeAuto::nav_guided_run", ModePortStatus::kOnMain,
     "mode_auto.cpp ~1150-1158; leftover ModeGuided::run flag; "
     "no ModeGuided body"},
    {"ModeAuto::nav_attitude_time_run", ModePortStatus::kOnMain,
     "leftover through pos_D_update leftover flags"},
    {"ModeRTL::init", ModePortStatus::kOnMain,
     "home_is_set gate + leftover wp_and_spline/STARTING flags; leftover "
     "leftover_precland_statemachine remaining"},
    {"ModeRTL::run", ModePortStatus::kThisSlice,
     "armed gate + STARTING leftover leftover leftover leftover_build_path/climb_start + "
     "INITIAL_CLIMB leftover leftover leftover leftover_return_start + RETURN_HOME leftover leftover leftover leftover_loiterathome_start + LOITER_AT_HOME leftover leftover leftover leftover_land_start/descent_start + leftover leftover leftover leftover_climb_return_run + leftover leftover leftover leftover_loiterathome_run + leftover leftover leftover leftover_descent_run + leftover leftover leftover leftover_rtl_land_run; leftover leftover leftover leftover_climb_return_run leftover leftover leftover leftover_body remaining"},
    {"FLTMODE_GCSBLOCK param", ModePortStatus::kOnMain,
     "Copter::gcs_mode_enabled + AP_Vehicle::block_GCS_mode_change; injected "
     "fltmode_gcsblock; LAND/RTL not in list"},
    {"fence recovery", ModePortStatus::kOnMain,
     "DISABLE_MODE_CHANGE gate + manual_recovery_start leftover; no AC_Fence"},
    {"update_flight_mode FAST_TASK", ModePortStatus::kOnMain,
     "lives in update_flight_mode.hpp (CCP-035)"},
    {"Write_Mode/notify", ModePortStatus::kOnMain,
     "leftover flags after successful enter; GCS heartbeat/ADSB/camera/rate_tc remaining"},
    {"Drift-as-manual-throttle", ModePortStatus::kOnMain,
     "MODE_DRIFT_ENABLED forces user_throttle true; injected is_drift (no ModeDrift)"},
    {"set_accel_throttle_I", ModePortStatus::kOnMain,
     "exit_mode manual-to-auto I transfer; Attitude.cpp helper reused"},
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
