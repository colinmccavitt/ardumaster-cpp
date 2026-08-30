#pragma once

// CCP-041 land_detector leftover scaffold — ArduCopter/land_detector.cpp
// (Plane-4.7.0). Thin leftover for update_land_and_crash_detectors (~16-33)
// and update_land_detector (~37+), including the multirotor stationary
// AND-gate mid-body (~92-155). No AHRS / motors / LPF / parachute /
// crash objects (ADR-0012): inject motors_armed, throttle/descent/
// accel/angle/rangefinder/WoW checks, land_complete.
//
// Slice 5 (close): stationary AND-gate leftover on this slice
// (motors_throttle_low && mix_min && !large_angle_* && accel_stationary
// && descent_rate_low && rangefinder && WoW → land_complete). crash_check
// / thrust_loss / yaw_imbalance bodies → CCP-042 OOS. land_run_normal →
// OOS shared helper. ModeRTL / ModeLand already CCP-036 on main.
// set_land_complete disarm/stats/logger side effects OOS (thin change-
// detect lives in update_land_and_crash_detectors.hpp CCP-035).
//
// Separate from update_land_and_crash_detectors.hpp (CCP-035 vehicle-loop
// leftover). Nested catalog under fwcpp::copter::land_detector so
// remaining_count() does not collide with copter_leftover / mode_leftover.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter {

// Injected inputs for land_detector leftovers. accel_ef_z_plus_g is the
// earth-frame z after GRAVITY_MSS has been applied (or the pre-sum inject
// the caller chooses); filter apply is a flag only this slice.
struct LandDetectorInputs {
    bool motors_armed{false};
    bool throttle_zero{false};
    bool land_complete{false};
    float accel_ef_z_plus_g{0.0f};
    // Armed-path injects (descent/throttle + stationary AND-gate).
    bool descent_rate_low{false};
    bool throttle_at_lower_limit{false};
    // Stationary AND-gate mid-body injects (land_detector.cpp ~92-155).
    bool motors_throttle_low{false};   // motors->limit.throttle_lower
    bool throttle_mix_min{false};      // attitude_control->is_throttle_mix_min
    bool accel_stationary{false};
    bool large_angle_request{false};
    bool large_angle_error{false};
    bool rangefinder_check{true};      // !rf_ok || alt < LAND_RANGEFINDER_MIN
    bool wow_check{true};              // WoW or unknown / no gear
};

struct LandDetectorEffects {
    bool land_accel_filter_applied{false};
    bool update_land_detector_ran{false};
    bool crash_check_ran{false};
    bool thrust_loss_check_ran{false};
    bool yaw_imbalance_check_ran{false};
    bool land_complete_set{false};
    bool land_complete{false};
    // Armed path: injects consulted.
    bool descent_check_inject{false};
    bool throttle_check_inject{false};
    bool stationary_and_gate{false};
    float accel_ef_z_plus_g{0.0f};
};

// Leftover Copter::update_land_detector (land_detector.cpp ~37+). Thin:
// !armed → land_complete; else if already complete leave as-is; else
// stationary AND-gate injects → land_complete when all true.
inline void leftover_update_land_detector(const LandDetectorInputs& in,
                                          LandDetectorEffects& fx) {
    fx.land_complete = in.land_complete;
    fx.land_complete_set = false;
    fx.stationary_and_gate = false;
    if (!in.motors_armed) {
        fx.land_complete = true;
        fx.land_complete_set = true;
        return;
    }
    fx.descent_check_inject = in.descent_rate_low;
    fx.throttle_check_inject = in.throttle_zero || in.throttle_at_lower_limit ||
                               in.motors_throttle_low;
    if (in.land_complete) {
        // Upstream ~57-69 clear-on-high-throttle path not ported here
        // (taking-off / spool injects live on CCP-035 update_land_detector).
        return;
    }
    // Multirotor stationary AND-gate (~145-151). Count threshold skipped:
    // all injects true → land_complete (thin leftover).
    const bool gate = in.motors_throttle_low && in.throttle_mix_min &&
                      !in.large_angle_request && !in.large_angle_error &&
                      in.accel_stationary && in.descent_rate_low &&
                      in.rangefinder_check && in.wow_check;
    fx.stationary_and_gate = gate;
    if (gate) {
        fx.land_complete = true;
        fx.land_complete_set = true;
    }
}

// Leftover Copter::update_land_and_crash_detectors (~16-33). Filter apply
// + always call leftover_update_land_detector. crash/thrust/yaw flags stay
// false (bodies CCP-042 OOS). HAL_PARACHUTE_ENABLED parachute_check skipped.
inline void leftover_update_land_and_crash_detectors(const LandDetectorInputs& in,
                                                     LandDetectorEffects& fx) {
    fx.accel_ef_z_plus_g = in.accel_ef_z_plus_g;
    fx.land_accel_filter_applied = true;
    leftover_update_land_detector(in, fx);
    fx.update_land_detector_ran = true;
    fx.crash_check_ran = false;
    fx.thrust_loss_check_ran = false;
    fx.yaw_imbalance_check_ran = false;
}

}  // namespace fwcpp::copter

namespace fwcpp::copter::land_detector {

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
    {"leftover_update_land_and_crash_detectors", PortStatus::kThisSlice,
     "land_detector.cpp ~16-33; filter apply + update_land_detector; "
     "crash/thrust/yaw flags false"},
    {"leftover_update_land_detector", PortStatus::kThisSlice,
     "land_detector.cpp ~37+; !armed→land_complete; else AND-gate injects"},
    {"ModeRTL", PortStatus::kOnMain,
     "CCP-036; mode.hpp ModeRTL::init/run leftovers on main"},
    {"ModeLand", PortStatus::kOnMain,
     "CCP-036; mode.hpp ModeLand::init/run leftovers on main"},
    {"land_run_normal body", PortStatus::kOutOfScope,
     "land_run_normal_or_precland / land_run_horizontal_control shared "
     "helper; not this ticket"},
    {"takeoff helpers", PortStatus::kThisSlice,
     "takeoff.cpp do_user_takeoff_U_m gates (~18-40); see takeoff.hpp"},
    {"Mode::_TakeOff::start_m", PortStatus::kThisSlice,
     "takeoff.cpp ~51-57; leftover_takeoff_start_m sets _running/start_alt/"
     "complete_alt from pos_estimate_U_m inject"},
    {"do_pilot_takeoff_ms body", PortStatus::kThisSlice,
     "takeoff.cpp ~74-111; leftover_do_pilot_takeoff_ms: !_running return; "
     "land_complete→throttle/D_init flags; else pos_vel + near-alt stop"},
    {"crash_check / thrust_loss / yaw_imbalance", PortStatus::kOutOfScope,
     "land_detector.cpp ~30-32 call sites; full bodies CCP-042 OOS"},
    {"update_land_detector stationary AND-gate", PortStatus::kThisSlice,
     "land_detector.cpp ~92-155; motors_throttle_low && mix_min && "
     "!large_angle_* && accel_stationary && descent && rf && WoW → "
     "land_complete (count threshold skipped)"},
    {"set_land_complete disarm-on-land", PortStatus::kOutOfScope,
     "land_detector.cpp ~207-263 logging/stats/flying/disarm side effects; "
     "thin change-detect CCP-035 update_land_and_crash_detectors.hpp"},
    {"Log_LDET / HAL_LOGGING", PortStatus::kOutOfScope, "logger objects ADR-0012"},
    {"HELI_FRAME land path", PortStatus::kOutOfScope, "multirotor leftover only"},
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

}  // namespace fwcpp::copter::land_detector
