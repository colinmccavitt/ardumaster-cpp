#pragma once

// CCP-041 land_detector leftover scaffold — ArduCopter/land_detector.cpp
// (Plane-4.7.0). Thin leftover for update_land_and_crash_detectors (~16-33)
// and update_land_detector start (~37+). No AHRS / motors / LPF / parachute /
// crash objects (ADR-0012): inject motors_armed, throttle/descent checks,
// land_complete, accel_ef_z_plus_g.
//
// Do NOT port crash_check / thrust_loss_check / yaw_imbalance_check bodies
// this slice — catalog remaining. ModeRTL / ModeLand leftovers already on
// main (CCP-036). takeoff.cpp helpers and land_run_normal body remain.
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
    // Armed-path injects (stationary AND-gate body remaining).
    bool descent_rate_low{false};
    bool throttle_at_lower_limit{false};
};

struct LandDetectorEffects {
    bool land_accel_filter_applied{false};
    bool update_land_detector_ran{false};
    bool crash_check_ran{false};
    bool thrust_loss_check_ran{false};
    bool yaw_imbalance_check_ran{false};
    bool land_complete_set{false};
    bool land_complete{false};
    // Armed path: injects consulted (no full AND-gate).
    bool descent_check_inject{false};
    bool throttle_check_inject{false};
    float accel_ef_z_plus_g{0.0f};
};

// Leftover Copter::update_land_detector (land_detector.cpp ~37+). Thin:
// !armed → land_complete true path flags; else descent/throttle as injects.
inline void leftover_update_land_detector(const LandDetectorInputs& in,
                                          LandDetectorEffects& fx) {
    fx.land_complete = in.land_complete;
    if (!in.motors_armed) {
        fx.land_complete = true;
        fx.land_complete_set = true;
        return;
    }
    fx.descent_check_inject = in.descent_rate_low;
    fx.throttle_check_inject = in.throttle_zero || in.throttle_at_lower_limit;
    // Full motor_at_lower_limit && mix_min && angle && accel && vel &&
    // rangefinder && WoW AND-gate + land_detector_count trigger remain.
}

// Leftover Copter::update_land_and_crash_detectors (~16-33). Filter apply
// + always call leftover_update_land_detector. crash/thrust/yaw flags stay
// false (bodies not ported). HAL_PARACHUTE_ENABLED parachute_check skipped.
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
     "land_detector.cpp ~37+; !armed→land_complete; else descent/throttle injects"},
    {"ModeRTL", PortStatus::kOnMain,
     "CCP-036; mode.hpp ModeRTL::init/run leftovers on main"},
    {"ModeLand", PortStatus::kOnMain,
     "CCP-036; mode.hpp ModeLand::init/run leftovers on main"},
    {"land_run_normal body", PortStatus::kRemaining,
     "land_run_normal_or_precland / land_run_horizontal_control bodies"},
    {"takeoff helpers", PortStatus::kRemaining,
     "ArduCopter/takeoff.cpp; Mode::_TakeOff / do_user_takeoff / auto_takeoff"},
    {"crash_check / thrust_loss / yaw_imbalance", PortStatus::kRemaining,
     "land_detector.cpp ~30-32 call sites; full bodies not ported"},
    {"update_land_detector stationary AND-gate", PortStatus::kRemaining,
     "motor_at_lower_limit && mix_min && angle/accel/vel/rangefinder/WoW + count"},
    {"set_land_complete disarm-on-land", PortStatus::kRemaining,
     "land_detector.cpp ~207-263 logging/stats/flying/disarm side effects"},
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
