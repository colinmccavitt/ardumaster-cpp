#pragma once

// Copter::takeoff_check leftover. Upstream ArduCopter/takeoff_check.cpp
// ~8-55. HAL_WITH_ESC_TELEM && FRAME_CONFIG != HELI_FRAME: this port
// is not heli, so the leftover body always runs (no empty #else).
// Inject now_ms, spoolup_block, land_complete, motor_check_passed
// (do not port motors_takeoff_check RPM math), has_system_load /
// avg_load / peak_load, and warning_ms. No GCS object —
// gcs_cpu_overload is true only on the 2 s cadence when load is
// inadequate (upstream send_text "Takeoff blocked: CPU overload").
//
// Do not port Copter::get_wp_distance_m.

#include <cstdint>

namespace fwcpp::copter {

inline constexpr std::uint32_t kTakeoffCheckWarningIntervalMs = 2000;
inline constexpr float kTakeoffCheckAvgLoadMax = 95.0f;
inline constexpr float kTakeoffCheckPeakLoadMax = 99.5f;

struct TakeoffCheckInputs {
    std::uint32_t now_ms{0};
    bool spoolup_block{false};
    bool land_complete{false};
    bool motor_check_passed{false};
    bool has_system_load{false};
    float avg_load{0.0f};
    float peak_load{0.0f};
    std::uint32_t warning_ms{0};
};

struct TakeoffCheckEffects {
    bool spoolup_block{false};
    std::uint32_t warning_ms{0};
    bool gcs_cpu_overload{false};
};

[[nodiscard]] inline TakeoffCheckEffects takeoff_check(
    const TakeoffCheckInputs& in = {}) {
    TakeoffCheckEffects fx{};
    fx.spoolup_block = in.spoolup_block;
    fx.warning_ms = in.warning_ms;

    // Unblocked: reset the warning timer so a later block can warn
    // immediately after the 2 s cadence. Motors can only be blocked
    // immediately after arming.
    if (!in.spoolup_block) {
        fx.warning_ms = in.now_ms;
        return fx;
    }

    // Immediately clear the spool-up block if not landed.
    if (!in.land_complete) {
        fx.spoolup_block = false;
        return fx;
    }

    // motors_takeoff_check RPM math is injected, not ported.
    const bool motor_check_passed = in.motor_check_passed;

    bool load_adequate = true;
    if (in.has_system_load) {
        if (in.avg_load > kTakeoffCheckAvgLoadMax ||
            in.peak_load > kTakeoffCheckPeakLoadMax) {
            load_adequate = false;
        }
    }

    if (motor_check_passed && load_adequate) {
        fx.spoolup_block = false;
        return fx;
    }

    if (in.now_ms - in.warning_ms > kTakeoffCheckWarningIntervalMs) {
        fx.warning_ms = in.now_ms;
        if (!load_adequate) {
            fx.gcs_cpu_overload = true;
        }
    }

    return fx;
}

}  // namespace fwcpp::copter
