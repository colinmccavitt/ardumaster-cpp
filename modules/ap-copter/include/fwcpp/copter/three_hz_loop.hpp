#pragma once

// Copter::three_hz_loop leftover. Upstream Copter.cpp ~742-760 always
// calls failsafe_gcs_check, failsafe_terrain_check,
// failsafe_deadreckon_check, and low_alt_avoidance. Those callee
// bodies stay out of this slice (failsafe bodies are CCP-042) — this
// tick only records that they ran.
//
// #if AP_RC_TRANSMITTER_TUNING_ENABLED tuning() is gated leftover;
// transmitter_tuning stays false this slice. No GCS / terrain / RC
// objects — flags only.

namespace fwcpp::copter {

struct ThreeHzLoopEffects {
    bool failsafe_gcs_check{true};
    bool failsafe_terrain_check{true};
    bool failsafe_deadreckon_check{true};
    bool transmitter_tuning{false};
    bool low_alt_avoidance{true};
};

[[nodiscard]] inline constexpr ThreeHzLoopEffects three_hz_loop() {
    return ThreeHzLoopEffects{
        .failsafe_gcs_check = true,
        .failsafe_terrain_check = true,
        .failsafe_deadreckon_check = true,
        .transmitter_tuning = false,
        .low_alt_avoidance = true,
    };
}

}  // namespace fwcpp::copter
