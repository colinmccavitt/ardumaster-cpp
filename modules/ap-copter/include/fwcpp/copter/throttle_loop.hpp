#pragma once

// Copter::throttle_loop leftover. Upstream Copter.cpp always calls
// update_throttle_mix, update_auto_armed, update_ground_effect_detector,
// and update_ekf_terrain_height_stable. Those callees stay in the catalog
// as their own leftover rows — this tick only records that they ran.
//
// #if FRAME_CONFIG == HELI_FRAME heli_update_rotor_speed_targets +
// heli_update_landing_swash are out of scope; this port is not heli and
// does not invent those bodies. No motors / AHRS / battery objects —
// flags only.

namespace fwcpp::copter {

struct ThrottleLoopEffects {
    bool update_throttle_mix{true};
    bool update_auto_armed{true};
    bool heli_update_rotor_speed_targets{false};
    bool heli_update_landing_swash{false};
    bool update_ground_effect_detector{true};
    bool update_ekf_terrain_height_stable{true};
};

[[nodiscard]] inline constexpr ThrottleLoopEffects throttle_loop() {
    return ThrottleLoopEffects{
        .update_throttle_mix = true,
        .update_auto_armed = true,
        .heli_update_rotor_speed_targets = false,
        .heli_update_landing_swash = false,
        .update_ground_effect_detector = true,
        .update_ekf_terrain_height_stable = true,
    };
}

}  // namespace fwcpp::copter
