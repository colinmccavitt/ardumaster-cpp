#pragma once

// Copter::update_rangefinder_terrain_offset leftover. Upstream
// ArduCopter/sensors.cpp ~54-68. No rangefinder / WPNav / Circle objects
// — inject down + up rangefinder-state structs, G_Dt, and surftrak_tc.
//
// Down: terrain_u_m = ref_pos_u_m - alt_glitch_protected_m (MINUS).
// Up:   terrain_u_m = ref_pos_u_m + alt_glitch_protected_m (PLUS).
// Both apply the surftrak LPF:
//   terrain_u_m += (target - terrain_u_m) * (G_Dt / max(surftrak_tc, G_Dt))
//
// If down.alt_healthy || down.data_stale, record wp_nav
// set_rangefinder_terrain_u_m(enabled, alt_healthy, updated terrain).
// MODE_CIRCLE_ENABLED circle_nav->set_rangefinder_terrain_U_m is leftover
// (fwcpp::wpnav::Circle has no setter this slice); flag stays false.
//
// read_barometer / init_rangefinder / read_rangefinder / rangefinder_alt_ok
// are later leftovers.

#include <algorithm>

namespace fwcpp::copter {

struct RangefinderTerrainState {
    float ref_pos_u_m{0};
    float alt_glitch_protected_m{0};
    float terrain_u_m{0};
    bool enabled{false};
    bool alt_healthy{false};
    bool data_stale{false};
};

struct UpdateRangefinderTerrainOffsetInputs {
    RangefinderTerrainState rangefinder_state{};
    RangefinderTerrainState rangefinder_up_state{};
    float g_dt{0};
    float surftrak_tc{0};
};

struct UpdateRangefinderTerrainOffsetEffects {
    bool wp_nav_set_rangefinder_terrain{false};
    bool wp_nav_enabled{false};
    bool wp_nav_alt_healthy{false};
    float wp_nav_terrain_u_m{0};
    bool circle_nav_set_rangefinder_terrain{false};  // MODE_CIRCLE_ENABLED leftover
};

[[nodiscard]] inline UpdateRangefinderTerrainOffsetEffects
update_rangefinder_terrain_offset(UpdateRangefinderTerrainOffsetInputs& in) {
    UpdateRangefinderTerrainOffsetEffects fx{};

    const float alpha = in.g_dt / std::max(in.surftrak_tc, in.g_dt);

    float terrain_u_m =
        in.rangefinder_state.ref_pos_u_m - in.rangefinder_state.alt_glitch_protected_m;
    in.rangefinder_state.terrain_u_m +=
        (terrain_u_m - in.rangefinder_state.terrain_u_m) * alpha;

    terrain_u_m =
        in.rangefinder_up_state.ref_pos_u_m + in.rangefinder_up_state.alt_glitch_protected_m;
    in.rangefinder_up_state.terrain_u_m +=
        (terrain_u_m - in.rangefinder_up_state.terrain_u_m) * alpha;

    if (in.rangefinder_state.alt_healthy || in.rangefinder_state.data_stale) {
        fx.wp_nav_set_rangefinder_terrain = true;
        fx.wp_nav_enabled = in.rangefinder_state.enabled;
        fx.wp_nav_alt_healthy = in.rangefinder_state.alt_healthy;
        fx.wp_nav_terrain_u_m = in.rangefinder_state.terrain_u_m;
        // circle_nav leftover — MODE_CIRCLE_ENABLED, no Circle setter this slice
    }
    return fx;
}

}  // namespace fwcpp::copter
