#pragma once

// Copter::read_inertia leftover. Upstream ArduCopter/inertia.cpp.
// No AHRS / pos_control / Follow objects — inject high_vibes,
// follow_enabled, AHRS lat/lng, relative-pos-D availability, and
// AltitudeContext for change_alt_frame.
//
// Follow is MODE_FOLLOW leftover only: flag update_follow_estimates
// when follow_enabled; do not implement Follow.

#include <cstdint>

#include <fwcpp/location.hpp>

namespace fwcpp::copter {

struct ReadInertiaInputs {
    bool high_vibes{false};
    bool follow_enabled{false};
    std::int32_t ahrs_lat{0};
    std::int32_t ahrs_lng{0};
    bool has_rel_pos_d{false};
    float pos_d_m{0};
    bool home_is_set{false};
};

struct ReadInertiaEffects {
    bool pos_control_update_estimates{true};
    bool high_vibes{false};
    bool update_follow_estimates{false};
    bool alt_written{false};
    bool set_alt_above_home_fallback{false};
};

[[nodiscard]] inline ReadInertiaEffects read_inertia(const ReadInertiaInputs& in,
                                                     Location& current_loc,
                                                     const AltitudeContext& ctx) {
    ReadInertiaEffects fx{};
    fx.pos_control_update_estimates = true;
    fx.high_vibes = in.high_vibes;
    fx.update_follow_estimates = in.follow_enabled;

    current_loc.lat = in.ahrs_lat;
    current_loc.lng = in.ahrs_lng;

    // Upstream returns after lat/lng copy when AHRS has no
    // relative-position-D-origin estimate. Do not touch alt.
    if (!in.has_rel_pos_d) {
        return fx;
    }

    const float alt_above_origin_m = -in.pos_d_m;
    current_loc.set_alt_m(alt_above_origin_m, Location::AltFrame::ABOVE_ORIGIN);
    fx.alt_written = true;

    if (!in.home_is_set ||
        !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME, ctx)) {
        current_loc.set_alt_m(alt_above_origin_m, Location::AltFrame::ABOVE_HOME);
        fx.set_alt_above_home_fallback = true;
    }
    return fx;
}

}  // namespace fwcpp::copter
