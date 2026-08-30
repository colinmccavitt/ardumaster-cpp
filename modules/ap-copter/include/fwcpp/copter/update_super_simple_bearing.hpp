#pragma once

// Copter::update_super_simple_bearing leftover. Upstream Copter.cpp
// ~891-914. Inject force_update, simple_mode, home_distance_m,
// home_bearing_rad, super_simple_last_bearing_rad. Reuses SimpleMode
// from update_simple_mode.hpp — do not change that leftover.
//
// !force_update: return unless SUPERSIMPLE and home_distance_m >=
// SUPER_SIMPLE_RADIUS_M (10.0). force_update bypasses mode and radius.
// Then skip if fabs(wrap_PI(last - home)) < radians(5). On update:
// last = home_bearing; cos/sin of last + radians(180).
//
// run_nav_updates.hpp only records calling this with false — do not
// steal that leftover.

#include <cmath>

#include <fwcpp/copter/update_simple_mode.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::copter {

inline constexpr float kSuperSimpleRadiusM = 10.0f;

struct UpdateSuperSimpleBearingInputs {
    bool force_update{false};
    SimpleMode simple_mode{SimpleMode::NONE};
    float home_distance_m{0};
    float home_bearing_rad{0};
    float super_simple_last_bearing_rad{0};
};

struct UpdateSuperSimpleBearingEffects {
    bool updated{false};
    float super_simple_last_bearing_rad{0};
    float super_simple_cos_yaw{0};
    float super_simple_sin_yaw{0};
    bool skipped_not_supersimple{false};
    bool skipped_inside_radius{false};
    bool skipped_bearing_unchanged{false};
};

[[nodiscard]] inline UpdateSuperSimpleBearingEffects update_super_simple_bearing(
    const UpdateSuperSimpleBearingInputs& in = {}) {
    UpdateSuperSimpleBearingEffects fx{};
    fx.super_simple_last_bearing_rad = in.super_simple_last_bearing_rad;

    if (!in.force_update) {
        if (in.simple_mode != SimpleMode::SUPERSIMPLE) {
            fx.skipped_not_supersimple = true;
            return fx;
        }
        if (in.home_distance_m < kSuperSimpleRadiusM) {
            fx.skipped_inside_radius = true;
            return fx;
        }
    }

    const float bearing_rad = in.home_bearing_rad;

    if (std::fabs(fwcpp::math::wrap_PI(in.super_simple_last_bearing_rad - bearing_rad)) <
        fwcpp::math::radians(5.0f)) {
        fx.skipped_bearing_unchanged = true;
        return fx;
    }

    fx.super_simple_last_bearing_rad = bearing_rad;
    const float angle_rad =
        fx.super_simple_last_bearing_rad + fwcpp::math::radians(180.0f);
    fx.super_simple_cos_yaw = std::cos(angle_rad);
    fx.super_simple_sin_yaw = std::sin(angle_rad);
    fx.updated = true;
    return fx;
}

}  // namespace fwcpp::copter
