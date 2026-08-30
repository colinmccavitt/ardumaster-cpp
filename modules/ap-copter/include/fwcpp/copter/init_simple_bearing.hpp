#pragma once

// Copter::init_simple_bearing leftover. Upstream Copter.cpp ~837-854.
// Inject ahrs cos_yaw / sin_yaw / yaw_rad and should_log(MASK_LOG_ANY).
// No ahrs / logger objects. Log leftover is a bool flag only — do not
// invent a yaw_sensor centidegree conversion.
//
// simple_cos_yaw / simple_sin_yaw capture injected AHRS yaw trig.
// super_simple_last_bearing_rad = wrap_2PI(yaw_rad + radians(180)).
// super_simple_cos/sin copy the simple values.
//
// Do not port update_simple_mode (roll/pitch rotate) or
// update_super_simple_bearing (radius / 5deg) — own remaining rows.

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::copter {

struct InitSimpleBearingInputs {
    float cos_yaw{0};
    float sin_yaw{0};
    float yaw_rad{0};
    bool should_log_any{false};
};

struct InitSimpleBearingEffects {
    float simple_cos_yaw{0};
    float simple_sin_yaw{0};
    float super_simple_last_bearing_rad{0};
    float super_simple_cos_yaw{0};
    float super_simple_sin_yaw{0};
    bool log_init_simple_bearing{false};
};

[[nodiscard]] inline InitSimpleBearingEffects init_simple_bearing(
    const InitSimpleBearingInputs& in = {}) {
    InitSimpleBearingEffects fx{};

    fx.simple_cos_yaw = in.cos_yaw;
    fx.simple_sin_yaw = in.sin_yaw;

    fx.super_simple_last_bearing_rad =
        fwcpp::math::wrap_2PI(in.yaw_rad + fwcpp::math::radians(180.0f));
    fx.super_simple_cos_yaw = fx.simple_cos_yaw;
    fx.super_simple_sin_yaw = fx.simple_sin_yaw;

    if (in.should_log_any) {
        fx.log_init_simple_bearing = true;
    }

    return fx;
}

}  // namespace fwcpp::copter
