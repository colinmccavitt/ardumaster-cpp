#pragma once

#include <fwcpp/math/vector3.hpp>

namespace fwcpp::q_loiter {

struct QLoiterSystemidInputs {
    bool systemid_enabled{false};
    math::Vector3f attitude_offset_deg{};
};

struct QLoiterSystemidEffects {
    bool vtol_update_called{false};
    bool apply_attitude_offset{false};
};

/// Port of `#if AP_PLANE_SYSTEMID_ENABLED` in ModeQLoiter::run.
/// Compile-time optional: `systemid_enabled` false leaves the attitude target unchanged.
[[nodiscard]] inline math::Vector3f qloiter_apply_systemid_to_target(
    math::Vector3f target, const QLoiterSystemidInputs& in, QLoiterSystemidEffects& effects) {
    effects = QLoiterSystemidEffects{};
    if (!in.systemid_enabled) {
        return target;
    }
    effects.vtol_update_called = true;
    effects.apply_attitude_offset = true;
    target += in.attitude_offset_deg;
    return target;
}

}  // namespace fwcpp::q_loiter
