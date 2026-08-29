#pragma once

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>
#include <fwcpp/qrtl/mode_qrtl_enter.hpp>
#include <fwcpp/qrtl/qrtl_defaults.hpp>

#include <algorithm>
#include <cmath>

namespace fwcpp::qrtl {

using fwcpp::quadplane::PositionControlState;

enum class QrtlTargetAltAction : std::uint8_t {
    kDelegateBase = 0,
    kApproachProfile = 1,
};

struct QrtlTargetAltView {
    QrtlSubMode submode{QrtlSubMode::kRtl};
    PositionControlState poscontrol_state{PositionControlState::kNone};
    float loiter_radius_m{kWpLoiterRadDefaultM};
    float rtl_radius_m{kRtlRadiusDefaultM};
    float rtl_altitude_m{60.0F};
    float qrtl_alt_m{kQRtlAltDefaultM};
    float max_sinkrate_ms{2.0F};
    float airspeed_cruise_ms{15.0F};
    float wp_distance_m{100.0F};
};

struct QrtlTargetAltResult {
    QrtlTargetAltAction action{QrtlTargetAltAction::kDelegateBase};
    float approach_offset_up_m{0.0F};
};

[[nodiscard]] inline float qrtl_approach_alt_offset_m(const QrtlTargetAltView& view) {
    const float radius =
        std::max(std::fabs(view.loiter_radius_m), std::fabs(view.rtl_radius_m));
    const float rtl_alt_delta = std::max(0.0F, view.rtl_altitude_m - view.qrtl_alt_m);
    const float sink_time = rtl_alt_delta / std::max(0.6F * view.max_sinkrate_ms, 1.0F);
    const float sink_dist = view.airspeed_cruise_ms * sink_time;
    const float dist = view.wp_distance_m;
    const float rad_min = 2.0F * radius;
    const float rad_max = 20.0F * radius;
    const float upper = std::max(rad_min, std::min(rad_max, rad_min + sink_dist));
    return fwcpp::math::linear_interpolate(0.0F, rtl_alt_delta, dist, rad_min, upper);
}

/// Port of ModeQRTL::update_target_altitude() control flow.
[[nodiscard]] inline QrtlTargetAltResult qrtl_update_target_altitude(const QrtlTargetAltView& view) {
    if (view.submode != QrtlSubMode::kRtl ||
        view.poscontrol_state != PositionControlState::kApproach) {
        QrtlTargetAltResult out{};
        out.action = QrtlTargetAltAction::kDelegateBase;
        return out;
    }
    QrtlTargetAltResult out{};
    out.action = QrtlTargetAltAction::kApproachProfile;
    out.approach_offset_up_m = qrtl_approach_alt_offset_m(view);
    return out;
}

/// Port of ModeQRTL::allows_throttle_nudging().
[[nodiscard]] inline bool qrtl_allows_throttle_nudging(QrtlSubMode submode,
                                                       PositionControlState poscontrol_state) {
    return submode == QrtlSubMode::kRtl && poscontrol_state == PositionControlState::kApproach;
}

}  // namespace fwcpp::qrtl
