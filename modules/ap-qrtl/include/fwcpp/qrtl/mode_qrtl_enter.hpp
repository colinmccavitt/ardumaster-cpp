#pragma once

#include <fwcpp/qrtl/qrtl_defaults.hpp>
#include <fwcpp/qrtl/qrtl_geometry.hpp>

#include <cstdint>

namespace fwcpp::qrtl {

enum class QrtlSubMode : std::uint8_t {
    kClimb = 0,
    kRtl = 1,
};

enum class QrtlEnterAction : std::uint8_t {
    kQLandInstead = 0,
    kClimb = 1,
    kRtl = 2,
};

struct QrtlEnterView {
    bool throttle_unlimited{true};
    float home_dist_m{200.0F};
    float rally_dist_m{0.0F};
    bool has_rally{false};
    bool rally_incl_home{true};
    float loiter_radius_m{kWpLoiterRadDefaultM};
    float rtl_radius_m{kRtlRadiusDefaultM};
    float qrtl_alt_m{kQRtlAltDefaultM};
    float qrtl_alt_min_m{kQRtlAltMinDefaultM};
    float land_final_alt_m{kQLandFinalAltDefaultM};
    float relative_ground_alt_m{5.0F};
    std::int32_t current_alt_abs_cm{500};
    std::int32_t home_alt_abs_cm{0};
};

struct QrtlEnterResult {
    bool accepted{true};
    QrtlEnterAction action{QrtlEnterAction::kRtl};
    QrtlSubMode submode{QrtlSubMode::kRtl};
    QrtlDestination dest{QrtlDestination::kHome};
    float dist_m{0.0F};
    float radius_m{0.0F};
    std::int32_t rtl_alt_abs_cm{0};
    float climb_target_alt_m{0.0F};
    float dist_to_climb_m{0.0F};
    std::int32_t climb_next_wp_alt_cm{0};
};

struct QrtlEnterEffects {
    bool request_qland_instead{false};
    bool set_position1{false};
    bool do_rtl{false};
    bool poscontrol_init_approach{false};
    bool slow_descent{false};
};

[[nodiscard]] inline QrtlEnterResult qrtl_enter(const QrtlEnterView& view, bool guided_wait_takeoff,
                                                QrtlEnterEffects& effects) {
    effects = QrtlEnterEffects{};
    if (guided_wait_takeoff) {
        effects.request_qland_instead = true;
        QrtlEnterResult out{};
        out.action = QrtlEnterAction::kQLandInstead;
        out.submode = QrtlSubMode::kRtl;
        out.dest = QrtlDestination::kHome;
        return out;
    }

    const QrtlDestination dest =
        calc_best_rally_or_home(view.home_dist_m, view.rally_dist_m, view.rally_incl_home,
                              view.has_rally);
    const float dist_m = qrtl_destination_dist_m(dest, view.home_dist_m, view.rally_dist_m);
    const float radius_m = qrtl_vtol_return_radius_m(view.loiter_radius_m, view.rtl_radius_m);
    const float min_climb_m =
        qrtl_min_climb_m(view.qrtl_alt_min_m, view.land_final_alt_m, view.qrtl_alt_m);
    const float climb_target_alt_m =
        qrtl_climb_cone_target_alt_m(view.qrtl_alt_m, dist_m, radius_m, min_climb_m);
    const float dist_to_climb_m = climb_target_alt_m - view.relative_ground_alt_m;
    std::int32_t rtl_alt_abs_cm =
        view.home_alt_abs_cm + static_cast<std::int32_t>(view.qrtl_alt_m * 100.0F);
    const std::int32_t climb_next_wp_alt_cm =
        view.current_alt_abs_cm + static_cast<std::int32_t>(dist_to_climb_m * 100.0F);

    if (view.throttle_unlimited && is_positive(dist_to_climb_m)) {
        QrtlEnterResult out{};
        out.action = QrtlEnterAction::kClimb;
        out.submode = QrtlSubMode::kClimb;
        out.dest = dest;
        out.dist_m = dist_m;
        out.radius_m = radius_m;
        out.rtl_alt_abs_cm = rtl_alt_abs_cm;
        out.climb_target_alt_m = climb_target_alt_m;
        out.dist_to_climb_m = dist_to_climb_m;
        out.climb_next_wp_alt_cm = climb_next_wp_alt_cm;
        return out;
    }

    if (view.throttle_unlimited && dist_m < radius_m) {
        if (view.current_alt_abs_cm < rtl_alt_abs_cm) {
            rtl_alt_abs_cm = view.current_alt_abs_cm;
        }
        effects.set_position1 = true;
    }

    effects.do_rtl = true;
    effects.poscontrol_init_approach = true;
    effects.slow_descent = view.current_alt_abs_cm > rtl_alt_abs_cm;

    QrtlEnterResult out{};
    out.action = QrtlEnterAction::kRtl;
    out.submode = QrtlSubMode::kRtl;
    out.dest = dest;
    out.dist_m = dist_m;
    out.radius_m = radius_m;
    out.rtl_alt_abs_cm = rtl_alt_abs_cm;
    out.climb_target_alt_m = climb_target_alt_m;
    out.dist_to_climb_m = dist_to_climb_m;
    out.climb_next_wp_alt_cm = climb_next_wp_alt_cm;
    return out;
}

}  // namespace fwcpp::qrtl
