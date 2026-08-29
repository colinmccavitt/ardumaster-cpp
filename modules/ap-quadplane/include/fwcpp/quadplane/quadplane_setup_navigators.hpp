#pragma once

// Navigator objects allocated in QuadPlane::setup — upstream
// AC_WPNav / AC_Loiter construction (Plane-4.7.0 quadplane.cpp).
// ADR-0012: AHRS view and pos/attitude controller readiness are explicit
// inputs; wp_and_spline_init_m uses caller-supplied stopping point, time,
// and attitude jerk limits (no AP::ahrs() / PosControl singletons).

#include <cstdint>

#include <fwcpp/math/vector3.hpp>
#include <fwcpp/wpnav/wpnav.hpp>

namespace fwcpp::quadplane {

struct NavigatorDeps {
    bool ahrs_view_created{false};
    bool attitude_control_inited{false};
    bool pos_control_inited{false};
};

struct WpNavSetupInputs {
    float init_speed_ms{0.f};
    fwcpp::math::Vector3<float> stopping_point_ned_m{};
    std::uint32_t now_ms{0};
    fwcpp::wpnav::AttitudeJerkLimits attitude{};
};

struct LoiterNavStub {
    bool created{false};
    bool ahrs_view_wired{false};
    bool pos_control_wired{false};
    bool attitude_control_wired{false};
};

struct NavigatorSetupResult {
    bool ok{false};
    fwcpp::wpnav::WpNav wp_nav{};
    LoiterNavStub loiter_nav{};
    bool wp_and_spline_inited{false};
};

[[nodiscard]] inline bool navigator_deps_ready(const NavigatorDeps& deps) {
    return deps.ahrs_view_created && deps.attitude_control_inited && deps.pos_control_inited;
}

[[nodiscard]] inline NavigatorSetupResult wire_setup_navigators(const NavigatorDeps& deps,
                                                                const WpNavSetupInputs& wp_in) {
    NavigatorSetupResult out{};
    if (!navigator_deps_ready(deps)) {
        return out;
    }

    out.wp_nav = fwcpp::wpnav::WpNav{};
    out.loiter_nav.created = true;
    out.loiter_nav.ahrs_view_wired = deps.ahrs_view_created;
    out.loiter_nav.pos_control_wired = deps.pos_control_inited;
    out.loiter_nav.attitude_control_wired = deps.attitude_control_inited;

    out.wp_nav.wp_and_spline_init_m(wp_in.init_speed_ms, wp_in.stopping_point_ned_m, wp_in.now_ms,
                                    wp_in.attitude);
    out.wp_and_spline_inited = true;
    out.ok = true;
    return out;
}

}  // namespace fwcpp::quadplane
