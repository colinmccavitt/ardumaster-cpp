// CCP-028 slice 3: advance_wp_target_along_track parity (Rust tests/advance_wp_target.rs)

#include <cmath>
#include <numbers>
#include <optional>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/wpnav/wpnav.hpp>

using Catch::Approx;
using fwcpp::math::Vector3;
using fwcpp::wpnav::AdvanceWpTargetContext;
using fwcpp::wpnav::AttitudeJerkLimits;
using fwcpp::wpnav::SetWpDestinationContext;
using fwcpp::wpnav::WpNav;
using fwcpp::wpnav::kWpRadiusMDefault;
using fwcpp::wpnav::kWpSpdDefault;

namespace {

void almost(float a, float b) { REQUIRE(std::abs(a - b) <= 1e-4f); }

SetWpDestinationContext dest_ctx(std::uint32_t now_ms, const Vector3<float>& stopping_point_ned_m) {
    return SetWpDestinationContext{.now_ms = now_ms,
                                   .attitude = AttitudeJerkLimits{},
                                   .stopping_point_ned_m = stopping_point_ned_m,
                                   .terrain_d_m = std::nullopt};
}

void seat_dest(WpNav& nav, const Vector3<float>& dest) {
    const Vector3<float> stop{};
    nav.wp_and_spline_init_m(kWpSpdDefault, stop, 1000, AttitudeJerkLimits{});
    REQUIRE(nav.set_wp_destination_ned_m(dest, false, 0.0f, dest_ctx(1000, stop)));
    REQUIRE_FALSE(nav.reached_wp_destination());
}

AdvanceWpTargetContext advance_ctx(const Vector3<float>& pos, bool path_finished) {
    AdvanceWpTargetContext ctx{};
    ctx.pos_estimate_ned_m = pos;
    ctx.path_finished = path_finished;
    return ctx;
}

}  // namespace

TEST_CASE("terrain alt without offset fails advance", "[wpnav][advance]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -4.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 0, AttitudeJerkLimits{});
    auto ctx = dest_ctx(0, stop);
    ctx.terrain_d_m = 2.0f;
    REQUIRE(nav.set_wp_destination_ned_m(Vector3<float>{8.0f, 0.0f, -1.0f}, true, 0.0f, ctx));
    REQUIRE(nav.origin_and_destination_are_terrain_alt());

    AdvanceWpTargetContext adv{};
    adv.path_finished = true;
    const auto leftover = nav.advance_wp_target_along_track(adv);
    REQUIRE_FALSE(leftover.ok);
    REQUIRE_FALSE(leftover.need_set_pos_terrain_target);
    REQUIRE_FALSE(leftover.need_scurve_advance);
    REQUIRE_FALSE(leftover.need_spline_advance);
    REQUIRE_FALSE(leftover.need_set_pos_vel_accel);
    REQUIRE_FALSE(nav.reached_wp_destination());
}

TEST_CASE("path unfinished does not set reached", "[wpnav][advance]") {
    WpNav nav;
    seat_dest(nav, Vector3<float>{10.0f, 0.0f, 0.0f});

    const auto leftover = nav.advance_wp_target_along_track(advance_ctx({10.0f, 0.0f, 0.0f}, false));
    REQUIRE(leftover.ok);
    REQUIRE(leftover.need_scurve_advance);
    REQUIRE_FALSE(leftover.need_spline_advance);
    REQUIRE(leftover.need_set_pos_vel_accel);
    REQUIRE(leftover.need_set_pos_terrain_target);
    REQUIRE_FALSE(nav.reached_wp_destination());
}

TEST_CASE("regular waypoint reaches only inside 3d radius", "[wpnav][advance]") {
    WpNav nav;
    const Vector3<float> dest{10.0f, 0.0f, 0.0f};
    seat_dest(nav, dest);
    REQUIRE_FALSE(nav.flags().fast_waypoint);

    auto leftover = nav.advance_wp_target_along_track(advance_ctx({6.0f, 0.0f, 0.0f}, true));
    REQUIRE(leftover.ok);
    REQUIRE_FALSE(nav.reached_wp_destination());
    REQUIRE_FALSE(nav.reached_wp_destination_ne({6.0f, 0.0f, 0.0f}));

    leftover = nav.advance_wp_target_along_track(advance_ctx({11.0f, 0.0f, 3.0f}, true));
    REQUIRE(leftover.ok);
    REQUIRE_FALSE(nav.reached_wp_destination());
    REQUIRE(nav.reached_wp_destination_ne({11.0f, 0.0f, 3.0f}));

    leftover = nav.advance_wp_target_along_track(advance_ctx({11.0f, 0.0f, 0.5f}, true));
    REQUIRE(leftover.ok);
    REQUIRE(nav.reached_wp_destination());
}

TEST_CASE("fast waypoint reaches when path finishes", "[wpnav][advance]") {
    WpNav nav;
    seat_dest(nav, Vector3<float>{30.0f, 0.0f, 0.0f});
    nav.set_fast_waypoint(true);

    auto leftover = nav.advance_wp_target_along_track(advance_ctx({}, true));
    REQUIRE(leftover.ok);
    REQUIRE(nav.reached_wp_destination());

    leftover = nav.advance_wp_target_along_track(advance_ctx({}, false));
    REQUIRE(leftover.ok);
    REQUIRE(nav.reached_wp_destination());
}

TEST_CASE("pause shapes offset vel toward zero", "[wpnav][advance]") {
    WpNav nav;
    seat_dest(nav, Vector3<float>{20.0f, 0.0f, 0.0f});
    almost(nav.offset_vel_ms(), kWpSpdDefault);
    almost(nav.offset_accel_mss(), 0.0f);

    nav.set_pause();
    REQUIRE(nav.paused());
    AdvanceWpTargetContext ctx{};
    ctx.dt_s = 0.05f;
    ctx.shaping_jerk_ne_msss = 5.0f;
    auto leftover = nav.advance_wp_target_along_track(ctx);
    REQUIRE(leftover.ok);
    almost(leftover.vel_dt_scalar, 1.0f);
    almost(nav.offset_vel_ms(), kWpSpdDefault);
    REQUIRE(nav.offset_accel_mss() < 0.0f);

    leftover = nav.advance_wp_target_along_track(ctx);
    REQUIRE(leftover.ok);
    REQUIRE(leftover.vel_dt_scalar < 1.0f);
    REQUIRE(nav.offset_vel_ms() < kWpSpdDefault);

    nav.set_resume();
    REQUIRE_FALSE(nav.paused());
}

TEST_CASE("track dt scalar filters toward speed alignment", "[wpnav][advance]") {
    WpNav nav;
    seat_dest(nav, Vector3<float>{20.0f, 0.0f, 0.0f});
    almost(nav.track_dt_scalar(), 1.0f);

    AdvanceWpTargetContext ctx{};
    ctx.dt_s = 0.01f;
    ctx.vel_desired_ned_ms = Vector3<float>{10.0f, 0.0f, 0.0f};
    ctx.vel_estimate_ned_ms = Vector3<float>{5.0f, 0.0f, 0.0f};
    ctx.pos_error_ned_m = Vector3<float>{2.0f, 0.0f, 0.0f};
    ctx.pos_p_kp = 1.0f;
    const auto leftover = nav.advance_wp_target_along_track(ctx);
    REQUIRE(leftover.ok);
    almost(leftover.raw_track_dt_scalar, 0.35f);
    almost(nav.track_dt_scalar(), 1.0f + (0.35f - 1.0f) * (0.01f / 2.5f));
    almost(leftover.dt_along_track_s, nav.track_dt_scalar() * leftover.vel_dt_scalar * 0.01f);
}

TEST_CASE("bearing is clockwise from north", "[wpnav][advance]") {
    WpNav nav;
    seat_dest(nav, Vector3<float>{10.0f, 0.0f, -3.0f});

    almost(nav.get_wp_bearing_to_destination_rad({10.0f, 0.0f, 99.0f}), 0.0f);
    almost(nav.get_wp_bearing_to_destination_rad({10.0f, -5.0f, 0.0f}),
           std::numbers::pi_v<float> / 2.0f);
    almost(nav.get_wp_bearing_to_destination_rad({0.0f, 0.0f, 0.0f}), 0.0f);
    REQUIRE(nav.get_wp_bearing_to_destination_cd({0.0f, 0.0f, 0.0f}) == 0);

    const float west = nav.get_wp_bearing_to_destination_rad({10.0f, 5.0f, 0.0f});
    almost(west, 3.0f * std::numbers::pi_v<float> / 2.0f);
}

TEST_CASE("reached ne uses horizontal radius only", "[wpnav][advance]") {
    WpNav nav;
    seat_dest(nav, Vector3<float>{});
    almost(nav.wp_radius_m(), kWpRadiusMDefault);

    REQUIRE(nav.reached_wp_destination_ne({1.0f, 0.0f, 50.0f}));
    REQUIRE_FALSE(nav.reached_wp_destination_ne({3.0f, 0.0f, 0.0f}));
}

TEST_CASE("update track limits leftover flags legs", "[wpnav][advance]") {
    WpNav nav;
    nav.wp_and_spline_init_m(kWpSpdDefault, Vector3<float>{}, 0, AttitudeJerkLimits{});
    const auto limits = nav.update_track_with_speed_accel_limits();
    REQUIRE(limits.need_this_scurve_speed_max);
    REQUIRE_FALSE(limits.need_this_spline_speed_accel);
    REQUIRE(limits.need_next_scurve_speed_max);
    REQUIRE_FALSE(limits.need_next_spline_speed_accel);
}
