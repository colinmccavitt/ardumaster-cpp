// CCP-028 slice 2: update_wpnav parity (Rust tests/update_wpnav.rs)

#include <cmath>
#include <optional>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/wpnav/wpnav.hpp>

using Catch::Approx;
using fwcpp::math::Vector3;
using fwcpp::wpnav::AttitudeJerkLimits;
using fwcpp::wpnav::SetWpDestinationContext;
using fwcpp::wpnav::UpdateWpNavContext;
using fwcpp::wpnav::WpNav;
using fwcpp::wpnav::kWpSpdDefault;
using fwcpp::wpnav::kWpSpdDownDefault;
using fwcpp::wpnav::kWpSpdMin;
using fwcpp::wpnav::kWpSpdUpDefault;
using fwcpp::wpnav::kWpnavAccelerationMss;
using fwcpp::wpnav::kWpnavActiveTimeoutMs;

namespace {

void almost(float a, float b) { REQUIRE(std::abs(a - b) <= 1e-5f); }

SetWpDestinationContext dest_ctx(std::uint32_t now_ms, const Vector3<float>& stopping_point_ned_m) {
    return SetWpDestinationContext{.now_ms = now_ms,
                                   .attitude = AttitudeJerkLimits{},
                                   .stopping_point_ned_m = stopping_point_ned_m,
                                   .terrain_d_m = std::nullopt};
}

UpdateWpNavContext tick(std::uint32_t now_ms) {
    return UpdateWpNavContext{.now_ms = now_ms, .dt_s = 0.01f, .terrain_d_m = std::nullopt};
}

}  // namespace

TEST_CASE("update stamps last update and stays active", "[wpnav][update]") {
    WpNav nav;
    nav.wp_and_spline_init_m(0.0f, Vector3<float>{}, 1000, AttitudeJerkLimits{});
    REQUIRE_FALSE(nav.is_active(1000 + kWpnavActiveTimeoutMs));

    const auto leftover = nav.update_wpnav(tick(1250));
    REQUIRE(leftover.advance_ok);
    REQUIRE(leftover.need_advance_track);
    REQUIRE(leftover.need_ne_update_controller);
    REQUIRE_FALSE(leftover.applied_speed_ne);
    REQUIRE_FALSE(leftover.applied_speed_up);
    REQUIRE_FALSE(leftover.applied_speed_down);
    REQUIRE_FALSE(leftover.need_update_track_limits);
    almost(leftover.dt_s, 0.01f);
    REQUIRE(nav.is_active(1250));
    REQUIRE(nav.is_active(1250 + kWpnavActiveTimeoutMs - 1));
    REQUIRE_FALSE(nav.is_active(1250 + kWpnavActiveTimeoutMs));
}

TEST_CASE("wp spd change applies set speed ne when watched", "[wpnav][update]") {
    WpNav nav;
    nav.wp_and_spline_init_m(0.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    REQUIRE(nav.check_wp_speed_change());
    almost(nav.desired_speed_ne_ms(), kWpSpdDefault);
    almost(nav.offset_vel_ms(), kWpSpdDefault);

    nav.set_wp_speed_ms(7.0f);
    auto leftover = nav.update_wpnav(tick(10));
    REQUIRE(leftover.applied_speed_ne);
    REQUIRE(leftover.need_update_track_limits);
    almost(nav.desired_speed_ne_ms(), 7.0f);
    almost(nav.offset_vel_ms(), 7.0f);
    almost(nav.last_wp_speed_ms(), 7.0f);
    almost(nav.pos_speed_accel().ne_speed_ms, 7.0f);
    almost(nav.pos_speed_accel().ne_accel_mss, kWpnavAccelerationMss);

    leftover = nav.update_wpnav(tick(20));
    REQUIRE_FALSE(leftover.applied_speed_ne);
    REQUIRE_FALSE(leftover.need_update_track_limits);
    almost(nav.desired_speed_ne_ms(), 7.0f);
}

TEST_CASE("explicit init speed does not watch wp spd", "[wpnav][update]") {
    WpNav nav;
    nav.wp_and_spline_init_m(4.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    REQUIRE_FALSE(nav.check_wp_speed_change());

    nav.set_wp_speed_ms(8.0f);
    const auto leftover = nav.update_wpnav(tick(10));
    REQUIRE_FALSE(leftover.applied_speed_ne);
    almost(nav.desired_speed_ne_ms(), 4.0f);
    almost(nav.offset_vel_ms(), 4.0f);
}

TEST_CASE("climb and descent param changes always apply", "[wpnav][update]") {
    WpNav nav;
    nav.wp_and_spline_init_m(3.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    almost(nav.pos_speed_accel().speed_up_ms, kWpSpdUpDefault);
    almost(nav.pos_speed_accel().speed_down_ms, kWpSpdDownDefault);

    nav.set_wp_speed_up_ms(1.25f);
    nav.set_wp_speed_down_ms(0.75f);
    const auto leftover = nav.update_wpnav(tick(10));
    REQUIRE(leftover.applied_speed_up);
    REQUIRE(leftover.applied_speed_down);
    REQUIRE(leftover.need_update_track_limits);
    almost(nav.pos_speed_accel().speed_up_ms, 1.25f);
    almost(nav.pos_speed_accel().speed_down_ms, 0.75f);
    almost(nav.last_wp_speed_up_ms(), 1.25f);
    almost(nav.last_wp_speed_down_ms(), 0.75f);
}

TEST_CASE("set speed ne rejects below floor and zero desired", "[wpnav][update]") {
    WpNav nav;
    REQUIRE_FALSE(nav.set_speed_ne_ms(5.0f));
    almost(nav.desired_speed_ne_ms(), 0.0f);

    nav.wp_and_spline_init_m(4.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    REQUIRE_FALSE(nav.set_speed_ne_ms(kWpSpdMin - 0.001f));
    almost(nav.desired_speed_ne_ms(), 4.0f);
    REQUIRE(nav.set_speed_ne_ms(kWpSpdMin));
    almost(nav.desired_speed_ne_ms(), kWpSpdMin);
}

TEST_CASE("set speed ne scales offset vel ratio", "[wpnav][update]") {
    WpNav nav;
    nav.wp_and_spline_init_m(10.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    REQUIRE(nav.set_speed_ne_ms(10.0f));
    REQUIRE(nav.set_speed_ne_ms(5.0f));
    almost(nav.offset_vel_ms(), 5.0f);
    almost(nav.desired_speed_ne_ms(), 5.0f);
}

TEST_CASE("terrain alt without offset fails advance but still stamps", "[wpnav][update]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -4.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 0, AttitudeJerkLimits{});
    auto ctx = dest_ctx(0, stop);
    ctx.terrain_d_m = 2.0f;
    REQUIRE(nav.set_wp_destination_ned_m(Vector3<float>{8.0f, 0.0f, -1.0f}, true, 0.0f, ctx));
    REQUIRE(nav.origin_and_destination_are_terrain_alt());

    const auto leftover = nav.update_wpnav(tick(50));
    REQUIRE_FALSE(leftover.advance_ok);
    REQUIRE(leftover.need_advance_track);
    REQUIRE(leftover.need_ne_update_controller);
    REQUIRE(nav.is_active(50));
}

TEST_CASE("terrain alt with offset advance ok", "[wpnav][update]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -4.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 0, AttitudeJerkLimits{});
    auto ctx = dest_ctx(0, stop);
    ctx.terrain_d_m = 2.0f;
    REQUIRE(nav.set_wp_destination_ned_m(Vector3<float>{8.0f, 0.0f, -1.0f}, true, 0.0f, ctx));

    UpdateWpNavContext upd{.now_ms = 50, .dt_s = 0.02f, .terrain_d_m = 2.0f};
    const auto leftover = nav.update_wpnav(upd);
    REQUIRE(leftover.advance_ok);
    almost(leftover.dt_s, 0.02f);
}

TEST_CASE("get wp distance is horizontal and ignores z", "[wpnav][update]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -10.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 0, AttitudeJerkLimits{});
    REQUIRE(nav.set_wp_destination_ned_m(Vector3<float>{3.0f, 4.0f, -1.0f}, false, 0.0f, dest_ctx(0, stop)));

    const Vector3<float> pos{0.0f, 0.0f, 99.0f};
    almost(nav.get_wp_distance_to_destination_m(pos), 5.0f);
    almost(nav.get_wp_distance_to_destination_cm(pos), 500.0f);

    const Vector3<float> at_dest{3.0f, 4.0f, 0.0f};
    almost(nav.get_wp_distance_to_destination_m(at_dest), 0.0f);
}
