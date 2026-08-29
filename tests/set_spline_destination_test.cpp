// CCP-028 slice 6: set_spline_destination parity (Rust tests/set_spline_destination.rs)

#include <cmath>
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

namespace {

void almost(float a, float b) { REQUIRE(std::abs(a - b) <= 1e-5f); }

void almost_vec(const Vector3<float>& got, const Vector3<float>& expected) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
    almost(got.z, expected.z);
}

SetWpDestinationContext ctx_at(std::uint32_t now_ms, const Vector3<float>& stopping_point_ned_m) {
    return SetWpDestinationContext{.now_ms = now_ms,
                                   .attitude = AttitudeJerkLimits{},
                                   .stopping_point_ned_m = stopping_point_ned_m,
                                   .terrain_d_m = std::nullopt};
}

}  // namespace

TEST_CASE("set spline destination uses previous dest as origin", "[wpnav][spline]") {
    WpNav nav;
    const Vector3<float> stop{1.0f, 2.0f, 3.0f};
    nav.wp_and_spline_init_m(4.0f, stop, 1000, AttitudeJerkLimits{});

    const Vector3<float> dest{10.0f, -4.0f, 1.5f};
    const Vector3<float> next{20.0f, 0.0f, 0.0f};
    REQUIRE(nav.set_spline_destination_ned_m(dest, false, next, false, false, ctx_at(1000, stop)));

    REQUIRE(nav.wp_origin_ned_m().x == Approx(stop.x));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));
    REQUIRE_FALSE(nav.reached_wp_destination());
    REQUIRE(nav.this_leg_is_spline());
    REQUIRE_FALSE(nav.next_leg_is_spline());
    REQUIRE(nav.spline_this_leg_set());
    REQUIRE_FALSE(nav.scurve_this_leg_calculated());
    REQUIRE(nav.next_destination_ned_m().x == Approx(next.x));
    REQUIRE(nav.flags().fast_waypoint);
    almost_vec(nav.spline_origin_vel_ned_ms(), Vector3<float>{});
    almost_vec(nav.spline_destination_vel_ned_ms(), next - dest);
    REQUIRE_FALSE(nav.origin_and_destination_are_terrain_alt());
}

TEST_CASE("next is spline aims origin to next", "[wpnav][spline]") {
    WpNav nav;
    const Vector3<float> stop{};
    nav.wp_and_spline_init_m(5.0f, stop, 0, AttitudeJerkLimits{});

    const Vector3<float> dest{8.0f, 0.0f, 0.0f};
    const Vector3<float> next{12.0f, 6.0f, -1.0f};
    REQUIRE(nav.set_spline_destination_ned_m(dest, false, next, false, true, ctx_at(0, stop)));

    almost_vec(nav.spline_destination_vel_ned_ms(), next - stop);
    REQUIRE(nav.flags().fast_waypoint);
    REQUIRE(nav.this_leg_is_spline());
}

TEST_CASE("mismatched next terrain clears fast waypoint on spline", "[wpnav][spline]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -2.0f};
    nav.wp_and_spline_init_m(3.0f, stop, 0, AttitudeJerkLimits{});

    const Vector3<float> dest{4.0f, 0.0f, -2.0f};
    const Vector3<float> next{9.0f, 0.0f, -1.0f};
    REQUIRE(nav.set_spline_destination_ned_m(dest, false, next, true, false, ctx_at(0, stop)));

    REQUIRE_FALSE(nav.flags().fast_waypoint);
    almost_vec(nav.spline_destination_vel_ned_ms(), Vector3<float>{});
    REQUIRE(nav.this_leg_is_spline());
    REQUIRE_FALSE(nav.reached_wp_destination());
}

TEST_CASE("interrupted spline reinitialises from stopping point", "[wpnav][spline]") {
    WpNav nav;
    const Vector3<float> first_stop{5.0f, 5.0f, 0.0f};
    nav.wp_and_spline_init_m(3.0f, first_stop, 1000, AttitudeJerkLimits{});
    const Vector3<float> mid{20.0f, 0.0f, 0.0f};
    const Vector3<float> next{30.0f, 0.0f, 0.0f};
    REQUIRE(nav.set_spline_destination_ned_m(mid, false, next, false, false, ctx_at(1000, first_stop)));
    REQUIRE_FALSE(nav.reached_wp_destination());

    const Vector3<float> new_stop{8.0f, 1.0f, -0.5f};
    const Vector3<float> dest{40.0f, 4.0f, -1.0f};
    const Vector3<float> next2{50.0f, 4.0f, -1.0f};
    REQUIRE(nav.set_spline_destination_ned_m(dest, false, next2, false, false, ctx_at(1050, new_stop)));

    REQUIRE(nav.wp_origin_ned_m().x == Approx(new_stop.x));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));
    almost(nav.desired_speed_ne_ms(), 3.0f);
    almost_vec(nav.spline_origin_vel_ned_ms(), Vector3<float>{});
}

TEST_CASE("terrain frame flip without offset fails on spline", "[wpnav][spline]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -10.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 0, AttitudeJerkLimits{});
    const Vector3<float> dest{4.0f, 0.0f, -5.0f};
    const Vector3<float> next{8.0f, 0.0f, -5.0f};
    REQUIRE_FALSE(nav.set_spline_destination_ned_m(dest, true, next, true, false, ctx_at(0, stop)));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(stop.x));
    REQUIRE(nav.reached_wp_destination());
    REQUIRE_FALSE(nav.this_leg_is_spline());
    REQUIRE_FALSE(nav.spline_this_leg_set());
}

TEST_CASE("terrain frame flip shifts origin z on spline", "[wpnav][spline]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -10.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 0, AttitudeJerkLimits{});

    const Vector3<float> dest{4.0f, 1.0f, -3.0f};
    const Vector3<float> next{7.0f, 1.0f, -3.0f};
    auto ctx = ctx_at(0, stop);
    ctx.terrain_d_m = 7.0f;
    REQUIRE(nav.set_spline_destination_ned_m(dest, true, next, true, false, ctx));

    almost(nav.wp_origin_ned_m().z, -10.0f - 7.0f);
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));
    REQUIRE(nav.origin_and_destination_are_terrain_alt());
    almost(nav.pos_terrain_d_m(), 7.0f);
    REQUIRE(nav.this_leg_is_spline());
}

TEST_CASE("reached fast straight leg seeds origin velocity for spline", "[wpnav][spline]") {
    WpNav nav;
    const Vector3<float> stop{};
    nav.wp_and_spline_init_m(4.0f, stop, 1000, AttitudeJerkLimits{});

    const Vector3<float> mid{10.0f, 0.0f, 0.0f};
    REQUIRE(nav.set_wp_destination_ned_m(mid, false, 0.0f, ctx_at(1000, stop)));
    nav.set_fast_waypoint(true);

    AdvanceWpTargetContext adv{};
    adv.path_finished = true;
    adv.pos_estimate_ned_m = mid;
    const auto leftover = nav.advance_wp_target_along_track(adv);
    REQUIRE(leftover.ok);
    REQUIRE(nav.reached_wp_destination());
    REQUIRE_FALSE(nav.this_leg_is_spline());

    const Vector3<float> dest{20.0f, 5.0f, 0.0f};
    const Vector3<float> next{30.0f, 5.0f, 0.0f};
    REQUIRE(nav.set_spline_destination_ned_m(dest, false, next, false, false, ctx_at(1010, stop)));

    REQUIRE(nav.wp_origin_ned_m().x == Approx(mid.x));
    almost_vec(nav.spline_origin_vel_ned_ms(), mid - stop);
    almost_vec(nav.spline_destination_vel_ned_ms(), next - dest);
    REQUIRE(nav.this_leg_is_spline());
}

TEST_CASE("chained spline reuses previous destination vel", "[wpnav][spline]") {
    WpNav nav;
    const Vector3<float> stop{};
    nav.wp_and_spline_init_m(4.0f, stop, 1000, AttitudeJerkLimits{});

    const Vector3<float> first{8.0f, 0.0f, 0.0f};
    const Vector3<float> second{16.0f, 4.0f, 0.0f};
    REQUIRE(nav.set_spline_destination_ned_m(first, false, second, false, true, ctx_at(1000, stop)));
    const auto first_dest_vel = nav.spline_destination_vel_ned_ms();
    almost_vec(first_dest_vel, second - stop);
    REQUIRE(nav.flags().fast_waypoint);

    nav.set_fast_waypoint(true);
    AdvanceWpTargetContext adv2{};
    adv2.path_finished = true;
    adv2.pos_estimate_ned_m = first;
    const auto leftover = nav.advance_wp_target_along_track(adv2);
    REQUIRE(leftover.ok);
    REQUIRE(nav.reached_wp_destination());
    REQUIRE(nav.this_leg_is_spline());

    const Vector3<float> third{24.0f, 0.0f, 0.0f};
    REQUIRE(nav.set_spline_destination_ned_m(second, false, third, false, false, ctx_at(1010, stop)));

    almost_vec(nav.spline_origin_vel_ned_ms(), first_dest_vel);
    REQUIRE(nav.wp_origin_ned_m().x == Approx(first.x));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(second.x));
}