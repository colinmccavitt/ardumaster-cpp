// CCP-028 slice 2: set_wp_destination parity (Rust tests/set_wp_destination.rs)

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
using fwcpp::wpnav::WpNav;
using fwcpp::wpnav::kWpnavActiveTimeoutMs;

namespace {

void almost(float a, float b) { REQUIRE(std::abs(a - b) <= 1e-5f); }

SetWpDestinationContext ctx_at(std::uint32_t now_ms, const Vector3<float>& stopping_point_ned_m) {
    return SetWpDestinationContext{.now_ms = now_ms,
                                   .attitude = AttitudeJerkLimits{},
                                   .stopping_point_ned_m = stopping_point_ned_m,
                                   .terrain_d_m = std::nullopt};
}

}  // namespace

TEST_CASE("set destination uses previous dest as origin", "[wpnav][dest]") {
    WpNav nav;
    const Vector3<float> stop{1.0f, 2.0f, 3.0f};
    nav.wp_and_spline_init_m(4.0f, stop, 1000, AttitudeJerkLimits{});

    const Vector3<float> dest{10.0f, -4.0f, 1.5f};
    REQUIRE(nav.set_wp_destination_ned_m(dest, false, 0.25f, ctx_at(1000, stop)));

    REQUIRE(nav.wp_origin_ned_m().x == Approx(stop.x));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));
    REQUIRE_FALSE(nav.reached_wp_destination());
    REQUIRE_FALSE(nav.flags().fast_waypoint);
    REQUIRE_FALSE(nav.this_leg_is_spline());
    REQUIRE_FALSE(nav.next_leg_is_spline());
    REQUIRE(nav.next_destination_ned_m().x == Approx(0.0f));
    REQUIRE(nav.scurve_this_leg_calculated());
    almost(nav.last_arc_rad(), 0.25f);
    REQUIRE_FALSE(nav.origin_and_destination_are_terrain_alt());
}

TEST_CASE("neu cm wrapper converts and getters round trip", "[wpnav][dest]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -2.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 500, AttitudeJerkLimits{});

    const Vector3<float> neu_cm{1200.0f, -300.0f, 400.0f};
    REQUIRE(nav.set_wp_destination_neu_cm(neu_cm, false, ctx_at(500, stop)));

    const auto dest = nav.wp_destination_ned_m();
    almost(dest.x, 12.0f);
    almost(dest.y, -3.0f);
    almost(dest.z, -4.0f);

    const auto got = nav.wp_destination_neu_cm();
    almost(got.x, 1200.0f);
    almost(got.y, -300.0f);
    almost(got.z, 400.0f);

    const auto origin_cm = nav.wp_origin_neu_cm();
    almost(origin_cm.x, 0.0f);
    almost(origin_cm.y, 0.0f);
    almost(origin_cm.z, 200.0f);
}

TEST_CASE("interrupted leg reinitialises from stopping point", "[wpnav][dest]") {
    WpNav nav;
    const Vector3<float> first_stop{5.0f, 5.0f, 0.0f};
    nav.wp_and_spline_init_m(3.0f, first_stop, 1000, AttitudeJerkLimits{});
    const Vector3<float> mid{20.0f, 0.0f, 0.0f};
    REQUIRE(nav.set_wp_destination_ned_m(mid, false, 0.0f, ctx_at(1000, first_stop)));
    REQUIRE_FALSE(nav.reached_wp_destination());

    const Vector3<float> new_stop{8.0f, 1.0f, -0.5f};
    const Vector3<float> dest{30.0f, 4.0f, -1.0f};
    REQUIRE(nav.set_wp_destination_ned_m(dest, false, 0.0f, ctx_at(1050, new_stop)));

    REQUIRE(nav.wp_origin_ned_m().x == Approx(new_stop.x));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));
    almost(nav.desired_speed_ne_ms(), 3.0f);
}

TEST_CASE("inactive navigator also reinitialises", "[wpnav][dest]") {
    WpNav nav;
    const Vector3<float> first_stop{1.0f, 0.0f, 0.0f};
    nav.wp_and_spline_init_m(2.0f, first_stop, 100, AttitudeJerkLimits{});
    REQUIRE(nav.reached_wp_destination());

    const std::uint32_t later = 100 + kWpnavActiveTimeoutMs + 1;
    const Vector3<float> new_stop{2.0f, 2.0f, 0.0f};
    const Vector3<float> dest{9.0f, 0.0f, 0.0f};
    REQUIRE(nav.set_wp_destination_ned_m(dest, false, 0.0f, ctx_at(later, new_stop)));
    REQUIRE(nav.wp_origin_ned_m().x == Approx(new_stop.x));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));
}

TEST_CASE("terrain frame flip without offset fails", "[wpnav][dest]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -10.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 0, AttitudeJerkLimits{});
    const Vector3<float> dest{4.0f, 0.0f, -5.0f};
    REQUIRE_FALSE(nav.set_wp_destination_ned_m(dest, true, 0.0f, ctx_at(0, stop)));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(stop.x));
    REQUIRE(nav.reached_wp_destination());
}

TEST_CASE("terrain frame flip shifts origin z", "[wpnav][dest]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -10.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 0, AttitudeJerkLimits{});

    const Vector3<float> dest{4.0f, 1.0f, -3.0f};
    auto ctx = ctx_at(0, stop);
    ctx.terrain_d_m = 7.0f;
    REQUIRE(nav.set_wp_destination_ned_m(dest, true, 0.0f, ctx));

    almost(nav.wp_origin_ned_m().z, -10.0f - 7.0f);
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));
    REQUIRE(nav.origin_and_destination_are_terrain_alt());
    almost(nav.pos_terrain_d_m(), 7.0f);
}

TEST_CASE("terrain frame flip back to origin clears pos terrain", "[wpnav][dest]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -4.0f};
    nav.wp_and_spline_init_m(0.0f, stop, 0, AttitudeJerkLimits{});

    auto ctx = ctx_at(0, stop);
    ctx.terrain_d_m = 2.0f;
    REQUIRE(nav.set_wp_destination_ned_m(Vector3<float>{1.0f, 0.0f, -1.0f}, true, 0.0f, ctx));

    ctx.now_ms = 10;
    ctx.stopping_point_ned_m = Vector3<float>{0.5f, 0.0f, -3.0f};
    ctx.terrain_d_m = std::nullopt;
    const Vector3<float> dest{8.0f, 0.0f, -2.0f};
    REQUIRE(nav.set_wp_destination_ned_m(dest, false, 0.0f, ctx));
    REQUIRE_FALSE(nav.origin_and_destination_are_terrain_alt());
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));
}
