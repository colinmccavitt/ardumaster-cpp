// CCP-028 slice 6: set_wp_destination_next parity (Rust wpnav_leftover next-dest tests)

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

namespace {

void almost(float a, float b) { REQUIRE(std::abs(a - b) <= 1e-5f); }

SetWpDestinationContext ctx_at(std::uint32_t now_ms, const Vector3<float>& stopping_point_ned_m) {
    return SetWpDestinationContext{.now_ms = now_ms,
                                   .attitude = AttitudeJerkLimits{},
                                   .stopping_point_ned_m = stopping_point_ned_m,
                                   .terrain_d_m = std::nullopt};
}

}  // namespace

TEST_CASE("next dest preloads fast waypoint and scurve leftover", "[wpnav][next]") {
    WpNav nav;
    const Vector3<float> stop{1.0f, 2.0f, 0.0f};
    nav.wp_and_spline_init_m(5.0f, stop, 0, AttitudeJerkLimits{});

    const Vector3<float> dest{11.0f, 2.0f, 0.0f};
    REQUIRE(nav.set_wp_destination_ned_m(dest, false, 0.0f, ctx_at(0, stop)));
    REQUIRE_FALSE(nav.flags().fast_waypoint);
    REQUIRE_FALSE(nav.scurve_next_leg_calculated());

    const Vector3<float> next{20.0f, 8.0f, 1.0f};
    REQUIRE(nav.set_wp_destination_next_ned_m(next, false, 0.4f));

    REQUIRE(nav.flags().fast_waypoint);
    REQUIRE(nav.scurve_next_leg_calculated());
    almost(nav.last_next_arc_rad(), 0.4f);
    REQUIRE_FALSE(nav.next_leg_is_spline());
    REQUIRE_FALSE(nav.this_leg_is_spline());
    REQUIRE_FALSE(nav.need_this_leg_dest_speed_max());
    REQUIRE(nav.next_destination_ned_m().x == Approx(next.x));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));

    const auto limits = nav.update_track_with_speed_accel_limits();
    REQUIRE(limits.need_this_scurve_speed_max);
    REQUIRE_FALSE(limits.need_this_spline_speed_accel);
    REQUIRE(limits.need_next_scurve_speed_max);
    REQUIRE_FALSE(limits.need_next_spline_speed_accel);
}

TEST_CASE("next dest after spline records speed handoff", "[wpnav][next]") {
    WpNav nav;
    const Vector3<float> stop{};
    nav.wp_and_spline_init_m(4.0f, stop, 1000, AttitudeJerkLimits{});

    const Vector3<float> dest{10.0f, 0.0f, 0.0f};
    const Vector3<float> look{18.0f, 6.0f, 0.0f};
    REQUIRE(nav.set_spline_destination_ned_m(dest, false, look, false, true, ctx_at(1000, stop)));
    REQUIRE(nav.this_leg_is_spline());

    const Vector3<float> next{28.0f, 6.0f, 0.0f};
    REQUIRE(nav.set_wp_destination_next_ned_m(next, false, 0.0f));

    REQUIRE(nav.need_this_leg_dest_speed_max());
    REQUIRE(nav.scurve_next_leg_calculated());
    REQUIRE_FALSE(nav.next_leg_is_spline());
    REQUIRE(nav.flags().fast_waypoint);
    REQUIRE(nav.next_destination_ned_m().x == Approx(next.x));
}

TEST_CASE("mismatched next terrain skips without changing state", "[wpnav][next]") {
    WpNav nav;
    const Vector3<float> stop{0.0f, 0.0f, -2.0f};
    nav.wp_and_spline_init_m(3.0f, stop, 0, AttitudeJerkLimits{});
    const Vector3<float> dest{6.0f, 0.0f, -2.0f};
    REQUIRE(nav.set_wp_destination_ned_m(dest, false, 0.0f, ctx_at(0, stop)));

    REQUIRE(nav.set_wp_destination_next_ned_m(Vector3<float>{12.0f, 0.0f, -1.0f}, true, 0.2f));

    REQUIRE_FALSE(nav.flags().fast_waypoint);
    REQUIRE_FALSE(nav.scurve_next_leg_calculated());
    almost(nav.last_next_arc_rad(), 0.0f);
    REQUIRE(nav.next_destination_ned_m().x == Approx(0.0f));
    REQUIRE(nav.wp_destination_ned_m().x == Approx(dest.x));
}