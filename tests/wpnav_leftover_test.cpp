// CCP-028 slice 7: wpnav leftovers (Rust wpnav_leftover.rs)

#include <cmath>
#include <optional>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/wpnav/wpnav.hpp>

using fwcpp::Location;
using fwcpp::math::Vector2;
using fwcpp::math::Vector3;
using fwcpp::wpnav::AttitudeJerkLimits;
using fwcpp::wpnav::GetTerrainContext;
using fwcpp::wpnav::GetVectorNedContext;
using fwcpp::wpnav::SetWpDestinationContext;
using fwcpp::wpnav::TerrainSource;
using fwcpp::wpnav::WpNav;

namespace {
void almost(float a, float b) { REQUIRE(std::abs(a - b) <= 1e-5f); }
void almost_vec(const Vector3<float>& got, const Vector3<float>& expected) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
    almost(got.z, expected.z);
}
SetWpDestinationContext ctx_at(std::uint32_t now_ms, const Vector3<float>& stop) {
    return SetWpDestinationContext{.now_ms = now_ms,
                                   .attitude = AttitudeJerkLimits{},
                                   .stopping_point_ned_m = stop,
                                   .terrain_d_m = std::nullopt};
}
Location origin_loc() { return Location(35000000, -110000000, 0, Location::AltFrame::ABSOLUTE); }
GetVectorNedContext vec_ctx_origin_alt() {
    GetVectorNedContext ctx;
    ctx.origin = origin_loc();
    ctx.alt.origin_is_set = true;
    ctx.alt.ekf_origin = Location(0, 0, 0, Location::AltFrame::ABSOLUTE);
    return ctx;
}
}  // namespace

TEST_CASE("force_stop clears fast and records scurve leftover", "[wpnav][leftover]") {
    WpNav nav;
    const Vector3<float> stop{};
    nav.wp_and_spline_init_m(4.0f, stop, 0, AttitudeJerkLimits{});
    REQUIRE(nav.set_wp_destination_ned_m(Vector3<float>{8, 0, 0}, false, 0.0f, ctx_at(0, stop)));
    REQUIRE_FALSE(nav.force_stop_at_next_wp());
    REQUIRE(nav.set_wp_destination_next_ned_m(Vector3<float>{16, 0, 0}, false, 0.0f));
    REQUIRE(nav.flags().fast_waypoint);
    REQUIRE(nav.scurve_next_leg_calculated());
    REQUIRE(nav.force_stop_at_next_wp());
    REQUIRE_FALSE(nav.flags().fast_waypoint);
    REQUIRE(nav.need_this_leg_dest_speed_max_zero());
    REQUIRE(nav.need_next_scurve_init());
    REQUIRE_FALSE(nav.scurve_next_leg_calculated());
    almost_vec(nav.next_destination_ned_m(), Vector3<float>{16, 0, 0});
}

TEST_CASE("stopping point wrappers convert poscontrol leftover", "[wpnav][leftover]") {
    const Vector3<float> leftover{3, -1.5f, 0.25f};
    almost_vec(WpNav::get_wp_stopping_point_ned_m(leftover), leftover);
    const Vector2<float> ne = WpNav::get_wp_stopping_point_ne_m(leftover);
    almost(ne.x, 3.0f);
    almost(ne.y, -1.5f);
    const Vector2<float> ne_cm = WpNav::get_wp_stopping_point_ne_cm(leftover);
    almost(ne_cm.x, 300.0f);
    almost(ne_cm.y, -150.0f);
    const Vector3<float> neu_cm = WpNav::get_wp_stopping_point_neu_cm(leftover);
    almost(neu_cm.x, 300.0f);
    almost(neu_cm.y, -150.0f);
    almost(neu_cm.z, -25.0f);
}

TEST_CASE("terrain source prefers rangefinder then database", "[wpnav][leftover]") {
    WpNav nav;
    REQUIRE(nav.get_terrain_source(false) == TerrainSource::Unavailable);
    REQUIRE(nav.get_terrain_source(true) == TerrainSource::FromTerrainDatabase);
    REQUIRE(nav.rangefinder_used());
    REQUIRE_FALSE(nav.rangefinder_used_and_healthy());
    nav.set_rangefinder_terrain_u_cm(true, true, 250.0f);
    REQUIRE(nav.get_terrain_source(true) == TerrainSource::FromRangefinder);
    almost(*nav.get_terrain_u_m(GetTerrainContext{}), 2.5f);
    almost(*nav.get_terrain_d_m(GetTerrainContext{}), -2.5f);
    nav.set_terrain_margin_m(0.01f);
    almost(nav.terrain_margin_m(), 0.1f);
}

TEST_CASE("corner accel defaults to twice horizontal", "[wpnav][leftover]") {
    WpNav nav;
    almost(nav.corner_acceleration_mss(), 2.0f * nav.wp_acceleration_mss());
    nav.set_wp_accel_c_mss(3.5f);
    almost(nav.corner_acceleration_mss(), 3.5f);
}

TEST_CASE("get_vector_ned_m origin and terrain frames", "[wpnav][leftover]") {
    const Location origin = origin_loc();
    const Location dest(origin.lat + 1000, origin.lng, 500, Location::AltFrame::ABOVE_ORIGIN);
    const auto ctx = vec_ctx_origin_alt();
    const auto ned = WpNav::get_vector_ned_m(dest, ctx);
    REQUIRE(ned.has_value());
    REQUIRE_FALSE(ned->second);
    almost(ned->first.y, 0.0f);
    almost(ned->first.z, -5.0f);
    REQUIRE(ned->first.x > 10.0f);
    const Location terr(origin.lat, origin.lng, 800, Location::AltFrame::ABOVE_TERRAIN);
    const auto ned_t = WpNav::get_vector_ned_m(terr, ctx);
    REQUIRE(ned_t.has_value());
    REQUIRE(ned_t->second);
    almost_vec(ned_t->first, Vector3<float>{0, 0, -8});
    REQUIRE_FALSE(WpNav::get_vector_ned_m(dest, GetVectorNedContext{}).has_value());
}
