// CCP-028 slice 7: AC_Circle set_center / closest-point (Rust circle_leftover.rs)

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/wpnav/circle.hpp>

using fwcpp::Location;
using fwcpp::math::Vector3;
using fwcpp::wpnav::Circle;
using fwcpp::wpnav::GetVectorNedContext;
using fwcpp::wpnav::kCircleRadiusMDefault;
using fwcpp::wpnav::kCircleRadiusMaxM;

namespace {
void almost(float a, float b) { REQUIRE(std::abs(a - b) <= 1e-5f); }
void almost_vec(const Vector3<float>& got, const Vector3<float>& expected) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
    almost(got.z, expected.z);
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

TEST_CASE("set_center origin frame seats ned", "[circle][leftover]") {
    Circle circle;
    const Location origin = origin_loc();
    const Location dest(origin.lat + 1000, origin.lng, 500, Location::AltFrame::ABOVE_ORIGIN);
    const auto leftover = circle.set_center(dest, vec_ctx_origin_alt(), Vector3<float>{9, 9, -1});
    REQUIRE_FALSE(leftover.need_nav_error_log);
    REQUIRE_FALSE(leftover.used_pos_estimate_fallback);
    REQUIRE_FALSE(circle.center_is_terrain_alt());
    almost(circle.center_ned_m().y, 0.0f);
    almost(circle.center_ned_m().z, -5.0f);
    REQUIRE(circle.center_ned_m().x > 10.0f);
}

TEST_CASE("closest point zero radius returns center", "[circle][leftover]") {
    Circle circle;
    circle.set_center_ned_m(Vector3<float>{4, -1, -6}, false);
    circle.set_radius_m(0.0f);
    const auto closest = circle.get_closest_point_on_circle_ned_m(Vector3<float>{10, 2, -6}, 1.0f, 0.0f);
    almost_vec(closest.point_ned_m, Vector3<float>{4, -1, -6});
    almost(closest.dist_to_edge_m, 0.0f);
}

TEST_CASE("radius cm wrappers and center neu", "[circle][leftover]") {
    Circle circle;
    almost(circle.get_radius_cm(), kCircleRadiusMDefault * 100.0f);
    circle.set_radius_cm(250.0f);
    almost(circle.radius_m(), 2.5f);
    circle.set_radius_cm(500000.0f);
    almost(circle.radius_m(), kCircleRadiusMaxM);
    circle.set_center_ned_m(Vector3<float>{1, 2, -3}, true);
    almost_vec(circle.get_center_neu_cm(), Vector3<float>{100, 200, 300});
}
