// CCP-028 slice 5: AC_Circle init / set_center / update (Rust circle_init.rs + set_center)

#include <cmath>
#include <numbers>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/wpnav/circle.hpp>

using fwcpp::Location;
using fwcpp::math::Vector3;
using fwcpp::wpnav::Circle;
using fwcpp::wpnav::CircleOption;
using fwcpp::wpnav::GetVectorNedContext;
using fwcpp::wpnav::InitCircleContext;
using fwcpp::wpnav::UpdateCircleContext;
using fwcpp::wpnav::kCircleDefaultOptions;
using fwcpp::wpnav::kCircleRadiusMDefault;
using fwcpp::wpnav::kCircleRateDefault;
using fwcpp::wpnav::kCircleActiveTimeoutMs;
using fwcpp::wpnav::kCircleAngularAccelMin;

namespace {

void almost(float a, float b) { REQUIRE(std::abs(a - b) <= 1e-5f); }

void almost_vec(const Vector3<float>& got, const Vector3<float>& expected) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
    almost(got.z, expected.z);
}

InitCircleContext init_ctx(const Vector3<float>& pos, float yaw_rad) {
    return InitCircleContext{.yaw_rad = yaw_rad,
                             .cos_yaw = std::cos(yaw_rad),
                             .sin_yaw = std::sin(yaw_rad),
                             .pos_desired_ned_m = pos};
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

TEST_CASE("constructor records groupinfo defaults", "[circle][init]") {
    const Circle circle;
    almost(circle.radius_parm_m(), kCircleRadiusMDefault);
    almost(circle.radius_m(), 0.0f);
    almost(circle.get_radius_m(), kCircleRadiusMDefault);
    almost(circle.get_rate_degs(), kCircleRateDefault);
    almost(circle.rotation_rate_max_rads(), fwcpp::math::radians(kCircleRateDefault));
    REQUIRE(circle.option_is_set(CircleOption::ManualControl));
    REQUIRE(kCircleDefaultOptions == 1);
    REQUIRE(circle.pilot_control_enabled());
    REQUIRE_FALSE(circle.roi_at_center());
    almost(circle.angle_rad(), 0.0f);
    almost(circle.angular_vel_rads(), 0.0f);
    almost_vec(circle.center_ned_m(), Vector3<float>{});
    REQUIRE_FALSE(circle.center_is_terrain_alt());
}

TEST_CASE("init_ned_m panorama uses yaw and records stopping point", "[circle][init]") {
    Circle circle;
    const auto leftover = circle.init_ned_m(Vector3<float>{3.0f, -1.0f, -8.0f}, true, 15.0f,
                                            init_ctx({1.0f, 2.0f, -4.0f}, 0.4f));
    REQUIRE(leftover.need_ne_init_controller_stopping_point);
    REQUIRE(leftover.need_d_init_controller_stopping_point);
    almost_vec(circle.center_ned_m(), {3.0f, -1.0f, -8.0f});
    REQUIRE(circle.center_is_terrain_alt());
    almost(circle.rotation_rate_max_rads(), fwcpp::math::radians(15.0f));
    almost(circle.angular_vel_rads(), 0.0f);
    almost(circle.angular_vel_max_rads(), fwcpp::math::radians(15.0f));
    almost(circle.angular_accel_radss(),
           std::max(std::abs(fwcpp::math::radians(15.0f)), fwcpp::math::radians(kCircleAngularAccelMin)));
    almost(circle.angle_rad(), 0.4f);
    almost(circle.get_angle_total_rad(), 0.0f);
}

TEST_CASE("init projects center along heading", "[circle][init]") {
    Circle circle;
    const Vector3<float> stop{5.0f, 3.0f, -2.0f};
    const auto leftover = circle.init(init_ctx(stop, 0.0f));
    REQUIRE(leftover.need_ne_init_controller_stopping_point);
    almost(circle.radius_m(), kCircleRadiusMDefault);
    almost_vec(circle.center_ned_m(), {5.0f + kCircleRadiusMDefault, 3.0f, -2.0f});
    REQUIRE_FALSE(circle.center_is_terrain_alt());
    almost(circle.angle_rad(), fwcpp::math::wrap_PI(0.0f - std::numbers::pi_v<float>));
}

TEST_CASE("init at center keeps stopping point", "[circle][init]") {
    Circle circle;
    circle.set_options(static_cast<std::int16_t>(CircleOption::InitAtCenter));
    const Vector3<float> stop{-2.0f, 7.0f, -3.0f};
    circle.init(init_ctx(stop, 0.8f));
    almost_vec(circle.center_ned_m(), stop);
    almost(circle.angle_rad(), fwcpp::math::wrap_PI(0.8f - std::numbers::pi_v<float>));
}

TEST_CASE("update records ne and climb leftovers", "[circle][update]") {
    Circle circle;
    const Vector3<float> stop{5.0f, 3.0f, -2.0f};
    circle.init(init_ctx(stop, 0.0f));
    const auto leftover = circle.update_ms(
        0.4f, UpdateCircleContext{.now_ms = 1000,
                                  .dt_s = 0.01f,
                                  .pos_desired_ned_m = stop,
                                  .pos_desired_u_m = 2.0f});
    REQUIRE(leftover.ok);
    REQUIRE(leftover.need_input_pos_vel_accel_ne);
    REQUIRE_FALSE(leftover.need_input_pos_vel_accel_d);
    REQUIRE(leftover.need_d_set_pos_target_from_climb_rate);
    REQUIRE(leftover.need_ne_update_controller);
    almost(leftover.climb_rate_ms, 0.4f);
    almost(static_cast<float>(circle.last_update_ms()), 1000.0f);
    REQUIRE(circle.is_active(1100));
    REQUIRE_FALSE(circle.is_active(1000 + kCircleActiveTimeoutMs));
    almost(leftover.target_ned_m.z, -2.0f);
}

TEST_CASE("update terrain missing fails after angle advance", "[circle][update]") {
    Circle circle;
    circle.set_radius_m(8.0f);
    circle.init_ned_m({0.0f, 0.0f, -10.0f}, true, kCircleRateDefault,
                      init_ctx({8.0f, 0.0f, -10.0f}, 0.0f));
    const auto leftover = circle.update_ms(0.0f, UpdateCircleContext{});
    REQUIRE_FALSE(leftover.ok);
    almost(static_cast<float>(circle.last_update_ms()), 0.0f);
}

TEST_CASE("set_center origin frame seats ned", "[circle][set_center]") {
    Circle circle;
    const Location origin = origin_loc();
    Location dest(origin.lat + 1000, origin.lng, 500, Location::AltFrame::ABOVE_ORIGIN);
    const auto leftover = circle.set_center(dest, vec_ctx_origin_alt(), {9.0f, 9.0f, -1.0f});
    REQUIRE_FALSE(leftover.need_nav_error_log);
    REQUIRE_FALSE(leftover.used_pos_estimate_fallback);
    REQUIRE_FALSE(circle.center_is_terrain_alt());
    almost(circle.center_ned_m().y, 0.0f);
    almost(circle.center_ned_m().z, -5.0f);
    REQUIRE(circle.center_ned_m().x > 10.0f);
}

TEST_CASE("set_center terrain frame marks terrain alt", "[circle][set_center]") {
    Circle circle;
    const Location origin = origin_loc();
    Location terr(origin.lat, origin.lng, 800, Location::AltFrame::ABOVE_TERRAIN);
    const auto leftover = circle.set_center(terr, vec_ctx_origin_alt(), Vector3<float>{});
    REQUIRE_FALSE(leftover.need_nav_error_log);
    REQUIRE(circle.center_is_terrain_alt());
    almost_vec(circle.center_ned_m(), {0.0f, 0.0f, -8.0f});
}

TEST_CASE("set_center unset origin falls back to estimate", "[circle][set_center]") {
    Circle circle;
    const Location origin = origin_loc();
    Location dest(origin.lat + 1000, origin.lng, 500, Location::AltFrame::ABOVE_ORIGIN);
    const Vector3<float> estimate{1.5f, -2.25f, -3.0f};
    const auto leftover = circle.set_center(dest, GetVectorNedContext{}, estimate);
    REQUIRE(leftover.need_nav_error_log);
    REQUIRE(leftover.used_pos_estimate_fallback);
    almost_vec(circle.center_ned_m(), estimate);
}
