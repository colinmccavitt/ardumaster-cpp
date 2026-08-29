// CCP-028: wp_and_spline_init_m parity (Rust tests/wp_and_spline_init.rs)
// plus horizontal distance/bearing helpers.

#include <cmath>
#include <numbers>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/wpnav/wpnav.hpp>

using Catch::Approx;
using fwcpp::math::Vector3;
using fwcpp::wpnav::AttitudeJerkLimits;
using fwcpp::wpnav::WpNav;
using fwcpp::wpnav::kGravityMss;
using fwcpp::wpnav::kWpAccZDefault;
using fwcpp::wpnav::kWpJerkDefault;
using fwcpp::wpnav::kWpRadiusMDefault;
using fwcpp::wpnav::kWpRadiusMMin;
using fwcpp::wpnav::kWpSpdDefault;
using fwcpp::wpnav::kWpSpdDownDefault;
using fwcpp::wpnav::kWpSpdMin;
using fwcpp::wpnav::kWpSpdUpDefault;
using fwcpp::wpnav::kWpnavAccelerationMss;
using fwcpp::wpnav::kWpnavActiveTimeoutMs;

namespace {

void almost(float a, float b) {
    REQUIRE(std::abs(a - b) <= 1e-5f);
}

}  // namespace

TEST_CASE("constructor records groupinfo defaults", "[wpnav][init]") {
    WpNav nav;
    almost(nav.default_speed_ne_ms(), kWpSpdDefault);
    almost(nav.default_speed_up_ms(), kWpSpdUpDefault);
    almost(nav.default_speed_down_ms(), kWpSpdDownDefault);
    almost(nav.wp_acceleration_mss(), kWpnavAccelerationMss);
    almost(nav.accel_d_mss(), kWpAccZDefault);
    almost(nav.wp_radius_m(), kWpRadiusMDefault);
    REQUIRE_FALSE(nav.flags().reached_destination);
    REQUIRE_FALSE(nav.flags().fast_waypoint);
    REQUIRE_FALSE(nav.scurve_legs_inited());
}

TEST_CASE("init seats origin and destination on the stopping point", "[wpnav][init]") {
    WpNav nav;
    const Vector3<float> stop{12.0f, -3.5f, 4.0f};
    nav.wp_and_spline_init_m(5.0f, stop, 1000, AttitudeJerkLimits{});
    REQUIRE(nav.wp_destination_ned_m().x == Approx(stop.x));
    REQUIRE(nav.wp_origin_ned_m().x == Approx(stop.x));
    almost(nav.desired_speed_ne_ms(), 5.0f);
    REQUIRE_FALSE(nav.check_wp_speed_change());
    REQUIRE(nav.flags().reached_destination);
    REQUIRE_FALSE(nav.flags().fast_waypoint);
    REQUIRE_FALSE(nav.origin_and_destination_are_terrain_alt());
    REQUIRE_FALSE(nav.this_leg_is_spline());
    REQUIRE_FALSE(nav.paused());
    almost(nav.track_dt_scalar(), 1.0f);
    almost(nav.offset_vel_ms(), 5.0f);
    almost(nav.offset_accel_mss(), 0.0f);
    REQUIRE(nav.scurve_legs_inited());
    REQUIRE(nav.pos_control_stopping_point_inited());
}

TEST_CASE("zero speed uses wp_spd and watches for changes", "[wpnav][init]") {
    WpNav nav;
    nav.wp_and_spline_init_m(0.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    almost(nav.desired_speed_ne_ms(), kWpSpdDefault);
    REQUIRE(nav.check_wp_speed_change());
    almost(nav.offset_vel_ms(), kWpSpdDefault);
}

TEST_CASE("clamps radius and speed floors", "[wpnav][init]") {
    WpNav nav;
    nav.set_wp_radius_m(0.01f);
    nav.set_wp_speed_ms(0.0f);
    nav.wp_and_spline_init_m(0.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    almost(nav.wp_radius_m(), kWpRadiusMMin);
    almost(nav.default_speed_ne_ms(), kWpSpdMin);
    almost(nav.desired_speed_ne_ms(), kWpSpdMin);
}

TEST_CASE("records pos control speed and accel", "[wpnav][init]") {
    WpNav nav;
    nav.wp_and_spline_init_m(3.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    const auto lim = nav.pos_speed_accel();
    almost(lim.ne_speed_ms, 3.0f);
    almost(lim.ne_accel_mss, kWpnavAccelerationMss);
    almost(lim.speed_down_ms, kWpSpdDownDefault);
    almost(lim.speed_up_ms, kWpSpdUpDefault);
    almost(lim.accel_d_mss, kWpAccZDefault);
}

TEST_CASE("unset jerk falls back to horizontal accel", "[wpnav][init]") {
    WpNav nav;
    nav.set_wp_jerk_msss(0.0f);
    nav.wp_and_spline_init_m(0.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    almost(nav.scurve_jerk_max_msss(), kWpnavAccelerationMss);
}

TEST_CASE("zero attitude rates use wp jerk and half snap", "[wpnav][init]") {
    WpNav nav;
    nav.wp_and_spline_init_m(0.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    almost(nav.scurve_jerk_max_msss(), kWpJerkDefault);
    const float expected_snap = (kWpJerkDefault * std::numbers::pi_v<float>) / (2.0f * 0.1f) * 0.5f;
    almost(nav.scurve_snap_max_mssss(), expected_snap);
}

TEST_CASE("attitude rate caps jerk below wp jerk", "[wpnav][init]") {
    WpNav nav;
    AttitudeJerkLimits attitude{};
    attitude.ang_vel_roll_max_rads = 0.05f;
    attitude.ang_vel_pitch_max_rads = 0.08f;
    attitude.input_tc = 0.2f;
    nav.wp_and_spline_init_m(0.0f, Vector3<float>{}, 0, attitude);
    const float jerk = 0.05f * kGravityMss;
    almost(nav.scurve_jerk_max_msss(), jerk);
    const float snap = (jerk * std::numbers::pi_v<float>) / (2.0f * 0.2f) * 0.5f;
    almost(nav.scurve_snap_max_mssss(), snap);
}

TEST_CASE("is active for two hundred milliseconds after init", "[wpnav][init]") {
    WpNav nav;
    nav.wp_and_spline_init_m(0.0f, Vector3<float>{}, 1000, AttitudeJerkLimits{});
    REQUIRE(nav.is_active(1000));
    REQUIRE(nav.is_active(1000 + kWpnavActiveTimeoutMs - 1));
    REQUIRE_FALSE(nav.is_active(1000 + kWpnavActiveTimeoutMs));
}

TEST_CASE("zero accel param falls back to wpnav acceleration", "[wpnav][init]") {
    WpNav nav;
    nav.set_wp_accel_mss(0.0f);
    almost(nav.wp_acceleration_mss(), kWpnavAccelerationMss);
    nav.wp_and_spline_init_m(1.0f, Vector3<float>{}, 0, AttitudeJerkLimits{});
    almost(nav.pos_speed_accel().ne_accel_mss, kWpnavAccelerationMss);
}

TEST_CASE("horizontal distance ignores z", "[wpnav][distance]") {
    WpNav nav;
    const Vector3<float> dest{10.0f, 0.0f, 0.0f};
    nav.wp_and_spline_init_m(1.0f, dest, 0, AttitudeJerkLimits{});
    const Vector3<float> pos{0.0f, 0.0f, 99.0f};
    almost(nav.get_wp_distance_to_destination_m(pos), 10.0f);
    almost(nav.get_wp_distance_to_destination_cm(pos), 1000.0f);
}

TEST_CASE("bearing east from origin is pi over two", "[wpnav][bearing]") {
    WpNav nav;
    const Vector3<float> dest{0.0f, 5.0f, 0.0f};
    nav.wp_and_spline_init_m(1.0f, dest, 0, AttitudeJerkLimits{});
    const Vector3<float> pos{};
    almost(nav.get_wp_bearing_to_destination_rad(pos), std::numbers::pi_v<float> / 2.0f);
}