// CCP-028 slice 7: AC_Loiter pilot-accel (Rust loiter_pilot_accel.rs)

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/wpnav/loiter.hpp>

using fwcpp::math::Vector2;
using fwcpp::math::Vector3;
using fwcpp::wpnav::InitTargetContext;
using fwcpp::wpnav::Loiter;
using fwcpp::wpnav::LoiterOption;
using fwcpp::wpnav::PilotAccelContext;
using fwcpp::wpnav::UpdateLoiterContext;

namespace {
void almost(float a, float b) { REQUIRE(std::abs(a - b) <= 1e-5f); }
void almost_vec(const Vector2<float>& got, const Vector2<float>& expected) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
}
Vector2<float> lean_xy(float roll, float pitch, float yaw) {
    const float sin_roll = std::sin(roll);
    const float cos_roll = std::cos(roll);
    const float sin_pitch = std::sin(pitch);
    const float cos_pitch = std::cos(pitch);
    const float sin_yaw = std::sin(yaw);
    const float cos_yaw = std::cos(yaw);
    const float divisor = std::max(cos_roll * cos_pitch, 0.1f);
    return Vector2<float>{
        9.80665f * (-cos_yaw * sin_pitch * cos_roll - sin_yaw * sin_roll) / divisor,
        9.80665f * (-sin_yaw * sin_pitch * cos_roll + cos_yaw * sin_roll) / divisor};
}
}  // namespace

TEST_CASE("zero stick does not reset brake timer", "[loiter][pilot]") {
    Loiter loiter;
    loiter.set_options(0);
    PilotAccelContext ctx;
    ctx.now_ms = 1500;
    loiter.set_pilot_desired_acceleration_rad(0, 0, ctx);
    REQUIRE(loiter.brake_timer_ms() == 0);
    almost_vec(loiter.get_pilot_desired_acceleration_ne_mss(), Vector2<float>{});
}

TEST_CASE("lean writes accel and resets brake timer", "[loiter][pilot]") {
    Loiter loiter;
    loiter.set_options(0);
    PilotAccelContext ctx;
    ctx.now_ms = 250;
    ctx.yaw_rad = 0;
    loiter.set_pilot_desired_acceleration_rad(0.2f, -0.1f, ctx);
    const Vector2<float> desired = lean_xy(0.2f, -0.1f, 0.0f);
    almost_vec(loiter.desired_accel_ne_mss(), desired);
    REQUIRE(loiter.brake_timer_ms() == 250);
}

TEST_CASE("coordinated turn adds yaw rate feed forward", "[loiter][pilot]") {
    Loiter loiter;
    REQUIRE(loiter.loiter_option_is_set(LoiterOption::CoordinatedTurnEnabled));
    PilotAccelContext ctx;
    ctx.vel_desired_ned_ms = Vector3<float>{2, -1, 0};
    ctx.target_ang_vel_z_rads = 0.5f;
    loiter.set_pilot_desired_acceleration_rad(0, 0, ctx);
    const Vector2<float> turn{-(-1.0f) * 0.5f, 2.0f * 0.5f};
    almost_vec(loiter.desired_accel_ne_mss(), turn);
}
