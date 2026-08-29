// CCP-028 slice 4: AC_Loiter init / update leftover (Rust tests/loiter_init.rs)

#include <cmath>
#include <optional>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/wpnav/loiter.hpp>

using fwcpp::math::Vector2;
using fwcpp::wpnav::InitTargetContext;
using fwcpp::wpnav::Loiter;
using fwcpp::wpnav::LoiterOption;
using fwcpp::wpnav::UpdateLoiterContext;
using fwcpp::wpnav::kLoiterAccelMaxDefaultMss;
using fwcpp::wpnav::kLoiterBrakeAccelDefaultMss;
using fwcpp::wpnav::kLoiterBrakeJerkDefaultMsss;
using fwcpp::wpnav::kLoiterBrakeStartDelayDefaultS;
using fwcpp::wpnav::kLoiterPosCorrectionMaxM;
using fwcpp::wpnav::kLoiterSpeedDefaultMs;
using fwcpp::wpnav::kLoiterSpeedMinMs;
using fwcpp::wpnav::kLoiterVelCorrectionMaxMs;
using fwcpp::wpnav::kLoiterGravityMss;

namespace {

void almost(float a, float b) {
    REQUIRE(std::abs(a - b) <= 1e-5f);
}

void almost_vec(const Vector2<float>& got, const Vector2<float>& expected) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
}

float angle_rad_to_accel_mss(float angle_rad) { return kLoiterGravityMss * std::tan(angle_rad); }

}  // namespace

TEST_CASE("constructor records groupinfo defaults", "[loiter][init]") {
    const Loiter loiter;
    almost(loiter.speed_max_ne_ms(), kLoiterSpeedDefaultMs);
    almost(loiter.accel_max_ne_mss(), kLoiterAccelMaxDefaultMss);
    almost(loiter.angle_max_deg(), 0.0f);
    REQUIRE(loiter.loiter_option_is_set(LoiterOption::CoordinatedTurnEnabled));
    almost_vec(loiter.desired_accel_ne_mss(), Vector2<float>{});
    almost_vec(loiter.predicted_accel_ne_mss(), Vector2<float>{});
    almost(loiter.brake_accel_mss(), 0.0f);
}

TEST_CASE("init_target_m zeros state and records pos control leftover", "[loiter][init]") {
    Loiter loiter;
    loiter.set_accel_max_ne_mss(20.0f);
    const Vector2<float> pos{4.0f, -1.5f};
    const auto leftover = loiter.init_target_m(
        pos, InitTargetContext{.lean_angle_max_rad = 0.4f,
                               .accel_target_ne_mss = {3.0f, 1.0f},
                               .roll_rad = 0.2f,
                               .pitch_rad = -0.1f});

    almost(leftover.correction_speed_ms, kLoiterVelCorrectionMaxMs);
    almost(leftover.correction_accel_mss, kLoiterGravityMss * std::tan(0.4f));
    almost(leftover.pos_error_max_m, kLoiterPosCorrectionMaxM);
    REQUIRE(leftover.need_ne_init_controller_stopping_point);
    REQUIRE_FALSE(leftover.need_ne_relax_velocity_controller);
    REQUIRE(leftover.pos_desired_ne_m.has_value());
    almost_vec(*leftover.pos_desired_ne_m, pos);
    almost_vec(loiter.predicted_accel_ne_mss(), Vector2<float>{});
    almost_vec(loiter.desired_accel_ne_mss(), Vector2<float>{});
    almost_vec(loiter.predicted_euler_angle_rad(), Vector2<float>{});
    almost(loiter.brake_accel_mss(), 0.0f);
    almost(loiter.accel_max_ne_mss(), leftover.correction_accel_mss);
}

TEST_CASE("init_target copies pos control leftovers", "[loiter][init]") {
    Loiter loiter;
    const auto leftover = loiter.init_target(InitTargetContext{
        .lean_angle_max_rad = 0.6f,
        .accel_target_ne_mss = {1.25f, -0.5f},
        .roll_rad = 0.08f,
        .pitch_rad = -0.03f,
    });

    almost(leftover.correction_speed_ms, kLoiterVelCorrectionMaxMs);
    almost(leftover.pos_error_max_m, kLoiterPosCorrectionMaxM);
    REQUIRE_FALSE(leftover.need_ne_init_controller_stopping_point);
    REQUIRE(leftover.need_ne_relax_velocity_controller);
    REQUIRE_FALSE(leftover.pos_desired_ne_m.has_value());
    almost_vec(loiter.predicted_accel_ne_mss(), Vector2<float>{1.25f, -0.5f});
    almost_vec(loiter.predicted_euler_angle_rad(), Vector2<float>{0.08f, -0.03f});
    almost_vec(loiter.predicted_euler_rate(), Vector2<float>{});
    almost_vec(loiter.predicted_euler_accel(), Vector2<float>{});
    almost(loiter.brake_accel_mss(), 0.0f);
}

TEST_CASE("sanity_check_params floors speed and clamps accel", "[loiter][init]") {
    Loiter loiter;
    loiter.set_speed_max_ne_ms(0.01f);
    loiter.set_accel_max_ne_mss(40.0f);
    loiter.sanity_check_params(0.3f);
    almost(loiter.speed_max_ne_ms(), kLoiterSpeedMinMs);
    almost(loiter.accel_max_ne_mss(), kLoiterGravityMss * std::tan(0.3f));
}

TEST_CASE("get_angle_max_rad defaults to two thirds", "[loiter][init]") {
    const Loiter loiter;
    almost(loiter.get_angle_max_rad(0.6f, 0.3f), 0.2f);
    almost(loiter.get_angle_max_rad(0.3f, 0.9f), 0.2f);
}

TEST_CASE("get_angle_max_rad uses configured min psc", "[loiter][init]") {
    Loiter loiter;
    loiter.set_angle_max_deg(20.0f);
    almost(loiter.get_angle_max_rad(1.0f, 1.0f), fwcpp::math::radians(20.0f));
    almost(loiter.get_angle_max_rad(1.0f, 0.1f), 0.1f);
}

TEST_CASE("set_speed_max floors at min", "[loiter][init]") {
    Loiter loiter;
    loiter.set_speed_max_ne_ms(0.05f);
    almost(loiter.speed_max_ne_ms(), kLoiterSpeedMinMs);
    loiter.set_speed_max_ne_ms(8.0f);
    almost(loiter.speed_max_ne_ms(), 8.0f);
}

TEST_CASE("update records velocity and ne controller leftovers", "[loiter][update]") {
    Loiter loiter;
    const auto leftover = loiter.update(UpdateLoiterContext{});
    REQUIRE(leftover.need_calc_desired_velocity);
    REQUIRE(leftover.need_ne_update_controller);
    REQUIRE(leftover.need_set_pos_vel_accel_ne);
    REQUIRE(leftover.need_avoidance_adjust_velocity);
    REQUIRE(loiter.soften_for_landing());
}

TEST_CASE("update negative dt skips set pos vel accel", "[loiter][update]") {
    Loiter loiter;
    const auto leftover = loiter.update(UpdateLoiterContext{.dt_s = -0.01f, .avoidance_on = true});
    REQUIRE(leftover.need_calc_desired_velocity);
    REQUIRE(leftover.need_ne_update_controller);
    REQUIRE_FALSE(leftover.need_set_pos_vel_accel_ne);
    REQUIRE_FALSE(leftover.need_avoidance_adjust_velocity);
}

TEST_CASE("update stationary holds the seat", "[loiter][update]") {
    Loiter loiter;
    const Vector2<float> pos{10.0f, -4.0f};
    loiter.init_target_m(pos, InitTargetContext{});
    const auto leftover = loiter.update(UpdateLoiterContext{
        .vel_desired_ne_ms = {},
        .pos_desired_ne_m = pos,
        .avoidance_on = false,
    });
    almost_vec(leftover.pos_desired_ne_m, pos);
    almost_vec(leftover.vel_desired_ne_ms, Vector2<float>{});
    almost_vec(leftover.accel_desired_ne_mss, Vector2<float>{});
    REQUIRE_FALSE(leftover.need_avoidance_adjust_velocity);
}

TEST_CASE("update brakes after delay", "[loiter][update]") {
    Loiter loiter;
    loiter.init_target(InitTargetContext{.lean_angle_max_rad = 0.5f});

    const auto leftover = loiter.update(UpdateLoiterContext{
        .now_ms = 2000,
        .dt_s = 0.01f,
        .ekf_gnd_spd_limit_ms = 50.0f,
        .vel_desired_ne_ms = {5.0f, 0.0f},
        .pos_desired_ne_m = {},
        .vel_pid_kp = 1.0f,
        .attitude_lean_angle_max_rad = 0.5f,
        .pos_lean_angle_max_rad = 0.5f,
        .avoidance_on = false,
    });

    const float gnd = std::max(std::min(kLoiterSpeedDefaultMs, 50.0f), kLoiterSpeedMinMs);
    const float angle_max = loiter.get_angle_max_rad(0.5f, 0.5f);
    const float pilot_accel = angle_rad_to_accel_mss(angle_max);
    const float drag = pilot_accel * 5.0f / gnd;
    const float brake_cmd = fwcpp::math::constrain_value(
        fwcpp::math::sqrt_controller(5.0f, 0.5f, kLoiterBrakeJerkDefaultMsss, 0.01f), 0.0f,
        kLoiterBrakeAccelDefaultMss);
    const float brake = fwcpp::math::constrain_value(
        brake_cmd, -kLoiterBrakeJerkDefaultMsss * 0.01f, kLoiterBrakeJerkDefaultMsss * 0.01f);
    const float speed = std::max(5.0f - (drag + brake) * 0.01f, 0.0f);
    almost(loiter.brake_accel_mss(), brake);
    almost(leftover.vel_desired_ne_ms.x, speed);
    almost(leftover.vel_desired_ne_ms.y, 0.0f);
    almost(leftover.accel_desired_ne_mss.x, -brake);
    almost(leftover.pos_desired_ne_m.x, speed * 0.01f);
    almost(leftover.pos_desired_ne_m.y, 0.0f);
    (void)kLoiterBrakeStartDelayDefaultS;
}
