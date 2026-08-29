#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/copter/mode_acro.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/copter/pilot_input.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

using Catch::Approx;
using fwcpp::control::AttitudeTargetState;
using fwcpp::control::EulerAngleRateShapingGains;
using fwcpp::control::input_rate_bf_roll_pitch_yaw_2_rads;
using fwcpp::control::input_rate_bf_roll_pitch_yaw_rads;
using fwcpp::copter::AcroOptions;
using fwcpp::copter::AcroRunInputs;
using fwcpp::copter::DesiredSpoolState;
using fwcpp::copter::SpoolState;
using fwcpp::copter::acro_run;
using fwcpp::copter::get_pilot_desired_rates_rads;
using fwcpp::copter::input_expo;
using fwcpp::copter::acro::PortStatus;
using fwcpp::copter::acro::completeness_has;
using fwcpp::copter::acro::completeness_size;
using fwcpp::copter::acro::on_main_count;
using fwcpp::copter::acro::out_of_scope_count;
using fwcpp::copter::acro::remaining_count;
using fwcpp::copter::acro::this_slice_count;
using fwcpp::math::Quaternion;
using fwcpp::math::Vector3f;

namespace {

EulerAngleRateShapingGains test_gains() {
    EulerAngleRateShapingGains g;
    g.rate_bf_ff_enabled = true;
    g.input_tc = 0.15f;
    g.rate_y_tc = 0.2f;
    g.rate_rp_tc = 0.15f;
    g.ang_vel_roll_max_degs = 220.0f;
    g.ang_vel_pitch_max_degs = 220.0f;
    g.ang_vel_yaw_max_degs = 200.0f;
    g.accel_roll_max_radss = fwcpp::math::radians(400.0f);
    g.accel_pitch_max_radss = fwcpp::math::radians(400.0f);
    g.accel_yaw_max_radss = fwcpp::math::radians(200.0f);
    g.rate_yaw_kp = 2.0f;
    g.angle_yaw_kp = 1.0f;
    g.angle_kp_roll = 6.0f;
    g.angle_kp_pitch = 6.0f;
    g.angle_kp_yaw = 4.0f;
    return g;
}

AttitudeTargetState fresh_state() {
    AttitudeTargetState s;
    s.attitude_target.from_euler(0.0f, 0.0f, 0.0f);
    return s;
}

AcroRunInputs base_in() {
    AcroRunInputs in;
    in.roll_norm_dz = 0.4f;
    in.pitch_norm_dz = -0.3f;
    in.yaw_norm_dz = 0.5f;
    in.acro_rp_rate = 360.0f;
    in.acro_rp_expo = 0.3f;
    in.acro_y_rate = 202.5f;
    in.acro_y_expo = 0.25f;
    in.acro_options = 0;
    in.mid_stick = 500;
    in.throttle_control = 700;
    in.throttle_hover = 0.5f;
    in.throttle_zero = false;
    in.spool_state = SpoolState::THROTTLE_UNLIMITED;
    in.throttle_lower_limit = false;
    in.land_complete = true;
    in.attitude_body.from_euler(0.0f, 0.0f, 0.0f);
    in.gains = test_gains();
    in.dt = 0.0025f;
    return in;
}

}  // namespace

TEST_CASE("circular stick limit scales pitch and roll when hypot > 1", "[copter][acro]") {
    const float roll = 0.8f;
    const float pitch = 0.8f;
    const float yaw = 0.2f;
    const float rp_rate = 360.0f;
    const float rp_expo = 0.3f;
    const float y_rate = 202.5f;
    const float y_expo = 0.25f;

    REQUIRE(std::hypot(pitch, roll) > 1.0f);

    const auto limited = get_pilot_desired_rates_rads(roll, pitch, yaw, rp_rate, rp_expo, y_rate, y_expo);

    const float ratio = 1.0f / std::hypot(pitch, roll);
    const float roll_scaled = roll * ratio;
    const float pitch_scaled = pitch * ratio;
    REQUIRE(std::hypot(pitch_scaled, roll_scaled) == Approx(1.0f).margin(1e-6f));

    const float expect_roll = fwcpp::math::radians(rp_rate) * input_expo(roll_scaled, rp_expo);
    const float expect_pitch = fwcpp::math::radians(rp_rate) * input_expo(pitch_scaled, rp_expo);
    const float expect_yaw = fwcpp::math::radians(y_rate) * input_expo(yaw, y_expo);
    const float unscaled_roll = fwcpp::math::radians(rp_rate) * input_expo(roll, rp_expo);

    REQUIRE(limited.roll_rads == Approx(expect_roll));
    REQUIRE(limited.pitch_rads == Approx(expect_pitch));
    REQUIRE(limited.yaw_rads == Approx(expect_yaw));
    REQUIRE(std::fabs(limited.roll_rads) < std::fabs(unscaled_roll));

    const auto inside = get_pilot_desired_rates_rads(0.4f, -0.3f, yaw, rp_rate, rp_expo, y_rate, y_expo);
    REQUIRE(std::hypot(-0.3f, 0.4f) <= 1.0f);
    REQUIRE(inside.roll_rads == Approx(fwcpp::math::radians(rp_rate) * input_expo(0.4f, rp_expo)));
    REQUIRE(inside.pitch_rads == Approx(fwcpp::math::radians(rp_rate) * input_expo(-0.3f, rp_expo)));
}

TEST_CASE("RATE_LOOP_ONLY vs stabilized input_rate_bf both invoked", "[copter][acro]") {
    auto in = base_in();
    in.roll_norm_dz = 0.8f;
    in.pitch_norm_dz = 0.8f;

    const auto expect_rates = get_pilot_desired_rates_rads(in.roll_norm_dz, in.pitch_norm_dz, in.yaw_norm_dz,
                                                           in.acro_rp_rate, in.acro_rp_expo, in.acro_y_rate,
                                                           in.acro_y_expo);

    SECTION("stabilized path calls input_rate_bf_roll_pitch_yaw_rads") {
        in.acro_options = 0;
        AttitudeTargetState via_run = fresh_state();
        const auto out = acro_run(in, via_run);

        REQUIRE(out.rates.roll_rads == Approx(expect_rates.roll_rads));
        REQUIRE(out.rates.pitch_rads == Approx(expect_rates.pitch_rads));
        REQUIRE(out.rates.yaw_rads == Approx(expect_rates.yaw_rads));
        REQUIRE(out.input_rate_bf_invoked);
        REQUIRE_FALSE(out.input_rate_bf_2_invoked);
        REQUIRE_FALSE(out.rate_loop_only);
        REQUIRE_FALSE(out.scale_I_to_angle_P);
        REQUIRE_FALSE(out.angle_boost);

        AttitudeTargetState via_direct = fresh_state();
        float thrust_angle = 0.0f;
        float thrust_error = 0.0f;
        float feedforward = 0.0f;
        Quaternion ang_error;
        Vector3f ang_vel;
        input_rate_bf_roll_pitch_yaw_rads(expect_rates.roll_rads, expect_rates.pitch_rads, expect_rates.yaw_rads,
                                          via_direct, in.attitude_body, in.gyro_body_rads, in.gains, in.dt,
                                          thrust_angle, thrust_error, feedforward, ang_error, ang_vel);

        REQUIRE(via_run.ang_vel_target_rads.x == Approx(via_direct.ang_vel_target_rads.x));
        REQUIRE(via_run.ang_vel_target_rads.y == Approx(via_direct.ang_vel_target_rads.y));
        REQUIRE(via_run.ang_vel_target_rads.z == Approx(via_direct.ang_vel_target_rads.z));
        REQUIRE(out.ang_vel_body_rads.x == Approx(ang_vel.x));
        REQUIRE(out.ang_vel_body_rads.y == Approx(ang_vel.y));
        REQUIRE(out.ang_vel_body_rads.z == Approx(ang_vel.z));
        REQUIRE(out.thrust_angle_rad == Approx(thrust_angle));
    }

    SECTION("RATE_LOOP_ONLY calls input_rate_bf_roll_pitch_yaw_2_rads") {
        in.acro_options = static_cast<std::uint8_t>(AcroOptions::RATE_LOOP_ONLY);
        AttitudeTargetState via_run = fresh_state();
        const auto out = acro_run(in, via_run);

        REQUIRE(out.input_rate_bf_2_invoked);
        REQUIRE_FALSE(out.input_rate_bf_invoked);
        REQUIRE(out.rate_loop_only);
        REQUIRE(out.scale_I_to_angle_P);
        REQUIRE_FALSE(out.angle_boost);

        AttitudeTargetState via_direct = fresh_state();
        Vector3f ang_vel;
        input_rate_bf_roll_pitch_yaw_2_rads(expect_rates.roll_rads, expect_rates.pitch_rads, expect_rates.yaw_rads,
                                            via_direct, in.attitude_body, in.gains, in.dt, ang_vel);

        REQUIRE(via_run.attitude_target.q1 == Approx(via_direct.attitude_target.q1));
        REQUIRE(via_run.ang_vel_target_rads.x == Approx(via_direct.ang_vel_target_rads.x));
        REQUIRE(via_run.ang_vel_target_rads.y == Approx(via_direct.ang_vel_target_rads.y));
        REQUIRE(via_run.ang_vel_target_rads.z == Approx(via_direct.ang_vel_target_rads.z));
        REQUIRE(out.ang_vel_body_rads.x == Approx(ang_vel.x));
        REQUIRE(out.ang_vel_body_rads.y == Approx(ang_vel.y));
        REQUIRE(out.ang_vel_body_rads.z == Approx(ang_vel.z));
    }
}

TEST_CASE("SHUT_DOWN zeros throttle", "[copter][acro]") {
    auto in = base_in();
    in.spool_state = SpoolState::SHUT_DOWN;
    in.throttle_control = 800;
    AttitudeTargetState state = fresh_state();
    const auto out = acro_run(in, state);

    REQUIRE(out.throttle_out == 0.0f);
    REQUIRE(out.reset_target_and_rate);
    REQUIRE(out.reset_I);
    REQUIRE_FALSE(out.reset_I_smoothly);
    REQUIRE_FALSE(out.angle_boost);
    REQUIRE(out.desired_spool == DesiredSpoolState::THROTTLE_UNLIMITED);
}

TEST_CASE("leftover remaining_count is not zero", "[copter][acro][leftover]") {
    REQUIRE(remaining_count() > 0);
    REQUIRE(this_slice_count() == 6);
    REQUIRE(on_main_count() == 1);
    REQUIRE(out_of_scope_count() == 0);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("acro_run", PortStatus::kThisSlice));
    REQUIRE(completeness_has("stabilize_run", PortStatus::kOnMain));
    REQUIRE(completeness_has("althold_run", PortStatus::kRemaining));
    REQUIRE(completeness_has("trainer LEVEL/LIMITED", PortStatus::kRemaining));
    REQUIRE(completeness_has("scale_I_to_angle_P", PortStatus::kRemaining));
    REQUIRE(completeness_has("AIR_MODE init", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("AUTO_RTL", PortStatus::kRemaining));
}
