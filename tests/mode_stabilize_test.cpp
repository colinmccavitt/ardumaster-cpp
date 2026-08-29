#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/copter/pilot_input.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

using Catch::Approx;
using fwcpp::control::AttitudeTargetState;
using fwcpp::control::EulerAngleRateShapingGains;
using fwcpp::control::input_euler_angle_roll_pitch_euler_rate_yaw_rad;
using fwcpp::copter::DesiredSpoolState;
using fwcpp::copter::ModeStabilize;
using fwcpp::copter::SpoolState;
using fwcpp::copter::StabilizeRunInputs;
using fwcpp::copter::get_pilot_desired_lean_angles_rad;
using fwcpp::copter::get_pilot_desired_yaw_rate_rads;
using fwcpp::copter::stabilize_run;
using fwcpp::copter::stabilize::PortStatus;
using fwcpp::copter::stabilize::completeness_has;
using fwcpp::copter::stabilize::completeness_size;
using fwcpp::copter::stabilize::on_main_count;
using fwcpp::copter::stabilize::out_of_scope_count;
using fwcpp::copter::stabilize::remaining_count;
using fwcpp::copter::stabilize::this_slice_count;
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

StabilizeRunInputs base_in() {
    StabilizeRunInputs in;
    in.has_valid_input = true;
    in.roll_norm_dz = 0.4f;
    in.pitch_norm_dz = -0.3f;
    in.yaw_norm_dz = 0.5f;
    in.lean_angle_max_rad = fwcpp::math::radians(45.0f);
    in.command_model_pilot_y_rate = 45.0f;
    in.yaw_expo = 0.25f;
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

TEST_CASE("invalid RC zeros lean; input_euler_angle still invoked", "[copter][stabilize]") {
    auto in = base_in();
    in.has_valid_input = false;
    AttitudeTargetState state = fresh_state();
    const auto out = stabilize_run(in, state);

    REQUIRE(out.lean.roll_rad == 0.0f);
    REQUIRE(out.lean.pitch_rad == 0.0f);
    REQUIRE(out.target_yaw_rate_rads == 0.0f);
    REQUIRE(out.input_euler_angle_invoked);

    AttitudeTargetState replay = fresh_state();
    float thrust_angle = 0.0f;
    float thrust_error = 0.0f;
    float feedforward = 0.0f;
    Quaternion ang_error;
    Vector3f ang_vel;
    input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.0f, 0.0f, 0.0f, replay, in.attitude_body,
                                                    in.gyro_body_rads, in.gains, in.dt, thrust_angle,
                                                    thrust_error, feedforward, ang_error, ang_vel);
    REQUIRE(state.euler_angle_target_rad.x == Approx(replay.euler_angle_target_rad.x));
    REQUIRE(state.euler_angle_target_rad.y == Approx(replay.euler_angle_target_rad.y));
    REQUIRE(out.ang_vel_body_rads.x == Approx(ang_vel.x));
    REQUIRE(out.ang_vel_body_rads.y == Approx(ang_vel.y));
    REQUIRE(out.ang_vel_body_rads.z == Approx(ang_vel.z));
}

TEST_CASE("SHUT_DOWN forces throttle 0", "[copter][stabilize]") {
    auto in = base_in();
    in.spool_state = SpoolState::SHUT_DOWN;
    in.throttle_control = 800;
    AttitudeTargetState state = fresh_state();
    const auto out = stabilize_run(in, state);

    REQUIRE(out.throttle_out == 0.0f);
    REQUIRE(out.reset_yaw_target_and_rate);
    REQUIRE(out.reset_I);
    REQUIRE_FALSE(out.reset_I_smoothly);
    REQUIRE(out.angle_boost);
}

TEST_CASE("THROTTLE_UNLIMITED + !throttle_lower clears land_complete", "[copter][stabilize]") {
    auto in = base_in();
    in.spool_state = SpoolState::THROTTLE_UNLIMITED;
    in.throttle_lower_limit = false;
    in.land_complete = true;
    AttitudeTargetState state = fresh_state();
    const auto out = stabilize_run(in, state);

    REQUIRE_FALSE(out.land_complete);
    REQUIRE(out.desired_spool == DesiredSpoolState::THROTTLE_UNLIMITED);
    REQUIRE(out.angle_boost);
}

TEST_CASE("input_euler_angle is the CCP-029 entry (shaped yaw rate)", "[copter][stabilize]") {
    auto in = base_in();
    AttitudeTargetState via_run = fresh_state();
    const auto out = stabilize_run(in, via_run);

    const auto lean = get_pilot_desired_lean_angles_rad(in.has_valid_input, in.roll_norm_dz,
                                                        in.pitch_norm_dz, in.lean_angle_max_rad,
                                                        in.lean_angle_max_rad);
    const float yaw = get_pilot_desired_yaw_rate_rads(in.has_valid_input, in.yaw_norm_dz,
                                                      in.command_model_pilot_y_rate, in.yaw_expo);
    REQUIRE(out.lean.roll_rad == Approx(lean.roll_rad));
    REQUIRE(out.lean.pitch_rad == Approx(lean.pitch_rad));
    REQUIRE(out.target_yaw_rate_rads == Approx(yaw));
    REQUIRE(yaw != Approx(0.0f));

    AttitudeTargetState via_direct = fresh_state();
    float thrust_angle = 0.0f;
    float thrust_error = 0.0f;
    float feedforward = 0.0f;
    Quaternion ang_error;
    Vector3f ang_vel;
    input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        lean.roll_rad, lean.pitch_rad, yaw, via_direct, in.attitude_body, in.gyro_body_rads,
        in.gains, in.dt, thrust_angle, thrust_error, feedforward, ang_error, ang_vel);

    REQUIRE(via_run.euler_rate_target_rads.z == Approx(via_direct.euler_rate_target_rads.z));
    REQUIRE(via_run.ang_vel_target_rads.z == Approx(via_direct.ang_vel_target_rads.z));
    REQUIRE(out.ang_vel_body_rads.z == Approx(ang_vel.z));
    REQUIRE(out.thrust_angle_rad == Approx(thrust_angle));
}

TEST_CASE("throttle_zero requests GROUND_IDLE; GROUND_IDLE zeros throttle", "[copter][stabilize]") {
    auto in = base_in();
    in.throttle_zero = true;
    in.spool_state = SpoolState::GROUND_IDLE;
    in.throttle_control = 800;
    AttitudeTargetState state = fresh_state();
    const auto out = stabilize_run(in, state);

    REQUIRE(out.desired_spool == DesiredSpoolState::GROUND_IDLE);
    REQUIRE(out.throttle_out == 0.0f);
    REQUIRE(out.reset_yaw_target_and_rate);
    REQUIRE(out.reset_I_smoothly);
    REQUIRE_FALSE(out.reset_I);
}

TEST_CASE("ModeStabilize::run stays a no-op wrapper", "[copter][stabilize]") {
    ModeStabilize mode;
    mode.run();
    REQUIRE(mode.mode_number() == fwcpp::copter::Mode::Number::STABILIZE);
}

TEST_CASE("leftover remaining_count is not zero", "[copter][stabilize][leftover]") {
    REQUIRE(remaining_count() > 0);
    REQUIRE(this_slice_count() == 5);
    REQUIRE(on_main_count() == 0);
    REQUIRE(out_of_scope_count() == 0);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("stabilize_run", PortStatus::kThisSlice));
    REQUIRE(completeness_has("update_simple_mode", PortStatus::kRemaining));
    REQUIRE(completeness_has("acro_run", PortStatus::kRemaining));
    REQUIRE(completeness_has("althold_run", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("AUTO_RTL", PortStatus::kRemaining));
}
