#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/mode_althold.hpp>
#include <fwcpp/copter/pilot_input.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

using Catch::Approx;
using fwcpp::control::AttitudeTargetState;
using fwcpp::control::EulerAngleRateShapingGains;
using fwcpp::control::input_euler_angle_roll_pitch_euler_rate_yaw_rad;
using fwcpp::copter::AltHoldModeState;
using fwcpp::copter::AltHoldRunInputs;
using fwcpp::copter::DesiredSpoolState;
using fwcpp::copter::ModeAltHold;
using fwcpp::copter::SpoolState;
using fwcpp::copter::althold_run;
using fwcpp::copter::get_pilot_desired_climb_rate_ms;
using fwcpp::copter::get_pilot_speed_dn_ms;
using fwcpp::copter::althold::PortStatus;
using fwcpp::copter::althold::completeness_has;
using fwcpp::copter::althold::completeness_size;
using fwcpp::copter::althold::on_main_count;
using fwcpp::copter::althold::out_of_scope_count;
using fwcpp::copter::althold::remaining_count;
using fwcpp::copter::althold::this_slice_count;
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

AltHoldRunInputs base_in() {
    AltHoldRunInputs in;
    in.has_valid_input = true;
    in.roll_norm_dz = 0.4f;
    in.pitch_norm_dz = -0.3f;
    in.yaw_norm_dz = 0.5f;
    in.lean_angle_max_rad = fwcpp::math::radians(45.0f);
    in.althold_lean_angle_max_rad = fwcpp::math::radians(45.0f);
    in.command_model_pilot_y_rate = 45.0f;
    in.yaw_expo = 0.25f;
    in.mid_stick = 500.0f;
    in.throttle_control = 800.0f;
    in.throttle_deadzone = 100;
    in.speed_dn_ms = 1.5f;
    in.speed_up_ms = 2.5f;
    in.accel_leftover = true;
    in.armed = true;
    in.takeoff_running = false;
    in.takeoff_triggered = false;
    in.auto_armed = true;
    in.land_complete = false;
    in.using_interlock = false;
    in.spool_state = SpoolState::THROTTLE_UNLIMITED;
    in.attitude_body.from_euler(0.0f, 0.0f, 0.0f);
    in.gains = test_gains();
    in.dt = 0.0025f;
    return in;
}

}  // namespace

TEST_CASE("invalid RC zeros climb and lean; input_euler_angle still invoked", "[copter][althold]") {
    auto in = base_in();
    in.has_valid_input = false;
    AttitudeTargetState state = fresh_state();
    const auto out = althold_run(in, state);

    REQUIRE(out.update_simple_mode);
    REQUIRE(out.lean.roll_rad == 0.0f);
    REQUIRE(out.lean.pitch_rad == 0.0f);
    REQUIRE(out.target_yaw_rate_rads == 0.0f);
    REQUIRE(out.target_climb_rate_ms == 0.0f);
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

TEST_CASE("climb_rate constrained to +/- speed", "[copter][althold]") {
    auto in = base_in();
    in.speed_dn_ms = 1.5f;
    in.speed_up_ms = 2.0f;
    const float speed_dn = get_pilot_speed_dn_ms(in.speed_dn_ms, in.speed_up_ms);

    SECTION("full-up stick is +speed_up") {
        in.throttle_control = 1000.0f;
        AttitudeTargetState state = fresh_state();
        const auto out = althold_run(in, state);
        const float raw = get_pilot_desired_climb_rate_ms(in.has_valid_input, in.throttle_control, in.mid_stick,
                                                          in.throttle_deadzone, in.speed_dn_ms, in.speed_up_ms);
        const float expect = fwcpp::math::constrain_value(raw, -speed_dn, in.speed_up_ms);
        REQUIRE(out.target_climb_rate_ms == Approx(expect));
        REQUIRE(out.target_climb_rate_ms == Approx(in.speed_up_ms));
        REQUIRE(out.target_climb_rate_ms <= in.speed_up_ms);
        REQUIRE(out.target_climb_rate_ms >= -speed_dn);
        REQUIRE(out.D_set_max_speed_accel);
        REQUIRE(out.speed_dn_ms == Approx(speed_dn));
        REQUIRE(out.speed_up_ms == Approx(in.speed_up_ms));
    }

    SECTION("full-down stick is -get_pilot_speed_dn_ms") {
        in.throttle_control = 0.0f;
        AttitudeTargetState state = fresh_state();
        const auto out = althold_run(in, state);
        const float raw = get_pilot_desired_climb_rate_ms(in.has_valid_input, in.throttle_control, in.mid_stick,
                                                          in.throttle_deadzone, in.speed_dn_ms, in.speed_up_ms);
        const float expect = fwcpp::math::constrain_value(raw, -speed_dn, in.speed_up_ms);
        REQUIRE(out.target_climb_rate_ms == Approx(expect));
        REQUIRE(out.target_climb_rate_ms == Approx(-speed_dn));
        REQUIRE(out.target_climb_rate_ms <= in.speed_up_ms);
        REQUIRE(out.target_climb_rate_ms >= -speed_dn);
    }

    SECTION("raw PILOT_SPD_DN zero uses |speed_up| as down limit") {
        in.speed_dn_ms = 0.0f;
        in.speed_up_ms = 2.0f;
        in.throttle_control = 0.0f;
        const float dn = get_pilot_speed_dn_ms(in.speed_dn_ms, in.speed_up_ms);
        REQUIRE(dn == Approx(2.0f));
        AttitudeTargetState state = fresh_state();
        const auto out = althold_run(in, state);
        REQUIRE(out.target_climb_rate_ms == Approx(-dn));
        REQUIRE(out.speed_dn_ms == Approx(dn));
    }
}

TEST_CASE("!armed SHUT_DOWN spool -> MotorStopped", "[copter][althold]") {
    auto in = base_in();
    in.armed = false;
    in.spool_state = SpoolState::SHUT_DOWN;
    AttitudeTargetState state = fresh_state();
    const auto out = althold_run(in, state);

    REQUIRE(out.update_simple_mode);
    REQUIRE(out.state == AltHoldModeState::MotorStopped);
    REQUIRE(out.desired_spool == DesiredSpoolState::SHUT_DOWN);
    REQUIRE(out.desired_spool_set);
    REQUIRE(out.reset_I);
    REQUIRE(out.reset_yaw_target_and_rate);
    REQUIRE_FALSE(out.reset_I_smoothly);
    REQUIRE(out.D_relax_controller);
    REQUIRE(out.D_relax_throttle == 0.0f);
    REQUIRE_FALSE(out.D_set_pos_target_from_climb_rate);
    REQUIRE(out.input_euler_angle_invoked);
}

TEST_CASE("Takeoff !running sets takeoff_start avoidance do_pilot_takeoff", "[copter][althold]") {
    auto in = base_in();
    in.armed = true;
    in.takeoff_running = false;
    in.takeoff_triggered = true;
    in.auto_armed = true;
    in.land_complete = false;
    in.spool_state = SpoolState::THROTTLE_UNLIMITED;
    AttitudeTargetState state = fresh_state();
    const auto out = althold_run(in, state);

    REQUIRE(out.update_simple_mode);
    REQUIRE(out.state == AltHoldModeState::Takeoff);
    REQUIRE(out.takeoff_start);
    REQUIRE(out.avoidance);
    REQUIRE_FALSE(out.adjust_roll_pitch_avoidance);
    REQUIRE(out.do_pilot_takeoff);
    REQUIRE_FALSE(out.D_set_pos_target_from_climb_rate);
    REQUIRE_FALSE(out.surface_tracking);
    REQUIRE(out.D_update_controller);
    REQUIRE(out.input_euler_angle_invoked);
}

TEST_CASE("Takeoff already running skips takeoff_start", "[copter][althold]") {
    auto in = base_in();
    in.armed = true;
    in.takeoff_running = true;
    in.takeoff_triggered = false;
    in.auto_armed = true;
    in.land_complete = false;
    in.spool_state = SpoolState::THROTTLE_UNLIMITED;
    AttitudeTargetState state = fresh_state();
    const auto out = althold_run(in, state);

    REQUIRE(out.update_simple_mode);
    REQUIRE(out.state == AltHoldModeState::Takeoff);
    REQUIRE_FALSE(out.takeoff_start);
    REQUIRE(out.avoidance);
    REQUIRE_FALSE(out.adjust_roll_pitch_avoidance);
    REQUIRE(out.do_pilot_takeoff);
}

TEST_CASE("Flying sets adjust_roll_pitch_avoidance surface_tracking avoidance", "[copter][althold]") {
    auto in = base_in();
    in.armed = true;
    in.takeoff_running = false;
    in.takeoff_triggered = false;
    in.auto_armed = true;
    in.land_complete = false;
    in.spool_state = SpoolState::THROTTLE_UNLIMITED;
    in.throttle_control = 800.0f;
    AttitudeTargetState state = fresh_state();
    const auto out = althold_run(in, state);

    REQUIRE(out.update_simple_mode);
    REQUIRE(out.state == AltHoldModeState::Flying);
    REQUIRE(out.desired_spool == DesiredSpoolState::THROTTLE_UNLIMITED);
    REQUIRE(out.D_set_pos_target_from_climb_rate);
    REQUIRE(out.pos_target_climb_rate_ms == Approx(out.target_climb_rate_ms));
    REQUIRE(out.pos_target_climb_rate_ms != Approx(0.0f));
    REQUIRE(out.adjust_roll_pitch_avoidance);
    REQUIRE(out.avoidance);
    REQUIRE(out.surface_tracking);
    REQUIRE_FALSE(out.D_relax_controller);
    REQUIRE(out.D_update_controller);
    REQUIRE(out.input_euler_angle_invoked);
    REQUIRE(out.D_set_max_speed_accel);
}

TEST_CASE("leftover remaining_count is not zero", "[copter][althold][leftover]") {
    REQUIRE(remaining_count() == 1);
    REQUIRE(this_slice_count() == 9);
    REQUIRE(on_main_count() == 2);
    REQUIRE(out_of_scope_count() == 0);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("althold_run", PortStatus::kThisSlice));
    REQUIRE(completeness_has("get_alt_hold_state_D_ms", PortStatus::kThisSlice));
    REQUIRE(completeness_has("update_simple_mode", PortStatus::kThisSlice));
    REQUIRE(completeness_has("takeoff", PortStatus::kThisSlice));
    REQUIRE(completeness_has("avoidance", PortStatus::kThisSlice));
    REQUIRE(completeness_has("surface_tracking", PortStatus::kThisSlice));
    REQUIRE(completeness_has("stabilize_run", PortStatus::kOnMain));
    REQUIRE(completeness_has("acro_run", PortStatus::kOnMain));
    REQUIRE(completeness_has("D_update_controller", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("avoidance", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("surface_tracking", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("takeoff", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("update_simple_mode", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("AUTO_RTL", PortStatus::kRemaining));
}

TEST_CASE("ModeAltHold::run stays a no-op wrapper", "[copter][althold]") {
    ModeAltHold mode;
    mode.run();
    REQUIRE(mode.mode_number() == fwcpp::copter::Mode::Number::ALT_HOLD);
}
