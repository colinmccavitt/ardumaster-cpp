#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_frame.hpp>
#include <fwcpp/quadplane/quadplane_stabilize.hpp>
#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_transition.hpp>

using Catch::Approx;
using fwcpp::math::Vector3f;
using fwcpp::quadplane::AhrsViewRotation;
using fwcpp::quadplane::AirMode;
using fwcpp::quadplane::AttitudeRateInputs;
using fwcpp::quadplane::DesiredSpoolState;
using fwcpp::quadplane::DesiredYawRateInputs;
using fwcpp::quadplane::HoldStabilizeInputs;
using fwcpp::quadplane::MotorFrameClass;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::RelaxAttitudeInputs;
using fwcpp::quadplane::SpoolState;
using fwcpp::quadplane::ZCtrlInputs;
using fwcpp::quadplane::ZCtrlState;
using fwcpp::quadplane::hold_stabilize;
using fwcpp::quadplane::kCommandModelPilotRateDefault;
using fwcpp::quadplane::kPidzReinitGapMs;
using fwcpp::quadplane::kPilotAccelZMssDefault;
using fwcpp::quadplane::kPilotSpeedZMaxUpMsDefault;
using fwcpp::quadplane::motor_frame_class_as_u8;
using fwcpp::quadplane::multicopter_attitude_rate_update;
using fwcpp::quadplane::relax_attitude_control;
using fwcpp::quadplane::rotate_ahrs_view;
using fwcpp::quadplane::run_z_controller;
using fwcpp::tailsitter::TailsitterTransitionState;
using fwcpp::tailsitter::kTailsitterInputBfRoll;
using fwcpp::tailsitter::kTailsitterInputPlane;

static AttitudeRateInputs vtol_att(float yaw_rate_cds = 0.0f) {
    AttitudeRateInputs in{};
    in.in_vtol_mode = true;
    in.nav_roll_cd = 300.0f;
    in.nav_pitch_cd = -150.0f;
    in.yaw_rate_cds = yaw_rate_cds;
    return in;
}

static ZCtrlInputs unlimited_z(std::uint32_t now_ms) {
    ZCtrlInputs in{};
    in.now_ms = now_ms;
    in.spool = SpoolState::kThrottleUnlimited;
    in.d_is_active = false;
    in.pilot_speed_z_max_up_ms = kPilotSpeedZMaxUpMsDefault;
    in.pilot_speed_z_max_dn_ms = 0.0f;
    in.pilot_accel_z_mss = kPilotAccelZMssDefault;
    return in;
}

TEST_CASE("hold_stabilize idle vs unlimited+boost", "[quadplane][stabilize]") {
    HoldStabilizeInputs in{};
    in.throttle_in = 0.0f;
    in.yaw.pilot.throttle_input = 0.0f;
    in.attitude = vtol_att();

    const auto idle = hold_stabilize(in);
    REQUIRE(idle.multicopter_attitude_rate_update);
    REQUIRE(idle.attitude.use_multicopter_control);
    REQUIRE(idle.desired_spool == DesiredSpoolState::kGroundIdle);
    REQUIRE(idle.set_throttle_out);
    REQUIRE(idle.throttle_out == Approx(0.0f));
    REQUIRE(idle.should_boost);
    REQUIRE(idle.throttle_filt_hz == Approx(0.0f));
    REQUIRE(idle.relax_attitude_control);
    REQUIRE_FALSE(idle.relax.relax_pitch_disabled);

    in.throttle_in = 0.45f;
    const auto unlimited = hold_stabilize(in);
    REQUIRE(unlimited.desired_spool == DesiredSpoolState::kThrottleUnlimited);
    REQUIRE(unlimited.throttle_out == Approx(0.45f));
    REQUIRE(unlimited.should_boost);
    REQUIRE_FALSE(unlimited.relax_attitude_control);

    in.throttle_in = 0.0f;
    in.air_mode_active = true;
    const auto air = hold_stabilize(in);
    REQUIRE(air.desired_spool == DesiredSpoolState::kThrottleUnlimited);
    REQUIRE(air.throttle_out == Approx(0.0f));
    REQUIRE(air.should_boost);
    REQUIRE_FALSE(air.relax_attitude_control);

    QuadPlane qp{1};
    const auto wired_idle = qp.hold_stabilize(0.0f, {});
    REQUIRE(wired_idle.desired_spool == DesiredSpoolState::kGroundIdle);
    REQUIRE(wired_idle.relax_attitude_control);
    qp.set_air_mode(AirMode::kOn);
    const auto wired_air = qp.hold_stabilize(0.0f, {});
    REQUIRE(wired_air.desired_spool == DesiredSpoolState::kThrottleUnlimited);
}

TEST_CASE("hold_stabilize tailsitter assist disables boost", "[quadplane][stabilize]") {
    HoldStabilizeInputs in{};
    in.throttle_in = 0.6f;
    in.tailsitter_enabled = true;
    in.assisted_flight = true;
    in.attitude = vtol_att();
    const auto tick = hold_stabilize(in);
    REQUIRE(tick.desired_spool == DesiredSpoolState::kThrottleUnlimited);
    REQUIRE_FALSE(tick.should_boost);
    REQUIRE(tick.throttle_out == Approx(0.6f));

    in.assisted_flight = false;
    REQUIRE(hold_stabilize(in).should_boost);

    QuadPlane qp{1};
    qp.set_frame_class(motor_frame_class_as_u8(MotorFrameClass::kTailsitter));
    REQUIRE(qp.setup());
    REQUIRE(qp.tailsitter().enabled());
    qp.set_assisted_flight(true);
    HoldStabilizeInputs wired{};
    wired.attitude = vtol_att();
    const auto qp_tick = qp.hold_stabilize(0.3f, wired);
    REQUIRE_FALSE(qp_tick.should_boost);
}

TEST_CASE("relax_attitude_control uses tailsitter relax_pitch", "[quadplane][stabilize]") {
    RelaxAttitudeInputs in{};
    REQUIRE_FALSE(relax_attitude_control(in).relax_pitch_disabled);
    in.tailsitter_enabled = true;
    in.tailsitter_is_vectored = true;
    in.vtol_limit_start_ms = 0;
    REQUIRE(relax_attitude_control(in).relax_pitch_disabled);
    in.vtol_limit_start_ms = 1;
    REQUIRE_FALSE(relax_attitude_control(in).relax_pitch_disabled);
}

TEST_CASE("run_z_controller spool and transition skips", "[quadplane][stabilize][z]") {
    ZCtrlState state{};
    ZCtrlInputs in = unlimited_z(1000);
    in.spool = SpoolState::kGroundIdle;
    auto tick = run_z_controller(state, in);
    REQUIRE(tick.early_return);
    REQUIRE_FALSE(tick.d_update);
    REQUIRE(state.last_pidz_active_ms == 0);
    REQUIRE(state.last_pidz_init_ms == 0);

    in.spool = SpoolState::kThrottleUnlimited;
    in.tailsitter_enabled = true;
    in.in_vtol_mode = true;
    in.transition_state = TailsitterTransitionState::kAngleWaitVtol;
    tick = run_z_controller(state, in);
    REQUIRE(tick.early_return);
    REQUIRE_FALSE(tick.d_update);
    REQUIRE(state.last_pidz_active_ms == 0);

    in.transition_state = TailsitterTransitionState::kDone;
    in.now_ms = 2000;
    in.last_vtol_mode_ms = 0;
    tick = run_z_controller(state, in);
    REQUIRE(tick.early_return);

    QuadPlane qp{1};
    ZCtrlInputs wired = unlimited_z(50);
    wired.spool = SpoolState::kSpoolingUp;
    const auto qp_skip = qp.run_z_controller(wired);
    REQUIRE(qp_skip.early_return);
    REQUIRE(qp.last_pidz_active_ms() == 0);
}

TEST_CASE("run_z_controller 20ms reinit vs active", "[quadplane][stabilize][z]") {
    ZCtrlState state{};
    ZCtrlInputs in = unlimited_z(1000);
    auto tick = run_z_controller(state, in);
    REQUIRE_FALSE(tick.early_return);
    REQUIRE(tick.d_set_max);
    REQUIRE(tick.d_init);
    REQUIRE_FALSE(tick.d_init_no_descent);
    REQUIRE(tick.d_update);
    REQUIRE(tick.d_max_speed_dn_m == Approx(2.0f));
    REQUIRE(tick.d_max_speed_up_ms == Approx(kPilotSpeedZMaxUpMsDefault));
    REQUIRE(tick.d_max_accel_z_mss == Approx(kPilotAccelZMssDefault));
    REQUIRE(state.last_pidz_init_ms == 1000);
    REQUIRE(state.last_pidz_active_ms == 1000);

    in.now_ms = 1000 + kPidzReinitGapMs;
    in.d_is_active = true;
    tick = run_z_controller(state, in);
    REQUIRE_FALSE(tick.d_set_max);
    REQUIRE_FALSE(tick.d_init);
    REQUIRE(tick.d_update);
    REQUIRE(state.last_pidz_init_ms == 1000);
    REQUIRE(state.last_pidz_active_ms == 1020);

    in.now_ms = 1020 + kPidzReinitGapMs + 1;
    tick = run_z_controller(state, in);
    REQUIRE(tick.d_set_max);
    REQUIRE(tick.d_init);
    REQUIRE(state.last_pidz_init_ms == 1041);
    REQUIRE(state.last_pidz_active_ms == 1041);

    QuadPlane qp{1};
    qp.set_pilot_speed_z_max_up_ms(3.0f);
    qp.set_pilot_speed_z_max_dn_ms(1.0f);
    qp.set_pilot_accel_z_mss(4.0f);
    const auto wired = qp.run_z_controller(unlimited_z(500));
    REQUIRE(wired.d_init);
    REQUIRE(wired.d_max_speed_dn_m == Approx(1.0f));
    REQUIRE(wired.d_max_speed_up_ms == Approx(3.0f));
    REQUIRE(qp.last_pidz_active_ms() == 500);
    REQUIRE(qp.last_pidz_init_ms() == 500);
}

TEST_CASE("run_z_controller tailsitter no-descent init", "[quadplane][stabilize][z]") {
    ZCtrlState state{};
    ZCtrlInputs in = unlimited_z(400);
    in.tailsitter_enabled = true;
    const auto tick = run_z_controller(state, in);
    REQUIRE(tick.d_set_max);
    REQUIRE_FALSE(tick.d_init);
    REQUIRE(tick.d_init_no_descent);
    REQUIRE(tick.d_update);

    QuadPlane qp{1};
    qp.set_frame_class(motor_frame_class_as_u8(MotorFrameClass::kTailsitter));
    REQUIRE(qp.setup());
    const auto wired = qp.run_z_controller(unlimited_z(400));
    REQUIRE(wired.d_init_no_descent);
    REQUIRE_FALSE(wired.d_init);
}

TEST_CASE("multicopter_attitude_rate_update VTOL vs FW path", "[quadplane][stabilize][att]") {
    auto vtol = multicopter_attitude_rate_update(vtol_att(80.0f));
    REQUIRE(vtol.use_multicopter_control);
    REQUIRE_FALSE(vtol.use_yaw_target);
    REQUIRE(vtol.set_pilot_yaw_rate_time_constant);
    REQUIRE(vtol.input_euler_angle_roll_pitch_euler_rate_yaw);
    REQUIRE_FALSE(vtol.input_euler_angle_roll_pitch_yaw);
    REQUIRE(vtol.roll_cd == Approx(300.0f));
    REQUIRE(vtol.pitch_cd == Approx(-150.0f));
    REQUIRE(vtol.yaw_cd_or_rate == Approx(80.0f));
    REQUIRE_FALSE(vtol.ahrs_view_rotate);
    REQUIRE_FALSE(vtol.input_rate_bf_roll_pitch_yaw_no_shaping);

    AttitudeRateInputs fw{};
    fw.in_vtol_mode = false;
    fw.fw_roll_pid_target = 1.5f;
    fw.fw_pitch_pid_target = -0.25f;
    fw.yaw_rate_cds = 40.0f;
    auto fw_tick = multicopter_attitude_rate_update(fw);
    REQUIRE_FALSE(fw_tick.use_multicopter_control);
    REQUIRE(fw_tick.ahrs_view_rotate);
    REQUIRE(fw_tick.disable_yaw_rate_time_constant);
    REQUIRE(fw_tick.input_rate_bf_roll_pitch_yaw_no_shaping);
    REQUIRE(fw_tick.bf_input_cd.x == Approx(150.0f));
    REQUIRE(fw_tick.bf_input_cd.y == Approx(-25.0f));
    REQUIRE(fw_tick.bf_input_cd.z == Approx(40.0f));

    fw.ahrs_view_rotation = AhrsViewRotation::kPitch90;
    fw_tick = multicopter_attitude_rate_update(fw);
    REQUIRE(fw_tick.bf_input_cd.x == Approx(40.0f));
    REQUIRE(fw_tick.bf_input_cd.y == Approx(-25.0f));
    REQUIRE(fw_tick.bf_input_cd.z == Approx(-150.0f));

    Vector3f already{9.0f, 8.0f, 7.0f};
    fw.ahrs_view_already_rotated = true;
    fw.ahrs_view_rotated_bf = already;
    fw_tick = multicopter_attitude_rate_update(fw);
    REQUIRE(fw_tick.bf_input_cd.x == Approx(9.0f));
    REQUIRE(fw_tick.bf_input_cd.y == Approx(8.0f));
    REQUIRE(fw_tick.bf_input_cd.z == Approx(7.0f));

    Vector3f helper{150.0f, -25.0f, 40.0f};
    rotate_ahrs_view(helper, AhrsViewRotation::kPitch90);
    REQUIRE(helper.x == Approx(40.0f));
    REQUIRE(helper.z == Approx(-150.0f));
}

TEST_CASE("multicopter_attitude_rate_update BF_ROLL plane vs copter", "[quadplane][stabilize][att]") {
    AttitudeRateInputs in = vtol_att(200.0f);
    in.tailsitter_enabled = true;
    in.tailsitter_input_type = static_cast<std::int8_t>(kTailsitterInputBfRoll);
    in.nav_roll_cd = 1200.0f;
    in.nav_pitch_cd = 0.0f;
    in.lean_angle_max_cd = 4500.0f;

    auto copter = multicopter_attitude_rate_update(in);
    REQUIRE(copter.use_multicopter_control);
    REQUIRE(copter.input_euler_rate_yaw_euler_angle_pitch_bf_roll);
    REQUIRE_FALSE(copter.bf_roll_plane);
    REQUIRE(copter.bf_roll_cd == Approx(1200.0f));
    REQUIRE(copter.bf_pitch_cd == Approx(0.0f));
    REQUIRE(copter.bf_yaw_rate_cds == Approx(200.0f));
    REQUIRE_FALSE(copter.input_euler_angle_roll_pitch_euler_rate_yaw);

    in.tailsitter_input_type =
        static_cast<std::int8_t>(kTailsitterInputBfRoll | kTailsitterInputPlane);
    in.nav_pitch_cd = 0.0f;
    auto plane0 = multicopter_attitude_rate_update(in);
    REQUIRE(plane0.bf_roll_plane);
    REQUIRE(plane0.y2r_scale == Approx(1.0f));
    REQUIRE(plane0.bf_yaw_rate_cds == Approx(1200.0f));
    REQUIRE(plane0.bf_roll_cd == Approx(-200.0f));
    REQUIRE(plane0.bf_pitch_cd == Approx(0.0f));

    in.nav_pitch_cd = 9000.0f;
    in.command_model_pilot_rate = kCommandModelPilotRateDefault;
    auto plane90 = multicopter_attitude_rate_update(in);
    REQUIRE(plane90.y2r_scale == Approx(4500.0f / 10000.0f));
    REQUIRE(plane90.bf_yaw_rate_cds == Approx(1200.0f / (4500.0f / 10000.0f)));
    REQUIRE(plane90.bf_roll_cd == Approx(-(4500.0f / 10000.0f) * 200.0f));
    REQUIRE(plane90.bf_pitch_cd == Approx(9000.0f));

    in.tailsitter_max_roll_angle = 30.0f;
    auto limited = multicopter_attitude_rate_update(in);
    REQUIRE(limited.y2r_scale == Approx(3000.0f / 10000.0f));
}

TEST_CASE("multicopter_attitude_rate_update yaw-target from transition", "[quadplane][stabilize][att]") {
    AttitudeRateInputs in{};
    in.in_vtol_mode = false;
    in.transition_update_yaw_target = true;
    in.yaw_target_cd = 4500.0f;
    in.nav_roll_cd = 100.0f;
    in.nav_pitch_cd = 50.0f;
    in.yaw_rate_cds = 9.0f;
    auto tick = multicopter_attitude_rate_update(in);
    REQUIRE(tick.use_multicopter_control);
    REQUIRE(tick.use_yaw_target);
    REQUIRE(tick.set_pilot_yaw_rate_time_constant);
    REQUIRE(tick.input_euler_angle_roll_pitch_yaw);
    REQUIRE_FALSE(tick.input_euler_angle_roll_pitch_euler_rate_yaw);
    REQUIRE(tick.roll_cd == Approx(100.0f));
    REQUIRE(tick.pitch_cd == Approx(50.0f));
    REQUIRE(tick.yaw_cd_or_rate == Approx(4500.0f));

    in.force_fw_control_recovery = true;
    in.fw_roll_pid_target = 0.1f;
    tick = multicopter_attitude_rate_update(in);
    REQUIRE_FALSE(tick.use_multicopter_control);
    REQUIRE_FALSE(tick.use_yaw_target);
    REQUIRE(tick.input_rate_bf_roll_pitch_yaw_no_shaping);

    QuadPlane qp{1};
    AttitudeRateInputs wired{};
    wired.in_vtol_mode = false;
    wired.transition_update_yaw_target = true;
    wired.yaw_target_cd = 1200.0f;
    const auto qp_tick = qp.multicopter_attitude_rate_update(wired);
    REQUIRE(qp_tick.use_yaw_target);
    REQUIRE(qp_tick.yaw_cd_or_rate == Approx(1200.0f));
}
