#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_takeoff_controller.hpp>

using Catch::Approx;
using fwcpp::Location;
using fwcpp::quadplane::DesiredSpoolState;
using fwcpp::quadplane::PosControlState;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::SetupTargetPositionInputs;
using fwcpp::quadplane::TakeoffArmMethod;
using fwcpp::quadplane::TakeoffControllerInputs;
using fwcpp::quadplane::TakeoffNavState;
using fwcpp::quadplane::WaypointControllerInputs;
using fwcpp::quadplane::kGuidedTakeoffAltMarginCm;
using fwcpp::quadplane::kPilotAccelZMssDefault;
using fwcpp::quadplane::kPilotSpeedZMaxUpMsDefault;
using fwcpp::quadplane::kTakeoffLastRunGapMs;
using fwcpp::quadplane::kTakeoffRudderWarningTimeoutMs;
using fwcpp::quadplane::kTakeoffWpNavSpeedUpMsDefault;
using fwcpp::quadplane::kWaypointDestRefreshMs;
using fwcpp::quadplane::setup_target_position;
using fwcpp::quadplane::takeoff_controller;
using fwcpp::quadplane::waypoint_controller;

static Location loc_alt(std::int32_t alt_cm) {
    Location loc{};
    loc.alt = alt_cm;
    return loc;
}

static SetupTargetPositionInputs origin_and_wp(std::int32_t origin_alt_cm, std::int32_t wp_alt_cm) {
    SetupTargetPositionInputs in{};
    in.has_origin = true;
    in.origin = loc_alt(origin_alt_cm);
    in.next_wp = loc_alt(wp_alt_cm);
    return in;
}

static TakeoffControllerInputs armed_unlimited() {
    TakeoffControllerInputs in{};
    in.now_ms = 5000;
    in.armed_and_safety_off = true;
    in.desired_spool = DesiredSpoolState::kThrottleUnlimited;
    in.target = origin_and_wp(1000, 2000);
    in.pos_control_roll_cd = 120.f;
    in.pos_control_pitch_cd = -40.f;
    return in;
}

TEST_CASE("disarmed takeoff returns after zero nav", "[quadplane][takeoff]") {
    TakeoffNavState nav{};
    nav.takeoff_last_run_ms = 99;
    nav.takeoff_start_time_ms = 11;
    PosControlState pc{};
    TakeoffControllerInputs in = armed_unlimited();
    in.armed_and_safety_off = false;
    in.pos_control_roll_cd = 900.f;

    const auto tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.nav_roll_cd == 0.f);
    REQUIRE(tick.nav_pitch_cd == 0.f);
    REQUIRE(tick.early_return);
    REQUIRE_FALSE(tick.setup_target_position);
    REQUIRE_FALSE(tick.run_xy_controller);
    REQUIRE_FALSE(tick.run_z_controller);
    REQUIRE(nav.takeoff_last_run_ms == 99);
    REQUIRE(nav.takeoff_start_time_ms == 11);
    REQUIRE(pc.target_ned_d_m == 0.f);

    QuadPlane qp{1};
    qp.takeoff_nav_mut().takeoff_last_run_ms = 7;
    const auto wired = qp.takeoff_controller(in);
    REQUIRE(wired.early_return);
    REQUIRE(qp.takeoff_nav().takeoff_last_run_ms == 7);
}

TEST_CASE("spool wait tiltrotor guided takeoff", "[quadplane][takeoff]") {
    TakeoffNavState nav{};
    PosControlState pc{};
    TakeoffControllerInputs in = armed_unlimited();
    in.desired_spool = DesiredSpoolState::kGroundIdle;
    in.mode_is_guided = true;
    in.guided_takeoff = true;
    in.tiltrotor_enabled = true;
    in.tiltrotor_fully_up = false;
    in.now_ms = 2500;

    const auto tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.early_return);
    REQUIRE_FALSE(tick.setup_target_position);
    REQUIRE_FALSE(tick.set_desired_spool);
    REQUIRE(nav.takeoff_start_time_ms == 2500);
    REQUIRE(nav.takeoff_last_run_ms == 0);

    in.tiltrotor_fully_up = true;
    nav = {};
    const auto proceed = takeoff_controller(nav, pc, in);
    REQUIRE_FALSE(proceed.early_return);
    REQUIRE(proceed.setup_target_position);
    REQUIRE(proceed.run_z_controller);
}

TEST_CASE("rudder-arm wait holds ground idle", "[quadplane][takeoff]") {
    TakeoffNavState nav{};
    PosControlState pc{};
    TakeoffControllerInputs in = armed_unlimited();
    in.desired_spool = DesiredSpoolState::kShutDown;
    in.last_arm_method = TakeoffArmMethod::kRudder;
    in.seen_neutral_rudder = false;
    in.now_ms = 4000;

    auto tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.early_return);
    REQUIRE(tick.rudder_waiting);
    REQUIRE(tick.rudder_warn);
    REQUIRE(tick.set_desired_spool);
    REQUIRE(tick.desired_spool == DesiredSpoolState::kGroundIdle);
    REQUIRE(nav.takeoff_start_time_ms == 4000);
    REQUIRE(nav.rudder_takeoff_warn_ms == 4000);
    REQUIRE_FALSE(tick.setup_target_position);

    in.now_ms = 4000 + kTakeoffRudderWarningTimeoutMs;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.rudder_waiting);
    REQUIRE_FALSE(tick.rudder_warn);

    in.now_ms = 4000 + kTakeoffRudderWarningTimeoutMs + 1;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.rudder_warn);
    REQUIRE(nav.rudder_takeoff_warn_ms == in.now_ms);

    nav.takeoff_last_run_ms = in.now_ms;
    in.now_ms += 10;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE_FALSE(tick.early_return);
    REQUIRE(tick.setup_target_position);

    nav = {};
    in.seen_neutral_rudder = true;
    in.now_ms = 100;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE_FALSE(tick.rudder_waiting);
    REQUIRE(tick.setup_target_position);
}

TEST_CASE("navalt min latches start alt and suppresses navigation", "[quadplane][takeoff]") {
    TakeoffNavState nav{};
    PosControlState pc{};
    TakeoffControllerInputs in = armed_unlimited();
    in.takeoff_navalt_min_m = 5.f;
    in.current_alt_cm = 1000;
    in.now_ms = 1000;

    auto tick = takeoff_controller(nav, pc, in);
    REQUIRE(nav.takeoff_start_alt_m == Approx(10.f));
    REQUIRE(nav.takeoff_last_run_ms == 1000);
    REQUIRE(tick.no_navigation);
    REQUIRE(tick.ne_relax);
    REQUIRE_FALSE(tick.input_vel_accel_ne);
    REQUIRE_FALSE(tick.assign_tilt_to_fwd_thr);
    REQUIRE(tick.nav_roll_cd == 0.f);
    REQUIRE(tick.nav_pitch_cd == 0.f);
    REQUIRE(tick.run_xy_controller);
    REQUIRE(tick.run_z_controller);

    in.current_alt_cm = 1400;
    in.now_ms = 1500;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE(nav.takeoff_start_alt_m == Approx(10.f));
    REQUIRE(tick.no_navigation);

    in.current_alt_cm = 1600;
    in.now_ms = 1600;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE_FALSE(tick.no_navigation);
    REQUIRE(tick.input_vel_accel_ne);
    REQUIRE(tick.assign_tilt_to_fwd_thr);
    REQUIRE(tick.nav_roll_cd == 120.f);
    REQUIRE(tick.nav_pitch_cd == -40.f);

    in.current_alt_cm = 1100;
    in.now_ms = 1600 + kTakeoffLastRunGapMs + 1;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE(nav.takeoff_start_alt_m == Approx(11.f));
    REQUIRE(tick.no_navigation);
}

TEST_CASE("guided takeoff aims at next_WP alt; otherwise climb speed_up", "[quadplane][takeoff]") {
    TakeoffNavState nav{};
    PosControlState pc{};
    TakeoffControllerInputs in = armed_unlimited();
    in.wp_nav_default_speed_up_ms = 3.25f;

    auto tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.set_climb_rate);
    REQUIRE(tick.climb_rate_ms == Approx(3.25f));
    REQUIRE_FALSE(tick.input_pos_vel_accel_d);
    REQUIRE(tick.run_z_controller);

    in.mode_is_guided = true;
    in.guided_takeoff = true;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE_FALSE(tick.set_climb_rate);
    REQUIRE(tick.input_pos_vel_accel_d);
    REQUIRE(tick.vel_d_ms == 0.f);
    REQUIRE(tick.pos_d_m ==
            Approx(-static_cast<float>(kGuidedTakeoffAltMarginCm + 2000 - 1000) * 0.01f));

    in.target.has_origin = false;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.set_climb_rate);
    REQUIRE(tick.climb_rate_ms == Approx(3.25f));
    REQUIRE_FALSE(tick.input_pos_vel_accel_d);

    QuadPlane qp{1};
    REQUIRE(qp.setup());
    qp.set_guided_takeoff(true);
    TakeoffControllerInputs wired = in;
    wired.target.has_origin = true;
    wired.mode_is_guided = true;
    const auto qp_tick = qp.takeoff_controller(wired);
    REQUIRE(qp_tick.input_pos_vel_accel_d);
    REQUIRE(qp.guided_takeoff());
}

TEST_CASE("waypoint dest refresh on loc change or 500ms", "[quadplane][waypoint]") {
    TakeoffNavState nav{};
    PosControlState pc{};
    WaypointControllerInputs in{};
    in.now_ms = 100;
    in.target = origin_and_wp(0, 1500);
    in.wp_nav_roll_cd = 11.f;
    in.wp_nav_pitch_cd = 22.f;
    in.wp_nav_yaw_cd = 333.f;
    in.assist_climb_rate_cms = 250.f;

    auto tick = waypoint_controller(nav, pc, in);
    REQUIRE(tick.set_wp_destination_ned);
    REQUIRE(tick.dest_ned_d_m == Approx(-15.f));
    REQUIRE(tick.update_wpnav);
    REQUIRE(tick.nav_roll_cd == 11.f);
    REQUIRE(tick.nav_pitch_cd == 22.f);
    REQUIRE(tick.assign_tilt_to_fwd_thr);
    REQUIRE(tick.disable_yaw_rate_time_constant);
    REQUIRE(tick.input_euler_angle_roll_pitch_yaw);
    REQUIRE(tick.attitude_yaw_cd == 333.f);
    REQUIRE(tick.set_climb_rate);
    REQUIRE(tick.climb_rate_ms == Approx(2.5f));
    REQUIRE(tick.run_z_controller);
    REQUIRE(nav.last_loiter_ms == 100);
    REQUIRE(nav.last_auto_target.same_loc_as(in.target.next_wp));

    in.now_ms = 200;
    tick = waypoint_controller(nav, pc, in);
    REQUIRE_FALSE(tick.set_wp_destination_ned);
    REQUIRE(nav.last_loiter_ms == 200);

    in.now_ms = 200 + kWaypointDestRefreshMs + 1;
    tick = waypoint_controller(nav, pc, in);
    REQUIRE(tick.set_wp_destination_ned);

    in.target.next_wp.lat = 12345;
    in.now_ms += 10;
    tick = waypoint_controller(nav, pc, in);
    REQUIRE(tick.set_wp_destination_ned);
    REQUIRE(nav.last_auto_target.lat == 12345);

    in.vtol_roll_pitch_limited = true;
    tick = waypoint_controller(nav, pc, in);
    REQUIRE(tick.ne_set_externally_limited);
}

TEST_CASE("LAND_FINAL skips unlimited spool in setup_target_position", "[quadplane][takeoff]") {
    PosControlState pc{};
    auto in = origin_and_wp(500, 2500);
    in.correction_north_m = 1.5f;
    in.correction_east_m = -0.25f;
    in.pos_state = PositionControlState::kLandFinal;

    auto tick = setup_target_position(pc, in);
    REQUIRE_FALSE(tick.spool_throttle_unlimited);
    REQUIRE(tick.target_ned_n_m == Approx(1.5f));
    REQUIRE(tick.target_ned_e_m == Approx(-0.25f));
    REQUIRE(tick.target_ned_d_m == Approx(-20.f));
    REQUIRE(pc.target_ned_d_m == Approx(-20.f));
    REQUIRE(tick.d_set_max_speed_accel);
    REQUIRE(tick.d_set_correction_speed_accel);
    REQUIRE(tick.d_max_speed_dn_m ==
            Approx(static_cast<float>(fwcpp::quadplane::get_pilot_velocity_z_max_dn_m(
                fwcpp::quadplane::kPilotSpeedZMaxDnMsDefault, kPilotSpeedZMaxUpMsDefault))));
    REQUIRE(tick.d_max_speed_up_ms == Approx(kPilotSpeedZMaxUpMsDefault));
    REQUIRE(tick.d_max_accel_z_mss == Approx(kPilotAccelZMssDefault));

    in.pos_state = PositionControlState::kApproach;
    in.in_vtol_land_approach = true;
    tick = setup_target_position(pc, in);
    REQUIRE_FALSE(tick.spool_throttle_unlimited);

    in.pos_state = PositionControlState::kAirbrake;
    tick = setup_target_position(pc, in);
    REQUIRE(tick.spool_throttle_unlimited);

    in.in_vtol_land_approach = false;
    in.pos_state = PositionControlState::kNone;
    tick = setup_target_position(pc, in);
    REQUIRE(tick.spool_throttle_unlimited);

    in.has_origin = false;
    in.next_wp.alt = 800;
    tick = setup_target_position(pc, in);
    REQUIRE(tick.target_ned_d_m == Approx(-8.f));

    QuadPlane qp{1};
    qp.poscontrol_mut().state = PositionControlState::kLandFinal;
    qp.poscontrol_mut().correction_north_m = 4.f;
    const auto wired = qp.setup_target_position(origin_and_wp(0, 100));
    REQUIRE_FALSE(wired.spool_throttle_unlimited);
    REQUIRE(wired.target_ned_n_m == Approx(4.f));
    REQUIRE(qp.poscontrol().target_ned_d_m == Approx(-1.f));
}

TEST_CASE("esc telem motor check holds ground idle until passed", "[quadplane][takeoff]") {
    TakeoffNavState nav{};
    PosControlState pc{};
    TakeoffControllerInputs in = armed_unlimited();
    in.desired_spool = DesiredSpoolState::kShutDown;
    in.motor_check_passed = false;
    in.now_ms = 800;

    auto tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.early_return);
    REQUIRE(tick.desired_spool == DesiredSpoolState::kGroundIdle);
    REQUIRE(nav.takeoff_start_time_ms == 800);

    in.motor_check_passed = true;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE_FALSE(tick.early_return);
    REQUIRE(tick.setup_target_position);
}

TEST_CASE("velocity match is used only when fresh", "[quadplane][takeoff]") {
    TakeoffNavState nav{};
    PosControlState pc{};
    TakeoffControllerInputs in = armed_unlimited();
    in.now_ms = 3000;
    in.last_velocity_match_ms = 2001;
    in.velocity_match_north_ms = 1.2f;
    in.velocity_match_east_ms = -0.4f;

    auto tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.vel_ne_n_ms == Approx(1.2f));
    REQUIRE(tick.vel_ne_e_ms == Approx(-0.4f));
    REQUIRE(tick.input_vel_accel_ne);

    in.last_velocity_match_ms = 2000;
    tick = takeoff_controller(nav, pc, in);
    REQUIRE(tick.vel_ne_n_ms == 0.f);
    REQUIRE(tick.vel_ne_e_ms == 0.f);
}
