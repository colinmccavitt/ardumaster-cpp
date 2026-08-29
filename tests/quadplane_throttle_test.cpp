#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_auto_vtol_mission.hpp>
#include <fwcpp/quadplane/quadplane_throttle.hpp>
#include <fwcpp/quadplane_transition/transition_fsm.hpp>
#include <fwcpp/quadplane_transition/transition_state.hpp>

using Catch::Approx;
using fwcpp::math::Vector3f;
using fwcpp::quadplane::AirMode;
using fwcpp::quadplane::DesiredSpoolState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::ThrottleHoverInputs;
using fwcpp::quadplane::ThrottleMixInputs;
using fwcpp::quadplane::ThrottleMixKind;
using fwcpp::quadplane::ThrottleSuppressionInputs;
using fwcpp::quadplane::get_throttle_input;
using fwcpp::quadplane::kHoverUpdateTc;
using fwcpp::quadplane::kLandCheckGravityMss;
using fwcpp::quadplane::kMavCmdNavVtolTakeoff;
using fwcpp::quadplane::kThrottleMixMax;
using fwcpp::quadplane::update_throttle_hover;
using fwcpp::quadplane::update_throttle_mix;
using fwcpp::quadplane::update_throttle_suppression;
using fwcpp::quadplane_transition::TransitionState;

static ThrottleSuppressionInputs suppress_idle_in() {
    ThrottleSuppressionInputs in{};
    in.now_ms = 10000;
    in.desired_spool = DesiredSpoolState::kThrottleUnlimited;
    in.throttle_suppressed = true;
    in.available = true;
    return in;
}

static ThrottleHoverInputs hover_ok() {
    ThrottleHoverInputs in{};
    in.available = true;
    in.armed = true;
    in.is_flying_vtol = true;
    in.now_ms = 1000;
    in.last_pidz_active_ms = 990;
    in.motors_throttle = 0.5f;
    in.have_airspeed = true;
    in.aspeed = 1.0f;
    in.airspeed_min = 10.0f;
    return in;
}

static ThrottleMixInputs mix_auto_in() {
    ThrottleMixInputs in{};
    in.allow_update_throttle_mix = true;
    in.armed = true;
    in.vel_desired_U_ms = -1.0f;
    return in;
}

TEST_CASE("get_throttle_input reverses when requested", "[quadplane][throttle]") {
    REQUIRE(get_throttle_input(40.0f, false) == Approx(40.0f));
    REQUIRE(get_throttle_input(40.0f, true) == Approx(-40.0f));
    REQUIRE(get_throttle_input(0.0f, true) == Approx(0.0f));

    QuadPlane qp{1};
    REQUIRE(qp.get_throttle_input(-12.0f, true) == Approx(12.0f));
}

TEST_CASE("update_throttle_suppression recent-active returns", "[quadplane][throttle][suppress]") {
    auto in = suppress_idle_in();
    std::uint32_t last_ms = 9000;
    const auto tick = update_throttle_suppression(last_ms, in);
    REQUIRE(tick.early_return);
    REQUIRE_FALSE(tick.idle);
    REQUIRE(last_ms == 9000);

    QuadPlane qp{1};
    qp.motors_output_state_mut().last_motors_active_ms = 9500;
    in.now_ms = 10000;
    const auto wired = qp.update_throttle_suppression(in);
    REQUIRE(wired.early_return);
    REQUIRE(qp.motors_output_state().last_motors_active_ms == 9500);
}

TEST_CASE("update_throttle_suppression spool below unlimited returns", "[quadplane][throttle][suppress]") {
    auto in = suppress_idle_in();
    in.desired_spool = DesiredSpoolState::kGroundIdle;
    std::uint32_t last_ms = 0;
    const auto idle_spool = update_throttle_suppression(last_ms, in);
    REQUIRE(idle_spool.early_return);
    REQUIRE_FALSE(idle_spool.idle);

    in.desired_spool = DesiredSpoolState::kShutDown;
    REQUIRE(update_throttle_suppression(last_ms, in).early_return);
}

TEST_CASE("update_throttle_suppression guided_wait idles", "[quadplane][throttle][suppress]") {
    auto in = suppress_idle_in();
    in.guided_wait_takeoff = true;
    in.height_above_ground_m = 20.0f;
    std::uint32_t last_ms = 100;
    const auto tick = update_throttle_suppression(last_ms, in);
    REQUIRE(tick.idle);
    REQUIRE(tick.set_desired_spool);
    REQUIRE(tick.desired_spool == DesiredSpoolState::kGroundIdle);
    REQUIRE(tick.set_throttle);
    REQUIRE(tick.throttle == Approx(0.0f));
    REQUIRE(last_ms == 0);

    QuadPlane qp{1};
    qp.set_guided_wait_takeoff(true);
    qp.motors_output_state_mut().last_motors_active_ms = 50;
    auto wired_in = suppress_idle_in();
    wired_in.height_above_ground_m = 20.0f;
    const auto wired = qp.update_throttle_suppression(wired_in);
    REQUIRE(wired.idle);
    REQUIRE(qp.motors_output_state().last_motors_active_ms == 0);
}

TEST_CASE("update_throttle_suppression throttle input keeps motors", "[quadplane][throttle][suppress]") {
    auto in = suppress_idle_in();
    in.throttle_control_in = 25.0f;
    in.arming_check_throttle = true;
    std::uint32_t last_ms = 0;
    REQUIRE(update_throttle_suppression(last_ms, in).early_return);

    in.arming_check_throttle = false;
    in.is_vtol_man_throttle = true;
    REQUIRE(update_throttle_suppression(last_ms, in).early_return);

    in.is_vtol_man_throttle = false;
    in.throttle_norm_input_dz = 0.2f;
    REQUIRE(update_throttle_suppression(last_ms, in).early_return);

    in.throttle_norm_input_dz = 0.0f;
    REQUIRE(update_throttle_suppression(last_ms, in).idle);
}

TEST_CASE("update_throttle_suppression height and takeoff keep motors", "[quadplane][throttle][suppress]") {
    auto in = suppress_idle_in();
    in.height_above_ground_m = 5.1f;
    std::uint32_t last_ms = 0;
    REQUIRE(update_throttle_suppression(last_ms, in).early_return);

    in.height_above_ground_m = 5.0f;
    in.mode_auto = true;
    in.nav_cmd_id = kMavCmdNavVtolTakeoff;
    REQUIRE(update_throttle_suppression(last_ms, in).early_return);

    in.mode_auto = false;
    REQUIRE(update_throttle_suppression(last_ms, in).idle);
}

TEST_CASE("update_throttle_suppression idle zeros spool and last active", "[quadplane][throttle][suppress]") {
    auto in = suppress_idle_in();
    std::uint32_t last_ms = 1;
    const auto tick = update_throttle_suppression(last_ms, in);
    REQUIRE(tick.idle);
    REQUIRE_FALSE(tick.early_return);
    REQUIRE(tick.desired_spool == DesiredSpoolState::kGroundIdle);
    REQUIRE(tick.set_throttle);
    REQUIRE(tick.throttle == Approx(0.0f));
    REQUIRE(last_ms == 0);
}

TEST_CASE("update_throttle_hover unavailable and not flying return", "[quadplane][throttle][hover]") {
    auto in = hover_ok();
    in.available = false;
    REQUIRE(update_throttle_hover(in).early_return);

    in = hover_ok();
    in.armed = false;
    REQUIRE(update_throttle_hover(in).early_return);

    in = hover_ok();
    in.is_flying_vtol = false;
    REQUIRE(update_throttle_hover(in).early_return);

    QuadPlane qp{1};
    auto wired = hover_ok();
    REQUIRE(qp.update_throttle_hover(wired).early_return);
}

TEST_CASE("update_throttle_hover climb and fw throttle return", "[quadplane][throttle][hover]") {
    auto in = hover_ok();
    in.vel_desired_U_ms = 0.5f;
    REQUIRE(update_throttle_hover(in).early_return);

    in = hover_ok();
    in.fw_throttle_scaled = 15.0f;
    in.throttle_min = 0.0f;
    REQUIRE(update_throttle_hover(in).early_return);

    in.tailsitter_enabled = true;
    REQUIRE_FALSE(update_throttle_hover(in).early_return);
    REQUIRE(update_throttle_hover(in).update_throttle_hover);
}

TEST_CASE("update_throttle_hover pidz stale returns", "[quadplane][throttle][hover]") {
    auto in = hover_ok();
    in.now_ms = 1000;
    in.last_pidz_active_ms = 979;
    REQUIRE(update_throttle_hover(in).early_return);

    in.last_pidz_active_ms = 980;
    REQUIRE_FALSE(update_throttle_hover(in).early_return);
}

TEST_CASE("update_throttle_hover level hover updates", "[quadplane][throttle][hover]") {
    auto in = hover_ok();
    const auto tick = update_throttle_hover(in);
    REQUIRE_FALSE(tick.early_return);
    REQUIRE(tick.update_throttle_hover);
    REQUIRE(tick.hover_tc == Approx(kHoverUpdateTc));

    in.motors_throttle = 0.0f;
    REQUIRE_FALSE(update_throttle_hover(in).update_throttle_hover);

    in = hover_ok();
    in.vel_z_up_cms = 60.0f;
    REQUIRE_FALSE(update_throttle_hover(in).update_throttle_hover);

    in = hover_ok();
    in.roll_cd = 500;
    REQUIRE_FALSE(update_throttle_hover(in).update_throttle_hover);

    in = hover_ok();
    in.have_airspeed = false;
    REQUIRE_FALSE(update_throttle_hover(in).update_throttle_hover);

    in = hover_ok();
    in.aspeed = 3.0f;
    REQUIRE_FALSE(update_throttle_hover(in).update_throttle_hover);

    QuadPlane qp{1};
    REQUIRE(qp.setup());
    qp.z_ctrl_mut().last_pidz_active_ms = 990;
    auto wired = hover_ok();
    wired.is_flying_vtol = true;
    const auto qp_tick = qp.update_throttle_hover(wired);
    REQUIRE(qp_tick.update_throttle_hover);
}

TEST_CASE("update_throttle_mix disarmed min", "[quadplane][throttle][mix]") {
    auto in = mix_auto_in();
    in.armed = false;
    const auto tick = update_throttle_mix(in);
    REQUIRE(tick.filter_applied);
    REQUIRE(tick.mix == ThrottleMixKind::kMin);
    REQUIRE(tick.accel_ef_mss.z == Approx(kLandCheckGravityMss));

    in.allow_update_throttle_mix = false;
    const auto blocked = update_throttle_mix(in);
    REQUIRE(blocked.filter_applied);
    REQUIRE(blocked.early_return);
    REQUIRE(blocked.mix == ThrottleMixKind::kNone);
}

TEST_CASE("update_throttle_mix man min vs man", "[quadplane][throttle][mix]") {
    auto in = mix_auto_in();
    in.is_vtol_man_throttle = true;
    in.throttle_control_in = 0.0f;
    REQUIRE(update_throttle_mix(in).mix == ThrottleMixKind::kMin);

    in.air_mode_active = true;
    REQUIRE(update_throttle_mix(in).mix == ThrottleMixKind::kMan);

    in.air_mode_active = false;
    in.throttle_control_in = 10.0f;
    REQUIRE(update_throttle_mix(in).mix == ThrottleMixKind::kMan);

    QuadPlane qp{1};
    qp.set_air_mode(AirMode::kOn);
    auto wired = mix_auto_in();
    wired.is_vtol_man_throttle = true;
    wired.throttle_control_in = 0.0f;
    REQUIRE(qp.update_throttle_mix(wired).mix == ThrottleMixKind::kMan);
}

TEST_CASE("update_throttle_mix land sequence vs land final", "[quadplane][throttle][mix]") {
    auto in = mix_auto_in();
    in.in_vtol_land_sequence = true;
    in.in_vtol_land_final = false;
    in.vel_desired_U_ms = -1.0f;
    in.filtered_accel_length = 0.0f;
    auto tick = update_throttle_mix(in);
    REQUIRE(tick.mix == ThrottleMixKind::kMax);
    REQUIRE(tick.mix_max == Approx(kThrottleMixMax));

    in.in_vtol_land_final = true;
    tick = update_throttle_mix(in);
    REQUIRE(tick.mix == ThrottleMixKind::kMin);
}

TEST_CASE("update_throttle_mix large angle requests max", "[quadplane][throttle][mix]") {
    auto in = mix_auto_in();
    in.angle_target_cd = Vector3f{1600.0f, 0.0f, 0.0f};
    auto tick = update_throttle_mix(in);
    REQUIRE(tick.mix == ThrottleMixKind::kMax);

    in.angle_target_cd = {};
    in.att_error_deg = 30.1f;
    REQUIRE(update_throttle_mix(in).mix == ThrottleMixKind::kMax);

    in.att_error_deg = 0.0f;
    in.filtered_accel_length = 3.1f;
    REQUIRE(update_throttle_mix(in).mix == ThrottleMixKind::kMax);

    in.filtered_accel_length = 0.0f;
    in.vel_desired_U_ms = 0.0f;
    REQUIRE(update_throttle_mix(in).mix == ThrottleMixKind::kMax);

    in.vel_desired_U_ms = -0.1f;
    REQUIRE(update_throttle_mix(in).mix == ThrottleMixKind::kMin);

    QuadPlane qp{1};
    REQUIRE(qp.slt_transition_mut().set_state(TransitionState::kDone));
    qp.set_assisted_flight(true);
    auto wired = mix_auto_in();
    wired.angle_target_cd = Vector3f{1600.0f, 0.0f, 0.0f};
    REQUIRE(qp.update_throttle_mix(wired).mix == ThrottleMixKind::kMax);

    REQUIRE(qp.slt_transition_mut().set_state(TransitionState::kAirspeedWait));
    REQUIRE(qp.update_throttle_mix(wired).early_return);
}
