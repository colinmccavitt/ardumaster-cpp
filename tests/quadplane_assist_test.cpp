#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_assist.hpp>
#include <fwcpp/quadplane/quadplane_land_detector.hpp>
#include <fwcpp/quadplane/quadplane_landing.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/quadplane/quadplane_pilot_input.hpp>
#include <fwcpp/quadplane/quadplane_stabilize.hpp>
#include <fwcpp/quadplane/quadplane_xy_controller.hpp>
#include <fwcpp/quadplane_transition/transition_base.hpp>

using Catch::Approx;
using fwcpp::quadplane::AssistClimbInputs;
using fwcpp::quadplane::AutoYawRateInputs;
using fwcpp::quadplane::DesiredSpoolState;
using fwcpp::quadplane::FlyingVtolInputs;
using fwcpp::quadplane::PosControlLandStub;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::ShouldRelaxInputs;
using fwcpp::quadplane::SpoolState;
using fwcpp::quadplane::WeathervaneStub;
using fwcpp::quadplane::WeathervaneYawInputs;
using fwcpp::quadplane::ZCtrlState;
using fwcpp::quadplane::assist_climb_rate_cms;
using fwcpp::quadplane::desired_auto_yaw_rate_cds;
using fwcpp::quadplane::get_weathervane_yaw_rate_cds;
using fwcpp::quadplane::is_flying_vtol;
using fwcpp::quadplane::kAirspeedMinDefault;
using fwcpp::quadplane::kAssistClimbAltSpread;
using fwcpp::quadplane::kAssistClimbRampMs;
using fwcpp::quadplane::kAssistMinAspeedMs;
using fwcpp::quadplane::kCommandModelPilotRateDefault;
using fwcpp::quadplane::kDefaultSpeedDownMs;
using fwcpp::quadplane::kDefaultSpeedUpMs;
using fwcpp::quadplane::kFlybywireClimbRateDefault;
using fwcpp::quadplane::kFlyingVtolLandDetectMs;
using fwcpp::quadplane::kFlyingVtolThrottle;
using fwcpp::quadplane::kGravityMss;
using fwcpp::quadplane::kMavCmdNavVtolTakeoff;
using fwcpp::quadplane::kMavCmdNavWaypoint;
using fwcpp::quadplane::kPitchLimitMaxDefault;
using fwcpp::quadplane::kShouldRelaxLowerLimitMs;
using fwcpp::quadplane::kWeathervaneOutLimit;
using fwcpp::quadplane::kWeathervaneRateFrac;
using fwcpp::quadplane::kWeathervaneScale;

static FlyingVtolInputs live_vtol() {
    FlyingVtolInputs in{};
    in.available = true;
    in.spool = SpoolState::kThrottleUnlimited;
    return in;
}

static ZCtrlState ramped_z() {
    ZCtrlState z{};
    z.last_pidz_init_ms = 1000;
    z.last_pidz_active_ms = 1000 + kAssistClimbRampMs;
    return z;
}

static WeathervaneYawInputs wv_live() {
    WeathervaneYawInputs in{};
    in.in_vtol_mode = true;
    in.allow_weathervane = true;
    in.armed = true;
    in.desired_spool = DesiredSpoolState::kThrottleUnlimited;
    in.relax.now_ms = 10;
    in.relax.throttle = 0.5f;
    return in;
}

TEST_CASE("is_flying_vtol unavailable and spool shutdown", "[quadplane][assist][flying]") {
    PosControlLandStub land{};
    FlyingVtolInputs in = live_vtol();
    in.motors_throttle = 0.5f;

    in.available = false;
    REQUIRE_FALSE(is_flying_vtol(land, in));

    in.available = true;
    in.spool = SpoolState::kShutDown;
    REQUIRE_FALSE(is_flying_vtol(land, in));

    in.spool = SpoolState::kGroundIdle;
    REQUIRE(is_flying_vtol(land, in));
}

TEST_CASE("is_flying_vtol throttle airmode guided man land-detect", "[quadplane][assist][flying]") {
    PosControlLandStub land{};
    FlyingVtolInputs in = live_vtol();

    in.motors_throttle = kFlyingVtolThrottle;
    REQUIRE_FALSE(is_flying_vtol(land, in));
    in.motors_throttle = kFlyingVtolThrottle + 0.001f;
    REQUIRE(is_flying_vtol(land, in));

    in.motors_throttle = 0.f;
    in.is_vtol_man_throttle = true;
    REQUIRE_FALSE(is_flying_vtol(land, in));
    in.air_mode_active = true;
    REQUIRE(is_flying_vtol(land, in));

    in.is_vtol_man_throttle = false;
    in.air_mode_active = false;
    in.mode_guided = true;
    REQUIRE_FALSE(is_flying_vtol(land, in));
    in.guided_takeoff = true;
    REQUIRE(is_flying_vtol(land, in));

    in.mode_guided = false;
    in.guided_takeoff = false;
    in.is_vtol_man_mode = true;
    in.throttle_control_in = 0.f;
    REQUIRE_FALSE(is_flying_vtol(land, in));
    in.throttle_control_in = 1.f;
    REQUIRE(is_flying_vtol(land, in));
    in.reversed_throttle = true;
    REQUIRE_FALSE(is_flying_vtol(land, in));

    in.is_vtol_man_mode = false;
    in.reversed_throttle = false;
    in.throttle_control_in = 0.f;
    in.in_vtol_mode = true;
    land.lower_limit_start_ms = 100;
    in.now_ms = 100 + kFlyingVtolLandDetectMs;
    REQUIRE_FALSE(is_flying_vtol(land, in));
    in.now_ms = 100 + kFlyingVtolLandDetectMs + 1;
    REQUIRE(is_flying_vtol(land, in));

    in.in_vtol_mode = false;
    REQUIRE_FALSE(is_flying_vtol(land, in));
}

TEST_CASE("assist_climb_rate_cms auto vs pitch throttle and 2s ramp", "[quadplane][assist][climb]") {
    AssistClimbInputs in{};
    in.does_auto_throttle = true;
    in.altitude_error_cm = 400.f;

    const float auto_full = 400.f * kAssistClimbAltSpread;
    REQUIRE(assist_climb_rate_cms(ramped_z(), in) == Approx(auto_full));

    in.altitude_error_cm = 10000.f;
    REQUIRE(assist_climb_rate_cms(ramped_z(), in) == Approx(kDefaultSpeedUpMs * 100.0f));
    in.altitude_error_cm = -10000.f;
    REQUIRE(assist_climb_rate_cms(ramped_z(), in) == Approx(-kDefaultSpeedDownMs * 100.0f));

    in.does_auto_throttle = false;
    in.nav_pitch_cd = 1250.f;
    in.pitch_limit_max = kPitchLimitMaxDefault;
    in.flybywire_climb_rate = kFlybywireClimbRateDefault;
    in.throttle_control_in = 0.5f;
    const float pitch_full = kFlybywireClimbRateDefault * (1250.f / (kPitchLimitMaxDefault * 100.0f)) * 0.5f;
    REQUIRE(assist_climb_rate_cms(ramped_z(), in) == Approx(pitch_full));

    in.reversed_throttle = true;
    REQUIRE(assist_climb_rate_cms(ramped_z(), in) == Approx(-pitch_full));

    in.reversed_throttle = false;
    ZCtrlState mid{};
    mid.last_pidz_init_ms = 0;
    mid.last_pidz_active_ms = 1000;
    const float ramped = fwcpp::math::linear_interpolate(0.0f, pitch_full, 1000.0f, 0.0f,
                                                         static_cast<float>(kAssistClimbRampMs));
    REQUIRE(assist_climb_rate_cms(mid, in) == Approx(ramped));

    ZCtrlState start{};
    REQUIRE(assist_climb_rate_cms(start, in) == Approx(0.f));
}

TEST_CASE("desired_auto_yaw_rate_cds sin vs tan and min aspeed", "[quadplane][assist][yaw]") {
    AutoYawRateInputs in{};
    in.have_airspeed = true;
    in.aspeed = 12.f;
    in.nav_roll_cd = 1500.f;
    const float roll_rad = fwcpp::math::cd_to_rad(1500.f);

    const float body = fwcpp::math::degrees(kGravityMss * std::sin(roll_rad) / 12.f) * 100.0f;
    const float earth = fwcpp::math::degrees(kGravityMss * std::tan(roll_rad) / 12.f) * 100.0f;
    REQUIRE(desired_auto_yaw_rate_cds(true, in) == Approx(body));
    REQUIRE(desired_auto_yaw_rate_cds(false, in) == Approx(earth));
    REQUIRE(std::fabs(earth) > std::fabs(body));

    in.aspeed = 4.f;
    const float min_aspeed = kAirspeedMinDefault;
    const float body_min = fwcpp::math::degrees(kGravityMss * std::sin(roll_rad) / min_aspeed) * 100.0f;
    REQUIRE(desired_auto_yaw_rate_cds(true, in) == Approx(body_min));

    in.have_airspeed = false;
    in.aspeed = 20.f;
    REQUIRE(desired_auto_yaw_rate_cds(true, in) == Approx(body_min));

    in.have_airspeed = true;
    in.aspeed = 0.2f;
    in.airspeed_min = 0.4f;
    const float floor_aspeed = kAssistMinAspeedMs;
    const float body_floor = fwcpp::math::degrees(kGravityMss * std::sin(roll_rad) / floor_aspeed) * 100.0f;
    REQUIRE(desired_auto_yaw_rate_cds(true, in) == Approx(body_floor));
}

TEST_CASE("get_weathervane_yaw_rate_cds gates reset", "[quadplane][assist][weathervane]") {
    PosControlLandStub land{};
    WeathervaneStub wv{};
    WeathervaneYawInputs in{};
    in.weathervane_ok = true;
    in.wv_output = 45.f;

    auto dead = get_weathervane_yaw_rate_cds(wv, land, in);
    REQUIRE(dead.reset);
    REQUIRE(wv.reset);
    REQUIRE(dead.rate_cds == Approx(0.f));

    auto check_gate = [&](auto mutate) {
        wv = {};
        land = {};
        WeathervaneYawInputs gated = wv_live();
        gated.weathervane_ok = true;
        gated.wv_output = 45.f;
        mutate(gated);
        const auto tick = get_weathervane_yaw_rate_cds(wv, land, gated);
        REQUIRE(tick.reset);
        REQUIRE(wv.reset);
        REQUIRE(tick.rate_cds == Approx(0.f));
        REQUIRE_FALSE(tick.is_takeoff);
    };

    check_gate([](WeathervaneYawInputs& g) { g.in_vtol_mode = false; });
    check_gate([](WeathervaneYawInputs& g) { g.allow_weathervane = false; });
    check_gate([](WeathervaneYawInputs& g) { g.armed = false; });
    check_gate([](WeathervaneYawInputs& g) { g.desired_spool = DesiredSpoolState::kGroundIdle; });
    check_gate([](WeathervaneYawInputs& g) { g.mode_qstabilize = true; });
    check_gate([](WeathervaneYawInputs& g) { g.in_qautotune = true; });
    check_gate([](WeathervaneYawInputs& g) { g.mode_qhover = true; });

    wv = {};
    land = {};
    land.lower_limit_start_ms = 10;
    WeathervaneYawInputs relax = wv_live();
    relax.weathervane_ok = true;
    relax.wv_output = 45.f;
    relax.relax.now_ms = 10 + kShouldRelaxLowerLimitMs + 1;
    relax.relax.throttle = 0.f;
    relax.relax.throttle_lower = true;
    relax.relax.throttle_mix_min = true;
    const auto relaxed = get_weathervane_yaw_rate_cds(wv, land, relax);
    REQUIRE(relaxed.reset);
    REQUIRE(wv.reset);
    REQUIRE(relaxed.rate_cds == Approx(0.f));

    REQUIRE(fwcpp::quadplane_transition::TransitionBaseDefaults::allow_weathervane());
}

TEST_CASE("get_weathervane_yaw_rate_cds scales 1/45 rate 0.5", "[quadplane][assist][weathervane]") {
    PosControlLandStub land{};
    WeathervaneStub wv{};
    WeathervaneYawInputs in = wv_live();
    in.weathervane_ok = true;
    in.wv_output = 45.f;
    in.in_vtol_auto = true;
    in.available = true;
    in.nav_cmd_id = kMavCmdNavVtolTakeoff;

    const auto mid = get_weathervane_yaw_rate_cds(wv, land, in);
    REQUIRE_FALSE(mid.reset);
    REQUIRE_FALSE(wv.reset);
    REQUIRE(mid.is_takeoff);
    REQUIRE(mid.rate_cds == Approx(45.f * kWeathervaneScale * kCommandModelPilotRateDefault * kWeathervaneRateFrac));
    REQUIRE(mid.rate_cds == Approx(50.f));

    in.wv_output = 90.f;
    REQUIRE(get_weathervane_yaw_rate_cds(wv, land, in).rate_cds ==
            Approx(2.f * kCommandModelPilotRateDefault * kWeathervaneRateFrac));

    in.wv_output = 4500.f;
    REQUIRE(get_weathervane_yaw_rate_cds(wv, land, in).rate_cds ==
            Approx(kWeathervaneOutLimit * kCommandModelPilotRateDefault * kWeathervaneRateFrac));

    in.wv_output = -90.f;
    REQUIRE(get_weathervane_yaw_rate_cds(wv, land, in).rate_cds ==
            Approx(-2.f * kCommandModelPilotRateDefault * kWeathervaneRateFrac));

    in.weathervane_ok = false;
    in.nav_cmd_id = kMavCmdNavWaypoint;
    const auto miss = get_weathervane_yaw_rate_cds(wv, land, in);
    REQUIRE_FALSE(miss.reset);
    REQUIRE_FALSE(miss.is_takeoff);
    REQUIRE(miss.rate_cds == Approx(0.f));
}

TEST_CASE("QuadPlane assist ticks wire pidz land and weathervane", "[quadplane][assist][wired]") {
    QuadPlane qp{1};
    REQUIRE(qp.setup());

    FlyingVtolInputs fly = live_vtol();
    fly.motors_throttle = 0.2f;
    REQUIRE(qp.is_flying_vtol(fly));

    qp.set_guided_takeoff(true);
    FlyingVtolInputs guided{};
    guided.spool = SpoolState::kThrottleUnlimited;
    guided.mode_guided = true;
    REQUIRE(qp.is_flying_vtol(guided));

    qp.z_ctrl_mut().last_pidz_init_ms = 0;
    qp.z_ctrl_mut().last_pidz_active_ms = kAssistClimbRampMs;
    AssistClimbInputs climb{};
    climb.does_auto_throttle = true;
    climb.altitude_error_cm = 200.f;
    REQUIRE(qp.assist_climb_rate_cms(climb) == Approx(20.f));

    AutoYawRateInputs yaw{};
    yaw.have_airspeed = true;
    yaw.aspeed = 10.f;
    yaw.nav_roll_cd = 1000.f;
    REQUIRE(qp.desired_auto_yaw_rate_cds(true, yaw) !=
            Approx(qp.desired_auto_yaw_rate_cds(false, yaw)));

    WeathervaneYawInputs wv = wv_live();
    wv.weathervane_ok = true;
    wv.wv_output = 45.f;
    const auto tick = qp.get_weathervane_yaw_rate_cds(wv);
    REQUIRE_FALSE(tick.reset);
    REQUIRE(tick.rate_cds == Approx(50.f));
    REQUIRE_FALSE(qp.weathervane().reset);

    WeathervaneYawInputs gated{};
    REQUIRE(qp.get_weathervane_yaw_rate_cds(gated).reset);
    REQUIRE(qp.weathervane().reset);
}
