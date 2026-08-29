#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_landing.hpp>

using Catch::Approx;
using fwcpp::quadplane::AbortLandingInputs;
using fwcpp::quadplane::LandPositioningInputs;
using fwcpp::quadplane::LandingDescentRateInputs;
using fwcpp::quadplane::PosControlLandStub;
using fwcpp::quadplane::PosControlSetStateSink;
using fwcpp::quadplane::PosControlState;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QOption;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::abort_landing;
using fwcpp::quadplane::kDefaultSpeedDownMs;
using fwcpp::quadplane::kDefaultSpeedUpMs;
using fwcpp::quadplane::kLandFinalAltDefaultM;
using fwcpp::quadplane::kLandFinalInterpSpanM;
using fwcpp::quadplane::kLandFinalSpeedMsDefault;
using fwcpp::quadplane::kLandStickScale;
using fwcpp::quadplane::kOverrideDescentWindowMs;
using fwcpp::quadplane::kWpAccelMssDefault;
using fwcpp::quadplane::landing_descent_rate_ms;
using fwcpp::quadplane::update_land_positioning;

static LandingDescentRateInputs descent_in(std::uint32_t now_ms = 0) {
    LandingDescentRateInputs in{};
    in.now_ms = now_ms;
    return in;
}

TEST_CASE("landing_descent_rate_ms override window 1000ms", "[quadplane][landing][descent]") {
    PosControlState pc{};
    PosControlLandStub land{};
    pc.override_descent_rate_ms = 2.25f;
    pc.last_override_descent_ms = 5000;

    auto inside = landing_descent_rate_ms(pc, land, 20.f, descent_in(5000 + kOverrideDescentWindowMs - 1));
    REQUIRE(inside.used_override);
    REQUIRE(inside.rate_ms == Approx(2.25f));

    auto at_edge = landing_descent_rate_ms(pc, land, 20.f, descent_in(5000 + kOverrideDescentWindowMs));
    REQUIRE_FALSE(at_edge.used_override);
    REQUIRE(at_edge.rate_ms == Approx(kDefaultSpeedDownMs));

    pc.last_override_descent_ms = 0;
    pc.override_descent_rate_ms = 9.f;
    auto none = landing_descent_rate_ms(pc, land, 20.f, descent_in(100));
    REQUIRE_FALSE(none.used_override);
    REQUIRE(none.rate_ms == Approx(kDefaultSpeedDownMs));
}

TEST_CASE("landing_descent_rate_ms LAND_FINAL clamps height", "[quadplane][landing][descent]") {
    PosControlState pc{};
    PosControlLandStub land{};
    pc.state = PositionControlState::kLandFinal;

    const auto clamped = landing_descent_rate_ms(pc, land, 40.f, descent_in());
    REQUIRE(clamped.clamped_final_height);
    REQUIRE(clamped.rate_ms == Approx(kLandFinalSpeedMsDefault));

    pc.state = PositionControlState::kLandDescend;
    const auto open = landing_descent_rate_ms(pc, land, 40.f, descent_in());
    REQUIRE_FALSE(open.clamped_final_height);
    REQUIRE(open.rate_ms == Approx(kDefaultSpeedDownMs));
}

TEST_CASE("landing_descent_rate_ms interpolates final to speed_down", "[quadplane][landing][descent]") {
    PosControlState pc{};
    PosControlLandStub land{};

    const float low = kLandFinalAltDefaultM;
    const float high = kLandFinalAltDefaultM + kLandFinalInterpSpanM;
    const float mid = 0.5f * (low + high);

    REQUIRE(landing_descent_rate_ms(pc, land, low, descent_in()).rate_ms ==
            Approx(kLandFinalSpeedMsDefault));
    REQUIRE(landing_descent_rate_ms(pc, land, high, descent_in()).rate_ms ==
            Approx(kDefaultSpeedDownMs));
    REQUIRE(landing_descent_rate_ms(pc, land, mid, descent_in()).rate_ms ==
            Approx(fwcpp::math::linear_interpolate(kLandFinalSpeedMsDefault, kDefaultSpeedDownMs, mid, low,
                                                   high)));
}

TEST_CASE("landing_descent_rate_ms throttle climb hold descend", "[quadplane][landing][descent]") {
    PosControlState pc{};
    PosControlLandStub land{};
    LandingDescentRateInputs in = descent_in();
    in.options = static_cast<std::int32_t>(QOption::kThrLandingControl);
    in.land_throttle.range = 100.f;

    in.land_throttle.control_in = 80.f;  // 0.8 > 0.7 latch, > 0.6 climb
    const auto climb = landing_descent_rate_ms(pc, land, 20.f, in);
    REQUIRE(land.thr_ctrl_land);
    REQUIRE(climb.thr_ctrl_land);
    const float climb_expected = -(0.8f - 0.6f) * 2.5f * kDefaultSpeedUpMs;
    REQUIRE(climb.rate_ms == Approx(climb_expected));
    REQUIRE(climb.rate_ms < 0.f);

    in.land_throttle.control_in = 50.f;  // 0.5 hold
    const auto hold = landing_descent_rate_ms(pc, land, 20.f, in);
    REQUIRE(land.thr_ctrl_land);
    REQUIRE(hold.rate_ms == Approx(0.f));

    in.land_throttle.control_in = 20.f;  // 0.2 descend scale
    const float base = fwcpp::math::linear_interpolate(
        kLandFinalSpeedMsDefault, kDefaultSpeedDownMs, 20.f, kLandFinalAltDefaultM,
        kLandFinalAltDefaultM + kLandFinalInterpSpanM);
    const auto descend = landing_descent_rate_ms(pc, land, 20.f, in);
    REQUIRE(descend.rate_ms == Approx(base * (0.4f - 0.2f) * 2.5f));
    REQUIRE(descend.rate_ms > 0.f);

    PosControlLandStub fresh{};
    in.land_throttle.control_in = 60.f;  // 0.6 does not latch
    REQUIRE_FALSE(landing_descent_rate_ms(pc, fresh, 20.f, in).thr_ctrl_land);
    REQUIRE_FALSE(fresh.thr_ctrl_land);
}

TEST_CASE("landing_descent_rate_ms correction stops descent not climb",
          "[quadplane][landing][descent]") {
    PosControlState pc{};
    PosControlLandStub land{};
    pc.pilot_correction_active = true;

    const auto stopped = landing_descent_rate_ms(pc, land, 20.f, descent_in());
    REQUIRE(stopped.stopped_for_correction);
    REQUIRE(stopped.rate_ms == Approx(0.f));

    LandingDescentRateInputs climb = descent_in();
    climb.options = static_cast<std::int32_t>(QOption::kThrLandingControl);
    climb.land_throttle.control_in = 90.f;
    climb.land_throttle.range = 100.f;
    const auto still_climb = landing_descent_rate_ms(pc, land, 20.f, climb);
    REQUIRE(still_climb.stopped_for_correction);
    REQUIRE(still_climb.rate_ms < 0.f);
    REQUIRE(still_climb.rate_ms == Approx(-(0.9f - 0.6f) * 2.5f * kDefaultSpeedUpMs));
}

TEST_CASE("update_land_positioning disabled zeros vel", "[quadplane][landing][reposition]") {
    PosControlState pc{};
    pc.target_vel_north_ms = 3.f;
    pc.target_vel_east_ms = -2.f;
    pc.target_vel_down_ms = 1.f;
    pc.correction_north_m = 4.f;
    pc.correction_east_m = 5.f;
    pc.pilot_correction_active = true;
    pc.pilot_correction_done = true;

    LandPositioningInputs in{};
    in.roll_control_in = 4500.f;
    in.pitch_control_in = -4500.f;
    in.loop_period_s = 0.02f;
    const auto tick = update_land_positioning(pc, in);
    REQUIRE_FALSE(tick.enabled);
    REQUIRE_FALSE(pc.pilot_correction_active);
    REQUIRE(pc.pilot_correction_done);
    REQUIRE(pc.target_vel_north_ms == Approx(0.f));
    REQUIRE(pc.target_vel_east_ms == Approx(0.f));
    REQUIRE(pc.target_vel_down_ms == Approx(0.f));
    REQUIRE(pc.correction_north_m == Approx(4.f));
    REQUIRE(pc.correction_east_m == Approx(5.f));
}

TEST_CASE("update_land_positioning rotate and integrate", "[quadplane][landing][reposition]") {
    PosControlState pc{};
    LandPositioningInputs in{};
    in.options = static_cast<std::int32_t>(QOption::kRepositionLanding);
    in.roll_control_in = 4500.f;
    in.pitch_control_in = 0.f;
    in.wp_accel_mss = kWpAccelMssDefault;
    in.loop_period_s = 0.1f;
    in.yaw_rad = 0.f;

    const float speed_max = kWpAccelMssDefault * 0.5f;
    const auto east = update_land_positioning(pc, in);
    REQUIRE(east.enabled);
    REQUIRE(east.pilot_correction_active);
    REQUIRE(pc.pilot_correction_done);
    REQUIRE(pc.target_vel_north_ms == Approx(0.f));
    REQUIRE(pc.target_vel_east_ms == Approx(speed_max));
    REQUIRE(pc.target_vel_down_ms == Approx(0.f));
    REQUIRE(pc.correction_north_m == Approx(0.f));
    REQUIRE(pc.correction_east_m == Approx(speed_max * 0.1f));

    pc = {};
    in.roll_control_in = 0.f;
    in.pitch_control_in = 4500.f;
    in.yaw_rad = static_cast<float>(M_PI / 2);
    fwcpp::math::Vector3f expected(-1.f * kLandStickScale * 4500.f, 0.f, 0.f);
    expected *= speed_max;
    expected.rotate_xy(in.yaw_rad);
    const auto rotated = update_land_positioning(pc, in);
    REQUIRE(rotated.pilot_correction_active);
    REQUIRE(pc.target_vel_north_ms == Approx(expected.x));
    REQUIRE(pc.target_vel_east_ms == Approx(expected.y));
    REQUIRE(pc.correction_north_m == Approx(expected.x * 0.1f));
    REQUIRE(pc.correction_east_m == Approx(expected.y * 0.1f));

    pc = {};
    in.roll_control_in = 0.f;
    in.pitch_control_in = 0.f;
    in.yaw_rad = 0.f;
    const auto idle = update_land_positioning(pc, in);
    REQUIRE(idle.enabled);
    REQUIRE_FALSE(idle.pilot_correction_active);
    REQUIRE_FALSE(pc.pilot_correction_done);
    REQUIRE(pc.target_vel_north_ms == Approx(0.f));
    REQUIRE(pc.target_vel_east_ms == Approx(0.f));
}

TEST_CASE("abort_landing auto payload and descent gates", "[quadplane][landing][abort]") {
    PosControlState pc{};
    PosControlLandStub land{};
    PosControlSetStateSink sink{};

    AbortLandingInputs in{};
    in.mode_auto = true;
    in.land_descent.control_is_qrtl = true;

    pc.state = PositionControlState::kLandAbort;
    REQUIRE_FALSE(abort_landing(pc, land, sink, in).aborted);

    pc.state = PositionControlState::kLandDescend;
    in.mode_auto = false;
    REQUIRE_FALSE(abort_landing(pc, land, sink, in).aborted);
    REQUIRE(pc.state == PositionControlState::kLandDescend);

    in.mode_auto = true;
    in.land_descent.control_is_qrtl = false;
    REQUIRE_FALSE(abort_landing(pc, land, sink, in).aborted);

    in.in_auto_payload_place = true;
    pc.state = PositionControlState::kLandComplete;
    auto payload = abort_landing(pc, land, sink, in);
    REQUIRE(payload.payload_place_landed);
    REQUIRE(payload.aborted);
    REQUIRE(pc.state == PositionControlState::kLandAbort);

    pc.state = PositionControlState::kLandFinal;
    land.thr_ctrl_land = true;
    in.in_auto_payload_place = false;
    in.land_descent.control_is_qrtl = true;
    auto descent = abort_landing(pc, land, sink, in);
    REQUIRE_FALSE(descent.payload_place_landed);
    REQUIRE(descent.aborted);
    REQUIRE(pc.state == PositionControlState::kLandAbort);
    REQUIRE_FALSE(land.thr_ctrl_land);
}

TEST_CASE("QuadPlane landing ticks wire options and stubs", "[quadplane][landing][wired]") {
    QuadPlane qp{1};
    qp.set_options(static_cast<std::int32_t>(QOption::kThrLandingControl) |
                   static_cast<std::int32_t>(QOption::kRepositionLanding));
    qp.poscontrol_mut().state = PositionControlState::kLandDescend;

    LandingDescentRateInputs d{};
    d.land_throttle.control_in = 85.f;
    const auto climb = qp.landing_descent_rate_ms(20.f, d);
    REQUIRE(climb.rate_ms < 0.f);
    REQUIRE(qp.poscontrol_land().thr_ctrl_land);

    LandPositioningInputs p{};
    p.roll_control_in = 2250.f;
    p.loop_period_s = 0.05f;
    const auto repo = qp.update_land_positioning(p);
    REQUIRE(repo.enabled);
    REQUIRE(qp.poscontrol().pilot_correction_active);

    AbortLandingInputs a{};
    a.mode_auto = true;
    a.land_descent.control_is_qrtl = true;
    REQUIRE(qp.abort_landing(a).aborted);
    REQUIRE(qp.poscontrol().state == PositionControlState::kLandAbort);
}
