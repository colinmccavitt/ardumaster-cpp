#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_guided.hpp>

using Catch::Approx;
using fwcpp::quadplane::DesiredSpoolState;
using fwcpp::quadplane::GuidedStartInputs;
using fwcpp::quadplane::GuidedUpdateInputs;
using fwcpp::quadplane::PosControlLandStub;
using fwcpp::quadplane::PosControlSetStateSink;
using fwcpp::quadplane::PosControlState;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QOption;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::UserTakeoffInputs;
using fwcpp::quadplane::UserTakeoffText;
using fwcpp::quadplane::do_user_takeoff;
using fwcpp::quadplane::guided_start;
using fwcpp::quadplane::guided_update;
using fwcpp::quadplane::user_takeoff_text;

static GuidedStartInputs abs_alts(std::int32_t from_cm, std::int32_t to_cm) {
    GuidedStartInputs in{};
    in.from_alt_abs_cm = from_cm;
    in.to_alt_abs_cm = to_cm;
    in.current_loc_alt_cm = 0;
    in.next_wp_alt_cm = 10000;
    return in;
}

static GuidedStartInputs fallback_alts(std::int32_t current_cm, std::int32_t next_cm) {
    GuidedStartInputs in{};
    in.current_loc_alt_cm = current_cm;
    in.next_wp_alt_cm = next_cm;
    return in;
}

TEST_CASE("guided_start slow_descent uses absolute alts when both available",
          "[quadplane][guided][start]") {
    PosControlState pc{};
    PosControlLandStub land{};
    pc.slow_descent = false;

    auto descent = guided_start(pc, land, abs_alts(2000, 1000));
    REQUIRE(descent.setup_target_position);
    REQUIRE(descent.poscontrol_init_approach);
    REQUIRE_FALSE(descent.guided_takeoff);
    REQUIRE(descent.slow_descent);
    REQUIRE(pc.slow_descent);

    auto climb = guided_start(pc, land, abs_alts(1000, 2000));
    REQUIRE_FALSE(climb.slow_descent);
    REQUIRE_FALSE(pc.slow_descent);

    auto equal = guided_start(pc, land, abs_alts(1500, 1500));
    REQUIRE_FALSE(equal.slow_descent);

    GuidedStartInputs only_from = abs_alts(3000, 1000);
    only_from.to_alt_abs_cm.reset();
    only_from.current_loc_alt_cm = 100;
    only_from.next_wp_alt_cm = 200;
    REQUIRE_FALSE(guided_start(pc, land, only_from).slow_descent);

    QuadPlane qp{1};
    qp.set_guided_takeoff(true);
    const auto wired = qp.guided_start(abs_alts(4000, 1000));
    REQUIRE(wired.slow_descent);
    REQUIRE(qp.poscontrol().slow_descent);
    REQUIRE_FALSE(qp.guided_takeoff());
}

TEST_CASE("guided_start slow_descent falls back to raw loc.alt", "[quadplane][guided][start]") {
    PosControlState pc{};
    PosControlLandStub land{};

    REQUIRE(guided_start(pc, land, fallback_alts(800, 400)).slow_descent);
    REQUIRE(pc.slow_descent);
    REQUIRE_FALSE(guided_start(pc, land, fallback_alts(400, 800)).slow_descent);
    REQUIRE_FALSE(pc.slow_descent);
    REQUIRE_FALSE(guided_start(pc, land, fallback_alts(500, 500)).slow_descent);

    GuidedStartInputs missing_from{};
    missing_from.to_alt_abs_cm = 100;
    missing_from.current_loc_alt_cm = 900;
    missing_from.next_wp_alt_cm = 100;
    REQUIRE(guided_start(pc, land, missing_from).slow_descent);
}

TEST_CASE("guided_start approach_prep zero is overwritten by abs compare",
          "[quadplane][guided][start]") {
    PosControlState pc{};
    PosControlLandStub land{};
    pc.slow_descent = false;
    const auto tick = guided_start(pc, land, abs_alts(2500, 500));
    REQUIRE(tick.poscontrol_init_approach);
    REQUIRE(tick.slow_descent);
    REQUIRE(pc.slow_descent);
    REQUIRE(tick.setup.spool_throttle_unlimited);
}

TEST_CASE("guided_update takeoff path vs POSITION2 handoff", "[quadplane][guided][update]") {
    PosControlState pc{};
    PosControlLandStub land{};
    PosControlSetStateSink sink{};

    GuidedUpdateInputs climb{};
    climb.mode_guided = true;
    climb.guided_takeoff = true;
    climb.current_alt_cm = 100;
    climb.next_wp_alt_cm = 2000;
    auto takeoff = guided_update(pc, land, sink, climb);
    REQUIRE(takeoff.clear_throttle_wait);
    REQUIRE_FALSE(takeoff.throttle_wait);
    REQUIRE(takeoff.set_desired_spool);
    REQUIRE(takeoff.desired_spool == DesiredSpoolState::kThrottleUnlimited);
    REQUIRE(takeoff.run_takeoff);
    REQUIRE(takeoff.guided_takeoff);
    REQUIRE_FALSE(takeoff.set_state_position2);
    REQUIRE_FALSE(takeoff.run_vtol_position_controller);
    REQUIRE(pc.state == PositionControlState::kNone);

    GuidedUpdateInputs reached = climb;
    reached.current_alt_cm = 2000;
    auto handoff = guided_update(pc, land, sink, reached);
    REQUIRE_FALSE(handoff.run_takeoff);
    REQUIRE(handoff.set_state_position2);
    REQUIRE_FALSE(handoff.guided_takeoff);
    REQUIRE(handoff.run_vtol_position_controller);
    REQUIRE(pc.state == PositionControlState::kPosition2);

    pc.state = PositionControlState::kApproach;
    GuidedUpdateInputs not_guided = climb;
    not_guided.mode_guided = false;
    auto leave = guided_update(pc, land, sink, not_guided);
    REQUIRE(leave.set_state_position2);
    REQUIRE(leave.run_vtol_position_controller);
    REQUIRE_FALSE(leave.guided_takeoff);
    REQUIRE(pc.state == PositionControlState::kPosition2);

    GuidedUpdateInputs already{};
    already.mode_guided = true;
    already.guided_takeoff = false;
    already.current_alt_cm = 50;
    already.next_wp_alt_cm = 2000;
    const auto cruise = guided_update(pc, land, sink, already);
    REQUIRE_FALSE(cruise.run_takeoff);
    REQUIRE_FALSE(cruise.set_state_position2);
    REQUIRE(cruise.run_vtol_position_controller);
    REQUIRE_FALSE(cruise.guided_takeoff);
}

TEST_CASE("QuadPlane guided_update persists takeoff then POSITION2", "[quadplane][guided][update]") {
    QuadPlane qp{1};
    qp.set_guided_takeoff(true);
    qp.set_throttle_wait(true);

    GuidedUpdateInputs climb{};
    climb.mode_guided = true;
    climb.current_alt_cm = 10;
    climb.next_wp_alt_cm = 1500;
    const auto takeoff = qp.guided_update(climb);
    REQUIRE(takeoff.run_takeoff);
    REQUIRE(qp.guided_takeoff());
    REQUIRE_FALSE(qp.throttle_wait());

    climb.current_alt_cm = 1600;
    const auto handoff = qp.guided_update(climb);
    REQUIRE(handoff.set_state_position2);
    REQUIRE(handoff.run_vtol_position_controller);
    REQUIRE_FALSE(qp.guided_takeoff());
    REQUIRE(qp.poscontrol().state == PositionControlState::kPosition2);
}

static UserTakeoffInputs armed_guided() {
    UserTakeoffInputs in{};
    in.mode_guided = true;
    in.armed_and_safety_off = true;
    in.is_flying = false;
    return in;
}

TEST_CASE("do_user_takeoff rejects mode arm and flying", "[quadplane][guided][takeoff]") {
    PosControlState pc{};
    PosControlLandStub land{};

    UserTakeoffInputs in{};
    auto not_guided = do_user_takeoff(pc, land, 10.f, in);
    REQUIRE_FALSE(not_guided.ok);
    REQUIRE(not_guided.send_text == UserTakeoffText::kOnlyInGuided);
    REQUIRE(user_takeoff_text(not_guided.send_text) ==
            std::string("User Takeoff only in GUIDED mode"));
    REQUIRE_FALSE(not_guided.offset_up_m);
    REQUIRE_FALSE(not_guided.call_guided_start);

    in.mode_guided = true;
    auto disarmed = do_user_takeoff(pc, land, 10.f, in);
    REQUIRE_FALSE(disarmed.ok);
    REQUIRE(disarmed.send_text == UserTakeoffText::kMustBeArmed);
    REQUIRE(user_takeoff_text(disarmed.send_text) == std::string("Must be armed for takeoff"));

    in.armed_and_safety_off = true;
    in.is_flying = true;
    auto flying = do_user_takeoff(pc, land, 10.f, in);
    REQUIRE_FALSE(flying.ok);
    REQUIRE(flying.send_text == UserTakeoffText::kAlreadyFlying);
    REQUIRE(user_takeoff_text(flying.send_text) == std::string("Already flying - no takeoff"));
    REQUIRE_FALSE(flying.guided_takeoff);
    REQUIRE_FALSE(flying.set_takeoff_expected);

    QuadPlane qp{1};
    qp.set_guided_takeoff(false);
    UserTakeoffInputs wired{};
    wired.mode_guided = false;
    REQUIRE_FALSE(qp.do_user_takeoff(8.f, wired).ok);
    REQUIRE_FALSE(qp.guided_takeoff());
}

TEST_CASE("do_user_takeoff success sets flags offset and takeoff_expected",
          "[quadplane][guided][takeoff]") {
    PosControlState pc{};
    PosControlLandStub land{};
    auto in = armed_guided();
    in.start = abs_alts(100, 2000);

    const auto ok = do_user_takeoff(pc, land, 12.5f, in);
    REQUIRE(ok.ok);
    REQUIRE(ok.send_text == UserTakeoffText::kNone);
    REQUIRE(ok.vtol_loiter);
    REQUIRE(ok.prev_wp_from_current);
    REQUIRE(ok.next_wp_from_current);
    REQUIRE(ok.offset_up_m);
    REQUIRE(ok.takeoff_altitude_m == Approx(12.5f));
    REQUIRE(ok.set_desired_spool);
    REQUIRE(ok.desired_spool == DesiredSpoolState::kThrottleUnlimited);
    REQUIRE(ok.call_guided_start);
    REQUIRE(ok.start.setup_target_position);
    REQUIRE(ok.start.poscontrol_init_approach);
    REQUIRE(ok.guided_takeoff);
    REQUIRE_FALSE(ok.guided_wait_takeoff);
    REQUIRE(ok.set_takeoff_expected);

    in.options = static_cast<std::int32_t>(QOption::kDisableGroundEffectComp);
    const auto no_gec = do_user_takeoff(pc, land, 5.f, in);
    REQUIRE(no_gec.ok);
    REQUIRE_FALSE(no_gec.set_takeoff_expected);
    REQUIRE(no_gec.offset_up_m);
    REQUIRE(no_gec.takeoff_altitude_m == Approx(5.f));

    QuadPlane qp{1};
    qp.set_guided_wait_takeoff(true);
    qp.set_guided_takeoff(false);
    qp.set_options(0);
    UserTakeoffInputs wired = armed_guided();
    const auto wired_ok = qp.do_user_takeoff(7.f, wired);
    REQUIRE(wired_ok.ok);
    REQUIRE(wired_ok.set_takeoff_expected);
    REQUIRE(qp.guided_takeoff());
    REQUIRE_FALSE(qp.guided_wait_takeoff());

    qp.set_options(static_cast<std::int32_t>(QOption::kDisableGroundEffectComp));
    qp.set_guided_wait_takeoff(true);
    const auto wired_gec = qp.do_user_takeoff(3.f, armed_guided());
    REQUIRE(wired_gec.ok);
    REQUIRE_FALSE(wired_gec.set_takeoff_expected);
    REQUIRE(qp.guided_takeoff());
    REQUIRE_FALSE(qp.guided_wait_takeoff());
}
