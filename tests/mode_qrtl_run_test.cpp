#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qrtl/mode_qrtl_land_handoff.hpp>
#include <fwcpp/qrtl/mode_qrtl_run.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

using Catch::Approx;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::PosControlState;
using fwcpp::qrtl::QrtlDestination;
using fwcpp::qrtl::QrtlRunAction;
using fwcpp::qrtl::QrtlRunEffects;
using fwcpp::qrtl::QrtlSubMode;
using fwcpp::qrtl::kQWpSpdUpDefaultMs;
using fwcpp::qrtl::qrtl_climb_finished;
using fwcpp::qrtl::qrtl_copy_home_alt;
using fwcpp::qrtl::qrtl_land_handoff;
using fwcpp::qrtl::qrtl_run;
using fwcpp::qrtl::qrtl_run_view_climb_done_close;
using fwcpp::qrtl::qrtl_run_view_climb_done_far;
using fwcpp::qrtl::qrtl_run_view_climbing;
using fwcpp::qrtl::qrtl_run_view_returning;
using fwcpp::qrtl::qrtl_run_view_tailsitter_fw_transition;
using fwcpp::qrtl::qrtl_should_verify_land;
using fwcpp::qrtl::qrtl_stick_mixing_fbw;

TEST_CASE("qrtl climb finished predicate", "[qrtl][run]") {
    fwcpp::qrtl::QrtlHeightAbove below{};
    below.valid = true;
    below.meters = -5.0F;
    REQUIRE_FALSE(qrtl_climb_finished(below));
    below.meters = 0.0F;
    REQUIRE_FALSE(qrtl_climb_finished(below));
    below.meters = 0.1F;
    REQUIRE(qrtl_climb_finished(below));
    REQUIRE(qrtl_climb_finished({}));
}

TEST_CASE("qrtl run climb holds xy and climbs", "[qrtl][run]") {
    PosControlState pc{};
    QrtlRunEffects effects{};
    const auto out = qrtl_run(qrtl_run_view_climbing(), pc, effects);
    REQUIRE(out.action == QrtlRunAction::kClimb);
    REQUIRE(out.submode == QrtlSubMode::kClimb);
    REQUIRE(out.climb_rate_ms == Approx(kQWpSpdUpDefaultMs));
    REQUIRE(out.xy_hold);
    REQUIRE(out.tilt_assigned);
    REQUIRE(out.weathervane);
    REQUIRE(out.z_controller);
    REQUIRE(out.fw_stabilize);
    REQUIRE_FALSE(out.do_rtl);
    REQUIRE_FALSE(effects.do_rtl);
    REQUIRE(pc.state == PositionControlState::kNone);
}

TEST_CASE("qrtl run climb then return far", "[qrtl][run]") {
    PosControlState pc{};
    QrtlRunEffects effects{};
    const auto out = qrtl_run(qrtl_run_view_climb_done_far(), pc, effects);
    REQUIRE(out.action == QrtlRunAction::kClimbThenReturn);
    REQUIRE(out.submode == QrtlSubMode::kRtl);
    REQUIRE(out.dest == QrtlDestination::kHome);
    REQUIRE(out.dist_m == Approx(200.0F));
    REQUIRE(out.do_rtl);
    REQUIRE(effects.do_rtl);
    REQUIRE(effects.poscontrol_init_approach);
    REQUIRE_FALSE(out.position1);
    REQUIRE(out.rtl_alt_abs_cm == 1500);
    REQUIRE_FALSE(out.slow_descent);
}

TEST_CASE("qrtl run climb then return close position1", "[qrtl][run]") {
    PosControlState pc{};
    QrtlRunEffects effects{};
    const auto out = qrtl_run(qrtl_run_view_climb_done_close(), pc, effects);
    REQUIRE(out.action == QrtlRunAction::kClimbThenReturn);
    REQUIRE(out.position1);
    REQUIRE(effects.set_position1);
    REQUIRE(pc.state == PositionControlState::kPosition1);
    REQUIRE(out.rtl_alt_abs_cm == 1200);
}

TEST_CASE("qrtl run climb done failed height lookup", "[qrtl][run]") {
    PosControlState pc{};
    QrtlRunEffects effects{};
    auto view = qrtl_run_view_climbing();
    view.stopping_height_above_next_wp = {};
    const auto out = qrtl_run(view, pc, effects);
    REQUIRE(out.action == QrtlRunAction::kClimbThenReturn);
    REQUIRE(out.do_rtl);
}

TEST_CASE("qrtl run returning branch", "[qrtl][run]") {
    PosControlState pc{};
    QrtlRunEffects effects{};
    const auto out = qrtl_run(qrtl_run_view_returning(), pc, effects);
    REQUIRE(out.action == QrtlRunAction::kReturn);
    REQUIRE(out.vtol_position_controller);
    REQUIRE(out.fw_stabilize);
    REQUIRE_FALSE(out.xy_hold);
    REQUIRE_FALSE(out.copy_home_alt);
    REQUIRE_FALSE(out.verify_vtol_land);
}

TEST_CASE("qrtl run tailsitter fw pullup", "[qrtl][run]") {
    PosControlState pc{};
    QrtlRunEffects effects{};
    const auto out = qrtl_run(qrtl_run_view_tailsitter_fw_transition(), pc, effects);
    REQUIRE(out.action == QrtlRunAction::kFwControllers);
    REQUIRE(out.delegate_mode_run);
    REQUIRE_FALSE(out.fw_stabilize);
}

TEST_CASE("qrtl land handoff flags", "[qrtl][run][land]") {
    REQUIRE_FALSE(qrtl_copy_home_alt(PositionControlState::kPosition2));
    REQUIRE(qrtl_copy_home_alt(PositionControlState::kLandDescend));
    REQUIRE(qrtl_should_verify_land(PositionControlState::kPosition2));
    REQUIRE_FALSE(qrtl_should_verify_land(PositionControlState::kPosition1));
    REQUIRE(qrtl_stick_mixing_fbw(PositionControlState::kApproach));
    const auto handoff = qrtl_land_handoff(PositionControlState::kLandFinal);
    REQUIRE(handoff.copy_home_alt);
    REQUIRE(handoff.verify_vtol_land);
    REQUIRE_FALSE(handoff.stick_mixing_fbw);
}