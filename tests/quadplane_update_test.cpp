#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_update.hpp>
#include <fwcpp/quadplane_transition/transition_state.hpp>
#include <fwcpp/quadplane_transition/transition_timing.hpp>

using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::QuadPlaneSetupInputs;
using fwcpp::quadplane::QuadPlaneUpdateView;
using fwcpp::quadplane::mav_vtol_phase;
using fwcpp::quadplane::run_quadplane_update;
using fwcpp::quadplane_transition::SltTransition;
using fwcpp::quadplane_transition::TransFailOutcome;
using fwcpp::quadplane_transition::TransitionPhase;
using fwcpp::quadplane_transition::TransitionState;


TEST_CASE("quadplane update no-op when not available", "[quadplane][update]") {
    QuadPlane qp(0);
    QuadPlaneUpdateView view{.now_ms = 1000};
    const auto tick = qp.update(view);
    REQUIRE_FALSE(tick.ran_transition_update);
    REQUIRE(tick.phase == TransitionPhase::kVtol);
    REQUIRE(tick.trans_fail == TransFailOutcome::kContinue);
}

TEST_CASE("quadplane update wires slt forward transition to timer", "[quadplane][update]") {
    QuadPlane qp(1);
    REQUIRE(qp.setup(QuadPlaneSetupInputs{}));
    QuadPlaneUpdateView view{
        .now_ms = 5000,
        .armed_and_safety_off = true,
        .in_vtol_mode = false,
        .have_airspeed = true,
        .airspeed_ms = 15.f,
        .airspeed_min_ms = 10.f,
        .tilt_forward_complete = true,
    };
    const auto tick = qp.update(view);
    REQUIRE(tick.ran_transition_update);
    REQUIRE(qp.slt_transition().state() == TransitionState::kTimer);
    REQUIRE(tick.phase == TransitionPhase::kTransition);
}

TEST_CASE("mav vtol phase air when forward transition complete", "[quadplane][update]") {
    SltTransition slt = SltTransition::with_defaults();
    slt.update_forward_timing(1000, true, 15.f, 10.f, false, true);
    slt.update_forward_timing(1000U + slt.timer_duration_ms() + 1, true, 15.f, 10.f, false, true);
    REQUIRE(slt.complete());
    REQUIRE(mav_vtol_phase(false, slt) == TransitionPhase::kAir);
    REQUIRE(mav_vtol_phase(true, slt) == TransitionPhase::kVtol);
}

TEST_CASE("run_quadplane_update syncs q options for trans fail to fw", "[quadplane][update]") {
    QuadPlane qp(1);
    REQUIRE(qp.setup(QuadPlaneSetupInputs{}));
    qp.set_options(fwcpp::quadplane_transition::kQOptionsTransFailToFw);
    qp.slt_transition_mut().set_transition_fail_timeout_s(1);
    QuadPlaneUpdateView view{
        .now_ms = 100,
        .armed_and_safety_off = true,
        .in_vtol_mode = false,
        .have_airspeed = false,
        .airspeed_min_ms = 10.f,
        .tiltrotor_with_ground_speed = true,
    };
    qp.update(view);
    const auto tick2 = qp.update(QuadPlaneUpdateView{
        .now_ms = 2500,
        .armed_and_safety_off = true,
        .in_vtol_mode = false,
        .have_airspeed = false,
        .airspeed_min_ms = 10.f,
        .tiltrotor_with_ground_speed = true,
    });
    REQUIRE(tick2.trans_fail == TransFailOutcome::kCompleteToFw);
}