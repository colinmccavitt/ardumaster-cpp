#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_loiter/mode_qloiter_run.hpp>
#include <fwcpp/q_loiter/mode_qland_run.hpp>

using fwcpp::q_loiter::QLoiterRunInputs;
using fwcpp::q_loiter::QLoiterRunPhase;
using fwcpp::q_loiter::QLoiterVerticalBranch;
using fwcpp::q_loiter::qloiter_run;
using fwcpp::q_loiter::qloiter_should_reinit_target;
using fwcpp::q_loiter::qland_run;

TEST_CASE("qloiter run phases", "[q_loiter][run]") {
    QLoiterRunInputs in{};
    in.assist_vtol_recovery = true;
    REQUIRE(qloiter_run(in).phase == QLoiterRunPhase::kAssistRecovery);
    in.assist_vtol_recovery = false;
    in.tailsitter_in_vtol_transition = true;
    REQUIRE(qloiter_run(in).phase == QLoiterRunPhase::kFwTransitionControllers);
    in.tailsitter_in_vtol_transition = false;
    in.throttle_wait = true;
    REQUIRE(qloiter_run(in).phase == QLoiterRunPhase::kThrottleWait);
    in.throttle_wait = false;
    auto main = qloiter_run(in);
    REQUIRE(main.phase == QLoiterRunPhase::kLoiterControl);
    REQUIRE(main.actions.loiter_nav_update);
    REQUIRE(main.vertical == QLoiterVerticalBranch::kPilotClimb);
}

TEST_CASE("qloiter reinit and qland delegate", "[q_loiter][run]") {
    REQUIRE(qloiter_should_reinit_target(600, 0));
    REQUIRE_FALSE(qloiter_should_reinit_target(400, 0));
    QLoiterRunInputs in{};
    in.active_control_is_qland = true;
    auto qland = qland_run(in);
    REQUIRE(qland.delegates_qloiter_run);
    REQUIRE(qland.qloiter.vertical == QLoiterVerticalBranch::kQlandDescent);
    REQUIRE(qland.qloiter.actions.qland_descent_rate);
}
