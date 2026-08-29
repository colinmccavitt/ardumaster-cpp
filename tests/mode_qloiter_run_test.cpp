#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_loiter/mode_qloiter_precland_run.hpp>
#include <fwcpp/q_loiter/mode_qloiter_run.hpp>
#include <fwcpp/q_loiter/mode_qland_run.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::PosControlState;
using fwcpp::q_loiter::QLoiterRunInputs;
using fwcpp::q_loiter::QLoiterRunPhase;
using fwcpp::q_loiter::QLoiterVerticalBranch;
using fwcpp::q_loiter::qloiter_precland_effects;
using fwcpp::q_loiter::qloiter_run;
using fwcpp::q_loiter::qloiter_run_view_loiter;
using fwcpp::q_loiter::qloiter_run_view_qland_descent;
using fwcpp::q_loiter::qloiter_run_view_qland_final;
using fwcpp::q_loiter::qloiter_run_view_throttle_wait;
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

TEST_CASE("qloiter throttle wait matches QHover wait body", "[q_loiter][run]") {
    const auto out = qloiter_run(qloiter_run_view_throttle_wait());
    REQUIRE(out.actions.ground_idle_spool);
    REQUIRE(out.actions.throttle_out_zero);
    REQUIRE(out.actions.relax_attitude);
    REQUIRE(out.actions.relax_pos_z);
    REQUIRE(out.actions.clear_pilot_accel);
    REQUIRE(out.actions.reinit_loiter_target);
    REQUIRE(out.actions.stabilize_fw_surfaces);
    REQUIRE_FALSE(out.actions.loiter_nav_update);
}

TEST_CASE("qloiter loiter tick wires NE and attitude", "[q_loiter][run]") {
    QLoiterRunInputs in = qloiter_run_view_loiter();
    in.ne_is_active = false;
    in.vtol_roll_pitch_limited = true;
    const auto out = qloiter_run(in);
    REQUIRE(out.actions.ne_init_controller);
    REQUIRE(out.actions.process_pilot_lean_angles);
    REQUIRE(out.actions.ne_set_externally_limited);
    REQUIRE(out.actions.pilot_yaw_rate_time_constant);
    REQUIRE(out.actions.attitude_euler_input);
    REQUIRE(out.actions.assign_tilt_to_fwd_thr);
}

TEST_CASE("qloiter precland overrides", "[q_loiter][run]") {
    fwcpp::q_loiter::QLoiterPreclandInputs in{};
    in.now_ms = 100;
    in.last_target_loc_set_ms = 50;
    in.rel_origin_valid = true;
    in.rel_origin_n_m = 1.0F;
    in.rel_origin_e_m = -2.0F;
    auto fx = qloiter_precland_effects(in);
    REQUIRE(fx.apply_pos_override);
    REQUIRE(fx.clear_target_loc_set_ms);
    in.last_target_loc_set_ms = 0;
    in.last_velocity_match_ms = 80;
    in.velocity_match_n_ms = 0.5F;
    fx = qloiter_precland_effects(in);
    REQUIRE(fx.apply_vel_override);
    REQUIRE(fx.clear_velocity_match_ms);
}

TEST_CASE("qloiter reinit and qland delegate", "[q_loiter][run]") {
    REQUIRE(qloiter_should_reinit_target(600, 0));
    REQUIRE_FALSE(qloiter_should_reinit_target(400, 0));
    QLoiterRunInputs in = qloiter_run_view_qland_descent();
    PosControlState pc{};
    pc.state = PositionControlState::kLandDescend;
    const auto qland = qland_run(in, pc);
    REQUIRE(qland.delegates_qloiter_run);
    REQUIRE(qland.qloiter.vertical == QLoiterVerticalBranch::kQlandDescent);
    REQUIRE(qland.qloiter.actions.qland_descent_rate);
    REQUIRE_FALSE(qland.qloiter.actions.qland_touchdown_expected);
}

TEST_CASE("qloiter qland land final transition", "[q_loiter][run]") {
    PosControlState pc{};
    pc.state = PositionControlState::kLandDescend;
    const auto qland = qland_run(qloiter_run_view_qland_final(), pc);
    REQUIRE(qland.qloiter.actions.qland_land_final_transition);
    REQUIRE(qland.qloiter.effects.set_poscontrol_land_final);
    REQUIRE(qland.qloiter.actions.qland_touchdown_expected);
    REQUIRE(pc.state == PositionControlState::kLandFinal);
}