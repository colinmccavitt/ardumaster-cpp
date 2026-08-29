#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

using fwcpp::quadplane::ApproachInitView;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::PosControlLandStub;
using fwcpp::quadplane::PosControlSetStateInputs;
using fwcpp::quadplane::PosControlSetStateSink;
using fwcpp::quadplane::PosControlState;
using fwcpp::quadplane::QOption;
using fwcpp::quadplane::QuadPlane;

static QuadPlane available_qp() {
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    return qp;
}

TEST_CASE("mode_enter lean angle when available", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    qp.set_lean_angle_max_cd(4500);
    qp.mode_enter();
    REQUIRE(qp.lean_angle_max_cd() == 0);
}

TEST_CASE("mode_enter lean angle when unavailable", "[quadplane][poscontrol]") {
    QuadPlane qp{1};
    qp.set_lean_angle_max_cd(4500);
    qp.mode_enter();
    REQUIRE(qp.lean_angle_max_cd() == 4500);
}

TEST_CASE("mode_enter resets poscontrol state", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    qp.poscontrol_mut().state = PositionControlState::kApproach;
    qp.poscontrol_mut().correction_north_m = 12.f;
    qp.poscontrol_mut().mode_enter_cleared = false;
    qp.mode_enter();
    REQUIRE(qp.poscontrol().state == PositionControlState::kNone);
    REQUIRE(qp.poscontrol().correction_north_m == 0.f);
    REQUIRE(qp.poscontrol().mode_enter_cleared);
}

TEST_CASE("mode_enter guided wait takeoff snapshot", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    qp.set_guided_wait_takeoff(true);
    qp.mode_enter();
    REQUIRE(qp.guided_wait_takeoff_on_mode_enter());
    qp.mode_enter();
    REQUIRE_FALSE(qp.guided_wait_takeoff_on_mode_enter());
}

TEST_CASE("set_state position1 records yaw reset and speed limit", "[quadplane][poscontrol]") {
    PosControlState pc{};
    PosControlLandStub land{};
    PosControlSetStateSink sink{};
    PosControlSetStateInputs in{.now_ms = 1000, .groundspeed_ms = 14.f};
    fwcpp::quadplane::poscontrol_apply_set_state(
        pc, PositionControlState::kPosition1, in, sink, land);
    REQUIRE(pc.state == PositionControlState::kPosition1);
    REQUIRE(sink.reset_yaw_target);
    REQUIRE(pc.pos1_speed_limit_ms == 14.f);
    REQUIRE_FALSE(pc.reached_wp_speed);
    REQUIRE(sink.qpos_log_writes == 2);
    REQUIRE(pc.last_run_ms == 1000);
}

TEST_CASE("set_state land final resets landing detect stub", "[quadplane][poscontrol]") {
    PosControlState pc{};
    PosControlLandStub land{.land_start_ms = 50, .lower_limit_start_ms = 60};
    PosControlSetStateSink sink{};
    PosControlSetStateInputs in{.now_ms = 2000, .ahrs_position_ne_reset_count = 7};
    fwcpp::quadplane::poscontrol_apply_set_state(
        pc, PositionControlState::kLandFinal, in, sink, land);
    REQUIRE(pc.ahrs_position_ne_reset_count == 7);
    REQUIRE(sink.reset_landing_detect);
    REQUIRE(land.land_start_ms == 0);
    REQUIRE(land.lower_limit_start_ms == 0);
}

TEST_CASE("poscontrol_init_approach far wp selects approach", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    ApproachInitView view{.dist_m = 200.f, .transition_threshold_m = 50.f};
    PosControlSetStateInputs st{.now_ms = 5000, .groundspeed_ms = 10.f};
    const auto result = qp.poscontrol_init_approach(view, st);
    REQUIRE(result.chosen == PositionControlState::kApproach);
    REQUIRE(qp.poscontrol().state == PositionControlState::kApproach);
    REQUIRE_FALSE(result.transition_prep.set_last_fw_pitch);
}

TEST_CASE("poscontrol_init_approach close fw selects airbrake", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    ApproachInitView view{.dist_m = 20.f, .transition_threshold_m = 50.f};
    const auto result = qp.poscontrol_init_approach(view, {});
    REQUIRE(result.chosen == PositionControlState::kAirbrake);
    REQUIRE(qp.last_set_state_sink().clear_d_integrator);
}

TEST_CASE("poscontrol_init_approach close tailsitter skips airbrake", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    ApproachInitView view{
        .dist_m = 20.f, .transition_threshold_m = 50.f, .tailsitter_enabled = true};
    const auto result = qp.poscontrol_init_approach(view, {});
    REQUIRE(result.chosen == PositionControlState::kPosition1);
    REQUIRE(qp.last_transition_prep().set_last_fw_pitch);
}

TEST_CASE("poscontrol_init_approach disable approach option", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    qp.set_options(static_cast<std::int32_t>(QOption::kDisableApproach));
    ApproachInitView view{.dist_m = 200.f, .transition_threshold_m = 50.f};
    const auto result = qp.poscontrol_init_approach(view, {});
    REQUIRE(result.chosen == PositionControlState::kPosition1);
}
