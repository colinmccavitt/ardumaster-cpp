#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_vtol_position_controller.hpp>

using Catch::Approx;
using fwcpp::quadplane::DesiredSpoolState;
using fwcpp::quadplane::InVtolModeInputs;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::VtolPosText;
using fwcpp::quadplane::VtolPositionControllerInputs;
using fwcpp::quadplane::kShouldRelaxLowerLimitMs;
using fwcpp::quadplane::kVtolMinAirbrakeMs;
using fwcpp::quadplane::kVtolVelocityMatchFreshMs;

static QuadPlane available_qp() {
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    return qp;
}

static VtolPositionControllerInputs available_in(std::uint32_t now_ms) {
    VtolPositionControllerInputs in{};
    in.now_ms = now_ms;
    in.armed_and_safety_off = true;
    in.in_vtol.available = true;
    return in;
}

TEST_CASE("vtol position controller no-op when unavailable", "[quadplane][vtol_pos]") {
    QuadPlane qp{1};
    const auto tick = qp.vtol_position_controller({});
    REQUIRE_FALSE(tick.ran);
}

TEST_CASE("vtol position controller none failsafe to position1", "[quadplane][vtol_pos]") {
    auto qp = available_qp();
    VtolPositionControllerInputs in{.now_ms = 9000, .armed_and_safety_off = true};
    in.in_vtol.available = true;
    const auto tick = qp.vtol_position_controller(in);
    REQUIRE(tick.ran);
    REQUIRE(tick.none_to_position1_failsafe);
    REQUIRE(tick.flow_of_control);
    REQUIRE(qp.poscontrol().state == PositionControlState::kPosition1);
    REQUIRE(qp.poscontrol().last_run_ms == 9000);
}

TEST_CASE("vtol position controller approach nvtol failsafe fallthrough", "[quadplane][vtol_pos]") {
    auto qp = available_qp();
    qp.poscontrol_mut().state = PositionControlState::kApproach;
    VtolPositionControllerInputs in = available_in(1000);
    in.in_vtol.control_is_vtol_mode = true;
    const auto tick = qp.vtol_position_controller(in);
    REQUIRE(tick.approach_nvtol_failsafe);
    REQUIRE(tick.flow_of_control);
    REQUIRE(tick.send_text == VtolPosText::kPosition1Nvtol);
    REQUIRE(tick.fw_nav_update_waypoint);
    REQUIRE(tick.tecs_throttle);
    REQUIRE(tick.calc_nav_roll);
    REQUIRE(qp.poscontrol().state == PositionControlState::kPosition1);
}

TEST_CASE("vtol position controller airbrake to position1 by speed and time", "[quadplane][vtol_pos]") {
    auto qp = available_qp();
    qp.poscontrol_mut().state = PositionControlState::kAirbrake;
    qp.poscontrol_mut().last_state_change_ms = 0;
    VtolPositionControllerInputs in = available_in(kVtolMinAirbrakeMs + 200);
    in.aspeed_ms = 5.f;
    in.have_airspeed = true;
    in.airspeed_min = 9.f;
    in.assist_speed = 0.f;
    in.throttle_scaled = 40.f;
    in.throttle_cruise = 50.f;
    in.closing_vel_north_ms = 4.f;
    in.desired_closing_vel_north_ms = 8.f;
    const auto tick = qp.vtol_position_controller(in);
    REQUIRE(qp.poscontrol().state == PositionControlState::kPosition1);
    REQUIRE(tick.set_last_fw_pitch);
    REQUIRE(tick.send_text == VtolPosText::kPosition1);
    REQUIRE(tick.vel_forward_last_ms == in.now_ms);
    REQUIRE(tick.vel_forward_integrator >= 0.f);
    REQUIRE(tick.vel_forward_integrator <= in.throttle_cruise * 0.5f);
}

TEST_CASE("vtol position controller approach airbrake vs tailsitter position1", "[quadplane][vtol_pos]") {
    SECTION("non-tailsitter enters airbrake") {
        auto qp = available_qp();
        qp.poscontrol_mut().state = PositionControlState::kApproach;
        VtolPositionControllerInputs in = available_in(100);
        in.groundspeed_ms = 10.f;
        in.closing_vel_north_ms = 5.f;
        in.distance_m = 20.f;
        in.desired_spool = DesiredSpoolState::kGroundIdle;
        const auto tick = qp.vtol_position_controller(in);
        REQUIRE(tick.send_text == VtolPosText::kAirbrake);
        REQUIRE(qp.poscontrol().state == PositionControlState::kAirbrake);
        REQUIRE_FALSE(tick.set_last_fw_pitch);
    }
    SECTION("tailsitter skips airbrake to position1") {
        auto qp = available_qp();
        qp.poscontrol_mut().state = PositionControlState::kApproach;
        VtolPositionControllerInputs in = available_in(100);
        in.groundspeed_ms = 10.f;
        in.closing_vel_north_ms = 5.f;
        in.distance_m = 20.f;
        in.tailsitter_enabled = true;
        const auto tick = qp.vtol_position_controller(in);
        REQUIRE(tick.send_text == VtolPosText::kPosition1);
        REQUIRE(tick.set_last_fw_pitch);
        REQUIRE(qp.poscontrol().state == PositionControlState::kPosition1);
    }
}

TEST_CASE("vtol position controller position1 to position2 by dist and speed", "[quadplane][vtol_pos]") {
    auto qp = available_qp();
    qp.poscontrol_mut().state = PositionControlState::kPosition1;
    VtolPositionControllerInputs in = available_in(4000);
    in.distance_m = 5.f;
    in.wp_distance_north_m = 5.f;
    in.closing_vel_north_ms = 1.f;
    in.tilt_angle_achieved = true;
    const auto tick = qp.vtol_position_controller(in);
    REQUIRE(tick.setup_target_position);
    REQUIRE(tick.input_vel_accel_NE);
    REQUIRE(tick.run_xy);
    REQUIRE(tick.assign_tilt);
    REQUIRE(tick.send_text == VtolPosText::kPosition2Started);
    REQUIRE(qp.poscontrol().state == PositionControlState::kPosition2);
}

TEST_CASE("vtol position controller land final relax vs vel", "[quadplane][vtol_pos]") {
    SECTION("should_relax uses NE_relax") {
        auto qp = available_qp();
        qp.poscontrol_mut().state = PositionControlState::kLandFinal;
        qp.poscontrol_land_mut().lower_limit_start_ms = 1;
        VtolPositionControllerInputs in = available_in(1 + kShouldRelaxLowerLimitMs + 1);
        in.relax.throttle = 0.f;
        in.relax.now_ms = in.now_ms;
        const auto tick = qp.vtol_position_controller(in);
        REQUIRE(tick.should_relax);
        REQUIRE(tick.NE_relax);
        REQUIRE_FALSE(tick.input_vel_accel_NE);
        REQUIRE_FALSE(tick.input_pos_vel_accel_NE);
    }
    SECTION("low throttle uses velocity") {
        auto qp = available_qp();
        qp.poscontrol_mut().state = PositionControlState::kLandFinal;
        qp.poscontrol_mut().last_pos_reset_ms = 11;
        VtolPositionControllerInputs in = available_in(200);
        in.relax.throttle = 0.8f;
        in.throttle_lower = true;
        in.motors_throttle = 0.8f;
        in.throttle_hover = 1.f;
        in.last_pos_ne_reset_ms = 11;
        const auto tick = qp.vtol_position_controller(in);
        REQUIRE_FALSE(tick.should_relax);
        REQUIRE(tick.input_vel_accel_NE);
        REQUIRE_FALSE(tick.input_pos_vel_accel_NE);
    }
    SECTION("matched pos reset uses pos") {
        auto qp = available_qp();
        qp.poscontrol_mut().state = PositionControlState::kLandFinal;
        qp.poscontrol_mut().last_pos_reset_ms = 11;
        VtolPositionControllerInputs in = available_in(200);
        in.relax.throttle = 0.8f;
        in.motors_throttle = 0.8f;
        in.throttle_hover = 1.f;
        in.last_pos_ne_reset_ms = 11;
        const auto tick = qp.vtol_position_controller(in);
        REQUIRE_FALSE(tick.should_relax);
        REQUIRE(tick.input_pos_vel_accel_NE);
        REQUIRE_FALSE(tick.input_vel_accel_NE);
    }
}

TEST_CASE("vtol position controller height abort climb", "[quadplane][vtol_pos]") {
    auto qp = available_qp();
    qp.poscontrol_mut().state = PositionControlState::kLandAbort;
    VtolPositionControllerInputs in = available_in(3000);
    in.speed_up_ms = 2.5f;
    const auto tick = qp.vtol_position_controller(in);
    REQUIRE(tick.set_climb_rate);
    REQUIRE(tick.climb_rate_ms == Approx(2.5f));
    REQUIRE_FALSE(tick.landing_descent_rate);
    REQUIRE(tick.run_z_controller);
}

TEST_CASE("vtol position controller suppress_z skips run_z", "[quadplane][vtol_pos]") {
    auto qp = available_qp();
    qp.poscontrol_mut().state = PositionControlState::kAirbrake;
    VtolPositionControllerInputs in = available_in(100);
    in.tiltrotor_enabled = true;
    in.current_tilt = 1.f;
    in.fully_forward_tilt = 1.f;
    const auto tick = qp.vtol_position_controller(in);
    REQUIRE(tick.suppress_z_controller);
    REQUIRE(tick.hold_stabilize);
    REQUIRE(tick.hold_stabilize_throttle == Approx(0.01f));
    REQUIRE_FALSE(tick.run_z_controller);
}

TEST_CASE("vtol position controller landing velocity window", "[quadplane][vtol_pos]") {
    SECTION("fresh match is used") {
        auto qp = available_qp();
        qp.poscontrol_mut().velocity_match_north_ms = 1.5f;
        qp.poscontrol_mut().velocity_match_east_ms = -0.5f;
        qp.poscontrol_mut().last_velocity_match_ms = 400;
        VtolPositionControllerInputs in = available_in(400 + kVtolVelocityMatchFreshMs - 1);
        const auto tick = qp.vtol_position_controller(in);
        REQUIRE(tick.landing_velocity_north_ms == Approx(1.5f));
        REQUIRE(tick.landing_velocity_east_ms == Approx(-0.5f));
    }
    SECTION("stale match is zero") {
        auto qp = available_qp();
        qp.poscontrol_mut().velocity_match_north_ms = 1.5f;
        qp.poscontrol_mut().velocity_match_east_ms = -0.5f;
        qp.poscontrol_mut().last_velocity_match_ms = 400;
        VtolPositionControllerInputs in = available_in(400 + kVtolVelocityMatchFreshMs);
        const auto tick = qp.vtol_position_controller(in);
        REQUIRE(tick.landing_velocity_north_ms == Approx(0.f));
        REQUIRE(tick.landing_velocity_east_ms == Approx(0.f));
    }
}
