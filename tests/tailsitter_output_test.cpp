#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/tailsitter/tailsitter_output.hpp>
#include <fwcpp/tailsitter/tailsitter_transition_complete.hpp>

using fwcpp::tailsitter::CopterSurfaceMapInputs;
using fwcpp::tailsitter::ForwardMotorMaskInputs;
using fwcpp::tailsitter::MixingInputs;
using fwcpp::tailsitter::OutputPath;
using fwcpp::tailsitter::SurfaceAssign;
using fwcpp::tailsitter::TailsitterOutputInputs;
using fwcpp::tailsitter::TailsitterTransition;
using fwcpp::tailsitter::TailsitterTransitionState;
using fwcpp::tailsitter::TransitionCompleteSample;
using fwcpp::tailsitter::VectoredForwardTiltInputs;
using fwcpp::tailsitter::VectoredHoverAssistInputs;
using fwcpp::tailsitter::VectoredHoverVtolInputs;
using fwcpp::tailsitter::VtolTransitionThrottleInputs;
using fwcpp::tailsitter::apply_vectored_forward_tilt;
using fwcpp::tailsitter::apply_vectored_hover_assist_tilt;
using fwcpp::tailsitter::apply_vectored_hover_vtol_tilt;
using fwcpp::tailsitter::compute_forward_motor_mask;
using fwcpp::tailsitter::compute_vtol_transition_throttle;
using fwcpp::tailsitter::extra_hover_elevator;
using fwcpp::tailsitter::kTailsitterServoMax;
using fwcpp::tailsitter::map_copter_surfaces_to_plane;
using fwcpp::tailsitter::mix_elevons_vtail;
using fwcpp::tailsitter::output_requires_motor_min;
using fwcpp::tailsitter::output_runs_assisted_copter;
using fwcpp::tailsitter::output_should_run;
using fwcpp::tailsitter::output_uses_fw_or_vtol_trans_path;
using fwcpp::tailsitter::q_assist_motors_only_relax;
using fwcpp::tailsitter::surface_saturation_limits;
using fwcpp::tailsitter::tailsitter_output;
using fwcpp::tailsitter::throttle_pwm_from_actuator;
using fwcpp::tailsitter::transition_fw_complete_bool;
using fwcpp::tailsitter::transition_vtol_complete_bool;

TEST_CASE("tailsitter output gates and motor mask", "[tailsitter][output]") {
    REQUIRE(output_should_run({.enabled = true}));
    REQUIRE(!output_should_run({.motor_test_running = true}));
    REQUIRE(!output_should_run({.enabled = true, .quadplane_initialised = false}));
    REQUIRE(output_requires_motor_min({.soft_armed = false}));
    REQUIRE(output_requires_motor_min({.emergency_stop = true}));
    REQUIRE(!output_requires_motor_min({}));

    const auto mask = compute_forward_motor_mask({.throttle = 0.6f, .motor_mask = 0b11});
    REQUIRE(mask.motor_mask == 0b11);
    REQUIRE_THAT(mask.throttle, Catch::Matchers::WithinAbs(0.6f, 1e-6f));
}

TEST_CASE("tailsitter vtol transition throttle", "[tailsitter][output]") {
    VtolTransitionThrottleInputs in{};
    in.transition_throttle_vtol = 50.0f;
    const auto pos = compute_vtol_transition_throttle(in);
    REQUIRE(pos.apply);
    REQUIRE(pos.center_rudder);
    REQUIRE_THAT(pos.throttle_thrust, Catch::Matchers::WithinAbs(0.5f, 1e-6f));

    in.transition_throttle_vtol = -1.0f;
    in.hover_thrust = 0.35f;
    in.cruise_thrust = 0.42f;
    const auto hover = compute_vtol_transition_throttle(in);
    REQUIRE_THAT(hover.throttle_thrust, Catch::Matchers::WithinAbs(0.42f, 1e-6f));

    in.cruise_thrust = 0.20f;
    const auto hover_wins = compute_vtol_transition_throttle(in);
    REQUIRE_THAT(hover_wins.throttle_thrust, Catch::Matchers::WithinAbs(0.35f, 1e-6f));

    in.throttle_wait = true;
    REQUIRE(!compute_vtol_transition_throttle(in).apply);
}

TEST_CASE("tailsitter hover and forward tilt outputs", "[tailsitter][output]") {
    float tl = 0.0f;
    float tr = 0.0f;
    apply_vectored_forward_tilt(VectoredForwardTiltInputs{.vectored_forward_gain = 0.5f,
                                                          .aileron_scaled = 1000.0f,
                                                          .elevator_scaled = 2000.0f},
                                tl, tr);
    REQUIRE_THAT(tl, Catch::Matchers::WithinAbs(1500.0f, 1e-3f));
    REQUIRE_THAT(tr, Catch::Matchers::WithinAbs(500.0f, 1e-3f));

    apply_vectored_hover_assist_tilt(
        VectoredHoverAssistInputs{.vectored_hover_gain = 0.5f, .tilt_left_in = 2000.0f}, tl, tr);
    REQUIRE_THAT(tl, Catch::Matchers::WithinAbs(700.0f, 1e-3f));

    apply_vectored_hover_vtol_tilt(VectoredHoverVtolInputs{.vectored_hover_gain = 0.5f,
                                                           .des_pitch_cd = 9000.0f,
                                                           .tilt_left_in = 1000.0f,
                                                           .tilt_right_in = 1000.0f},
                                   tl, tr);
    REQUIRE_THAT(tl, Catch::Matchers::WithinAbs(5000.0f, 1e-3f));
    REQUIRE_THAT(tr, Catch::Matchers::WithinAbs(5000.0f, 1e-3f));
}

TEST_CASE("tailsitter extra hover elevator power law", "[tailsitter][output]") {
    REQUIRE_THAT(extra_hover_elevator(0.0f, 0.0f, 2.5f, true), Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(extra_hover_elevator(9000.0f, 0.0f, 2.5f, false),
                 Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    const float half = extra_hover_elevator(4500.0f, 0.0f, 2.5f, true);
    REQUIRE_THAT(half, Catch::Matchers::WithinAbs(795.4951286f, 1e-3f));
}

TEST_CASE("tailsitter copter surfaces mix and transition bool adapters", "[tailsitter][output]") {
    const auto surf = map_copter_surfaces_to_plane(CopterSurfaceMapInputs{});
    REQUIRE_THAT(surf.aileron_scaled, Catch::Matchers::WithinAbs(-450.0f, 1e-3f));
    REQUIRE_THAT(surf.elevator_scaled, Catch::Matchers::WithinAbs(900.0f, 1e-3f));
    REQUIRE_THAT(surf.rudder_scaled, Catch::Matchers::WithinAbs(1350.0f, 1e-3f));

    const auto mix =
        mix_elevons_vtail(MixingInputs{.elevator_scaled = 3000.0f, .aileron_scaled = 4000.0f},
                          SurfaceAssign{.elevon = true});
    REQUIRE_THAT(mix.elevon_left, Catch::Matchers::WithinAbs(1500.0f, 1e-3f));
    REQUIRE_THAT(mix.elevon_right, Catch::Matchers::WithinAbs(4500.0f, 1e-3f));
    REQUIRE(mix.yaw_lim);

    const auto saturated =
        mix_elevons_vtail(MixingInputs{.elevator_scaled = 5000.0f, .aileron_scaled = 1000.0f},
                          SurfaceAssign{.elevon = true, .v_tail = true});
    REQUIRE_THAT(saturated.elevon_left, Catch::Matchers::WithinAbs(5000.0f, 1e-3f));
    REQUIRE(saturated.yaw_lim);
    REQUIRE(saturated.pitch_lim);
    REQUIRE(saturated.roll_lim);

    TailsitterTransition ts{};
    TransitionCompleteSample s{};
    s.armed_and_safety_off = false;
    REQUIRE(transition_fw_complete_bool(ts, s));
    REQUIRE(transition_vtol_complete_bool(ts, s));
}

TEST_CASE("tailsitter Q assist motors-only I-relax", "[tailsitter][output]") {
    const auto with_surfaces =
        q_assist_motors_only_relax(SurfaceAssign{.elevator = true, .rudder = true});
    REQUIRE(with_surfaces.relax_yaw_integrator);
    REQUIRE(with_surfaces.relax_pitch_integrator);
    REQUIRE(with_surfaces.relax_roll_integrator);
    REQUIRE(!with_surfaces.reset_plane_pitch_i);
    REQUIRE(!with_surfaces.reset_plane_yaw_i);

    const auto motors_only = q_assist_motors_only_relax(SurfaceAssign{});
    REQUIRE(motors_only.reset_plane_pitch_i);
    REQUIRE(motors_only.reset_plane_yaw_i);
    REQUIRE(!motors_only.relax_pitch_integrator);
    REQUIRE(!motors_only.relax_roll_integrator);
}

TEST_CASE("tailsitter output orchestrator FW and skip paths", "[tailsitter][output]") {
    REQUIRE(output_uses_fw_or_vtol_trans_path(false, false));
    REQUIRE(output_uses_fw_or_vtol_trans_path(true, true));
    REQUIRE(!output_uses_fw_or_vtol_trans_path(true, false));

    TailsitterOutputInputs skip{};
    skip.skip.motor_test_running = true;
    REQUIRE(tailsitter_output(skip).path == OutputPath::kSkip);

    TailsitterOutputInputs fw{};
    fw.active = false;
    fw.fw_throttle = 0.55f;
    fw.motor_mask.motor_mask = 0b0101;
    fw.motor_mask.rudder_dt = 0.2f;
    fw.forward_tilt.vectored_forward_gain = 0.5f;
    fw.forward_tilt.elevator_scaled = 2000.0f;
    fw.forward_tilt.aileron_scaled = 1000.0f;
    const auto fwd = tailsitter_output(fw);
    REQUIRE(fwd.path == OutputPath::kForwardFlight);
    REQUIRE(fwd.apply_fw_motor_mask);
    REQUIRE_THAT(fwd.motor_mask.throttle, Catch::Matchers::WithinAbs(0.55f, 1e-6f));
    REQUIRE(fwd.motor_mask.motor_mask == 0b0101);
    REQUIRE_THAT(fwd.tilt_left, Catch::Matchers::WithinAbs(1500.0f, 1e-3f));
    REQUIRE(!fwd.motors_output);
}

TEST_CASE("tailsitter output orchestrator VTOL transition unassisted", "[tailsitter][output]") {
    TailsitterOutputInputs in{};
    in.active = true;
    in.in_vtol_transition = true;
    in.assisted_flight = false;
    in.armed_and_safety_off = true;
    in.vtol_throttle.transition_throttle_vtol = 40.0f;
    in.selected_thrust_as_actuator = 0.45f;
    in.pwm_min = 1000;
    in.pwm_max = 2000;
    in.motor_mask.motor_mask = 0b11;
    in.motor_mask.rudder_dt = 0.3f;
    const auto out = tailsitter_output(in);
    REQUIRE(out.path == OutputPath::kForwardFlight);
    REQUIRE(out.center_rudder);
    REQUIRE(out.zero_rudder_dt);
    REQUIRE(out.set_attitude_throttle);
    REQUIRE_THAT(out.attitude_throttle, Catch::Matchers::WithinAbs(0.40f, 1e-6f));
    REQUIRE_THAT(out.throttle, Catch::Matchers::WithinAbs(0.45f, 1e-6f));
    REQUIRE(out.set_throttle_pwm);
    REQUIRE(out.throttle_pwm == throttle_pwm_from_actuator(0.45f, 1000, 2000));
    REQUIRE(out.motor_mask.rudder_dt == 0.0f);
}

TEST_CASE("tailsitter output orchestrator Q assist and ANGLE_WAIT_FW", "[tailsitter][output]") {
    REQUIRE(output_runs_assisted_copter(true, TailsitterTransitionState::kDone));
    REQUIRE(!output_runs_assisted_copter(true, TailsitterTransitionState::kAngleWaitFw));

    TailsitterOutputInputs assist{};
    assist.active = false;
    assist.assisted_flight = true;
    assist.q_assist_motors_only = true;
    assist.fw_throttle = 0.3f;
    assist.surfaces.elevator = true;
    assist.surfaces.rudder = true;
    assist.hover_assist.vectored_hover_gain = 0.5f;
    assist.hover_assist.tilt_left_in = 2000.0f;
    assist.hover_assist.tilt_right_in = 0.0f;
    const auto q = tailsitter_output(assist);
    REQUIRE(q.path == OutputPath::kQAssistMotorsOnly);
    REQUIRE(q.hold_stabilize);
    REQUIRE(q.motors_output_assisted);
    REQUIRE(q.assist_relax.relax_pitch_integrator);
    REQUIRE(q.assist_relax.relax_roll_integrator);
    REQUIRE_THAT(q.tilt_left, Catch::Matchers::WithinAbs(700.0f, 1e-3f));
    REQUIRE(!q.reset_plane_i);

    TailsitterOutputInputs wait_fw{};
    wait_fw.active = true;
    wait_fw.assisted_flight = true;
    wait_fw.transition_state = TailsitterTransitionState::kAngleWaitFw;
    wait_fw.copter.motor_yaw = 0.1f;
    wait_fw.copter.motor_pitch = 0.2f;
    wait_fw.copter.motor_roll = 0.3f;
    const auto vtol = tailsitter_output(wait_fw);
    REQUIRE(vtol.path == OutputPath::kVtolCopter);
    REQUIRE(vtol.motors_output);
    REQUIRE(!vtol.motors_output_assisted);
    REQUIRE(vtol.reset_plane_i);
    REQUIRE(vtol.apply_speed_scaling);
}

TEST_CASE("tailsitter output orchestrator VTOL mix and saturation", "[tailsitter][output]") {
    TailsitterOutputInputs in{};
    in.active = true;
    in.armed_and_safety_off = false;
    in.have_tailsitter_motors = true;
    in.is_vectored = true;
    in.surfaces.elevator = true;
    in.surfaces.aileron = true;
    in.surfaces.rudder = true;
    in.surfaces.elevon = true;
    in.copter.motor_yaw = 1.0f;
    in.copter.motor_pitch = 1.0f;
    in.copter.motor_roll = 1.0f;
    in.hover_vtol.vectored_hover_gain = 1.0f;
    in.hover_vtol.tilt_left_in = 4500.0f;
    in.hover_vtol.tilt_right_in = 4500.0f;
    const auto out = tailsitter_output(in);
    REQUIRE(out.path == OutputPath::kVtolCopter);
    REQUIRE(out.set_min_throttle_zero);
    REQUIRE(!out.apply_speed_scaling);
    REQUIRE(out.limits.pitch);
    REQUIRE(out.limits.yaw);
    REQUIRE(out.limits.roll);

    const auto sat = surface_saturation_limits(in.surfaces, 4500.0f, 0.0f, 0.0f, 0.0f, 0.0f, true);
    REQUIRE(sat.pitch);
    REQUIRE(sat.yaw);
    REQUIRE(!sat.roll);
}
