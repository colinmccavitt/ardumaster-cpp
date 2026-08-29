#pragma once

#include <fwcpp/quadplane_transition/transition_fsm.hpp>
#include <fwcpp/quadplane_transition/transition_state.hpp>
#include <fwcpp/quadplane_transition/transition_timing.hpp>
#include <fwcpp/quadplane/quadplane_subsystems.hpp>

#include <cstdint>

namespace fwcpp::quadplane {

struct QuadPlaneUpdateView {
    std::uint32_t now_ms{0};
    bool armed_and_safety_off{false};
    bool in_vtol_mode{false};
    bool have_airspeed{false};
    float airspeed_ms{0.f};
    float airspeed_min_ms{0.f};
    bool should_assist{false};
    bool tilt_forward_complete{false};
    bool tiltrotor_with_ground_speed{false};
    bool motor_test_running{false};
};

struct QuadPlaneUpdateTick {
    fwcpp::quadplane_transition::TransFailOutcome trans_fail{
        fwcpp::quadplane_transition::TransFailOutcome::kContinue};
    fwcpp::quadplane_transition::TransitionPhase phase{
        fwcpp::quadplane_transition::TransitionPhase::kVtol};
    bool ran_transition_update{false};
    bool ran_tiltrotor_update{false};
};

[[nodiscard]] inline fwcpp::quadplane_transition::TransitionPhase mav_vtol_phase(
    bool in_vtol_mode,
    const fwcpp::quadplane_transition::SltTransition& slt) {
    using fwcpp::quadplane_transition::TransitionPhase;
    using fwcpp::quadplane_transition::TransitionState;
    if (in_vtol_mode) {
        return TransitionPhase::kVtol;
    }
    if (slt.state() == TransitionState::kDone) {
        return TransitionPhase::kAir;
    }
    return TransitionPhase::kTransition;
}

inline void wire_slt_options(fwcpp::quadplane_transition::SltTransition& slt, std::int32_t q_options) {
    slt.set_q_options(q_options);
}

[[nodiscard]] inline QuadPlaneUpdateTick run_quadplane_update(
    fwcpp::quadplane_transition::SltTransition& slt,
    VtolSubsystemsState& subsystems,
    bool available,
    bool assisted_flight,
    std::int32_t q_options,
    const QuadPlaneUpdateView& view) {
    QuadPlaneUpdateTick tick{};
    if (!available) {
        return tick;
    }
    // QuadPlane::update ~1739: motor_test.running skips transition and tiltrotor.update.
    if (view.motor_test_running) {
        return tick;
    }
    wire_slt_options(slt, q_options);
    slt.reset_fail_timer_if_disarmed(view.now_ms, view.armed_and_safety_off);

    const bool assist = view.should_assist || assisted_flight;
    const bool run_transition = !view.in_vtol_mode || slt.in_transition();
    if (run_transition) {
        tick.ran_transition_update = true;
        slt.update_forward_timing(view.now_ms, view.have_airspeed, view.airspeed_ms, view.airspeed_min_ms,
                                  assist, view.tilt_forward_complete);
        tick.trans_fail = slt.apply_transition_fail(view.now_ms, view.tiltrotor_with_ground_speed);
    }

    tick.phase = mav_vtol_phase(view.in_vtol_mode, slt);
    const auto sub = wire_vtol_subsystems_update(subsystems);
    tick.ran_tiltrotor_update = sub.ran_tiltrotor_update;
    return tick;
}

}  // namespace fwcpp::quadplane
