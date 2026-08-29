#pragma once

#include <fwcpp/quadplane_transition/transition_fsm.hpp>
#include <fwcpp/quadplane_transition/transition_state.hpp>
#include <fwcpp/quadplane_transition/transition_timing.hpp>

#include <algorithm>
#include <cstdint>

namespace fwcpp::quadplane_transition {

enum class DesiredSpoolState : std::uint8_t {
    kShutDown = 0,
    kThrottleUnlimited = 1,
};

struct SltUpdateInputs {
    std::uint32_t now_ms{0};
    bool armed_and_safety_off{false};
    bool have_airspeed{false};
    float airspeed_ms{0.f};
    float airspeed_min_ms{10.f};
    bool should_assist{false};
    bool rotors_fully_fwd{false};
    bool tilt_fwd_complete{true};
    bool tiltrotor_enabled{false};
    bool tiltrotor_vectored{false};
    bool tiltrotor_has_fw_motor{true};
    bool level_transition_option{false};
    float assist_climb_rate_cms{100.f};
    float last_motor_throttle{0.5f};
    bool tiltrotor_with_ground_speed{false};
};

struct SltUpdateOutputs {
    bool assisted_flight{false};
    bool use_synthetic_airspeed{false};
    DesiredSpoolState spool{DesiredSpoolState::kThrottleUnlimited};
    bool call_motors_output{false};
    bool hold_hover{false};
    float hold_hover_climb_rate_cms{0.f};
    bool hold_stabilize{false};
    float hold_stabilize_throttle{0.f};
    float attitude_throttle_mix{1.f};
    bool set_throttle_mix_max{false};
    bool reset_pitch_roll_i{false};
    bool reset_tecs_throttle_i{false};
    bool coordinated_yaw_rate{false};
    bool stamp_last_fw_pitch{false};
    TransFailOutcome trans_fail{TransFailOutcome::kContinue};
};

inline SltUpdateOutputs run_slt_update(SltTransition& fsm, const SltUpdateInputs& in) {
    SltUpdateOutputs out{};
    fsm.reset_fail_timer_if_disarmed(in.now_ms, in.armed_and_safety_off);

    out.assisted_flight = in.should_assist;
    if (in.should_assist && !fsm.in_forced_transition()) {
        if (fsm.transition_start_ms() == 0) {
            fsm.update_airspeed_wait(in.now_ms, in.have_airspeed, in.airspeed_ms, in.airspeed_min_ms, true);
        } else if (fsm.state() != TransitionState::kAirspeedWait) {
            (void)fsm.set_state(TransitionState::kAirspeedWait);
        }
    }

    if (in.rotors_fully_fwd && fsm.state() != TransitionState::kAirspeedWait) {
        fsm.mark_transition_done();
    }

    const TransitionState state = fsm.state();
    out.use_synthetic_airspeed = state != TransitionState::kDone;

    switch (state) {
        case TransitionState::kAirspeedWait: {
            out.spool = DesiredSpoolState::kThrottleUnlimited;
            out.trans_fail = fsm.apply_transition_fail(in.now_ms, in.tiltrotor_with_ground_speed);
            fsm.update_airspeed_wait(in.now_ms, in.have_airspeed, in.airspeed_ms, in.airspeed_min_ms, true);
            out.assisted_flight = true;
            float climb = in.assist_climb_rate_cms;
            if (in.level_transition_option && !in.tiltrotor_enabled) {
                climb = std::min(climb, 0.0f);
            }
            out.hold_hover = true;
            out.hold_hover_climb_rate_cms = climb;
            if (!in.tiltrotor_vectored) {
                out.coordinated_yaw_rate = true;
            }
            if (in.tiltrotor_enabled && !in.tiltrotor_has_fw_motor) {
                out.reset_tecs_throttle_i = true;
            }
            fsm.record_motor_throttle(in.last_motor_throttle);
            break;
        }
        case TransitionState::kTimer: {
            out.spool = DesiredSpoolState::kThrottleUnlimited;
            const std::uint32_t trans_ms = fsm.timer_duration_ms();
            const std::uint32_t elapsed = in.now_ms - fsm.transition_low_airspeed_ms();
            fsm.update_timer(in.now_ms, in.tilt_fwd_complete);
            const float transition_scale =
                trans_ms > 0 ? static_cast<float>(trans_ms - elapsed) / static_cast<float>(trans_ms) : 0.f;
            float throttle_scaled = fsm.last_throttle() * transition_scale;
            out.attitude_throttle_mix = 0.5f * transition_scale;
            if (throttle_scaled < 0.01f) {
                throttle_scaled = 0.01f;
            }
            out.hold_stabilize = true;
            out.hold_stabilize_throttle = throttle_scaled;
            out.assisted_flight = true;
            if (!in.tiltrotor_vectored) {
                out.coordinated_yaw_rate = true;
            }
            break;
        }
        case TransitionState::kDone:
            out.spool = DesiredSpoolState::kShutDown;
            out.call_motors_output = true;
            out.stamp_last_fw_pitch = true;
            out.use_synthetic_airspeed = false;
            return out;
    }

    out.reset_pitch_roll_i = (state == TransitionState::kAirspeedWait);
    out.set_throttle_mix_max = (state == TransitionState::kAirspeedWait);
    out.call_motors_output = true;
    out.stamp_last_fw_pitch = true;
    return out;
}

}  // namespace fwcpp::quadplane_transition
