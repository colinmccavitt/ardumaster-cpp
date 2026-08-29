#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_transition.hpp>
#include <fwcpp/tailsitter/tailsitter_types.hpp>

namespace fwcpp::tailsitter {

/*
  Pure-function port of Tailsitter::output() (Plane-4.7.0 tailsitter.cpp ~285).
  SRV / motors / attitude_control side effects are returned as flags and values;
  the caller applies them. speed_scaling() itself is not ported here — the
  orchestrator only reports the armed call site.
 */

inline constexpr float kTailsitterServoMax = 4500.0f;

enum class OutputPath : std::uint8_t {
    kSkip = 0,
    kForwardFlight = 1,
    kQAssistMotorsOnly = 2,
    kVtolCopter = 3,
};

struct OutputSkipInputs {
    bool enabled{true};
    bool motor_test_running{false};
    bool quadplane_initialised{true};
};

struct DisarmMotorInputs {
    bool soft_armed{true};
    bool emergency_stop{false};
};

struct VtolTransitionThrottleInputs {
    bool armed_and_safety_off{true};
    bool in_vtol_transition{true};
    bool throttle_wait{false};
    float transition_throttle_vtol{kTransitionThrottleVtolDefault};
    float hover_thrust{0.35f};
    float cruise_thrust{0.25f};
};

struct VtolTransitionThrottleResult {
    bool apply{false};
    float throttle_thrust{0.0f};
    bool center_rudder{false};
};

struct ForwardMotorMaskInputs {
    float throttle{0.5f};
    float rudder_dt{0.1f};
    std::uint32_t motor_mask{0};
};

struct ForwardMotorMaskOutputs {
    float throttle{0.0f};
    std::uint32_t motor_mask{0};
    float rudder_dt{0.0f};
};

struct VectoredForwardTiltInputs {
    float vectored_forward_gain{0.0f};
    float fw_vector_throttle_scaler{1.0f};
    float speed_scaler{1.0f};
    bool manual_mode{false};
    float aileron_scaled{0.0f};
    float elevator_scaled{0.0f};
};

struct VectoredHoverAssistInputs {
    float vectored_hover_gain{0.0f};
    float hover_throttle{0.35f};
    float output_throttle{0.5f};
    float throttle_scale_max{kThrottleScaleMaxDefault};
    float gain_scaling_min{kGainScalingMinDefault};
    float tilt_left_in{100.0f};
    float tilt_right_in{-50.0f};
};

struct VectoredHoverVtolInputs {
    float vectored_hover_gain{0.0f};
    float vectored_hover_power{kVectoredHoverPowerDefault};
    bool in_vtol_mode{true};
    float des_pitch_cd{0.0f};
    float pitch_sensor_cd{0.0f};
    float tilt_left_in{0.0f};
    float tilt_right_in{0.0f};
};

struct CopterSurfaceMapInputs {
    float motor_yaw{0.1f};
    float motor_yaw_ff{0.0f};
    float motor_pitch{0.2f};
    float motor_pitch_ff{0.0f};
    float motor_roll{0.3f};
    float motor_roll_ff{0.0f};
    float vtol_yaw_scale{kVtolYawScaleDefault};
    float vtol_pitch_scale{kVtolPitchScaleDefault};
    float vtol_roll_scale{kVtolRollScaleDefault};
};

struct CopterSurfaceMapOutputs {
    float aileron_scaled{0.0f};
    float elevator_scaled{0.0f};
    float rudder_scaled{0.0f};
};

struct MixingInputs {
    float mixing_offset{0.0f};
    float mixing_gain{1.0f};
    float elevator_scaled{0.0f};
    float aileron_scaled{0.0f};
    float rudder_scaled{0.0f};
};

struct MixingOutputs {
    float elevon_left{0.0f};
    float elevon_right{0.0f};
    float vtail_left{0.0f};
    float vtail_right{0.0f};
    bool yaw_lim{false};
    bool pitch_lim{false};
    bool roll_lim{false};
};

struct MotorLimitFlags {
    bool roll{false};
    bool pitch{false};
    bool yaw{false};
};

struct QAssistRelaxOutputs {
    bool relax_yaw_integrator{true};
    bool limit_yaw{true};
    bool relax_pitch_integrator{false};
    bool limit_pitch{false};
    bool reset_plane_pitch_i{false};
    bool relax_roll_integrator{false};
    bool limit_roll{false};
    bool reset_plane_yaw_i{false};
};

struct TailsitterOutputInputs {
    OutputSkipInputs skip{};
    DisarmMotorInputs disarm{};
    bool active{false};
    bool in_vtol_transition{false};
    bool assisted_flight{false};
    bool throttle_wait{false};
    bool armed_and_safety_off{true};
    TailsitterTransitionState transition_state{TailsitterTransitionState::kDone};
    bool q_assist_motors_only{false};
    bool in_vtol_mode{false};
    bool have_tailsitter_motors{false};
    bool is_vectored{false};
    float fw_throttle{0.0f};
    VtolTransitionThrottleInputs vtol_throttle{};
    float selected_thrust_as_actuator{0.0f};
    std::uint16_t pwm_min{1000};
    std::uint16_t pwm_max{2000};
    ForwardMotorMaskInputs motor_mask{};
    VectoredForwardTiltInputs forward_tilt{};
    VectoredHoverAssistInputs hover_assist{};
    VectoredHoverVtolInputs hover_vtol{};
    CopterSurfaceMapInputs copter{};
    MixingInputs mixing{};
    SurfaceAssign surfaces{};
};

struct TailsitterOutputResult {
    OutputPath path{OutputPath::kSkip};
    bool ran{false};
    bool output_motor_min{false};
    float throttle{0.0f};
    bool center_rudder{false};
    bool zero_rudder_dt{false};
    bool set_attitude_throttle{false};
    float attitude_throttle{0.0f};
    bool set_throttle_pwm{false};
    std::uint16_t throttle_pwm{0};
    float throttle_scaled{0.0f};
    bool apply_fw_motor_mask{false};
    ForwardMotorMaskOutputs motor_mask{};
    float tilt_left{0.0f};
    float tilt_right{0.0f};
    bool hold_stabilize{false};
    bool motors_output{false};
    bool motors_output_assisted{false};
    QAssistRelaxOutputs assist_relax{};
    bool reset_plane_i{false};
    CopterSurfaceMapOutputs copter_surfaces{};
    MixingOutputs mix{};
    MotorLimitFlags limits{};
    bool apply_speed_scaling{false};
    bool set_min_throttle_zero{false};
};

[[nodiscard]] inline constexpr bool output_should_run(const OutputSkipInputs& in) {
    return in.enabled && !in.motor_test_running && in.quadplane_initialised;
}

[[nodiscard]] inline constexpr bool output_requires_motor_min(const DisarmMotorInputs& in) {
    return !in.soft_armed || in.emergency_stop;
}

[[nodiscard]] inline constexpr bool output_uses_fw_or_vtol_trans_path(bool active,
                                                                      bool in_vtol_transition) {
    return !active || in_vtol_transition;
}

[[nodiscard]] inline constexpr bool output_runs_assisted_copter(
    bool assisted_flight, TailsitterTransitionState transition_state) {
    return assisted_flight && (transition_state != TailsitterTransitionState::kAngleWaitFw);
}

[[nodiscard]] inline VtolTransitionThrottleResult compute_vtol_transition_throttle(
    const VtolTransitionThrottleInputs& in) {
    VtolTransitionThrottleResult out{};
    if (!in.armed_and_safety_off || !in.in_vtol_transition || in.throttle_wait) {
        return out;
    }
    out.apply = true;
    out.center_rudder = true;
    if (!math::is_negative(in.transition_throttle_vtol)) {
        out.throttle_thrust = std::min(in.transition_throttle_vtol * 0.01f, 1.0f);
    } else {
        out.throttle_thrust = std::max(in.hover_thrust, in.cruise_thrust);
    }
    return out;
}

[[nodiscard]] inline ForwardMotorMaskOutputs compute_forward_motor_mask(
    const ForwardMotorMaskInputs& in) {
    ForwardMotorMaskOutputs out{};
    out.throttle = in.throttle;
    out.motor_mask = in.motor_mask;
    out.rudder_dt = in.rudder_dt;
    return out;
}

[[nodiscard]] inline float compute_vectored_forward_scaler(bool manual_mode, float fw_vector_scaler,
                                                           float speed_scaler) {
    if (manual_mode) {
        return 1.0f;
    }
    return fw_vector_scaler / speed_scaler;
}

inline void apply_vectored_forward_tilt(const VectoredForwardTiltInputs& in, float& tilt_left,
                                        float& tilt_right) {
    tilt_left = 0.0f;
    tilt_right = 0.0f;
    if (!(in.vectored_forward_gain > 0.0f)) {
        return;
    }
    const float scaler =
        compute_vectored_forward_scaler(in.manual_mode, in.fw_vector_throttle_scaler, in.speed_scaler);
    tilt_left = (in.elevator_scaled + in.aileron_scaled) * in.vectored_forward_gain * scaler;
    tilt_right = (in.elevator_scaled - in.aileron_scaled) * in.vectored_forward_gain * scaler;
}

[[nodiscard]] inline float hover_throttle_scaler(float hover_throttle, float output_throttle,
                                                 float throttle_scale_max, float gain_scaling_min) {
    float scaler = throttle_scale_max;
    if (math::is_positive(output_throttle)) {
        scaler = math::constrain_value(hover_throttle / output_throttle, gain_scaling_min,
                                       throttle_scale_max);
    }
    return scaler;
}

inline void apply_vectored_hover_assist_tilt(const VectoredHoverAssistInputs& in, float& tilt_left,
                                             float& tilt_right) {
    tilt_left = 0.0f;
    tilt_right = 0.0f;
    if (!(in.vectored_hover_gain > 0.0f)) {
        return;
    }
    const float throttle_scaler = hover_throttle_scaler(
        in.hover_throttle, in.output_throttle, in.throttle_scale_max, in.gain_scaling_min);
    tilt_left = in.tilt_left_in * in.vectored_hover_gain * throttle_scaler;
    tilt_right = in.tilt_right_in * in.vectored_hover_gain * throttle_scaler;
}

[[nodiscard]] inline float extra_hover_elevator(float des_pitch_cd, float pitch_sensor_cd,
                                                float vectored_hover_power, bool in_vtol_mode) {
    const auto pitch_error_cd =
        static_cast<std::int32_t>((des_pitch_cd - pitch_sensor_cd) * 0.5f);
    const float extra_pitch =
        math::constrain_value(static_cast<float>(pitch_error_cd), -kTailsitterServoMax,
                              kTailsitterServoMax) /
        kTailsitterServoMax;
    if (math::is_zero(extra_pitch) || !in_vtol_mode) {
        return 0.0f;
    }
    const float extra_sign = extra_pitch > 0.0f ? 1.0f : -1.0f;
    return extra_sign * std::pow(std::fabs(extra_pitch), vectored_hover_power) * kTailsitterServoMax;
}

inline void apply_vectored_hover_vtol_tilt(const VectoredHoverVtolInputs& in, float& tilt_left,
                                           float& tilt_right) {
    tilt_left = 0.0f;
    tilt_right = 0.0f;
    if (!(in.vectored_hover_gain > 0.0f)) {
        return;
    }
    const float extra_elevator = extra_hover_elevator(
        in.des_pitch_cd, in.pitch_sensor_cd, in.vectored_hover_power, in.in_vtol_mode);
    tilt_left = extra_elevator + in.tilt_left_in * in.vectored_hover_gain;
    tilt_right = extra_elevator + in.tilt_right_in * in.vectored_hover_gain;
}

[[nodiscard]] inline CopterSurfaceMapOutputs map_copter_surfaces_to_plane(
    const CopterSurfaceMapInputs& in) {
    CopterSurfaceMapOutputs out{};
    out.aileron_scaled =
        (in.motor_yaw + in.motor_yaw_ff) * -kTailsitterServoMax * in.vtol_yaw_scale;
    out.elevator_scaled =
        (in.motor_pitch + in.motor_pitch_ff) * kTailsitterServoMax * in.vtol_pitch_scale;
    out.rudder_scaled =
        (in.motor_roll + in.motor_roll_ff) * kTailsitterServoMax * in.vtol_roll_scale;
    return out;
}

[[nodiscard]] inline MixingOutputs mix_elevons_vtail(const MixingInputs& in,
                                                     const SurfaceAssign& surfaces) {
    MixingOutputs out{};
    const float elevator_mix =
        in.elevator_scaled * (100.0f - in.mixing_offset) * 0.01f * in.mixing_gain;
    float aileron_mix = in.aileron_scaled * (100.0f + in.mixing_offset) * 0.01f * in.mixing_gain;
    float rudder_mix = in.rudder_scaled * (100.0f + in.mixing_offset) * 0.01f * in.mixing_gain;

    const float headroom = kTailsitterServoMax - std::fabs(elevator_mix);
    if (math::is_positive(headroom)) {
        if (std::fabs(aileron_mix) > headroom) {
            aileron_mix *= headroom / std::fabs(aileron_mix);
            out.yaw_lim |= surfaces.elevon;
        }
        if (std::fabs(rudder_mix) > headroom) {
            rudder_mix *= headroom / std::fabs(rudder_mix);
            out.roll_lim |= surfaces.v_tail;
        }
    } else {
        aileron_mix = 0.0f;
        rudder_mix = 0.0f;
        out.yaw_lim |= surfaces.elevon;
        out.pitch_lim |= surfaces.elevon || surfaces.v_tail;
        out.roll_lim |= surfaces.v_tail;
    }

    out.elevon_left = elevator_mix - aileron_mix;
    out.elevon_right = elevator_mix + aileron_mix;
    out.vtail_right = elevator_mix - rudder_mix;
    out.vtail_left = elevator_mix + rudder_mix;
    return out;
}

[[nodiscard]] inline MotorLimitFlags surface_saturation_limits(const SurfaceAssign& surfaces,
                                                              float tilt_left, float tilt_right,
                                                              float rudder_scaled,
                                                              float elevator_scaled,
                                                              float aileron_scaled,
                                                              bool is_vectored) {
    MotorLimitFlags lim{};
    const bool tilt_lim =
        is_vectored && ((std::fabs(tilt_left) >= kTailsitterServoMax) ||
                        (std::fabs(tilt_right) >= kTailsitterServoMax));
    const bool roll_lim = surfaces.rudder && (std::fabs(rudder_scaled) >= kTailsitterServoMax);
    const bool pitch_lim = surfaces.elevator && (std::fabs(elevator_scaled) >= kTailsitterServoMax);
    const bool yaw_lim = surfaces.aileron && (std::fabs(aileron_scaled) >= kTailsitterServoMax);
    if (roll_lim) {
        lim.roll = true;
    }
    if (pitch_lim || tilt_lim) {
        lim.pitch = true;
    }
    if (yaw_lim || tilt_lim) {
        lim.yaw = true;
    }
    return lim;
}

[[nodiscard]] inline QAssistRelaxOutputs q_assist_motors_only_relax(const SurfaceAssign& surfaces) {
    QAssistRelaxOutputs out{};
    if (surfaces.elevator || surfaces.elevon || surfaces.v_tail) {
        out.relax_pitch_integrator = true;
        out.limit_pitch = true;
    } else {
        out.reset_plane_pitch_i = true;
    }
    if (surfaces.rudder || surfaces.v_tail) {
        out.relax_roll_integrator = true;
        out.limit_roll = true;
    } else {
        out.reset_plane_yaw_i = true;
    }
    return out;
}

[[nodiscard]] inline std::uint16_t throttle_pwm_from_actuator(float actuator, std::uint16_t pwm_min,
                                                              std::uint16_t pwm_max) {
    return static_cast<std::uint16_t>(
        static_cast<float>(pwm_min) + static_cast<float>(pwm_max - pwm_min) * actuator);
}

[[nodiscard]] inline TailsitterOutputResult tailsitter_output(const TailsitterOutputInputs& in) {
    TailsitterOutputResult out{};
    if (!output_should_run(in.skip)) {
        return out;
    }
    out.ran = true;
    out.output_motor_min = output_requires_motor_min(in.disarm);

    float throttle = in.fw_throttle;

    if (output_uses_fw_or_vtol_trans_path(in.active, in.in_vtol_transition)) {
        VtolTransitionThrottleInputs thr_in = in.vtol_throttle;
        thr_in.armed_and_safety_off = in.armed_and_safety_off;
        thr_in.in_vtol_transition = in.in_vtol_transition;
        thr_in.throttle_wait = in.throttle_wait;
        const auto vtol_thr = compute_vtol_transition_throttle(thr_in);
        if (vtol_thr.apply) {
            out.center_rudder = true;
            out.zero_rudder_dt = true;
            throttle = vtol_thr.throttle_thrust;
            if (!in.assisted_flight) {
                out.set_attitude_throttle = true;
                out.attitude_throttle = vtol_thr.throttle_thrust;
                throttle = in.selected_thrust_as_actuator;
                out.set_throttle_pwm = true;
                out.throttle_pwm = throttle_pwm_from_actuator(throttle, in.pwm_min, in.pwm_max);
                out.throttle_scaled = throttle * 100.0f;
            }
        }
        if (!in.assisted_flight) {
            out.path = OutputPath::kForwardFlight;
            out.throttle = throttle;
            out.apply_fw_motor_mask = true;
            ForwardMotorMaskInputs mask_in = in.motor_mask;
            mask_in.throttle = throttle;
            if (out.zero_rudder_dt) {
                mask_in.rudder_dt = 0.0f;
            }
            out.motor_mask = compute_forward_motor_mask(mask_in);
            apply_vectored_forward_tilt(in.forward_tilt, out.tilt_left, out.tilt_right);
            return out;
        }
    }

    out.throttle = throttle;

    if (output_runs_assisted_copter(in.assisted_flight, in.transition_state)) {
        out.hold_stabilize = true;
        out.motors_output = true;
        out.motors_output_assisted = true;
        if (in.q_assist_motors_only) {
            out.path = OutputPath::kQAssistMotorsOnly;
            out.assist_relax = q_assist_motors_only_relax(in.surfaces);
            out.limits.roll = out.assist_relax.limit_roll;
            out.limits.pitch = out.assist_relax.limit_pitch;
            out.limits.yaw = out.assist_relax.limit_yaw;
            apply_vectored_hover_assist_tilt(in.hover_assist, out.tilt_left, out.tilt_right);
            return out;
        }
    } else {
        out.motors_output = true;
        out.motors_output_assisted = false;
    }

    out.path = OutputPath::kVtolCopter;
    out.reset_plane_i = true;
    out.copter_surfaces = map_copter_surfaces_to_plane(in.copter);

    if (in.armed_and_safety_off) {
        out.apply_speed_scaling = true;
    } else if (in.have_tailsitter_motors) {
        out.set_min_throttle_zero = true;
    }

    apply_vectored_hover_vtol_tilt(in.hover_vtol, out.tilt_left, out.tilt_right);

    MixingInputs mix_in = in.mixing;
    mix_in.elevator_scaled = out.copter_surfaces.elevator_scaled;
    mix_in.aileron_scaled = out.copter_surfaces.aileron_scaled;
    mix_in.rudder_scaled = out.copter_surfaces.rudder_scaled;
    out.mix = mix_elevons_vtail(mix_in, in.surfaces);

    const auto sat = surface_saturation_limits(
        in.surfaces, out.tilt_left, out.tilt_right, out.copter_surfaces.rudder_scaled,
        out.copter_surfaces.elevator_scaled, out.copter_surfaces.aileron_scaled, in.is_vectored);
    out.limits.roll = sat.roll || out.mix.roll_lim;
    out.limits.pitch = sat.pitch || out.mix.pitch_lim;
    out.limits.yaw = sat.yaw || out.mix.yaw_lim;
    return out;
}

}  // namespace fwcpp::tailsitter
