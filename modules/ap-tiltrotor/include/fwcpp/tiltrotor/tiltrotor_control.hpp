#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/quadplane_transition/transition_state.hpp>
#include <fwcpp/tiltrotor/tiltrotor_defaults.hpp>
#include <fwcpp/tiltrotor/tiltrotor_enable.hpp>
#include <fwcpp/tiltrotor/tiltrotor_predicates.hpp>
#include <fwcpp/tiltrotor/tiltrotor_types.hpp>

namespace fwcpp::tiltrotor {

struct TiltRateParams {
    float max_rate_up_dps{kTiltRateUpDefaultDps};
    float max_rate_down_dps{kTiltRateDownDefaultDps};
    float g_dt{0.02f};
};

[[nodiscard]] inline float tilt_max_change(const TiltRateParams& rate, TiltType type, bool up,
                                           bool in_flap_range, bool manual_mode, bool armed_and_safety_off,
                                           bool in_vtol_mode, bool assisted_flight) {
    float rate_dps = (up || rate.max_rate_down_dps <= 0.0f) ? rate.max_rate_up_dps : rate.max_rate_down_dps;
    if (type != TiltType::kBinary && !up && !in_flap_range) {
        bool fast_tilt = manual_mode;
        if (armed_and_safety_off && !in_vtol_mode && !assisted_flight) {
            fast_tilt = true;
        }
        if (fast_tilt) {
            rate_dps = std::max(rate_dps, kFastTiltMinRateDps);
        }
    }
    return rate_dps * rate.g_dt * kDegreesPerTiltUnit;
}

struct SlewResult {
    float current_tilt{0.0f};
    bool angle_achieved{true};
    float servo_motor_tilt{0.0f};
};

[[nodiscard]] inline SlewResult apply_slew(float current_tilt, float newtilt, float max_change) {
    SlewResult out{};
    out.current_tilt =
        fwcpp::math::constrain_value(newtilt, current_tilt - max_change, current_tilt + max_change);
    out.angle_achieved = fwcpp::math::is_equal(newtilt, out.current_tilt);
    out.servo_motor_tilt = kServoMotorTiltScale * out.current_tilt;
    return out;
}

struct SlewFlightFlags {
    bool manual_mode{false};
    bool armed_and_safety_off{true};
    bool in_vtol_mode{true};
    bool assisted_flight{false};
};

// Upstream Tiltrotor::slew: max_change = tilt_max_change(newtilt<current_tilt,
// newtilt > get_fully_forward_tilt()).
[[nodiscard]] inline SlewResult slew(float current_tilt, float newtilt, const TiltRateParams& rate,
                                     TiltType type, float flap_angle_deg, const SlewFlightFlags& flags) {
    const bool up = newtilt < current_tilt;
    const bool in_flap_range = newtilt > get_fully_forward_tilt(flap_angle_deg);
    const float max_change = tilt_max_change(rate, type, up, in_flap_range, flags.manual_mode,
                                             flags.armed_and_safety_off, flags.in_vtol_mode,
                                             flags.assisted_flight);
    return apply_slew(current_tilt, newtilt, max_change);
}

[[nodiscard]] inline float get_forward_flight_tilt(float flap_angle_deg, float flap_auto_slew_pct) {
    const float flap_fraction = flap_angle_deg * kDegreesPerTiltUnit;
    return 1.0f - (flap_fraction * flap_auto_slew_pct * 0.01f);
}

[[nodiscard]] inline bool fully_fwd(const TiltrotorGate& gate, std::uint16_t tilt_mask, float current_tilt,
                                    float flap_angle_deg) {
    if (!gate.enabled() || tilt_mask == 0u) {
        return false;
    }
    return current_tilt >= get_fully_forward_tilt(flap_angle_deg);
}

[[nodiscard]] inline bool fully_up(const TiltrotorGate& gate, std::uint16_t tilt_mask, float current_tilt) {
    if (!gate.enabled() || tilt_mask == 0u) {
        return false;
    }
    return current_tilt <= 0.0f;
}

[[nodiscard]] inline bool tilt_over_max_angle(float current_tilt, float max_angle_deg, float flap_angle_deg,
                                              float flap_auto_slew_pct) {
    const float tilt_threshold = max_angle_deg * kDegreesPerTiltUnit;
    return current_tilt > std::min(tilt_threshold, get_forward_flight_tilt(flap_angle_deg, flap_auto_slew_pct));
}

enum class ContinuousTiltStrategy : std::uint8_t {
    kFixedWingPath,
    kQautotuneZero,
    kFwdThrGain,
    kManualRcModes,
    kTransitionAllForward,
    kAssistedThrottleMap,
};

struct ContinuousTiltInputs {
    bool in_vtol_mode{true};
    bool armed_and_safety_off{true};
    bool assisted_flight{false};
    bool disarmed_tilt_up_option{false};
    bool manual_mode{false};
    bool qautotune_mode{false};
    bool use_calculated_fwd_thr{false};
    bool flying_vtol{true};
    bool rc_fwd_thr_present{false};
    bool qacro_qstab_qhover_mode{false};
    bool transition_at_or_past_timer{false};
    bool motor_test_running{false};
    float forward_throttle_pct{0.0f};
    float max_angle_deg{static_cast<float>(kTiltMaxAngleDefaultDeg)};
    float flap_angle_deg{0.0f};
    float flap_auto_slew_pct{0.0f};
    float throttle_out_scaled{0.0f};
    float throttle_min{0.0f};
    float motors_throttle{0.0f};
};

[[nodiscard]] inline ContinuousTiltStrategy resolve_continuous_strategy(const ContinuousTiltInputs& in) {
    if (!in.in_vtol_mode && (!in.armed_and_safety_off || !in.assisted_flight)) {
        return ContinuousTiltStrategy::kFixedWingPath;
    }
    if (in.qautotune_mode) {
        return ContinuousTiltStrategy::kQautotuneZero;
    }
    if (!in.assisted_flight && in.use_calculated_fwd_thr && in.flying_vtol) {
        return ContinuousTiltStrategy::kFwdThrGain;
    }
    if (!in.assisted_flight && in.qacro_qstab_qhover_mode) {
        return ContinuousTiltStrategy::kManualRcModes;
    }
    if (in.assisted_flight && in.transition_at_or_past_timer) {
        return ContinuousTiltStrategy::kTransitionAllForward;
    }
    return ContinuousTiltStrategy::kAssistedThrottleMap;
}

[[nodiscard]] inline float continuous_target_tilt(ContinuousTiltStrategy strategy, const ContinuousTiltInputs& in) {
    const float forward_cap = get_forward_flight_tilt(in.flap_angle_deg, in.flap_auto_slew_pct);
    switch (strategy) {
        case ContinuousTiltStrategy::kFixedWingPath: {
            const bool disarmed_tilt_up =
                !in.armed_and_safety_off && !in.manual_mode && in.disarmed_tilt_up_option;
            return disarmed_tilt_up ? 0.0f : forward_cap;
        }
        case ContinuousTiltStrategy::kQautotuneZero:
            return 0.0f;
        case ContinuousTiltStrategy::kFwdThrGain: {
            const float fwd_g_demand = 0.01f * in.forward_throttle_pct;
            const float fwd_tilt_deg =
                std::min(fwcpp::math::degrees(std::atan(fwd_g_demand)), in.max_angle_deg);
            return std::min(fwd_tilt_deg * kDegreesPerTiltUnit, forward_cap);
        }
        case ContinuousTiltStrategy::kManualRcModes:
            if (!in.rc_fwd_thr_present) {
                return 0.0f;
            }
            return std::min(0.01f * in.forward_throttle_pct * in.max_angle_deg * kDegreesPerTiltUnit, forward_cap);
        case ContinuousTiltStrategy::kTransitionAllForward:
            return forward_cap;
        case ContinuousTiltStrategy::kAssistedThrottleMap: {
            const float throttle_floor = std::max(in.throttle_min, 0.0f);
            const float settilt = fwcpp::math::constrain_value((in.throttle_out_scaled - throttle_floor) * 0.02f,
                                                               0.0f, 1.0f);
            return std::min(settilt * in.max_angle_deg * kDegreesPerTiltUnit, forward_cap);
        }
    }
    return 0.0f;
}

[[nodiscard]] inline bool transition_at_or_past_timer(fwcpp::quadplane_transition::TransitionState state) {
    using fwcpp::quadplane_transition::TransitionState;
    return state == TransitionState::kTimer || state == TransitionState::kDone;
}

struct BinarySlewResult {
    float current_tilt{0.0f};
    float servo_motor_tilt{0.0f};
};

[[nodiscard]] inline BinarySlewResult apply_binary_slew(float current_tilt, bool forward, float max_change) {
    BinarySlewResult out{};
    out.servo_motor_tilt = forward ? kServoMotorTiltScale : 0.0f;
    if (forward) {
        out.current_tilt = fwcpp::math::constrain_value(current_tilt + max_change, 0.0f, 1.0f);
    } else {
        out.current_tilt = fwcpp::math::constrain_value(current_tilt - max_change, 0.0f, 1.0f);
    }
    return out;
}

[[nodiscard]] inline BinarySlewResult binary_slew(float current_tilt, bool forward, const TiltRateParams& rate,
                                                  TiltType type, const SlewFlightFlags& flags) {
    const float max_change = tilt_max_change(rate, type, !forward, false, flags.manual_mode,
                                             flags.armed_and_safety_off, flags.in_vtol_mode,
                                             flags.assisted_flight);
    return apply_binary_slew(current_tilt, forward, max_change);
}

enum class TiltUpdatePath : std::uint8_t {
    kNone,
    kBinary,
    kContinuous,
    kContinuousThenVectoring,
};

[[nodiscard]] inline TiltUpdatePath resolve_update_path(const TiltrotorGate& gate, std::uint16_t tilt_mask,
                                                        TiltType type) {
    if (!gate.enabled() || tilt_mask == 0u) {
        return TiltUpdatePath::kNone;
    }
    if (type == TiltType::kBinary) {
        return TiltUpdatePath::kBinary;
    }
    if (type == TiltType::kVectoredYaw) {
        return TiltUpdatePath::kContinuousThenVectoring;
    }
    return TiltUpdatePath::kContinuous;
}

struct TiltControlState {
    float current_tilt{0.0f};
    float current_throttle{0.0f};
    bool motors_active{false};
    bool angle_achieved{true};
};

struct TiltMotorMaskCommand {
    bool apply{false};
    float throttle{0.0f};
    std::uint32_t mask{0};
};

struct TiltUpdateResult {
    TiltControlState state{};
    float servo_motor_tilt{0.0f};
    TiltMotorMaskCommand motor_mask{};
    TiltUpdatePath path{TiltUpdatePath::kNone};
    bool ran_vectoring{false};
};

[[nodiscard]] inline SlewFlightFlags flight_flags_from(const ContinuousTiltInputs& in) {
    return SlewFlightFlags{
        .manual_mode = in.manual_mode,
        .armed_and_safety_off = in.armed_and_safety_off,
        .in_vtol_mode = in.in_vtol_mode,
        .assisted_flight = in.assisted_flight,
    };
}

[[nodiscard]] inline TiltMotorMaskCommand motor_mask_command(float throttle, std::uint16_t tilt_mask) {
    TiltMotorMaskCommand cmd{};
    cmd.apply = true;
    cmd.throttle = throttle;
    cmd.mask = fwcpp::math::is_zero(throttle) ? 0u : static_cast<std::uint32_t>(tilt_mask);
    return cmd;
}

[[nodiscard]] inline TiltUpdateResult continuous_update(TiltControlState state, const ContinuousTiltInputs& in,
                                                        const TiltRateParams& rate, TiltType type,
                                                        std::uint16_t tilt_mask) {
    TiltUpdateResult out{};
    state.motors_active = false;
    const auto flags = flight_flags_from(in);
    const auto strategy = resolve_continuous_strategy(in);

    if (strategy == ContinuousTiltStrategy::kFixedWingPath) {
        const auto slewed = slew(state.current_tilt, continuous_target_tilt(strategy, in), rate, type,
                                 in.flap_angle_deg, flags);
        state.current_tilt = slewed.current_tilt;
        state.angle_achieved = slewed.angle_achieved;
        const float max_change = tilt_max_change(rate, type, false, false, flags.manual_mode,
                                                 flags.armed_and_safety_off, flags.in_vtol_mode,
                                                 flags.assisted_flight);
        const float new_throttle =
            fwcpp::math::constrain_value(in.throttle_out_scaled * kThrottleScaledToUnit, 0.0f, 1.0f);
        if (state.current_tilt < get_fully_forward_tilt(in.flap_angle_deg)) {
            state.current_throttle = fwcpp::math::constrain_value(
                new_throttle, state.current_throttle - max_change, state.current_throttle + max_change);
        } else {
            state.current_throttle = new_throttle;
        }
        if (!in.armed_and_safety_off) {
            state.current_throttle = 0.0f;
        } else {
            state.motors_active = true;
        }
        out.state = state;
        out.servo_motor_tilt = slewed.servo_motor_tilt;
        if (!in.motor_test_running) {
            out.motor_mask = motor_mask_command(state.current_throttle, tilt_mask);
        }
        out.path = TiltUpdatePath::kContinuous;
        return out;
    }

    const float throttle_max_change =
        tilt_max_change(rate, type, in.motors_throttle < state.current_throttle, false, flags.manual_mode,
                        flags.armed_and_safety_off, flags.in_vtol_mode, flags.assisted_flight);
    state.current_throttle = fwcpp::math::constrain_value(in.motors_throttle,
                                                          state.current_throttle - throttle_max_change,
                                                          state.current_throttle + throttle_max_change);

    const auto slewed = slew(state.current_tilt, continuous_target_tilt(strategy, in), rate, type,
                             in.flap_angle_deg, flags);
    state.current_tilt = slewed.current_tilt;
    state.angle_achieved = slewed.angle_achieved;
    out.state = state;
    out.servo_motor_tilt = slewed.servo_motor_tilt;
    out.path = TiltUpdatePath::kContinuous;
    return out;
}

[[nodiscard]] inline TiltUpdateResult binary_update(TiltControlState state, const ContinuousTiltInputs& in,
                                                    const TiltRateParams& rate, std::uint16_t tilt_mask) {
    TiltUpdateResult out{};
    state.motors_active = true;
    const auto flags = flight_flags_from(in);
    if (!in.in_vtol_mode) {
        const auto slewed = binary_slew(state.current_tilt, true, rate, TiltType::kBinary, flags);
        state.current_tilt = slewed.current_tilt;
        out.servo_motor_tilt = slewed.servo_motor_tilt;
        const float new_throttle = in.throttle_out_scaled * kThrottleScaledToUnit;
        if (state.current_tilt >= 1.0f) {
            out.motor_mask = motor_mask_command(new_throttle, tilt_mask);
        }
    } else {
        const auto slewed = binary_slew(state.current_tilt, false, rate, TiltType::kBinary, flags);
        state.current_tilt = slewed.current_tilt;
        out.servo_motor_tilt = slewed.servo_motor_tilt;
    }
    out.state = state;
    out.path = TiltUpdatePath::kBinary;
    return out;
}

[[nodiscard]] inline TiltUpdateResult update(const TiltrotorGate& gate, std::uint16_t tilt_mask, TiltType type,
                                             TiltControlState state, const ContinuousTiltInputs& in,
                                             const TiltRateParams& rate) {
    TiltUpdateResult out{};
    out.path = resolve_update_path(gate, tilt_mask, type);
    if (out.path == TiltUpdatePath::kNone) {
        out.state = state;
        return out;
    }
    if (out.path == TiltUpdatePath::kBinary) {
        out = binary_update(state, in, rate, tilt_mask);
    } else {
        out = continuous_update(state, in, rate, type, tilt_mask);
        out.path = resolve_update_path(gate, tilt_mask, type);
        out.ran_vectoring = (out.path == TiltUpdatePath::kContinuousThenVectoring);
    }
    return out;
}

}  // namespace fwcpp::tiltrotor
