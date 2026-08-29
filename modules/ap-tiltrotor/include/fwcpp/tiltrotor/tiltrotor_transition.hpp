#pragma once

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/quadplane_transition/transition_state.hpp>
#include <fwcpp/tiltrotor/tiltrotor_defaults.hpp>

namespace fwcpp::tiltrotor {

[[nodiscard]] inline float fixedwing_turn_rate_deg_s(float bank_angle_deg, float airspeed_ms) {
    bank_angle_deg = fwcpp::math::constrain_value(bank_angle_deg, -80.0f, 80.0f);
    const float spd = std::max(airspeed_ms, 1.0f);
    return fwcpp::math::degrees(kGravityMss * std::tan(fwcpp::math::radians(bank_angle_deg)) / spd);
}

struct YawTargetState {
    float transition_yaw_cd{0.0f};
    std::uint32_t transition_yaw_set_ms{0};
};

struct YawTargetSample {
    std::uint32_t now_ms{0};
    float pilot_yaw_rate_cds{0.0f};
    float ahrs_yaw_sensor_cd{0.0f};
    bool have_airspeed{false};
    float airspeed_eas_ms{0.0f};
    std::int32_t nav_roll_cd{0};
    float airspeed_min{10.0f};
};

[[nodiscard]] inline YawTargetState update_yaw_target(YawTargetState state, const YawTargetSample& sample) {
    if (sample.now_ms - state.transition_yaw_set_ms > kTransitionYawLockMs ||
        !fwcpp::math::is_zero(sample.pilot_yaw_rate_cds)) {
        state.transition_yaw_cd = sample.ahrs_yaw_sensor_cd;
    }
    if (sample.have_airspeed && std::abs(sample.nav_roll_cd) > kNavRollTransitionThresholdCd) {
        const float dt = static_cast<float>(sample.now_ms - state.transition_yaw_set_ms) * 0.001f;
        const float airspeed_min = std::max(sample.airspeed_min, kAirspeedMinTransitionMs);
        const float yaw_rate_cds =
            fixedwing_turn_rate_deg_s(sample.nav_roll_cd * 0.01f, std::max(sample.airspeed_eas_ms, airspeed_min)) *
            100.0f;
        state.transition_yaw_cd += yaw_rate_cds * dt;
    }
    state.transition_yaw_set_ms = sample.now_ms;
    return state;
}

struct MotorThrustSample {
    bool thrust_boost{false};
    float roll_factor{0.0f};
};

struct TiltrotorTransitionView {
    bool is_vectored{false};
    fwcpp::quadplane_transition::TransitionState transition_state{
        fwcpp::quadplane_transition::TransitionState::kDone};
};

[[nodiscard]] inline bool use_multirotor_control_in_fwd_transition(const TiltrotorTransitionView& view) {
    if (!view.is_vectored) {
        return false;
    }
    using fwcpp::quadplane_transition::TransitionState;
    switch (view.transition_state) {
        case TransitionState::kAirspeedWait:
        case TransitionState::kTimer:
            return true;
        case TransitionState::kDone:
            return false;
    }
    return false;
}

[[nodiscard]] inline bool show_vtol_view(bool in_vtol_mode, const TiltrotorTransitionView& view) {
    if (in_vtol_mode) {
        return true;
    }
    if (view.is_vectored && fwcpp::quadplane_transition::in_forward_transition(view.transition_state)) {
        return true;
    }
    return false;
}

[[nodiscard]] inline bool transition_update_yaw_target(const TiltrotorTransitionView& view, YawTargetState& state,
                                                       const YawTargetSample& sample, float& yaw_target_cd) {
    if (!use_multirotor_control_in_fwd_transition(view)) {
        return false;
    }
    state = update_yaw_target(state, sample);
    yaw_target_cd = state.transition_yaw_cd;
    return true;
}

[[nodiscard]] inline bool allow_vfwd(bool is_vectored, const MotorThrustSample& motors,
                                     bool lost_motor_is_tilting) {
    if (!is_vectored) {
        return true;
    }
    if (!motors.thrust_boost) {
        return true;
    }
    if (!lost_motor_is_tilting) {
        return true;
    }
    if (fwcpp::math::is_zero(motors.roll_factor)) {
        return true;
    }
    return false;
}

}  // namespace fwcpp::tiltrotor
