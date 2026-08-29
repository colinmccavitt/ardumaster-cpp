#pragma once

// QuadPlane hover / pilot-input helpers — Plane-4.7.0 ArduPlane/quadplane.cpp:
// hold_hover (1101-1117), get_pilot_throttle (1119-1137),
// get_pilot_desired_lean_angles (1143-1174), get_pilot_land_throttle (1179-1192),
// get_pilot_input_yaw_rate_cds (1321-1358), get_desired_yaw_rate_cds (1364-1380),
// get_pilot_desired_climb_rate_cms (1384-1395), get_pilot_velocity_z_max_dn_m (4308-4314).
//
// ADR-0012: decoded RC numbers and mode flags are injected; no Plane /
// SRV_Channels / RC_Channel objects. throttle_curve / input_expo are local
// AP_Math matches (those helpers are not in ap-math yet). weathervane yaw
// is injected, not reimplemented. hold_hover is tick/effects only.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/tailsitter/tailsitter_defaults.hpp>

namespace fwcpp::quadplane {

inline constexpr float kPilotSpeedZMaxUpMsDefault = 2.50f;
inline constexpr float kPilotSpeedZMaxDnMsDefault = 0.0f;
inline constexpr float kPilotAccelZMssDefault = 2.5f;
inline constexpr float kThrottleExpoDefault = 0.2f;
inline constexpr float kCommandModelPilotRateDefault = 100.0f;
inline constexpr float kCommandModelPilotExpoDefault = 0.25f;
inline constexpr float kRuddDtGainDefault = 10.0f;

enum class StickMixing : std::uint8_t {
    kNone = 0,
    kFbw = 1,
    kDirectRemoved = 2,
    kVtolYaw = 3,
    kFbwNoPitch = 4,
};

enum class RudderArming : std::uint8_t {
    kIsDisabled = 0,
    kArmOnly = 1,
    kArmDisarm = 2,
};

/// AP_Math.cpp expo_curve — also in plane.hpp; duplicated here to avoid a vehicle include.
[[nodiscard]] inline float expo_curve(float alpha, float x) {
    return (1.0f - alpha) * x + alpha * x * x * x;
}

/// AP_Math.cpp:134 throttle_curve.
[[nodiscard]] inline float throttle_curve(float thr_mid, float alpha, float thr_in) {
    float alpha2 = alpha + 1.25f * (1.0f - alpha) * (0.5f - thr_mid) / 0.5f;
    alpha2 = fwcpp::math::constrain_value(alpha2, 0.0f, 1.0f);
    float thr_out = 0.0f;
    if (thr_in < 0.5f) {
        const float t = fwcpp::math::linear_interpolate(-1.0f, 0.0f, thr_in, 0.0f, 0.5f);
        thr_out = fwcpp::math::linear_interpolate(0.0f, thr_mid, expo_curve(alpha, t), -1.0f, 0.0f);
    } else {
        const float t = fwcpp::math::linear_interpolate(0.0f, 1.0f, thr_in, 0.5f, 1.0f);
        thr_out = fwcpp::math::linear_interpolate(thr_mid, 1.0f, expo_curve(alpha2, t), 0.0f, 1.0f);
    }
    return thr_out;
}

/// AP_Math/control.cpp:760 input_expo.
[[nodiscard]] inline float input_expo(float input, float expo) {
    input = fwcpp::math::constrain_value(input, -1.0f, 1.0f);
    if (expo < 0.95f) {
        return (1.0f - expo) * input / (1.0f - expo * std::fabs(input));
    }
    return input;
}

/// Upstream comment says cm/s; the body returns abs(m/s param) as uint16_t.
[[nodiscard]] inline std::uint16_t get_pilot_velocity_z_max_dn_m(float pilot_speed_z_max_dn_ms,
                                                                float pilot_speed_z_max_up_ms) {
    if (fwcpp::math::is_zero(pilot_speed_z_max_dn_ms)) {
        return static_cast<std::uint16_t>(std::abs(pilot_speed_z_max_up_ms));
    }
    return static_cast<std::uint16_t>(std::abs(pilot_speed_z_max_dn_ms));
}

struct PilotThrottleInputs {
    float control_in{0.0f};
    float range{100.0f};
    float throttle_expo{kThrottleExpoDefault};
    float throttle_hover{0.5f};
};

[[nodiscard]] inline float get_pilot_throttle(const PilotThrottleInputs& in) {
    float throttle_in = in.control_in / in.range;
    if (fwcpp::math::is_positive(in.throttle_expo)) {
        const float thr_mid = in.throttle_hover;
        const float thrust_curve_expo = fwcpp::math::constrain_value(in.throttle_expo, 0.0f, 1.0f);
        return throttle_curve(thr_mid, thrust_curve_expo, throttle_in);
    }
    return throttle_in;
}

struct PilotLeanAngleInputs {
    bool rc_failsafe{false};
    bool throttle_counter_active{false};
    float roll_control_in{0.0f};
    float pitch_control_in{0.0f};
    float angle_max_cd{4500.0f};
    float angle_limit_cd{4500.0f};
};

struct PilotLeanAngles {
    float roll_out_cd{0.0f};
    float pitch_out_cd{0.0f};
};

[[nodiscard]] inline PilotLeanAngles get_pilot_desired_lean_angles(const PilotLeanAngleInputs& in) {
    PilotLeanAngles out{};
    if (in.rc_failsafe || in.throttle_counter_active) {
        return out;
    }

    float roll_out_cd = in.roll_control_in;
    float pitch_out_cd = in.pitch_control_in;
    const float angle_limit_cd = fwcpp::math::constrain_value(in.angle_limit_cd, 1000.0f, in.angle_max_cd);

    const float scaler = in.angle_max_cd / 4500.0f;
    roll_out_cd *= scaler;
    pitch_out_cd *= scaler;

    const float total_in = std::sqrt(pitch_out_cd * pitch_out_cd + roll_out_cd * roll_out_cd);
    if (total_in > angle_limit_cd) {
        const float ratio = angle_limit_cd / total_in;
        roll_out_cd *= ratio;
        pitch_out_cd *= ratio;
    }

    roll_out_cd = 100.0f * fwcpp::math::degrees(
        std::atan(std::cos(fwcpp::math::cd_to_rad(pitch_out_cd)) * std::tan(fwcpp::math::cd_to_rad(roll_out_cd))));
    out.roll_out_cd = roll_out_cd;
    out.pitch_out_cd = pitch_out_cd;
    return out;
}

struct PilotLandThrottleInputs {
    bool rc_failsafe_active{false};
    float control_in{0.0f};
    float range{100.0f};
};

[[nodiscard]] inline float get_pilot_land_throttle(const PilotLandThrottleInputs& in) {
    if (in.rc_failsafe_active) {
        return 0.0f;
    }
    const float throttle_in = in.control_in / in.range;
    return fwcpp::math::constrain_value(throttle_in, 0.0f, 1.0f);
}

struct PilotYawRateInputs {
    float rudder_in{0.0f};
    bool is_vtol_man_throttle{false};
    bool air_mode_active{false};
    float throttle_input{0.0f};
    bool does_auto_throttle{false};
    bool motors_throttle_lower_limit{false};
    RudderArming rudder_arming{RudderArming::kArmDisarm};
    float velocity_z_up_cms{0.0f};
    float pilot_speed_z_max_dn_ms{kPilotSpeedZMaxDnMsDefault};
    float pilot_speed_z_max_up_ms{kPilotSpeedZMaxUpMsDefault};
    StickMixing stick_mixing{StickMixing::kFbw};
    bool mode_is_qrtl{false};
    bool mode_is_guided{false};
    bool in_vtol_auto{false};
    float command_model_rate{kCommandModelPilotRateDefault};
    float command_model_expo{kCommandModelPilotExpoDefault};
    bool in_vtol_mode{true};
    bool tailsitter_enabled{false};
    std::int8_t tailsitter_input_type{0};
    float rudd_dt_gain{kRuddDtGainDefault};
};

[[nodiscard]] inline float get_pilot_input_yaw_rate_cds(const PilotYawRateInputs& in) {
    const bool manual_air_mode = in.is_vtol_man_throttle && in.air_mode_active;
    if (!manual_air_mode && !fwcpp::math::is_positive(in.throttle_input) &&
        (!in.does_auto_throttle || in.motors_throttle_lower_limit) &&
        in.rudder_arming == RudderArming::kArmDisarm && in.rudder_in < 0.0f &&
        std::fabs(in.velocity_z_up_cms) <
            (0.5f * static_cast<float>(get_pilot_velocity_z_max_dn_m(in.pilot_speed_z_max_dn_ms,
                                                                     in.pilot_speed_z_max_up_ms))) *
                100.0f) {
        return 0.0f;
    }

    if ((in.stick_mixing == StickMixing::kNone) &&
        (in.mode_is_qrtl || in.mode_is_guided || in.in_vtol_auto)) {
        return 0.0f;
    }

    const float yaw_rate_max = in.command_model_rate;
    float max_rate = yaw_rate_max;
    if (!in.in_vtol_mode && in.tailsitter_enabled) {
        max_rate *= in.rudd_dt_gain * 0.01f;
    }
    if (in.tailsitter_enabled &&
        (static_cast<std::uint8_t>(in.tailsitter_input_type) & fwcpp::tailsitter::kTailsitterInputBfRoll) != 0u) {
        max_rate = (yaw_rate_max < 1.0f) ? 1.0f : yaw_rate_max;
    }
    return input_expo(in.rudder_in * (1.0f / 4500.0f), in.command_model_expo) * max_rate * 100.0f;
}

struct DesiredYawRateInputs {
    bool assisted_flight{false};
    float desired_auto_yaw_rate_cds{0.0f};
    PilotYawRateInputs pilot{};
    bool should_weathervane{false};
    float weathervane_yaw_rate_cds{0.0f};
};

[[nodiscard]] inline float get_desired_yaw_rate_cds(const DesiredYawRateInputs& in) {
    float yaw_cds = 0.0f;
    if (in.assisted_flight) {
        yaw_cds += in.desired_auto_yaw_rate_cds;
    }
    yaw_cds += get_pilot_input_yaw_rate_cds(in.pilot);
    if (in.should_weathervane) {
        yaw_cds += in.weathervane_yaw_rate_cds;
    }
    return yaw_cds;
}

struct PilotClimbRateInputs {
    bool has_valid_input{true};
    float throttle_request{0.0f};
    float pilot_speed_z_max_up_ms{kPilotSpeedZMaxUpMsDefault};
    float pilot_speed_z_max_dn_ms{kPilotSpeedZMaxDnMsDefault};
};

[[nodiscard]] inline float get_pilot_desired_climb_rate_cms(const PilotClimbRateInputs& in) {
    if (!in.has_valid_input) {
        return -50.0f;
    }
    return in.throttle_request *
           (in.throttle_request > 0.0f
                ? in.pilot_speed_z_max_up_ms
                : static_cast<float>(get_pilot_velocity_z_max_dn_m(in.pilot_speed_z_max_dn_ms,
                                                                   in.pilot_speed_z_max_up_ms))) *
           100.0f;
}

struct HoldHoverInputs {
    float target_climb_rate_cms{0.0f};
    float pilot_speed_z_max_up_ms{kPilotSpeedZMaxUpMsDefault};
    float pilot_speed_z_max_dn_ms{kPilotSpeedZMaxDnMsDefault};
    float pilot_accel_z_mss{kPilotAccelZMssDefault};
    DesiredYawRateInputs yaw{};
};

struct HoldHoverTick {
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    float d_max_speed_dn_m{0.0f};
    float d_max_speed_up_ms{0.0f};
    float d_max_accel_z_mss{0.0f};
    float desired_yaw_rate_cds{0.0f};
    bool multicopter_attitude_rate_update{false};
    float climb_rate_ms{0.0f};
    bool run_z_controller{false};
};

[[nodiscard]] inline HoldHoverTick hold_hover(const HoldHoverInputs& in) {
    HoldHoverTick tick{};
    tick.desired_spool = DesiredSpoolState::kThrottleUnlimited;
    tick.d_max_speed_dn_m = static_cast<float>(
        get_pilot_velocity_z_max_dn_m(in.pilot_speed_z_max_dn_ms, in.pilot_speed_z_max_up_ms));
    tick.d_max_speed_up_ms = in.pilot_speed_z_max_up_ms;
    tick.d_max_accel_z_mss = in.pilot_accel_z_mss;

    DesiredYawRateInputs yaw = in.yaw;
    yaw.should_weathervane = false;
    tick.desired_yaw_rate_cds = get_desired_yaw_rate_cds(yaw);
    tick.multicopter_attitude_rate_update = true;

    tick.climb_rate_ms = in.target_climb_rate_cms * 0.01f;
    tick.run_z_controller = true;
    return tick;
}

}  // namespace fwcpp::quadplane
