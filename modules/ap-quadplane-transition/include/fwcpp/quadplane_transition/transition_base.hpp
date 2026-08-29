#pragma once

#include <fwcpp/quadplane_transition/transition_fsm.hpp>
#include <fwcpp/quadplane_transition/transition_state.hpp>
#include <fwcpp/quadplane_transition/transition_timing.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fwcpp::quadplane_transition {

struct TransitionBaseDefaults {
    static constexpr bool allow_update_throttle_mix() { return true; }
    static constexpr bool allow_weathervane() { return true; }
    static constexpr bool allow_vfwd() { return true; }
    static constexpr bool allow_stick_mixing() { return true; }
    static constexpr bool use_multirotor_control_in_fwd_transition() { return false; }
    static constexpr bool set_fw_roll_limit(int32_t&) { return false; }
    static constexpr bool set_vtol_roll_pitch_limit(int32_t&, int32_t&) { return false; }
    static constexpr bool update_yaw_target(float&) { return false; }
};

inline bool q_option_level_transition(std::int32_t q_options) {
    return (q_options & kQOptionsLevelTransition) != 0;
}

inline bool slt_allow_update_throttle_mix(TransitionState state, bool assisted_flight) {
    if (!assisted_flight) {
        return TransitionBaseDefaults::allow_update_throttle_mix();
    }
    return state != TransitionState::kAirspeedWait && state != TransitionState::kTimer;
}

inline bool slt_set_fw_roll_limit(int32_t& roll_limit_cd, TransitionState state,
                                  bool assisted_flight, std::int32_t q_options,
                                  std::int32_t level_roll_limit_cd) {
    if (assisted_flight &&
        (state == TransitionState::kAirspeedWait || state == TransitionState::kTimer) &&
        q_option_level_transition(q_options)) {
        roll_limit_cd = std::min(roll_limit_cd, level_roll_limit_cd);
        return true;
    }
    return false;
}

inline float linear_interpolate(float a, float b, float t, float t0, float t1) {
    if (t1 <= t0) {
        return b;
    }
    const float clamped = std::max(t0, std::min(t, t1));
    const float ratio = (clamped - t0) / (t1 - t0);
    return a + (b - a) * ratio;
}

struct SltFwRollPitchResult {
    bool limited{false};
    float tecs_pitch_max_rad{0.f};
    float tecs_pitch_min_rad{0.f};
};

inline SltFwRollPitchResult slt_set_fw_roll_pitch(
    int32_t& nav_pitch_cd, int32_t&, TransitionState state, bool in_vtol_mode,
    bool in_vtol_airbrake, bool does_auto_throttle, float groundspeed_ms,
    float transition_pitch_max) {
    SltFwRollPitchResult out{};
    if (in_vtol_mode || in_vtol_airbrake || state == TransitionState::kDone || !does_auto_throttle) {
        return out;
    }
    float max_pitch = transition_pitch_max;
    if (state == TransitionState::kAirspeedWait) {
        max_pitch = groundspeed_ms < 3.0f ? 0.0f : transition_pitch_max;
    } else {
        max_pitch = (transition_pitch_max + 1.0f) * 2.0f;
    }
    out.tecs_pitch_max_rad = max_pitch;
    out.tecs_pitch_min_rad = -max_pitch;
    const int32_t lim = static_cast<int32_t>(max_pitch * 100.0f);
    const int32_t clamped = std::max(-lim, std::min(nav_pitch_cd, lim));
    if (clamped != nav_pitch_cd) {
        nav_pitch_cd = clamped;
        out.limited = true;
    }
    return out;
}

inline bool slt_set_vtol_roll_pitch_limit(
    int32_t& roll_cd, int32_t& pitch_cd, float angle_max_cd, float airspeed_ms,
    float airspeed_min_ms, std::uint32_t now_ms, std::uint32_t back_trans_pitch_limit_ms,
    std::uint32_t& last_fw_mode_ms, std::int32_t last_fw_nav_pitch_cd,
    std::int32_t pitch_limit_max_cd, std::int32_t pitch_limit_min_cd) {
    bool ret = false;
    const int32_t angle_max_i = static_cast<int32_t>(angle_max_cd);
    const int32_t new_roll = std::max(-angle_max_i, std::min(roll_cd, angle_max_i));
    if (new_roll != roll_cd) {
        roll_cd = new_roll;
        ret = true;
    }
    if (pitch_cd < -angle_max_i) {
        pitch_cd = -angle_max_i;
        ret = true;
    }
    if (pitch_cd > angle_max_i && airspeed_ms < 0.5f * airspeed_min_ms) {
        const float max_limit = linear_interpolate(angle_max_cd, 4500.0f, airspeed_ms, 0.0f,
                                                   0.5f * airspeed_min_ms);
        const int32_t max_i = static_cast<int32_t>(max_limit);
        if (pitch_cd > max_i) {
            pitch_cd = max_i;
            ret = true;
        }
    }
    if (back_trans_pitch_limit_ms == 0 || last_fw_mode_ms == 0) {
        return ret;
    }
    const std::uint32_t dt = now_ms - last_fw_mode_ms;
    if (dt > back_trans_pitch_limit_ms) {
        last_fw_mode_ms = 0;
        return ret;
    }
    const float max_limit_cd = linear_interpolate(
        static_cast<float>(std::max(last_fw_nav_pitch_cd, 0)),
        static_cast<float>(std::min(angle_max_i, pitch_limit_max_cd)),
        static_cast<float>(dt), 0.0f, static_cast<float>(back_trans_pitch_limit_ms));
    if (pitch_cd > static_cast<int32_t>(max_limit_cd)) {
        pitch_cd = static_cast<int32_t>(max_limit_cd);
        return true;
    }
    const float min_limit_cd = linear_interpolate(
        static_cast<float>(std::min(last_fw_nav_pitch_cd, 0)),
        static_cast<float>(std::max(-angle_max_i, pitch_limit_min_cd)),
        static_cast<float>(dt), 0.0f, static_cast<float>(back_trans_pitch_limit_ms));
    if (pitch_cd < static_cast<int32_t>(min_limit_cd)) {
        pitch_cd = static_cast<int32_t>(min_limit_cd);
        return true;
    }
    return ret;
}

}  // namespace fwcpp::quadplane_transition
