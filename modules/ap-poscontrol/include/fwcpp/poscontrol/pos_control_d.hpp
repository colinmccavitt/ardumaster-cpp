#pragma once

#include <cstdint>
// CCP-027 slice 3–5: D_update_controller, limits, init/relax, throttle paths.
// Upstream: AC_PosControl.cpp D-axis methods.

#include <algorithm>
#include <cmath>

#include <fwcpp/math/control.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/pid/ac_p_1d.hpp>
#include <fwcpp/pid/ac_pid.hpp>
#include <fwcpp/pid/ac_pid_basic.hpp>
#include <fwcpp/poscontrol/pos_control_defaults.hpp>
#include <fwcpp/poscontrol/pos_control_lean.hpp>
#include <fwcpp/poscontrol/pos_control_ne.hpp>

namespace fwcpp::poscontrol {

inline constexpr float kPoscontrolVibeCompPGain = 0.250f;
inline constexpr float kPoscontrolVibeCompIGain = 0.125f;
inline constexpr float kTwoPi = 6.283185307f;

struct DOffsets {
    math::postype_t pos_m{};
    float vel_ms = 0.0f;
    float accel_mss = 0.0f;
};

struct DTerrain {
    math::postype_t pos_m{};
    float vel_ms = 0.0f;
    float accel_mss = 0.0f;
};

struct DEstimates {
    math::postype_t pos_m{};
    float vel_ms = 0.0f;
};

struct DLimits {
    float vel_max_up_ms = 0.0f;
    float vel_max_down_ms = 0.0f;
    float accel_max_d_mss = 0.0f;
    float jerk_max_d_msss = 0.0f;

    [[nodiscard]] static DLimits defaults() {
        return DLimits{kPoscontrolSpeedUpMs, kPoscontrolSpeedDownMs, kPoscontrolAccelDMss,
                       kPoscontrolJerkDMsss};
    }
};

[[nodiscard]] inline DLimits d_set_max_speed_accel_m(DLimits limits, float descent_speed_max_ms,
                                                     float climb_speed_max_ms, float accel_max_d_mss,
                                                     float shaping_jerk_d_msss, pid::AcPid& accel_pid) {
    if (!math::is_zero(descent_speed_max_ms)) {
        limits.vel_max_down_ms = std::fabs(descent_speed_max_ms);
    }
    if (!math::is_zero(climb_speed_max_ms)) {
        limits.vel_max_up_ms = std::fabs(climb_speed_max_ms);
    }
    if (!math::is_zero(accel_max_d_mss)) {
        limits.accel_max_d_mss = std::fabs(accel_max_d_mss);
    }

    limits.jerk_max_d_msss = shaping_jerk_d_msss;
    const float filt_accel_cap = std::min(kGravityMss, limits.accel_max_d_mss);
    if (math::is_positive(accel_pid.filt_T_hz())) {
        limits.jerk_max_d_msss =
            std::min(limits.jerk_max_d_msss,
                     filt_accel_cap * (kTwoPi * accel_pid.filt_T_hz()) / 5.0f);
    }
    if (math::is_positive(accel_pid.filt_E_hz())) {
        limits.jerk_max_d_msss =
            std::min(limits.jerk_max_d_msss,
                     filt_accel_cap * (kTwoPi * accel_pid.filt_E_hz()) / 5.0f);
    }
    return limits;
}

[[nodiscard]] inline DLimits d_set_max_speed_accel_cm(DLimits limits, float descent_speed_max_cms,
                                                        float climb_speed_max_cms,
                                                        float accel_max_d_cmss, float shaping_jerk_d_msss,
                                                        pid::AcPid& accel_pid) {
    return d_set_max_speed_accel_m(limits, descent_speed_max_cms * 0.01f, climb_speed_max_cms * 0.01f,
                                   accel_max_d_cmss * 0.01f, shaping_jerk_d_msss, accel_pid);
}

inline void d_set_correction_speed_accel_m(pid::AcP1d& pos_p, float descent_speed_max_ms,
                                           float climb_speed_max_ms, float accel_max_d_mss) {
    pos_p.set_limits(-std::fabs(descent_speed_max_ms), std::fabs(climb_speed_max_ms),
                     std::fabs(accel_max_d_mss), 0.0f);
}

inline void d_set_correction_speed_accel_cm(pid::AcP1d& pos_p, float descent_speed_max_cms,
                                            float climb_speed_max_cms, float accel_max_d_cmss) {
    d_set_correction_speed_accel_m(pos_p, descent_speed_max_cms * 0.01f, climb_speed_max_cms * 0.01f,
                                   accel_max_d_cmss * 0.01f);
}

[[nodiscard]] inline float calculate_d_overspeed_gain(float vel_desired_ms, float vel_max_down_ms,
                                                      float vel_max_up_ms) {
    if (vel_desired_ms > vel_max_down_ms && !math::is_zero(vel_max_down_ms)) {
        return kPoscontrolOverspeedGainU * vel_desired_ms / vel_max_down_ms;
    }
    if (vel_desired_ms < -vel_max_up_ms && !math::is_zero(vel_max_up_ms)) {
        return -kPoscontrolOverspeedGainU * vel_desired_ms / vel_max_up_ms;
    }
    return 1.0f;
}

[[nodiscard]] inline math::postype_t stopping_point_d(math::postype_t pos_estimate_m,
                                                      math::postype_t pos_offset_m,
                                                      float vel_estimate_ms, float vel_offset_ms,
                                                      float kp, float accel_max_d_mss) {
    const float curr_pos_d_m = static_cast<float>(pos_estimate_m - pos_offset_m);
    const float curr_vel_d_ms = vel_estimate_ms - vel_offset_ms;

    if (!math::is_positive(kp) || !math::is_positive(accel_max_d_mss)) {
        return math::postype_t{curr_pos_d_m};
    }

    const float delta_m = math::constrain_value(
        math::stopping_distance(curr_vel_d_ms, kp, accel_max_d_mss), -kPoscontrolStoppingDistUpMaxM,
        kPoscontrolStoppingDistDownMaxM);
    return math::postype_t{curr_pos_d_m + delta_m};
}

struct DUpdateInputs {
    float dt = 0.0f;
    std::uint32_t now_ms = 0;
    float ahrs_control_scale_z = 1.0f;
    DEstimates estimates{};
    DOffsets offsets{};
    DTerrain terrain{};
    float estimated_accel_d_mss = 0.0f;
    bool throttle_lower = false;
    bool throttle_upper = false;
    float throttle_hover = 0.0f;
    bool vibe_comp_enabled = false;
    float vel_max_down_ms = 0.0f;
};

struct DUpdateOutput {
    math::postype_t pos_target_m{};
    float vel_target_ms = 0.0f;
    float accel_target_mss = 0.0f;
    float thrust_d_norm = 0.0f;
    float throttle_out = 0.0f;
    float vel_d_control_ratio = 0.0f;
    float limit = 0.0f;
};

struct DInitInputs {
    DEstimates estimates{};
    float estimated_accel_d_mss = 0.0f;
    float throttle_in = 0.0f;
    float throttle_hover = 0.0f;
    float accel_max_d_mss = 0.0f;
    std::uint32_t now_ms = 0;
    std::uint32_t ticks = 0;
    std::uint32_t last_update_ticks = 0;
    std::uint16_t position_d_reset_count = 0;
};

struct DInitOutput {
    math::postype_t pos_target_m{};
    float vel_target_ms = 0.0f;
    float accel_target_mss = 0.0f;
    std::uint32_t last_update_ticks = 0;
    std::uint16_t position_d_reset_count = 0;
};

struct DOffsetState {
    DOffsets current{};
    DOffsets target{};
    std::uint32_t target_ms = 0;

    void init(std::uint32_t now_ms) {
        if (offset_target_timed_out(now_ms, target_ms)) {
            target = DOffsets{};
        }
        current = target;
    }
};

[[nodiscard]] inline float throttle_with_vibration_override(pid::AcPid& accel_pid, float vel_error,
                                                            float vel_kp, float accel_target_mss,
                                                            float dt, float throttle_hover,
                                                            bool throttle_limited) {
    const float i = accel_pid.get_i();
    const bool helping = (math::is_positive(i) && math::is_negative(vel_error)) ||
                         (math::is_negative(i) && math::is_positive(vel_error));
    if (!throttle_limited || helping) {
        accel_pid.set_integrator(i + dt * throttle_hover * vel_error * vel_kp *
                                        kPoscontrolVibeCompIGain);
    }
    return kPoscontrolVibeCompPGain * throttle_hover * accel_target_mss + accel_pid.get_i();
}

struct PosControlD {
    math::postype_t pos_desired_m{};
    float vel_desired_ms = 0.0f;
    float accel_desired_mss = 0.0f;
    float limit = 0.0f;
    float vel_d_control_ratio = 2.0f;
    float limit_vector = 0.0f;

    [[nodiscard]] static PosControlD zero() { return PosControlD{}; }

    void advance_desired(float dt, float pos_error, float vel_error) {
        math::update_pos_vel_accel(pos_desired_m, vel_desired_ms, accel_desired_mss, dt, limit_vector,
                                   pos_error, vel_error);
    }

    void input_accel(float accel_d_mss, const DLimits& limits, float dt, float pos_error,
                     float vel_error) {
        const float overspeed_gain = calculate_d_overspeed_gain(vel_desired_ms, limits.vel_max_down_ms,
                                                                limits.vel_max_up_ms);
        advance_desired(dt, pos_error, vel_error);
        math::shape_accel(accel_d_mss, accel_desired_mss, limits.jerk_max_d_msss * overspeed_gain, dt);
    }

    void input_vel_accel(float& vel_d_ms, float accel_d_mss, const DLimits& limits, float dt,
                         bool limit_output, float pos_error, float vel_error) {
        const float overspeed_gain = calculate_d_overspeed_gain(vel_desired_ms, limits.vel_max_down_ms,
                                                                limits.vel_max_up_ms);
        const float accel_max = limits.accel_max_d_mss * overspeed_gain;
        const float jerk_max = limits.jerk_max_d_msss * overspeed_gain;
        advance_desired(dt, pos_error, vel_error);
        math::shape_vel_accel(vel_d_ms, accel_d_mss, vel_desired_ms, accel_desired_mss, -accel_max,
                              math::constrain_value(accel_max, 0.0f, 7.5f), jerk_max, dt,
                              limit_output);
        math::update_vel_accel(vel_d_ms, accel_d_mss, dt, 0.0f, 0.0f);
    }

    void input_pos_vel_accel(math::postype_t& pos_d_m, float& vel_d_ms, float accel_d_mss,
                           const DLimits& limits, float dt, bool limit_output, float pos_error,
                           float vel_error) {
        const float overspeed_gain = calculate_d_overspeed_gain(vel_desired_ms, limits.vel_max_down_ms,
                                                                limits.vel_max_up_ms);
        const float accel_max = limits.accel_max_d_mss * overspeed_gain;
        const float jerk_max = limits.jerk_max_d_msss * overspeed_gain;
        advance_desired(dt, pos_error, vel_error);
        math::shape_pos_vel_accel(pos_d_m, vel_d_ms, accel_d_mss, pos_desired_m, vel_desired_ms,
                                  accel_desired_mss, -limits.vel_max_up_ms, limits.vel_max_down_ms,
                                  -accel_max, math::constrain_value(accel_max, 0.0f, 7.5f), jerk_max,
                                  dt, limit_output);
        math::update_pos_vel_accel(pos_d_m, vel_d_ms, accel_d_mss, dt, 0.0f, 0.0f, 0.0f);
    }

    [[nodiscard]] DInitOutput init_controller(DOffsetState& offsets, pid::AcPidBasic& vel_pid,
                                              pid::AcPid& accel_pid, const DInitInputs& inp) {
        offsets.init(inp.now_ms);

        const math::postype_t pos_target = inp.estimates.pos_m;
        pos_desired_m = pos_target - offsets.current.pos_m;

        const float vel_target = inp.estimates.vel_ms;
        vel_desired_ms = vel_target - offsets.current.vel_ms;

        vel_pid.reset_filter();
        vel_pid.set_integrator(0.0f);

        const float accel_target =
            math::constrain_value(inp.estimated_accel_d_mss, -inp.accel_max_d_mss, inp.accel_max_d_mss);
        accel_desired_mss = accel_target - offsets.current.accel_mss;

        accel_pid.reset_filter();
        accel_pid.set_integrator(-(inp.throttle_in - inp.throttle_hover) -
                                 accel_pid.kP() * (accel_target - inp.estimated_accel_d_mss) -
                                 accel_pid.ff() * accel_target);

        return DInitOutput{pos_target, vel_target, accel_target, inp.ticks,
                           inp.position_d_reset_count};
    }

    [[nodiscard]] DInitOutput init_controller_no_descent(DOffsetState& offsets,
                                                         pid::AcPidBasic& vel_pid,
                                                         pid::AcPid& accel_pid,
                                                         const DInitInputs& inp) {
        DInitOutput out = init_controller(offsets, vel_pid, accel_pid, inp);
        vel_desired_ms = std::min(0.0f, vel_desired_ms);
        out.vel_target_ms = std::min(0.0f, out.vel_target_ms);
        accel_desired_mss = std::min(0.0f, accel_desired_mss);
        out.accel_target_mss = std::min(0.0f, out.accel_target_mss);
        return out;
    }

    [[nodiscard]] DInitOutput init_controller_stopping_point(DOffsetState& offsets,
                                                             pid::AcPidBasic& vel_pid,
                                                             pid::AcPid& accel_pid,
                                                             const DInitInputs& inp, float kp,
                                                             const DLimits& limits) {
        DInitOutput out = init_controller(offsets, vel_pid, accel_pid, inp);
        pos_desired_m = stopping_point_d(inp.estimates.pos_m, offsets.current.pos_m,
                                         inp.estimates.vel_ms, offsets.current.vel_ms, kp,
                                         limits.accel_max_d_mss);
        out.pos_target_m = pos_desired_m + offsets.current.pos_m;
        vel_desired_ms = 0.0f;
        accel_desired_mss = 0.0f;
        out.vel_target_ms = inp.estimates.vel_ms;
        out.accel_target_mss = out.accel_target_mss;
        return out;
    }

    [[nodiscard]] DUpdateOutput update_controller(pid::AcP1d& pos_p, pid::AcPidBasic& vel_pid,
                                                  pid::AcPid& accel_pid, const DUpdateInputs& inp) {
        math::postype_t pos_target = pos_desired_m + inp.offsets.pos_m + inp.terrain.pos_m;

        float vel_target = pos_p.update_all(pos_target, inp.estimates.pos_m);
        vel_target *= inp.ahrs_control_scale_z;

        pos_desired_m = pos_target - (inp.offsets.pos_m + inp.terrain.pos_m);

        vel_target += vel_desired_ms + inp.offsets.vel_ms + inp.terrain.vel_ms;

        float accel_target = vel_pid.update_all(vel_target, inp.estimates.vel_ms, inp.dt,
                                                inp.throttle_lower, inp.throttle_upper);
        accel_target *= inp.ahrs_control_scale_z;
        accel_target += accel_desired_mss + inp.offsets.accel_mss + inp.terrain.accel_mss;

        if (inp.throttle_hover > accel_pid.imax()) {
            accel_pid.set_imax(std::fabs(inp.throttle_hover));
        }

        float thrust_d_norm = 0.0f;
        if (inp.vibe_comp_enabled) {
            thrust_d_norm = throttle_with_vibration_override(
                accel_pid, vel_pid.error(), vel_pid.kp, accel_target, inp.dt, inp.throttle_hover,
                inp.throttle_lower || inp.throttle_upper);
        } else {
            float thrust = accel_pid.update_all(accel_target, inp.estimated_accel_d_mss, inp.dt,
                                                inp.now_ms, inp.throttle_lower || inp.throttle_upper);
            thrust += accel_pid.get_ff();
            thrust_d_norm = thrust;
        }
        thrust_d_norm -= inp.throttle_hover;

        const float error_ratio = vel_pid.error() / inp.vel_max_down_ms;
        vel_d_control_ratio += inp.dt * 0.1f * (0.5f - error_ratio);
        vel_d_control_ratio = math::constrain_value(vel_d_control_ratio, 0.0f, 2.0f);

        limit = inp.throttle_upper ? -1.0f : (inp.throttle_lower ? 1.0f : 0.0f);

        return DUpdateOutput{pos_target,
                             vel_target,
                             accel_target,
                             thrust_d_norm,
                             -thrust_d_norm,
                             vel_d_control_ratio,
                             limit};
    }
};

inline void d_relax_controller(PosControlD& d, pid::AcPid& accel_pid, float dt,
                               float throttle_setting, float throttle_hover,
                               DOffsetState& offsets, pid::AcPidBasic& vel_pid,
                               const DInitInputs& inp) {
    (void)d.init_controller(offsets, vel_pid, accel_pid, inp);
    accel_pid.relax_integrator(-(throttle_setting - throttle_hover), dt, kPoscontrolRelaxTc);
}

}  // namespace fwcpp::poscontrol
