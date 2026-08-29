#pragma once

#include <cstdint>
// CCP-027 slice 3: D_update_controller vertical PID path.
// Rust spec: plane-fw-rust pos_control_ne.rs (D_update_controller, vibe override).

#include <cmath>

#include <fwcpp/math/control.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/pid/ac_p_1d.hpp>
#include <fwcpp/pid/ac_pid.hpp>
#include <fwcpp/pid/ac_pid_basic.hpp>

namespace fwcpp::poscontrol {

inline constexpr float kPoscontrolVibeCompPGain = 0.250f;
inline constexpr float kPoscontrolVibeCompIGain = 0.125f;

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

    [[nodiscard]] static PosControlD zero() { return PosControlD{}; }

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

}  // namespace fwcpp::poscontrol
