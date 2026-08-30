#pragma once

#include <utility>
// CCP-027 slice 2–4: NE_update_controller, limits, input_*, init/relax/soften/stop.
// Rust spec: ardumaster-rust pos_control_ne.rs (update_controller, yaw_from_ne_motion).

#include <algorithm>
#include <cstdint>
#include <cmath>

#include <fwcpp/math/control.hpp>
#include <fwcpp/math/control_vector.hpp>
#include <fwcpp/math/control_vector_kinematic.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/pid/ac_p_2d.hpp>
#include <fwcpp/pid/ac_pid_2d.hpp>
#include <fwcpp/poscontrol/pos_control_defaults.hpp>
#include <fwcpp/poscontrol/pos_control_lean.hpp>

namespace fwcpp::poscontrol {

inline constexpr float kNePosP = 1.0f;
inline constexpr std::uint32_t kPosvelaccelOffsetTargetTimeoutMs = 3000;

struct AttitudeCapability {
    float ang_vel_roll_max_rads = 0.0f;
    float ang_vel_pitch_max_rads = 0.0f;
    float accel_roll_max_radss = 0.0f;
    float accel_pitch_max_radss = 0.0f;
    bool bf_feedforward = false;
};

struct NeLimits {
    float vel_max_ne_ms = 0.0f;
    float accel_max_ne_mss = 0.0f;
    float jerk_max_ne_msss = 0.0f;

    [[nodiscard]] static NeLimits derive(float speed_ne_ms, float accel_ne_mss,
                                         float shaping_jerk_ne_msss,
                                         const AttitudeCapability& attitude) {
        NeLimits out{};
        out.vel_max_ne_ms = std::fabs(speed_ne_ms);
        out.accel_max_ne_mss = std::fabs(accel_ne_mss);

        const float jerk_max_msss = std::min(attitude.ang_vel_roll_max_rads,
                                             attitude.ang_vel_pitch_max_rads) *
                                    kGravityMss;
        const float snap_max_mssss = std::min(attitude.accel_roll_max_radss,
                                              attitude.accel_pitch_max_radss) *
                                     kGravityMss;

        out.jerk_max_ne_msss = shaping_jerk_ne_msss;

        if (math::is_positive(jerk_max_msss) && attitude.bf_feedforward) {
            out.jerk_max_ne_msss = std::min(out.jerk_max_ne_msss, jerk_max_msss);
        }

        if (math::is_positive(snap_max_mssss) && attitude.bf_feedforward) {
            out.jerk_max_ne_msss =
                std::min(0.5f * math::safe_sqrt(out.accel_max_ne_mss * snap_max_mssss),
                         out.jerk_max_ne_msss);
        }
        return out;
    }
};

[[nodiscard]] inline NeLimits ne_set_max_speed_accel_m(float speed_ne_ms, float accel_ne_mss,
                                                       float shaping_jerk_ne_msss,
                                                       const AttitudeCapability& attitude) {
    return NeLimits::derive(speed_ne_ms, accel_ne_mss, shaping_jerk_ne_msss, attitude);
}

[[nodiscard]] inline NeLimits ne_set_max_speed_accel_cm(float speed_ne_cms, float accel_ne_cmss,
                                                        float shaping_jerk_ne_msss,
                                                        const AttitudeCapability& attitude) {
    return ne_set_max_speed_accel_m(speed_ne_cms * 0.01f, accel_ne_cmss * 0.01f,
                                    shaping_jerk_ne_msss, attitude);
}

inline void ne_set_correction_speed_accel_m(pid::AcP2d& pos_p, float speed_ne_ms,
                                            float accel_ne_mss) {
    pos_p.set_limits(speed_ne_ms, accel_ne_mss, 0.0f);
}

inline void ne_set_correction_speed_accel_cm(pid::AcP2d& pos_p, float speed_ne_cms,
                                             float accel_ne_cmss) {
    ne_set_correction_speed_accel_m(pos_p, speed_ne_cms * 0.01f, accel_ne_cmss * 0.01f);
}

[[nodiscard]] inline bool offset_target_timed_out(std::uint32_t now_ms, std::uint32_t target_ms) {
    return now_ms - target_ms > kPosvelaccelOffsetTargetTimeoutMs;
}

[[nodiscard]] inline bool controller_is_active(std::uint32_t ticks, std::uint32_t last_update_ticks) {
    return ticks - last_update_ticks <= 1;
}

[[nodiscard]] inline math::Vector2<math::postype_t> stopping_point_ne(
    math::Vector2<math::postype_t> pos_estimate_m, math::Vector2<math::postype_t> pos_offset_m,
    math::Vector2f vel_estimate_ms, math::Vector2f vel_offset_ms, float kp, const NeLimits& limits) {
    math::Vector2<math::postype_t> stopping_point{
        pos_estimate_m.x - pos_offset_m.x,
        pos_estimate_m.y - pos_offset_m.y,
    };

    const math::Vector2f vel = vel_estimate_ms - vel_offset_ms;
    const float speed_ms = vel.length();
    if (!math::is_positive(speed_ms)) {
        return stopping_point;
    }

    const float stopping_dist_m = math::stopping_distance(
        math::constrain_value(speed_ms, 0.0f, limits.vel_max_ne_ms), kp, limits.accel_max_ne_mss);
    if (!math::is_positive(stopping_dist_m)) {
        return stopping_point;
    }

    const float stopping_time_s = stopping_dist_m / speed_ms;
    stopping_point.x += math::postype_t{vel.x * stopping_time_s};
    stopping_point.y += math::postype_t{vel.y * stopping_time_s};
    return stopping_point;
}

struct NeEstimates {
    math::Vector2<math::postype_t> pos_m{};
    math::Vector2f vel_ms{};
};

struct NeOffsets {
    math::Vector2<math::postype_t> pos_m{};
    math::Vector2f vel_ms{};
    math::Vector2f accel_mss{};
};

struct NeDisturbance {
    math::Vector2f pos_m{};
    math::Vector2f vel_ms{};
};

struct NeInitInputs {
    NeEstimates estimates{};
    math::Vector3f att_target_euler_rad{};
    float ahrs_yaw = 0.0f;
    float lean_angle_max_rad = 0.0f;
    std::uint32_t now_ms = 0;
    std::uint32_t ticks = 0;
    std::uint32_t last_update_ticks = 0;
    std::uint32_t ahrs_ekf_reset_ms = 0;
    math::Vector2f accel_target_mss{};
};

struct NeInitOutput {
    math::Vector2<math::postype_t> pos_target_m{};
    math::Vector2f vel_target_ms{};
    math::Vector2f accel_target_mss{};
    float roll_target_rad = 0.0f;
    float pitch_target_rad = 0.0f;
    float yaw_target_rad = 0.0f;
    float yaw_rate_target_rads = 0.0f;
    float angle_max_override_rad = 0.0f;
    std::uint32_t last_update_ticks = 0;
    std::uint32_t ekf_last_reset_ms = 0;
};

struct NeOffsetState {
    NeOffsets current{};
    NeOffsets target{};
    std::uint32_t target_ms = 0;

    void init(std::uint32_t now_ms) {
        if (offset_target_timed_out(now_ms, target_ms)) {
            target = NeOffsets{};
        }
        current = target;
    }
};

struct NeStopTargets {
    math::Vector2<math::postype_t> pos_target_m{};
    math::Vector2<math::postype_t> pos_desired_m{};
    math::Vector2f vel_target_ms{};
    math::Vector2f vel_desired_ms{};
    NeOffsets offsets{};
};


struct NeUpdateInputs {
    float dt = 0.0f;
    float ahrs_control_scale_xy = 1.0f;
    float ne_control_scale_factor = 1.0f;
    float vel_max_ne_ms = 0.0f;
    NeEstimates estimates{};
    NeOffsets offsets{};
    float lean_angle_max_rad = 0.0f;
    float cos_yaw = 1.0f;
    float sin_yaw = 0.0f;
    float att_yaw_target_rad = 0.0f;
};

struct NeUpdateOutput {
    math::Vector2<math::postype_t> pos_target_m{};
    math::Vector2f vel_target_ms{};
    math::Vector2f accel_target_mss{};
    float roll_target_rad = 0.0f;
    float pitch_target_rad = 0.0f;
    float yaw_target_rad = 0.0f;
    float yaw_rate_target_rads = 0.0f;
    bool limited = false;
    float ne_control_scale_factor = 1.0f;
};


[[nodiscard]] inline std::pair<float, float> yaw_from_ne_motion(math::Vector2f vel_desired_ms,
                                                                 math::Vector2f accel_desired_mss,
                                                                 float vel_max_ne_ms,
                                                                 float att_yaw_target_rad);

struct PosControlNe {
    math::Vector2<math::postype_t> pos_desired_m{};
    math::Vector2f vel_desired_ms{};
    math::Vector2f accel_desired_mss{};
    math::Vector2f limit_vector{};

    [[nodiscard]] static PosControlNe zero() { return PosControlNe{}; }


    void advance_desired(float dt, math::Vector2f pos_error, math::Vector2f vel_error) {
        math::update_pos_vel_accel_xy(pos_desired_m, vel_desired_ms, accel_desired_mss, dt, limit_vector,
                                      pos_error, vel_error);
    }

    void input_accel(math::Vector2f accel_ne_mss, const NeLimits& limits, float dt,
                     math::Vector2f pos_error, math::Vector2f vel_error) {
        advance_desired(dt, pos_error, vel_error);
        math::shape_accel_xy(accel_ne_mss, accel_desired_mss, limits.jerk_max_ne_msss, dt);
    }

    void input_vel_accel(math::Vector2f& vel_ne_ms, math::Vector2f accel_ne_mss, const NeLimits& limits,
                         float dt, bool limit_output, math::Vector2f pos_error,
                         math::Vector2f vel_error) {
        advance_desired(dt, pos_error, vel_error);
        math::shape_vel_accel_xy(vel_ne_ms, accel_ne_mss, vel_desired_ms, accel_desired_mss,
                                 limits.accel_max_ne_mss, limits.jerk_max_ne_msss, dt, limit_output);
        math::update_vel_accel_xy(vel_ne_ms, accel_ne_mss, dt, math::Vector2f{}, math::Vector2f{});
    }

    void input_pos_vel_accel(math::Vector2<math::postype_t>& pos_ne_m, math::Vector2f& vel_ne_ms,
                             math::Vector2f accel_ne_mss, const NeLimits& limits, float dt,
                             bool limit_output, math::Vector2f pos_error, math::Vector2f vel_error) {
        advance_desired(dt, pos_error, vel_error);
        math::shape_pos_vel_accel_xy(pos_ne_m, vel_ne_ms, accel_ne_mss, pos_desired_m, vel_desired_ms,
                                     accel_desired_mss, limits.vel_max_ne_ms, limits.accel_max_ne_mss,
                                     limits.jerk_max_ne_msss, dt, limit_output);
        math::update_pos_vel_accel_xy(pos_ne_m, vel_ne_ms, accel_ne_mss, dt, math::Vector2f{},
                                      math::Vector2f{}, math::Vector2f{});
    }

    void relax_velocity(math::Vector2f& accel_target_mss, float dt) {
        if (math::is_positive(dt)) {
            const float decay = 1.0f - dt / (dt + kPoscontrolRelaxTc);
            accel_target_mss *= decay;
        }
    }

    [[nodiscard]] NeInitOutput init_controller(NeOffsetState& offsets, pid::AcPid2d& vel_pid,
                                               const NeInitInputs& inp) {
        offsets.init(inp.now_ms);

        const math::Vector2<math::postype_t> pos_target = inp.estimates.pos_m;
        pos_desired_m = {
            pos_target.x - offsets.current.pos_m.x,
            pos_target.y - offsets.current.pos_m.y,
        };

        const math::Vector2f vel_target = inp.estimates.vel_ms;
        vel_desired_ms = vel_target - offsets.current.vel_ms;
        accel_desired_mss.zero();

        math::Vector2f accel_target = inp.accel_target_mss;
        if (!controller_is_active(inp.ticks, inp.last_update_ticks)) {
            math::Vector3f att = inp.att_target_euler_rad;
            att.z = inp.ahrs_yaw;
            const math::Vector3f accel = lean_angles_rad_to_accel_ned_mss(att);
            accel_target = {accel.x, accel.y};
        }

        const float accel_max = angle_rad_to_accel_mss(inp.lean_angle_max_rad);
        accel_target.limit_length(accel_max);

        vel_pid.reset_filter();
        vel_pid.set_integrator(accel_target - vel_target * vel_pid.kff);

        return NeInitOutput{
            pos_target,
            vel_target,
            accel_target,
            inp.att_target_euler_rad.x,
            inp.att_target_euler_rad.y,
            inp.att_target_euler_rad.z,
            0.0f,
            0.0f,
            inp.ticks,
            inp.ahrs_ekf_reset_ms,
        };
    }

    [[nodiscard]] NeInitOutput init_controller_stopping_point(NeOffsetState& offsets,
                                                              pid::AcPid2d& vel_pid,
                                                              const NeInitInputs& inp, float kp,
                                                              const NeLimits& limits) {
        NeInitOutput out = init_controller(offsets, vel_pid, inp);
        pos_desired_m = stopping_point_ne(inp.estimates.pos_m, offsets.current.pos_m,
                                          inp.estimates.vel_ms, offsets.current.vel_ms, kp, limits);
        out.pos_target_m = {
            pos_desired_m.x + offsets.current.pos_m.x,
            pos_desired_m.y + offsets.current.pos_m.y,
        };
        vel_desired_ms.zero();
        accel_desired_mss.zero();
        return out;
    }

    [[nodiscard]] NeUpdateOutput update_controller(pid::AcP2d& pos_p, pid::AcPid2d& vel_pid,
                                                   const NeUpdateInputs& inp,
                                                   NeDisturbance& disturb) {
        const float scale = inp.ahrs_control_scale_xy * inp.ne_control_scale_factor;

        math::Vector2<math::postype_t> pos_target{
            pos_desired_m.x + inp.offsets.pos_m.x,
            pos_desired_m.y + inp.offsets.pos_m.y,
        };

        math::Vector2<math::postype_t> comb_pos{
            inp.estimates.pos_m.x + math::postype_t{disturb.pos_m.x},
            inp.estimates.pos_m.y + math::postype_t{disturb.pos_m.y},
        };

        math::Vector2f vel_target = pos_p.update_all(pos_target, comb_pos);
        pos_desired_m = math::Vector2<math::postype_t>{
            pos_target.x - inp.offsets.pos_m.x,
            pos_target.y - inp.offsets.pos_m.y,
        };

        vel_target *= scale;
        vel_target += vel_desired_ms + inp.offsets.vel_ms;

        const math::Vector2f comb_vel = inp.estimates.vel_ms + disturb.vel_ms;
        math::Vector2f accel_target =
            vel_pid.update_all(vel_target, comb_vel, inp.dt, limit_vector);

        accel_target *= scale;
        accel_target += accel_desired_mss + inp.offsets.accel_mss;

        const float accel_max = angle_rad_to_accel_mss(inp.lean_angle_max_rad);
        limit_vector = accel_target;
        const bool limited = math::limit_accel_xy(vel_desired_ms, accel_target, accel_max);
        if (!limited) {
            limit_vector.zero();
        }

        float roll_target_rad = 0.0f;
        float pitch_target_rad = 0.0f;
        accel_ne_mss_to_lean_angles_rad(accel_target.x, accel_target.y, inp.cos_yaw, inp.sin_yaw,
                                        roll_target_rad, pitch_target_rad);

        const auto yaw_pair = yaw_from_ne_motion(vel_desired_ms, accel_desired_mss, inp.vel_max_ne_ms,
                                                 inp.att_yaw_target_rad);

        disturb.pos_m.zero();
        disturb.vel_ms.zero();

        return NeUpdateOutput{
            pos_target,
            vel_target,
            accel_target,
            roll_target_rad,
            pitch_target_rad,
            yaw_pair.first,
            yaw_pair.second,
            limited,
            1.0f,
        };
    }
};


inline void ne_relax_velocity_controller(PosControlNe& ne, math::Vector2f& accel_target_mss, float dt,
                                       NeOffsetState& offsets, pid::AcPid2d& vel_pid,
                                       const NeInitInputs& inp) {
    ne.relax_velocity(accel_target_mss, dt);
    (void)ne.init_controller(offsets, vel_pid, inp);
}

inline void ne_soften_for_landing(NeStopTargets& targets, const NeEstimates& estimates, float dt,
                                  math::Vector2f& limit_vector, math::Vector2f accel_target_mss) {
    if (math::is_positive(dt)) {
        const float blend = dt / (dt + kPoscontrolRelaxTc);
        targets.pos_target_m.x += math::postype_t{
            static_cast<float>(estimates.pos_m.x - targets.pos_target_m.x) * blend};
        targets.pos_target_m.y += math::postype_t{
            static_cast<float>(estimates.pos_m.y - targets.pos_target_m.y) * blend};
        targets.pos_desired_m = {
            targets.pos_target_m.x - targets.offsets.pos_m.x,
            targets.pos_target_m.y - targets.offsets.pos_m.y,
        };
    }
    limit_vector = accel_target_mss;
}

inline void ne_stop_pos_stabilisation(NeStopTargets& targets, const NeEstimates& estimates) {
    targets.pos_target_m = estimates.pos_m;
    targets.pos_desired_m = {
        targets.pos_target_m.x - targets.offsets.pos_m.x,
        targets.pos_target_m.y - targets.offsets.pos_m.y,
    };
}

inline void ne_stop_vel_stabilisation(NeStopTargets& targets, const NeEstimates& estimates,
                                      pid::AcPid2d& vel_pid) {
    ne_stop_pos_stabilisation(targets, estimates);
    targets.vel_target_ms = estimates.vel_ms;
    targets.vel_desired_ms = targets.vel_target_ms - targets.offsets.vel_ms;
    vel_pid.reset_filter();
    vel_pid.reset_i();
}

[[nodiscard]] inline std::pair<float, float> yaw_from_ne_motion(math::Vector2f vel_desired_ms,
                                                                 math::Vector2f accel_desired_mss,
                                                                 float vel_max_ne_ms,
                                                                 float att_yaw_target_rad) {
    const float vel_len = vel_desired_ms.length();
    float turn_rate_rads = 0.0f;
    if (math::is_positive(vel_len)) {
        const float accel_forward = vel_desired_ms * accel_desired_mss / vel_len;
        const math::Vector2f accel_turn =
            accel_desired_mss - vel_desired_ms * (accel_forward / vel_len);
        turn_rate_rads = accel_turn.length() / vel_len;
        if (vel_desired_ms % accel_turn < 0.0f) {
            turn_rate_rads = -turn_rate_rads;
        }
    }

    if (vel_len > vel_max_ne_ms * 0.05f) {
        return {vel_desired_ms.angle(), turn_rate_rads};
    }
    return {att_yaw_target_rad, 0.0f};
}

}  // namespace fwcpp::poscontrol
