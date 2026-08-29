#pragma once

#include <utility>
// CCP-027 slice 2: NE_update_controller PID path + yaw_from_ne_motion.
// Rust spec: plane-fw-rust pos_control_ne.rs (update_controller, yaw_from_ne_motion).

#include <fwcpp/math/control_vector.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/pid/ac_p_2d.hpp>
#include <fwcpp/pid/ac_pid_2d.hpp>
#include <fwcpp/poscontrol/pos_control_lean.hpp>

namespace fwcpp::poscontrol {

inline constexpr float kNePosP = 1.0f;

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
