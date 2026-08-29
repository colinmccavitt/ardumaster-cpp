#pragma once

// CCP-027 slice 4: vector kinematic helpers for NE position shaping (Rust ap-math/control.rs xy half).

#include <algorithm>

#include <fwcpp/math/control.hpp>
#include <fwcpp/math/control_vector.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>

namespace fwcpp::math {

inline void update_vel_accel_xy(Vector2f& vel, Vector2f accel, float dt, Vector2f limit,
                                Vector2f vel_error) {
    Vector2f delta_vel = accel * dt;
    if (!limit.is_zero() && !delta_vel.is_zero() && is_positive(delta_vel * limit) &&
        is_positive(vel_error * limit) && !is_negative(vel * limit)) {
        delta_vel.zero();
    }
    vel += delta_vel;
}

inline void update_pos_vel_accel_xy(Vector2<postype_t>& pos, Vector2f& vel, Vector2f accel, float dt,
                                    Vector2f limit, Vector2f pos_error, Vector2f vel_error) {
    Vector2f delta_pos = vel * dt + accel * (0.5f * dt * dt);

    if (!is_zero(limit.length_squared()) && is_positive(delta_pos * limit) &&
        is_positive(pos_error * limit)) {
        delta_pos.zero();
    }

    pos.x += postype_t{delta_pos.x};
    pos.y += postype_t{delta_pos.y};

    update_vel_accel_xy(vel, accel, dt, limit, vel_error);
}

inline void shape_accel_xy(Vector2f accel_desired, Vector2f& accel, float jerk_max, float dt) {
    if (!is_positive(jerk_max)) {
        return;
    }
    if (is_positive(dt)) {
        Vector2f accel_delta = accel_desired - accel;
        accel_delta.limit_length(jerk_max * dt);
        accel += accel_delta;
    }
}

[[nodiscard]] inline bool limit_accel_corner_xy(Vector2f vel, Vector2f& accel, float accel_max) {
    if (!is_positive(accel_max)) {
        return false;
    }

    if (vel.is_zero()) {
        return accel.limit_length(accel_max);
    }

    accel.limit_length(2.0f * accel_max);

    const Vector2f vel_unit = vel / vel.length();
    float accel_dir_scalar = vel_unit * accel;
    Vector2f accel_dir = vel_unit * accel_dir_scalar;
    Vector2f accel_cross = accel - accel_dir;

    if (is_positive(accel_dir_scalar)) {
        const float accel_cross_mag = std::min(accel_cross.length(), accel_max);
        const float accel_along_max = safe_sqrt(accel_max * accel_max - accel_cross_mag * accel_cross_mag);

        accel_cross.limit_length(accel_max);
        accel_dir.limit_length(accel_along_max);

        accel = accel_cross + accel_dir;
        return true;
    }

    accel_dir_scalar = std::max(accel_dir_scalar, -accel_max);
    accel_dir = vel_unit * accel_dir_scalar;

    const float accel_cross_max = safe_sqrt(accel_max * accel_max - accel_dir_scalar * accel_dir_scalar);
    accel_cross.limit_length(accel_cross_max);

    accel = accel_cross + accel_dir;
    return true;
}

inline void shape_vel_accel_xy(Vector2f vel_desired, Vector2f accel_desired, Vector2f vel,
                               Vector2f& accel, float accel_max, float jerk_max, float dt,
                               bool limit_total_accel) {
    if (!is_positive(accel_max) || !is_positive(jerk_max)) {
        return;
    }

    const float kpa = jerk_max / accel_max;

    const Vector2f vel_error = vel_desired - vel;
    Vector2f accel_target = sqrt_controller_xy(vel_error, kpa, jerk_max, dt);

    limit_accel_corner_xy(vel, accel_target, accel_max);

    accel_target += accel_desired;

    if (limit_total_accel) {
        accel_target.limit_length(accel_max);
    }

    shape_accel_xy(accel_target, accel, jerk_max, dt);
}

inline void shape_pos_vel_accel_xy(Vector2<postype_t> pos_desired, Vector2f vel_desired,
                                   Vector2f accel_desired, Vector2<postype_t> pos, Vector2f vel,
                                   Vector2f& accel, float vel_max, float accel_max, float jerk_max,
                                   float dt, bool limit_total) {
    if (is_negative(vel_max) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        return;
    }

    const float k_v = jerk_max / accel_max;

    Vector2f vel_corr_cmd{};

    const Vector2f pos_error{
        static_cast<float>(pos_desired.x - pos.x),
        static_cast<float>(pos_desired.y - pos.y),
    };
    const float pos_error_length = pos_error.length();

    if (is_positive(pos_error_length)) {
        const float vel_corr_proj = (vel - vel_desired) * pos_error / pos_error_length;

        float vel_corr_cmd_length = sqrt_controller(pos_error_length, k_v, accel_max, dt);

        const float accel_corr_cmd_length =
            sqrt_controller_accel(pos_error_length, vel_corr_cmd_length, vel_corr_proj, k_v, accel_max);

        vel_corr_cmd_length += accel_corr_cmd_length / k_v;

        if (is_positive(vel_max)) {
            vel_corr_cmd_length = constrain_value(vel_corr_cmd_length, -vel_max, vel_max);
        }

        vel_corr_cmd = pos_error * (vel_corr_cmd_length / pos_error_length);
    }

    Vector2f vel_target = vel_desired + vel_corr_cmd;
    if (limit_total && is_positive(vel_max)) {
        vel_target.limit_length(vel_max);
    }

    Vector2f accel_target = (vel_target - vel) * k_v;

    limit_accel_corner_xy(vel, accel_target, accel_max);

    accel_target += accel_desired;

    if (limit_total) {
        accel_target.limit_length(accel_max);
    }

    shape_accel_xy(accel_target, accel, jerk_max, dt);
}

}  // namespace fwcpp::math
