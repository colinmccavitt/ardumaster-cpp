#pragma once

// CCP-027 slice 7: 3D input_pos_NED_m path shaper (terrain-aware NE+D shaping).
// Upstream: AC_PosControl::input_pos_NED_m, terrain_scaler_D_m, calculate_overspeed_gain.

#include <algorithm>
#include <cmath>

#include <fwcpp/math/control.hpp>
#include <fwcpp/math/control_vector_kinematic.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/poscontrol/pos_control_d.hpp>
#include <fwcpp/poscontrol/pos_control_ne.hpp>

namespace fwcpp::poscontrol {

[[nodiscard]] inline float kinematic_limit(float segment_length_xy, float segment_length_z,
                                           float max_xy, float max_z_neg, float max_z_pos) {
    if (math::is_zero(max_xy) || math::is_zero(max_z_pos) || math::is_zero(max_z_neg)) {
        return 0.0f;
    }

    max_xy = std::fabs(max_xy);
    max_z_pos = std::fabs(max_z_pos);
    max_z_neg = std::fabs(max_z_neg);

    const float length =
        math::safe_sqrt(segment_length_xy * segment_length_xy + segment_length_z * segment_length_z);
    if (!math::is_positive(length)) {
        return 0.0f;
    }
    segment_length_xy /= length;
    segment_length_z /= length;

    if (math::is_zero(segment_length_xy)) {
        return math::is_positive(segment_length_z) ? max_z_pos : max_z_neg;
    }
    if (math::is_zero(segment_length_z)) {
        return max_xy;
    }

    const float slope = segment_length_z / segment_length_xy;
    if (math::is_positive(slope)) {
        if (std::fabs(slope) < max_z_pos / max_xy) {
            return max_xy / segment_length_xy;
        }
        return std::fabs(max_z_pos / segment_length_z);
    }

    if (std::fabs(slope) < max_z_neg / max_xy) {
        return max_xy / segment_length_xy;
    }
    return std::fabs(max_z_neg / segment_length_z);
}

[[nodiscard]] inline float kinematic_limit(const math::Vector3f& direction, float max_xy,
                                           float max_z_neg, float max_z_pos) {
    if (math::is_zero(direction.length_squared())) {
        return 0.0f;
    }
    return kinematic_limit(direction.xy().length(), direction.z, max_xy, max_z_neg, max_z_pos);
}

[[nodiscard]] inline float calculate_overspeed_gain(float vel_desired_d_ms, const DLimits& limits) {
    return calculate_d_overspeed_gain(vel_desired_d_ms, limits.vel_max_down_ms, limits.vel_max_up_ms);
}

[[nodiscard]] inline float terrain_scaler_d_m(float pos_estimate_d_m, float pos_target_d_m,
                                              float pos_terrain_d_m, float pos_terrain_target_d_m,
                                              float terrain_margin_m) {
    if (math::is_zero(terrain_margin_m)) {
        return 1.0f;
    }
    const float pos_offset_error_d_m =
        pos_estimate_d_m - (pos_target_d_m + (pos_terrain_target_d_m - pos_terrain_d_m));
    return math::constrain_value(
        (1.0f - (std::fabs(pos_offset_error_d_m) - 0.5f * terrain_margin_m) /
                    (0.5f * terrain_margin_m)),
        0.01f, 1.0f);
}

inline void set_pos_terrain_target_d_m(DTerrain& terrain, float pos_terrain_target_d_m) {
    terrain.pos_target_m = math::postype_t{pos_terrain_target_d_m};
}

inline void set_pos_terrain_target_u_cm(DTerrain& terrain, float pos_terrain_target_u_cm) {
    set_pos_terrain_target_d_m(terrain, -pos_terrain_target_u_cm * 0.01f);
}

struct InputPosNedPathPidErrors {
    math::Vector2f pos_p_ne{};
    math::Vector2f vel_pid_ne{};
    float pos_p_d = 0.0f;
    float vel_pid_d = 0.0f;
};

struct InputPosNedPathContext {
    float dt = 0.0f;
    float pos_estimate_d_m = 0.0f;
    float pos_target_d_m = 0.0f;
    float vel_max_ne_ms = 0.0f;
    NeLimits ne_limits{};
    DLimits d_limits{};
    InputPosNedPathPidErrors pid_errors{};
};

/// Terrain-aware 3D position path shaper, upstream AC_PosControl::input_pos_NED_m.
inline void input_pos_ned_m(math::Vector3<math::postype_t>& pos_ned_m,
                            float pos_terrain_target_d_m, float terrain_margin_m, PosControlNe& ne,
                            PosControlD& d, DTerrain& terrain, const InputPosNedPathContext& ctx) {
    const float offset_d_scalar = terrain_scaler_d_m(
        ctx.pos_estimate_d_m, ctx.pos_target_d_m, static_cast<float>(terrain.pos_m),
        pos_terrain_target_d_m, terrain_margin_m);
    set_pos_terrain_target_d_m(terrain, pos_terrain_target_d_m);

    const float overspeed_gain = calculate_overspeed_gain(d.vel_desired_ms, ctx.d_limits);
    const float accel_max_d_mss = ctx.d_limits.accel_max_d_mss * overspeed_gain;
    const float jerk_max_d_msss = ctx.d_limits.jerk_max_d_msss * overspeed_gain;

    math::update_pos_vel_accel_xy(ne.pos_desired_m, ne.vel_desired_ms, ne.accel_desired_mss, ctx.dt,
                                  ne.limit_vector, ctx.pid_errors.pos_p_ne, ctx.pid_errors.vel_pid_ne);

    math::update_pos_vel_accel(d.pos_desired_m, d.vel_desired_ms, d.accel_desired_mss, ctx.dt,
                               d.limit_vector, ctx.pid_errors.pos_p_d, ctx.pid_errors.vel_pid_d);

    const math::Vector3<math::postype_t> pos_desired_ned{ne.pos_desired_m.x, ne.pos_desired_m.y,
                                                         d.pos_desired_m};
    math::Vector3f travel_dir_unit = (pos_ned_m - pos_desired_ned).tofloat();
    float vel_max_ne_ms = 0.0f;
    float vel_max_d_ms = 0.0f;
    if (math::is_positive(travel_dir_unit.length_squared())) {
        travel_dir_unit.normalize();
        const float travel_dir_unit_ne_length = travel_dir_unit.xy().length();
        const float vel_max_ms =
            kinematic_limit(travel_dir_unit, ctx.vel_max_ne_ms, ctx.d_limits.vel_max_up_ms,
                            ctx.d_limits.vel_max_down_ms);
        vel_max_ne_ms = vel_max_ms * travel_dir_unit_ne_length;
        vel_max_d_ms = std::fabs(vel_max_ms * travel_dir_unit.z);
    }

    vel_max_ne_ms *= offset_d_scalar;

    math::Vector2<math::postype_t> pos_ne_m{pos_ned_m.x, pos_ned_m.y};
    math::Vector2f vel_ne_ms{};
    math::Vector2f accel_ne_mss{};
    math::shape_pos_vel_accel_xy(pos_ne_m, vel_ne_ms, accel_ne_mss, ne.pos_desired_m,
                                 ne.vel_desired_ms, ne.accel_desired_mss, vel_max_ne_ms,
                                 ctx.ne_limits.accel_max_ne_mss, ctx.ne_limits.jerk_max_ne_msss,
                                 ctx.dt, false);

    float pos_d_m = static_cast<float>(pos_ned_m.z);
    math::shape_pos_vel_accel(
        pos_d_m, 0.0f, 0.0f, d.pos_desired_m, d.vel_desired_ms, d.accel_desired_mss,
        -vel_max_d_ms, vel_max_d_ms, -accel_max_d_mss,
        math::constrain_value(accel_max_d_mss, 0.0f, 7.5f), jerk_max_d_msss, ctx.dt, false);

    pos_ned_m.x = pos_ne_m.x;
    pos_ned_m.y = pos_ne_m.y;
    pos_ned_m.z = math::postype_t{pos_d_m};
}

}  // namespace fwcpp::poscontrol
