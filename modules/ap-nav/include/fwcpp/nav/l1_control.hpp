#pragma once

// Port of AP_L1_Control/AP_L1_Control.h + AP_L1_Control.cpp. CPP-017,
// slice 1. Written by Brandon Jones 2013, modified by Paul Riseborough.
//
// AP_AHRS&/AP_TECS* REPLACED WITH AN EXPLICIT L1Inputs STRUCT: upstream
// reaches into a stored `AP_AHRS&` (get_location, groundspeed_vector,
// get_yaw_rad, get_pitch_rad, yaw_sensor, get_EAS2TAS) and an optional
// `const AP_TECS*` (get_target_airspeed) on every update. Neither AP_AHRS
// nor AP_TECS exist in this port yet - building either is a substantial
// sub-effort of its own, and L1 does not need to wait on them: it only
// needs their OUTPUT values, not their internals. Matches this port's
// standing pattern (constrain_value's InternalError*, SlewLimiter's/
// AC_PID's now_ms) of taking external state as an explicit parameter
// instead of reaching for it - here scaled up to a small struct because
// there are several such values instead of one. `target_airspeed` and
// `location_valid` fold in what upstream gets from a possibly-null _tecs
// pointer and a possibly-failing _ahrs.get_location() call respectively.
//
// AP_HAL::micros() folded into L1Inputs.now_us for the same reason.
//
// AP_Float REPLACED WITH PLAIN float for the tunable gains (L1_period,
// L1_damping, L1_xtrack_i_gain, loiter_bank_limit) - same precedent as
// AC_PID's Gains struct, no AP_Param in this port yet.
//
// SLICE BOUNDARY: nav_roll_cd, lateral_acceleration, nav_bearing_cd,
// bearing_error_cd, target_bearing_cd, turn_distance (both overloads),
// loiter_radius, reached_loiter_target, update_waypoint (the core L1
// tracking algorithm) - the accessor surface plus the algorithm this
// effort's own scope is built around. Deliberately NOT in this slice:
// update_loiter, update_heading_hold, update_level_flight (three more
// update_* variants sharing update_waypoint's shape but with their own
// geometry) - tracked as follow-on work in CPP-017's notes, not silent.
//
// LITERAL SAFETY: GRAVITY_MSS (9.80665f) and every other literal touched
// in this slice are already explicitly float-suffixed upstream - nothing
// here needed the compiled-.cpp treatment scalar.cpp's wrap_* family or
// Location::get_bearing needed.

#include <cmath>
#include <cstdint>

#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>

namespace fwcpp::nav {

inline constexpr float kGravityMss = 9.80665f;

// Everything L1 needs from the AHRS/TECS for one update - see file banner.
struct L1Inputs {
    Location current_loc;
    bool location_valid = false; // upstream: _ahrs.get_location() return value
    math::Vector2f groundspeed_vector;
    float yaw_rad = 0.0f;         // upstream: _ahrs.get_yaw_rad()
    std::int32_t yaw_sensor_cd = 0; // upstream: _ahrs.yaw_sensor
    float pitch_rad = 0.0f;       // upstream: _ahrs.get_pitch_rad()
    float eas2tas = 1.0f;         // upstream: _ahrs.get_EAS2TAS()
    float target_airspeed = 0.0f; // upstream: _tecs->get_target_airspeed(), 0 if _tecs is null
    std::uint32_t now_us = 0;     // upstream: AP_HAL::micros()
};

class L1Control {
public:
    struct Gains {
        float l1_period = 25.0f;    // upstream NAVL1_PERIOD default
        float l1_damping = 0.75f;   // upstream NAVL1_DAMPING default
        float l1_xtrack_i_gain = 0.02f; // upstream NAVL1_XTRACK_I default (fixed-wing)
        float loiter_bank_limit = 0.0f; // upstream NAVL1_LIM_BANK default
    };

    explicit L1Control(const Gains& g)
        : l1_period_(g.l1_period), l1_damping_(g.l1_damping),
          l1_xtrack_i_gain_(g.l1_xtrack_i_gain), loiter_bank_limit_(g.loiter_bank_limit) {}

    L1Control(const L1Control&) = delete;
    L1Control& operator=(const L1Control&) = delete;

    void set_reverse(bool reverse) { reverse_ = reverse; }
    void set_default_period(float period) { l1_period_ = period; }
    void set_data_is_stale() { data_is_stale_ = true; }
    [[nodiscard]] bool data_is_stale() const { return data_is_stale_; }

    [[nodiscard]] float crosstrack_error() const { return crosstrack_error_; }
    [[nodiscard]] float crosstrack_error_integrator() const { return l1_xtrack_i_; }
    [[nodiscard]] float lateral_acceleration() const { return lat_acc_dem_; }
    [[nodiscard]] bool reached_loiter_target() const { return wp_circle_; }

    [[nodiscard]] std::int32_t nav_bearing_cd() const {
        return math::wrap_180_cd(math::rad_to_cd(nav_bearing_));
    }
    [[nodiscard]] std::int32_t bearing_error_cd() const { return math::rad_to_cd(bearing_error_); }
    [[nodiscard]] std::int32_t target_bearing_cd() const { return math::wrap_180_cd(target_bearing_cd_); }

    // Bank angle (centidegrees) to achieve the lateral acceleration demand
    // from the last update_*() call.
    [[nodiscard]] std::int32_t nav_roll_cd(const L1Inputs& in) const {
        const float pitch_lim = math::radians(60.0f);
        const float pitch = math::constrain_value(in.pitch_rad, -pitch_lim, pitch_lim);
        float ret = math::degrees(std::atan(lat_acc_dem_ * (1.0f / (kGravityMss * std::cos(pitch))))) * 100.0f;
        ret = math::constrain_value(ret, -9000.0f, 9000.0f);
        return static_cast<std::int32_t>(ret);
    }

    [[nodiscard]] float turn_distance(float wp_radius, const L1Inputs& in) const {
        wp_radius *= in.eas2tas * in.eas2tas;
        return std::min(wp_radius, l1_dist_);
    }

    [[nodiscard]] float turn_distance(float wp_radius, float turn_angle, const L1Inputs& in) const {
        const float distance_90 = turn_distance(wp_radius, in);
        turn_angle = std::fabs(turn_angle);
        if (turn_angle >= 90.0f) {
            return distance_90;
        }
        return distance_90 * turn_angle / 90.0f;
    }

    [[nodiscard]] float loiter_radius(float radius, const L1Inputs& in) const {
        const float sanitized_bank_limit = math::constrain_value(loiter_bank_limit_, 0.0f, 89.0f);
        const float lateral_accel_sea_level = std::tan(math::radians(sanitized_bank_limit)) * kGravityMss;
        const float nominal_velocity_sea_level = in.target_airspeed;
        const float eas2tas_sq = in.eas2tas * in.eas2tas;

        if (math::is_zero(sanitized_bank_limit) || math::is_zero(nominal_velocity_sea_level)
            || math::is_zero(lateral_accel_sea_level)) {
            return radius * eas2tas_sq;
        }
        const float sea_level_radius = (nominal_velocity_sea_level * nominal_velocity_sea_level) / lateral_accel_sea_level;
        if (sea_level_radius > radius) {
            return radius * eas2tas_sq;
        }
        return std::max(sea_level_radius * eas2tas_sq, radius);
    }

    // The core L1 waypoint-tracking update. Computes lateral_acceleration()
    // and nav_bearing_cd() for the caller to read afterward, matching
    // upstream's side-effecting update_*() shape.
    void update_waypoint(const Location& prev_wp, const Location& next_wp, const L1Inputs& in, float dist_min = 0.0f) {
        float dt = static_cast<float>(in.now_us - last_update_waypoint_us_) * 1.0e-6f;
        if (dt > 1.0f) {
            l1_xtrack_i_ = 0.0f;
        }
        if (dt > 0.1f) {
            dt = 0.1f;
        }
        last_update_waypoint_us_ = in.now_us;

        const float k_l1 = 4.0f * l1_damping_ * l1_damping_;

        if (!in.location_valid) {
            data_is_stale_ = true;
            return;
        }
        const Location& current_loc = in.current_loc;
        math::Vector2f groundspeed_vector = in.groundspeed_vector;

        target_bearing_cd_ = current_loc.get_bearing_to(next_wp);

        float ground_speed = groundspeed_vector.length();

        const bool moving_forwards =
            std::fabs(math::wrap_PI(groundspeed_vector.angle() - get_yaw(in))) < static_cast<float>(M_PI_2);

        if (ground_speed < 0.1f || !moving_forwards) {
            ground_speed = 0.1f;
            groundspeed_vector = math::Vector2f(std::cos(get_yaw(in)), std::sin(get_yaw(in))) * ground_speed;
        }

        l1_dist_ = std::max(0.3183099f * l1_damping_ * l1_period_ * ground_speed, dist_min);

        math::Vector2f ab = prev_wp.get_distance_NE(next_wp);
        const float ab_length = ab.length();

        if (ab.length() < 1.0e-6f) {
            ab = current_loc.get_distance_NE(next_wp);
            if (ab.length() < 1.0e-6f) {
                ab = math::Vector2f(std::cos(get_yaw(in)), std::sin(get_yaw(in)));
            }
        }
        ab.normalize();

        const math::Vector2f a_air = prev_wp.get_distance_NE(current_loc);
        crosstrack_error_ = a_air % ab;

        const float wp_a_dist = a_air.length();
        const float along_track_dist = a_air * ab;

        float nu;
        if (wp_a_dist > l1_dist_ && along_track_dist / std::max(wp_a_dist, 1.0f) < -0.7071f) {
            const math::Vector2f a_air_unit = a_air.normalized();
            const float xtrack_vel = groundspeed_vector % (-a_air_unit);
            const float ltrack_vel = groundspeed_vector * (-a_air_unit);
            nu = std::atan2(xtrack_vel, ltrack_vel);
            nav_bearing_ = std::atan2(-a_air_unit.y, -a_air_unit.x);
        } else if (along_track_dist > ab_length + ground_speed * 3.0f) {
            const math::Vector2f b_air = next_wp.get_distance_NE(current_loc);
            const math::Vector2f b_air_unit = b_air.normalized();
            const float xtrack_vel = groundspeed_vector % (-b_air_unit);
            const float ltrack_vel = groundspeed_vector * (-b_air_unit);
            nu = std::atan2(xtrack_vel, ltrack_vel);
            nav_bearing_ = std::atan2(-b_air_unit.y, -b_air_unit.x);
        } else {
            const float xtrack_vel = groundspeed_vector % ab;
            const float ltrack_vel = groundspeed_vector * ab;
            const float nu2 = std::atan2(xtrack_vel, ltrack_vel);

            float sine_nu1 = crosstrack_error_ / std::max(l1_dist_, 0.1f);
            sine_nu1 = math::constrain_value(sine_nu1, -0.7071f, 0.7071f);
            const float nu1_base = std::asin(sine_nu1);
            float nu1 = nu1_base;

            if (l1_xtrack_i_gain_ <= 0.0f || !math::is_equal(l1_xtrack_i_gain_, l1_xtrack_i_gain_prev_)) {
                l1_xtrack_i_ = 0.0f;
                l1_xtrack_i_gain_prev_ = l1_xtrack_i_gain_;
            } else if (std::fabs(nu1) < math::radians(5.0f)) {
                l1_xtrack_i_ += nu1 * l1_xtrack_i_gain_ * dt;
                l1_xtrack_i_ = math::constrain_value(l1_xtrack_i_, -0.1f, 0.1f);
            }

            nu1 += l1_xtrack_i_;
            nu = nu1 + nu2;
            nav_bearing_ = math::wrap_PI(std::atan2(ab.y, ab.x) + nu1);
        }

        prevent_indecision(nu, in);
        last_nu_ = nu;

        nu = math::constrain_value(nu, -1.5708f, 1.5708f);
        lat_acc_dem_ = k_l1 * ground_speed * ground_speed / l1_dist_ * std::sin(nu);

        wp_circle_ = false;
        last_loiter_reached_ms_ = 0;

        bearing_error_ = nu;
        data_is_stale_ = false;
    }

private:
    [[nodiscard]] float get_yaw(const L1Inputs& in) const {
        if (reverse_) {
            return math::wrap_PI(static_cast<float>(M_PI) + in.yaw_rad);
        }
        return in.yaw_rad;
    }

    [[nodiscard]] std::int32_t get_yaw_sensor(const L1Inputs& in) const {
        if (reverse_) {
            return math::wrap_180_cd(18000 + in.yaw_sensor_cd);
        }
        return in.yaw_sensor_cd;
    }

    void prevent_indecision(float& nu, const L1Inputs& in) {
        constexpr float kNuLimit = 0.9f * static_cast<float>(M_PI);
        if (std::fabs(nu) > kNuLimit && std::fabs(last_nu_) > kNuLimit
            && std::abs(math::wrap_180_cd(target_bearing_cd_ - get_yaw_sensor(in))) > 12000
            && nu * last_nu_ < 0.0f) {
            nu = last_nu_;
        }
    }

    float l1_period_;
    float l1_damping_;
    float l1_xtrack_i_gain_;
    float loiter_bank_limit_;

    float lat_acc_dem_ = 0.0f;
    float l1_dist_ = 0.0f;
    bool wp_circle_ = false;
    float nav_bearing_ = 0.0f;
    float bearing_error_ = 0.0f;
    float crosstrack_error_ = 0.0f;
    std::int32_t target_bearing_cd_ = 0;
    float last_nu_ = 0.0f;
    float l1_xtrack_i_ = 0.0f;
    float l1_xtrack_i_gain_prev_ = 0.0f;
    std::uint32_t last_update_waypoint_us_ = 0;
    bool data_is_stale_ = true;
    std::uint32_t last_loiter_reached_ms_ = 0;
    bool reverse_ = false;
};

} // namespace fwcpp::nav
