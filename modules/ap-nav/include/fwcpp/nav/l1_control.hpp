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
// SLICE 2 adds update_loiter, update_heading_hold, update_level_flight -
// the three update_* variants deferred from slice 1. update_loiter needed
// Location::same_loc_as (added to CPP-011's Location alongside this) for
// its "keep _WPcircle latched" check, and math::cd_to_rad (added to
// CPP-004's scalar module alongside this) for update_heading_hold's
// centidegrees-to-radians conversion - upstream had both already; this
// port didn't need them until now.
//
// L1Inputs gained now_ms (upstream: AP_HAL::millis(), used only by
// update_loiter's 200ms latch window) alongside the pre-existing now_us
// (AP_HAL::micros(), used by update_waypoint's dt calc) - same explicit-
// parameter treatment, not derived from one another since upstream itself
// reads two independent clocks.
//
// SLICE BOUNDARY (now closed): nav_roll_cd, lateral_acceleration,
// nav_bearing_cd, bearing_error_cd, target_bearing_cd, turn_distance (both
// overloads), loiter_radius, reached_loiter_target, update_waypoint,
// update_loiter, update_heading_hold, update_level_flight - every public
// member of upstream AP_L1_Control is now ported.
//
// LITERAL SAFETY: GRAVITY_MSS (9.80665f) and every other literal touched
// in this slice are already explicitly float-suffixed upstream - nothing
// here needed the compiled-.cpp treatment scalar.cpp's wrap_* family or
// Location::get_bearing needed.

#include <algorithm>
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
    std::uint32_t now_us = 0;     // upstream: AP_HAL::micros(), used by update_waypoint's dt
    std::uint32_t now_ms = 0;     // upstream: AP_HAL::millis(), used by update_loiter's latch window
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
        last_loiter_.reached_loiter_target_ms = 0;

        bearing_error_ = nu;
        data_is_stale_ = false;
    }

    // L1-guided circular loiter around center_wp. Blends a "capture" law
    // (same L1 guidance as update_waypoint, pulling the aircraft toward
    // the circle) with a "circle" law (PD + centripetal, holding it on the
    // circle), switching between them at the point the two demands cross
    // so the transition is seamless rather than a mode-switch discontinuity.
    void update_loiter(const Location& center_wp, float radius, std::int8_t loiter_direction, const L1Inputs& in) {
        const float radius_unscaled = radius;
        radius = loiter_radius(std::fabs(radius), in);

        const float omega = 6.2832f / l1_period_;
        const float kx = omega * omega;
        const float kv = 2.0f * l1_damping_ * omega;
        const float k_l1 = 4.0f * l1_damping_ * l1_damping_;

        if (!in.location_valid) {
            data_is_stale_ = true;
            return;
        }
        const Location& current_loc = in.current_loc;
        const math::Vector2f& groundspeed_vector = in.groundspeed_vector;

        const float ground_speed = std::max(groundspeed_vector.length(), 1.0f);

        target_bearing_cd_ = current_loc.get_bearing_to(center_wp);

        l1_dist_ = 0.3183099f * l1_damping_ * l1_period_ * ground_speed;

        const math::Vector2f a_air = center_wp.get_distance_NE(current_loc);

        math::Vector2f a_air_unit;
        if (a_air.length() > 0.1f) {
            a_air_unit = a_air.normalized();
        } else if (groundspeed_vector.length() < 0.1f) {
            a_air_unit = math::Vector2f(std::cos(get_yaw(in)), std::sin(get_yaw(in)));
        } else {
            a_air_unit = groundspeed_vector.normalized();
        }

        const float xtrack_vel_cap = a_air_unit % groundspeed_vector;
        const float ltrack_vel_cap = -(groundspeed_vector * a_air_unit);
        float nu = std::atan2(xtrack_vel_cap, ltrack_vel_cap);

        prevent_indecision(nu, in);
        last_nu_ = nu;

        nu = math::constrain_value(nu, -1.5708f, 1.5708f);

        const float lat_acc_dem_cap = k_l1 * ground_speed * ground_speed / l1_dist_ * std::sin(nu);

        const float xtrack_vel_circ = -ltrack_vel_cap;
        const float xtrack_err_circ = a_air.length() - radius;

        crosstrack_error_ = xtrack_err_circ;

        float lat_acc_dem_circ_pd = xtrack_err_circ * kx + xtrack_vel_circ * kv;

        const float vel_tangent = xtrack_vel_cap * static_cast<float>(loiter_direction);

        if (ltrack_vel_cap < 0.0f && vel_tangent < 0.0f) {
            lat_acc_dem_circ_pd = std::max(lat_acc_dem_circ_pd, 0.0f);
        }

        const float lat_acc_dem_circ_ctr = vel_tangent * vel_tangent / std::max(0.5f * radius, radius + xtrack_err_circ);
        const float lat_acc_dem_circ = static_cast<float>(loiter_direction) * (lat_acc_dem_circ_pd + lat_acc_dem_circ_ctr);

        const std::uint32_t now_ms = in.now_ms;
        if (xtrack_err_circ > 0.0f
            && static_cast<float>(loiter_direction) * lat_acc_dem_cap < static_cast<float>(loiter_direction) * lat_acc_dem_circ) {
            lat_acc_dem_ = lat_acc_dem_cap;

            // See file banner / Location::same_loc_as: keeps _WPcircle
            // (wp_circle_) latched true across brief capture-mode blips
            // (a wind gust, an unachievable radius) rather than letting
            // reached_loiter_target() flicker false and back.
            if (wp_circle_ && last_loiter_.reached_loiter_target_ms != 0
                && now_ms - last_loiter_.reached_loiter_target_ms < 200U
                && loiter_direction == last_loiter_.direction
                && math::is_equal(radius_unscaled, last_loiter_.radius)
                && center_wp.same_loc_as(last_loiter_.center_wp)) {
                last_loiter_.reached_loiter_target_ms = now_ms;
            } else {
                wp_circle_ = false;
                last_loiter_.reached_loiter_target_ms = 0;
            }

            bearing_error_ = nu;
            nav_bearing_ = std::atan2(-a_air_unit.y, -a_air_unit.x);
        } else {
            lat_acc_dem_ = lat_acc_dem_circ;
            wp_circle_ = true;
            last_loiter_.reached_loiter_target_ms = now_ms;
            bearing_error_ = 0.0f;
            nav_bearing_ = std::atan2(-a_air_unit.y, -a_air_unit.x);
        }

        last_loiter_.radius = radius_unscaled;
        last_loiter_.direction = loiter_direction;
        last_loiter_.center_wp = center_wp;

        data_is_stale_ = false;
    }

    // Heading-hold navigation: track a commanded heading directly rather
    // than a waypoint line. Unlike update_waypoint/update_loiter, this
    // reads yaw_sensor_cd directly (NOT through get_yaw_sensor(in)'s
    // reverse-aware wrapper) - matching upstream's own choice to bypass
    // the reverse flag here.
    void update_heading_hold(std::int32_t navigation_heading_cd, const L1Inputs& in) {
        const float omega_a = 4.4428f / l1_period_; // sqrt(2)*pi/period

        target_bearing_cd_ = math::wrap_180_cd(navigation_heading_cd);
        nav_bearing_ = math::cd_to_rad(static_cast<float>(navigation_heading_cd));

        std::int32_t nu_cd = target_bearing_cd_ - math::wrap_180_cd(in.yaw_sensor_cd);
        nu_cd = math::wrap_180_cd(nu_cd);
        float nu = math::cd_to_rad(static_cast<float>(nu_cd));

        const float ground_speed = in.groundspeed_vector.length();

        l1_dist_ = ground_speed / omega_a;
        const float v_omega_a = ground_speed * omega_a;

        wp_circle_ = false;
        last_loiter_.reached_loiter_target_ms = 0;

        crosstrack_error_ = 0.0f;
        bearing_error_ = nu;

        nu = math::constrain_value(nu, -1.5708f, 1.5708f);
        lat_acc_dem_ = 2.0f * std::sin(nu) * v_omega_a;

        data_is_stale_ = false;
    }

    // Level flight on the current heading - no navigation demand at all.
    // Also bypasses the reverse-aware get_yaw()/get_yaw_sensor() helpers,
    // matching upstream's direct AHRS reads here.
    void update_level_flight(const L1Inputs& in) {
        target_bearing_cd_ = in.yaw_sensor_cd;
        nav_bearing_ = in.yaw_rad;
        bearing_error_ = 0.0f;
        crosstrack_error_ = 0.0f;

        wp_circle_ = false;
        last_loiter_.reached_loiter_target_ms = 0;

        lat_acc_dem_ = 0.0f;

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

    // Remembers the last update_loiter() decision, for the "keep _WPcircle
    // latched" check in update_loiter (see its body) - matches upstream's
    // anonymous _last_loiter struct.
    struct LastLoiter {
        std::uint32_t reached_loiter_target_ms = 0;
        float radius = 0.0f;
        std::int8_t direction = 1;
        Location center_wp;
    };
    LastLoiter last_loiter_;

    bool reverse_ = false;
};

} // namespace fwcpp::nav
