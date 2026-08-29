#pragma once

// Port of libraries/AC_WPNav/AC_WPNav (Copter-4.7.0 / plane-4.7.0) — CCP-028.
// Rust spec: ports/plane-fw-rust/crates/ap-wpnav/src/wpnav.rs.
//
// THIS SLICE (testable):
//   WpNav construction (GroupInfo defaults without AP_Param)
//   wp_and_spline_init_m (upstream enable/init collapsed in 4.7)
//   calc_scurve_jerk_and_snap (attitude limits as injected inputs)
//   is_active, horizontal distance/bearing helpers (pos estimate injected)
//   set_wp_destination_NED_m / set_wp_destination_NEU_cm
//   update_wpnav (speed-param watch + advance/NE-controller leftovers)
//   set_speed_NE/up/down_ms
//
// LEFTOVER / DEFERRED (future CCP-028 slices — do not assume present):
//   convert_parameters, var_info / AP_Param glue
//   set_spline_destination_*, set_wp_destination_next_*, force_stop_at_next_wp
//   advance_wp_target_along_track, update_track_with_speed_accel_limits
//   terrain get_terrain_* / rangefinder, Location wrappers
//   get_vector_NED from Location, get_wp_stopping_point_* wrappers
//   SCurve / SplineCurve object calls (live in ap-math when ported)
//   AC_PosControl method calls (PosControlSpeedAccel recorded only)
//   AC_WPNav_OA.{h,cpp} — obstacle avoidance variant, separate scope
//   AC_Loiter → loiter.hpp ; AC_Circle → circle.hpp
//
// Parity tests still to port (Rust): advance_wp_target.rs, set_spline_*.rs,
// wpnav_leftover.rs
//
// ADR-0004: no AHRS / PosControl / HAL millis singletons — caller supplies
// stopping point, attitude jerk inputs, now_ms, and pos estimate for queries.

#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::wpnav {

inline constexpr float kWpnavAccelerationMss = 2.5f;
inline constexpr float kWpSpdDefault = 10.0f;
inline constexpr float kWpSpdMin = 0.01f;
inline constexpr float kWpRadiusMDefault = 2.0f;
inline constexpr float kWpRadiusMMin = 0.05f;
inline constexpr float kWpSpdUpDefault = 2.5f;
inline constexpr float kWpSpdDownDefault = 1.5f;
inline constexpr float kWpAccZDefault = 1.0f;
inline constexpr float kWpJerkDefault = 1.0f;
inline constexpr float kTerrainMarginDefaultM = 10.0f;
inline constexpr std::uint32_t kWpnavActiveTimeoutMs = 200;
inline constexpr float kGravityMss = 9.80665f;

struct AttitudeJerkLimits {
    float ang_vel_roll_max_rads{0.0f};
    float ang_vel_pitch_max_rads{0.0f};
    float accel_roll_max_radss{0.0f};
    float accel_pitch_max_radss{0.0f};
    float input_tc{0.0f};
};

struct PosControlSpeedAccel {
    float ne_speed_ms{0.0f};
    float ne_accel_mss{0.0f};
    float speed_down_ms{0.0f};
    float speed_up_ms{0.0f};
    float accel_d_mss{0.0f};
};

struct SetWpDestinationContext {
    std::uint32_t now_ms{0};
    AttitudeJerkLimits attitude{};
    math::Vector3<float> stopping_point_ned_m{};
    std::optional<float> terrain_d_m{};
};

struct UpdateWpNavContext {
    std::uint32_t now_ms{0};
    float dt_s{0.01f};
    std::optional<float> terrain_d_m{};
};

struct UpdateWpNavLeftover {
    bool applied_speed_ne{false};
    bool applied_speed_up{false};
    bool applied_speed_down{false};
    bool need_update_track_limits{false};
    bool need_advance_track{true};
    bool need_ne_update_controller{true};
    bool advance_ok{true};
    float dt_s{0.0f};
};

struct WpNavFlags {
    bool reached_destination{false};
    bool fast_waypoint{false};
    bool wp_yaw_set{false};
};

class WpNav {
public:
    WpNav() {
        const float wp_speed_ms = kWpSpdDefault;
        const float wp_speed_up_ms = kWpSpdUpDefault;
        const float wp_speed_down_ms = kWpSpdDownDefault;
        wp_speed_ms_ = wp_speed_ms;
        wp_speed_up_ms_ = wp_speed_up_ms;
        wp_speed_down_ms_ = wp_speed_down_ms;
        wp_radius_m_ = kWpRadiusMDefault;
        wp_accel_mss_ = kWpnavAccelerationMss;
        wp_accel_z_mss_ = kWpAccZDefault;
        wp_jerk_msss_ = kWpJerkDefault;
        terrain_margin_m_ = kTerrainMarginDefaultM;
        last_wp_speed_ms_ = wp_speed_ms;
        last_wp_speed_up_ms_ = wp_speed_up_ms;
        last_wp_speed_down_ms_ = std::fabs(wp_speed_down_ms);
        flags_.reached_destination = false;
        flags_.fast_waypoint = false;
        rangefinder_use_ = true;
    }

    [[nodiscard]] float default_speed_ne_ms() const { return wp_speed_ms_; }
    [[nodiscard]] float default_speed_up_ms() const { return wp_speed_up_ms_; }
    [[nodiscard]] float default_speed_down_ms() const { return std::fabs(wp_speed_down_ms_); }

    [[nodiscard]] float wp_acceleration_mss() const {
        return math::is_positive(wp_accel_mss_) ? wp_accel_mss_ : kWpnavAccelerationMss;
    }

    [[nodiscard]] float accel_d_mss() const { return wp_accel_z_mss_; }
    [[nodiscard]] float wp_radius_m() const { return wp_radius_m_; }
    [[nodiscard]] WpNavFlags flags() const { return flags_; }
    [[nodiscard]] bool scurve_legs_inited() const { return scurve_legs_inited_; }
    [[nodiscard]] bool pos_control_stopping_point_inited() const { return pos_control_stopping_point_inited_; }
    [[nodiscard]] float desired_speed_ne_ms() const { return wp_desired_speed_ne_ms_; }
    [[nodiscard]] bool check_wp_speed_change() const { return check_wp_speed_change_; }
    [[nodiscard]] math::Vector3<float> wp_destination_ned_m() const { return destination_ned_m_; }
    [[nodiscard]] math::Vector3<float> wp_origin_ned_m() const { return origin_ned_m_; }
    [[nodiscard]] math::Vector3<float> wp_destination_neu_cm() const {
        return math::Vector3<float>{destination_ned_m_.x * 100.0f, destination_ned_m_.y * 100.0f,
                                    -destination_ned_m_.z * 100.0f};
    }
    [[nodiscard]] math::Vector3<float> wp_origin_neu_cm() const {
        return math::Vector3<float>{origin_ned_m_.x * 100.0f, origin_ned_m_.y * 100.0f,
                                    -origin_ned_m_.z * 100.0f};
    }
    [[nodiscard]] math::Vector3<float> next_destination_ned_m() const { return next_destination_ned_m_; }
    [[nodiscard]] bool reached_wp_destination() const { return flags_.reached_destination; }
    [[nodiscard]] bool origin_and_destination_are_terrain_alt() const { return is_terrain_alt_; }
    [[nodiscard]] bool this_leg_is_spline() const { return this_leg_is_spline_; }
    [[nodiscard]] bool next_leg_is_spline() const { return next_leg_is_spline_; }
    [[nodiscard]] bool paused() const { return paused_; }
    [[nodiscard]] float track_dt_scalar() const { return track_dt_scalar_; }
    [[nodiscard]] float offset_vel_ms() const { return offset_vel_ms_; }
    [[nodiscard]] float offset_accel_mss() const { return offset_accel_mss_; }
    [[nodiscard]] float scurve_jerk_max_msss() const { return scurve_jerk_max_msss_; }
    [[nodiscard]] float scurve_snap_max_mssss() const { return scurve_snap_max_mssss_; }
    [[nodiscard]] PosControlSpeedAccel pos_speed_accel() const { return pos_speed_accel_; }
    [[nodiscard]] bool scurve_this_leg_calculated() const { return scurve_this_leg_calculated_; }
    [[nodiscard]] float pos_terrain_d_m() const { return pos_terrain_d_m_; }
    [[nodiscard]] float last_arc_rad() const { return last_arc_rad_; }
    [[nodiscard]] bool scurve_next_leg_calculated() const { return scurve_next_leg_calculated_; }
    [[nodiscard]] float last_next_arc_rad() const { return last_next_arc_rad_; }
    [[nodiscard]] float last_wp_speed_ms() const { return last_wp_speed_ms_; }
    [[nodiscard]] float last_wp_speed_up_ms() const { return last_wp_speed_up_ms_; }
    [[nodiscard]] float last_wp_speed_down_ms() const { return last_wp_speed_down_ms_; }

    void set_wp_radius_m(float radius_m) { wp_radius_m_ = radius_m; }
    void set_wp_speed_ms(float speed_ms) { wp_speed_ms_ = speed_ms; }
    void set_wp_speed_up_ms(float speed_up_ms) { wp_speed_up_ms_ = speed_up_ms; }
    void set_wp_speed_down_ms(float speed_down_ms) { wp_speed_down_ms_ = speed_down_ms; }
    void set_wp_jerk_msss(float jerk_msss) { wp_jerk_msss_ = jerk_msss; }
    void set_wp_accel_mss(float accel_mss) { wp_accel_mss_ = accel_mss; }

    void wp_and_spline_init_m(float speed_ms, const math::Vector3<float>& stopping_point_ned_m,
                              std::uint32_t now_ms, AttitudeJerkLimits attitude) {
        wp_radius_m_ = std::max(wp_radius_m_, kWpRadiusMMin);
        wp_speed_ms_ = std::max(wp_speed_ms_, kWpSpdMin);

        pos_control_stopping_point_inited_ = true;

        check_wp_speed_change_ = !math::is_positive(speed_ms);
        wp_desired_speed_ne_ms_ = math::is_positive(speed_ms) ? speed_ms : default_speed_ne_ms();
        wp_desired_speed_ne_ms_ = std::max(wp_desired_speed_ne_ms_, kWpSpdMin);

        pos_speed_accel_ = PosControlSpeedAccel{
            wp_desired_speed_ne_ms_,
            wp_acceleration_mss(),
            default_speed_down_ms(),
            default_speed_up_ms(),
            accel_d_mss(),
        };

        if (!math::is_positive(wp_jerk_msss_)) {
            wp_jerk_msss_ = wp_acceleration_mss();
        }
        calc_scurve_jerk_and_snap(attitude);

        scurve_legs_inited_ = true;
        track_dt_scalar_ = 1.0f;

        flags_.reached_destination = true;
        flags_.fast_waypoint = false;

        origin_ned_m_ = stopping_point_ned_m;
        destination_ned_m_ = stopping_point_ned_m;
        is_terrain_alt_ = false;
        this_leg_is_spline_ = false;
        next_leg_is_spline_ = false;
        next_destination_ned_m_ = math::Vector3<float>{};
        scurve_this_leg_calculated_ = false;
        pos_terrain_d_m_ = 0.0f;
        last_arc_rad_ = 0.0f;
        spline_this_leg_set_ = false;
        spline_origin_vel_ned_ms_ = math::Vector3<float>{};
        spline_destination_vel_ned_ms_ = math::Vector3<float>{};
        spline_next_leg_set_ = false;
        spline_next_destination_ned_m_ = math::Vector3<float>{};
        spline_next_origin_vel_ned_ms_ = math::Vector3<float>{};
        spline_next_destination_vel_ned_ms_ = math::Vector3<float>{};
        need_this_leg_dest_speed_max_ = false;
        scurve_next_leg_calculated_ = false;
        last_next_arc_rad_ = 0.0f;
        need_this_leg_dest_speed_max_zero_ = false;
        need_next_scurve_init_ = false;

        offset_vel_ms_ = wp_desired_speed_ne_ms_;
        offset_accel_mss_ = 0.0f;
        paused_ = false;

        wp_last_update_ms_ = now_ms;
    }

    [[nodiscard]] bool set_wp_destination_neu_cm(const math::Vector3<float>& destination_neu_cm,
                                                   bool is_terrain_alt, SetWpDestinationContext ctx) {
        const math::Vector3<float> destination_ned_m{destination_neu_cm.x * 0.01f, destination_neu_cm.y * 0.01f,
                                                     -destination_neu_cm.z * 0.01f};
        return set_wp_destination_ned_m(destination_ned_m, is_terrain_alt, 0.0f, ctx);
    }

    [[nodiscard]] bool set_wp_destination_ned_m(const math::Vector3<float>& destination_ned_m,
                                                bool is_terrain_alt, float arc_rad,
                                                SetWpDestinationContext ctx) {
        if (!is_active(ctx.now_ms) || !flags_.reached_destination) {
            wp_and_spline_init_m(wp_desired_speed_ne_ms_, ctx.stopping_point_ned_m, ctx.now_ms, ctx.attitude);
        }

        origin_ned_m_ = destination_ned_m_;

        if (is_terrain_alt == is_terrain_alt_) {
            // Matching frame: SCurve / SplineCurve leftovers stay in ap-math.
        } else {
            if (!ctx.terrain_d_m.has_value()) {
                return false;
            }
            const float terrain_d_m = *ctx.terrain_d_m;
            if (is_terrain_alt) {
                origin_ned_m_.z -= terrain_d_m;
                pos_terrain_d_m_ = terrain_d_m;
            } else {
                origin_ned_m_.z += terrain_d_m;
                pos_terrain_d_m_ = 0.0f;
            }
        }

        destination_ned_m_ = destination_ned_m;
        is_terrain_alt_ = is_terrain_alt;

        scurve_this_leg_calculated_ = true;
        last_arc_rad_ = arc_rad;

        this_leg_is_spline_ = false;
        next_leg_is_spline_ = false;
        next_destination_ned_m_ = math::Vector3<float>{};
        spline_this_leg_set_ = false;
        spline_origin_vel_ned_ms_ = math::Vector3<float>{};
        spline_destination_vel_ned_ms_ = math::Vector3<float>{};
        spline_next_leg_set_ = false;
        spline_next_destination_ned_m_ = math::Vector3<float>{};
        spline_next_origin_vel_ned_ms_ = math::Vector3<float>{};
        spline_next_destination_vel_ned_ms_ = math::Vector3<float>{};
        need_this_leg_dest_speed_max_ = false;
        scurve_next_leg_calculated_ = false;
        last_next_arc_rad_ = 0.0f;
        need_this_leg_dest_speed_max_zero_ = false;
        need_next_scurve_init_ = false;
        flags_.fast_waypoint = false;
        flags_.reached_destination = false;

        return true;
    }

    [[nodiscard]] UpdateWpNavLeftover update_wpnav(UpdateWpNavContext ctx) {
        UpdateWpNavLeftover leftover{};
        leftover.need_advance_track = true;
        leftover.need_ne_update_controller = true;
        leftover.advance_ok = true;
        leftover.dt_s = ctx.dt_s;

        if (check_wp_speed_change_ && !math::is_equal(wp_speed_ms_, last_wp_speed_ms_)) {
            leftover.applied_speed_ne = set_speed_ne_ms(default_speed_ne_ms());
            last_wp_speed_ms_ = wp_speed_ms_;
            leftover.need_update_track_limits = leftover.need_update_track_limits || leftover.applied_speed_ne;
        }

        if (!math::is_equal(wp_speed_up_ms_, last_wp_speed_up_ms_)) {
            set_speed_up_ms(default_speed_up_ms());
            last_wp_speed_up_ms_ = wp_speed_up_ms_;
            leftover.applied_speed_up = true;
            leftover.need_update_track_limits = true;
        }
        if (!math::is_equal(wp_speed_down_ms_, last_wp_speed_down_ms_)) {
            set_speed_down_ms(default_speed_down_ms());
            last_wp_speed_down_ms_ = wp_speed_down_ms_;
            leftover.applied_speed_down = true;
            leftover.need_update_track_limits = true;
        }

        if (is_terrain_alt_) {
            leftover.advance_ok = ctx.terrain_d_m.has_value();
        }

        wp_last_update_ms_ = ctx.now_ms;
        return leftover;
    }

    [[nodiscard]] bool set_speed_ne_ms(float speed_ms) {
        if (speed_ms >= kWpSpdMin && math::is_positive(wp_desired_speed_ne_ms_)) {
            offset_vel_ms_ = speed_ms * offset_vel_ms_ / wp_desired_speed_ne_ms_;
            wp_desired_speed_ne_ms_ = speed_ms;
            pos_speed_accel_.ne_speed_ms = wp_desired_speed_ne_ms_;
            pos_speed_accel_.ne_accel_mss = wp_acceleration_mss();
            return true;
        }
        return false;
    }

    void set_speed_up_ms(float speed_up_ms) { pos_speed_accel_.speed_up_ms = speed_up_ms; }

    void set_speed_down_ms(float speed_down_ms) { pos_speed_accel_.speed_down_ms = speed_down_ms; }

    [[nodiscard]] bool is_active(std::uint32_t now_ms) const {
        return (now_ms - wp_last_update_ms_) < kWpnavActiveTimeoutMs;
    }

    [[nodiscard]] float get_wp_distance_to_destination_m(const math::Vector3<float>& pos_estimate_ned_m) const {
        const math::Vector2<float> delta = pos_estimate_ned_m.xy() - destination_ned_m_.xy();
        return delta.length();
    }

    [[nodiscard]] float get_wp_distance_to_destination_cm(const math::Vector3<float>& pos_estimate_ned_m) const {
        return get_wp_distance_to_destination_m(pos_estimate_ned_m) * 100.0f;
    }

    [[nodiscard]] float get_wp_bearing_to_destination_rad(const math::Vector3<float>& pos_estimate_ned_m) const {
        return bearing_rad(pos_estimate_ned_m.xy(), destination_ned_m_.xy());
    }

    [[nodiscard]] std::int32_t get_wp_bearing_to_destination_cd(const math::Vector3<float>& pos_estimate_ned_m) const {
        return static_cast<std::int32_t>(math::rad_to_cd(get_wp_bearing_to_destination_rad(pos_estimate_ned_m)) + 0.5f);
    }

private:
    static float bearing_rad(const math::Vector2<float>& origin, const math::Vector2<float>& destination) {
        return math::wrap_2PI(std::atan2(destination.y - origin.y, destination.x - origin.x));
    }

    void calc_scurve_jerk_and_snap(AttitudeJerkLimits attitude) {
        scurve_jerk_max_msss_ = std::min(attitude.ang_vel_roll_max_rads * kGravityMss,
                                         attitude.ang_vel_pitch_max_rads * kGravityMss);
        if (math::is_zero(scurve_jerk_max_msss_)) {
            scurve_jerk_max_msss_ = wp_jerk_msss_;
        } else {
            scurve_jerk_max_msss_ = std::min(scurve_jerk_max_msss_, wp_jerk_msss_);
        }

        const float tc = std::max(attitude.input_tc, 0.1f);
        scurve_snap_max_mssss_ = (scurve_jerk_max_msss_ * std::numbers::pi_v<float>) / (2.0f * tc);

        const float snap = std::min(attitude.accel_roll_max_radss, attitude.accel_pitch_max_radss) * kGravityMss;
        if (math::is_positive(snap)) {
            scurve_snap_max_mssss_ = std::min(scurve_snap_max_mssss_, snap);
        }

        scurve_snap_max_mssss_ *= 0.5f;
    }

    float wp_speed_ms_{0.0f};
    float wp_speed_up_ms_{0.0f};
    float wp_speed_down_ms_{0.0f};
    float wp_radius_m_{0.0f};
    float wp_accel_mss_{0.0f};
    float wp_accel_c_mss_{0.0f};
    float wp_accel_z_mss_{0.0f};
    float wp_jerk_msss_{0.0f};
    float terrain_margin_m_{0.0f};
    float last_wp_speed_ms_{0.0f};
    float last_wp_speed_up_ms_{0.0f};
    float last_wp_speed_down_ms_{0.0f};
    bool check_wp_speed_change_{false};
    float wp_desired_speed_ne_ms_{0.0f};
    math::Vector3<float> origin_ned_m_{};
    math::Vector3<float> destination_ned_m_{};
    float track_dt_scalar_{0.0f};
    float offset_vel_ms_{0.0f};
    float offset_accel_mss_{0.0f};
    bool paused_{false};
    bool is_terrain_alt_{false};
    bool this_leg_is_spline_{false};
    std::uint32_t wp_last_update_ms_{0};
    WpNavFlags flags_{};
    float scurve_jerk_max_msss_{0.0f};
    float scurve_snap_max_mssss_{0.0f};
    bool scurve_legs_inited_{false};
    bool pos_control_stopping_point_inited_{false};
    PosControlSpeedAccel pos_speed_accel_{};
    math::Vector3<float> next_destination_ned_m_{};
    bool next_leg_is_spline_{false};
    bool scurve_this_leg_calculated_{false};
    float pos_terrain_d_m_{0.0f};
    float last_arc_rad_{0.0f};
    bool spline_this_leg_set_{false};
    math::Vector3<float> spline_origin_vel_ned_ms_{};
    math::Vector3<float> spline_destination_vel_ned_ms_{};
    bool spline_next_leg_set_{false};
    math::Vector3<float> spline_next_destination_ned_m_{};
    math::Vector3<float> spline_next_origin_vel_ned_ms_{};
    math::Vector3<float> spline_next_destination_vel_ned_ms_{};
    bool need_this_leg_dest_speed_max_{false};
    bool scurve_next_leg_calculated_{false};
    float last_next_arc_rad_{0.0f};
    bool need_this_leg_dest_speed_max_zero_{false};
    bool need_next_scurve_init_{false};
    bool rangefinder_use_{true};
};

}  // namespace fwcpp::wpnav
