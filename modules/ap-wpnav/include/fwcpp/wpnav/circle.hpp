#pragma once

// Port of libraries/AC_WPNav/AC_Circle (Copter-4.7.0) — CCP-028 slice 5.
// Rust spec: ports/plane-fw-rust/crates/ap-wpnav/src/circle.rs.
//
// THIS SLICE (injected context, testable in header):
//   Circle construction (GroupInfo defaults)
//   init / init_ned_m / init_neu_cm (+ PosControl stopping-point leftovers)
//   set_center / set_center_ned_m (Location → NED via GetVectorNedContext)
//   update_ms / update_cms (+ PosControl input leftovers recorded)
//   calc_velocities, init_start_angle, is_active, option helpers
//
// CCP-028 slice 7: get_closest_point_on_circle_* , NEU/cm radius wrappers.
//
// DEFERRED (out of scope):
//   convert_parameters, PosControl passthroughs (roll/pitch/thrust, distance/bearing)
//
// ADR-0004: no AHRS / PosControl / millis singletons — caller supplies yaw,
// desired seat, NE speed/accel limits, dt_s, now_ms, and terrain height.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>

#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::wpnav {

inline constexpr float kCircleRadiusMDefault = 10.0f;
inline constexpr float kCircleRateDefault = 20.0f;
inline constexpr float kCircleAngularAccelMin = 2.0f;
inline constexpr float kCircleRadiusMaxM = 2000.0f;
inline constexpr std::uint32_t kCircleActiveTimeoutMs = 200;
inline constexpr std::int16_t kCircleDefaultOptions = 1;

enum class CircleOption : std::int16_t {
    ManualControl = 1 << 0,
    FaceDirectionOfTravel = 1 << 1,
    InitAtCenter = 1 << 2,
    RoiAtCenter = 1 << 3,
};

/// Leftover of `AC_WPNav::get_vector_NED_m` until a later wpnav slice owns it.
struct GetVectorNedContext {
    fwcpp::Location origin{};
    fwcpp::AltitudeContext alt{};
};

struct InitCircleContext {
    float yaw_rad{0.0f};
    float cos_yaw{1.0f};
    float sin_yaw{0.0f};
    math::Vector3<float> pos_desired_ned_m{};
    float ne_max_speed_ms{5.0f};
    float ne_max_accel_mss{2.5f};
};

struct InitCircleLeftover {
    bool need_ne_init_controller_stopping_point{false};
    bool need_d_init_controller_stopping_point{false};
};

struct UpdateCircleContext {
    std::uint32_t now_ms{0};
    float dt_s{0.01f};
    math::Vector3<float> pos_desired_ned_m{};
    float pos_desired_u_m{0.0f};
    float ne_max_speed_ms{5.0f};
    float ne_max_accel_mss{2.5f};
    std::optional<float> terrain_u_m{};
};

struct UpdateCircleLeftover {
    bool ok{false};
    bool need_input_pos_vel_accel_ne{false};
    bool need_input_pos_vel_accel_d{false};
    bool need_d_set_pos_target_from_climb_rate{false};
    bool need_ne_update_controller{false};
    math::Vector3<float> target_ned_m{};
    float climb_rate_ms{0.0f};
};

struct SetCenterLeftover {
    bool need_nav_error_log{false};
    bool used_pos_estimate_fallback{false};
};

namespace detail {
[[nodiscard]] inline float get_bearing_rad(const math::Vector2<float>& origin,
                                           const math::Vector2<float>& destination) {
    return math::wrap_2PI(std::atan2(destination.y - origin.y, destination.x - origin.x));
}
}  // namespace detail

[[nodiscard]] inline std::optional<std::pair<math::Vector3<float>, bool>>
get_vector_ned_m(const fwcpp::Location& loc, GetVectorNedContext ctx) {
    if (!ctx.origin.initialised()) {
        return std::nullopt;
    }
    const math::Vector2<float> ne = ctx.origin.get_distance_NE(loc);
    if (loc.get_alt_frame() == fwcpp::Location::AltFrame::ABOVE_TERRAIN) {
        std::int32_t terrain_u_cm = 0;
        if (!loc.get_alt_cm(fwcpp::Location::AltFrame::ABOVE_TERRAIN, ctx.alt, terrain_u_cm)) {
            return std::nullopt;
        }
        const float terrain_u_m = static_cast<float>(terrain_u_cm) * 0.01f;
        return std::make_pair(math::Vector3<float>{ne.x, ne.y, -terrain_u_m}, true);
    }
    std::int32_t origin_alt_cm = 0;
    if (!loc.get_alt_cm(fwcpp::Location::AltFrame::ABOVE_ORIGIN, ctx.alt, origin_alt_cm)) {
        return std::nullopt;
    }
    const float origin_alt_m = static_cast<float>(origin_alt_cm) * 0.01f;
    return std::make_pair(math::Vector3<float>{ne.x, ne.y, -origin_alt_m}, false);
}

class Circle {
public:
    Circle() {
        radius_parm_m_ = kCircleRadiusMDefault;
        rate_parm_degs_ = kCircleRateDefault;
        options_ = kCircleDefaultOptions;
        rotation_rate_max_rads_ = math::radians(kCircleRateDefault);
    }

    [[nodiscard]] float radius_parm_m() const { return radius_parm_m_; }
    void set_radius_parm_m(float radius_parm_m) { radius_parm_m_ = radius_parm_m; }

    [[nodiscard]] float radius_m() const { return radius_m_; }
    [[nodiscard]] float get_radius_m() const {
        return math::is_positive(radius_m_) ? radius_m_ : radius_parm_m_;
    }
    void set_radius_m(float radius_m) {
        radius_m_ = math::constrain_value(radius_m, 0.0f, kCircleRadiusMaxM);
    }

    [[nodiscard]] float get_radius_cm() const { return get_radius_m() * 100.0f; }
    void set_radius_cm(float radius_cm) { set_radius_m(radius_cm * 0.01f); }

    [[nodiscard]] math::Vector3<float> get_center_neu_cm() const {
        return math::Vector3<float>{center_ned_m_.x * 100.0f, center_ned_m_.y * 100.0f,
                                    -center_ned_m_.z * 100.0f};
    }

    [[nodiscard]] float get_rate_degs() const { return rate_parm_degs_; }
    void set_rate_parm_degs(float rate_parm_degs) { rate_parm_degs_ = rate_parm_degs; }
    [[nodiscard]] float get_rate_current() const { return math::degrees(angular_vel_rads_); }
    [[nodiscard]] float rotation_rate_max_rads() const { return rotation_rate_max_rads_; }
    void set_rate_degs(float rate_degs) { rotation_rate_max_rads_ = math::radians(rate_degs); }

    [[nodiscard]] math::Vector3<float> center_ned_m() const { return center_ned_m_; }
    void set_center_ned_m(const math::Vector3<float>& center_ned_m, bool is_terrain_alt) {
        center_ned_m_ = center_ned_m;
        is_terrain_alt_ = is_terrain_alt;
    }

    [[nodiscard]] SetCenterLeftover set_center(const fwcpp::Location& center,
                                               GetVectorNedContext vec_ctx,
                                               math::Vector3<float> pos_estimate_ned_m) {
        if (const auto converted = get_vector_ned_m(center, vec_ctx)) {
            set_center_ned_m(converted->first, converted->second);
            return SetCenterLeftover{};
        }
        set_center_ned_m(pos_estimate_ned_m, false);
        return SetCenterLeftover{.need_nav_error_log = true, .used_pos_estimate_fallback = true};
    }


    struct ClosestPointOnCircle {
        math::Vector3<float> point_ned_m{};
        float dist_to_edge_m{0.0f};
    };

    [[nodiscard]] ClosestPointOnCircle get_closest_point_on_circle_ned_m(
        math::Vector3<float> stopping_point_ned_m, float cos_yaw, float sin_yaw) const {
        const math::Vector3<float> vec_from_center = stopping_point_ned_m - center_ned_m_;
        if (!math::is_positive(radius_m_)) {
            return ClosestPointOnCircle{.point_ned_m = center_ned_m_, .dist_to_edge_m = 0.0f};
        }
        if (vec_from_center.length_squared() < 0.5f * 0.5f) {
            const math::Vector3<float> point_ned_m{center_ned_m_.x - radius_m_ * cos_yaw,
                                                   center_ned_m_.y - radius_m_ * sin_yaw,
                                                   center_ned_m_.z};
            return ClosestPointOnCircle{
                .point_ned_m = point_ned_m,
                .dist_to_edge_m = (stopping_point_ned_m - point_ned_m).length()};
        }
        const float dist_xy =
            math::Vector2<float>{vec_from_center.x, vec_from_center.y}.length();
        const math::Vector3<float> point_ned_m{
            center_ned_m_.x + vec_from_center.x / dist_xy * radius_m_,
            center_ned_m_.y + vec_from_center.y / dist_xy * radius_m_,
            center_ned_m_.z};
        return ClosestPointOnCircle{.point_ned_m = point_ned_m,
                                    .dist_to_edge_m = (stopping_point_ned_m - point_ned_m).length()};
    }

    [[nodiscard]] std::pair<math::Vector3<float>, float> get_closest_point_on_circle_neu_cm(
        math::Vector3<float> stopping_point_neu_cm, float cos_yaw, float sin_yaw) const {
        const math::Vector3<float> stopping_ned{stopping_point_neu_cm.x * 0.01f,
                                                stopping_point_neu_cm.y * 0.01f,
                                                -stopping_point_neu_cm.z * 0.01f};
        const ClosestPointOnCircle closest =
            get_closest_point_on_circle_ned_m(stopping_ned, cos_yaw, sin_yaw);
        const math::Vector3<float> neu_cm{closest.point_ned_m.x * 100.0f,
                                          closest.point_ned_m.y * 100.0f,
                                          -closest.point_ned_m.z * 100.0f};
        return {neu_cm, closest.dist_to_edge_m * 100.0f};
    }

    [[nodiscard]] bool center_is_terrain_alt() const { return is_terrain_alt_; }
    [[nodiscard]] float get_yaw_rad() const { return yaw_rad_; }
    [[nodiscard]] float angle_rad() const { return angle_rad_; }
    [[nodiscard]] float get_angle_total_rad() const { return angle_total_rad_; }
    [[nodiscard]] float angular_vel_rads() const { return angular_vel_rads_; }
    [[nodiscard]] float angular_vel_max_rads() const { return angular_vel_max_rads_; }
    [[nodiscard]] float angular_accel_radss() const { return angular_accel_radss_; }
    [[nodiscard]] std::uint32_t last_update_ms() const { return last_update_ms_; }

    void set_options(std::int16_t options) { options_ = options; }
    [[nodiscard]] bool option_is_set(CircleOption option) const {
        return (options_ & static_cast<std::int16_t>(option)) != 0;
    }
    [[nodiscard]] bool pilot_control_enabled() const {
        return option_is_set(CircleOption::ManualControl);
    }
    [[nodiscard]] bool roi_at_center() const { return option_is_set(CircleOption::RoiAtCenter); }

    [[nodiscard]] bool is_active(std::uint32_t now_ms) const {
        return now_ms - last_update_ms_ < kCircleActiveTimeoutMs;
    }

    void check_param_change() {
        if (!math::is_equal(last_radius_param_m_, radius_parm_m_)) {
            radius_m_ = radius_parm_m_;
            last_radius_param_m_ = radius_m_;
        }
    }

    [[nodiscard]] InitCircleLeftover init_ned_m(const math::Vector3<float>& center_ned_m,
                                                bool is_terrain_alt, float rate_degs,
                                                InitCircleContext ctx) {
        center_ned_m_ = center_ned_m;
        is_terrain_alt_ = is_terrain_alt;
        rotation_rate_max_rads_ = math::radians(rate_degs);
        calc_velocities(true, ctx.ne_max_speed_ms, ctx.ne_max_accel_mss);
        init_start_angle(false, ctx.yaw_rad, ctx.pos_desired_ned_m);
        return InitCircleLeftover{
            .need_ne_init_controller_stopping_point = true,
            .need_d_init_controller_stopping_point = true,
        };
    }

    [[nodiscard]] InitCircleLeftover init_neu_cm(const math::Vector3<float>& center_neu_cm,
                                                 bool is_terrain_alt, float rate_degs,
                                                 InitCircleContext ctx) {
        const math::Vector3<float> center_ned_m{center_neu_cm.x * 0.01f, center_neu_cm.y * 0.01f,
                                                -center_neu_cm.z * 0.01f};
        return init_ned_m(center_ned_m, is_terrain_alt, rate_degs, ctx);
    }

    [[nodiscard]] InitCircleLeftover init(InitCircleContext ctx) {
        radius_m_ = radius_parm_m_;
        last_radius_param_m_ = radius_m_;
        rotation_rate_max_rads_ = math::radians(rate_parm_degs_);

        math::Vector3<float> center = ctx.pos_desired_ned_m;
        if (!option_is_set(CircleOption::InitAtCenter)) {
            center.x += radius_m_ * ctx.cos_yaw;
            center.y += radius_m_ * ctx.sin_yaw;
        }
        center_ned_m_ = center;
        is_terrain_alt_ = false;

        calc_velocities(true, ctx.ne_max_speed_ms, ctx.ne_max_accel_mss);
        init_start_angle(true, ctx.yaw_rad, ctx.pos_desired_ned_m);
        return InitCircleLeftover{
            .need_ne_init_controller_stopping_point = true,
            .need_d_init_controller_stopping_point = true,
        };
    }

    [[nodiscard]] UpdateCircleLeftover update_ms(float climb_rate_ms, UpdateCircleContext ctx) {
        calc_velocities(false, ctx.ne_max_speed_ms, ctx.ne_max_accel_mss);

        const float dt = ctx.dt_s;
        if (angular_vel_rads_ < angular_vel_max_rads_) {
            angular_vel_rads_ += std::abs(angular_accel_radss_) * dt;
            angular_vel_rads_ = std::min(angular_vel_rads_, angular_vel_max_rads_);
        }
        if (angular_vel_rads_ > angular_vel_max_rads_) {
            angular_vel_rads_ -= std::abs(angular_accel_radss_) * dt;
            angular_vel_rads_ = std::max(angular_vel_rads_, angular_vel_max_rads_);
        }

        const float angle_change_rad = angular_vel_rads_ * dt;
        angle_rad_ += angle_change_rad;
        angle_rad_ = math::wrap_PI(angle_rad_);
        angle_total_rad_ += angle_change_rad;

        if (is_terrain_alt_ && !ctx.terrain_u_m.has_value()) {
            return UpdateCircleLeftover{.ok = false, .climb_rate_ms = climb_rate_ms};
        }

        const float target_d_m = is_terrain_alt_
                                     ? center_ned_m_.z - *ctx.terrain_u_m
                                     : -ctx.pos_desired_u_m;

        math::Vector3<float> target_ned_m{center_ned_m_.x, center_ned_m_.y, target_d_m};
        if (!math::is_zero(radius_m_)) {
            target_ned_m.x += radius_m_ * std::cos(-angle_rad_);
            target_ned_m.y += -radius_m_ * std::sin(-angle_rad_);
            yaw_rad_ = detail::get_bearing_rad(
                {ctx.pos_desired_ned_m.x, ctx.pos_desired_ned_m.y},
                {center_ned_m_.x, center_ned_m_.y});
            if (option_is_set(CircleOption::FaceDirectionOfTravel)) {
                yaw_rad_ += math::is_positive(rotation_rate_max_rads_) ? -math::radians(90.0f)
                                                                       : math::radians(90.0f);
                yaw_rad_ = math::wrap_2PI(yaw_rad_);
            }
        } else {
            yaw_rad_ = angle_rad_;
        }

        last_update_ms_ = ctx.now_ms;
        return UpdateCircleLeftover{
            .ok = true,
            .need_input_pos_vel_accel_ne = true,
            .need_input_pos_vel_accel_d = is_terrain_alt_,
            .need_d_set_pos_target_from_climb_rate = !is_terrain_alt_,
            .need_ne_update_controller = true,
            .target_ned_m = target_ned_m,
            .climb_rate_ms = climb_rate_ms,
        };
    }

    [[nodiscard]] UpdateCircleLeftover update_cms(float climb_rate_cms, UpdateCircleContext ctx) {
        return update_ms(climb_rate_cms * 0.01f, ctx);
    }

    void calc_velocities(bool init_velocity, float ne_max_speed_ms, float ne_max_accel_mss) {
        if (radius_m_ <= 0.0f) {
            angular_vel_max_rads_ = rotation_rate_max_rads_;
            angular_accel_radss_ =
                std::max(std::abs(angular_vel_max_rads_), math::radians(kCircleAngularAccelMin));
        } else {
            const float vel_max_ms =
                std::min(ne_max_speed_ms, math::safe_sqrt(0.5f * ne_max_accel_mss * radius_m_));
            angular_vel_max_rads_ = vel_max_ms / radius_m_;
            angular_vel_max_rads_ = math::constrain_value(
                rotation_rate_max_rads_, -angular_vel_max_rads_, angular_vel_max_rads_);
            angular_accel_radss_ =
                std::max(ne_max_accel_mss / radius_m_, math::radians(kCircleAngularAccelMin));
        }
        if (init_velocity) {
            angular_vel_rads_ = 0.0f;
        }
    }

    void init_start_angle(bool use_heading, float yaw_rad,
                          const math::Vector3<float>& pos_desired_ned_m) {
        angle_total_rad_ = 0.0f;
        if (radius_m_ <= 0.0f) {
            angle_rad_ = yaw_rad;
            return;
        }
        if (use_heading) {
            angle_rad_ = math::wrap_PI(yaw_rad - static_cast<float>(M_PI));
        } else if (math::is_equal(pos_desired_ned_m.x, center_ned_m_.x) &&
                   math::is_equal(pos_desired_ned_m.y, center_ned_m_.y)) {
            angle_rad_ = math::wrap_PI(yaw_rad - static_cast<float>(M_PI));
        } else {
            const float bearing_rad = std::atan2(pos_desired_ned_m.y - center_ned_m_.y,
                                                 pos_desired_ned_m.x - center_ned_m_.x);
            angle_rad_ = math::wrap_PI(bearing_rad);
        }
    }

private:
    float radius_parm_m_{0.0f};
    float rate_parm_degs_{0.0f};
    std::int16_t options_{0};
    math::Vector3<float> center_ned_m_{};
    float radius_m_{0.0f};
    float rotation_rate_max_rads_{0.0f};
    float yaw_rad_{0.0f};
    float angle_rad_{0.0f};
    float angle_total_rad_{0.0f};
    float angular_vel_rads_{0.0f};
    float angular_vel_max_rads_{0.0f};
    float angular_accel_radss_{0.0f};
    std::uint32_t last_update_ms_{0};
    float last_radius_param_m_{0.0f};
    bool is_terrain_alt_{false};
};

}  // namespace fwcpp::wpnav
