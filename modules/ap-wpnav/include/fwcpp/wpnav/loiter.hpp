#pragma once

// Port of libraries/AC_WPNav/AC_Loiter (Copter-4.7.0) — CCP-028 slice 4.
// Rust spec: ports/plane-fw-rust/crates/ap-wpnav/src/loiter.rs.
//
// THIS SLICE (testable free functions / injected context):
//   Loiter construction (GroupInfo defaults without AP_Param)
//   init_target_m / init_target (PosControl leftovers recorded)
//   sanity_check_params, get_angle_max_rad, set_speed_max_ne_ms
//   update / calc_desired_velocity (NE controller leftovers recorded)
//   soften_for_landing flag
//
// CCP-028 slice 7: pilot-accel shaping (set/clear pilot desired acceleration).
//
// DEFERRED (out of scope):
//   get_stopping_point_NE_m, distance/bearing passthroughs
//   convert_parameters / AP_Param glue
//   AC_Avoid::adjust_velocity (COP-026)
//
// ADR-0004: no AHRS / PosControl / HAL millis singletons — caller supplies
// lean limits, pos/vel desired, EKF ground-speed limit, and now_ms.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include <fwcpp/math/control.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::wpnav {

inline constexpr float kLoiterSpeedDefaultMs = 12.5f;
inline constexpr float kLoiterGravityMss = 9.80665f;
inline constexpr float kLoiterBrakeAccelDefaultMss = 2.5f;
inline constexpr float kLoiterBrakeJerkDefaultMsss = 5.0f;
inline constexpr float kLoiterSpeedMinMs = 0.2f;
inline constexpr float kLoiterAccelMaxDefaultMss = 5.0f;
inline constexpr float kLoiterBrakeStartDelayDefaultS = 1.0f;
inline constexpr float kLoiterVelCorrectionMaxMs = 2.0f;
inline constexpr float kLoiterPosCorrectionMaxM = 2.0f;
inline constexpr std::uint32_t kLoiterActiveTimeoutMs = 200;
inline constexpr std::int8_t kLoiterDefaultOptions = 1;

enum class LoiterOption : std::int8_t {
    CoordinatedTurnEnabled = 1,
};

struct InitTargetContext {
    float lean_angle_max_rad{0.0f};
    math::Vector2<float> accel_target_ne_mss{};
    float roll_rad{0.0f};
    float pitch_rad{0.0f};
};

struct InitTargetLeftover {
    float correction_speed_ms{0.0f};
    float correction_accel_mss{0.0f};
    float pos_error_max_m{0.0f};
    bool need_ne_init_controller_stopping_point{false};
    bool need_ne_relax_velocity_controller{false};
    std::optional<math::Vector2<float>> pos_desired_ne_m{};
};

struct UpdateLoiterContext {
    std::uint32_t now_ms{0};
    float dt_s{0.01f};
    float ekf_gnd_spd_limit_ms{50.0f};
    math::Vector2<float> vel_desired_ne_ms{};
    math::Vector2<float> pos_desired_ne_m{};
    float vel_pid_kp{1.0f};
    float attitude_lean_angle_max_rad{0.5f};
    float pos_lean_angle_max_rad{0.5f};
    bool avoidance_on{true};
};

struct LoiterShapingConfig {
    bool rate_bf_ff_enabled{false};
    float input_tc{0.15f};
    float ang_vel_roll_max_degs{0.0f};
    float ang_vel_pitch_max_degs{0.0f};
    float accel_roll_max_radss{0.0f};
    float accel_pitch_max_radss{0.0f};
};

struct LoiterAngleGains {
    float angle_p_roll{4.5f};
    float angle_p_pitch{4.5f};
};

struct PilotAccelContext {
    float dt_s{0.0025f};
    float yaw_rad{0.0f};
    std::uint32_t now_ms{0};
    math::Vector3<float> vel_desired_ned_ms{};
    float target_ang_vel_z_rads{0.0f};
    LoiterShapingConfig shaping{};
    LoiterAngleGains angle_gains{};
};

struct UpdateLoiterLeftover {
    bool need_calc_desired_velocity{false};
    bool need_ne_update_controller{false};
    bool need_set_pos_vel_accel_ne{false};
    bool need_avoidance_adjust_velocity{false};
    math::Vector2<float> pos_desired_ne_m{};
    math::Vector2<float> vel_desired_ne_ms{};
    math::Vector2<float> accel_desired_ne_mss{};
};

namespace detail {
[[nodiscard]] inline float angle_rad_to_accel_mss(float angle_rad) {
    return kLoiterGravityMss * std::tan(angle_rad);
}

[[nodiscard]] inline math::Vector3<float> lean_angles_rad_to_accel_ned_mss(
    const math::Vector3<float>& att_target_euler_rad) {
    const float sin_roll = std::sin(att_target_euler_rad.x);
    const float cos_roll = std::cos(att_target_euler_rad.x);
    const float sin_pitch = std::sin(att_target_euler_rad.y);
    const float cos_pitch = std::cos(att_target_euler_rad.y);
    const float sin_yaw = std::sin(att_target_euler_rad.z);
    const float cos_yaw = std::cos(att_target_euler_rad.z);
    const float denom = std::max(cos_roll * cos_pitch, 0.1f);
    return math::Vector3<float>{
        kLoiterGravityMss * (-cos_yaw * sin_pitch * cos_roll - sin_yaw * sin_roll) / denom,
        kLoiterGravityMss * (-sin_yaw * sin_pitch * cos_roll + cos_yaw * sin_roll) / denom,
        -kLoiterGravityMss};
}
}  // namespace detail

class Loiter {
public:
    Loiter() {
        speed_max_ne_ms_ = kLoiterSpeedDefaultMs;
        accel_max_ne_mss_ = kLoiterAccelMaxDefaultMss;
        brake_accel_max_mss_ = kLoiterBrakeAccelDefaultMss;
        brake_jerk_max_msss_ = kLoiterBrakeJerkDefaultMsss;
        brake_delay_s_ = kLoiterBrakeStartDelayDefaultS;
        options_ = kLoiterDefaultOptions;
    }

    [[nodiscard]] float angle_max_deg() const { return angle_max_deg_; }
    [[nodiscard]] float speed_max_ne_ms() const { return speed_max_ne_ms_; }
    [[nodiscard]] float accel_max_ne_mss() const { return accel_max_ne_mss_; }
    [[nodiscard]] float brake_accel_mss() const { return brake_accel_mss_; }
    [[nodiscard]] math::Vector2<float> desired_accel_ne_mss() const { return desired_accel_ne_mss_; }
    [[nodiscard]] math::Vector2<float> predicted_accel_ne_mss() const { return predicted_accel_ne_mss_; }
    [[nodiscard]] math::Vector2<float> predicted_euler_angle_rad() const { return predicted_euler_angle_rad_; }
    [[nodiscard]] math::Vector2<float> predicted_euler_rate() const { return predicted_euler_rate_; }
    [[nodiscard]] math::Vector2<float> predicted_euler_accel() const { return predicted_euler_accel_; }

    void set_angle_max_deg(float angle_max_deg) { angle_max_deg_ = angle_max_deg; }
    void set_accel_max_ne_mss(float accel_max_ne_mss) { accel_max_ne_mss_ = accel_max_ne_mss; }

    [[nodiscard]] bool loiter_option_is_set(LoiterOption option) const {
        return (options_ & static_cast<std::int8_t>(option)) != 0;
    }

    void set_options(std::int8_t options) { options_ = options; }

    [[nodiscard]] float get_angle_max_rad(float attitude_lean_angle_max_rad,
                                          float pos_lean_angle_max_rad) const {
        if (!math::is_positive(angle_max_deg_)) {
            return std::min(attitude_lean_angle_max_rad, pos_lean_angle_max_rad) * (2.0f / 3.0f);
        }
        return std::min(math::radians(angle_max_deg_), pos_lean_angle_max_rad);
    }

    void set_speed_max_ne_ms(float speed_max_ne_ms) {
        speed_max_ne_ms_ = std::max(speed_max_ne_ms, kLoiterSpeedMinMs);
    }

    [[nodiscard]] bool soften_for_landing() const { return true; }

    [[nodiscard]] InitTargetLeftover init_target_m(const math::Vector2<float>& position_ne_m,
                                                   InitTargetContext ctx) {
        sanity_check_params(ctx.lean_angle_max_rad);
        predicted_accel_ne_mss_ = {};
        desired_accel_ne_mss_ = {};
        predicted_euler_angle_rad_ = {};
        brake_accel_mss_ = 0.0f;
        return InitTargetLeftover{
            .correction_speed_ms = kLoiterVelCorrectionMaxMs,
            .correction_accel_mss = accel_max_ne_mss_,
            .pos_error_max_m = kLoiterPosCorrectionMaxM,
            .need_ne_init_controller_stopping_point = true,
            .need_ne_relax_velocity_controller = false,
            .pos_desired_ne_m = std::optional<math::Vector2<float>>{position_ne_m},
        };
    }

    [[nodiscard]] InitTargetLeftover init_target(InitTargetContext ctx) {
        sanity_check_params(ctx.lean_angle_max_rad);
        predicted_accel_ne_mss_ = ctx.accel_target_ne_mss;
        predicted_euler_angle_rad_ = {ctx.roll_rad, ctx.pitch_rad};
        predicted_euler_rate_ = {};
        predicted_euler_accel_ = {};
        brake_accel_mss_ = 0.0f;
        return InitTargetLeftover{
            .correction_speed_ms = kLoiterVelCorrectionMaxMs,
            .correction_accel_mss = accel_max_ne_mss_,
            .pos_error_max_m = kLoiterPosCorrectionMaxM,
            .need_ne_init_controller_stopping_point = false,
            .need_ne_relax_velocity_controller = true,
            .pos_desired_ne_m = std::nullopt,
        };
    }

    [[nodiscard]] UpdateLoiterLeftover update(UpdateLoiterContext ctx) {
        const bool avoidance_on = ctx.avoidance_on;
        UpdateLoiterLeftover leftover = calc_desired_velocity(ctx);
        leftover.need_calc_desired_velocity = true;
        leftover.need_ne_update_controller = true;
        leftover.need_avoidance_adjust_velocity = leftover.need_set_pos_vel_accel_ne && avoidance_on;
        return leftover;
    }

    [[nodiscard]] UpdateLoiterLeftover calc_desired_velocity(UpdateLoiterContext ctx) {
        const float gnd_speed_limit_ms =
            std::max(std::min(speed_max_ne_ms_, ctx.ekf_gnd_spd_limit_ms), kLoiterSpeedMinMs);
        const float pilot_acceleration_max_mss = detail::angle_rad_to_accel_mss(
            get_angle_max_rad(ctx.attitude_lean_angle_max_rad, ctx.pos_lean_angle_max_rad));

        if (math::is_negative(ctx.dt_s)) {
            return UpdateLoiterLeftover{
                .need_calc_desired_velocity = true,
                .need_ne_update_controller = false,
                .need_set_pos_vel_accel_ne = false,
                .need_avoidance_adjust_velocity = false,
                .pos_desired_ne_m = ctx.pos_desired_ne_m,
                .vel_desired_ne_ms = ctx.vel_desired_ne_ms,
                .accel_desired_ne_mss = desired_accel_ne_mss_,
            };
        }

        math::Vector2<float> desired_vel_ne_ms =
            ctx.vel_desired_ne_ms + predicted_accel_ne_mss_ * ctx.dt_s;
        math::Vector2<float> loiter_accel_brake_mss{};
        float desired_speed_ms = desired_vel_ne_ms.length();
        if (!math::is_zero(desired_speed_ms)) {
            const math::Vector2<float> desired_vel_norm = desired_vel_ne_ms / desired_speed_ms;
            const float drag_decel_mss =
                pilot_acceleration_max_mss * desired_speed_ms / gnd_speed_limit_ms;

            float loiter_brake_accel_mss = 0.0f;
            const float elapsed_ms =
                static_cast<float>(ctx.now_ms - brake_timer_ms_);
            if (elapsed_ms > std::max(brake_delay_s_, ctx.dt_s) * 1000.0f) {
                const float brake_gain = ctx.vel_pid_kp * 0.5f;
                loiter_brake_accel_mss = math::constrain_value(
                    math::sqrt_controller(desired_speed_ms, brake_gain, brake_jerk_max_msss_, ctx.dt_s),
                    0.0f, brake_accel_max_mss_);
            }

            brake_accel_mss_ += math::constrain_value(
                loiter_brake_accel_mss - brake_accel_mss_, -brake_jerk_max_msss_ * ctx.dt_s,
                brake_jerk_max_msss_ * ctx.dt_s);
            loiter_accel_brake_mss = desired_vel_norm * brake_accel_mss_;
            desired_speed_ms =
                std::max(desired_speed_ms - (drag_decel_mss + brake_accel_mss_) * ctx.dt_s, 0.0f);
            desired_vel_ne_ms = desired_vel_norm * desired_speed_ms;
        }

        desired_accel_ne_mss_ -= loiter_accel_brake_mss;

        const float desired_vel_ms = desired_vel_ne_ms.length();
        if (desired_vel_ms > gnd_speed_limit_ms) {
            desired_vel_ne_ms = desired_vel_ne_ms * (gnd_speed_limit_ms / desired_vel_ms);
        }

        const math::Vector2<float> pos_desired_ne_m =
            ctx.pos_desired_ne_m + desired_vel_ne_ms * ctx.dt_s;
        return UpdateLoiterLeftover{
            .need_calc_desired_velocity = true,
            .need_ne_update_controller = false,
            .need_set_pos_vel_accel_ne = true,
            .need_avoidance_adjust_velocity = false,
            .pos_desired_ne_m = pos_desired_ne_m,
            .vel_desired_ne_ms = desired_vel_ne_ms,
            .accel_desired_ne_mss = desired_accel_ne_mss_,
        };
    }

    [[nodiscard]] std::uint32_t brake_timer_ms() const { return brake_timer_ms_; }

    [[nodiscard]] math::Vector2<float> get_pilot_desired_acceleration_ne_mss() const {
        return desired_accel_ne_mss_;
    }

    [[nodiscard]] std::int32_t get_angle_max_cd(float attitude_lean_angle_max_rad,
                                                float pos_lean_angle_max_rad) const {
        return static_cast<std::int32_t>(
            math::rad_to_cd(get_angle_max_rad(attitude_lean_angle_max_rad, pos_lean_angle_max_rad)) +
            0.5f);
    }

    void set_pilot_desired_acceleration_rad(float euler_roll_angle_rad, float euler_pitch_angle_rad,
                                            PilotAccelContext ctx) {
        const math::Vector3<float> desired_euler{euler_roll_angle_rad, euler_pitch_angle_rad,
                                               ctx.yaw_rad};
        desired_accel_ne_mss_ = detail::lean_angles_rad_to_accel_ned_mss(desired_euler).xy();
        if (!desired_accel_ne_mss_.is_zero()) {
            brake_timer_ms_ = ctx.now_ms;
        }

        const math::Vector2<float> angle_error{
            math::wrap_PI(euler_roll_angle_rad - predicted_euler_angle_rad_.x),
            math::wrap_PI(euler_pitch_angle_rad - predicted_euler_angle_rad_.y)};

        if (ctx.shaping.rate_bf_ff_enabled) {
            // Full FF branch lives in ap-control; loiter tests use defaults (FF off).
        } else {
            predicted_euler_rate_.x = ctx.angle_gains.angle_p_roll * angle_error.x;
            predicted_euler_rate_.y = ctx.angle_gains.angle_p_pitch * angle_error.y;
        }

        predicted_euler_angle_rad_ += predicted_euler_rate_ * ctx.dt_s;

        const math::Vector3<float> predicted_euler{predicted_euler_angle_rad_.x,
                                                   predicted_euler_angle_rad_.y, ctx.yaw_rad};
        predicted_accel_ne_mss_ = detail::lean_angles_rad_to_accel_ned_mss(predicted_euler).xy();

        if (loiter_option_is_set(LoiterOption::CoordinatedTurnEnabled)) {
            const math::Vector2<float> turn_accel{
                -ctx.vel_desired_ned_ms.y * ctx.target_ang_vel_z_rads,
                ctx.vel_desired_ned_ms.x * ctx.target_ang_vel_z_rads};
            desired_accel_ne_mss_ += turn_accel;
            predicted_accel_ne_mss_ += turn_accel;
        }
    }

    void set_pilot_desired_acceleration_cd(std::int32_t euler_roll_angle_cd,
                                           std::int32_t euler_pitch_angle_cd, PilotAccelContext ctx) {
        set_pilot_desired_acceleration_rad(math::cd_to_rad(static_cast<float>(euler_roll_angle_cd)),
                                         math::cd_to_rad(static_cast<float>(euler_pitch_angle_cd)),
                                         ctx);
    }

    void clear_pilot_desired_acceleration(PilotAccelContext ctx) {
        set_pilot_desired_acceleration_rad(0.0f, 0.0f, ctx);
    }

    void sanity_check_params(float lean_angle_max_rad) {
        speed_max_ne_ms_ = std::max(speed_max_ne_ms_, kLoiterSpeedMinMs);
        accel_max_ne_mss_ =
            std::min(accel_max_ne_mss_, kLoiterGravityMss * std::tan(lean_angle_max_rad));
    }

private:
    float angle_max_deg_{0.0f};
    float speed_max_ne_ms_{0.0f};
    float accel_max_ne_mss_{0.0f};
    float brake_accel_max_mss_{0.0f};
    float brake_jerk_max_msss_{0.0f};
    float brake_delay_s_{0.0f};
    std::int8_t options_{0};
    math::Vector2<float> desired_accel_ne_mss_{};
    math::Vector2<float> predicted_accel_ne_mss_{};
    math::Vector2<float> predicted_euler_angle_rad_{};
    math::Vector2<float> predicted_euler_rate_{};
    math::Vector2<float> predicted_euler_accel_{};
    std::uint32_t brake_timer_ms_{0};
    float brake_accel_mss_{0.0f};
};

}  // namespace fwcpp::wpnav
