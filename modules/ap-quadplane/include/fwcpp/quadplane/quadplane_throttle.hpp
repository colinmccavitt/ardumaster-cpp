#pragma once

// QuadPlane throttle hover / suppression / mix — Plane-4.7.0
// ArduPlane/quadplane.cpp: update_throttle_suppression (1852-1914),
// update_throttle_hover (1918-1960), update_throttle_mix (4148-4206),
// get_throttle_input (4886-4894). LAND_CHECK_* at 4144-4146.
//
// ADR-0012: header-only ticks/effects. No motors / pos_control /
// attitude_control / SRV objects — return flags. is_flying_vtol is a
// later remaining group and is injected. HAL_GYROFFT skipped
// (kOutOfScope). filtered_accel_length is injected so ap-filter is not
// linked.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/quadplane/quadplane_auto_vtol_mission.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>

namespace fwcpp::quadplane {

inline constexpr std::uint32_t kMotorsRecentlyActiveMs = 2000;
inline constexpr std::uint32_t kHoverPidzStaleMs = 20;
inline constexpr float kSuppressVelZCms = 100.0f;
inline constexpr float kSuppressHeightM = 5.0f;
inline constexpr float kHoverVelZCms = 60.0f;
inline constexpr float kHoverAngleCd = 500.0f;
inline constexpr float kHoverAirspeedFrac = 0.3f;
inline constexpr float kHoverUpdateTc = 0.01f;
inline constexpr float kFwThrottleIdleMargin = 10.0f;
inline constexpr float kLandCheckAngleErrorDeg = 30.0f;
inline constexpr float kLandCheckLargeAngleCd = 1500.0f;
inline constexpr float kLandCheckAccelMoving = 3.0f;
inline constexpr float kLandCheckGravityMss = 9.80665f;
inline constexpr float kThrottleMixMax = 1.0f;

enum class ThrottleMixKind : std::uint8_t {
    kNone = 0,
    kMin = 1,
    kMan = 2,
    kMax = 3,
};

struct ThrottleSuppressionInputs {
    std::uint32_t now_ms{0};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool guided_wait_takeoff{false};
    float throttle_control_in{0.0f};
    bool reversed_throttle{false};
    bool arming_check_throttle{false};
    bool is_vtol_man_throttle{false};
    float throttle_norm_input_dz{0.0f};
    bool air_mode_active{false};
    bool does_auto_throttle{false};
    bool throttle_suppressed{true};
    float vel_z_up_cms{0.0f};
    float height_above_ground_m{0.0f};
    bool mode_auto{false};
    bool available{false};
    std::int32_t options{0};
    std::uint16_t nav_cmd_id{0};
};

struct ThrottleSuppressionTick {
    bool early_return{false};
    bool idle{false};
    bool set_desired_spool{false};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool set_throttle{false};
    float throttle{0.0f};
};

struct ThrottleHoverInputs {
    bool available{false};
    bool armed{false};
    bool is_flying_vtol{false};
    float vel_desired_U_ms{0.0f};
    bool tailsitter_enabled{false};
    float fw_throttle_scaled{0.0f};
    float throttle_min{0.0f};
    std::uint32_t now_ms{0};
    std::uint32_t last_pidz_active_ms{0};
    float motors_throttle{0.0f};
    float vel_z_up_cms{0.0f};
    std::int32_t roll_cd{0};
    std::int32_t pitch_cd{0};
    bool have_airspeed{false};
    float aspeed{0.0f};
    float airspeed_min{0.0f};
};

struct ThrottleHoverTick {
    bool early_return{false};
    bool update_throttle_hover{false};
    float hover_tc{0.0f};
};

struct ThrottleMixInputs {
    fwcpp::math::Vector3f accel_ef_mss{};
    float filtered_accel_length{0.0f};
    bool allow_update_throttle_mix{true};
    bool armed{false};
    bool is_vtol_man_throttle{false};
    float throttle_control_in{0.0f};
    bool reversed_throttle{false};
    bool air_mode_active{false};
    fwcpp::math::Vector3f angle_target_cd{};
    float att_error_deg{0.0f};
    float vel_desired_U_ms{0.0f};
    bool in_vtol_land_sequence{false};
    bool in_vtol_land_final{false};
};

struct ThrottleMixTick {
    bool filter_applied{false};
    bool early_return{false};
    ThrottleMixKind mix{ThrottleMixKind::kNone};
    float mix_max{0.0f};
    fwcpp::math::Vector3f accel_ef_mss{};
};

[[nodiscard]] inline float get_throttle_input(float throttle_control_in, bool reversed_throttle) {
    float ret = throttle_control_in;
    if (reversed_throttle) {
        ret = -ret;
    }
    return ret;
}

inline ThrottleSuppressionTick update_throttle_suppression(std::uint32_t& last_motors_active_ms,
                                                          const ThrottleSuppressionInputs& in) {
    ThrottleSuppressionTick tick{};
    if (in.now_ms - last_motors_active_ms < kMotorsRecentlyActiveMs) {
        tick.early_return = true;
        return tick;
    }
    if (static_cast<std::uint8_t>(in.desired_spool) <
        static_cast<std::uint8_t>(DesiredSpoolState::kThrottleUnlimited)) {
        tick.early_return = true;
        return tick;
    }

    bool idle = in.guided_wait_takeoff;
    if (!idle) {
        if (!fwcpp::math::is_zero(get_throttle_input(in.throttle_control_in, in.reversed_throttle)) &&
            (in.arming_check_throttle || in.is_vtol_man_throttle ||
             in.throttle_norm_input_dz > 0.0f)) {
            tick.early_return = true;
            return tick;
        }
        if (in.is_vtol_man_throttle && in.air_mode_active) {
            tick.early_return = true;
            return tick;
        }
        if (in.does_auto_throttle && !in.throttle_suppressed) {
            tick.early_return = true;
            return tick;
        }
        if (std::fabs(in.vel_z_up_cms) > kSuppressVelZCms) {
            tick.early_return = true;
            return tick;
        }
        if (in.height_above_ground_m > kSuppressHeightM) {
            tick.early_return = true;
            return tick;
        }
        if (in.mode_auto && is_vtol_takeoff(in.nav_cmd_id, in.available, in.options)) {
            tick.early_return = true;
            return tick;
        }
        idle = true;
    }

    tick.idle = idle;
    tick.set_desired_spool = true;
    tick.desired_spool = DesiredSpoolState::kGroundIdle;
    tick.set_throttle = true;
    tick.throttle = 0.0f;
    last_motors_active_ms = 0;
    return tick;
}

[[nodiscard]] inline ThrottleHoverTick update_throttle_hover(const ThrottleHoverInputs& in) {
    ThrottleHoverTick tick{};
    if (!in.available) {
        tick.early_return = true;
        return tick;
    }
    if (!in.armed || !in.is_flying_vtol) {
        tick.early_return = true;
        return tick;
    }
    if (!fwcpp::math::is_zero(in.vel_desired_U_ms)) {
        tick.early_return = true;
        return tick;
    }
    if (!in.tailsitter_enabled &&
        (in.fw_throttle_scaled > std::max(0.0f, in.throttle_min + kFwThrottleIdleMargin))) {
        tick.early_return = true;
        return tick;
    }
    if (in.now_ms - in.last_pidz_active_ms > kHoverPidzStaleMs) {
        tick.early_return = true;
        return tick;
    }
    if (in.motors_throttle > 0.0f && std::fabs(in.vel_z_up_cms) < kHoverVelZCms &&
        std::labs(in.roll_cd) < static_cast<long>(kHoverAngleCd) &&
        std::labs(in.pitch_cd) < static_cast<long>(kHoverAngleCd) && in.have_airspeed &&
        in.aspeed < in.airspeed_min * kHoverAirspeedFrac) {
        tick.update_throttle_hover = true;
        tick.hover_tc = kHoverUpdateTc;
    }
    return tick;
}

inline ThrottleMixTick update_throttle_mix(const ThrottleMixInputs& in) {
    ThrottleMixTick tick{};
    tick.accel_ef_mss = in.accel_ef_mss;
    tick.accel_ef_mss.z += kLandCheckGravityMss;
    tick.filter_applied = true;

    if (!in.allow_update_throttle_mix) {
        tick.early_return = true;
        return tick;
    }
    if (!in.armed) {
        tick.mix = ThrottleMixKind::kMin;
        return tick;
    }
    if (in.is_vtol_man_throttle) {
        if (!fwcpp::math::is_positive(get_throttle_input(in.throttle_control_in, in.reversed_throttle)) &&
            !in.air_mode_active) {
            tick.mix = ThrottleMixKind::kMin;
        } else {
            tick.mix = ThrottleMixKind::kMan;
        }
        return tick;
    }

    const bool large_angle_request = in.angle_target_cd.xy().length() > kLandCheckLargeAngleCd;
    const bool large_angle_error = in.att_error_deg > kLandCheckAngleErrorDeg;
    const bool accel_moving = in.filtered_accel_length > kLandCheckAccelMoving;
    const bool descent_not_demanded = in.vel_desired_U_ms >= 0.0f;
    bool use_mix_max = large_angle_request || large_angle_error || accel_moving || descent_not_demanded;
    if (in.in_vtol_land_sequence) {
        use_mix_max = !in.in_vtol_land_final;
    }
    if (use_mix_max) {
        tick.mix = ThrottleMixKind::kMax;
        tick.mix_max = kThrottleMixMax;
    } else {
        tick.mix = ThrottleMixKind::kMin;
    }
    return tick;
}

}  // namespace fwcpp::quadplane
