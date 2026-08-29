#pragma once

// QuadPlane xy / climb / tilt-to-fwd — Plane-4.7.0 ArduPlane/quadplane.cpp:
// set_climb_rate_ms (1092-1096), run_xy_controller (2210-2228),
// get_vfwd_method (2927-2967), assign_tilt_to_fwd_thr (2972-3088).
//
// ADR-0012: header-only ticks/effects. No pos_control / wp_nav /
// attitude_control objects (NE_* and D_* are flags). HAL_LOGGING QBRK/FWDT
// skipped. AP_ICENGINE / QAUTOTUNE compile gates are injected bools
// (ice_blocks_fwd, in_qautotune). Local accel_mss_to_angle_deg — do not
// link ap-poscontrol.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::quadplane {

inline constexpr float kGravityMss = 9.80665f;
inline constexpr float kWpAccelMssDefault = 2.5f;
inline constexpr float kDefaultSpeedNeMs = 10.0f;
inline constexpr float kQFwdThrGainDefault = 2.0f;
inline constexpr float kQFwdPitchLimDefault = 3.0f;
inline constexpr float kQBckPitchLimDefault = 10.0f;
inline constexpr float kMinAirspeedMinMs = 5.0f;
inline constexpr float kLeanAngleMaxCdCap = 4500.0f;
inline constexpr float kFwdThrExternalLimit = 0.95f;
inline constexpr float kFwdTiltMaxDeg = 45.0f;
inline constexpr float kBckPitchFilterTconst = 0.5f;
inline constexpr float kFwdPitchSlewPerSec = 0.1f;
inline constexpr float kFwdThrGroundBlendM = 2.0f;
inline constexpr float kAirspeedFloorMs = 0.1f;

// Match ArduPlane/quadplane.h ActiveFwdThr / FwdThrUse.
enum class ActiveFwdThr : std::uint8_t {
    kNone = 0,
    kOld = 1,
    kNew = 2,
};

enum class FwdThrUse : std::uint8_t {
    kOff = 0,
    kPosctrl = 1,
    kAll = 2,
};

struct FwdTiltState {
    float q_fwd_throttle{0.0f};
    float q_fwd_pitch_lim_cd{0.0f};
    float q_bck_pitch_lim_cd{0.0f};
    std::uint32_t q_pitch_limit_update_ms{0};
};

struct ClimbRateTick {
    bool input_vel_accel_D_m{false};
    float vel_d_m{0.0f};
    float accel_d_mss{0.0f};
    bool inhibit_descent_limit{false};
};

struct XyControllerInputs {
    float accel_limit_mss{0.0f};
    float wp_accel_mss{kWpAccelMssDefault};
    float default_speed_ne_ms{kDefaultSpeedNeMs};
    float lean_angle_max_cd{0.0f};
    float q_fwd_throttle{0.0f};
    bool ne_is_active{false};
};

struct XyControllerTick {
    float accel_mss{0.0f};
    float speed_ms{0.0f};
    bool ne_set_max_speed_accel{false};
    bool ne_set_correction_speed_accel{false};
    bool ne_init_controller{false};
    bool set_lean_angle_max_cd{false};
    float lean_angle_max_cd{0.0f};
    bool ne_set_externally_limited{false};
    bool ne_update_controller{false};
};

struct VfwdMethodInputs {
    bool allow_vfwd{true};
    bool ice_blocks_fwd{false};
    bool in_qautotune{false};
    float q_fwd_thr_gain{kQFwdThrGainDefault};
    bool vfwd_enable_active{false};
    FwdThrUse q_fwd_thr_use{FwdThrUse::kOff};
    bool ne_is_active{false};
    float vel_forward_gain{0.0f};
};

struct AssignTiltInputs {
    VfwdMethodInputs vfwd{};
    float nav_pitch_cd{0.0f};
    float q_fwd_thr_gain{kQFwdThrGainDefault};
    float q_fwd_pitch_lim{kQFwdPitchLimDefault};
    float q_bck_pitch_lim{kQBckPitchLimDefault};
    float angle_max_cd{0.0f};
    bool tiltrotor_enabled{false};
    bool ne_is_active{false};
    bool fwd_pitch_is_limited{false};
    float g_dt{0.0f};
    bool have_airspeed{false};
    float aspeed{0.0f};
    float airspeed_min{0.0f};
    std::uint32_t now_ms{0};
    bool in_vtol_land_approach{false};
    float vel_forward_alt_cutoff_m{0.0f};
    float height_above_ground_m{0.0f};
};

struct AssignTiltTick {
    bool early_return{false};
    float nav_pitch_cd{0.0f};
    float fwd_thr_scaler{0.0f};
    float nav_pitch_lower_limit_cd{0.0f};
    float nav_pitch_upper_limit_cd{0.0f};
};

// degrees(atanf(accel / GRAVITY_MSS)) — poscontrol has accel_mss_to_angle_rad
// but this module must not link ap-poscontrol.
[[nodiscard]] inline float accel_mss_to_angle_deg(float accel_mss) {
    return fwcpp::math::degrees(std::atan(accel_mss / kGravityMss));
}

[[nodiscard]] inline ClimbRateTick set_climb_rate_ms(float target_climb_rate_ms) {
    ClimbRateTick tick{};
    tick.vel_d_m = -target_climb_rate_ms;
    tick.accel_d_mss = 0.0f;
    tick.inhibit_descent_limit = false;
    tick.input_vel_accel_D_m = true;
    return tick;
}

[[nodiscard]] inline XyControllerTick run_xy_controller(const XyControllerInputs& in) {
    XyControllerTick tick{};
    tick.accel_mss = in.wp_accel_mss;
    if (fwcpp::math::is_positive(in.accel_limit_mss)) {
        tick.accel_mss = std::max(in.wp_accel_mss, in.accel_limit_mss);
    }
    tick.speed_ms = in.default_speed_ne_ms;
    tick.ne_set_max_speed_accel = true;
    tick.ne_set_correction_speed_accel = true;
    if (!in.ne_is_active) {
        tick.ne_init_controller = true;
    }
    tick.set_lean_angle_max_cd = true;
    tick.lean_angle_max_cd = std::min(
        kLeanAngleMaxCdCap,
        std::max(accel_mss_to_angle_deg(in.accel_limit_mss) * 100.0f, in.lean_angle_max_cd));
    if (in.q_fwd_throttle > kFwdThrExternalLimit) {
        tick.ne_set_externally_limited = true;
    }
    tick.ne_update_controller = true;
    return tick;
}

[[nodiscard]] inline ActiveFwdThr get_vfwd_method(const VfwdMethodInputs& in) {
    if (!in.allow_vfwd) {
        return ActiveFwdThr::kNone;
    }
    if (in.ice_blocks_fwd) {
        return ActiveFwdThr::kNone;
    }
    if (in.in_qautotune) {
        return ActiveFwdThr::kNone;
    }
    if (fwcpp::math::is_positive(in.q_fwd_thr_gain)) {
        if (in.vfwd_enable_active) {
            return ActiveFwdThr::kNew;
        }
        if (in.q_fwd_thr_use == FwdThrUse::kAll) {
            return ActiveFwdThr::kNew;
        }
        if (in.q_fwd_thr_use == FwdThrUse::kPosctrl && in.ne_is_active) {
            return ActiveFwdThr::kNew;
        }
    }
    if (fwcpp::math::is_positive(in.vel_forward_gain) && in.ne_is_active) {
        return ActiveFwdThr::kOld;
    }
    return ActiveFwdThr::kNone;
}

inline AssignTiltTick assign_tilt_to_fwd_thr(FwdTiltState& state, const AssignTiltInputs& in) {
    AssignTiltTick tick{};
    tick.nav_pitch_cd = in.nav_pitch_cd;

    VfwdMethodInputs vfwd = in.vfwd;
    vfwd.q_fwd_thr_gain = in.q_fwd_thr_gain;
    if (get_vfwd_method(vfwd) != ActiveFwdThr::kNew) {
        state.q_fwd_throttle = 0.0f;
        state.q_fwd_pitch_lim_cd = 100.0f * in.q_fwd_pitch_lim;
        tick.early_return = true;
        return tick;
    }

    const float fwd_tilt_rad = fwcpp::math::radians(
        fwcpp::math::constrain_value(-0.01f * in.nav_pitch_cd, 0.0f, kFwdTiltMaxDeg));
    state.q_fwd_throttle = std::min(in.q_fwd_thr_gain * std::tan(fwd_tilt_rad), 1.0f);

    if (!in.tiltrotor_enabled) {
        const float fwd_tilt_range_cd = in.angle_max_cd - 100.0f * in.q_fwd_pitch_lim;
        if (fwcpp::math::is_positive(fwd_tilt_range_cd)) {
            const bool fwd_limited = in.ne_is_active && in.fwd_pitch_is_limited;
            const float fwd_pitch_lim_cd_tgt =
                fwd_limited ? in.angle_max_cd : 100.0f * in.q_fwd_pitch_lim;
            const float delta_max = kFwdPitchSlewPerSec * fwd_tilt_range_cd * in.g_dt;
            state.q_fwd_pitch_lim_cd += fwcpp::math::constrain_value(
                fwd_pitch_lim_cd_tgt - state.q_fwd_pitch_lim_cd, -delta_max, delta_max);
            state.q_fwd_pitch_lim_cd = std::min(
                state.q_fwd_pitch_lim_cd, std::max(-in.nav_pitch_cd, 100.0f * in.q_fwd_pitch_lim));
        } else {
            state.q_fwd_pitch_lim_cd = in.angle_max_cd;
        }
    }

    tick.nav_pitch_upper_limit_cd = 100.0f * in.q_bck_pitch_lim;
    if (fwcpp::math::is_positive(in.q_bck_pitch_lim) && in.have_airspeed) {
        const float reference_speed = std::max(in.airspeed_min, kMinAirspeedMinMs);
        const float speed_ratio = reference_speed / std::max(in.aspeed, kAirspeedFloorMs);
        const float speed_scaler = speed_ratio * speed_ratio;
        tick.nav_pitch_upper_limit_cd *= speed_scaler;
        tick.nav_pitch_upper_limit_cd = std::min(tick.nav_pitch_upper_limit_cd, in.angle_max_cd);

        const float dt = static_cast<float>(in.now_ms - state.q_pitch_limit_update_ms);
        state.q_pitch_limit_update_ms = in.now_ms;
        if (fwcpp::math::is_positive(dt)) {
            const float coef = dt / (dt + kBckPitchFilterTconst);
            state.q_bck_pitch_lim_cd =
                (1.0f - coef) * state.q_bck_pitch_lim_cd + coef * tick.nav_pitch_upper_limit_cd;
        }
        tick.nav_pitch_cd = std::min(tick.nav_pitch_cd, static_cast<float>(static_cast<std::int32_t>(state.q_bck_pitch_lim_cd)));
    }

    if (!in.in_vtol_land_approach) {
        const float alt_cutoff_m = std::max(0.0f, in.vel_forward_alt_cutoff_m);
        tick.fwd_thr_scaler = fwcpp::math::linear_interpolate(
            0.0f, 1.0f, in.height_above_ground_m, alt_cutoff_m, alt_cutoff_m + kFwdThrGroundBlendM);
    } else {
        tick.fwd_thr_scaler = 1.0f;
    }
    state.q_fwd_throttle *= tick.fwd_thr_scaler;

    tick.nav_pitch_lower_limit_cd = -static_cast<float>(static_cast<std::int32_t>(
        in.angle_max_cd * (1.0f - tick.fwd_thr_scaler) + state.q_fwd_pitch_lim_cd * tick.fwd_thr_scaler));
    tick.nav_pitch_cd = std::max(
        tick.nav_pitch_cd, static_cast<float>(static_cast<std::int32_t>(tick.nav_pitch_lower_limit_cd)));
    return tick;
}

}  // namespace fwcpp::quadplane
