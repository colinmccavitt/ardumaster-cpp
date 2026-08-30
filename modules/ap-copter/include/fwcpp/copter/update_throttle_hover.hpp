#pragma once

// Copter::update_throttle_hover leftover. Upstream ArduCopter/Attitude.cpp
// ~32-67. No motors / ahrs / pos_control objects — inject armed,
// land_complete, standby_active, has_manual_throttle, mode_is_drift
// (Mode::Number::DRIFT), vel_desired_U_ms, velocity_D_ok (AHRS
// get_velocity_D), vel_d_ms, throttle, roll/pitch.
//
// Records that Copter would call motors->update_throttle_hover(0.01f)
// in a level hover. Does not port AP_MotorsMulticopter::update_throttle_hover
// filter math (motors.cpp ~557-562).
//
// HAL_GYROFFT_ENABLED gyro_fft.update_freq_hover stays remaining:
// gyro_fft_update_freq_hover is always false this slice.

#include <cmath>

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::copter {

struct UpdateThrottleHoverInputs {
    bool armed{false};
    bool land_complete{false};
    bool standby_active{false};
    bool has_manual_throttle{false};
    bool mode_is_drift{false};
    float vel_desired_U_ms{0};
    bool velocity_D_ok{false};
    float vel_d_ms{0};
    float throttle{0};
    float roll_rad{0};
    float roll_trim_rad{0};
    float pitch_rad{0};
};

struct UpdateThrottleHoverEffects {
    bool early_return{false};
    bool motors_update_throttle_hover{false};
    float hover_dt{0};
    bool gyro_fft_update_freq_hover{false};
};

[[nodiscard]] inline UpdateThrottleHoverEffects update_throttle_hover(
    const UpdateThrottleHoverInputs& in) {
    UpdateThrottleHoverEffects fx{};

    // if not armed or landed or on standby then exit
    if (!in.armed || in.land_complete || in.standby_active) {
        fx.early_return = true;
        return fx;
    }

    // do not update in manual throttle modes or Drift
    if (in.has_manual_throttle || in.mode_is_drift) {
        fx.early_return = true;
        return fx;
    }

    // do not update while climbing or descending
    if (!fwcpp::math::is_zero(in.vel_desired_U_ms)) {
        fx.early_return = true;
        return fx;
    }

    // do not update if no vertical velocity estimate
    if (!in.velocity_D_ok) {
        fx.early_return = true;
        return fx;
    }

    // calc average throttle if we are in a level hover. accounts for heli hover roll trim
    if ((in.throttle > 0.0f) && (std::fabs(in.vel_d_ms) < 0.6f) &&
        (std::fabs(in.roll_rad - in.roll_trim_rad) < fwcpp::math::radians(5.0f)) &&
        (std::fabs(in.pitch_rad) < fwcpp::math::radians(5.0f))) {
        fx.motors_update_throttle_hover = true;
        fx.hover_dt = 0.01f;
    }

    return fx;
}

}  // namespace fwcpp::copter
