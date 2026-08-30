#pragma once

// Copter AutoYaw::get_heading / update_weathervane — ArduCopter/autoyaw.cpp
// ~329-401 (Plane-4.7.0). Small AutoYaw state; get_heading injects
// has_valid_input / use_pilot_yaw / pilot_yaw_rate_rads already computed
// by get_pilot_desired_yaw_rate_rads. No AP::ahrs / rc() / flightmode /
// g2.weathervane singletons (ADR-0012).
//
// This slice: leftover update_weathervane tick. Injected
// allows_weathervaning + get_yaw_out_ok + yaw_rate_rads_from_wv (already
// rad — skip cd_to_rad). No AC_WeatherVane library math. set_mode_to_default
// leftover: injected default_mode (upstream default_mode(false) reads
// WP_YAW_BEHAVIOR). get_heading calls update_weathervane only when
// injected weathervane_enabled is true (upstream #if WEATHERVANE_ENABLED).
// Default false keeps weathervane_invoked false.
// Full yaw_rad / rate_rads LOOK_AT_NEXT_WP / ROI / LOOK_AHEAD / FIXED /
// CIRCLE / ANGLE_RATE bodies stay later; stored fields are returned.
// HeadingCommand / HeadingMode reused from attitude_kinematics.hpp.

#include <cstdint>

#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::copter {

// Compile-time stand-in for upstream WEATHERVANE_ENABLED. Runtime
// weathervane_enabled on get_heading is the leftover #if gate.
inline constexpr bool kWeathervaneEnabled = false;

class AutoYaw {
public:
    // Upstream Mode::AutoYaw::Mode (ArduCopter/mode.h ~317-329).
    enum class Mode : std::uint8_t {
        HOLD = 0,
        LOOK_AT_NEXT_WP = 1,
        ROI = 2,
        FIXED = 3,
        LOOK_AHEAD = 4,
        RESET_TO_ARMED_YAW = 5,
        ANGLE_RATE = 6,
        RATE = 7,
        CIRCLE = 8,
        PILOT_RATE = 9,
        WEATHERVANE = 10,
    };

    [[nodiscard]] Mode mode() const { return mode_; }
    [[nodiscard]] Mode last_mode() const { return last_mode_; }
    [[nodiscard]] bool weathervane_invoked() const { return weathervane_invoked_; }
    [[nodiscard]] float pilot_yaw_rate_rads() const { return pilot_yaw_rate_rads_; }
    // Leftover tick: set_mode_to_default(false) was taken (injected default_mode).
    [[nodiscard]] bool used_default_mode_leftover() const { return used_default_mode_leftover_; }

    // autoyaw.cpp:59. Early-return if unchanged; stash _last_mode.
    // RATE zeros stored rate (upstream ~100-103). LOOK_AHEAD ahrs init
    // and other bodies stay remaining — get_heading only needs HOLD /
    // PILOT_RATE (empty init).
    void set_mode(Mode yaw_mode) {
        if (mode_ == yaw_mode) {
            return;
        }
        last_mode_ = mode_;
        mode_ = yaw_mode;
        switch (mode_) {
        case Mode::RATE:
            yaw_rate_rads_ = 0.0f;
            break;
        case Mode::HOLD:
        case Mode::PILOT_RATE:
        case Mode::WEATHERVANE:
        case Mode::LOOK_AT_NEXT_WP:
        case Mode::ROI:
        case Mode::FIXED:
        case Mode::LOOK_AHEAD:
        case Mode::RESET_TO_ARMED_YAW:
        case Mode::ANGLE_RATE:
        case Mode::CIRCLE:
            break;
        }
    }

    // Stored field only. Full yaw_rad() switch (autoyaw.cpp ~233-294)
    // is remaining: ROI / FIXED slew / LOOK_AHEAD / CIRCLE / ANGLE_RATE
    // integrate / pos_control LOOK_AT_NEXT_WP / att-target for RATE.
    [[nodiscard]] float yaw_rad() const { return yaw_angle_rad_; }

    // autoyaw.cpp:298. HOLD zeros; PILOT_RATE copies stored pilot rate;
    // RATE / WEATHERVANE keep stored _yaw_rate_rads. Other cases
    // remaining (LOOK_AT_NEXT_WP pos_control, etc.).
    [[nodiscard]] float rate_rads() {
        switch (mode_) {
        case Mode::HOLD:
            yaw_rate_rads_ = 0.0f;
            break;
        case Mode::PILOT_RATE:
            yaw_rate_rads_ = pilot_yaw_rate_rads_;
            break;
        case Mode::RATE:
        case Mode::WEATHERVANE:
            break;
        case Mode::LOOK_AT_NEXT_WP:
        case Mode::ROI:
        case Mode::FIXED:
        case Mode::LOOK_AHEAD:
        case Mode::RESET_TO_ARMED_YAW:
        case Mode::ANGLE_RATE:
        case Mode::CIRCLE:
            break;
        }
        return yaw_rate_rads_;
    }

    // autoyaw.cpp:376. Injected allows_weathervaning + get_yaw_out
    // success/rate already in rad (skip cd conversion). No AC_WeatherVane
    // math, no flightmode alt/takeoff/landing or pos/att cd. pilot_yaw_rads
    // is unused here — upstream only forwards it into get_yaw_out.
    // set_mode_to_default leftover: inject default_mode.
    void update_weathervane([[maybe_unused]] float pilot_yaw_rads, bool allows_weathervaning,
                            bool get_yaw_out_ok, float yaw_rate_rads_from_wv, Mode default_mode) {
        weathervane_invoked_ = true;
        used_default_mode_leftover_ = false;
        if (allows_weathervaning && get_yaw_out_ok) {
            set_mode(Mode::WEATHERVANE);
            yaw_rate_rads_ = yaw_rate_rads_from_wv;
            return;
        }

        // Weathervane not allowed, or get_yaw_out thresholds not met.
        if (mode_ == Mode::WEATHERVANE) {
            yaw_rate_rads_ = 0.0f;
            if (last_mode_ == Mode::HOLD) {
                used_default_mode_leftover_ = true;
                set_mode(default_mode);
            } else {
                set_mode(last_mode_);
            }
        }
    }

    // autoyaw.cpp:329. Injected RC / use_pilot_yaw / already-computed
    // pilot rate. Calls update_weathervane only when weathervane_enabled
    // (upstream #if WEATHERVANE_ENABLED). Default false.
    [[nodiscard]] control::HeadingCommand get_heading(
        bool has_valid_input, bool use_pilot_yaw, float injected_pilot_yaw_rate_rads,
        bool weathervane_enabled = false, bool allows_weathervaning = false,
        bool get_yaw_out_ok = false, float yaw_rate_rads_from_wv = 0.0f,
        Mode default_mode = Mode::LOOK_AT_NEXT_WP) {
        weathervane_invoked_ = false;
        used_default_mode_leftover_ = false;
        pilot_yaw_rate_rads_ = 0.0f;
        if (has_valid_input && use_pilot_yaw) {
            pilot_yaw_rate_rads_ = injected_pilot_yaw_rate_rads;
            if (!math::is_zero(pilot_yaw_rate_rads_)) {
                set_mode(Mode::PILOT_RATE);
            }
        } else if (mode_ == Mode::PILOT_RATE) {
            set_mode(Mode::HOLD);
        }

        if (weathervane_enabled) {
            update_weathervane(pilot_yaw_rate_rads_, allows_weathervaning, get_yaw_out_ok,
                               yaw_rate_rads_from_wv, default_mode);
        }

        control::HeadingCommand heading{};
        heading.yaw_angle_rad = yaw_rad();
        heading.yaw_rate_rads = rate_rads();

        switch (mode_) {
        case Mode::HOLD:
        case Mode::RATE:
        case Mode::PILOT_RATE:
        case Mode::WEATHERVANE:
            heading.heading_mode = control::HeadingMode::Rate_Only;
            break;
        case Mode::LOOK_AT_NEXT_WP:
        case Mode::ROI:
        case Mode::FIXED:
        case Mode::LOOK_AHEAD:
        case Mode::RESET_TO_ARMED_YAW:
        case Mode::ANGLE_RATE:
        case Mode::CIRCLE:
            // Upstream assigns Angle_And_Rate. Full yaw_rad/rate bodies
            // for these cases remain later; heading_mode matches the
            // upstream switch so get_heading compiles.
            heading.heading_mode = control::HeadingMode::Angle_And_Rate;
            break;
        }
        return heading;
    }

private:
    Mode mode_{Mode::LOOK_AT_NEXT_WP};
    Mode last_mode_{Mode::LOOK_AT_NEXT_WP};
    float yaw_angle_rad_{0.0f};
    float yaw_rate_rads_{0.0f};
    float pilot_yaw_rate_rads_{0.0f};
    bool weathervane_invoked_{false};
    bool used_default_mode_leftover_{false};
};

}  // namespace fwcpp::copter
