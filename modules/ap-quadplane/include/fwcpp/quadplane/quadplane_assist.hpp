#pragma once

// QuadPlane assist climb / weathervane yaw / is_flying_vtol — Plane-4.7.0
// ArduPlane/quadplane.cpp: is_flying_vtol (1234-1263),
// assist_climb_rate_cms (1434-1455), desired_auto_yaw_rate_cds (1460-1473),
// get_weathervane_yaw_rate_cds (3907-3941).
//
// ADR-0012: header-only ticks. last_pidz_* on ZCtrlState; land detect
// lower_limit_start_ms on PosControlLandStub. Do not re-port
// get_desired_yaw_rate_cds / should_relax / land_detector. Do not port
// AC_WeatherVane — inject get_yaw_out success + wv_output.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/quadplane/quadplane_auto_vtol_mission.hpp>
#include <fwcpp/quadplane/quadplane_land_detector.hpp>
#include <fwcpp/quadplane/quadplane_landing.hpp>
#include <fwcpp/quadplane/quadplane_pilot_input.hpp>
#include <fwcpp/quadplane/quadplane_stabilize.hpp>
#include <fwcpp/quadplane/quadplane_throttle.hpp>
#include <fwcpp/quadplane_transition/transition_base.hpp>

namespace fwcpp::quadplane {

inline constexpr float kFlyingVtolThrottle = 0.01f;
inline constexpr std::uint32_t kFlyingVtolLandDetectMs = 5000;
inline constexpr std::uint32_t kAssistClimbRampMs = 2000;
inline constexpr float kAssistClimbAltSpread = 0.1f;
inline constexpr float kFlybywireClimbRateDefault = 2.0f;
inline constexpr float kPitchLimitMaxDefault = 25.0f;
inline constexpr float kAirspeedMinDefault = 9.0f;
inline constexpr float kAssistMinAspeedMs = 1.0f;
inline constexpr float kWeathervaneScale = 1.0f / 45.0f;
inline constexpr float kWeathervaneOutLimit = 100.0f;
inline constexpr float kWeathervaneRateFrac = 0.5f;

struct FlyingVtolInputs {
    bool available{false};
    SpoolState spool{SpoolState::kShutDown};
    float motors_throttle{0.f};
    bool is_vtol_man_throttle{false};
    bool air_mode_active{false};
    bool mode_guided{false};
    bool guided_takeoff{false};
    bool is_vtol_man_mode{false};
    float throttle_control_in{0.f};
    bool reversed_throttle{false};
    bool in_vtol_mode{false};
    std::uint32_t now_ms{0};
};

struct AssistClimbInputs {
    bool does_auto_throttle{false};
    float altitude_error_cm{0.f};
    float flybywire_climb_rate{kFlybywireClimbRateDefault};
    float nav_pitch_cd{0.f};
    float pitch_limit_max{kPitchLimitMaxDefault};
    float throttle_control_in{0.f};
    bool reversed_throttle{false};
    float default_speed_down_ms{kDefaultSpeedDownMs};
    float default_speed_up_ms{kDefaultSpeedUpMs};
};

struct AutoYawRateInputs {
    bool have_airspeed{false};
    float aspeed{0.f};
    float airspeed_min{kAirspeedMinDefault};
    float nav_roll_cd{0.f};
};

struct WeathervaneStub {
    bool reset{false};
};

struct WeathervaneYawInputs {
    bool in_vtol_mode{false};
    bool allow_weathervane{fwcpp::quadplane_transition::TransitionBaseDefaults::allow_weathervane()};
    bool armed{false};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool mode_qstabilize{false};
    bool in_qautotune{false};
    bool mode_qhover{false};
    ShouldRelaxInputs relax{};
    bool in_vtol_auto{false};
    bool available{false};
    std::int32_t options{0};
    std::uint16_t nav_cmd_id{0};
    bool weathervane_ok{false};
    float wv_output{0.f};
    float command_model_pilot_rate{kCommandModelPilotRateDefault};
};

struct WeathervaneYawTick {
    float rate_cds{0.f};
    bool reset{false};
    bool is_takeoff{false};
};

[[nodiscard]] inline bool is_flying_vtol(const PosControlLandStub& land, const FlyingVtolInputs& in) {
    if (!in.available) {
        return false;
    }
    if (in.spool == SpoolState::kShutDown) {
        return false;
    }
    if (in.motors_throttle > kFlyingVtolThrottle) {
        return true;
    }
    if (in.is_vtol_man_throttle && in.air_mode_active) {
        return true;
    }
    if (in.mode_guided && in.guided_takeoff) {
        return true;
    }
    if (in.is_vtol_man_mode) {
        return fwcpp::math::is_positive(get_throttle_input(in.throttle_control_in, in.reversed_throttle));
    }
    if (in.in_vtol_mode && (in.now_ms - land.lower_limit_start_ms) > kFlyingVtolLandDetectMs) {
        return true;
    }
    return false;
}

[[nodiscard]] inline float assist_climb_rate_cms(const ZCtrlState& z, const AssistClimbInputs& in) {
    float climb_rate_cms;
    if (in.does_auto_throttle) {
        climb_rate_cms = in.altitude_error_cm * kAssistClimbAltSpread;
    } else {
        climb_rate_cms = in.flybywire_climb_rate * (in.nav_pitch_cd / (in.pitch_limit_max * 100.0f));
        climb_rate_cms *= get_throttle_input(in.throttle_control_in, in.reversed_throttle);
    }
    climb_rate_cms = fwcpp::math::constrain_value(climb_rate_cms, -in.default_speed_down_ms * 100.0f,
                                                  in.default_speed_up_ms * 100.0f);

    const std::uint32_t dt_since_start = z.last_pidz_active_ms - z.last_pidz_init_ms;
    if (dt_since_start < kAssistClimbRampMs) {
        climb_rate_cms = fwcpp::math::linear_interpolate(0.0f, climb_rate_cms,
                                                         static_cast<float>(dt_since_start), 0.0f,
                                                         static_cast<float>(kAssistClimbRampMs));
    }
    return climb_rate_cms;
}

[[nodiscard]] inline float desired_auto_yaw_rate_cds(bool body_frame, const AutoYawRateInputs& in) {
    float aspeed = (in.have_airspeed && in.aspeed >= in.airspeed_min) ? in.aspeed : in.airspeed_min;
    if (aspeed < kAssistMinAspeedMs) {
        aspeed = kAssistMinAspeedMs;
    }
    const float roll_rad = fwcpp::math::cd_to_rad(in.nav_roll_cd);
    if (body_frame) {
        return fwcpp::math::degrees(kGravityMss * std::sin(roll_rad) / aspeed) * 100.0f;
    }
    return fwcpp::math::degrees(kGravityMss * std::tan(roll_rad) / aspeed) * 100.0f;
}

inline WeathervaneYawTick get_weathervane_yaw_rate_cds(WeathervaneStub& wv, PosControlLandStub& land,
                                                       const WeathervaneYawInputs& in) {
    WeathervaneYawTick tick{};
    bool gated = !in.in_vtol_mode || !in.allow_weathervane || !in.armed ||
                 (in.desired_spool != DesiredSpoolState::kThrottleUnlimited) || in.mode_qstabilize ||
                 in.in_qautotune || in.mode_qhover;
    if (!gated) {
        gated = should_relax(land, in.relax);
    }
    if (gated) {
        wv.reset = true;
        tick.reset = true;
        return tick;
    }

    tick.is_takeoff = in.in_vtol_auto && is_vtol_takeoff(in.nav_cmd_id, in.available, in.options);
    if (in.weathervane_ok) {
        tick.rate_cds = fwcpp::math::constrain_value(in.wv_output * kWeathervaneScale, -kWeathervaneOutLimit,
                                                     kWeathervaneOutLimit) *
                        in.command_model_pilot_rate * kWeathervaneRateFrac;
    }
    wv.reset = false;
    return tick;
}

}  // namespace fwcpp::quadplane
