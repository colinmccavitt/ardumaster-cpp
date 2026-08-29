#pragma once

#include "vtol_assist.hpp"

#include <cmath>
#include <cstdint>

namespace fwcpp::vtol_assist {

inline constexpr float kAllowedEnvelopeErrorDeg = 5.0f;

struct SpeedAssistSample {
    float aspeed{};
    bool have_airspeed{false};
    bool disable_synthetic_airspeed_assist{false};
    bool using_airspeed_sensor{false};
};

[[nodiscard]] inline bool evaluate_speed_assist(float assist_speed, SpeedAssistSample sample) {
    if (assist_speed <= 0.0f) {
        return false;
    }
    const bool below_speed = sample.have_airspeed && sample.aspeed < assist_speed;
    const bool synthetic_ok =
        !sample.disable_synthetic_airspeed_assist || sample.using_airspeed_sensor;
    return below_speed && synthetic_ok;
}

[[nodiscard]] inline bool force_assist_active(AssistState state) {
    return state == AssistState::kForceEnabled;
}

[[nodiscard]] inline bool angle_check_enabled(const VtolAssist& assist) {
    return assist.speed_checks_enabled() && assist.angle() > 0;
}

[[nodiscard]] inline bool alt_check_enabled(const VtolAssist& assist) {
    return assist.speed_checks_enabled() && assist.alt() > 0;
}

[[nodiscard]] inline bool evaluate_alt_assist_trigger(const VtolAssist& assist, float height_agl_m) {
    if (!alt_check_enabled(assist)) {
        return false;
    }
    return height_agl_m < static_cast<float>(assist.alt());
}

[[nodiscard]] inline bool inside_attitude_envelope(float ahrs_roll_deg, float ahrs_pitch_deg,
                                                   float roll_limit_deg, float pitch_limit_max_deg,
                                                   float pitch_limit_min_deg) {
    return std::fabs(ahrs_roll_deg) <= (roll_limit_deg + kAllowedEnvelopeErrorDeg) &&
           ahrs_pitch_deg < (pitch_limit_max_deg + kAllowedEnvelopeErrorDeg) &&
           ahrs_pitch_deg > (pitch_limit_min_deg - kAllowedEnvelopeErrorDeg);
}

[[nodiscard]] inline bool inside_angle_error(float ahrs_roll_deg, float ahrs_pitch_deg,
                                             float nav_roll_deg, float nav_pitch_deg,
                                             std::int8_t angle_deg) {
    const float limit = static_cast<float>(angle_deg);
    return std::fabs(ahrs_roll_deg - nav_roll_deg) < limit &&
           std::fabs(ahrs_pitch_deg - nav_pitch_deg) < limit;
}

[[nodiscard]] inline bool evaluate_angle_assist_trigger(const VtolAssist& assist, float ahrs_roll_deg,
                                                        float ahrs_pitch_deg, float nav_roll_deg,
                                                        float nav_pitch_deg, float roll_limit_deg,
                                                        float pitch_limit_max_deg,
                                                        float pitch_limit_min_deg) {
    if (!angle_check_enabled(assist)) {
        return false;
    }
    const bool in_envelope =
        inside_attitude_envelope(ahrs_roll_deg, ahrs_pitch_deg, roll_limit_deg, pitch_limit_max_deg,
                                 pitch_limit_min_deg);
    const bool in_angle = inside_angle_error(ahrs_roll_deg, ahrs_pitch_deg, nav_roll_deg, nav_pitch_deg,
                                             assist.angle());
    return !in_envelope && !in_angle;
}

[[nodiscard]] inline bool evaluate_angle_assist_trigger(const VtolAssist& assist,
                                                        std::int32_t nav_roll_cd, std::int32_t nav_pitch_cd,
                                                        float ahrs_roll_deg, float ahrs_pitch_deg,
                                                        float roll_limit_deg, float pitch_limit_max_deg,
                                                        float pitch_limit_min_deg) {
    return evaluate_angle_assist_trigger(
        assist, ahrs_roll_deg, ahrs_pitch_deg, nav_roll_cd * 0.01f, nav_pitch_cd * 0.01f,
        roll_limit_deg, pitch_limit_max_deg, pitch_limit_min_deg);
}

}  // namespace fwcpp::vtol_assist
