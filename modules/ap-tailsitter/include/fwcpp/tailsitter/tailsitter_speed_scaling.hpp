#pragma once

// Tailsitter::speed_scaling (Plane-4.7.0 tailsitter.cpp 644-798) and
// Tailsitter::relax_pitch (822-825). SRV / motors / logger side effects
// are returned; the caller stores last_spd_scaler and applies outputs.

#include <algorithm>
#include <cstdint>
#include <optional>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_output.hpp>

namespace fwcpp::tailsitter {

// AP_Math/definitions.h — not in ap-math; defined locally (ADR-0012).
inline constexpr float kSslAirDensity = 1.225f;
inline constexpr float kGravityMss = 9.80665f;

// cosf(.125f * M_PI) — upstream tailsitter.cpp ATT_THR trans_angle.
inline constexpr float kAttThrCTransAngle = 0.9238795f;
inline constexpr float kAttThrPosTc = 2.0f;
inline constexpr float kAttThrNegTc = 1.0f;

struct SpeedScalingInputs {
    float hover_throttle{0.35f};
    float throttle_out{0.0f};
    std::uint16_t gain_scaling_mask{0};
    float gain_scaling_min{kGainScalingMinDefault};
    float throttle_scale_max{kThrottleScaleMaxDefault};
    float disk_loading{0.0f};
    float disk_loading_min_outflow{0.0f};
    float body_to_ned_c_z{1.0f};
    float G_Dt{0.0025f};
    float last_spd_scaler{1.0f};
    std::optional<float> airspeed_eas{};
    float air_density_ratio{1.0f};
    std::optional<math::Vector3f> velocity_ned{};
    math::Vector3f wind_ned{};
    math::Matrix3f rotation_body_to_ned{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float aileron{0.0f};
    float elevator{0.0f};
    float rudder{0.0f};
    float tilt_left{0.0f};
    float tilt_right{0.0f};
};

struct SpeedScalingResult {
    float aileron{0.0f};
    float elevator{0.0f};
    float rudder{0.0f};
    float tilt_left{0.0f};
    float tilt_right{0.0f};
    float last_spd_scaler{1.0f};
    float min_throttle{0.0f};
    float throttle_scaler{1.0f};
    float speed_scaler{1.0f};
};

[[nodiscard]] inline constexpr bool relax_pitch(bool enabled, bool is_vectored,
                                                std::uint32_t vtol_limit_start_ms) {
    return !enabled || !is_vectored || (vtol_limit_start_ms != 0);
}

[[nodiscard]] inline SpeedScalingResult speed_scaling(const SpeedScalingInputs& in) {
    const float hover_throttle = in.hover_throttle;
    const float throttle = in.throttle_out;
    float spd_scaler = 1.0f;
    float disk_loading_min_throttle = 0.0f;
    float last_spd_scaler = in.last_spd_scaler;

    const float throttle_scaler = hover_throttle_scaler(
        hover_throttle, throttle, in.throttle_scale_max, in.gain_scaling_min);

    if ((in.gain_scaling_mask & kTailsitterGsclAttThr) != 0) {
        const float min_scale = in.gain_scaling_min;
        float tthr = 1.25f * hover_throttle;

        const float alpha = (1.0f - min_scale) / kAttThrCTransAngle;
        const float beta = 1.0f - alpha * kAttThrCTransAngle;

        const float c_tilt = in.body_to_ned_c_z;
        if (c_tilt < kAttThrCTransAngle) {
            spd_scaler = math::constrain_value(beta + alpha * c_tilt, min_scale, 1.0f);
            tthr = 0.5f * hover_throttle;
        }
        if (throttle > tthr) {
            const float throttle_atten = 1.0f - (throttle - tthr) / (1.0f - tthr);
            spd_scaler *= throttle_atten;
            spd_scaler = math::constrain_value(spd_scaler, min_scale, 1.0f);
        }

        const float posdelta = in.G_Dt / kAttThrPosTc;
        const float negdelta = in.G_Dt / kAttThrNegTc;
        spd_scaler = math::constrain_value(spd_scaler, last_spd_scaler - negdelta,
                                           last_spd_scaler + posdelta);
        last_spd_scaler = spd_scaler;

        if ((spd_scaler >= 1.0f) && ((in.gain_scaling_mask & kTailsitterGsclThrottle) != 0)) {
            spd_scaler = std::max(throttle_scaler, 1.0f);
        }
    } else if (((in.gain_scaling_mask & kTailsitterGsclDiskTheory) != 0) &&
               math::is_positive(in.disk_loading)) {
        if (!in.airspeed_eas.has_value()) {
            spd_scaler = throttle_scaler;
        } else {
            const float airspeed = *in.airspeed_eas;
            const float rho = kSslAirDensity * in.air_density_ratio;
            float hover_rho = rho;
            if ((in.gain_scaling_mask & kTailsitterGsclAltitude) != 0) {
                hover_rho = kSslAirDensity;
            }

            const float sq_hover_outflow = (in.disk_loading * kGravityMss) / (0.5f * hover_rho);
            const float airspeed_pos = std::max(airspeed, 0.0f);
            const float sq_outflow =
                (((throttle / hover_throttle) * in.disk_loading * kGravityMss) / (0.5f * rho)) +
                (airspeed_pos * airspeed_pos);

            spd_scaler = in.throttle_scale_max;
            if (math::is_positive(sq_outflow)) {
                spd_scaler = math::constrain_value(sq_hover_outflow / sq_outflow, in.gain_scaling_min,
                                                   in.throttle_scale_max);
            }

            if (math::is_positive(in.disk_loading_min_outflow)) {
                if (math::is_positive(airspeed)) {
                    disk_loading_min_throttle =
                        (((in.disk_loading_min_outflow * in.disk_loading_min_outflow -
                           airspeed * airspeed) *
                          (0.5f * rho)) /
                         (in.disk_loading * kGravityMss)) *
                        hover_throttle;
                } else {
                    float reverse_airspeed = 0.0f;
                    if (in.velocity_ned.has_value()) {
                        const math::Vector3f rel = *in.velocity_ned - in.wind_ned;
                        reverse_airspeed = in.rotation_body_to_ned.mul_transpose(rel).x;
                    }
                    reverse_airspeed = std::min(reverse_airspeed, 0.0f);
                    disk_loading_min_throttle =
                        (((in.disk_loading_min_outflow * in.disk_loading_min_outflow +
                           reverse_airspeed * reverse_airspeed) *
                          (0.5f * rho)) /
                         (in.disk_loading * kGravityMss)) *
                        hover_throttle;
                }
                disk_loading_min_throttle = std::max(disk_loading_min_throttle, 0.0f);
            }
        }
    } else if ((in.gain_scaling_mask & kTailsitterGsclThrottle) != 0) {
        spd_scaler = throttle_scaler;
    }

    if ((in.gain_scaling_mask & kTailsitterGsclAltitude) != 0) {
        spd_scaler /= in.air_density_ratio;
    }

    SpeedScalingResult out{};
    out.aileron = in.aileron * spd_scaler;
    out.elevator = in.elevator * spd_scaler;
    out.rudder = in.rudder * spd_scaler;
    out.tilt_left = in.tilt_left * throttle_scaler;
    out.tilt_right = in.tilt_right * throttle_scaler;
    out.last_spd_scaler = last_spd_scaler;
    out.min_throttle = disk_loading_min_throttle;
    out.throttle_scaler = throttle_scaler;
    out.speed_scaler = spd_scaler;
    return out;
}

}  // namespace fwcpp::tailsitter
