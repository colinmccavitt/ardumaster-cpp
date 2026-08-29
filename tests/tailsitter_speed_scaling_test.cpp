#include <cstdint>
#include <optional>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_output.hpp>
#include <fwcpp/tailsitter/tailsitter_speed_scaling.hpp>

using fwcpp::tailsitter::SpeedScalingInputs;
using fwcpp::tailsitter::hover_throttle_scaler;
using fwcpp::tailsitter::kAttThrCTransAngle;
using fwcpp::tailsitter::kAttThrNegTc;
using fwcpp::tailsitter::kGainScalingMinDefault;
using fwcpp::tailsitter::kTailsitterGsclAltitude;
using fwcpp::tailsitter::kTailsitterGsclAttThr;
using fwcpp::tailsitter::kTailsitterGsclDiskTheory;
using fwcpp::tailsitter::kTailsitterGsclThrottle;
using fwcpp::tailsitter::kThrottleScaleMaxDefault;
using fwcpp::tailsitter::relax_pitch;
using fwcpp::tailsitter::speed_scaling;

TEST_CASE("tailsitter relax_pitch three clauses", "[tailsitter][relax_pitch]") {
    REQUIRE(relax_pitch(false, true, 0));
    REQUIRE(relax_pitch(true, false, 0));
    REQUIRE(relax_pitch(true, true, 1));
    REQUIRE_FALSE(relax_pitch(true, true, 0));
}

TEST_CASE("tailsitter speed_scaling throttle-only mask", "[tailsitter][speed_scaling]") {
    SpeedScalingInputs in{};
    in.gain_scaling_mask = kTailsitterGsclThrottle;
    in.hover_throttle = 0.4f;
    in.throttle_out = 0.8f;
    in.aileron = 1000.0f;
    in.elevator = 2000.0f;
    in.rudder = -500.0f;
    in.tilt_left = 300.0f;
    in.tilt_right = -150.0f;

    const float expected = hover_throttle_scaler(0.4f, 0.8f, kThrottleScaleMaxDefault,
                                                 kGainScalingMinDefault);
    REQUIRE_THAT(expected, Catch::Matchers::WithinAbs(0.5f, 1e-6f));

    const auto out = speed_scaling(in);
    REQUIRE_THAT(out.throttle_scaler, Catch::Matchers::WithinAbs(expected, 1e-6f));
    REQUIRE_THAT(out.speed_scaler, Catch::Matchers::WithinAbs(expected, 1e-6f));
    REQUIRE_THAT(out.aileron, Catch::Matchers::WithinAbs(500.0f, 1e-4f));
    REQUIRE_THAT(out.elevator, Catch::Matchers::WithinAbs(1000.0f, 1e-4f));
    REQUIRE_THAT(out.rudder, Catch::Matchers::WithinAbs(-250.0f, 1e-4f));
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(150.0f, 1e-4f));
    REQUIRE_THAT(out.tilt_right, Catch::Matchers::WithinAbs(-75.0f, 1e-4f));
    REQUIRE_THAT(out.last_spd_scaler, Catch::Matchers::WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(out.min_throttle, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("tailsitter ATT_THR tilt atten and slew", "[tailsitter][speed_scaling]") {
    SpeedScalingInputs in{};
    in.gain_scaling_mask = kTailsitterGsclAttThr;
    in.hover_throttle = 0.5f;
    in.throttle_out = 0.2f;
    in.body_to_ned_c_z = 0.5f;
    in.gain_scaling_min = kGainScalingMinDefault;
    in.last_spd_scaler = 1.0f;
    in.G_Dt = 2.0f;
    in.aileron = 1000.0f;

    const float alpha = (1.0f - kGainScalingMinDefault) / kAttThrCTransAngle;
    const float beta = 1.0f - alpha * kAttThrCTransAngle;
    const float unslewed = beta + alpha * 0.5f;
    REQUIRE(unslewed < 1.0f);
    REQUIRE(unslewed > kGainScalingMinDefault);

    const auto full = speed_scaling(in);
    REQUIRE_THAT(full.speed_scaler, Catch::Matchers::WithinAbs(unslewed, 1e-5f));
    REQUIRE_THAT(full.last_spd_scaler, Catch::Matchers::WithinAbs(unslewed, 1e-5f));
    REQUIRE_THAT(full.aileron, Catch::Matchers::WithinAbs(1000.0f * unslewed, 1e-3f));

    in.G_Dt = 0.02f;
    in.last_spd_scaler = 1.0f;
    const float negdelta = 0.02f / kAttThrNegTc;
    const auto slewed = speed_scaling(in);
    REQUIRE_THAT(slewed.speed_scaler, Catch::Matchers::WithinAbs(1.0f - negdelta, 1e-6f));
    REQUIRE_THAT(slewed.last_spd_scaler, Catch::Matchers::WithinAbs(1.0f - negdelta, 1e-6f));

    in.last_spd_scaler = slewed.last_spd_scaler;
    const auto slewed2 = speed_scaling(in);
    REQUIRE_THAT(slewed2.last_spd_scaler,
                 Catch::Matchers::WithinAbs(1.0f - 2.0f * negdelta, 1e-6f));
    REQUIRE(slewed2.last_spd_scaler > unslewed);
}

TEST_CASE("tailsitter disk theory no-airspeed falls back to throttle_scaler",
          "[tailsitter][speed_scaling]") {
    SpeedScalingInputs in{};
    in.gain_scaling_mask = kTailsitterGsclDiskTheory;
    in.disk_loading = 4.0f;
    in.hover_throttle = 0.4f;
    in.throttle_out = 0.2f;
    in.airspeed_eas = std::nullopt;
    in.aileron = 800.0f;
    in.tilt_left = 400.0f;

    const float expected = hover_throttle_scaler(0.4f, 0.2f, kThrottleScaleMaxDefault,
                                                 kGainScalingMinDefault);
    REQUIRE_THAT(expected, Catch::Matchers::WithinAbs(2.0f, 1e-6f));

    const auto out = speed_scaling(in);
    REQUIRE_THAT(out.throttle_scaler, Catch::Matchers::WithinAbs(expected, 1e-6f));
    REQUIRE_THAT(out.speed_scaler, Catch::Matchers::WithinAbs(expected, 1e-6f));
    REQUIRE_THAT(out.aileron, Catch::Matchers::WithinAbs(1600.0f, 1e-4f));
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(800.0f, 1e-4f));
    REQUIRE_THAT(out.min_throttle, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("tailsitter GSCL_ALTITUDE divides spd_scaler by density", "[tailsitter][speed_scaling]") {
    SpeedScalingInputs in{};
    in.gain_scaling_mask = static_cast<std::uint16_t>(kTailsitterGsclThrottle | kTailsitterGsclAltitude);
    in.hover_throttle = 0.4f;
    in.throttle_out = 0.2f;
    in.air_density_ratio = 0.8f;
    in.aileron = 1000.0f;
    in.tilt_left = 1000.0f;

    const float throttle_scaler = hover_throttle_scaler(0.4f, 0.2f, kThrottleScaleMaxDefault,
                                                        kGainScalingMinDefault);
    const auto out = speed_scaling(in);
    REQUIRE_THAT(out.throttle_scaler, Catch::Matchers::WithinAbs(throttle_scaler, 1e-6f));
    REQUIRE_THAT(out.speed_scaler, Catch::Matchers::WithinAbs(throttle_scaler / 0.8f, 1e-5f));
    REQUIRE_THAT(out.aileron, Catch::Matchers::WithinAbs(1000.0f * throttle_scaler / 0.8f, 1e-3f));
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(1000.0f * throttle_scaler, 1e-3f));
}

TEST_CASE("tailsitter tilts use throttle_scaler surfaces use spd_scaler",
          "[tailsitter][speed_scaling]") {
    SpeedScalingInputs in{};
    in.gain_scaling_mask = kTailsitterGsclAttThr;
    in.hover_throttle = 0.5f;
    in.throttle_out = 0.2f;
    in.body_to_ned_c_z = 0.5f;
    in.G_Dt = 2.0f;
    in.last_spd_scaler = 1.0f;
    in.aileron = 1000.0f;
    in.elevator = -1000.0f;
    in.rudder = 500.0f;
    in.tilt_left = 1000.0f;
    in.tilt_right = -1000.0f;

    const float throttle_scaler = hover_throttle_scaler(0.5f, 0.2f, kThrottleScaleMaxDefault,
                                                        kGainScalingMinDefault);
    REQUIRE_THAT(throttle_scaler, Catch::Matchers::WithinAbs(2.0f, 1e-6f));

    const auto out = speed_scaling(in);
    REQUIRE_THAT(out.throttle_scaler, Catch::Matchers::WithinAbs(throttle_scaler, 1e-6f));
    REQUIRE(out.speed_scaler < 1.0f);
    REQUIRE(out.speed_scaler != out.throttle_scaler);
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(1000.0f * throttle_scaler, 1e-3f));
    REQUIRE_THAT(out.tilt_right, Catch::Matchers::WithinAbs(-1000.0f * throttle_scaler, 1e-3f));
    REQUIRE_THAT(out.aileron, Catch::Matchers::WithinAbs(1000.0f * out.speed_scaler, 1e-3f));
    REQUIRE_THAT(out.elevator, Catch::Matchers::WithinAbs(-1000.0f * out.speed_scaler, 1e-3f));
    REQUIRE_THAT(out.rudder, Catch::Matchers::WithinAbs(500.0f * out.speed_scaler, 1e-3f));
}
