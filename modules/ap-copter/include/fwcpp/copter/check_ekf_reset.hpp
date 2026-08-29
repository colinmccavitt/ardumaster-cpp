#pragma once

// Copter::check_ekf_reset leftover. Upstream ArduCopter/ekf_check.cpp
// ~248-266. No AHRS / attitude_control / GCS / logger objects — inject
// the getLastYawResetAngle pair and get_primary_core_index; emit flags.
//
// yaw_angle_change_rad is the AHRS out-param. This leftover does not
// apply it (upstream only uses the timestamp). check_vibration is a
// later leftover.

#include <cstdint>

namespace fwcpp::copter {

struct CheckEkfResetInputs {
    std::uint32_t last_yaw_reset_ms{0};
    std::uint32_t new_ekf_yaw_reset_ms{0};
    float yaw_angle_change_rad{0};
    std::int8_t primary_core_index{0};
    std::int8_t new_primary_core_index{-1};
};

struct CheckEkfResetEffects {
    bool inertial_frame_reset{false};
    std::uint32_t last_yaw_reset_ms{0};
    std::int8_t primary_core_index{0};
    bool log_ekf_yaw_reset{false};
    bool log_ekf_primary_error{false};
    bool gcs_text{false};
};

[[nodiscard]] inline constexpr CheckEkfResetEffects check_ekf_reset(
    const CheckEkfResetInputs& in) {
    CheckEkfResetEffects fx{};
    fx.last_yaw_reset_ms = in.last_yaw_reset_ms;
    fx.primary_core_index = in.primary_core_index;

    if (in.new_ekf_yaw_reset_ms != in.last_yaw_reset_ms) {
        fx.inertial_frame_reset = true;
        fx.last_yaw_reset_ms = in.new_ekf_yaw_reset_ms;
        fx.log_ekf_yaw_reset = true;
    }

    if ((in.new_primary_core_index != in.primary_core_index) &&
        (in.new_primary_core_index != static_cast<std::int8_t>(-1))) {
        fx.inertial_frame_reset = true;
        fx.primary_core_index = in.new_primary_core_index;
        fx.log_ekf_primary_error = true;
        fx.gcs_text = true;
    }

    return fx;
}

}  // namespace fwcpp::copter
