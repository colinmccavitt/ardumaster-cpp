#pragma once

#include <fwcpp/qautotune/qautotune_defaults.hpp>

namespace fwcpp::qautotune {

struct QAutotunePilotDesiredInputs {
    std::int16_t roll_control_in{0};
    std::int16_t pitch_control_in{0};
    std::int32_t nav_roll_cd{0};
    std::int32_t nav_pitch_cd{0};
    std::int32_t desired_yaw_rate_cds{0};
};

struct QAutotunePilotDesired {
    float roll_rad{0.0f};
    float pitch_rad{0.0f};
    float yaw_rate_rads{0.0f};
};

[[nodiscard]] inline QAutotunePilotDesired resolve_pilot_desired_rp_yrate(
    const QAutotunePilotDesiredInputs& in) {
    QAutotunePilotDesired out{};
    if (in.roll_control_in == 0 && in.pitch_control_in == 0) {
        out.roll_rad = 0.0f;
        out.pitch_rad = 0.0f;
    } else {
        out.roll_rad = static_cast<float>(in.nav_roll_cd) * kCentidegToRad;
        out.pitch_rad = static_cast<float>(in.nav_pitch_cd) * kCentidegToRad;
    }
    out.yaw_rate_rads = static_cast<float>(in.desired_yaw_rate_cds) * kCentidegToRad;
    return out;
}

}  // namespace fwcpp::qautotune
