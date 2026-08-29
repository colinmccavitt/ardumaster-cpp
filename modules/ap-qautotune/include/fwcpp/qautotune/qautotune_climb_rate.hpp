#pragma once

namespace fwcpp::qautotune {

[[nodiscard]] inline float qautotune_desired_climb_rate_ms(float pilot_desired_climb_rate_cms) {
    return pilot_desired_climb_rate_cms * 0.01f;
}

}  // namespace fwcpp::qautotune
