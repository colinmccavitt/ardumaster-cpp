#pragma once

#include "vtol_assist.hpp"

namespace fwcpp::vtol_assist {

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

}  // namespace fwcpp::vtol_assist
