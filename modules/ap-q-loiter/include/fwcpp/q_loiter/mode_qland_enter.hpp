#pragma once

#include <fwcpp/q_loiter/mode_qloiter_enter.hpp>

#include <cstdint>

namespace fwcpp::q_loiter {

enum class QPosLandState : std::uint8_t {
    kLandDescend = 0,
    kLandAbort = 1,
    kLandFinal = 2,
    kLandComplete = 3,
};

struct QLandEnterEffects {
    QLoiterEnterEffects qloiter{};
    bool throttle_wait_false{false};
    bool setup_target_position{false};
    bool poscontrol_land_descend{false};
    bool capture_land_final_agl{false};
    bool reset_landing_detect_timers{false};
    bool deploy_landing_gear{false};
};

struct QLandEnterInputs {
    QLoiterEnterInputs qloiter{};
    float relative_ground_alt_m{0.0F};
    bool landing_gear_enabled{false};
};

struct QLandEnterResult {
    bool entered{true};
    QPosLandState pos_state{QPosLandState::kLandDescend};
    float last_land_final_agl_m{0.0F};
};

[[nodiscard]] inline QLandEnterResult qland_enter(const QLandEnterInputs& in,
                                                  QLandEnterEffects& effects) {
    effects = QLandEnterEffects{};
    (void)qloiter_enter(in.qloiter, effects.qloiter);
    effects.throttle_wait_false = true;
    effects.setup_target_position = true;
    effects.poscontrol_land_descend = true;
    effects.capture_land_final_agl = true;
    effects.reset_landing_detect_timers = true;
    if (in.landing_gear_enabled) {
        effects.deploy_landing_gear = true;
    }

    QLandEnterResult out{};
    out.last_land_final_agl_m = in.relative_ground_alt_m;
    return out;
}

}  // namespace fwcpp::q_loiter
