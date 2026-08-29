#pragma once

#include <fwcpp/q_loiter/q_loiter_defaults.hpp>

#include <cstdint>

namespace fwcpp::q_loiter {

struct QLoiterEnterEffects {
    bool clear_pilot_desired_acceleration{false};
    bool loiter_init_target{false};
    bool set_pos_z_limits{false};
    bool init_throttle_wait{false};
    bool clear_precland_target_ms{false};
};

struct QLoiterEnterResult {
    bool entered{true};
    std::uint32_t last_loiter_ms{0};
};

struct QLoiterEnterInputs {
    float pilot_velocity_z_max_dn_ms{1.0F};
    float pilot_speed_z_max_up_ms{2.5F};
    float pilot_accel_z_mss{1.0F};
    std::uint32_t now_ms{0};
};

[[nodiscard]] inline QLoiterEnterResult qloiter_enter(const QLoiterEnterInputs& in,
                                                      QLoiterEnterEffects& effects) {
    effects = QLoiterEnterEffects{};
    effects.clear_pilot_desired_acceleration = true;
    effects.loiter_init_target = true;
    effects.set_pos_z_limits = true;
    effects.init_throttle_wait = true;
    effects.clear_precland_target_ms = true;

    QLoiterEnterResult out{};
    out.last_loiter_ms = in.now_ms;
    return out;
}

}  // namespace fwcpp::q_loiter
