#pragma once

#include <fwcpp/q_loiter/q_loiter_defaults.hpp>

#include <cstdint>

namespace fwcpp::q_loiter {

struct QLoiterPreclandInputs {
    std::uint32_t now_ms{0};
    std::uint32_t last_target_loc_set_ms{0};
    std::uint32_t last_velocity_match_ms{0};
    bool rel_origin_valid{false};
    float rel_origin_n_m{0.0F};
    float rel_origin_e_m{0.0F};
    float velocity_match_n_ms{0.0F};
    float velocity_match_e_ms{0.0F};
};

struct QLoiterPreclandEffects {
    bool apply_pos_override{false};
    float pos_desired_n_m{0.0F};
    float pos_desired_e_m{0.0F};
    bool clear_target_loc_set_ms{false};
    bool apply_vel_override{false};
    float vel_n_ms{0.0F};
    float vel_e_ms{0.0F};
    bool clear_velocity_match_ms{false};
};

[[nodiscard]] inline QLoiterPreclandEffects qloiter_precland_effects(const QLoiterPreclandInputs& in) {
    QLoiterPreclandEffects out{};
    if (in.last_target_loc_set_ms != 0 &&
        in.now_ms - in.last_target_loc_set_ms < kPreclandOverrideTimeoutMs && in.rel_origin_valid) {
        out.apply_pos_override = true;
        out.pos_desired_n_m = in.rel_origin_n_m;
        out.pos_desired_e_m = in.rel_origin_e_m;
        out.clear_target_loc_set_ms = true;
    }
    if (in.last_velocity_match_ms != 0 &&
        in.now_ms - in.last_velocity_match_ms < kPreclandOverrideTimeoutMs) {
        out.apply_vel_override = true;
        out.vel_n_ms = in.velocity_match_n_ms;
        out.vel_e_ms = in.velocity_match_e_ms;
        out.clear_velocity_match_ms = true;
    }
    return out;
}

}  // namespace fwcpp::q_loiter
