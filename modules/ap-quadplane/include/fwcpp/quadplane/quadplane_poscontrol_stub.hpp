#pragma once

#include <cstdint>

namespace fwcpp::quadplane {

enum class PositionControlState : std::uint8_t {
    kNone = 0,
    kApproach = 1,
    kAirbrake = 2,
    kPosition1 = 3,
    kPosition2 = 4,
    kLandDescend = 5,
    kLandAbort = 6,
    kLandFinal = 7,
    kLandComplete = 8,
};

struct PosControlState {
    PositionControlState state{PositionControlState::kNone};
    float correction_north_m{0.f};
    float correction_east_m{0.f};
    float target_ned_n_m{0.f};
    float target_ned_e_m{0.f};
    float target_ned_d_m{0.f};
    float velocity_match_north_ms{0.f};
    float velocity_match_east_ms{0.f};
    std::uint32_t last_velocity_match_ms{0};
    bool pilot_correction_done{false};
    bool pilot_correction_active{false};
    float target_vel_north_ms{0.f};
    float target_vel_east_ms{0.f};
    float target_vel_down_ms{0.f};
    bool mode_enter_cleared{false};
    std::uint32_t last_state_change_ms{0};
    std::uint32_t last_run_ms{0};
    std::uint32_t last_log_ms{0};
    bool reached_wp_speed{false};
    float pos1_speed_limit_ms{0.f};
    bool done_accel_init{false};
    std::uint32_t thrust_loss_start_ms{0};
    bool slow_descent{false};
    std::uint16_t ahrs_position_ne_reset_count{0};
    bool overshoot{false};
    float override_descent_rate_ms{0.f};
    std::uint32_t last_override_descent_ms{0};
    std::uint32_t last_pos_reset_ms{0};

    void reset_on_mode_enter() {
        state = PositionControlState::kNone;
        correction_north_m = 0.f;
        correction_east_m = 0.f;
        target_ned_n_m = 0.f;
        target_ned_e_m = 0.f;
        target_ned_d_m = 0.f;
        velocity_match_north_ms = 0.f;
        velocity_match_east_ms = 0.f;
        last_velocity_match_ms = 0;
        pilot_correction_done = false;
        pilot_correction_active = false;
        target_vel_north_ms = 0.f;
        target_vel_east_ms = 0.f;
        target_vel_down_ms = 0.f;
        last_state_change_ms = 0;
        last_run_ms = 0;
        last_log_ms = 0;
        reached_wp_speed = false;
        pos1_speed_limit_ms = 0.f;
        done_accel_init = false;
        thrust_loss_start_ms = 0;
        slow_descent = false;
        ahrs_position_ne_reset_count = 0;
        overshoot = false;
        override_descent_rate_ms = 0.f;
        last_override_descent_ms = 0;
        last_pos_reset_ms = 0;
        mode_enter_cleared = true;
    }
};

}
