#pragma once

#include <cstdint>

namespace fwcpp::quadplane {

/// Upstream QuadPlane::position_control_state / QPOS_*.
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

/// QuadPlane-side poscontrol fields cleared in mode_enter (not AC_PosControl).
struct PosControlState {
    PositionControlState state{PositionControlState::kNone};
    float correction_north_m{0.f};
    float correction_east_m{0.f};
    float velocity_match_north_ms{0.f};
    float velocity_match_east_ms{0.f};
    std::uint32_t last_velocity_match_ms{0};
    bool pilot_correction_done{false};
    bool pilot_correction_active{false};
    float target_vel_north_ms{0.f};
    float target_vel_east_ms{0.f};
    float target_vel_down_ms{0.f};
    bool mode_enter_cleared{false};

    void reset_on_mode_enter() {
        state = PositionControlState::kNone;
        correction_north_m = 0.f;
        correction_east_m = 0.f;
        velocity_match_north_ms = 0.f;
        velocity_match_east_ms = 0.f;
        last_velocity_match_ms = 0;
        pilot_correction_done = false;
        pilot_correction_active = false;
        target_vel_north_ms = 0.f;
        target_vel_east_ms = 0.f;
        target_vel_down_ms = 0.f;
        mode_enter_cleared = true;
    }
};

}  // namespace fwcpp::quadplane
