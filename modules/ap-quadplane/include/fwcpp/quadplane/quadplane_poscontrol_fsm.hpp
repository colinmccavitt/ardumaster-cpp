#pragma once

#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

#include <cstdint>

namespace fwcpp::quadplane {

/// Side-effects upstream `PosControlState::set_state` would drive (COP / logging stubs).
struct PosControlSetStateSink {
    bool reset_yaw_target{false};
    bool clear_d_integrator{false};
    bool thr_ctrl_land{false};
    bool latch_land_descend_start_alt_m{false};
    float land_descend_start_alt_m{0.f};
    bool reset_landing_detect{false};
    std::uint8_t qpos_log_writes{0};
};

/// QuadPlane landing fields touched by QPOS `set_state` (not AC_PosControl).
struct PosControlLandStub {
    bool thr_ctrl_land{false};
    float land_descend_start_alt_m{0.f};
    std::uint32_t land_start_ms{0};
    std::uint32_t lower_limit_start_ms{0};
};

struct PosControlSetStateInputs {
    std::uint32_t now_ms{0};
    float groundspeed_ms{0.f};
    float current_alt_m{0.f};
    std::uint16_t ahrs_position_ne_reset_count{0};
};

inline void poscontrol_apply_set_state(PosControlState& pc,
                                       PositionControlState to,
                                       const PosControlSetStateInputs& in,
                                       PosControlSetStateSink& sink,
                                       PosControlLandStub& land) {
    if (pc.state != to) {
        pc.pilot_correction_done = false;
        switch (to) {
            case PositionControlState::kPosition1:
                pc.reached_wp_speed = false;
                sink.reset_yaw_target = true;
                pc.pos1_speed_limit_ms = in.groundspeed_ms;
                pc.done_accel_init = false;
                break;
            case PositionControlState::kAirbrake:
                sink.clear_d_integrator = true;
                break;
            case PositionControlState::kLandDescend:
                land.thr_ctrl_land = false;
                sink.thr_ctrl_land = false;
                sink.latch_land_descend_start_alt_m = true;
                sink.land_descend_start_alt_m = in.current_alt_m;
                pc.last_override_descent_ms = 0;
                break;
            case PositionControlState::kLandAbort:
                land.thr_ctrl_land = false;
                sink.thr_ctrl_land = false;
                break;
            case PositionControlState::kLandFinal:
                pc.ahrs_position_ne_reset_count = in.ahrs_position_ne_reset_count;
                sink.reset_landing_detect = true;
                break;
            default:
                break;
        }
        sink.qpos_log_writes = static_cast<std::uint8_t>(sink.qpos_log_writes + 2);
        pc.state = to;
        pc.overshoot = false;
    }
    pc.last_state_change_ms = in.now_ms;
    pc.last_run_ms = in.now_ms;

    if (sink.latch_land_descend_start_alt_m) {
        land.land_descend_start_alt_m = sink.land_descend_start_alt_m;
    }
    if (sink.reset_landing_detect) {
        land.land_start_ms = 0;
        land.lower_limit_start_ms = 0;
    }
}

[[nodiscard]] inline std::uint32_t poscontrol_time_since_state_start_ms(const PosControlState& pc,
                                                                          std::uint32_t now_ms) {
    return now_ms - pc.last_state_change_ms;
}

}  // namespace fwcpp::quadplane
