#pragma once

#include <fwcpp/quadplane/quadplane_options.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

#include <cstdint>

namespace fwcpp::quadplane {

inline constexpr std::uint16_t kMavCmdNavVtolTakeoff = 84;
inline constexpr std::uint16_t kMavCmdNavTakeoff = 22;
inline constexpr std::uint16_t kMavCmdNavVtolLand = 85;
inline constexpr std::uint16_t kMavCmdNavPayloadPlace = 94;
inline constexpr std::uint16_t kMavCmdNavLand = 21;

enum class MavVtolState : std::uint8_t { kMc = 3, kFw = 4 };

[[nodiscard]] inline bool is_vtol_takeoff(std::uint16_t cmd_id, bool available, std::int32_t options) {
    if (cmd_id == kMavCmdNavVtolTakeoff) {
        return true;
    }
    if (cmd_id == kMavCmdNavTakeoff && available &&
        !option_is_set(options, QOption::kAllowFwTakeoff)) {
        return true;
    }
    return false;
}

[[nodiscard]] inline bool is_vtol_land(std::uint16_t cmd_id, bool available, std::int32_t options,
                                       bool spiral_vtol_landing = true) {
    if (cmd_id == kMavCmdNavVtolLand || cmd_id == kMavCmdNavPayloadPlace) {
        return spiral_vtol_landing;
    }
    if (cmd_id == kMavCmdNavLand && available && !option_is_set(options, QOption::kAllowFwLand)) {
        return true;
    }
    return false;
}

struct DoVtolTakeoffResult {
    bool ok{false};
    bool throttle_wait_cleared{false};
    std::uint32_t takeoff_start_time_ms{0};
    std::uint32_t takeoff_time_limit_ms{0};
};

struct DoVtolTakeoffInputs {
    bool setup_ok{false};
    bool respect_takeoff_frame{false};
    std::int32_t current_alt_cm{0};
    std::int32_t next_wp_alt_cm{0};
    std::uint32_t now_ms{0};
    float pilot_accel_z_mss{1.f};
    float pilot_speed_z_max_up_ms{2.f};
    float vel_u_ms{0.f};
    float takeoff_failure_scalar{1.f};
};

[[nodiscard]] inline DoVtolTakeoffResult do_vtol_takeoff(const DoVtolTakeoffInputs& in) {
    DoVtolTakeoffResult out{};
    if (!in.setup_ok) {
        return out;
    }
    if (in.respect_takeoff_frame && in.current_alt_cm >= in.next_wp_alt_cm) {
        return out;
    }
    out.throttle_wait_cleared = true;
    const float d_total_m = static_cast<float>(in.next_wp_alt_cm - in.current_alt_cm) * 0.01f;
    const float accel = in.pilot_accel_z_mss > 0.1f ? in.pilot_accel_z_mss : 0.1f;
    const float vel_max = in.pilot_speed_z_max_up_ms > 0.1f ? in.pilot_speed_z_max_up_ms : 0.1f;
    const float t_accel = (vel_max - in.vel_u_ms) / accel;
    const float d_accel = in.vel_u_ms * t_accel + 0.5f * accel * t_accel * t_accel;
    const float d_remaining = d_total_m - d_accel;
    const float travel_s = (t_accel > 0.f ? t_accel : 0.f) + (d_remaining > 0.f ? d_remaining / vel_max : 0.f);
    out.takeoff_start_time_ms = in.now_ms;
    const float limit_ms = travel_s * in.takeoff_failure_scalar * 1000.f;
    out.takeoff_time_limit_ms = limit_ms > 5000.f ? static_cast<std::uint32_t>(limit_ms) : 5000u;
    out.ok = true;
    return out;
}

struct DoVtolLandResult {
    bool ok{false};
    bool throttle_wait_cleared{false};
    bool crash_state_cleared{false};
    bool init_approach{false};
};

struct DoVtolLandInputs {
    bool setup_ok{false};
};

[[nodiscard]] inline DoVtolLandResult do_vtol_land(const DoVtolLandInputs& in) {
    DoVtolLandResult out{};
    if (!in.setup_ok) {
        return out;
    }
    out.throttle_wait_cleared = true;
    out.crash_state_cleared = true;
    out.init_approach = true;
    out.ok = true;
    return out;
}

struct VerifyVtolTakeoffInputs {
    bool available{false};
    bool armed_and_safety_off{false};
    std::uint32_t now_ms{0};
    std::uint32_t takeoff_start_time_ms{0};
    std::uint32_t takeoff_time_limit_ms{0};
    float takeoff_failure_scalar{1.f};
    float maximum_takeoff_airspeed_ms{0.f};
    float airspeed_ms{0.f};
    std::int32_t current_alt_cm{0};
    std::int32_t target_alt_cm{0};
    bool control_is_auto{false};
};

struct VerifyVtolTakeoffResult {
    bool complete{false};
    bool restart_do_takeoff{false};
    bool failed_takeoff{false};
    bool reset_tecs{false};
    bool next_wp_no_crosstrack{false};
};

[[nodiscard]] inline VerifyVtolTakeoffResult verify_vtol_takeoff(const VerifyVtolTakeoffInputs& in) {
    VerifyVtolTakeoffResult out{};
    if (!in.available) {
        out.complete = true;
        return out;
    }
    if (!in.armed_and_safety_off) {
        out.restart_do_takeoff = true;
        return out;
    }
    if (in.takeoff_failure_scalar > 0.f && in.now_ms - in.takeoff_start_time_ms > in.takeoff_time_limit_ms) {
        out.failed_takeoff = true;
        return out;
    }
    if (in.maximum_takeoff_airspeed_ms > 0.f && in.airspeed_ms > in.maximum_takeoff_airspeed_ms) {
        out.failed_takeoff = true;
        return out;
    }
    if (in.current_alt_cm < in.target_alt_cm) {
        return out;
    }
    if (in.control_is_auto) {
        out.reset_tecs = true;
    }
    out.next_wp_no_crosstrack = true;
    out.complete = true;
    return out;
}

struct VerifyVtolLandInputs {
    bool available{false};
    PositionControlState pos_state{PositionControlState::kNone};
};

[[nodiscard]] inline bool verify_vtol_land(const VerifyVtolLandInputs& in) {
    if (!in.available) {
        return true;
    }
    return in.pos_state == PositionControlState::kLandComplete;
}

struct HandleDoVtolTransitionInputs {
    bool available{false};
    bool control_is_auto{false};
    MavVtolState state{MavVtolState::kMc};
    bool auto_vtol_mode{false};
};

struct HandleDoVtolTransitionResult {
    bool ok{false};
    bool auto_vtol_mode{false};
    bool clear_fwd_throttle{false};
};

[[nodiscard]] inline HandleDoVtolTransitionResult handle_do_vtol_transition(const HandleDoVtolTransitionInputs& in) {
    HandleDoVtolTransitionResult out{.auto_vtol_mode = in.auto_vtol_mode};
    if (!in.available || !in.control_is_auto) {
        return out;
    }
    switch (in.state) {
        case MavVtolState::kMc:
            out.auto_vtol_mode = true;
            out.clear_fwd_throttle = true;
            out.ok = true;
            break;
        case MavVtolState::kFw:
            out.auto_vtol_mode = false;
            out.ok = true;
            break;
        default:
            break;
    }
    return out;
}

}  // namespace fwcpp::quadplane
