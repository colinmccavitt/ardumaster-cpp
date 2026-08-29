#pragma once

// QuadPlane takeoff / waypoint controllers — Plane-4.7.0 ArduPlane/quadplane.cpp:
// setup_target_position (3113-3135), takeoff_controller (3140-3259),
// waypoint_controller (3264-3303). control_auto is quadplane_control_auto.hpp.
//
// ADR-0012: no pos_control / wp_nav / attitude_control objects. Motors
// spool, origin, next_WP, corrections, tiltrotor fully_up, weathervane,
// and ESC-telem motor_check_passed are injected; results are flags and
// numbers the caller would apply. Pilot yaw / z-speed helpers come from
// quadplane_pilot_input.hpp (already ported).

#include <cstdint>
#include <optional>

#include <fwcpp/location.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/quadplane/quadplane_pilot_input.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

namespace fwcpp::quadplane {

inline constexpr std::uint32_t kTakeoffRudderWarningTimeoutMs = 3000;
inline constexpr std::uint32_t kTakeoffLastRunGapMs = 1000;
inline constexpr std::uint32_t kVelocityMatchFreshMs = 1000;
inline constexpr std::uint32_t kWaypointDestRefreshMs = 500;
inline constexpr std::int32_t kGuidedTakeoffAltMarginCm = 5;
inline constexpr float kTakeoffWpNavSpeedUpMsDefault = 2.5f;  // WPNAV_SPEED_UP

enum class TakeoffArmMethod : std::uint8_t {
    kOther = 0,
    kRudder = 1,
};

struct TakeoffNavState {
    std::uint32_t takeoff_start_time_ms{0};
    std::uint32_t takeoff_last_run_ms{0};
    float takeoff_start_alt_m{0.f};
    Location last_auto_target{};
    std::uint32_t last_loiter_ms{0};
    std::uint32_t rudder_takeoff_warn_ms{0};
};

struct SetupTargetPositionInputs {
    bool has_origin{false};
    Location origin{};
    Location next_wp{};
    float correction_north_m{0.f};
    float correction_east_m{0.f};
    bool in_vtol_land_approach{false};
    PositionControlState pos_state{PositionControlState::kNone};
    float pilot_speed_z_max_up_ms{kPilotSpeedZMaxUpMsDefault};
    float pilot_speed_z_max_dn_ms{kPilotSpeedZMaxDnMsDefault};
    float pilot_accel_z_mss{kPilotAccelZMssDefault};
};

struct SetupTargetPositionTick {
    bool spool_throttle_unlimited{false};
    float target_ned_n_m{0.f};
    float target_ned_e_m{0.f};
    float target_ned_d_m{0.f};
    float d_max_speed_dn_m{0.f};
    float d_max_speed_up_ms{0.f};
    float d_max_accel_z_mss{0.f};
    bool d_set_max_speed_accel{false};
    bool d_set_correction_speed_accel{false};
};

struct TakeoffControllerInputs {
    std::uint32_t now_ms{0};
    bool armed_and_safety_off{false};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool mode_is_guided{false};
    bool guided_takeoff{false};
    bool tiltrotor_enabled{false};
    bool tiltrotor_fully_up{true};
    std::optional<bool> motor_check_passed{};
    TakeoffArmMethod last_arm_method{TakeoffArmMethod::kOther};
    bool seen_neutral_rudder{true};
    SetupTargetPositionInputs target{};
    float velocity_match_north_ms{0.f};
    float velocity_match_east_ms{0.f};
    std::uint32_t last_velocity_match_ms{0};
    float takeoff_navalt_min_m{0.f};
    std::int32_t current_alt_cm{0};
    float pos_control_roll_cd{0.f};
    float pos_control_pitch_cd{0.f};
    float weathervane_yaw_rate_cds{0.f};
    PilotYawRateInputs pilot_yaw{};
    float wp_nav_default_speed_up_ms{kTakeoffWpNavSpeedUpMsDefault};
};

struct TakeoffControllerTick {
    float nav_roll_cd{0.f};
    float nav_pitch_cd{0.f};
    bool early_return{false};
    bool set_desired_spool{false};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool rudder_waiting{false};
    bool rudder_warn{false};
    SetupTargetPositionTick setup{};
    bool setup_target_position{false};
    float vel_ne_n_ms{0.f};
    float vel_ne_e_ms{0.f};
    bool no_navigation{false};
    bool ne_relax{false};
    bool input_vel_accel_ne{false};
    bool assign_tilt_to_fwd_thr{false};
    bool run_xy_controller{false};
    bool set_pilot_yaw_rate_time_constant{false};
    bool input_euler_rate_yaw{false};
    float euler_rate_yaw_cds{0.f};
    bool input_pos_vel_accel_d{false};
    float pos_d_m{0.f};
    float vel_d_ms{0.f};
    bool set_climb_rate{false};
    float climb_rate_ms{0.f};
    bool run_z_controller{false};
};

struct WaypointControllerInputs {
    std::uint32_t now_ms{0};
    SetupTargetPositionInputs target{};
    float wp_nav_roll_cd{0.f};
    float wp_nav_pitch_cd{0.f};
    float wp_nav_yaw_cd{0.f};
    bool vtol_roll_pitch_limited{false};
    float assist_climb_rate_cms{0.f};
};

struct WaypointControllerTick {
    SetupTargetPositionTick setup{};
    bool set_wp_destination_ned{false};
    float dest_ned_n_m{0.f};
    float dest_ned_e_m{0.f};
    float dest_ned_d_m{0.f};
    bool update_wpnav{false};
    float nav_roll_cd{0.f};
    float nav_pitch_cd{0.f};
    bool assign_tilt_to_fwd_thr{false};
    bool ne_set_externally_limited{false};
    bool disable_yaw_rate_time_constant{false};
    bool input_euler_angle_roll_pitch_yaw{false};
    float attitude_yaw_cd{0.f};
    bool set_climb_rate{false};
    float climb_rate_ms{0.f};
    bool run_z_controller{false};
};

[[nodiscard]] inline Location setup_origin_or_zero(const SetupTargetPositionInputs& in) {
    if (in.has_origin) {
        return in.origin;
    }
    Location origin{};
    origin.zero();
    return origin;
}

/// Unlimited spool unless (in_vtol_land_approach && state<=APPROACH)
/// or state==LAND_FINAL. Writes target_ned onto PosControlState.
[[nodiscard]] inline SetupTargetPositionTick setup_target_position(PosControlState& pc,
                                                                  const SetupTargetPositionInputs& in) {
    SetupTargetPositionTick tick{};
    const Location origin = setup_origin_or_zero(in);
    const bool skip_unlimited =
        (in.in_vtol_land_approach &&
         static_cast<std::uint8_t>(in.pos_state) <=
             static_cast<std::uint8_t>(PositionControlState::kApproach)) ||
        in.pos_state == PositionControlState::kLandFinal;
    if (!skip_unlimited) {
        tick.spool_throttle_unlimited = true;
    }

    const auto diff2d = origin.get_distance_NE(in.next_wp);
    tick.target_ned_n_m = diff2d.x + in.correction_north_m;
    tick.target_ned_e_m = diff2d.y + in.correction_east_m;
    tick.target_ned_d_m = -static_cast<float>(in.next_wp.alt - origin.alt) * 0.01f;
    pc.target_ned_n_m = tick.target_ned_n_m;
    pc.target_ned_e_m = tick.target_ned_e_m;
    pc.target_ned_d_m = tick.target_ned_d_m;

    tick.d_max_speed_dn_m = static_cast<float>(
        get_pilot_velocity_z_max_dn_m(in.pilot_speed_z_max_dn_ms, in.pilot_speed_z_max_up_ms));
    tick.d_max_speed_up_ms = in.pilot_speed_z_max_up_ms;
    tick.d_max_accel_z_mss = in.pilot_accel_z_mss;
    tick.d_set_max_speed_accel = true;
    tick.d_set_correction_speed_accel = true;
    return tick;
}

[[nodiscard]] inline TakeoffControllerTick takeoff_controller(TakeoffNavState& nav, PosControlState& pc,
                                                             const TakeoffControllerInputs& in) {
    TakeoffControllerTick tick{};
    tick.nav_roll_cd = 0.f;
    tick.nav_pitch_cd = 0.f;

    if (!in.armed_and_safety_off) {
        tick.early_return = true;
        return tick;
    }

    if (in.desired_spool != DesiredSpoolState::kThrottleUnlimited) {
        if (in.mode_is_guided && in.guided_takeoff && in.tiltrotor_enabled && !in.tiltrotor_fully_up) {
            nav.takeoff_start_time_ms = in.now_ms;
            tick.early_return = true;
            return tick;
        }
        if (in.motor_check_passed.has_value() && !(*in.motor_check_passed)) {
            tick.set_desired_spool = true;
            tick.desired_spool = DesiredSpoolState::kGroundIdle;
            nav.takeoff_start_time_ms = in.now_ms;
            tick.early_return = true;
            return tick;
        }
        if (in.last_arm_method == TakeoffArmMethod::kRudder &&
            (nav.takeoff_last_run_ms == 0 ||
             (in.now_ms - nav.takeoff_last_run_ms) > kTakeoffLastRunGapMs) &&
            !in.seen_neutral_rudder) {
            tick.set_desired_spool = true;
            tick.desired_spool = DesiredSpoolState::kGroundIdle;
            nav.takeoff_start_time_ms = in.now_ms;
            tick.rudder_waiting = true;
            if ((in.now_ms - nav.rudder_takeoff_warn_ms) > kTakeoffRudderWarningTimeoutMs) {
                tick.rudder_warn = true;
                nav.rudder_takeoff_warn_ms = in.now_ms;
            }
            tick.early_return = true;
            return tick;
        }
    }

    tick.setup = setup_target_position(pc, in.target);
    tick.setup_target_position = true;
    if (tick.setup.spool_throttle_unlimited) {
        tick.set_desired_spool = true;
        tick.desired_spool = DesiredSpoolState::kThrottleUnlimited;
    }

    if ((in.now_ms - in.last_velocity_match_ms) < kVelocityMatchFreshMs) {
        tick.vel_ne_n_ms = in.velocity_match_north_ms;
        tick.vel_ne_e_ms = in.velocity_match_east_ms;
    }

    if (in.takeoff_navalt_min_m > 0.f) {
        const float alt_m = static_cast<float>(in.current_alt_cm) * 0.01f;
        if (nav.takeoff_last_run_ms == 0 ||
            (in.now_ms - nav.takeoff_last_run_ms) > kTakeoffLastRunGapMs) {
            nav.takeoff_start_alt_m = alt_m;
        }
        if ((alt_m - nav.takeoff_start_alt_m) < in.takeoff_navalt_min_m) {
            tick.no_navigation = true;
        }
    }
    nav.takeoff_last_run_ms = in.now_ms;

    if (tick.no_navigation) {
        tick.ne_relax = true;
    } else {
        tick.input_vel_accel_ne = true;
        tick.nav_roll_cd = in.pos_control_roll_cd;
        tick.nav_pitch_cd = in.pos_control_pitch_cd;
        tick.assign_tilt_to_fwd_thr = true;
    }

    tick.run_xy_controller = true;
    tick.set_pilot_yaw_rate_time_constant = true;
    tick.input_euler_rate_yaw = true;
    tick.euler_rate_yaw_cds = get_pilot_input_yaw_rate_cds(in.pilot_yaw) + in.weathervane_yaw_rate_cds;

    if (in.mode_is_guided && in.guided_takeoff) {
        if (in.target.has_origin) {
            tick.input_pos_vel_accel_d = true;
            tick.pos_d_m = -static_cast<float>(kGuidedTakeoffAltMarginCm + in.target.next_wp.alt -
                                               in.target.origin.alt) *
                           0.01f;
            tick.vel_d_ms = 0.f;
        } else {
            tick.set_climb_rate = true;
            tick.climb_rate_ms = in.wp_nav_default_speed_up_ms;
        }
    } else {
        tick.set_climb_rate = true;
        tick.climb_rate_ms = in.wp_nav_default_speed_up_ms;
    }
    tick.run_z_controller = true;
    return tick;
}

[[nodiscard]] inline WaypointControllerTick waypoint_controller(TakeoffNavState& nav, PosControlState& pc,
                                                               const WaypointControllerInputs& in) {
    WaypointControllerTick tick{};
    tick.setup = setup_target_position(pc, in.target);
    if (!in.target.next_wp.same_loc_as(nav.last_auto_target) ||
        (in.now_ms - nav.last_loiter_ms) > kWaypointDestRefreshMs) {
        tick.set_wp_destination_ned = true;
        tick.dest_ned_n_m = pc.target_ned_n_m;
        tick.dest_ned_e_m = pc.target_ned_e_m;
        tick.dest_ned_d_m = pc.target_ned_d_m;
        nav.last_auto_target = in.target.next_wp;
    }
    nav.last_loiter_ms = in.now_ms;

    tick.update_wpnav = true;
    tick.nav_roll_cd = in.wp_nav_roll_cd;
    tick.nav_pitch_cd = in.wp_nav_pitch_cd;
    tick.assign_tilt_to_fwd_thr = true;
    if (in.vtol_roll_pitch_limited) {
        tick.ne_set_externally_limited = true;
    }
    tick.disable_yaw_rate_time_constant = true;
    tick.input_euler_angle_roll_pitch_yaw = true;
    tick.attitude_yaw_cd = in.wp_nav_yaw_cd;
    tick.set_climb_rate = true;
    tick.climb_rate_ms = in.assist_climb_rate_cms * 0.01f;
    tick.run_z_controller = true;
    return tick;
}

}  // namespace fwcpp::quadplane
