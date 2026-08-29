#pragma once

// QuadPlane::vtol_position_controller — Plane-4.7.0 ArduPlane/quadplane.cpp
// 2351-2922. Header-only ticks/effects (ADR-0012).
//
// No pos_control / nav_controller / GCS objects. INTERNAL_ERROR is
// tick.flow_of_control. log_QPOS / HAL_LOGGING skipped (OOS).
// setup_target_position, run_xy, assign_tilt, landing_descent_rate,
// hold_hover / hold_stabilize, run_z, should_relax, and
// poscontrol_init_approach_prep are called or flagged — not re-ported.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/quadplane/quadplane_auto_vtol_mission.hpp>
#include <fwcpp/quadplane/quadplane_land_detector.hpp>
#include <fwcpp/quadplane/quadplane_landing.hpp>
#include <fwcpp/quadplane/quadplane_mode_predicates.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/quadplane/quadplane_options.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_approach.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>

namespace fwcpp::quadplane {

inline constexpr float kVtolPosition2DistThresholdM = 10.0f;
inline constexpr float kVtolPosition2TargetSpeedMs = 3.0f;
inline constexpr float kVtolTransDecelDefaultMss = 2.0f;
inline constexpr float kVtolAirspeedMinDefault = 9.0f;
inline constexpr float kVtolThrottleCruiseDefault = 45.0f;
inline constexpr float kVtolThrottleMaxDefault = 100.0f;
inline constexpr float kVtolWpSpeedDefaultMs = 10.0f;
inline constexpr float kVtolSinkThrustLossMs = 0.2f;
inline constexpr std::uint32_t kVtolVelocityMatchFreshMs = 1000;
inline constexpr std::uint32_t kVtolMinAirbrakeMs = 1000;
inline constexpr std::uint32_t kVtolThrustLossMs = 5000;
inline constexpr std::uint32_t kVtolTiltZSuppressMs = 2000;
inline constexpr std::int32_t kVtolAttitudeErrorThresholdCd = 1000;

enum class VtolPosText : std::uint8_t {
    kNone = 0,
    kPosition1Nvtol,
    kPosition1,
    kAirbrake,
    kPosition1ThrustLoss,
    kPosition1LowSpeed,
    kOvershoot,
    kPosition2Started,
};

struct VtolPositionControllerInputs {
    std::uint32_t now_ms{0};
    bool armed_and_safety_off{false};
    InVtolModeInputs in_vtol{};

    float closing_vel_north_ms{0.f};
    float closing_vel_east_ms{0.f};
    float desired_closing_vel_north_ms{0.f};
    float desired_closing_vel_east_ms{0.f};
    float groundspeed_ms{0.f};
    float distance_m{0.f};
    bool have_airspeed{false};
    float aspeed_ms{0.f};
    float airspeed_min{kVtolAirspeedMinDefault};
    float assist_speed{0.f};
    float transition_decel_mss{kVtolTransDecelDefaultMss};

    bool tiltrotor_enabled{false};
    bool tilt_over_max_angle{false};
    float current_tilt{0.f};
    float fully_forward_tilt{1.f};
    bool tilt_angle_achieved{true};
    std::uint32_t last_pidz_active_ms{0};

    bool tailsitter_enabled{false};
    bool tailsitter_in_vtol_transition{false};
    bool transition_complete{false};

    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    float sink_rate{0.f};
    float throttle_scaled{0.f};
    float throttle_max{kVtolThrottleMaxDefault};
    float throttle_cruise{kVtolThrottleCruiseDefault};

    std::int32_t tecs_pitch_demand_cd{0};
    std::int32_t pitch_limit_min_deg{0};
    std::int32_t pitch_limit_max_deg{25};
    std::int32_t nav_roll_cd{0};
    std::int32_t nav_pitch_cd{0};
    std::int32_t ahrs_roll_cd{0};
    std::int32_t ahrs_pitch_cd{0};

    float wp_distance_north_m{0.f};
    float wp_distance_east_m{0.f};
    float wp_speed_ms{kVtolWpSpeedDefaultMs};
    float yaw_deg{0.f};
    float wind_north_ms{0.f};
    float wind_east_ms{0.f};
    float groundspeed_north_ms{0.f};
    float groundspeed_east_ms{0.f};
    bool vtol_roll_pitch_limited{false};
    float pos_control_roll_cd{0.f};
    float pos_control_pitch_cd{0.f};

    bool mode_guided{false};
    bool mode_qrtl{false};
    bool mode_auto{false};
    std::uint16_t nav_cmd_id{0};
    bool have_target_alt{true};
    std::int32_t target_alt_cm{0};
    float prev_to_next_wp_dist_m{0.f};
    float wp_proportion{0.f};
    bool have_prev_alt{false};
    std::int32_t prev_alt_cm{0};
    std::int32_t qrtl_alt_cm{0};
    float height_above_ground_m{0.f};
    float speed_up_ms{kDefaultSpeedUpMs};
    std::int32_t options{0};

    bool throttle_lower{false};
    float motors_throttle{0.f};
    float throttle_hover{1.f};
    std::uint32_t last_pos_ne_reset_ms{0};

    ShouldRelaxInputs relax{};
    LandingDescentRateInputs descent{};
    PoscontrolApproachInitInputs approach_init{};
    LandPositioningInputs land_positioning{};
    PosControlSetStateInputs set_state{};
};

struct VtolPositionControllerTick {
    bool ran{false};
    bool none_to_position1_failsafe{false};
    bool approach_nvtol_failsafe{false};
    bool flow_of_control{false};
    bool suppress_z_controller{false};
    VtolPosText send_text{VtolPosText::kNone};

    float landing_velocity_north_ms{0.f};
    float landing_velocity_east_ms{0.f};

    bool hold_stabilize{false};
    float hold_stabilize_throttle{0.f};
    bool hold_hover{false};
    float hold_hover_climb_cms{0.f};

    bool fw_nav_update_waypoint{false};
    bool tecs_throttle{false};
    bool tecs_pitch{false};
    bool calc_nav_roll{false};
    std::int32_t nav_pitch_cd{0};

    float stop_distance_m{0.f};
    float aspeed_ms{0.f};
    float aspeed_threshold_ms{0.f};
    float closing_speed_ms{0.f};
    float desired_closing_speed_ms{0.f};

    bool set_last_fw_pitch{false};
    bool poscontrol_init_approach{false};
    PoscontrolApproachInitResult approach{};

    bool setup_target_position{false};
    bool tailsitter_transition_break{false};

    float approach_speed_ms{0.f};
    float approach_accel_mss{0.f};
    bool input_vel_accel_NE{false};
    bool NE_stop_pos_stabilisation{false};
    bool set_accel_desired_NE{false};
    bool run_xy{false};
    float run_xy_accel_mss{0.f};
    bool assign_tilt{false};
    bool NE_set_externally_limited{false};
    bool disable_yaw_rate_time_constant{false};
    bool setup_rp_fw_angle_gains{false};
    bool input_euler_yaw{false};
    bool input_euler_rate_yaw{false};
    float target_yaw_deg{0.f};

    bool input_pos_vel_accel_NE{false};
    bool update_land_positioning{false};
    bool set_pilot_yaw_rate_time_constant{false};

    bool should_relax{false};
    bool NE_relax{false};

    bool D_relax{false};
    bool input_pos_vel_accel_D{false};
    float target_d_m{0.f};
    bool set_climb_rate{false};
    float climb_rate_ms{0.f};
    bool landing_descent_rate{false};
    float descent_rate_ms{0.f};
    bool D_set_pos_target_from_climb_rate{false};
    bool set_touchdown_expected{false};
    bool run_z_controller{false};

    float vel_forward_integrator{0.f};
    std::uint32_t vel_forward_last_ms{0};
};

[[nodiscard]] inline float stopping_distance_m(float gs_sq, float transition_decel_mss) {
    return gs_sq / (2.0f * transition_decel_mss);
}

[[nodiscard]] inline float vtol_accel_needed(float stop_distance, float ground_speed_squared) {
    return ground_speed_squared / (2.0f * std::max(1.0f, stop_distance));
}

[[nodiscard]] inline float vtol_scaled_wp_speed(float wp_speed_ms, float yaw_deg,
                                                float target_bearing_deg) {
    const float yaw_difference = std::fabs(fwcpp::math::wrap_180(yaw_deg - target_bearing_deg));
    if (yaw_difference > 20.0f) {
        const float speed_reduction =
            fwcpp::math::linear_interpolate(1.0f, 3.0f, yaw_difference, 20.0f, 160.0f);
        return wp_speed_ms / speed_reduction;
    }
    return wp_speed_ms;
}

[[nodiscard]] inline bool vtol_loiter_auto_cmd(bool mode_auto, std::uint16_t nav_cmd_id) {
    if (!mode_auto) {
        return false;
    }
    switch (nav_cmd_id) {
        case kMavCmdNavLoiterUnlim:
        case kMavCmdNavLoiterTime:
        case kMavCmdNavLoiterTurns:
        case kMavCmdNavLoiterToAlt:
            return true;
        default:
            return false;
    }
}

inline PosControlSetStateInputs vtol_pos_set_in(const VtolPositionControllerInputs& in) {
    PosControlSetStateInputs set_in = in.set_state;
    set_in.now_ms = in.now_ms;
    set_in.groundspeed_ms = in.groundspeed_ms;
    if (in.last_pos_ne_reset_ms != 0) {
        set_in.last_pos_reset_ms = in.last_pos_ne_reset_ms;
    }
    return set_in;
}

inline void vtol_pos_approach_airbrake(PosControlState& pc, PosControlLandStub& land,
                                       PosControlSetStateSink& sink,
                                       VtolPositionControllerTick& tick,
                                       const VtolPositionControllerInputs& in,
                                       const PosControlSetStateInputs& set_in) {
    const fwcpp::math::Vector2f closing_vel{in.closing_vel_north_ms, in.closing_vel_east_ms};
    const fwcpp::math::Vector2f desired_closing_vel{in.desired_closing_vel_north_ms,
                                                    in.desired_closing_vel_east_ms};
    tick.closing_speed_ms = closing_vel.length();
    tick.desired_closing_speed_ms = desired_closing_vel.length();
    tick.aspeed_ms = in.have_airspeed ? in.aspeed_ms : in.groundspeed_ms;

    if (in.tiltrotor_enabled && pc.state == PositionControlState::kAirbrake) {
        if ((in.now_ms - in.last_pidz_active_ms > kVtolTiltZSuppressMs && in.tilt_over_max_angle) ||
            in.current_tilt >= in.fully_forward_tilt) {
            tick.suppress_z_controller = true;
            tick.hold_stabilize = true;
            tick.hold_stabilize_throttle = 0.01f;
        }
    }

    tick.aspeed_threshold_ms = std::max(in.airspeed_min - 2.0f, in.assist_speed);

    tick.fw_nav_update_waypoint = true;
    tick.tecs_throttle = true;
    tick.tecs_pitch = true;
    tick.nav_pitch_cd = fwcpp::math::constrain_value(in.tecs_pitch_demand_cd,
                                                     in.pitch_limit_min_deg * 100,
                                                     in.pitch_limit_max_deg * 100);
    if (pc.state == PositionControlState::kAirbrake) {
        tick.nav_pitch_cd = std::max(tick.nav_pitch_cd, 0);
    }
    tick.calc_nav_roll = true;

    const float gs_sq = in.groundspeed_ms * in.groundspeed_ms;
    tick.stop_distance_m = stopping_distance_m(gs_sq, in.transition_decel_mss) +
                           2.0f * tick.closing_speed_ms;

    if (!tick.suppress_z_controller && pc.state == PositionControlState::kAirbrake) {
        tick.hold_hover = true;
        tick.hold_hover_climb_cms = 0.f;
        tick.suppress_z_controller = true;
    }

    if (pc.state == PositionControlState::kApproach && in.distance_m < tick.stop_distance_m) {
        if (in.tailsitter_enabled || in.desired_spool == DesiredSpoolState::kThrottleUnlimited) {
            tick.send_text = VtolPosText::kPosition1;
            poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
            tick.set_last_fw_pitch = true;
        } else {
            tick.send_text = VtolPosText::kAirbrake;
            poscontrol_apply_set_state(pc, PositionControlState::kAirbrake, set_in, sink, land);
        }
    }

    if (pc.state == PositionControlState::kAirbrake &&
        poscontrol_time_since_state_start_ms(pc, in.now_ms) > kVtolMinAirbrakeMs &&
        (tick.aspeed_ms < tick.aspeed_threshold_ms ||
         std::fabs(fwcpp::math::degrees(closing_vel.angle(desired_closing_vel))) > 60.0f ||
         tick.closing_speed_ms >
             std::max(tick.desired_closing_speed_ms * 1.2f, tick.desired_closing_speed_ms + 2.0f) ||
         tick.closing_speed_ms < tick.desired_closing_speed_ms * 0.5f ||
         std::abs(in.ahrs_roll_cd - in.nav_roll_cd) > kVtolAttitudeErrorThresholdCd ||
         std::abs(in.ahrs_pitch_cd - tick.nav_pitch_cd) > kVtolAttitudeErrorThresholdCd)) {
        tick.send_text = VtolPosText::kPosition1;
        poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
        tick.set_last_fw_pitch = true;
        tick.vel_forward_integrator = in.throttle_scaled;
        tick.vel_forward_integrator = fwcpp::math::linear_interpolate(
            0.f, tick.vel_forward_integrator, tick.closing_speed_ms,
            1.2f * tick.desired_closing_speed_ms, 0.5f * tick.desired_closing_speed_ms);
        tick.vel_forward_integrator = fwcpp::math::constrain_value(
            tick.vel_forward_integrator, 0.f, in.throttle_cruise * 0.5f);
        tick.vel_forward_last_ms = in.now_ms;
    }

    if (!in.tiltrotor_enabled && !in.tailsitter_enabled) {
        const bool throttle_saturated = in.throttle_scaled >= in.throttle_max;
        if (throttle_saturated && in.desired_spool < DesiredSpoolState::kThrottleUnlimited &&
            in.sink_rate > kVtolSinkThrustLossMs &&
            tick.aspeed_ms < tick.aspeed_threshold_ms + 4.0f) {
            if (pc.thrust_loss_start_ms == 0) {
                pc.thrust_loss_start_ms = in.now_ms;
            }
            if (in.now_ms - pc.thrust_loss_start_ms > kVtolThrustLossMs) {
                tick.send_text = VtolPosText::kPosition1ThrustLoss;
                poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
                tick.set_last_fw_pitch = true;
            }
        } else {
            pc.thrust_loss_start_ms = 0;
        }

        if (pc.state == PositionControlState::kApproach &&
            tick.aspeed_ms < tick.aspeed_threshold_ms &&
            in.desired_spool < DesiredSpoolState::kThrottleUnlimited) {
            tick.send_text = VtolPosText::kPosition1LowSpeed;
            poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
            tick.set_last_fw_pitch = true;
        }
    }

    if (pc.state == PositionControlState::kApproach) {
        PoscontrolApproachInitInputs ap = in.approach_init;
        ap.set_state = set_in;
        tick.approach = poscontrol_init_approach_prep(pc, land, ap);
        tick.poscontrol_init_approach = true;
    }
}

inline void vtol_pos_position1(PosControlState& pc, PosControlLandStub& land,
                               PosControlSetStateSink& sink, VtolPositionControllerTick& tick,
                               const VtolPositionControllerInputs& in,
                               const PosControlSetStateInputs& set_in,
                               float land_vel_n, float land_vel_e) {
    tick.setup_target_position = true;
    if (in.tailsitter_enabled && in.tailsitter_in_vtol_transition) {
        tick.tailsitter_transition_break = true;
        return;
    }

    const fwcpp::math::Vector2f wp_distance_ne{in.wp_distance_north_m, in.wp_distance_east_m};
    const float wp_distance_m = wp_distance_ne.length();
    const fwcpp::math::Vector2f rel_gs{in.closing_vel_north_ms, in.closing_vel_east_ms};
    const float rel_groundspeed_sq = rel_gs.length_squared();
    float closing_groundspeed_ms = 0.f;
    if (wp_distance_m > 0.1f) {
        closing_groundspeed_ms = rel_gs * wp_distance_ne.normalized();
    }

    const float stopping_speed_ms = fwcpp::math::safe_sqrt(
        std::max(0.f, wp_distance_m - kVtolPosition2DistThresholdM) * 2.0f *
            in.transition_decel_mss +
        kVtolPosition2TargetSpeedMs * kVtolPosition2TargetSpeedMs);

    float approach_speed_ms = stopping_speed_ms;
    const float wp_speed_ms = std::max(1.0f, in.wp_speed_ms);
    const float scaled_wp_speed_ms =
        vtol_scaled_wp_speed(wp_speed_ms, in.yaw_deg, fwcpp::math::degrees(wp_distance_ne.angle()));

    approach_speed_ms =
        std::min(std::max(pc.pos1_speed_limit_ms, 2.0f * wp_speed_ms), approach_speed_ms);

    if (pc.reached_wp_speed || rel_groundspeed_sq < wp_speed_ms * wp_speed_ms ||
        wp_speed_ms > 1.35f * scaled_wp_speed_ms) {
        approach_speed_ms = std::min(approach_speed_ms, scaled_wp_speed_ms);
        pc.reached_wp_speed = true;
    }

    tick.fw_nav_update_waypoint = true;

    fwcpp::math::Vector2f target_speed_ne;
    fwcpp::math::Vector2f target_accel_ne;
    bool have_target_yaw = false;
    float target_yaw_deg = 0.f;
    const float approach_accel_mss =
        std::min(vtol_accel_needed(wp_distance_m, closing_groundspeed_ms * closing_groundspeed_ms),
                 in.transition_decel_mss * 2.0f);
    if (wp_distance_m > 0.1f) {
        const fwcpp::math::Vector2f diff_wp_norm = wp_distance_ne.normalized();
        target_speed_ne = diff_wp_norm * approach_speed_ms;
        target_accel_ne = diff_wp_norm * (-approach_accel_mss);
        target_yaw_deg = fwcpp::math::degrees(diff_wp_norm.angle());
        const float yaw_err_deg = fwcpp::math::wrap_180(target_yaw_deg - in.yaw_deg);
        const bool overshoot = (closing_groundspeed_ms < 0.f || std::fabs(yaw_err_deg) > 60.0f);
        if (overshoot && !pc.overshoot) {
            tick.send_text = VtolPosText::kOvershoot;
            pc.overshoot = true;
            tick.set_accel_desired_NE = true;
        }
        if (pc.overshoot) {
            target_accel_ne.zero();
            const float rel_gs_sq_stop = stopping_distance_m(rel_groundspeed_sq, in.transition_decel_mss);
            approach_speed_ms = fwcpp::math::linear_interpolate(
                kVtolPosition2TargetSpeedMs, wp_speed_ms, wp_distance_m,
                kVtolPosition2DistThresholdM * 1.5f,
                2.0f * kVtolPosition2DistThresholdM + rel_gs_sq_stop);
            target_speed_ne = diff_wp_norm * approach_speed_ms;

            const fwcpp::math::Vector2f airspeed_ne{in.groundspeed_north_ms - in.wind_north_ms,
                                                    in.groundspeed_east_ms - in.wind_east_ms};
            if (airspeed_ne.length_squared() < 1.0f) {
                have_target_yaw = false;
            } else {
                have_target_yaw = true;
                target_yaw_deg = fwcpp::math::degrees(airspeed_ne.angle());
            }
        }
    }

    const float target_speed_ms = target_speed_ne.length();
    target_speed_ne.x += land_vel_n;
    target_speed_ne.y += land_vel_e;
    tick.approach_speed_ms = approach_speed_ms;
    tick.approach_accel_mss = approach_accel_mss;
    tick.target_yaw_deg = target_yaw_deg;

    if (!pc.reached_wp_speed && rel_groundspeed_sq < target_speed_ms * target_speed_ms &&
        rel_groundspeed_sq > (2.0f * wp_speed_ms) * (2.0f * wp_speed_ms) &&
        in.nav_pitch_cd < 0) {
        pc.pos1_speed_limit_ms = std::sqrt(rel_groundspeed_sq);
    }

    tick.input_vel_accel_NE = true;
    tick.NE_stop_pos_stabilisation = true;
    tick.run_xy = true;
    tick.run_xy_accel_mss = std::max(approach_accel_mss, in.transition_decel_mss) * 1.5f;
    if (!pc.done_accel_init) {
        pc.done_accel_init = true;
        tick.set_accel_desired_NE = true;
    }
    tick.assign_tilt = true;
    if (in.vtol_roll_pitch_limited) {
        tick.NE_set_externally_limited = true;
    }
    tick.disable_yaw_rate_time_constant = true;
    tick.setup_rp_fw_angle_gains = true;
    if (have_target_yaw) {
        tick.input_euler_yaw = true;
    } else {
        tick.input_euler_rate_yaw = true;
    }

    if (in.distance_m < kVtolPosition2DistThresholdM && in.tilt_angle_achieved &&
        std::fabs(rel_groundspeed_sq) <
            (3.0f * kVtolPosition2TargetSpeedMs) * (3.0f * kVtolPosition2TargetSpeedMs)) {
        poscontrol_apply_set_state(pc, PositionControlState::kPosition2, set_in, sink, land);
        pc.pilot_correction_done = false;
        tick.send_text = VtolPosText::kPosition2Started;
    }
}

inline void vtol_pos_position2_land(PosControlState& pc, VtolPositionControllerTick& tick,
                                    const VtolPositionControllerInputs& in, float land_vel_n,
                                    float land_vel_e) {
    tick.setup_target_position = true;
    tick.input_pos_vel_accel_NE = true;
    (void)land_vel_n;
    (void)land_vel_e;
    tick.fw_nav_update_waypoint = true;
    LandPositioningInputs pos_in = in.land_positioning;
    pos_in.options = in.options;
    update_land_positioning(pc, pos_in);
    tick.update_land_positioning = true;
    tick.run_xy = true;
    tick.run_xy_accel_mss = in.transition_decel_mss * 1.5f;
    tick.assign_tilt = true;
    if (in.vtol_roll_pitch_limited) {
        tick.NE_set_externally_limited = true;
    }
    tick.set_pilot_yaw_rate_time_constant = true;
    tick.input_euler_rate_yaw = true;
}

inline void vtol_pos_land_final(PosControlState& pc, PosControlLandStub& land,
                                VtolPositionControllerTick& tick,
                                const VtolPositionControllerInputs& in) {
    tick.setup_target_position = true;
    LandPositioningInputs pos_in = in.land_positioning;
    pos_in.options = in.options;
    update_land_positioning(pc, pos_in);
    tick.update_land_positioning = true;

    ShouldRelaxInputs relax = in.relax;
    relax.now_ms = in.now_ms;
    tick.should_relax = should_relax(land, relax);
    if (tick.should_relax) {
        tick.NE_relax = true;
    } else if (in.throttle_lower || in.motors_throttle < 0.5f * in.throttle_hover ||
               in.last_pos_ne_reset_ms != pc.last_pos_reset_ms) {
        tick.input_vel_accel_NE = true;
    } else {
        tick.input_pos_vel_accel_NE = true;
    }

    tick.run_xy = true;
    tick.assign_tilt = true;
    tick.set_pilot_yaw_rate_time_constant = true;
    tick.input_euler_rate_yaw = true;
}

inline void vtol_pos_height_position2(PosControlState& pc, VtolPositionControllerTick& tick,
                                      const VtolPositionControllerInputs& in) {
    if (in.mode_guided || vtol_loiter_auto_cmd(in.mode_auto, in.nav_cmd_id)) {
        if (!in.have_target_alt) {
            return;
        }
        std::int32_t target_altitude_cm = in.target_alt_cm;
        if (pc.slow_descent && in.prev_to_next_wp_dist_m > 50.0f && in.have_prev_alt) {
            target_altitude_cm = static_cast<std::int32_t>(fwcpp::math::linear_interpolate(
                static_cast<float>(in.prev_alt_cm), static_cast<float>(target_altitude_cm),
                in.wp_proportion, 0.f, 1.f));
        }
        tick.input_pos_vel_accel_D = true;
        tick.target_d_m = -static_cast<float>(target_altitude_cm) * 0.01f;
    } else if (in.mode_qrtl) {
        tick.input_pos_vel_accel_D = true;
        tick.target_d_m = -static_cast<float>(in.qrtl_alt_cm) * 0.01f;
    } else {
        tick.set_climb_rate = true;
        tick.climb_rate_ms = 0.f;
    }
}

inline VtolPositionControllerTick run_vtol_position_controller(PosControlState& pc,
                                                               PosControlLandStub& land,
                                                               PosControlSetStateSink& sink,
                                                               const VtolPositionControllerInputs& in) {
    VtolPositionControllerTick tick{};
    if (!in.in_vtol.available) {
        return tick;
    }
    tick.ran = true;
    if (in.armed_and_safety_off) {
        pc.last_run_ms = in.now_ms;
    }

    tick.suppress_z_controller = false;
    if (in.now_ms - pc.last_velocity_match_ms < kVtolVelocityMatchFreshMs) {
        tick.landing_velocity_north_ms = pc.velocity_match_north_ms;
        tick.landing_velocity_east_ms = pc.velocity_match_east_ms;
    }

    const PosControlSetStateInputs set_in = vtol_pos_set_in(in);
    const PositionControlState start_state = pc.state;

    switch (start_state) {
        case PositionControlState::kNone:
            poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
            tick.none_to_position1_failsafe = true;
            tick.flow_of_control = true;
            break;

        case PositionControlState::kApproach:
            if (compute_in_vtol_mode(in.in_vtol)) {
                tick.send_text = VtolPosText::kPosition1Nvtol;
                poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
                tick.approach_nvtol_failsafe = true;
                tick.flow_of_control = true;
            }
            // Upstream FALLTHROUGH: APPROACH failsafe still runs the AIRBRAKE body.
            vtol_pos_approach_airbrake(pc, land, sink, tick, in, set_in);
            break;

        case PositionControlState::kAirbrake:
            vtol_pos_approach_airbrake(pc, land, sink, tick, in, set_in);
            break;

        case PositionControlState::kPosition1:
            vtol_pos_position1(pc, land, sink, tick, in, set_in, tick.landing_velocity_north_ms,
                               tick.landing_velocity_east_ms);
            break;

        case PositionControlState::kPosition2:
        case PositionControlState::kLandAbort:
        case PositionControlState::kLandDescend:
            vtol_pos_position2_land(pc, tick, in, tick.landing_velocity_north_ms,
                                    tick.landing_velocity_east_ms);
            break;

        case PositionControlState::kLandFinal:
            vtol_pos_land_final(pc, land, tick, in);
            break;

        case PositionControlState::kLandComplete:
            break;
    }

    switch (pc.state) {
        case PositionControlState::kNone:
            poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
            tick.none_to_position1_failsafe = true;
            tick.flow_of_control = true;
            break;

        case PositionControlState::kApproach:
        case PositionControlState::kAirbrake:
            if (in.transition_complete) {
                tick.D_relax = true;
            }
            break;

        case PositionControlState::kPosition1:
            if (in.tailsitter_enabled && in.tailsitter_in_vtol_transition) {
                tick.D_relax = true;
                break;
            }
            vtol_pos_height_position2(pc, tick, in);
            break;

        case PositionControlState::kPosition2:
            vtol_pos_height_position2(pc, tick, in);
            break;

        case PositionControlState::kLandDescend:
        case PositionControlState::kLandAbort:
        case PositionControlState::kLandFinal: {
            if (pc.state == PositionControlState::kLandFinal) {
                if (!option_is_set(in.options, QOption::kDisableGroundEffectComp)) {
                    tick.set_touchdown_expected = true;
                }
            }
            if (pc.state == PositionControlState::kLandAbort) {
                tick.set_climb_rate = true;
                tick.climb_rate_ms = in.speed_up_ms;
                break;
            }
            LandingDescentRateInputs descent = in.descent;
            descent.now_ms = in.now_ms;
            descent.options = in.options;
            const auto descent_tick =
                landing_descent_rate_ms(pc, land, in.height_above_ground_m, descent);
            tick.landing_descent_rate = true;
            tick.descent_rate_ms = descent_tick.rate_ms;
            tick.D_set_pos_target_from_climb_rate = true;
            break;
        }

        case PositionControlState::kLandComplete:
            break;
    }

    if (!tick.suppress_z_controller) {
        tick.run_z_controller = true;
    }
    return tick;
}

}  // namespace fwcpp::quadplane
