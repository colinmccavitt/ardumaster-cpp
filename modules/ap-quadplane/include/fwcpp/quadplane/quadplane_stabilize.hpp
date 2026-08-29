#pragma once

// QuadPlane stabilize / z / attitude-rate — Plane-4.7.0 ArduPlane/quadplane.cpp:
// multicopter_attitude_rate_update (915-1013), hold_stabilize (1016-1037),
// run_z_controller (1040-1066), relax_attitude_control (1068-1073).
//
// ADR-0012: header-only ticks/effects. No pos_control / attitude_control /
// motors objects (D_* and spool are flags). AP_PLANE_SYSTEMID offsets stay 0.
// ahrs_view->rotate is a flag plus Vector3f; PITCH_90 is applied here when
// the caller did not inject an already-rotated vector.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/quadplane/quadplane_pilot_input.hpp>
#include <fwcpp/quadplane/quadplane_setup_channels.hpp>
#include <fwcpp/tailsitter/tailsitter_control.hpp>
#include <fwcpp/tailsitter/tailsitter_input_type.hpp>
#include <fwcpp/tailsitter/tailsitter_speed_scaling.hpp>
#include <fwcpp/tailsitter/tailsitter_transition.hpp>

namespace fwcpp::quadplane {

inline constexpr std::uint32_t kPidzReinitGapMs = 20;
inline constexpr float kSystemIdThrottleOffset = 0.0f;
inline constexpr float kSystemIdAttitudeOffsetCd = 0.0f;

struct ZCtrlState {
    std::uint32_t last_pidz_active_ms{0};
    std::uint32_t last_pidz_init_ms{0};
};

struct ZCtrlInputs {
    std::uint32_t now_ms{0};
    SpoolState spool{SpoolState::kShutDown};
    bool d_is_active{false};
    bool tailsitter_enabled{false};
    bool in_vtol_mode{false};
    fwcpp::tailsitter::TailsitterTransitionState transition_state{
        fwcpp::tailsitter::TailsitterTransitionState::kDone};
    std::uint32_t last_vtol_mode_ms{0};
    float pilot_speed_z_max_up_ms{kPilotSpeedZMaxUpMsDefault};
    float pilot_speed_z_max_dn_ms{kPilotSpeedZMaxDnMsDefault};
    float pilot_accel_z_mss{kPilotAccelZMssDefault};
};

struct ZCtrlTick {
    bool early_return{false};
    bool d_set_max{false};
    float d_max_speed_dn_m{0.0f};
    float d_max_speed_up_ms{0.0f};
    float d_max_accel_z_mss{0.0f};
    bool d_init{false};
    bool d_init_no_descent{false};
    bool d_update{false};
};

struct RelaxAttitudeInputs {
    bool tailsitter_enabled{false};
    bool tailsitter_is_vectored{false};
    std::uint32_t vtol_limit_start_ms{0};
};

struct RelaxAttitudeTick {
    bool relax_pitch_disabled{false};
};

struct AttitudeRateInputs {
    bool in_vtol_mode{false};
    bool tailsitter_enabled{false};
    fwcpp::tailsitter::TailsitterTransitionState transition_state{
        fwcpp::tailsitter::TailsitterTransitionState::kDone};
    std::uint32_t last_vtol_mode_ms{0};
    bool force_fw_control_recovery{false};
    bool transition_update_yaw_target{false};
    float yaw_target_cd{0.0f};
    float nav_roll_cd{0.0f};
    float nav_pitch_cd{0.0f};
    float yaw_rate_cds{0.0f};
    std::int8_t tailsitter_input_type{0};
    float lean_angle_max_cd{0.0f};
    float tailsitter_max_roll_angle{0.0f};
    float command_model_pilot_rate{kCommandModelPilotRateDefault};
    float fw_roll_pid_target{0.0f};
    float fw_pitch_pid_target{0.0f};
    AhrsViewRotation ahrs_view_rotation{AhrsViewRotation::kNone};
    bool ahrs_view_already_rotated{false};
    math::Vector3f ahrs_view_rotated_bf{};
};

struct AttitudeRateTick {
    bool use_multicopter_control{false};
    bool use_yaw_target{false};
    bool set_pilot_yaw_rate_time_constant{false};
    bool disable_yaw_rate_time_constant{false};
    bool input_euler_rate_yaw_euler_angle_pitch_bf_roll{false};
    bool bf_roll_plane{false};
    float bf_roll_cd{0.0f};
    float bf_pitch_cd{0.0f};
    float bf_yaw_rate_cds{0.0f};
    float y2r_scale{0.0f};
    bool input_euler_angle_roll_pitch_yaw{false};
    bool input_euler_angle_roll_pitch_euler_rate_yaw{false};
    float roll_cd{0.0f};
    float pitch_cd{0.0f};
    float yaw_cd_or_rate{0.0f};
    bool ahrs_view_rotate{false};
    math::Vector3f bf_input_cd{};
    bool input_rate_bf_roll_pitch_yaw_no_shaping{false};
};

struct HoldStabilizeInputs {
    float throttle_in{0.0f};
    bool air_mode_active{false};
    bool tailsitter_enabled{false};
    bool assisted_flight{false};
    DesiredYawRateInputs yaw{};
    AttitudeRateInputs attitude{};
    RelaxAttitudeInputs relax{};
};

struct HoldStabilizeTick {
    bool multicopter_attitude_rate_update{false};
    float desired_yaw_rate_cds{0.0f};
    AttitudeRateTick attitude{};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool set_throttle_out{false};
    float throttle_out{0.0f};
    bool should_boost{false};
    float throttle_filt_hz{0.0f};
    bool relax_attitude_control{false};
    RelaxAttitudeTick relax{};
};

// AP_AHRS_View::rotate for ROTATION_PITCH_90 (Vector3::rotate PITCH_90).
// NONE is identity. Does not pull an AHRS object.
inline void rotate_ahrs_view(math::Vector3f& vec, AhrsViewRotation rotation) {
    if (rotation == AhrsViewRotation::kPitch90) {
        const float tmp = vec.z;
        vec.z = -vec.x;
        vec.x = tmp;
    }
}

[[nodiscard]] inline RelaxAttitudeTick relax_attitude_control(const RelaxAttitudeInputs& in) {
    RelaxAttitudeTick tick{};
    tick.relax_pitch_disabled = !fwcpp::tailsitter::relax_pitch(
        in.tailsitter_enabled, in.tailsitter_is_vectored, in.vtol_limit_start_ms);
    return tick;
}

[[nodiscard]] inline AttitudeRateTick multicopter_attitude_rate_update(const AttitudeRateInputs& in) {
    AttitudeRateTick tick{};
    const bool in_trans = fwcpp::tailsitter::in_vtol_transition(
        in.tailsitter_enabled, in.in_vtol_mode, in.transition_state, 0, in.last_vtol_mode_ms);
    bool use_mc = in.in_vtol_mode && !in_trans && !in.force_fw_control_recovery;
    bool use_yaw_target = false;

    if (!use_mc && in.transition_update_yaw_target && !in.force_fw_control_recovery) {
        use_mc = true;
        use_yaw_target = true;
    }
    tick.use_multicopter_control = use_mc;
    tick.use_yaw_target = use_yaw_target;

    if (use_mc) {
        tick.set_pilot_yaw_rate_time_constant = true;
        if (in.tailsitter_enabled &&
            fwcpp::tailsitter::input_body_frame_roll(in.tailsitter_input_type)) {
            tick.input_euler_rate_yaw_euler_angle_pitch_bf_roll = true;
            tick.bf_pitch_cd = in.nav_pitch_cd;
            if (!fwcpp::tailsitter::input_plane_mode(in.tailsitter_input_type)) {
                tick.bf_roll_plane = false;
                tick.bf_roll_cd = in.nav_roll_cd;
                tick.bf_yaw_rate_cds = in.yaw_rate_cds;
                return tick;
            }
            tick.bf_roll_plane = true;
            float roll_limit = in.lean_angle_max_cd;
            if (in.tailsitter_max_roll_angle > 0.0f) {
                roll_limit = in.tailsitter_max_roll_angle * 100.0f;
            }
            const float yaw_rate_max = in.command_model_pilot_rate;
            const float yaw_rate_limit = ((yaw_rate_max < 1.0f) ? 1.0f : yaw_rate_max) * 100.0f;
            const float yaw2roll_scale = roll_limit / yaw_rate_limit;
            const float euler_pitch = fwcpp::math::radians(0.01f * in.nav_pitch_cd);
            const float spitch = std::fabs(std::sin(euler_pitch));
            tick.y2r_scale = fwcpp::math::linear_interpolate(1.0f, yaw2roll_scale, spitch, 0.0f, 1.0f);
            tick.bf_yaw_rate_cds = in.nav_roll_cd / tick.y2r_scale;
            tick.bf_roll_cd = -tick.y2r_scale * in.yaw_rate_cds;
            return tick;
        }

        const float off = kSystemIdAttitudeOffsetCd;
        tick.roll_cd = in.nav_roll_cd + off;
        tick.pitch_cd = in.nav_pitch_cd + off;
        if (use_yaw_target) {
            tick.input_euler_angle_roll_pitch_yaw = true;
            tick.yaw_cd_or_rate = in.yaw_target_cd + off;
        } else {
            tick.input_euler_angle_roll_pitch_euler_rate_yaw = true;
            tick.yaw_cd_or_rate = in.yaw_rate_cds + off;
        }
        return tick;
    }

    if (in.ahrs_view_already_rotated) {
        tick.bf_input_cd = in.ahrs_view_rotated_bf;
    } else {
        tick.bf_input_cd = math::Vector3f{in.fw_roll_pid_target * 100.0f,
                                          in.fw_pitch_pid_target * 100.0f, in.yaw_rate_cds};
        rotate_ahrs_view(tick.bf_input_cd, in.ahrs_view_rotation);
    }
    tick.ahrs_view_rotate = true;
    tick.disable_yaw_rate_time_constant = true;
    tick.input_rate_bf_roll_pitch_yaw_no_shaping = true;
    return tick;
}

[[nodiscard]] inline HoldStabilizeTick hold_stabilize(const HoldStabilizeInputs& in) {
    HoldStabilizeTick tick{};
    DesiredYawRateInputs yaw = in.yaw;
    yaw.should_weathervane = false;
    tick.desired_yaw_rate_cds = get_desired_yaw_rate_cds(yaw);

    AttitudeRateInputs att = in.attitude;
    att.yaw_rate_cds = tick.desired_yaw_rate_cds;
    att.tailsitter_enabled = in.tailsitter_enabled;
    tick.attitude = multicopter_attitude_rate_update(att);
    tick.multicopter_attitude_rate_update = true;

    if ((in.throttle_in <= 0.0f) && !in.air_mode_active) {
        tick.desired_spool = DesiredSpoolState::kGroundIdle;
        tick.set_throttle_out = true;
        tick.throttle_out = 0.0f;
        tick.should_boost = true;
        tick.throttle_filt_hz = 0.0f;
        tick.relax_attitude_control = true;
        RelaxAttitudeInputs relax = in.relax;
        relax.tailsitter_enabled = in.tailsitter_enabled;
        tick.relax = relax_attitude_control(relax);
    } else {
        tick.desired_spool = DesiredSpoolState::kThrottleUnlimited;
        tick.should_boost = !(in.tailsitter_enabled && in.assisted_flight);
        const float throttle_in = in.throttle_in + kSystemIdThrottleOffset;
        tick.set_throttle_out = true;
        tick.throttle_out = throttle_in;
        tick.throttle_filt_hz = 0.0f;
    }
    return tick;
}

inline ZCtrlTick run_z_controller(ZCtrlState& state, const ZCtrlInputs& in) {
    ZCtrlTick tick{};
    if (in.spool != SpoolState::kThrottleUnlimited) {
        tick.early_return = true;
        return tick;
    }
    if (fwcpp::tailsitter::in_vtol_transition(in.tailsitter_enabled, in.in_vtol_mode,
                                              in.transition_state, in.now_ms,
                                              in.last_vtol_mode_ms)) {
        tick.early_return = true;
        return tick;
    }
    if ((in.now_ms - state.last_pidz_active_ms) > kPidzReinitGapMs || !in.d_is_active) {
        tick.d_set_max = true;
        tick.d_max_speed_dn_m = static_cast<float>(
            get_pilot_velocity_z_max_dn_m(in.pilot_speed_z_max_dn_ms, in.pilot_speed_z_max_up_ms));
        tick.d_max_speed_up_ms = in.pilot_speed_z_max_up_ms;
        tick.d_max_accel_z_mss = in.pilot_accel_z_mss;
        if (!in.tailsitter_enabled) {
            tick.d_init = true;
        } else {
            tick.d_init_no_descent = true;
        }
        state.last_pidz_init_ms = in.now_ms;
    }
    state.last_pidz_active_ms = in.now_ms;
    tick.d_update = true;
    return tick;
}

}  // namespace fwcpp::quadplane
