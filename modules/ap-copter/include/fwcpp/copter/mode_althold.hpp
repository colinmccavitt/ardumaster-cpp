#pragma once

// ModeAltHold::run — ArduCopter/mode_althold.cpp ~26-104 and
// Mode::get_alt_hold_state_D_ms — mode.cpp ~1012-1061 (Plane-4.7.0).
// Free function althold_run; no AP:: / motors / pos_control objects
// (ADR-0012). Tests inject sticks, spool, land/takeoff flags, and
// kinematics state.
//
// This slice: call-site-complete Flying avoidance + surface_tracking
// leftovers (ADR-0012 flags). Flying records
// adjust_roll_pitch_avoidance (AP_AVOIDANCE_ALTHOLD) / avoidance
// climbrate / surface_tracking.update_surface_offset. Takeoff still
// records avoidance climbrate only (no adjust_roll_pitch). Prior
// slices: update_simple_mode + takeoff flags; real CCP-037 lean / yaw /
// climb_rate; constrain climb; get_alt_hold_state_D_ms; Flying
// D_set_pos_target_from_climb_rate. Remaining: D_update_controller
// (call-site flag; attitude already real). ALWAYS calls the real
// input_euler_angle_roll_pitch_euler_rate_yaw_rad (CCP-029).
//
// Reuses DesiredSpoolState / SpoolState from mode_stabilize.hpp.
// AltHoldModeState matches mode.h ~256-262.

#include <cstddef>
#include <cstdint>

#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/copter/pilot_input.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::copter {

enum class AltHoldModeState : std::uint8_t {
    MotorStopped = 0,
    Takeoff = 1,
    Landed_Ground_Idle = 2,
    Landed_Pre_Takeoff = 3,
    Flying = 4,
};

struct AltHoldStateInputs {
    bool armed{false};
    bool takeoff_running{false};
    bool takeoff_triggered{false};
    bool auto_armed{false};
    bool land_complete{true};
    bool using_interlock{false};
    SpoolState spool{SpoolState::SHUT_DOWN};
    float target_climb_rate_ms{0.0f};
};

struct AltHoldStateResult {
    AltHoldModeState state{AltHoldModeState::MotorStopped};
    DesiredSpoolState desired_spool{DesiredSpoolState::SHUT_DOWN};
    bool desired_spool_set{false};
};

// Upstream Mode::get_alt_hold_state_D_ms ~1012-1061. takeoff.triggered_ms
// body is leftover; the already-evaluated bool is injected.
[[nodiscard]] inline AltHoldStateResult get_alt_hold_state_D_ms(const AltHoldStateInputs& in) {
    AltHoldStateResult out{};

    if (!in.armed) {
        out.desired_spool = DesiredSpoolState::SHUT_DOWN;
        out.desired_spool_set = true;
        switch (in.spool) {
            case SpoolState::SHUT_DOWN:
                out.state = AltHoldModeState::MotorStopped;
                break;
            case SpoolState::GROUND_IDLE:
                out.state = AltHoldModeState::Landed_Ground_Idle;
                break;
            default:
                out.state = AltHoldModeState::Landed_Pre_Takeoff;
                break;
        }
        return out;
    }

    if (in.takeoff_running || in.takeoff_triggered) {
        out.state = AltHoldModeState::Takeoff;
        return out;
    }

    if (!in.auto_armed || in.land_complete) {
        if (in.target_climb_rate_ms < 0.0f && !in.using_interlock) {
            out.desired_spool = DesiredSpoolState::GROUND_IDLE;
        } else {
            out.desired_spool = DesiredSpoolState::THROTTLE_UNLIMITED;
        }
        out.desired_spool_set = true;
        if (in.spool == SpoolState::GROUND_IDLE) {
            out.state = AltHoldModeState::Landed_Ground_Idle;
        } else {
            out.state = AltHoldModeState::Landed_Pre_Takeoff;
        }
        return out;
    }

    out.desired_spool = DesiredSpoolState::THROTTLE_UNLIMITED;
    out.desired_spool_set = true;
    out.state = AltHoldModeState::Flying;
    return out;
}

struct AltHoldRunInputs {
    bool has_valid_input{false};
    float roll_norm_dz{0.0f};
    float pitch_norm_dz{0.0f};
    float yaw_norm_dz{0.0f};
    float lean_angle_max_rad{0.0f};
    float althold_lean_angle_max_rad{0.0f};
    float command_model_pilot_y_rate{0.0f};
    float yaw_expo{0.0f};
    float mid_stick{500.0f};
    float throttle_control{0.0f};
    std::int16_t throttle_deadzone{0};
    // Raw PILOT_SPD_DN / PILOT_SPD_UP. Down limit resolved via
    // get_pilot_speed_dn_ms. get_pilot_accel_D_mss is leftover.
    float speed_dn_ms{0.0f};
    float speed_up_ms{0.0f};
    bool accel_leftover{true};

    bool armed{false};
    bool takeoff_running{false};
    bool takeoff_triggered{false};
    bool auto_armed{false};
    bool land_complete{true};
    bool using_interlock{false};
    SpoolState spool_state{SpoolState::SHUT_DOWN};

    math::Quaternion attitude_body{};
    math::Vector3f gyro_body_rads{};
    control::EulerAngleRateShapingGains gains{};
    float dt{0.0f};
};

struct AltHoldRunResult {
    LeanAnglesRad lean{};
    float target_yaw_rate_rads{0.0f};
    float target_climb_rate_ms{0.0f};
    AltHoldModeState state{AltHoldModeState::MotorStopped};
    DesiredSpoolState desired_spool{DesiredSpoolState::SHUT_DOWN};
    bool desired_spool_set{false};
    bool update_simple_mode{false};
    bool reset_yaw_target_and_rate{false};
    bool reset_I{false};
    bool reset_I_smoothly{false};
    bool D_set_max_speed_accel{false};
    float speed_dn_ms{0.0f};
    float speed_up_ms{0.0f};
    bool accel_leftover{true};
    bool D_relax_controller{false};
    float D_relax_throttle{0.0f};
    bool takeoff_start{false};
    bool do_pilot_takeoff{false};
    bool avoidance{false};
    // Flying AP_AVOIDANCE_ALTHOLD avoid.adjust_roll_pitch_rad call-site.
    bool adjust_roll_pitch_avoidance{false};
    bool surface_tracking{false};
    bool D_set_pos_target_from_climb_rate{false};
    float pos_target_climb_rate_ms{0.0f};
    bool D_update_controller{false};
    bool input_euler_angle_invoked{false};
    float thrust_angle_rad{0.0f};
    float thrust_error_angle_rad{0.0f};
    float feedforward_scalar{0.0f};
    math::Quaternion attitude_ang_error{};
    math::Vector3f ang_vel_body_rads{};
};

// Upstream ModeAltHold::run ~26-104. state is mutated in place by the
// real CCP-029 entry point (same as stabilize_run).
[[nodiscard]] inline AltHoldRunResult althold_run(const AltHoldRunInputs& in,
                                                  control::AttitudeTargetState& state) {
    AltHoldRunResult out{};

    const float speed_dn = get_pilot_speed_dn_ms(in.speed_dn_ms, in.speed_up_ms);
    out.D_set_max_speed_accel = true;
    out.speed_dn_ms = speed_dn;
    out.speed_up_ms = in.speed_up_ms;
    out.accel_leftover = in.accel_leftover;

    // Upstream ModeAltHold::run ~31-32: always call update_simple_mode().
    // Sticks are injected already-transformed; call-site flag only.
    out.update_simple_mode = true;

    out.lean = get_pilot_desired_lean_angles_rad(in.has_valid_input, in.roll_norm_dz, in.pitch_norm_dz,
                                                 in.lean_angle_max_rad, in.althold_lean_angle_max_rad);

    out.target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads(
        in.has_valid_input, in.yaw_norm_dz, in.command_model_pilot_y_rate, in.yaw_expo);

    float target_climb_rate_ms =
        get_pilot_desired_climb_rate_ms(in.has_valid_input, in.throttle_control, in.mid_stick,
                                        in.throttle_deadzone, in.speed_dn_ms, in.speed_up_ms);
    target_climb_rate_ms = math::constrain_value(target_climb_rate_ms, -speed_dn, in.speed_up_ms);
    out.target_climb_rate_ms = target_climb_rate_ms;

    AltHoldStateInputs st;
    st.armed = in.armed;
    st.takeoff_running = in.takeoff_running;
    st.takeoff_triggered = in.takeoff_triggered;
    st.auto_armed = in.auto_armed;
    st.land_complete = in.land_complete;
    st.using_interlock = in.using_interlock;
    st.spool = in.spool_state;
    st.target_climb_rate_ms = target_climb_rate_ms;
    const auto hold = get_alt_hold_state_D_ms(st);
    out.state = hold.state;
    out.desired_spool = hold.desired_spool;
    out.desired_spool_set = hold.desired_spool_set;

    switch (out.state) {
        case AltHoldModeState::MotorStopped:
            out.reset_I = true;
            out.reset_yaw_target_and_rate = true;
            out.D_relax_controller = true;
            out.D_relax_throttle = 0.0f;
            break;

        case AltHoldModeState::Landed_Ground_Idle:
            out.reset_yaw_target_and_rate = true;
            [[fallthrough]];

        case AltHoldModeState::Landed_Pre_Takeoff:
            out.reset_I_smoothly = true;
            out.D_relax_controller = true;
            out.D_relax_throttle = 0.0f;
            break;

        case AltHoldModeState::Takeoff:
            // Upstream ~66-77: takeoff.start_m if !running; avoidance
            // climbrate; do_pilot_takeoff_ms. Bodies remaining; flags only.
            if (!in.takeoff_running) {
                out.takeoff_start = true;
            }
            out.avoidance = true;
            out.do_pilot_takeoff = true;
            break;

        case AltHoldModeState::Flying:
            // Upstream ~79-95: AP_AVOIDANCE_ALTHOLD adjust_roll_pitch;
            // get_avoidance_adjusted_climbrate_ms; surface_tracking
            // update_surface_offset; D_set_pos_target_from_climb_rate.
            // Bodies remaining (no AC_Avoid / surface_tracking objects);
            // call-site flags only.
            out.adjust_roll_pitch_avoidance = true;
            out.avoidance = true;
            out.surface_tracking = true;
            out.D_set_pos_target_from_climb_rate = true;
            out.pos_target_climb_rate_ms = target_climb_rate_ms;
            break;
    }

    control::input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        out.lean.roll_rad, out.lean.pitch_rad, out.target_yaw_rate_rads, state, in.attitude_body,
        in.gyro_body_rads, in.gains, in.dt, out.thrust_angle_rad, out.thrust_error_angle_rad,
        out.feedforward_scalar, out.attitude_ang_error, out.ang_vel_body_rads);
    out.input_euler_angle_invoked = true;

    // pos_control->D_update_controller() call site; body remaining.
    out.D_update_controller = true;
    return out;
}

// Nested so leftover remaining_count() does not collide with
// copter_leftover.hpp / mode_leftover.hpp / stabilize / acro leftovers.
namespace althold {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct PortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr PortItem kCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"althold_run", PortStatus::kThisSlice,
     "mode_althold.cpp ~26-104; climb_rate + get_alt_hold_state + real "
     "input_euler_angle_roll_pitch_euler_rate_yaw_rad"},
    {"get_alt_hold_state_D_ms", PortStatus::kThisSlice,
     "mode.cpp ~1012-1061; spool side effects; injected takeoff/armed/land"},
    {"D_set_max_speed_accel record", PortStatus::kThisSlice,
     "records dn/up; accel leftover flag; no PosControl object"},
    {"climb_rate constrain", PortStatus::kThisSlice,
     "constrain to [-get_pilot_speed_dn_ms(raw PILOT_SPD_DN), +speed_up]"},
    {"stabilize_run", PortStatus::kOnMain, "mode_stabilize.hpp; CCP-039 slice 1; notes only"},
    {"acro_run", PortStatus::kOnMain, "mode_acro.hpp; CCP-039 slice 2; notes only"},
    {"update_simple_mode", PortStatus::kThisSlice,
     "mode_althold.cpp ~31-32; call-site flag; sticks injected already-transformed"},
    {"takeoff", PortStatus::kThisSlice,
     "mode_althold.cpp ~66-77; takeoff_start/do_pilot_takeoff flags-complete ADR-0012; no start_m body"},
    {"avoidance", PortStatus::kThisSlice,
     "Flying adjust_roll_pitch_avoidance + climbrate flags; Takeoff climbrate; no AC_Avoid body"},
    {"surface_tracking", PortStatus::kThisSlice,
     "mode_althold.cpp ~89-91; Flying surface_tracking flag; no update_surface_offset body"},
    {"D_update_controller", PortStatus::kRemaining,
     "pos_control D_update_controller; call-site flag only, no object"},
};

[[nodiscard]] inline constexpr std::size_t completeness_size() {
    return sizeof(kCompleteness) / sizeof(kCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

}  // namespace althold

}  // namespace fwcpp::copter
