#pragma once

// ModeStabilize::run — ArduCopter/mode_stabilize.cpp ~9-64 (Plane-4.7.0).
// Free function stabilize_run; ModeStabilize::run stays a no-op
// (ADR-0012: no AP:: / motors / attitude_control objects). Tests call
// this function with injected sticks, spool, and kinematics state.
//
// Calls the real fwcpp::control::input_euler_angle_roll_pitch_euler_rate_yaw_rad
// (CCP-029). Pilot lean / yaw / throttle via CCP-037. update_simple_mode
// is remaining (catalogued). set_throttle_out returns the throttle value
// plus angle_boost=true; boost math is not ported this slice.
//
// ACRO run is mode_acro.hpp (CCP-039 slice 2). ALTHOLD run() stays empty.

#include <cstddef>
#include <cstdint>

#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/copter/pilot_input.hpp>

namespace fwcpp::copter {

// AP_Motors_Class.h DesiredSpoolState ~171-175 / SpoolState ~184-190.
// Values match upstream. This slice returns the desired enumerator; it
// does not call a motors set_desired_spool_state object.
enum class DesiredSpoolState : std::uint8_t {
    SHUT_DOWN = 0,
    GROUND_IDLE = 1,
    THROTTLE_UNLIMITED = 2,
};

enum class SpoolState : std::uint8_t {
    SHUT_DOWN = 0,
    GROUND_IDLE = 1,
    SPOOLING_UP = 2,
    THROTTLE_UNLIMITED = 3,
    SPOOLING_DOWN = 4,
};

struct StabilizeRunInputs {
    bool has_valid_input{false};
    float roll_norm_dz{0.0f};
    float pitch_norm_dz{0.0f};
    float yaw_norm_dz{0.0f};
    float lean_angle_max_rad{0.0f};
    // Upstream g2.command_model_pilot_y.get_rate() — deg/s.
    float command_model_pilot_y_rate{0.0f};
    float yaw_expo{0.0f};
    std::int16_t mid_stick{500};
    std::int16_t throttle_control{0};
    float throttle_hover{0.5f};
    bool throttle_zero{true};
    SpoolState spool_state{SpoolState::SHUT_DOWN};
    bool throttle_lower_limit{false};
    bool land_complete{true};

    math::Quaternion attitude_body{};
    math::Vector3f gyro_body_rads{};
    control::EulerAngleRateShapingGains gains{};
    float dt{0.0f};
};

struct StabilizeRunResult {
    LeanAnglesRad lean{};
    float target_yaw_rate_rads{0.0f};
    DesiredSpoolState desired_spool{DesiredSpoolState::SHUT_DOWN};
    bool reset_yaw_target_and_rate{false};
    bool reset_I{false};
    bool reset_I_smoothly{false};
    bool land_complete{true};
    float throttle_out{0.0f};
    bool angle_boost{true};
    bool input_euler_angle_invoked{false};
    float thrust_angle_rad{0.0f};
    float thrust_error_angle_rad{0.0f};
    float feedforward_scalar{0.0f};
    math::Quaternion attitude_ang_error{};
    math::Vector3f ang_vel_body_rads{};
};

// Upstream ModeStabilize::run ~9-64. state is mutated in place by the
// real CCP-029 entry point (same as attitude_control's target).
[[nodiscard]] inline StabilizeRunResult stabilize_run(const StabilizeRunInputs& in,
                                                      control::AttitudeTargetState& state) {
    StabilizeRunResult out{};
    out.land_complete = in.land_complete;

    // update_simple_mode() (Copter.cpp ~857) is remaining. Sticks are
    // injected already-transformed.

    const float angle_max = in.lean_angle_max_rad;
    out.lean = get_pilot_desired_lean_angles_rad(in.has_valid_input, in.roll_norm_dz, in.pitch_norm_dz,
                                                 angle_max, angle_max);

    out.target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads(
        in.has_valid_input, in.yaw_norm_dz, in.command_model_pilot_y_rate, in.yaw_expo);

    if (in.throttle_zero) {
        out.desired_spool = DesiredSpoolState::GROUND_IDLE;
    } else {
        out.desired_spool = DesiredSpoolState::THROTTLE_UNLIMITED;
    }

    float pilot_desired_throttle =
        get_pilot_desired_throttle(in.mid_stick, in.throttle_control, in.throttle_hover);

    switch (in.spool_state) {
        case SpoolState::SHUT_DOWN:
            out.reset_yaw_target_and_rate = true;
            out.reset_I = true;
            pilot_desired_throttle = 0.0f;
            break;
        case SpoolState::GROUND_IDLE:
            out.reset_yaw_target_and_rate = true;
            out.reset_I_smoothly = true;
            pilot_desired_throttle = 0.0f;
            break;
        case SpoolState::THROTTLE_UNLIMITED:
            if (!in.throttle_lower_limit) {
                out.land_complete = false;
            }
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::SPOOLING_DOWN:
            break;
    }

    control::input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        out.lean.roll_rad, out.lean.pitch_rad, out.target_yaw_rate_rads, state, in.attitude_body,
        in.gyro_body_rads, in.gains, in.dt, out.thrust_angle_rad, out.thrust_error_angle_rad,
        out.feedforward_scalar, out.attitude_ang_error, out.ang_vel_body_rads);
    out.input_euler_angle_invoked = true;

    out.throttle_out = pilot_desired_throttle;
    out.angle_boost = true;
    return out;
}

// Nested so leftover remaining_count() does not collide with
// copter_leftover.hpp / mode_leftover.hpp in fwcpp::copter.
namespace stabilize {

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
    {"stabilize_run", PortStatus::kThisSlice,
     "mode_stabilize.cpp ~9-64; pilot lean/yaw/throttle + spool switch + "
     "real input_euler_angle_roll_pitch_euler_rate_yaw_rad"},
    {"desired spool from throttle_zero", PortStatus::kThisSlice,
     "GROUND_IDLE vs THROTTLE_UNLIMITED; no motors object"},
    {"spool_state switch", PortStatus::kThisSlice,
     "SHUT_DOWN reset_yaw+reset_I throttle=0; GROUND_IDLE reset_yaw+"
     "reset_I_smoothly throttle=0; THROTTLE_UNLIMITED clears land_complete; "
     "spooling no extra"},
    {"set_throttle_out flag", PortStatus::kThisSlice,
     "returns throttle_out + angle_boost=true; boost math not this slice"},
    {"update_simple_mode", PortStatus::kRemaining,
     "mode.cpp ~1095 / Copter.cpp ~857; sticks injected already-transformed"},
    {"set_throttle_out angle-boost math", PortStatus::kRemaining,
     "AC_AttitudeControl_Multi::set_throttle_out; flag only this slice"},
    {"motors set_desired_spool_state object", PortStatus::kRemaining,
     "AP_Motors::set_desired_spool_state; enumerator returned only"},
    {"reset_yaw_target_and_rate / reset_I bodies", PortStatus::kRemaining,
     "attitude_kinematics leftover; flags only this slice"},
    {"acro_run", PortStatus::kRemaining,
     "mode_acro.cpp; rates this slice live in mode_acro leftover"},
    {"althold_run", PortStatus::kRemaining,
     "mode_althold.cpp; needs get_pilot_desired_climb_rate leftover"},
    {"trainer LEVEL/LIMITED", PortStatus::kRemaining,
     "mode_acro.cpp ~133-193; earth-frame level mix; catalogued in acro leftover"},
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

}  // namespace stabilize

}  // namespace fwcpp::copter
