#pragma once

#include "vtol_assist.hpp"

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

#include <cmath>
#include <cstdint>

namespace fwcpp::vtol_assist {

/// Upstream SERVO_MAX (centidegrees) for spin-recovery surface commands.
inline constexpr float kAssistServoMax = 4500.0f;

/// Injected plane/quadplane/ahrs inputs (ADR-0012 — no QuadPlane singleton).
struct AssistRecoveryInputs {
    bool tailsitter_enabled{false};
    bool in_qacro_mode{false};
    float roll_sensor_cd{0.0f};
    float pitch_sensor_cd{0.0f};
    float lean_angle_max_cd{0.0f};
    float groundspeed_ms{0.0f};
    float wp_default_speed_ne_ms{0.0f};
    math::Vector3f gyro{};
    float pitch_deg{0.0f};
    bool vtol_motors_throttle_unlimited{false};
};

/// Latched recovery flags normally held on QuadPlane upstream.
struct AssistRecoveryLatch {
    bool force_fw_control_recovery{false};
    bool in_spin_recovery{false};
};

/// Explicit downstream actions the integrator applies (reset/init hooks).
struct AssistRecoveryActions {
    bool reset_attitude_target_and_rate{false};
    bool pos_control_d_init{false};
    bool pos_control_ne_init{false};
};

struct SpinRecoveryOutputs {
    bool apply_surfaces{false};
    float rudder_scaled{0.0f};
    float elevator_scaled{0.0f};
    bool cleared_in_spin_recovery{false};
};

[[nodiscard]] inline bool allow_fw_vtol_recovery(const VtolAssist& assist,
                                                 const AssistRecoveryInputs& in) {
    return !assist.option_is_set(AssistOption::kFwForceDisabled) && !in.tailsitter_enabled &&
           !in.in_qacro_mode;
}

[[nodiscard]] inline float recovery_abs_angle_cd(const AssistRecoveryInputs& in) {
    return std::fabs(math::Vector2f{in.roll_sensor_cd, in.pitch_sensor_cd}.length());
}

[[nodiscard]] inline bool evaluate_spin_recovery_latch(const VtolAssist& assist,
                                                       const AssistRecoveryInputs& in,
                                                       bool force_fw_control_recovery) {
    if (assist.option_is_set(AssistOption::kSpinDisabled) || !force_fw_control_recovery) {
        return false;
    }
    const math::Vector3f& gyro = in.gyro;
    return std::fabs(gyro.z) > math::radians(10.0f) && std::fabs(gyro.x) > math::radians(30.0f) &&
           std::fabs(gyro.y) > math::radians(30.0f) && gyro.x * gyro.z < 0.0f && in.pitch_deg < -45.0f;
}

/*
  Port of VTOL_Assist::check_VTOL_recovery — updates latch + action flags;
  returns whether fixed-wing recovery control is active.
 */
[[nodiscard]] inline bool check_vtol_recovery(const VtolAssist& assist, const AssistRecoveryInputs& in,
                                              AssistRecoveryLatch& latch, AssistRecoveryActions& actions) {
    actions = {};

    if (!allow_fw_vtol_recovery(assist, in)) {
        latch.force_fw_control_recovery = false;
        latch.in_spin_recovery = false;
        return false;
    }

    const float abs_angle_cd = recovery_abs_angle_cd(in);

    if (abs_angle_cd > 2.0f * in.lean_angle_max_cd) {
        latch.force_fw_control_recovery = true;
    }

    if (latch.force_fw_control_recovery) {
        if (abs_angle_cd <= in.lean_angle_max_cd) {
            latch.force_fw_control_recovery = false;
            actions.reset_attitude_target_and_rate = true;

            if (in.groundspeed_ms > in.wp_default_speed_ne_ms) {
                actions.pos_control_d_init = true;
                actions.pos_control_ne_init = true;
            }
        }
    }

    latch.in_spin_recovery =
        evaluate_spin_recovery_latch(assist, in, latch.force_fw_control_recovery);

    return latch.force_fw_control_recovery;
}

/*
  Port of VTOL_Assist::output_spin_recovery — explicit surface outputs;
  may clear in_spin_recovery when VTOL motors are no longer unlimited.
 */
[[nodiscard]] inline SpinRecoveryOutputs output_spin_recovery(const AssistRecoveryInputs& in,
                                                              AssistRecoveryLatch& latch) {
    SpinRecoveryOutputs out{};
    if (!latch.in_spin_recovery) {
        return out;
    }

    if (!in.vtol_motors_throttle_unlimited) {
        latch.in_spin_recovery = false;
        out.cleared_in_spin_recovery = true;
        return out;
    }

    const math::Vector3f& gyro = in.gyro;
    out.apply_surfaces = true;
    out.rudder_scaled = gyro.z > 0.0f ? -kAssistServoMax : kAssistServoMax;
    out.elevator_scaled = 0.0f;
    return out;
}

}  // namespace fwcpp::vtol_assist
