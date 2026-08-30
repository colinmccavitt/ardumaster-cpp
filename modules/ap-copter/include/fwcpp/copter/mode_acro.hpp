#pragma once

// ModeAcro::run — ArduCopter/mode_acro.cpp ~11-68 and
// get_pilot_desired_rates_rads ~102-end (Plane-4.7.0).
// Free function acro_run; no AP:: / motors / attitude_control objects
// (ADR-0012). Tests inject sticks, spool, AcroOptions, trainer, and
// kinematics state.
//
// This slice: circular stick limit, input_expo rates, trainer OFF plus
// LEVELING / LIMITED earth-frame level mix (wrap_PI, constrain,
// sqrt_controller, euler_derivative_to_body). RATE_LOOP_ONLY calls the
// real input_rate_bf_roll_pitch_yaw_2_rads; otherwise the real
// input_rate_bf_roll_pitch_yaw_rads (CCP-032). set_throttle_out with
// angle_boost=false. scale_I_to_angle_P and AIR_MODE init remain
// catalogued. ALTHOLD takeoff/avoidance leftover stays on mode_althold.
//
// Reuses DesiredSpoolState / SpoolState from mode_stabilize.hpp.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/copter/pilot_input.hpp>
#include <fwcpp/math/control.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::copter {

// mode.h ModeAcro::AcroOptions ~446-448.
enum class AcroOptions : std::uint8_t {
    AIR_MODE = 1 << 0,
    RATE_LOOP_ONLY = 1 << 1,
};

// mode.h ModeAcro::Trainer ~440-444.
enum class AcroTrainer : std::uint8_t {
    OFF = 0,
    LEVELING = 1,
    LIMITED = 2,
};

// config.h ACRO_LEVEL_MAX_ANGLE_RAD / ACRO_LEVEL_MAX_OVERSHOOT_RAD.
[[nodiscard]] inline float acro_level_max_angle_rad() {
    return math::radians(30.0f);
}
[[nodiscard]] inline float acro_level_max_overshoot_rad() {
    return math::radians(10.0f);
}

struct AcroRatesRads {
    float roll_rads{0.0f};
    float pitch_rads{0.0f};
    float yaw_rads{0.0f};
};

struct AcroRunInputs {
    float roll_norm_dz{0.0f};
    float pitch_norm_dz{0.0f};
    float yaw_norm_dz{0.0f};
    // Upstream g2.command_model_acro_rp.get_rate() — deg/s.
    float acro_rp_rate{0.0f};
    float acro_rp_expo{0.0f};
    // Upstream g2.command_model_acro_y.get_rate() — deg/s.
    float acro_y_rate{0.0f};
    float acro_y_expo{0.0f};
    std::uint8_t acro_options{0};
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

    // Injected trainer state (mode_acro.cpp ~133-193). Defaults OFF so
    // existing acro_run callers keep the circular-limit expo path.
    AcroTrainer trainer{AcroTrainer::OFF};
    math::Vector3f att_target_euler_rad{};
    float balance_roll{0.0f};
    float balance_pitch{0.0f};
    float lean_angle_max_rad{0.0f};
    float accel_roll_max_radss{0.0f};
    float accel_pitch_max_radss{0.0f};
    float cos_pitch{1.0f};
    math::Quaternion att_target_quat{};
};

// Injected trainer inputs for get_pilot_desired_rates_rads (no AP::).
struct AcroTrainerInputs {
    AcroTrainer trainer{AcroTrainer::OFF};
    math::Vector3f att_target_euler_rad{};
    float balance_roll{0.0f};
    float balance_pitch{0.0f};
    float lean_angle_max_rad{0.0f};
    float accel_roll_max_radss{0.0f};
    float accel_pitch_max_radss{0.0f};
    float dt{0.0f};
    float cos_pitch{1.0f};
    math::Quaternion att_target_quat{};
};

struct AcroRunResult {
    AcroRatesRads rates{};
    DesiredSpoolState desired_spool{DesiredSpoolState::SHUT_DOWN};
    bool reset_target_and_rate{false};
    bool reset_I{false};
    bool reset_I_smoothly{false};
    bool land_complete{true};
    float throttle_out{0.0f};
    bool angle_boost{false};
    bool rate_loop_only{false};
    bool scale_I_to_angle_P{false};
    bool input_rate_bf_invoked{false};
    bool input_rate_bf_2_invoked{false};
    float thrust_angle_rad{0.0f};
    float thrust_error_angle_rad{0.0f};
    float feedforward_scalar{0.0f};
    math::Quaternion attitude_ang_error{};
    math::Vector3f ang_vel_body_rads{};
};

// Upstream ModeAcro::get_pilot_desired_rates_rads ~102-199.
// Trainer OFF is the circular-limit + input_expo path only. LEVELING and
// LIMITED add earth-frame level rates, convert via
// euler_derivative_to_body, then mix (LEVELING) or add (LIMITED).
[[nodiscard]] inline AcroRatesRads get_pilot_desired_rates_rads(float roll_norm_dz, float pitch_norm_dz,
                                                                float yaw_norm_dz, float acro_rp_rate,
                                                                float acro_rp_expo, float acro_y_rate,
                                                                float acro_y_expo,
                                                                const AcroTrainerInputs& trainer = {}) {
    float roll_in_norm = roll_norm_dz;
    float pitch_in_norm = pitch_norm_dz;
    const float yaw_in_norm = yaw_norm_dz;

    const float norm_in_length = std::hypot(pitch_in_norm, roll_in_norm);
    if (norm_in_length > 1.0f) {
        const float ratio = 1.0f / norm_in_length;
        roll_in_norm *= ratio;
        pitch_in_norm *= ratio;
    }

    math::Vector3f rate_bf_request_rads;
    rate_bf_request_rads.x = math::radians(acro_rp_rate) * input_expo(roll_in_norm, acro_rp_expo);
    rate_bf_request_rads.y = math::radians(acro_rp_rate) * input_expo(pitch_in_norm, acro_rp_expo);
    rate_bf_request_rads.z = math::radians(acro_y_rate) * input_expo(yaw_in_norm, acro_y_expo);

    if (trainer.trainer != AcroTrainer::OFF) {
        math::Vector3f rate_ef_level_rads;

        const float roll_angle_rad = math::wrap_PI(trainer.att_target_euler_rad.x);
        rate_ef_level_rads.x = -math::constrain_value(roll_angle_rad, -acro_level_max_angle_rad(),
                                                     acro_level_max_angle_rad()) *
                               trainer.balance_roll;

        const float pitch_angle_rad = math::wrap_PI(trainer.att_target_euler_rad.y);
        rate_ef_level_rads.y = -math::constrain_value(pitch_angle_rad, -acro_level_max_angle_rad(),
                                                     acro_level_max_angle_rad()) *
                               trainer.balance_pitch;

        rate_ef_level_rads.z = 0.0f;

        if (trainer.trainer == AcroTrainer::LIMITED) {
            const float angle_max_rad = trainer.lean_angle_max_rad;
            const float p = math::radians(acro_rp_rate) / acro_level_max_overshoot_rad();
            if (roll_angle_rad > angle_max_rad) {
                rate_ef_level_rads.x +=
                    math::sqrt_controller(angle_max_rad - roll_angle_rad, p, trainer.accel_roll_max_radss,
                                          trainer.dt);
            } else if (roll_angle_rad < -angle_max_rad) {
                rate_ef_level_rads.x +=
                    math::sqrt_controller(-angle_max_rad - roll_angle_rad, p, trainer.accel_roll_max_radss,
                                          trainer.dt);
            }

            if (pitch_angle_rad > angle_max_rad) {
                rate_ef_level_rads.y +=
                    math::sqrt_controller(angle_max_rad - pitch_angle_rad, p, trainer.accel_pitch_max_radss,
                                          trainer.dt);
            } else if (pitch_angle_rad < -angle_max_rad) {
                rate_ef_level_rads.y +=
                    math::sqrt_controller(-angle_max_rad - pitch_angle_rad, p, trainer.accel_pitch_max_radss,
                                          trainer.dt);
            }
        }

        math::Vector3f rate_bf_level_rads =
            control::euler_derivative_to_body(trainer.att_target_quat, rate_ef_level_rads);

        if (trainer.trainer == AcroTrainer::LIMITED) {
            rate_bf_request_rads.x += rate_bf_level_rads.x;
            rate_bf_request_rads.y += rate_bf_level_rads.y;
            rate_bf_request_rads.z += rate_bf_level_rads.z;
        } else {
            const float stick_abs =
                std::max(std::max(std::fabs(roll_in_norm), std::fabs(pitch_in_norm)), std::fabs(yaw_in_norm));
            const float acro_level_mix =
                math::constrain_value(1.0f - stick_abs, 0.0f, 1.0f) * trainer.cos_pitch;

            rate_bf_level_rads = rate_bf_level_rads * acro_level_mix;

            float rate_delta_max_rads =
                std::fabs(std::fabs(rate_bf_request_rads.x) - std::fabs(rate_bf_level_rads.x));
            rate_bf_request_rads.x += rate_bf_level_rads.x;
            rate_bf_request_rads.x =
                math::constrain_value(rate_bf_request_rads.x, -rate_delta_max_rads, rate_delta_max_rads);

            rate_delta_max_rads =
                std::fabs(std::fabs(rate_bf_request_rads.y) - std::fabs(rate_bf_level_rads.y));
            rate_bf_request_rads.y += rate_bf_level_rads.y;
            rate_bf_request_rads.y =
                math::constrain_value(rate_bf_request_rads.y, -rate_delta_max_rads, rate_delta_max_rads);

            rate_delta_max_rads =
                std::fabs(std::fabs(rate_bf_request_rads.z) - std::fabs(rate_bf_level_rads.z));
            rate_bf_request_rads.z += rate_bf_level_rads.z;
            rate_bf_request_rads.z =
                math::constrain_value(rate_bf_request_rads.z, -rate_delta_max_rads, rate_delta_max_rads);
        }
    }

    AcroRatesRads out;
    out.roll_rads = rate_bf_request_rads.x;
    out.pitch_rads = rate_bf_request_rads.y;
    out.yaw_rads = rate_bf_request_rads.z;
    return out;
}

// Upstream ModeAcro::run ~11-68. state is mutated in place by the real
// CCP-032 entry point (same as attitude_control's target).
[[nodiscard]] inline AcroRunResult acro_run(const AcroRunInputs& in, control::AttitudeTargetState& state) {
    AcroRunResult out{};
    out.land_complete = in.land_complete;

    AcroTrainerInputs trainer;
    trainer.trainer = in.trainer;
    trainer.att_target_euler_rad = in.att_target_euler_rad;
    trainer.balance_roll = in.balance_roll;
    trainer.balance_pitch = in.balance_pitch;
    trainer.lean_angle_max_rad = in.lean_angle_max_rad;
    trainer.accel_roll_max_radss = in.accel_roll_max_radss;
    trainer.accel_pitch_max_radss = in.accel_pitch_max_radss;
    trainer.dt = in.dt;
    trainer.cos_pitch = in.cos_pitch;
    trainer.att_target_quat = in.att_target_quat;

    out.rates = get_pilot_desired_rates_rads(in.roll_norm_dz, in.pitch_norm_dz, in.yaw_norm_dz, in.acro_rp_rate,
                                             in.acro_rp_expo, in.acro_y_rate, in.acro_y_expo, trainer);

    if (in.throttle_zero) {
        out.desired_spool = DesiredSpoolState::GROUND_IDLE;
    } else {
        out.desired_spool = DesiredSpoolState::THROTTLE_UNLIMITED;
    }

    float pilot_desired_throttle =
        get_pilot_desired_throttle(in.mid_stick, in.throttle_control, in.throttle_hover);

    switch (in.spool_state) {
        case SpoolState::SHUT_DOWN:
            out.reset_target_and_rate = true;
            out.reset_I = true;
            pilot_desired_throttle = 0.0f;
            break;
        case SpoolState::GROUND_IDLE:
            out.reset_target_and_rate = true;
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

    out.rate_loop_only =
        (in.acro_options & static_cast<std::uint8_t>(AcroOptions::RATE_LOOP_ONLY)) != 0;
    if (out.rate_loop_only) {
        // scale_I_to_angle_P body is leftover; flag only this slice.
        out.scale_I_to_angle_P = true;
        control::input_rate_bf_roll_pitch_yaw_2_rads(out.rates.roll_rads, out.rates.pitch_rads, out.rates.yaw_rads,
                                                     state, in.attitude_body, in.gains, in.dt,
                                                     out.ang_vel_body_rads);
        out.input_rate_bf_2_invoked = true;
    } else {
        control::input_rate_bf_roll_pitch_yaw_rads(
            out.rates.roll_rads, out.rates.pitch_rads, out.rates.yaw_rads, state, in.attitude_body,
            in.gyro_body_rads, in.gains, in.dt, out.thrust_angle_rad, out.thrust_error_angle_rad,
            out.feedforward_scalar, out.attitude_ang_error, out.ang_vel_body_rads);
        out.input_rate_bf_invoked = true;
    }

    out.throttle_out = pilot_desired_throttle;
    out.angle_boost = false;
    return out;
}

// Nested so leftover remaining_count() does not collide with
// copter_leftover.hpp / mode_leftover.hpp / stabilize leftover.
namespace acro {

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
    {"acro_run", PortStatus::kThisSlice,
     "mode_acro.cpp ~11-68; circular rates + spool switch + real "
     "input_rate_bf_roll_pitch_yaw_rads / _2_rads"},
    {"get_pilot_desired_rates_rads", PortStatus::kThisSlice,
     "mode_acro.cpp ~102-199; hypot circular limit + input_expo + trainer mix"},
    {"circular stick limit", PortStatus::kThisSlice,
     "if hypot(pitch,roll)>1 scale both"},
    {"RATE_LOOP_ONLY vs stabilized input_rate_bf", PortStatus::kThisSlice,
     "_2_rads when RATE_LOOP_ONLY else _rads; both real CCP-032 entries"},
    {"set_throttle_out flag", PortStatus::kThisSlice,
     "returns throttle_out + angle_boost=false; boost math not this slice"},
    {"stabilize_run", PortStatus::kOnMain,
     "mode_stabilize.hpp; CCP-039 slice 1 on main"},
    {"althold_run", PortStatus::kOnMain,
     "mode_althold.hpp; CCP-039 slice 3 on main; takeoff/avoidance leftover"},
    {"trainer LEVEL/LIMITED", PortStatus::kThisSlice,
     "mode_acro.cpp ~133-193; wrap_PI + constrain + sqrt_controller + euler_derivative_to_body mix"},
    {"scale_I_to_angle_P", PortStatus::kRemaining,
     "AC_AttitudeControl::scale_I_to_angle_P; flag only if RATE_LOOP_ONLY"},
    {"AIR_MODE init", PortStatus::kRemaining,
     "ModeAcro::init/exit/air_mode_aux_changed; AIR_MODE bit leftover"},
    {"reset_target_and_rate / reset_I bodies", PortStatus::kRemaining,
     "attitude_kinematics leftover; flags only this slice"},
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

}  // namespace acro

}  // namespace fwcpp::copter
