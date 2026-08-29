#pragma once

// Upstream Tiltrotor::bicopter_output / get_forward_throttle (Plane-4.7.0
// tiltrotor.cpp 667-710 and 828-853). Header-only tick/effects: return
// flags and scaled servo values, do not call SRV or motors.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/tiltrotor/tiltrotor_control.hpp>
#include <fwcpp/tiltrotor/tiltrotor_defaults.hpp>
#include <fwcpp/tiltrotor/tiltrotor_enable.hpp>
#include <fwcpp/tiltrotor/tiltrotor_predicates.hpp>
#include <fwcpp/tiltrotor/tiltrotor_types.hpp>

namespace fwcpp::tiltrotor {

enum class BicopterOutputPath : std::uint8_t {
    kNone,
    kFullyFwdFixedWing,
    kAssisted,
    kUnassisted,
};

struct BicopterOutputInputs {
    TiltType type{TiltType::kContinuous};
    bool motor_test_running{false};
    bool in_vtol_mode{true};
    bool assisted_flight{false};
    TiltrotorGate gate{};
    std::uint16_t tilt_mask{0};
    float current_tilt{0.0f};
    float flap_angle_deg{0.0f};
    float throttle_scaled{0.0f};
    float tilt_left{0.0f};
    float tilt_right{0.0f};
    float tilt_yaw_angle{kTiltYawAngleDefault};
};

struct BicopterOutputResult {
    BicopterOutputPath path{BicopterOutputPath::kNone};
    bool write_tilt_servos{false};
    float tilt_left{0.0f};
    float tilt_right{0.0f};
    bool hold_stabilize{false};
    float hold_stabilize_throttle{0.0f};
    bool motors_output{false};
    bool motors_output_assisted{false};
};

[[nodiscard]] inline float bicopter_scale_negative_tilt(float tilt, float tilt_yaw_angle) {
    if (fwcpp::math::is_negative(tilt)) {
        tilt *= tilt_yaw_angle * kDegreesPerTiltUnit;
    }
    return tilt;
}

[[nodiscard]] inline float bicopter_constrain_tilt(float current_tilt, float tilt) {
    return fwcpp::math::constrain_value(-(current_tilt * kServoMax) + tilt, -kServoMax, kServoMax);
}

// Upstream Tiltrotor::bicopter_output: motor-test / non-bicopter is a
// no-op (do not override motor test). Fully-forward FW writes -SERVO_MAX
// and skips the throttle/motors path. Else: assisted hold_stabilize +
// motors_output(true), or motors_output(false), then scale/constrain tilts.
[[nodiscard]] inline BicopterOutputResult bicopter_output(const BicopterOutputInputs& in) {
    BicopterOutputResult out{};
    if (in.type != TiltType::kBicopter || in.motor_test_running) {
        return out;
    }

    if (!in.in_vtol_mode && fully_fwd(in.gate, in.tilt_mask, in.current_tilt, in.flap_angle_deg)) {
        out.path = BicopterOutputPath::kFullyFwdFixedWing;
        out.write_tilt_servos = true;
        out.tilt_left = -kServoMax;
        out.tilt_right = -kServoMax;
        return out;
    }

    if (in.assisted_flight) {
        out.path = BicopterOutputPath::kAssisted;
        out.hold_stabilize = true;
        out.hold_stabilize_throttle = in.throttle_scaled * kThrottleScaledToUnit;
        out.motors_output = true;
        out.motors_output_assisted = true;
    } else {
        out.path = BicopterOutputPath::kUnassisted;
        out.motors_output = true;
        out.motors_output_assisted = false;
    }

    float tilt_left = bicopter_scale_negative_tilt(in.tilt_left, in.tilt_yaw_angle);
    float tilt_right = bicopter_scale_negative_tilt(in.tilt_right, in.tilt_yaw_angle);

    const float scaling = std::cos(in.current_tilt * static_cast<float>(M_PI_2));
    tilt_left *= scaling;
    tilt_right *= scaling;

    out.write_tilt_servos = true;
    out.tilt_left = bicopter_constrain_tilt(in.current_tilt, tilt_left);
    out.tilt_right = bicopter_constrain_tilt(in.current_tilt, tilt_right);
    return out;
}

struct ForwardThrottleThrLin {
    float spin_min{0.0f};
    float spin_max{1.0f};

    [[nodiscard]] float thrust_to_actuator(float thrust) const {
        return spin_min + (spin_max - spin_min) * fwcpp::math::constrain_value(thrust, 0.0f, 1.0f);
    }
};

struct ForwardThrottleMotorLookup {
    bool ok[kMaxNumMotors]{};
    float thrust[kMaxNumMotors]{};

    [[nodiscard]] bool get_thrust(std::uint8_t i, float& thrust_out) const {
        if (i >= kMaxNumMotors || !ok[i]) {
            return false;
        }
        thrust_out = thrust[i];
        return true;
    }
};

struct ForwardThrottleInputs {
    TiltrotorGate gate{};
    bool is_vectored{false};
    std::uint16_t tilt_mask{0};
    ForwardThrottleThrLin thr_lin{};
    ForwardThrottleMotorLookup motors{};
};

struct ForwardThrottleResult {
    bool ok{false};
    float throttle{0.0f};
};

// Upstream Tiltrotor::get_forward_throttle: average of tilting motors'
// (thrust_to_actuator(thrust) - spin_min) / (spin_max - spin_min).
// Scan is capped at AP_MOTORS_MAX_NUM_MOTORS (32).
[[nodiscard]] inline ForwardThrottleResult get_forward_throttle(const ForwardThrottleInputs& in) {
    ForwardThrottleResult out{};
    if (!in.gate.enabled() || !in.is_vectored) {
        return out;
    }
    const float throttle_range = in.thr_lin.spin_max - in.thr_lin.spin_min;
    if (!fwcpp::math::is_positive(throttle_range)) {
        return out;
    }
    float throttle_sum = 0.0f;
    std::uint8_t num_vectored_motors = 0;
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        if (is_motor_tilting(in.tilt_mask, i)) {
            float thrust = 0.0f;
            if (in.motors.get_thrust(i, thrust)) {
                throttle_sum +=
                    (in.thr_lin.thrust_to_actuator(thrust) - in.thr_lin.spin_min) / throttle_range;
                ++num_vectored_motors;
            }
        }
    }
    if (num_vectored_motors > 0) {
        out.throttle = throttle_sum / static_cast<float>(num_vectored_motors);
        out.ok = true;
    }
    return out;
}

}  // namespace fwcpp::tiltrotor
