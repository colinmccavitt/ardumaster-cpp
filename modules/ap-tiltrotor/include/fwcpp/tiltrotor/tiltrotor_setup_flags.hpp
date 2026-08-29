#pragma once

// Leftover Tiltrotor::setup() after the enable heuristic (Plane-4.7.0
// tiltrotor.cpp 109-145). Caller injects SRV assigned flags and motor
// enabled bits; this port does not persist params, call motors, or
// allocate Tiltrotor_Transition (NEW_NOTHROW stays kOutOfScope).
//
// thrust_type = TILTROTOR is returned as set_thrust_tiltrotor (do not
// include quadplane headers from here — ap-quadplane depends on this
// module). Caller applies QuadPlane::ThrustType::kTiltrotor.

#include <cstdint>

#include <fwcpp/tiltrotor/tiltrotor_predicates.hpp>
#include <fwcpp/tiltrotor/tiltrotor_setup.hpp>
#include <fwcpp/tiltrotor/tiltrotor_types.hpp>

namespace fwcpp::tiltrotor {

inline constexpr std::uint8_t kSetupMotorScanCap = 32;
inline constexpr std::uint16_t kTiltServoRange = 1000;

struct TiltrotorThrottleAssigned {
    bool throttle{false};
    bool throttle_left{false};
    bool throttle_right{false};
};

struct TiltServoRangeFlags {
    bool left{false};
    bool right{false};
    bool rear{false};
    bool rear_left{false};
    bool rear_right{false};

    [[nodiscard]] constexpr bool any() const {
        return left || right || rear || rear_left || rear_right;
    }

    [[nodiscard]] constexpr bool all() const {
        return left && right && rear && rear_left && rear_right;
    }
};

struct TiltrotorSetupFlagInputs {
    TiltrotorThrottleAssigned throttle{};
    std::uint32_t motor_enabled{0};
    std::uint8_t num_motors{0};
    std::uint16_t tilt_mask{0};
    TiltType type{TiltType::kContinuous};
};

struct TiltrotorSetupFlags {
    bool set_thrust_tiltrotor{false};
    bool is_vectored{false};
    bool have_fw_motor{false};
    bool have_vtol_motor{false};
    bool disable_yaw_torque{false};
    bool bind_thrust_compensation{false};
    TiltServoRangeFlags tilt_servo_range{};
};

struct TiltrotorSetupWithFlags {
    TiltrotorSetupResult setup{};
    TiltrotorSetupFlags flags{};
};

[[nodiscard]] inline constexpr bool resolve_have_fw_motor(const TiltrotorThrottleAssigned& assigned,
                                                          TiltType type) {
    return assigned.throttle ||
           ((assigned.throttle_left || assigned.throttle_right) && type != TiltType::kBicopter);
}

[[nodiscard]] inline constexpr bool resolve_have_vtol_motor(std::uint32_t motor_enabled,
                                                            std::uint8_t num_motors,
                                                            std::uint16_t tilt_mask) {
    const std::uint8_t n = num_motors < kSetupMotorScanCap ? num_motors : kSetupMotorScanCap;
    for (std::uint8_t i = 0; i < n; ++i) {
        const bool enabled = ((motor_enabled >> i) & 1u) != 0u;
        if (enabled && !is_motor_tilting(tilt_mask, i)) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr TiltServoRangeFlags resolve_tilt_servo_ranges(std::uint16_t tilt_mask,
                                                                             TiltType type) {
    TiltServoRangeFlags out{};
    if (tilt_mask != 0u && type == TiltType::kVectoredYaw) {
        out.left = true;
        out.right = true;
        out.rear = true;
        out.rear_left = true;
        out.rear_right = true;
    }
    return out;
}

[[nodiscard]] inline constexpr TiltrotorSetupFlags resolve_setup_flags(
    const TiltrotorSetupResult& setup, const TiltrotorSetupFlagInputs& in) {
    TiltrotorSetupFlags out{};
    if (setup.enable <= 0) {
        return out;
    }
    out.set_thrust_tiltrotor = true;
    out.is_vectored = setup.is_vectored;
    out.have_fw_motor = resolve_have_fw_motor(in.throttle, in.type);
    out.have_vtol_motor = resolve_have_vtol_motor(in.motor_enabled, in.num_motors, in.tilt_mask);
    if (out.is_vectored) {
        out.disable_yaw_torque = true;
    }
    if (in.tilt_mask != 0u) {
        out.bind_thrust_compensation = true;
        out.tilt_servo_range = resolve_tilt_servo_ranges(in.tilt_mask, in.type);
    }
    return out;
}

[[nodiscard]] inline TiltrotorSetupWithFlags resolve_setup_with_flags(
    const TiltrotorSetupInputs& setup_in, const TiltrotorSetupFlagInputs& flag_in) {
    TiltrotorSetupWithFlags out{};
    out.setup = resolve_setup(setup_in);
    TiltrotorSetupFlagInputs merged = flag_in;
    merged.tilt_mask = setup_in.tilt_mask;
    merged.type = setup_in.type;
    out.flags = resolve_setup_flags(out.setup, merged);
    return out;
}

}  // namespace fwcpp::tiltrotor
