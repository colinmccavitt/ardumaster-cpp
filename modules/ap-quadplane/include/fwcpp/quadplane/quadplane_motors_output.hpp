#pragma once

// Leftover VTOL motors-output stub — upstream QuadPlane::motors_output
// (Plane-4.7.0 quadplane.cpp). ADR-0012: Plane arming / SRV / tailsitter
// state is passed via MotorsOutputView.

#include <cstdint>

#include <fwcpp/quadplane/quadplane_options.hpp>

namespace fwcpp::quadplane {

inline constexpr std::uint32_t kMotorsInactiveMs = 100;
inline constexpr std::uint32_t kAttControlRelaxMs = 100;
inline constexpr float kMotorsActiveThrottle = 0.01f;

enum class DesiredSpoolState : std::uint8_t {
    kShutDown = 0,
    kGroundIdle = 1,
    kThrottleUnlimited = 2,
};

enum class SpoolState : std::uint8_t {
    kShutDown = 0,
    kGroundIdle = 1,
    kSpoolingUp = 2,
    kThrottleUnlimited = 3,
    kSpoolingDown = 4,
};

enum class MotorsOutputAction : std::uint8_t {
    kDelayArming = 0,
    kDisarmed = 1,
    kEscCalibration = 2,
    kTailsitterTransition = 3,
    kOutput = 4,
    kMotorTest = 5,
};

struct MotorsOutputTick {
    MotorsOutputAction action{MotorsOutputAction::kOutput};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool motors_output_ran{false};
    bool rate_controller_ran{false};
    bool attitude_relaxed{false};
    bool motors_inactive{false};
};

struct MotorsOutputView {
    bool arming_delay_active{false};
    bool armed_and_safety_off{false};
    bool emergency_stop{false};
    bool afs_should_crash{false};
    bool esc_calibration_qstabilize{false};
    bool tailsitter_in_vtol_transition{false};
    bool run_rate_controller{true};
    std::uint32_t now_ms{0};
    float motors_throttle{0.f};
    bool tiltrotor_motors_active{false};
    bool motor_test_running{false};
};

struct MotorsOutputState {
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    std::uint32_t last_motors_active_ms{0};
    std::uint32_t last_att_control_ms{0};
};

[[nodiscard]] inline constexpr bool motors_delay_arming_gate(std::int32_t options,
                                                               bool arming_delay_active) {
    if (!arming_delay_active) {
        return false;
    }
    return option_is_set(options, QOption::kDelayArming) ||
           option_is_set(options, QOption::kDisarmedTilt);
}

[[nodiscard]] inline constexpr bool motors_output_skip_tailsitter_transition(
    bool tailsitter_in_vtol_transition, bool assisted_flight) {
    return tailsitter_in_vtol_transition && !assisted_flight;
}

[[nodiscard]] inline constexpr bool att_control_relax_stale(std::uint32_t now_ms,
                                                              std::uint32_t last_att_control_ms) {
    return (now_ms - last_att_control_ms) > kAttControlRelaxMs;
}

[[nodiscard]] inline constexpr bool motors_inactive(std::uint32_t now_ms,
                                                      std::uint32_t last_motors_active_ms) {
    return (now_ms - last_motors_active_ms) > kMotorsInactiveMs;
}

[[nodiscard]] inline constexpr bool motors_were_active(float motors_throttle,
                                                         bool tiltrotor_motors_active) {
    return motors_throttle > kMotorsActiveThrottle || tiltrotor_motors_active;
}

inline MotorsOutputTick motors_output_shutdown(MotorsOutputAction action, MotorsOutputView view,
                                                 MotorsOutputState& state) {
    state.desired_spool = DesiredSpoolState::kShutDown;
    MotorsOutputTick tick{};
    tick.action = action;
    tick.desired_spool = DesiredSpoolState::kShutDown;
    tick.motors_output_ran = true;
    tick.motors_inactive = motors_inactive(view.now_ms, state.last_motors_active_ms);
    return tick;
}

inline MotorsOutputTick run_motors_output(MotorsOutputView view, std::int32_t options,
                                            bool assisted_flight, MotorsOutputState& state) {
    if (view.motor_test_running) {
        return motors_output_shutdown(MotorsOutputAction::kMotorTest, view, state);
    }
    if (motors_delay_arming_gate(options, view.arming_delay_active)) {
        return motors_output_shutdown(MotorsOutputAction::kDelayArming, view, state);
    }
    if (!view.armed_and_safety_off || view.emergency_stop || view.afs_should_crash) {
        return motors_output_shutdown(MotorsOutputAction::kDisarmed, view, state);
    }
    if (view.esc_calibration_qstabilize) {
        MotorsOutputTick tick{};
        tick.action = MotorsOutputAction::kEscCalibration;
        tick.desired_spool = state.desired_spool;
        tick.motors_inactive = motors_inactive(view.now_ms, state.last_motors_active_ms);
        return tick;
    }
    if (motors_output_skip_tailsitter_transition(view.tailsitter_in_vtol_transition,
                                                   assisted_flight)) {
        MotorsOutputTick tick{};
        tick.action = MotorsOutputAction::kTailsitterTransition;
        tick.desired_spool = state.desired_spool;
        tick.motors_inactive = motors_inactive(view.now_ms, state.last_motors_active_ms);
        return tick;
    }

    bool attitude_relaxed = false;
    if (view.run_rate_controller) {
        if (att_control_relax_stale(view.now_ms, state.last_att_control_ms)) {
            attitude_relaxed = true;
        }
        state.last_att_control_ms = view.now_ms;
    }
    const bool inactive = motors_inactive(view.now_ms, state.last_motors_active_ms);
    if (motors_were_active(view.motors_throttle, view.tiltrotor_motors_active)) {
        state.last_motors_active_ms = view.now_ms;
    }

    MotorsOutputTick tick{};
    tick.action = MotorsOutputAction::kOutput;
    tick.desired_spool = state.desired_spool;
    tick.motors_output_ran = true;
    tick.rate_controller_ran = view.run_rate_controller;
    tick.attitude_relaxed = attitude_relaxed;
    tick.motors_inactive = inactive;
    return tick;
}

}  // namespace fwcpp::quadplane
