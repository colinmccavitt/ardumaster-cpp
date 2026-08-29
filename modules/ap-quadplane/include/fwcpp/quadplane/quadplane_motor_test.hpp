#pragma once

// motors_output / update gate against motor_test.running —
// upstream QuadPlane::update (Plane-4.7.0 quadplane.cpp ~1739).
// MAV_CMD_DO_MOTOR_TEST / motor_test_output live in ArduPlane/motor_test.cpp
// and are outside VCP-001 (quadplane.cpp / quadplane.h only).

namespace fwcpp::quadplane {

struct MotorTestState {
    bool running{false};
};

[[nodiscard]] inline constexpr bool motor_test_running(const MotorTestState& state) {
    return state.running;
}

inline bool motor_test_start(MotorTestState& state) {
    state.running = true;
    return true;
}

inline void motor_test_stop(MotorTestState& state) {
    state.running = false;
}

}  // namespace fwcpp::quadplane
