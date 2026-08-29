#pragma once

#include <cstdint>

#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_enable.hpp>
#include <fwcpp/tailsitter/tailsitter_input_type.hpp>
#include <fwcpp/tailsitter/tailsitter_transition.hpp>
#include <fwcpp/tailsitter/tailsitter_transition_ramp.hpp>

namespace fwcpp::tailsitter {

[[nodiscard]] inline constexpr bool tailsitter_active(const TailsitterGate& gate, bool in_vtol_mode,
                                                      bool angle_wait_fw) {
    return gate.enabled() && (in_vtol_mode || angle_wait_fw);
}

[[nodiscard]] inline std::int16_t check_input_remapped_roll(std::int16_t roll, std::int16_t yaw,
                                                            const TailsitterGate& gate,
                                                            std::int8_t input_type, bool in_vtol_mode,
                                                            bool angle_wait_fw) {
    if (tailsitter_active(gate, in_vtol_mode, angle_wait_fw) && input_plane_mode(input_type)) {
        return yaw;
    }
    return roll;
}

[[nodiscard]] inline std::int16_t check_input_remapped_yaw(std::int16_t roll, std::int16_t yaw,
                                                           const TailsitterGate& gate,
                                                           std::int8_t input_type, bool in_vtol_mode,
                                                           bool angle_wait_fw) {
    if (tailsitter_active(gate, in_vtol_mode, angle_wait_fw) && input_plane_mode(input_type)) {
        const int wide = -static_cast<int>(roll);
        if (wide < -32768) {
            return -32768;
        }
        if (wide > 32767) {
            return 32767;
        }
        return static_cast<std::int16_t>(wide);
    }
    return yaw;
}

[[nodiscard]] inline constexpr bool in_vtol_transition(bool enabled, bool in_vtol_mode,
                                                       TailsitterTransitionState transition_state,
                                                       std::uint32_t now_ms,
                                                       std::uint32_t last_vtol_mode_ms) {
    if (!enabled || !in_vtol_mode) {
        return false;
    }
    if (transition_state == TailsitterTransitionState::kAngleWaitVtol) {
        return true;
    }
    if (now_ms != 0 && (now_ms - last_vtol_mode_ms) > kLastVtolModeMs) {
        return true;
    }
    return false;
}

[[nodiscard]] inline constexpr std::int8_t get_transition_angle_vtol(const TransitionRamp& ramp) {
    return ramp.get_transition_angle_vtol();
}

}  // namespace fwcpp::tailsitter
