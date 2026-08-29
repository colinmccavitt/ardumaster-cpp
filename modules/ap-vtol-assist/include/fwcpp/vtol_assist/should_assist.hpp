#pragma once

#include "assist_hysteresis.hpp"
#include "assist_inputs.hpp"
#include "assist_triggers.hpp"
#include "vtol_assist.hpp"

namespace fwcpp::vtol_assist {

enum class FlareMode : std::uint8_t { kDisabled, kActive };

struct ShouldAssistGates {
    bool armed_and_safety_off{false};
    AssistState assist_state{AssistState::kAssistEnabled};
    bool control_surface_tailsitter{false};
    bool tailsitter_enabled{false};
    bool does_auto_throttle{false};
    bool throttle_suppressed{false};
    float throttle_input{0.0f};
    bool is_flying{false};
    FlareMode flare_mode{FlareMode::kDisabled};
};

[[nodiscard]] inline bool should_assist_gates_open(const ShouldAssistGates& gates) {
    if (!gates.armed_and_safety_off || gates.assist_state == AssistState::kAssistDisabled ||
        gates.control_surface_tailsitter) {
        return false;
    }

    if (!gates.tailsitter_enabled &&
        !((gates.does_auto_throttle && !gates.throttle_suppressed) || gates.throttle_input > 0.0f ||
          gates.is_flying)) {
        return false;
    }

    if (gates.flare_mode != FlareMode::kDisabled) {
        return false;
    }

    return true;
}

struct ShouldAssistSlice1Result {
    bool requested{false};
    bool force_assist{false};
    bool speed_assist{false};
};

[[nodiscard]] inline ShouldAssistSlice1Result should_assist_slice1(const VtolAssist& assist,
                                                                   const ShouldAssistGates& gates,
                                                                   SpeedAssistSample speed_sample) {
    ShouldAssistSlice1Result out{};
    if (!should_assist_gates_open(gates)) {
        return out;
    }

    out.force_assist = force_assist_active(gates.assist_state);
    if (assist.speed() <= 0.0f) {
        out.requested = out.force_assist;
        return out;
    }

    out.speed_assist = evaluate_speed_assist(assist.speed(), speed_sample);
    out.requested = out.force_assist || out.speed_assist;
    return out;
}

struct ShouldAssistHysteresis {
    AssistHysteresis alt_error{};
    AssistHysteresis angle_error{};

    void reset() {
        alt_error.reset();
        angle_error.reset();
    }
};

struct ShouldAssistResult {
    bool requested{false};
    bool force_assist{false};
    bool speed_assist{false};
    bool alt_assist{false};
    bool angle_assist{false};
    bool alt_assist_first_edge{false};
    bool angle_assist_first_edge{false};
};

[[nodiscard]] inline ShouldAssistResult should_assist(const VtolAssist& assist,
                                                      const ShouldAssistGates& gates,
                                                      const AssistInputs& inputs, std::uint32_t now_ms,
                                                      ShouldAssistHysteresis& hysteresis) {
    ShouldAssistResult out{};
    if (!should_assist_gates_open(gates)) {
        hysteresis.reset();
        return out;
    }

    out.force_assist = force_assist_active(gates.assist_state);
    if (assist.speed() <= 0.0f) {
        hysteresis.reset();
        out.requested = out.force_assist;
        return out;
    }

    out.speed_assist = evaluate_speed_assist(assist.speed(), inputs.speed);

    const std::uint32_t trig_ms = trigger_delay_ms(assist.delay());
    const std::uint32_t clear_ms = clear_delay_ms(assist.delay());

    if (assist.alt() <= 0) {
        hysteresis.alt_error.reset();
    } else {
        const bool alt_trigger =
            evaluate_alt_assist_trigger(assist, inputs.height_above_ground_m);
        out.alt_assist_first_edge =
            hysteresis.alt_error.update(alt_trigger, now_ms, trig_ms, clear_ms);
        out.alt_assist = hysteresis.alt_error.is_active();
    }

    if (assist.angle() <= 0) {
        hysteresis.angle_error.reset();
    } else {
        const bool angle_trigger = evaluate_angle_assist_trigger(
            assist, inputs.nav_roll_cd, inputs.nav_pitch_cd, inputs.ahrs_roll_deg,
            inputs.ahrs_pitch_deg, inputs.roll_limit_deg, inputs.pitch_limit_max_deg,
            inputs.pitch_limit_min_deg);
        out.angle_assist_first_edge =
            hysteresis.angle_error.update(angle_trigger, now_ms, trig_ms, clear_ms);
        out.angle_assist = hysteresis.angle_error.is_active();
    }

    out.requested =
        out.force_assist || out.speed_assist || out.alt_assist || out.angle_assist;
    return out;
}

}  // namespace fwcpp::vtol_assist
