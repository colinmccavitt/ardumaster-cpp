#pragma once

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

}  // namespace fwcpp::vtol_assist
