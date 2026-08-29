#pragma once

#include "assist_hysteresis.hpp"
#include "should_assist.hpp"
#include "vtol_assist_completeness.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace fwcpp::vtol_assist {

/// Mirrors upstream VTOL_Assist latch flags (force/speed + hysteresis outputs).
struct AssistActiveLatch {
    bool force_assist{false};
    bool speed_assist{false};
    const AssistHysteresis* alt_error{nullptr};
    const AssistHysteresis* angle_error{nullptr};
};

[[nodiscard]] inline bool in_force_assist(const AssistActiveLatch& latch) {
    return latch.force_assist;
}

[[nodiscard]] inline bool in_speed_assist(const AssistActiveLatch& latch) {
    return latch.speed_assist;
}

[[nodiscard]] inline bool in_alt_assist(const AssistActiveLatch& latch) {
    return latch.alt_error != nullptr && latch.alt_error->is_active();
}

[[nodiscard]] inline bool in_angle_assist(const AssistActiveLatch& latch) {
    return latch.angle_error != nullptr && latch.angle_error->is_active();
}

[[nodiscard]] inline AssistActiveLatch assist_active_latch_from_result(
    const ShouldAssistResult& result, const ShouldAssistHysteresis& hysteresis) {
    return AssistActiveLatch{result.force_assist, result.speed_assist, &hysteresis.alt_error,
                             &hysteresis.angle_error};
}

struct AssistGcsStatustext {
    std::optional<std::string> alt_assist;
    std::optional<std::string> angle_assist;
};

[[nodiscard]] inline AssistGcsStatustext assist_gcs_statustext_from_edges(
    const ShouldAssistResult& result, float height_above_ground_m, int ahrs_roll_deg,
    int ahrs_pitch_deg) {
    AssistGcsStatustext out{};
    if (result.alt_assist_first_edge) {
        out.alt_assist = std::string(kGcsAltAssistPrefix) + " " +
                         std::to_string(height_above_ground_m) + "m";
    }
    if (result.angle_assist_first_edge) {
        out.angle_assist = std::string(kGcsAngleAssistPrefix) + " r=" +
                           std::to_string(ahrs_roll_deg) + " p=" +
                           std::to_string(ahrs_pitch_deg);
    }
    return out;
}

}  // namespace fwcpp::vtol_assist
