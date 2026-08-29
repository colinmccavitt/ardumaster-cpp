#pragma once

#include "assist_triggers.hpp"
#include <cstdint>

namespace fwcpp::vtol_assist {

/// Injected plane/ahrs/nav inputs for should_assist (ADR-0012 — no QuadPlane singleton).
struct AssistInputs {
    SpeedAssistSample speed{};
    /// Upstream relative_ground_altitude(RangeFinderUse::ASSIST), metres AGL.
    float height_above_ground_m{0.0f};
    float ahrs_roll_deg{0.0f};
    float ahrs_pitch_deg{0.0f};
    std::int32_t nav_roll_cd{0};
    std::int32_t nav_pitch_cd{0};
    float roll_limit_deg{0.0f};
    float pitch_limit_max_deg{0.0f};
    float pitch_limit_min_deg{0.0f};
};

}  // namespace fwcpp::vtol_assist
