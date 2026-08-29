#pragma once

#include <fwcpp/q_loiter/loiter_alt_qland.hpp>

namespace fwcpp::q_loiter {

struct LoiterAltQlandGuidedEffects {
    bool set_guided_wp{false};
    float alt_m{0.0F};
    GuidedAltFrame alt_frame{GuidedAltFrame::kAboveHome};
};

/// Port of ModeLoiterAltQLand::handle_guided_request.
/// ABOVE_TERRAIN only when terrain is available and enabled in QLAND; else ABOVE_HOME.
/// Always `set_guided_WP` afterwards (upstream returns true).
[[nodiscard]] inline LoiterAltQlandGuidedEffects loiter_alt_qland_handle_guided(
    float qrtl_alt_m, bool terrain_enabled_in_qland, bool terrain_available = true) {
    LoiterAltQlandGuidedEffects out{};
    out.set_guided_wp = true;
    out.alt_m = qrtl_alt_m;
    out.alt_frame = (terrain_available && terrain_enabled_in_qland)
                        ? GuidedAltFrame::kAboveTerrain
                        : GuidedAltFrame::kAboveHome;
    return out;
}

[[nodiscard]] inline LoiterAltQlandGuidedEffects loiter_alt_qland_handle_guided_request(
    const LoiterAltQlandEnterInputs& in) {
    return loiter_alt_qland_handle_guided(in.qrtl_alt_m, in.terrain_enabled);
}

}  // namespace fwcpp::q_loiter
