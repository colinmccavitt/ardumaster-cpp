#pragma once

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/q_loiter/q_loiter_defaults.hpp>

#include <cstdint>

namespace fwcpp::q_loiter {

enum class LoiterAltQlandEnterAction : std::uint8_t {
    kSwitchQlandImmediate = 0,
    kFwLoiterThenGuided = 1,
};

enum class LoiterAltQlandSwitchAction : std::uint8_t {
    kNone = 0,
    kSwitchQland = 1,
};

enum class GuidedAltFrame : std::uint8_t {
    kAboveHome = 0,
    kAboveTerrain = 1,
};

struct LoiterAltQlandEnterInputs {
    bool previous_mode_is_vtol{false};
    bool in_vtol_mode{false};
    bool already_in_a_loiter{false};
    bool reached_loiter_target{false};
    bool nav_data_stale{false};
    float qrtl_alt_m{kQrtlAltDefaultM};
    bool terrain_enabled{false};
};

struct LoiterAltQlandEnterEffects {
    bool request_qland_mode{false};
    bool mode_loiter_enter{false};
    bool handle_guided_request{false};
    bool switch_qland_after_enter{false};
};

struct LoiterAltQlandEnterResult {
    LoiterAltQlandEnterAction action{LoiterAltQlandEnterAction::kFwLoiterThenGuided};
    GuidedAltFrame guided_alt_frame{GuidedAltFrame::kAboveHome};
    float guided_alt_m{kQrtlAltDefaultM};
};

[[nodiscard]] inline bool loiter_alt_already_in_loiter(bool reached_loiter_target,
                                                       bool nav_data_stale) {
    return reached_loiter_target && !nav_data_stale;
}

[[nodiscard]] inline LoiterAltQlandEnterResult loiter_alt_qland_enter(
    const LoiterAltQlandEnterInputs& in, LoiterAltQlandEnterEffects& effects) {
    effects = LoiterAltQlandEnterEffects{};
    LoiterAltQlandEnterResult out{};

    if (in.previous_mode_is_vtol || in.in_vtol_mode) {
        effects.request_qland_mode = true;
        out.action = LoiterAltQlandEnterAction::kSwitchQlandImmediate;
        return out;
    }

    effects.mode_loiter_enter = true;
    effects.handle_guided_request = true;
    effects.switch_qland_after_enter = true;
    out.guided_alt_m = in.qrtl_alt_m;
    out.guided_alt_frame =
        in.terrain_enabled ? GuidedAltFrame::kAboveTerrain : GuidedAltFrame::kAboveHome;
    out.action = LoiterAltQlandEnterAction::kFwLoiterThenGuided;
    return out;
}

struct LoiterAltQlandSwitchInputs {
    bool height_above_valid{true};
    float height_above_m{10.0F};
    bool reached_loiter_target{false};
};

[[nodiscard]] inline LoiterAltQlandSwitchAction loiter_alt_qland_switch(
    const LoiterAltQlandSwitchInputs& in) {
    const bool invalid_or_below =
        !in.height_above_valid || fwcpp::math::is_negative(in.height_above_m);
    if (invalid_or_below && in.reached_loiter_target) {
        return LoiterAltQlandSwitchAction::kSwitchQland;
    }
    return LoiterAltQlandSwitchAction::kNone;
}

}  // namespace fwcpp::q_loiter
