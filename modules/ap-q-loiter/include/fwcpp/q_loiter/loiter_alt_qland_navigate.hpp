#pragma once

#include <fwcpp/q_loiter/loiter_alt_qland.hpp>

namespace fwcpp::q_loiter {

struct LoiterAltQlandNavigateEffects {
    LoiterAltQlandSwitchAction switch_action{LoiterAltQlandSwitchAction::kNone};
    bool request_qland_mode{false};
    bool delegate_mode_loiter_navigate{true};
};

/// Port of ModeLoiterAltQLand::navigate — `switch_qland()` then `ModeLoiter::navigate()`.
[[nodiscard]] inline LoiterAltQlandNavigateEffects loiter_alt_qland_navigate(
    const LoiterAltQlandSwitchInputs& switch_in) {
    LoiterAltQlandNavigateEffects out{};
    out.switch_action = loiter_alt_qland_switch(switch_in);
    if (out.switch_action == LoiterAltQlandSwitchAction::kSwitchQland) {
        out.request_qland_mode = true;
    }
    return out;
}

}  // namespace fwcpp::q_loiter
