#pragma once

#include <cstdint>

namespace fwcpp::q_modes {

/// Shared first branch in ModeQ*::run when tailsitter is in FW pull-up of VTOL transition.
[[nodiscard]] inline constexpr bool run_delegates_to_fw_controllers(bool tailsitter_in_vtol_transition) {
    return tailsitter_in_vtol_transition;
}

struct QRunTiming {
    std::uint32_t now_ms{0};
};

}  // namespace fwcpp::q_modes
