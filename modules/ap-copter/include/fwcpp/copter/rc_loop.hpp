#pragma once

// Copter::rc_loop leftover. Upstream always calls read_radio() then
// rc().read_mode_switch(). No RC objects — inject has_valid_input and
// an optional flight-mode channel index (nullptr-equivalent = missing).
//
// A late / missing radio frame does not skip the mode switch; that
// refuse lives inside read_mode_switch. Folding the skip up into
// rc_loop would drop a valid switch edge on the same tick the receiver
// recovered.

#include <cstdint>
#include <optional>

namespace fwcpp::copter {

struct ModeSwitchReadInputs {
    bool has_valid_input{false};
    std::optional<std::uint8_t> flight_mode_channel{};
};

enum class ModeSwitchReadLeftover : std::uint8_t {
    kNoValidInput = 0,
    kNoChannel = 1,
    kRead = 2,
};

struct RcLoopEffects {
    bool read_radio{true};
    ModeSwitchReadLeftover mode_switch{ModeSwitchReadLeftover::kNoValidInput};
};

[[nodiscard]] inline constexpr ModeSwitchReadLeftover read_mode_switch(
    const ModeSwitchReadInputs& inputs) {
    if (!inputs.has_valid_input) {
        return ModeSwitchReadLeftover::kNoValidInput;
    }
    if (!inputs.flight_mode_channel.has_value()) {
        return ModeSwitchReadLeftover::kNoChannel;
    }
    return ModeSwitchReadLeftover::kRead;
}

[[nodiscard]] inline constexpr RcLoopEffects rc_loop(const ModeSwitchReadInputs& inputs) {
    return RcLoopEffects{
        .read_radio = true,
        .mode_switch = read_mode_switch(inputs),
    };
}

}  // namespace fwcpp::copter
