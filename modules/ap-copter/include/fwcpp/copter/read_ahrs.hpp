#pragma once

// Copter::read_AHRS leftover. Upstream Copter.cpp: ahrs.update(true).
// INS already ran as the first FAST_TASK; skip_ins_update is the whole
// function. No AHRS object — the flag is the leftover.

namespace fwcpp::copter {

struct ReadAhrsEffects {
    bool skip_ins_update{true};
};

[[nodiscard]] inline constexpr ReadAhrsEffects read_ahrs() {
    return ReadAhrsEffects{.skip_ins_update = true};
}

}  // namespace fwcpp::copter
