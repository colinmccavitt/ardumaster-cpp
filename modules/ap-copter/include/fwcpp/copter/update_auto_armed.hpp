#pragma once

// Copter::update_auto_armed leftover. Upstream ArduCopter/system.cpp
// ~317-348 and set_auto_armed in AP_State.cpp ~4-14. No motors /
// Mode* / RC objects — inject auto_armed, motors_armed, throttle
// flags, interlock, spool, and THROW as mode_is_throw
// (Mode::Number::THROW == 18); do not include mode.hpp.
//
// Returns the resulting ap.auto_armed bool. set_auto_armed's
// logger-on-rising-true is not a leftover this slice.
//
// SpoolState::THROTTLE_UNLIMITED is 3 (actual spool). Do not mix
// with DesiredSpoolState::THROTTLE_UNLIMITED (2).
//
// Do not port Copter::startup_INS_ground, allocate_motors, or
// init_ardupilot.

#include <fwcpp/copter/mode_stabilize.hpp>

namespace fwcpp::copter {

struct UpdateAutoArmedInputs {
    bool auto_armed{false};
    bool motors_armed{false};
    bool has_manual_throttle{false};
    bool throttle_zero{false};
    bool has_valid_input{false};
    bool using_interlock{false};
    SpoolState spool_state{SpoolState::SHUT_DOWN};
    bool mode_is_throw{false};
};

[[nodiscard]] inline bool update_auto_armed(const UpdateAutoArmedInputs& in = {}) {
    if (in.auto_armed) {
        if (!in.motors_armed) {
            return false;
        }
        if (in.has_manual_throttle && in.throttle_zero && in.has_valid_input) {
            return false;
        }
        return true;
    }

    if (in.motors_armed && in.using_interlock) {
        if (!in.throttle_zero &&
            in.spool_state == SpoolState::THROTTLE_UNLIMITED) {
            return true;
        }
    } else if (in.motors_armed && !in.using_interlock) {
        if (!in.throttle_zero || in.mode_is_throw) {
            return true;
        }
    }
    return false;
}

}  // namespace fwcpp::copter
