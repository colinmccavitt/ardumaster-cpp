#pragma once

// Copter::standby_update leftover. Upstream ArduCopter/standby.cpp
// ~14-23. If !standby_active return; else record
// attitude_control->reset_rate_controller_I_terms(),
// reset_yaw_target_and_rate(), and pos_control->NED_standby_reset()
// as leftover flags. Inject standby_active (default false).
//
// The comment above the function lists other standby effects
// (crash_check / thrust_loss / parachute disabled, hover learn off,
// landing detection off). Those are other remaining rows, not this
// function body.
//
// Do not port Copter::lost_vehicle_check.

namespace fwcpp::copter {

struct StandbyUpdateInputs {
    bool standby_active{false};
};

struct StandbyUpdateEffects {
    bool reset_rate_I{false};
    bool reset_yaw_target_and_rate{false};
    bool ned_standby_reset{false};
};

[[nodiscard]] inline StandbyUpdateEffects standby_update(
    const StandbyUpdateInputs& in = {}) {
    StandbyUpdateEffects fx{};
    if (!in.standby_active) {
        return fx;
    }

    fx.reset_rate_I = true;
    fx.reset_yaw_target_and_rate = true;
    fx.ned_standby_reset = true;
    return fx;
}

}  // namespace fwcpp::copter
