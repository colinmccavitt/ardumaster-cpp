#pragma once

// Copter::auto_disarm_check leftover. Upstream ArduCopter/motors.cpp
// ~11-58. No motors / arming / SRV / flightmode objects — inject
// tnow, armed, delay, spool, interlock, throttle, land, and the
// auto_disarm_begin timer. THROW is injected as mode_is_throw
// (Mode::Number::THROW == 18); do not include mode.hpp.
//
// disarm_delay_s is constrained 0..127 (INT8_MAX) then * 1000 ms.
// This port is not heli: interlock-off / e-stop always halves delay.
// Records arming.disarm(DISARMDELAY) as a leftover bool only.
//
// Do not port Copter::standby_update or lost_vehicle_check.

#include <cstdint>

#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::copter {

struct AutoDisarmCheckInputs {
    std::uint32_t tnow_ms{0};
    bool armed{false};
    std::int16_t disarm_delay_s{0};
    bool mode_is_throw{false};
    DesiredSpoolState desired_spool{DesiredSpoolState::SHUT_DOWN};
    SpoolState spool_state{SpoolState::SHUT_DOWN};
    bool using_interlock{false};
    bool interlock{false};
    bool emergency_stop{false};
    bool has_manual_throttle{false};
    bool sprung_throttle_stick{false};
    bool throttle_zero{false};
    std::int16_t throttle_control_in{0};
    std::int16_t throttle_mid{0};
    std::int16_t throttle_deadzone{0};
    bool land_complete{false};
    std::uint32_t auto_disarm_begin{0};
};

struct AutoDisarmCheckEffects {
    std::uint32_t auto_disarm_begin{0};
    bool disarm{false};
};

[[nodiscard]] inline AutoDisarmCheckEffects auto_disarm_check(
    const AutoDisarmCheckInputs& in = {}) {
    AutoDisarmCheckEffects fx{};
    fx.auto_disarm_begin = in.auto_disarm_begin;

    const std::int16_t delay_s = fwcpp::math::constrain_value(
        in.disarm_delay_s, std::int16_t{0}, std::int16_t{127});
    std::uint32_t disarm_delay_ms = 1000u * static_cast<std::uint32_t>(delay_s);

    // Reset timer and exit if disarmed, auto-disarm disabled, or THROW.
    if (!in.armed || disarm_delay_ms == 0 || in.mode_is_throw) {
        fx.auto_disarm_begin = in.tnow_ms;
        return fx;
    }

    // Inhibit auto-disarm while takeoff / spool-up is requested.
    if (in.desired_spool > DesiredSpoolState::GROUND_IDLE ||
        in.spool_state > SpoolState::GROUND_IDLE) {
        fx.auto_disarm_begin = in.tnow_ms;
        return fx;
    }

    // Interlock disengaged or e-stop: motors may be stopped, so arming
    // is less obvious — shorten delay. Not heli: always divide.
    if ((in.using_interlock && !in.interlock) || in.emergency_stop) {
        disarm_delay_ms /= 2;
    } else {
        bool thr_low = false;
        if (in.has_manual_throttle || !in.sprung_throttle_stick) {
            thr_low = in.throttle_zero;
        } else {
            const float deadband_top =
                static_cast<float>(in.throttle_mid) +
                static_cast<float>(in.throttle_deadzone);
            thr_low = static_cast<float>(in.throttle_control_in) <= deadband_top;
        }

        if (!thr_low || !in.land_complete) {
            fx.auto_disarm_begin = in.tnow_ms;
        }
    }

    if ((in.tnow_ms - fx.auto_disarm_begin) >= disarm_delay_ms) {
        fx.disarm = true;
        fx.auto_disarm_begin = in.tnow_ms;
    }

    return fx;
}

}  // namespace fwcpp::copter
