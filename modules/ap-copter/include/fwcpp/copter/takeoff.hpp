#pragma once

// CCP-041 takeoff helper leftover — ArduCopter/takeoff.cpp (Plane-4.7.0).
// Thin leftover for Mode::do_user_takeoff_U_m (~18-47) gate chain and
// Mode::_TakeOff::start_m (~51-57). No motors / flightmode / current_loc /
// pos_control objects (ADR-0012): inject armed, land_complete,
// has_user_takeoff, altitudes, interlock, pos_estimate_U_m.
//
// do_user_takeoff_start_m → leftover_takeoff_start_m (body this slice).
// set_auto_armed is an effect flag. do_pilot_takeoff_ms body remains.
//
// Catalog rows live in land_detector.hpp (CCP-041 shared catalog); this
// header is the leftover body for takeoff helpers / start_m.

namespace fwcpp::copter {

// Injected inputs for do_user_takeoff_U_m gates (takeoff.cpp ~18-40).
// takeoff_alt_m / current_alt_m are both meters (upstream compares
// takeoff_alt_m to current_loc.alt * 0.01).
struct UserTakeoffInputs {
    bool motors_armed{false};
    bool land_complete{false};
    bool has_user_takeoff{false};
    float takeoff_alt_m{0.0f};
    float current_alt_m{0.0f};
    bool interlock{false};
    bool using_interlock{false};
};

struct UserTakeoffEffects {
    bool leftover_takeoff_start_m{false};
    bool set_auto_armed{false};
};

// Mode::_TakeOff state fields used by start_m / do_pilot_takeoff_ms.
struct TakeOffState {
    bool _running{false};
    float start_alt{0.0f};
    float complete_alt{0.0f};
};

// Leftover Mode::_TakeOff::start_m (takeoff.cpp ~51-57).
// _running = true; start_alt = pos_estimate_U; complete_alt = start + alt_m.
inline void leftover_takeoff_start_m(TakeOffState& state, float alt_m,
                                     float pos_estimate_U_m) {
    state._running = true;
    state.start_alt = pos_estimate_U_m;
    state.complete_alt = pos_estimate_U_m + alt_m;
}

// Leftover Mode::do_user_takeoff_U_m (takeoff.cpp ~18-47). Gate chain then
// flag start_m + set_auto_armed. When state != nullptr, also runs
// leftover_takeoff_start_m(state, takeoff_alt_m, pos_estimate_U_m).
// Returns false on any gate failure.
[[nodiscard]] inline bool leftover_do_user_takeoff_U_m(
    const UserTakeoffInputs& in, UserTakeoffEffects& fx,
    TakeOffState* state = nullptr, float pos_estimate_U_m = 0.0f) {
    fx.leftover_takeoff_start_m = false;
    fx.set_auto_armed = false;

    if (!in.motors_armed) {
        return false;
    }
    if (!in.land_complete) {
        // can't takeoff again!
        return false;
    }
    if (!in.has_user_takeoff) {
        // this mode doesn't support user takeoff
        return false;
    }
    if (in.takeoff_alt_m <= in.current_alt_m) {
        // can't takeoff downwards...
        return false;
    }
    // Vehicles using motor interlock should return false if interlock disabled.
    if (!in.interlock && in.using_interlock) {
        return false;
    }

    // do_user_takeoff_start_m → leftover_takeoff_start_m body.
    fx.leftover_takeoff_start_m = true;
    if (state != nullptr) {
        leftover_takeoff_start_m(*state, in.takeoff_alt_m, pos_estimate_U_m);
    }
    fx.set_auto_armed = true;
    return true;
}

}  // namespace fwcpp::copter
