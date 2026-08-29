#pragma once

// Copter::update_home_from_EKF leftover. Upstream ArduCopter/commands.cpp
// ~4-78. No AHRS / motors / SmartRTL objects — inject home_is_set, armed,
// get_location ok + loc, get_origin ok + origin, set_home success.
// Optionally mutates a caller-owned Location& home.
//
// MODE_SMARTRTL_ENABLED g2.smart_rtl.set_home is leftover; this tick
// does not call it. GPS vs EKF is only what commands.cpp does:
// inflight uses get_location (horizontal) + copy_alt_from(origin)
// (vertical); disarmed uses get_location then set_home.

#include <fwcpp/location.hpp>

namespace fwcpp::copter {

struct UpdateHomeFromEkfInputs {
    bool home_is_set{false};
    bool armed{false};
    bool get_location_ok{false};
    Location location{};
    bool get_origin_ok{false};
    Location origin{};
    bool set_home_success{true};
};

struct UpdateHomeFromEkfEffects {
    bool home_is_set{false};
    bool set_home_called{false};
    bool set_home_ok{false};
    bool lock_home{false};
    bool copy_alt_from_origin{false};
    bool inflight{false};
    bool smart_rtl_set_home{false};  // MODE_SMARTRTL_ENABLED leftover
};

// Copter::set_home leftover. Requires origin; ahrs.set_home is writing
// home + home_is_set. lock_home only if lock. Both callers this slice
// pass lock=false.
[[nodiscard]] inline bool set_home(const Location& loc, bool lock,
                                   const UpdateHomeFromEkfInputs& in,
                                   Location& home,
                                   UpdateHomeFromEkfEffects& fx) {
    fx.set_home_called = true;
    if (!in.get_origin_ok) {
        return false;
    }
    if (!in.set_home_success) {
        return false;
    }
    home = loc;
    fx.home_is_set = true;
    fx.set_home_ok = true;
    if (lock) {
        fx.lock_home = true;
    }
    return true;
}

[[nodiscard]] inline UpdateHomeFromEkfEffects update_home_from_ekf(
    const UpdateHomeFromEkfInputs& in, Location& home) {
    UpdateHomeFromEkfEffects fx{};
    fx.home_is_set = in.home_is_set;

    // exit immediately if home already set
    if (in.home_is_set) {
        return fx;
    }

    if (in.armed) {
        // set_home_to_current_location_inflight
        fx.inflight = true;
        if (in.get_location_ok && in.get_origin_ok) {
            Location temp_loc = in.location;
            temp_loc.copy_alt_from(in.origin);
            fx.copy_alt_from_origin = true;
            if (!set_home(temp_loc, false, in, home, fx)) {
                return fx;
            }
            // SmartRTL leftover
        }
        return fx;
    }

    // set_home_to_current_location(false) — ignore failure
    if (in.get_location_ok) {
        if (!set_home(in.location, false, in, home, fx)) {
            return fx;
        }
        // SmartRTL leftover
    }
    return fx;
}

}  // namespace fwcpp::copter
