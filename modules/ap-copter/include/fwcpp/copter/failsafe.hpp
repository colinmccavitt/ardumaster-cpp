#pragma once

// CCP-042 failsafe leftover scaffold — ArduCopter radio failsafe thin gate
// (events.cpp failsafe_radio_on_event → set_mode_RTL_or_land_with_pause;
//  AP_State.cpp set_failsafe_radio edge). failsafe.cpp CPU watchdog
//  (failsafe_enable / failsafe_check) stays catalog remaining.
//
// ADR-0012: no GCS / Notify / logger objects — announce_failsafe and
// mode-change notify are bool flags. Inject motors_armed + radio_failsafe.
// Full FS_THR_ENABLE override ladder, GCS failsafe, crash_check remain
// (see failsafe_leftover.hpp). Do NOT copy Rust COP-019.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter {

// Injected inputs for radio-failsafe thin gate. radio_failsafe stands in for
// failsafe.radio already latched (or the edge inject from set_failsafe_radio).
struct FailsafeInputs {
    bool motors_armed{false};
    bool radio_failsafe{false};
};

struct FailsafeEffects {
    // Thin stand-in for set_mode_RTL_or_land_with_pause (events.cpp ~389).
    bool leftover_set_mode_rtl_or_land{false};
    // ADR-0012: announce_failsafe("Radio") / GCS send_text as flags.
    bool gcs_announce_radio_failsafe{false};
    bool notify_failsafe_mode_change{false};
    bool radio_failsafe_acted{false};
};

// Thin leftover: armed && radio failsafe inject → RTL-or-land flags.
// Full failsafe_radio_on_event desired_action table + do_failsafe_action
// remain in the catalog.
inline void leftover_failsafe_radio_check(const FailsafeInputs& in,
                                          FailsafeEffects& fx) {
    if (!(in.motors_armed && in.radio_failsafe)) {
        return;
    }
    fx.radio_failsafe_acted = true;
    fx.leftover_set_mode_rtl_or_land = true;
    fx.gcs_announce_radio_failsafe = true;
    fx.notify_failsafe_mode_change = true;
}

// Thin leftover for Copter::set_mode_RTL_or_land_with_pause — flag only.
// No Mode::set_mode / ModeLand pause body this slice.
inline void leftover_set_mode_rtl_or_land(FailsafeEffects& fx) {
    fx.leftover_set_mode_rtl_or_land = true;
    fx.notify_failsafe_mode_change = true;
}

}  // namespace fwcpp::copter
