#pragma once

// CCP-042 failsafe leftover — ArduCopter radio failsafe (events.cpp
// failsafe_radio_on_event FS_THR_ENABLE → FailsafeAction; AP_State.cpp
// set_failsafe_radio edge). failsafe.cpp CPU watchdog (failsafe_enable /
// failsafe_check) stays catalog remaining.
//
// ADR-0012: no GCS / Notify / logger objects — announce_failsafe and
// mode-change notify are bool flags. Inject motors_armed + radio_failsafe.
// Override ladder (continue-landing/auto/guided), do_failsafe_action body,
// GCS failsafe, crash_check remain (see failsafe_leftover.hpp).
// Do NOT copy Rust COP-019.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter {

// FS_THR_ENABLE (defines.h) — g.failsafe_throttle switch cases.
enum class FsThrEnable : std::uint8_t {
    Disabled = 0,             // FS_THR_DISABLED
    AlwaysRtl = 1,            // FS_THR_ENABLED_ALWAYS_RTL
    ContinueMission = 2,      // FS_THR_ENABLED_CONTINUE_MISSION (legacy → RTL)
    AlwaysLand = 3,           // FS_THR_ENABLED_ALWAYS_LAND
    AlwaysSmartRtlOrRtl = 4,  // FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL
    AlwaysSmartRtlOrLand = 5, // FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND
    AutoRtlOrRtl = 6,         // FS_THR_ENABLED_AUTO_RTL_OR_RTL
    BrakeOrLand = 7,          // FS_THR_ENABLED_BRAKE_OR_LAND
};

// Copter::FailsafeAction (Copter.h) — numeric values match upstream.
enum class FailsafeAction : std::uint8_t {
    None = 0,
    Land = 1,
    Rtl = 2,
    SmartRtl = 3,           // SMARTRTL
    SmartRtlLand = 4,       // SMARTRTL_LAND
    Terminate = 5,
    AutoDoLandStart = 6,    // AUTO_DO_LAND_START
    BrakeLand = 7,          // BRAKE_LAND
};

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
    // Stand-in for do_failsafe_action call site (events.cpp ~78). Body remains.
    bool leftover_do_failsafe_action{false};
};

// Thin leftover: armed && radio failsafe inject → RTL-or-land flags.
// Full override ladder + do_failsafe_action body remain in the catalog.
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

// Leftover failsafe_radio_on_event FS_THR_ENABLE switch (events.cpp ~17-44).
// Maps FsThrEnable → FailsafeAction and sets leftover_do_failsafe_action.
// Does NOT run do_failsafe_action body or the override ladder (~46-75).
[[nodiscard]] inline FailsafeAction leftover_failsafe_radio_on_event(
    FsThrEnable thr, FailsafeEffects& fx) {
    FailsafeAction desired_action;
    switch (thr) {
        case FsThrEnable::Disabled:
            desired_action = FailsafeAction::None;
            break;
        case FsThrEnable::AlwaysRtl:
        case FsThrEnable::ContinueMission:
            desired_action = FailsafeAction::Rtl;
            break;
        case FsThrEnable::AlwaysSmartRtlOrRtl:
            desired_action = FailsafeAction::SmartRtl;
            break;
        case FsThrEnable::AlwaysSmartRtlOrLand:
            desired_action = FailsafeAction::SmartRtlLand;
            break;
        case FsThrEnable::AlwaysLand:
            desired_action = FailsafeAction::Land;
            break;
        case FsThrEnable::AutoRtlOrRtl:
            desired_action = FailsafeAction::AutoDoLandStart;
            break;
        case FsThrEnable::BrakeOrLand:
            desired_action = FailsafeAction::BrakeLand;
            break;
        default:
            desired_action = FailsafeAction::Land;
            break;
    }
    fx.leftover_do_failsafe_action = true;
    return desired_action;
}

}  // namespace fwcpp::copter
