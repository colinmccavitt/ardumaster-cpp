#pragma once

// Copter::init_ardupilot leftover. Upstream ArduCopter/system.cpp
// ~16-55 (after barometer.init(); stop BEFORE surface_tracking /
// allocate_motors()). No notify / battery / barometer / winch /
// rssi / GCS / OSD / RC_Channel objects — record leftover flags only.
//
// Always-on this slice (AP_WINCH_ENABLED / AP_RSSI_ENABLED are not
// enabled in this port):
//   notify.init()
//   notify_flight_mode()
//   battery.init()
//   barometer.init()
// winch_init and rssi_init stay false (remaining / not enabled).
//
// GCS/OSD remaining this slice (do not invent objects):
//   gcs().setup_uarts()  — leftover gcs_setup_uarts stays false
//   osd.init()           — leftover osd_init stays false
//
// This slice leftover (non-heli, FRAME_CONFIG != HELI_FRAME):
//   update_using_interlock() — using_interlock = motor_interlock_aux
//     (rc().find_channel_for_option(MOTOR_INTERLOCK) != nullptr)
//   init_rc_in() — bind + set_angle/set_range leftover flags
//
// The rest of init_ardupilot (surface_tracking, allocate_motors call,
// rc convert/init, init_rc_out, ESC cal, GPS/compass,
// startup_INS_ground call, etc.) is catalog row
// "Copter::init_ardupilot rest".

#include <cstdint>

namespace fwcpp::copter {

// ROLL_PITCH_YAW_INPUT_MAX — ArduCopter/config.h ~470-471
inline constexpr std::int16_t kRollPitchYawInputMax = 4500;

struct InitArdupilotInputs {
    bool motor_interlock_aux{false};
};

struct InitArdupilotEffects {
    bool winch_init{false};           // remaining AP_WINCH_ENABLED
    bool notify_init{false};
    bool notify_flight_mode{false};
    bool battery_init{false};
    bool rssi_init{false};            // remaining AP_RSSI_ENABLED
    bool barometer_init{false};
    bool gcs_setup_uarts{false};      // remaining — do not invent GCS
    bool osd_init{false};             // remaining OSD_ENABLED
    bool using_interlock{false};
    bool roll_bind{false};
    bool pitch_bind{false};
    bool throttle_bind{false};
    bool yaw_bind{false};
    std::int16_t roll_angle{0};
    std::int16_t pitch_angle{0};
    std::int16_t yaw_angle{0};
    std::int16_t throttle_range{0};
    bool rc_tuning{false};            // remaining AP_RC_TRANSMITTER_TUNING
    bool rc_tuning2{false};
    bool default_dead_zones{false};
    bool throttle_zero{false};
};

[[nodiscard]] inline InitArdupilotEffects init_ardupilot(
    const InitArdupilotInputs& in = {}) {
    InitArdupilotEffects fx{};
    fx.notify_init = true;
    fx.notify_flight_mode = true;
    fx.battery_init = true;
    fx.barometer_init = true;
    // gcs_setup_uarts / osd_init stay false (remaining)
    fx.using_interlock = in.motor_interlock_aux;
    // library guarantees non-nullptr
    fx.roll_bind = true;
    fx.pitch_bind = true;
    fx.throttle_bind = true;
    fx.yaw_bind = true;
    fx.roll_angle = kRollPitchYawInputMax;
    fx.pitch_angle = kRollPitchYawInputMax;
    fx.yaw_angle = kRollPitchYawInputMax;
    fx.throttle_range = 1000;
    // rc_tuning / rc_tuning2 stay false
    fx.default_dead_zones = true;
    fx.throttle_zero = true;
    return fx;
}

}  // namespace fwcpp::copter
