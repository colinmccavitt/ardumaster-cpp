#pragma once

// Copter::init_ardupilot leftover. Upstream ArduCopter/system.cpp
// ~16-35 only (stop before gcs().setup_uarts). No notify / battery /
// barometer / winch / rssi objects — record leftover flags only.
//
// Always-on this slice (AP_WINCH_ENABLED / AP_RSSI_ENABLED are not
// enabled in this port):
//   notify.init()
//   notify_flight_mode()
//   battery.init()
//   barometer.init()
// winch_init and rssi_init stay false (remaining / not enabled).
//
// The rest of init_ardupilot (GCS uarts, OSD, interlock, RC in/out,
// allocate_motors call, GPS/compass, startup_INS_ground call, etc.)
// is catalog row "Copter::init_ardupilot rest".

namespace fwcpp::copter {

struct InitArdupilotEffects {
    bool winch_init{false};           // remaining AP_WINCH_ENABLED
    bool notify_init{false};
    bool notify_flight_mode{false};
    bool battery_init{false};
    bool rssi_init{false};            // remaining AP_RSSI_ENABLED
    bool barometer_init{false};
};

[[nodiscard]] inline InitArdupilotEffects init_ardupilot() {
    InitArdupilotEffects fx{};
    fx.notify_init = true;
    fx.notify_flight_mode = true;
    fx.battery_init = true;
    fx.barometer_init = true;
    return fx;
}

}  // namespace fwcpp::copter
