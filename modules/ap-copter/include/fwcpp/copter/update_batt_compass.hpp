#pragma once

// Copter::update_batt_compass leftover. Upstream ArduCopter/Copter.cpp
// ~619-632. No AP::battery / AP::compass / motors objects — inject
// compass_available, throttle, and voltage.
//
// Always battery.read() first (compassmot uses battery). If
// compass_available: compass.set_throttle, set_voltage, compass.read().
// Else skip the compass path. loop_rate_logging / battery object /
// CompassMot compensation math stay later leftovers.

namespace fwcpp::copter {

struct UpdateBattCompassInputs {
    bool compass_available{false};
    float throttle{0};
    float voltage{0};
};

struct UpdateBattCompassEffects {
    bool battery_read{false};
    bool set_throttle{false};
    float throttle{0};
    bool set_voltage{false};
    float voltage{0};
    bool compass_read{false};
};

[[nodiscard]] inline UpdateBattCompassEffects update_batt_compass(
    const UpdateBattCompassInputs& in) {
    UpdateBattCompassEffects fx{};
    fx.battery_read = true;

    if (in.compass_available) {
        fx.set_throttle = true;
        fx.throttle = in.throttle;
        fx.set_voltage = true;
        fx.voltage = in.voltage;
        fx.compass_read = true;
    }
    return fx;
}

}  // namespace fwcpp::copter
