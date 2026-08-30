#pragma once

// Copter::lost_vehicle_check leftover. Upstream ArduCopter/motors.cpp
// ~130-155 (LOST_VEHICLE_DELAY at ~5). Called at 10 Hz so delay 10 is
// 1 second. Inject aux assignment (find_channel_for_option
// LOST_VEHICLE_SOUND), throttle_zero, armed, roll/pitch control_in,
// soundalarm_counter, and vehicle_lost. No Notify / GCS objects —
// gcs_locate_alarm is true only on the false→true vehicle_lost edge
// (upstream send_text "Locate Copter alarm").
//
// Do not port Copter::takeoff_check.

#include <cstdint>

namespace fwcpp::copter {

inline constexpr std::uint8_t kLostVehicleDelay = 10;
inline constexpr std::int16_t kLostVehicleStickThreshold = 4000;

struct LostVehicleCheckInputs {
    bool aux_lost_vehicle_sound{false};
    bool throttle_zero{false};
    bool armed{false};
    std::int16_t roll_control_in{0};
    std::int16_t pitch_control_in{0};
    std::uint8_t soundalarm_counter{0};
    bool vehicle_lost{false};
};

struct LostVehicleCheckEffects {
    std::uint8_t soundalarm_counter{0};
    bool vehicle_lost{false};
    bool gcs_locate_alarm{false};
};

[[nodiscard]] inline LostVehicleCheckEffects lost_vehicle_check(
    const LostVehicleCheckInputs& in = {}) {
    LostVehicleCheckEffects fx{};
    fx.soundalarm_counter = in.soundalarm_counter;
    fx.vehicle_lost = in.vehicle_lost;

    // Aux switch already owns the alarm; the two would interfere.
    if (in.aux_lost_vehicle_sound) {
        return fx;
    }

    if (in.throttle_zero && !in.armed &&
        (in.roll_control_in > kLostVehicleStickThreshold) &&
        (in.pitch_control_in > kLostVehicleStickThreshold)) {
        if (fx.soundalarm_counter >= kLostVehicleDelay) {
            if (!fx.vehicle_lost) {
                fx.vehicle_lost = true;
                fx.gcs_locate_alarm = true;
            }
        } else {
            ++fx.soundalarm_counter;
        }
    } else {
        fx.soundalarm_counter = 0;
        if (fx.vehicle_lost) {
            fx.vehicle_lost = false;
        }
    }

    return fx;
}

}  // namespace fwcpp::copter
