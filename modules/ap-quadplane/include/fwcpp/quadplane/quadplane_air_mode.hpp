#pragma once

#include <cstdint>

namespace fwcpp::quadplane {

/// Upstream `AirMode` (`ArduPlane/defines.h`).
enum class AirMode : std::uint8_t {
    kOff = 0,
    kOn = 1,
    kAssistedFlightOnly = 2,
};

enum class AirModeAuxPos : std::uint8_t { kLow = 0, kMiddle = 1, kHigh = 2 };

[[nodiscard]] inline constexpr bool air_mode_active(AirMode mode, bool assisted_flight) {
    return mode == AirMode::kOn || (mode == AirMode::kAssistedFlightOnly && assisted_flight);
}

/// RC AUX_FUNC::AIRMODE latch (`RC_Channel_Plane.cpp`).
inline void apply_air_mode_aux(AirModeAuxPos pos, AirMode& latched, bool& throttle_wait) {
    switch (pos) {
        case AirModeAuxPos::kHigh:
            latched = AirMode::kOn;
            throttle_wait = false;
            break;
        case AirModeAuxPos::kMiddle:
            break;
        case AirModeAuxPos::kLow:
            latched = AirMode::kOff;
            break;
    }
}

/// RC AUX_FUNC::ARMDISARM_AIRMODE when armed (`RC_Channel_Plane.cpp`).
inline void apply_armdisarm_airmode_latch(bool armed, AirMode& latched, bool& throttle_wait) {
    if (armed) {
        latched = AirMode::kOn;
        throttle_wait = false;
    }
}

}  // namespace fwcpp::quadplane
