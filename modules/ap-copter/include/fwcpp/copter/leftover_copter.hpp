#pragma once

// CCP-043: thin leftover Copter vehicle shell for SitlCopterHarness.
// Upstream counterpart is ArduCopter/Copter.h state the SITL HAL feeds;
// this port has no full Copter object yet (CCP-035 leftovers are free
// functions + Mode*). ADR-0012: no AP:: singletons — sensor samples are
// buffers/flags the harness writes, then leftover_copter_tick() consumes.
//
// Full closed-loop arm/spool/attitude-hold and baro/GPS/compass synthesis
// remain (see sitl_copter_harness.hpp completeness catalog).

#include <cstdint>

#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/update_flight_mode.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::copter {

// Minimal leftover vehicle: sensor buffers the SITL harness injects into,
// plus the Mode* CCP-035 update_flight_mode already understands.
struct LeftoverCopter {
    math::Vector3f gyro_buffer{};
    math::Vector3f accel_buffer{};
    // Prefer flags this slice — values may be zero until synthesis fills them.
    bool gyro_injected{false};
    bool accel_injected{false};
    bool baro_injected{false};   // remaining: baro synthesis
    bool gps_injected{false};    // remaining: GPS synthesis
    bool compass_injected{false}; // remaining: compass synthesis

    std::uint32_t tick_count{0};
    Mode* current{nullptr};
    bool land_complete{false};
    bool move_vehicle_on_ekf_reset{false};
};

// Thin leftover stand-in for Copter::fast_loop / scheduler tick: bump the
// harness-visible counter and wire CCP-035 update_flight_mode when a Mode*
// is present. No motors / AHRS / INS objects this slice.
inline void leftover_copter_tick(LeftoverCopter& copter) {
    ++copter.tick_count;
    UpdateFlightModeInputs in;
    in.land_complete = copter.land_complete;
    in.move_vehicle_on_ekf_reset = copter.move_vehicle_on_ekf_reset;
    in.current = copter.current;
    (void)update_flight_mode(in);
}

}  // namespace fwcpp::copter
