#pragma once

// CCP-043: thin leftover Copter vehicle shell for SitlCopterHarness.
// Upstream counterpart is ArduCopter/Copter.h state the SITL HAL feeds;
// this port has no full Copter object yet (CCP-035 leftovers are free
// functions + Mode*). ADR-0012: no AP:: singletons — sensor samples are
// buffers/flags the harness writes, then leftover_copter_tick() consumes.
//
// Slice 3: closed-loop arm/spool/hold smoke flags on this leftover
// (motors_armed inject + spool + attitude-hold). Multirotor aero /
// motor→SimPlane feedback is kOutOfScope (SimPlane rigid body only;
// CCP-044 may deepen). See sitl_copter_harness.hpp completeness catalog.

#include <cstdint>

#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/copter/update_flight_mode.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::copter {

// Minimal leftover vehicle: sensor buffers the SITL harness injects into,
// plus the Mode* CCP-035 update_flight_mode already understands.
struct LeftoverCopter {
    math::Vector3f gyro_buffer{};
    math::Vector3f accel_buffer{};
    float baro_altitude_m{0.0f};
    // GPS lat/lng in Location 1e7-degree units (home + SimPlane NED offset).
    std::int32_t gps_lat{0};
    std::int32_t gps_lng{0};
    math::Vector3f compass_field_bf{};

    bool gyro_injected{false};
    bool accel_injected{false};
    bool baro_injected{false};
    bool gps_injected{false};
    bool compass_injected{false};

    // Closed-loop arm/spool/hold smoke (no multirotor aero). Caller may
    // set motors_armed before SitlCopterHarness::step(); step injects
    // spool + attitude_hold from that arm state.
    bool motors_armed{false};
    bool motors_armed_injected{false};
    SpoolState spool_state{SpoolState::SHUT_DOWN};
    bool spool_injected{false};
    bool attitude_hold{false};
    bool attitude_hold_injected{false};

    // SITL default home (CMAC) for lat/lng synthesis from NED position —
    // same coords location / update_home_from_ekf tests use.
    std::int32_t home_lat{-353632621};
    std::int32_t home_lng{1491652374};

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
