#pragma once

// CCP-043: SitlCopterHarness — Copter analogue of SitlHarness (CPP-084).
// Upstream ROLE: AP_HAL_SITL SITL_State sensor synthesis for ArduCopter.
// Not a port of AP_HAL_SITL source (ADR-0012: no HAL singleton/scheduler
// tangle). Mirrors SitlHarness's structure: hold refs to vehicle + SimPlane,
// step() synthesizes sensors from sim truth, then advances the vehicle tick.
//
// SLICE 2: gyro/accel + baro altitude + GPS lat/lng + compass body field
// from SimPlane truth into LeftoverCopter buffers/flags, then
// leftover_copter_tick(). Closed-loop arm/spool/hold and multirotor aero
// remain. Do NOT copy Rust.

#include <cstddef>
#include <cstdint>

#include <fwcpp/compass/compass.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/sim/sim_plane.hpp>

namespace fwcpp::hal_sitl {

class SitlCopterHarness {
public:
    // Does not own LeftoverCopter / SimPlane — caller constructs and configures
    // both (arm flags, Mode*, etc.) before calling step(), matching SitlHarness.
    SitlCopterHarness(copter::LeftoverCopter& copter, sim::SimPlane& sim_plane)
        : copter_(copter), sim_plane_(sim_plane) {}

    // Synthesize gyro/accel/baro/GPS/compass from sim_plane_ into leftover
    // sensor buffers, set inject flags, then leftover_copter_tick(). dt is
    // reserved for future closed-loop SimPlane feedback (arm/spool/hold).
    void step(float dt) {
        (void)dt;
        copter_.gyro_buffer = sim_plane_.gyro;
        copter_.accel_buffer = sim_plane_.accel_body;
        copter_.gyro_injected = true;
        copter_.accel_injected = true;

        // Baro — SitlHarness feeds current_altitude_m = -position.z.
        copter_.baro_altitude_m = -sim_plane_.position.z;
        copter_.baro_injected = true;

        // GPS lat/lng — leftover home offset by SimPlane NED north/east.
        Location gps_loc(copter_.home_lat, copter_.home_lng, 0, Location::AltFrame::ABSOLUTE);
        gps_loc.offset(sim_plane_.position.x, sim_plane_.position.y);
        copter_.gps_lat = gps_loc.lat;
        copter_.gps_lng = gps_loc.lng;
        copter_.gps_injected = true;

        // Compass — earth field rotated into body (SitlHarness /
        // Compass::rotate_earth_field_to_body; upstream update_mag_field_bf).
        copter_.compass_field_bf = compass_.rotate_earth_field_to_body(sim_plane_.dcm);
        copter_.compass_injected = true;

        copter::leftover_copter_tick(copter_);
    }

    [[nodiscard]] copter::LeftoverCopter& copter() { return copter_; }
    [[nodiscard]] sim::SimPlane& sim_plane() { return sim_plane_; }
    [[nodiscard]] const compass::Compass& compass() const { return compass_; }
    [[nodiscard]] std::uint32_t tick_count() const { return copter_.tick_count; }

private:
    copter::LeftoverCopter& copter_;
    sim::SimPlane& sim_plane_;
    compass::Compass compass_{};
};

namespace sitl_copter {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct PortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr PortItem kCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"SitlCopterHarness scaffold", PortStatus::kThisSlice,
     "refs LeftoverCopter + SimPlane; step sensor inject + leftover_copter_tick"},
    {"leftover_copter_tick", PortStatus::kThisSlice,
     "tick counter + CCP-035 update_flight_mode when Mode* set"},
    {"gyro/accel synthesis", PortStatus::kThisSlice,
     "SimPlane::gyro / accel_body → leftover buffers + inject flags"},
    {"baro synthesis", PortStatus::kThisSlice,
     "SimPlane altitude (-position.z) → leftover baro_altitude_m + flag"},
    {"GPS synthesis", PortStatus::kThisSlice,
     "home lat/lng + SimPlane NED north/east → leftover gps_lat/gps_lng + flag"},
    {"compass synthesis", PortStatus::kThisSlice,
     "Compass earth field via SimPlane::dcm → compass_field_bf + flag"},
    {"SitlHarness Plane path (CPP-084)", PortStatus::kOnMain,
     "sitl_harness.hpp; Plane+SimPlane closed loop"},
    {"CCP-035 update_flight_mode", PortStatus::kOnMain,
     "update_flight_mode.hpp; harness wires via leftover_copter_tick"},
    {"closed-loop arm/spool/hold", PortStatus::kRemaining,
     "Catch2 arm, spool, attitude hold driving SimPlane feedback"},
    {"multirotor aero / motor feedback", PortStatus::kRemaining,
     "SimPlane or multirotor stub update from motor outputs"},
    {"AP:: / HAL SITL singletons", PortStatus::kOutOfScope, "ADR-0012 explicit refs"},
    {"Rust copter-sitl", PortStatus::kOutOfScope, "Do not copy Rust"},
};

[[nodiscard]] inline constexpr std::size_t completeness_size() {
    return sizeof(kCompleteness) / sizeof(kCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

}  // namespace sitl_copter

}  // namespace fwcpp::hal_sitl
