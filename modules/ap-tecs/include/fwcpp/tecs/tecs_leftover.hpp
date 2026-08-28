#pragma once

// CPP-029 leftover-complete catalog.
//
// Surfaces already on main vs this leftover closer. Items marked
// PortStatus::kOnMain landed in earlier slices (NORMAL energy law,
// CPP-040 flare blend, CPP-041 get_land_sinkrate) and must not be
// redone. PortStatus::kThisSlice is leftover TECS surfaces stubbed
// here (no-airspeed throttle fallback, get_land_airspeed,
// set_path_proportion, leftover landing/takeoff input surfaces).
// PortStatus::kRemaining is empty — leftover TAKEOFF/LAND control-law
// bodies are ThisSlice stubs (surfaces exist, NORMAL law unchanged);
// VTOL and GCS/logging are PortStatus::kOutOfScope.

#include <cstddef>
#include <cstdint>

namespace fwcpp::tecs {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct TecsPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr TecsPortItem kTecsCompleteness[] = {
    {"NORMAL-stage energy control law", PortStatus::kOnMain,
     "slice 1: update_50hz / update_speed / update_speed_demand / "
     "update_height_demand / detect_underspeed / update_energies / "
     "update_throttle_with_airspeed / update_pitch"},
    {"TECS flare height-rate blend", PortStatus::kOnMain,
     "CPP-040: TecsLandingInputs::is_flaring + LAND_SINK/LAND_SRC/FLARE_HGT"},
    {"get_land_sinkrate", PortStatus::kOnMain,
     "CPP-041: Plane::setup_landing_glide_slope reads get_land_sinkrate()"},
    {"use_airspeed / TecsInputs::using_airspeed_sensor", PortStatus::kOnMain,
     "CPP-082/083: airspeed sensor exists; using_airspeed_sensor can be false"},
    {"update_throttle_without_airspeed", PortStatus::kThisSlice,
     "AP_TECS.cpp:910-957 pitch-to-throttle mapping; LAND_THR leftover arm"},
    {"get_land_airspeed", PortStatus::kThisSlice,
     "AP_TECS.h:107-109 LAND_ARSPD default -1 disabled sentinel"},
    {"set_path_proportion", PortStatus::kThisSlice,
     "AP_TECS.h:117-119 constrain [0,1]; LAND_SPDWGT consumer is a no-op stub"},
    {"leftover LAND_ARSPD / LAND_THR gains", PortStatus::kThisSlice,
     "Gains::land_airspeed / land_throttle; not in tecs_group_info()"},
    {"leftover is_doing_auto_land / throttle_nudge / pitch_trim_deg", PortStatus::kThisSlice,
     "TecsLandingInputs leftover fields; default keeps NORMAL path"},
    {"leftover FlightStage / reached_speed_takeoff stubs", PortStatus::kThisSlice,
     "TAKEOFF/LAND/ABORT_LANDING surfaces stored; control-law bodies no-op"},
    {"leftover-complete catalog", PortStatus::kThisSlice,
     "this table"},
    {"VTOL flight-stage branches", PortStatus::kOutOfScope,
     "fw-cpp is fixed-wing only: VTOL SPE-zero / underspeed-clear / "
     "bad-descent early-out / speed-weighting"},
    {"GCS send_TECS_status / HAL_LOGGING_ENABLED", PortStatus::kOutOfScope,
     "no GCS/MAVLink; standing rule"},
};

[[nodiscard]] inline constexpr std::size_t tecs_completeness_size() {
    return sizeof(kTecsCompleteness) / sizeof(kTecsCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kTecsCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kTecsCompleteness) {
        bool match = true;
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                match = false;
                break;
            }
            ++a;
            ++b;
        }
        if (match && *a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t remaining_count() { return count_status(PortStatus::kRemaining); }
[[nodiscard]] inline constexpr std::size_t on_main_count() { return count_status(PortStatus::kOnMain); }
[[nodiscard]] inline constexpr std::size_t this_slice_count() { return count_status(PortStatus::kThisSlice); }
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() { return count_status(PortStatus::kOutOfScope); }

}  // namespace fwcpp::tecs
