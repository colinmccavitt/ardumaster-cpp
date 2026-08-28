#pragma once

// CPP-030 leftover-complete catalog.
//
// Surfaces already on main vs this leftover closer. Items marked
// PortStatus::kOnMain landed in earlier slices (skywalker_2013 aero,
// rigid-body integrator, CPP-051 wind, CPP-082 airspeed pressure,
// CPP-084/085 SitlHarness/sitl_run) and must not be redone.
// PortStatus::kThisSlice is leftover SimPlane surfaces stubbed here
// (ground_behavior taxi/takeoff variants and FW airframe mix).
// PortStatus::kRemaining is empty — tailsitter and GCS/MAVLink are
// PortStatus::kOutOfScope (fw-cpp is fixed-wing only; no GCS).

#include <cstddef>
#include <cstdint>

namespace fwcpp::sim {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct SimPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr SimPortItem kSimCompleteness[] = {
    {"skywalker_2013 aero + rigid-body integrator", PortStatus::kOnMain,
     "slice 1: liftCoeff/dragCoeff/getForce/getTorque + update_dynamics"},
    {"Wind modeling (CPP-051)", PortStatus::kOnMain,
     "update_wind / WindConfig; turbulence IIR + steady vector"},
    {"airspeed_sensor_differential_pressure (CPP-082)", PortStatus::kOnMain,
     "SITL pitot pressure model feeding AirspeedSensor"},
    {"SitlHarness / sitl_run (CPP-084, CPP-085)", PortStatus::kOnMain,
     "closed-loop harness + standalone sitl_run; do not rewrite here"},
    {"flat-earth on_ground clamp", PortStatus::kOnMain,
     "position.z>=0 + velocity_ef.z floor; no terrain snap"},
    {"ground_behavior NONE / NO_MOVEMENT / FWD_ONLY", PortStatus::kThisSlice,
     "SIM_Aircraft.cpp:787-846 taxi/takeoff-roll; default kNone"},
    {"elevons mix", PortStatus::kThisSlice,
     "SIM_Plane.cpp:409-418; FrameConfig::mix = kElevons"},
    {"vtail mix", PortStatus::kThisSlice,
     "SIM_Plane.cpp:419-425; FrameConfig::mix = kVtail"},
    {"dspoilers mix leftover surface", PortStatus::kThisSlice,
     "mix_dspoilers(); four-channel update() path leaves surfaces as given"},
    {"redundant mix leftover surface", PortStatus::kThisSlice,
     "mix_redundant(); paired-channel average"},
    {"reverse_elevator_rudder / reverse_thrust leftover surfaces", PortStatus::kThisSlice,
     "reverse_elevator_rudder negates elevator/rudder; reverse_thrust stores flag, throttle used as given"},
    {"leftover-complete catalog", PortStatus::kThisSlice,
     "this table"},
    {"tailsitter (airframe + GROUND_BEHAVIOR_TAILSITTER)", PortStatus::kOutOfScope,
     "fw-cpp is fixed-wing only; kTailsitter apply_ground_behavior is a no-op"},
    {"GCS / MAVLink / fill_fdm / hit-ground text", PortStatus::kOutOfScope,
     "no GCS/MAVLink; standing rule"},
};

[[nodiscard]] inline constexpr std::size_t sim_completeness_size() {
    return sizeof(kSimCompleteness) / sizeof(kSimCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kSimCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kSimCompleteness) {
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

}  // namespace fwcpp::sim
