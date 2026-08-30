#pragma once

// CCP-044 leftover completeness catalog + leftover motor->SimPlane aero
// and arm/takeoff/hold/land mission. Upstream ROLE: SITL::MultiCopter
// (libraries/SITL/SIM_Multicopter.cpp calculate_forces / update) plus
// the standalone SITL executable shape of AP_HAL_SITL HAL_SITL_Class
// (mirrors CPP-085 sitl/sitl_run, not a literal HAL port).
//
// Slice 2 (close): leftover_multirotor_aero (collective throttle as
// body-z thrust through SimPlane::update_dynamics) + leftover_mission
// advance (arm / takeoff / hold / land) + copter_sitl_run main() drives
// leftover_copter_sitl_step. Full SIM_Multicopter Frame/Motor mixing,
// GCS/MAVLink, and interactive run are kOutOfScope (disclosed
// simplification; ADR-0012). remaining_count()==0.
//
// Do NOT copy Rust. Do NOT change SitlCopterHarness::step() physics
// (CCP-043 remaining_count==0; aero lives here, called before step()).

#include <cstddef>
#include <cstdint>

#include <fwcpp/copter/land_detector.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/takeoff.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_plane.hpp>

namespace fwcpp::hal_sitl::copter_sitl_run {

enum class MissionPhase : std::uint8_t {
    kDisarmed = 0,
    kTakeoff = 1,
    kHold = 2,
    kLand = 3,
    kLanded = 4,
};

struct LeftoverMission {
    MissionPhase phase{MissionPhase::kDisarmed};
    float takeoff_alt_m{10.0f};
    float climb_throttle{0.90f};
    float land_throttle{0.40f};
    float hold_s{2.0f};
    float hold_elapsed_s{0.0f};
    float throttle{0.0f};
    copter::TakeOffState takeoff{};
};

[[nodiscard]] inline const char* mission_phase_name(MissionPhase phase) {
    switch (phase) {
    case MissionPhase::kDisarmed:
        return "DISARMED";
    case MissionPhase::kTakeoff:
        return "TAKEOFF";
    case MissionPhase::kHold:
        return "HOLD";
    case MissionPhase::kLand:
        return "LAND";
    case MissionPhase::kLanded:
        return "LANDED";
    }
    return "?";
}

// Leftover SITL::MultiCopter::calculate_forces -- collective throttle as
// body-z thrust only. hover_throttle produces 1g (hover). Full
// SIM_Frame / SIM_Motor mixing is kOutOfScope.
inline void leftover_multirotor_aero(sim::SimPlane& sim, float throttle, float dt) {
    const float thr = math::constrain_value(throttle, 0.0f, 1.0f);
    const float hover = (sim.hover_throttle > 0.0f) ? sim.hover_throttle : 0.7f;
    const float thrust_acc = (thr / hover) * sim::kGravityMss;
    sim.accel_body = math::Vector3f{0.0f, 0.0f, -thrust_acc};
    sim.update_dynamics(math::Vector3f{0.0f, 0.0f, 0.0f}, dt);
}

// Leftover hover throttle: hover_throttle plus a thin vertical-rate
// damper so HOLD kills climb/descent (not AC_PosControl). NED +z down:
// positive vz (descending) -> more throttle.
[[nodiscard]] inline float leftover_hover_throttle(const sim::SimPlane& sim) {
    const float hover = (sim.hover_throttle > 0.0f) ? sim.hover_throttle : 0.7f;
    constexpr float kVelGain = 0.08f;
    return math::constrain_value(hover + kVelGain * sim.velocity_ef.z, 0.0f, 1.0f);
}

inline void leftover_mission_begin_takeoff(LeftoverMission& mission) {
    mission.phase = MissionPhase::kTakeoff;
    mission.hold_elapsed_s = 0.0f;
}

// One leftover tick of the charter mission: arm, takeoff, hold, land.
// Sets leftover motors_armed / land_complete / throttle; caller then
// leftover_multirotor_aero + SitlCopterHarness::step.
inline void leftover_mission_advance(copter::LeftoverCopter& copter, sim::SimPlane& sim,
                                     LeftoverMission& mission, float dt) {
    const float alt_m = -sim.position.z;

    switch (mission.phase) {
    case MissionPhase::kDisarmed:
        copter.motors_armed = false;
        copter.land_complete = true;
        mission.throttle = 0.0f;
        break;

    case MissionPhase::kTakeoff: {
        copter.motors_armed = true;
        if (copter.land_complete) {
            copter::UserTakeoffInputs in;
            in.motors_armed = true;
            in.land_complete = true;
            in.has_user_takeoff = true;
            in.takeoff_alt_m = mission.takeoff_alt_m;
            in.current_alt_m = alt_m;
            copter::UserTakeoffEffects fx;
            if (leftover_do_user_takeoff_U_m(in, fx, &mission.takeoff, alt_m) &&
                fx.leftover_takeoff_start_m) {
                copter.land_complete = false;
            }
        }
        mission.throttle = mission.climb_throttle;
        if (alt_m >= mission.takeoff_alt_m) {
            mission.takeoff._running = false;
            mission.phase = MissionPhase::kHold;
            mission.hold_elapsed_s = 0.0f;
            mission.throttle = leftover_hover_throttle(sim);
        }
        break;
    }

    case MissionPhase::kHold:
        copter.motors_armed = true;
        mission.throttle = leftover_hover_throttle(sim);
        mission.hold_elapsed_s += dt;
        if (mission.hold_elapsed_s >= mission.hold_s) {
            mission.phase = MissionPhase::kLand;
        }
        break;

    case MissionPhase::kLand: {
        copter.motors_armed = true;
        mission.throttle = mission.land_throttle;
        if (sim.on_ground()) {
            copter::LandDetectorInputs lin;
            lin.motors_armed = true;
            lin.land_complete = copter.land_complete;
            lin.descent_rate_low = true;
            lin.throttle_at_lower_limit = true;
            lin.motors_throttle_low = true;
            lin.throttle_mix_min = true;
            lin.accel_stationary = true;
            lin.rangefinder_check = true;
            lin.wow_check = true;
            copter::LandDetectorEffects lfx;
            leftover_update_land_and_crash_detectors(lin, lfx);
            if (lfx.land_complete) {
                copter.land_complete = true;
                copter.motors_armed = false;
                mission.throttle = 0.0f;
                mission.phase = MissionPhase::kLanded;
            }
        }
        break;
    }

    case MissionPhase::kLanded:
        copter.motors_armed = false;
        copter.land_complete = true;
        mission.throttle = 0.0f;
        break;
    }
}

// Physics then CCP-043 sensor inject + leftover_copter_tick.
inline void leftover_copter_sitl_step(SitlCopterHarness& harness, LeftoverMission& mission,
                                      float dt) {
    leftover_mission_advance(harness.copter(), harness.sim_plane(), mission, dt);
    leftover_multirotor_aero(harness.sim_plane(), mission.throttle, dt);
    harness.step(dt);
}

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
    {"leftover catalog", PortStatus::kThisSlice, "this table; CCP-044 close"},
    {"copter_sitl_run scaffold", PortStatus::kThisSlice,
     "sitl/copter_main.cpp + CMake copter_sitl_run target (slice 1)"},
    {"leftover_multirotor_aero", PortStatus::kThisSlice,
     "collective throttle as body-z thrust; SimPlane::update_dynamics"},
    {"leftover_mission_advance", PortStatus::kThisSlice,
     "arm / takeoff / hold / land leftover state machine"},
    {"leftover_hover_throttle", PortStatus::kThisSlice,
     "HOLD vertical-rate damper around hover_throttle; not AC_PosControl"},
    {"leftover_copter_sitl_step", PortStatus::kThisSlice,
     "aero then SitlCopterHarness::step (CCP-043 sensor inject)"},
    {"copter_sitl_run arm/takeoff/hold/land", PortStatus::kThisSlice,
     "main() drives leftover mission; periodic telemetry; success on LANDED"},
    {"SitlCopterHarness sensor synth (CCP-043)", PortStatus::kOnMain,
     "sitl_copter_harness.hpp remaining_count()==0"},
    {"leftover takeoff / land_detector (CCP-041)", PortStatus::kOnMain,
     "takeoff.hpp + land_detector.hpp remaining_count()==0"},
    {"SIM_Multicopter Frame/Motor mixing", PortStatus::kOutOfScope,
     "disclosed simplification: collective body-z thrust only, no SIM_Frame"},
    {"GCS / MAVLink / interactive run", PortStatus::kOutOfScope,
     "no GCS in this port; bounded duration like CPP-085"},
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

}  // namespace fwcpp::hal_sitl::copter_sitl_run
