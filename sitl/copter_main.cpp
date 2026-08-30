// CCP-044 slice 2: standalone copter_sitl_run -- arm, takeoff, hold, land.
// Mirrors sitl/main.cpp (CPP-085 Plane sitl_run) but drives LeftoverCopter
// + SimPlane through leftover_copter_sitl_step (CCP-044 leftover aero +
// CCP-043 SitlCopterHarness sensor inject). Charter definition of done:
// arm, takeoff, hold, land in SITL.
//
// USAGE: copter_sitl_run [--help] [duration_seconds]
// Default duration is 20 simulated seconds @ 400Hz -- long enough for the
// leftover climb/hold/land mission. Exits 0 if LANDED after climbing,
// 1 otherwise.

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/hal_sitl/copter_sitl_run_leftover.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_plane.hpp>

namespace {

void print_usage(const char* argv0) {
    std::cout << "usage: " << argv0 << " [--help] [duration_seconds > 0]\n"
              << "  CCP-044: LeftoverCopter + SimPlane + leftover_copter_sitl_step\n"
              << "  Mission: arm, takeoff to 10m, hold 2s, land.\n"
              << "  Default duration: 20 simulated seconds @ 400Hz.\n";
}

}  // namespace

int main(int argc, char** argv) {
    constexpr float kDt = 0.0025f;
    constexpr int kTicksPerSecond = static_cast<int>(1.0f / kDt);

    int duration_s = 20;
    if (argc > 1) {
        const std::string_view arg1{argv[1]};
        if (arg1 == "--help" || arg1 == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        duration_s = std::atoi(argv[1]);
        if (duration_s <= 0) {
            print_usage(argv[0]);
            return 1;
        }
    }
    const int num_ticks = duration_s * kTicksPerSecond;

    fwcpp::copter::LeftoverCopter copter{};
    fwcpp::sim::SimPlane sim_plane;
    fwcpp::hal_sitl::SitlCopterHarness harness(copter, sim_plane);
    fwcpp::hal_sitl::copter_sitl_run::LeftoverMission mission{};

    std::cout << "CCP-044 SITL: LeftoverCopter+SimPlane leftover_copter_sitl_step, "
              << duration_s << " simulated seconds (" << num_ticks << " ticks @ 400Hz)\n";
    std::cout << "mission: arm, takeoff " << mission.takeoff_alt_m << "m, hold "
              << mission.hold_s << "s, land\n";
    std::cout << std::fixed << std::setprecision(3);

    auto print_telemetry = [&](float t_s) {
        float true_roll_rad = 0.0f;
        float true_pitch_rad = 0.0f;
        float true_yaw_rad = 0.0f;
        sim_plane.dcm.to_euler(&true_roll_rad, &true_pitch_rad, &true_yaw_rad);

        std::cout << "t=" << std::setw(6) << t_s << "s"
                  << "  phase=" << fwcpp::hal_sitl::copter_sitl_run::mission_phase_name(mission.phase)
                  << "  armed=" << (copter.motors_armed ? 1 : 0)
                  << "  land_complete=" << (copter.land_complete ? 1 : 0)
                  << "  alt=" << (-sim_plane.position.z) << "m"
                  << "  baro=" << copter.baro_altitude_m << "m"
                  << "  thr=" << mission.throttle
                  << "  pos_ned=(" << sim_plane.position.x << ", " << sim_plane.position.y << ", "
                  << sim_plane.position.z << ")"
                  << "  true_rpy_deg=(" << fwcpp::math::degrees(true_roll_rad) << ", "
                  << fwcpp::math::degrees(true_pitch_rad) << ", "
                  << fwcpp::math::degrees(true_yaw_rad) << ")"
                  << "  ticks=" << harness.tick_count() << '\n';
    };

    print_telemetry(0.0f);

    // Brief disarmed dwell, then begin leftover takeoff (charter: arm).
    leftover_copter_sitl_step(harness, mission, kDt);
    fwcpp::hal_sitl::copter_sitl_run::leftover_mission_begin_takeoff(mission);

    float max_alt_m = 0.0f;
    int landed_tick = -1;
    for (int i = 1; i < num_ticks; ++i) {
        leftover_copter_sitl_step(harness, mission, kDt);
        const float alt = -sim_plane.position.z;
        if (alt > max_alt_m) {
            max_alt_m = alt;
        }
        if (landed_tick < 0 &&
            mission.phase == fwcpp::hal_sitl::copter_sitl_run::MissionPhase::kLanded) {
            landed_tick = i;
        }
        if ((i + 1) % kTicksPerSecond == 0) {
            print_telemetry(static_cast<float>(i + 1) * kDt);
        }
        // Stop ~1s after landing so the log shows LANDED.
        if (landed_tick >= 0 && (i - landed_tick) >= kTicksPerSecond) {
            break;
        }
    }

    print_telemetry(static_cast<float>(harness.tick_count()) * kDt);
    const bool ok = mission.phase == fwcpp::hal_sitl::copter_sitl_run::MissionPhase::kLanded &&
                    max_alt_m >= (mission.takeoff_alt_m * 0.9f) && copter.land_complete &&
                    sim_plane.on_ground();
    std::cout << "Done: ticks=" << harness.tick_count() << " max_alt=" << max_alt_m
              << "m phase=" << fwcpp::hal_sitl::copter_sitl_run::mission_phase_name(mission.phase)
              << (ok ? " SUCCESS\n" : " FAIL\n");
    return ok ? 0 : 1;
}
