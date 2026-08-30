// CCP-044 slice 1: scaffold standalone executable for Copter SITL.
// Mirrors sitl/main.cpp (CPP-085 Plane sitl_run) but drives
// LeftoverCopter + SimPlane through SitlCopterHarness (CCP-043) instead
// of Plane + SitlHarness.
//
// This slice is deliberately thin: construct the three objects, loop
// harness.step(dt), print minimal telemetry. No arm / takeoff / hold /
// land closed loop yet — that remains for later CCP-044 slices once
// CCP-043's closed-loop leftovers mature. Completeness catalog lives in
// sitl_copter_harness.hpp (optional reference from this executable).
//
// USAGE: copter_sitl_run [--help] [duration_seconds]
// Default duration is a few simulated seconds so a human can confirm
// the binary runs without waiting for a full flight.

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string_view>

#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_plane.hpp>

namespace {

void print_usage(const char* argv0) {
    std::cout << "usage: " << argv0 << " [--help] [duration_seconds > 0]\n"
              << "  CCP-044 slice 1 scaffold: LeftoverCopter + SimPlane + "
                 "SitlCopterHarness step loop.\n"
              << "  Default duration: 2 simulated seconds @ 400Hz "
                 "(Copter tick rate).\n";
}

} // namespace

int main(int argc, char** argv) {
    // 400Hz — common Copter fast-loop rate; SitlCopterHarness tests use
    // the same 0.0025s dt.
    constexpr float kDt = 0.0025f;
    constexpr int kTicksPerSecond = static_cast<int>(1.0f / kDt);

    int duration_s = 2;
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

    std::cout << "CCP-044 SITL scaffold: LeftoverCopter+SimPlane+SitlCopterHarness, "
              << duration_s << " simulated seconds (" << num_ticks << " ticks @ 400Hz)\n";
    std::cout << std::fixed << std::setprecision(3);

    auto print_telemetry = [&](float t_s) {
        float true_roll_rad = 0.0f;
        float true_pitch_rad = 0.0f;
        float true_yaw_rad = 0.0f;
        sim_plane.dcm.to_euler(&true_roll_rad, &true_pitch_rad, &true_yaw_rad);

        std::cout << "t=" << std::setw(6) << t_s << "s"
                  << "  ticks=" << harness.tick_count()
                  << "  pos_ned=(" << sim_plane.position.x << ", " << sim_plane.position.y << ", "
                  << sim_plane.position.z << ")"
                  << "  alt=" << (-sim_plane.position.z) << "m"
                  << "  baro=" << copter.baro_altitude_m << "m"
                  << "  true_rpy_deg=(" << fwcpp::math::degrees(true_roll_rad) << ", "
                  << fwcpp::math::degrees(true_pitch_rad) << ", "
                  << fwcpp::math::degrees(true_yaw_rad) << ")"
                  << "  gyro_inj=" << (copter.gyro_injected ? 1 : 0)
                  << " gps_inj=" << (copter.gps_injected ? 1 : 0)
                  << " compass_inj=" << (copter.compass_injected ? 1 : 0) << '\n';
    };

    print_telemetry(0.0f);

    for (int i = 0; i < num_ticks; ++i) {
        harness.step(kDt);

        if ((i + 1) % kTicksPerSecond == 0) {
            print_telemetry(static_cast<float>(i + 1) * kDt);
        }
    }

    std::cout << "Done: " << num_ticks << " ticks (" << duration_s
              << " simulated seconds) completed.\n";
    return 0;
}
