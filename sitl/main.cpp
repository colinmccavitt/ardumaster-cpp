// CPP-085: the FIRST real, standalone, runnable executable this entire
// port has ever had. Every previous ticket proved its own code correct
// only through the Catch2 test suite (tests/) - this program is not a
// test: it is a genuine `int main()` that constructs a real `Plane` +
// `SimPlane`, drives them tick-by-tick through `fwcpp::hal_sitl::
// SitlHarness` (CPP-084), and prints periodic telemetry a human can
// actually read and judge as "a plane is flying" or not.
//
// SCENARIO CHOSEN, AND WHY: tests/vehicle_test.cpp's own "Closed loop:
// FBWA holding a constant commanded bank angle converges in SimPlane's
// ground truth" test (search that exact string) is this port's own
// already-proven "getting a Plane flying" recipe - re-used here
// VERBATIM, not reinvented:
//   - ModeFBWA, wired in via `plane.control_mode = &fbwa;` (the same
//     CPP-031-slice-7 dispatch pattern that test's own comment cites).
//   - `plane.armed = true; plane.hal.rc_output.force_safety_off();` -
//     that test's own comment explains why `plane.arm()` itself isn't
//     used instead: its rc_received_if_enabled_check() gate would fail
//     before set_sticks() is ever called.
//   - The exact same fixed stick command every tick: set_sticks(plane,
//     1650, 1500, 1700, 1500) - "a fixed, moderate right-roll stick
//     command plus enough throttle to build and hold airspeed; pitch/
//     rudder centered" (that test's own comment, quoted verbatim).
//   - set_sticks() itself is copied unchanged from that file's own
//     anonymous-namespace helper (same body, same two calls -
//     rc_input.set_channel() then rc_channels.read_input() - even
//     though SitlHarness::step()'s own tick() call internally repeats
//     that same read_input() call as its own step 1; harmless, and
//     kept for exact fidelity to the proven helper rather than a
//     "cleverer" divergent version).
// The only thing this executable does differently from that test is
// HOW each tick is driven: that test hand-rolls gyro/accel/compass/GPS/
// airspeed synthesis and the SimPlane feedback loop inline; this
// executable delegates all of that, every tick, to SitlHarness::step()
// (CPP-084) - the exact "Phase 2" relationship CPP-084's own file
// banner describes.
//
// THE REAL GAP THIS EXECUTABLE CLOSES (the ticket's own primary
// motivation, per CPP-083's investigation): plane.airspeed_sensor has a
// real, tested boot-time zero-offset calibration routine
// (start_calibration(), CPP-083) that no live caller anywhere in this
// port had ever invoked - every existing airspeed_sensor_test.cpp/
// vehicle_test.cpp caller either skips calibration entirely (calling
// update() with the single-argument overload, so calibration_state()
// stays NotStarted forever) or drives start_calibration() only as a
// unit under test, never as part of an actual "boot a vehicle" sequence.
// This main() calls it exactly once, before the loop begins, mirroring
// upstream AP_Vehicle::setup()'s own unconditional airspeed.calibrate
// (true) boot call (see airspeed_sensor.hpp's own file banner) - this
// port's first genuine equivalent of that moment.
//
// DURATION: a bounded default (60 simulated seconds = 3000 ticks @
// 50Hz, matching this port's established tick rate throughout the test
// suite) - long enough to watch the commanded bank angle converge (the
// FBWA test above converges within ~30s) and then hold. An optional
// first command-line argument overrides the simulated duration in
// seconds. An indefinite/interactive run mode is real future work, not
// built here (see the ticket's own "your call" framing).
//
// OUTPUT: one telemetry line per simulated second (not per 20ms tick) -
// SimPlane's own TRUE position/attitude/airspeed (the actual physics
// ground truth being verified, not the AHRS estimate of it - the same
// ground-truth fields vehicle_test.cpp's own closed-loop tests assert
// against), plus the AHRS's own attitude estimate for comparison, plus
// the fixed mode name (this executable never switches modes) and the
// airspeed sensor's own calibration_state() transition, logged once,
// the moment it completes.

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>

#include <fwcpp/hal_sitl/sitl_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_plane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

namespace {

// Copied verbatim from tests/vehicle_test.cpp's own anonymous-namespace
// helper of the same name - see this file's own top banner for why.
void set_sticks(fwcpp::vehicle::Plane& plane, std::uint16_t roll_pwm, std::uint16_t pitch_pwm,
                 std::uint16_t throttle_pwm, std::uint16_t rudder_pwm) {
    plane.hal.rc_input.set_channel(fwcpp::vehicle::kChannelRoll, roll_pwm);
    plane.hal.rc_input.set_channel(fwcpp::vehicle::kChannelPitch, pitch_pwm);
    plane.hal.rc_input.set_channel(fwcpp::vehicle::kChannelThrottle, throttle_pwm);
    plane.hal.rc_input.set_channel(fwcpp::vehicle::kChannelRudder, rudder_pwm);
    plane.rc_channels.read_input(plane.hal.rc_input);
}

const char* calibration_state_name(fwcpp::airspeed::CalibrationState state) {
    switch (state) {
    case fwcpp::airspeed::CalibrationState::NotStarted:
        return "NotStarted";
    case fwcpp::airspeed::CalibrationState::InProgress:
        return "InProgress";
    case fwcpp::airspeed::CalibrationState::Success:
        return "Success";
    case fwcpp::airspeed::CalibrationState::Failed:
        return "Failed";
    }
    return "?";
}

} // namespace

int main(int argc, char** argv) {
    constexpr float kDt = 0.02f; // 50Hz - this port's established tick rate throughout the test suite.
    constexpr int kTicksPerSecond = static_cast<int>(1.0f / kDt);

    int duration_s = 60; // bounded default - see this file's own top banner.
    if (argc > 1) {
        duration_s = std::atoi(argv[1]);
        if (duration_s <= 0) {
            std::cerr << "usage: " << argv[0] << " [duration_seconds > 0]\n";
            return 1;
        }
    }
    const int num_ticks = duration_s * kTicksPerSecond;

    fwcpp::vehicle::Plane plane;
    fwcpp::sim::SimPlane sim_plane;
    fwcpp::vehicle::ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;

    // Same real underlying primitives the proven FBWA closed-loop test
    // sets directly, in place of plane.arm() - see this file's own top
    // banner for why.
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();

    // THE REAL GAP THIS EXECUTABLE CLOSES: this port's first-ever live
    // caller of the boot-time airspeed zero-offset calibration routine,
    // called exactly once, before the loop begins - see this file's own
    // top banner.
    plane.airspeed_sensor.start_calibration(0);

    fwcpp::hal_sitl::SitlHarness harness(plane, sim_plane);

    std::cout << "CPP-085 SITL executable: Plane+SimPlane+SitlHarness, ModeFBWA, "
              << duration_s << " simulated seconds (" << num_ticks << " ticks @ 50Hz)\n";
    std::cout << std::fixed << std::setprecision(3);

    auto print_telemetry = [&](std::uint32_t now_ms) {
        float true_roll_rad = 0.0f;
        float true_pitch_rad = 0.0f;
        float true_yaw_rad = 0.0f;
        sim_plane.dcm.to_euler(&true_roll_rad, &true_pitch_rad, &true_yaw_rad);

        std::cout << "t=" << std::setw(6) << (static_cast<float>(now_ms) / 1000.0f) << "s"
                  << "  mode=FBWA"
                  << "  pos_ned=(" << sim_plane.position.x << ", " << sim_plane.position.y << ", " << sim_plane.position.z
                  << ")"
                  << "  alt=" << (-sim_plane.position.z) << "m"
                  << "  true_rpy_deg=(" << fwcpp::math::degrees(true_roll_rad) << ", " << fwcpp::math::degrees(true_pitch_rad)
                  << ", " << fwcpp::math::degrees(true_yaw_rad) << ")"
                  << "  ahrs_rpy_deg=(" << fwcpp::math::degrees(plane.ahrs->get_roll()) << ", "
                  << fwcpp::math::degrees(plane.ahrs->get_pitch()) << ", " << fwcpp::math::degrees(plane.ahrs->get_yaw())
                  << ")"
                  << "  true_airspeed=" << sim_plane.airspeed << "m/s\n";
    };

    fwcpp::airspeed::CalibrationState prev_cal_state = plane.airspeed_sensor.calibration_state();

    std::uint32_t now_ms = 0;
    print_telemetry(now_ms); // t=0 baseline, before the first tick.

    for (int i = 0; i < num_ticks; ++i) {
        now_ms += 20;

        // Same fixed, moderate right-roll stick command as the proven
        // FBWA closed-loop test - see this file's own top banner.
        set_sticks(plane, 1650, 1500, 1700, 1500);

        harness.step(now_ms, kDt);

        const fwcpp::airspeed::CalibrationState cal_state = plane.airspeed_sensor.calibration_state();
        if (cal_state != prev_cal_state) {
            std::cout << "  [airspeed_sensor] calibration_state: " << calibration_state_name(prev_cal_state) << " -> "
                      << calibration_state_name(cal_state) << " at t=" << (static_cast<float>(now_ms) / 1000.0f) << "s\n";
            prev_cal_state = cal_state;
        }

        if ((i + 1) % kTicksPerSecond == 0) {
            print_telemetry(now_ms);
        }
    }

    std::cout << "Done: " << num_ticks << " ticks (" << duration_s << " simulated seconds) completed.\n";
    return 0;
}
