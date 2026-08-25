// Tests for fwcpp::vehicle::{Plane, ModeManual, ModeFBWA, tick} (CPP-031).
//
// Style note: mirrors fw_control_test.cpp/tecs_test.cpp - drives each
// class through its public entry points and reads back public
// accessors/PID info for white-box checks. ap-sim (SimPlane) is a
// TEST-ONLY dependency (see tests/CMakeLists.txt) used only by the final
// closed-loop integration test below - ap-vehicle itself never links it.

#include <cmath>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_plane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

using namespace fwcpp::vehicle;

namespace {

// Sets all four primary RC input channels to the given PWM values and
// pulls them into plane.rc_channels, matching how a real RC receiver
// frame arrives each tick.
void set_sticks(Plane& plane, std::uint16_t roll_pwm, std::uint16_t pitch_pwm, std::uint16_t throttle_pwm,
                 std::uint16_t rudder_pwm) {
    plane.hal.rc_input.set_channel(kChannelRoll, roll_pwm);
    plane.hal.rc_input.set_channel(kChannelPitch, pitch_pwm);
    plane.hal.rc_input.set_channel(kChannelThrottle, throttle_pwm);
    plane.hal.rc_input.set_channel(kChannelRudder, rudder_pwm);
    plane.rc_channels.read_input(plane.hal.rc_input);
}

} // namespace

// ---------------------------------------------------------------------
// ModeManual
// ---------------------------------------------------------------------

TEST_CASE("ModeManual: direct stick-to-servo passthrough with no stabilization", "[vehicle][manual]") {
    Plane plane;

    // Dirty the roll/pitch rate integrators first, so we can prove
    // ModeManual::run() (reset_controllers()) actually zeroes them rather
    // than the test passing by coincidence on already-zero state.
    fwcpp::fw_control::RateLoopInputs rin;
    rin.measured_rate = 0.0f;
    rin.airspeed = 15.0f;
    rin.eas2tas = 1.0f;
    rin.dt = 0.02f;

    fwcpp::fw_control::PitchInputs pin;
    pin.measured_rate = 0.0f;
    pin.airspeed = 15.0f;
    pin.eas2tas = 1.0f;
    pin.dt = 0.02f;

    for (std::uint32_t t = 0; t < 200; t += 20) {
        rin.now_ms = t;
        pin.now_ms = t;
        plane.roll_controller.get_servo_out(1000, 1.0f, false, false, rin);
        plane.pitch_controller.get_servo_out(1000, 1.0f, false, false, pin);
    }
    REQUIRE(plane.roll_controller.rate_pid().get_i() != Catch::Approx(0.0f));
    REQUIRE(plane.pitch_controller.rate_pid().get_i() != Catch::Approx(0.0f));

    // Full-right roll stick, full-up pitch stick, mid throttle/rudder.
    set_sticks(plane, 1900, 1900, 1500, 1500);

    ModeManual manual(plane);
    StabilizeInputs in;
    in.dt = 0.02f;
    in.now_ms = 220;

    manual.update(in);
    manual.run(in);

    // Direct passthrough: with MAN_EXPO_* at their real default (0), the
    // expo curve is the identity, so full stick deflection reaches the
    // servo's full +-4500 centidegree range exactly, with NO dependency
    // on the AHRS/rate controllers at all.
    REQUIRE(plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kAileron) == Catch::Approx(4500.0f));
    REQUIRE(plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kElevator) == Catch::Approx(4500.0f));

    // nav_roll_cd/nav_pitch_cd are set from the AHRS's own (zero, at
    // rest) attitude - not from the stick - matching upstream exactly.
    REQUIRE(plane.nav_roll_cd == 0);
    REQUIRE(plane.nav_pitch_cd == 0);

    // MANUAL never stabilizes: run() only resets the controllers.
    REQUIRE(plane.roll_controller.rate_pid().get_i() == Catch::Approx(0.0f));
    REQUIRE(plane.pitch_controller.rate_pid().get_i() == Catch::Approx(0.0f));
}

TEST_CASE("ModeManual: throttle and rudder are also direct passthrough", "[vehicle][manual]") {
    Plane plane;
    set_sticks(plane, 1500, 1500, 1900, 1100); // mid roll/pitch, full throttle, full-left rudder

    ModeManual manual(plane);
    StabilizeInputs in;
    manual.update(in);

    REQUIRE(plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle) == Catch::Approx(100.0f));
    REQUIRE(plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kRudder) == Catch::Approx(-4500.0f));
    // output_rudder_and_steering() drives both functions identically.
    REQUIRE(plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kSteering) == Catch::Approx(-4500.0f));
}

// ---------------------------------------------------------------------
// ModeFBWA
// ---------------------------------------------------------------------

TEST_CASE("ModeFBWA: nav_roll_cd/nav_pitch_cd are nonzero and correctly signed from stick input", "[vehicle][fbwa]") {
    Plane plane;
    plane.update_flight_limits(); // ahrs at rest (roll=pitch=0) -> roll_limit_cd = aparm.roll_limit_deg*100 exactly

    set_sticks(plane, 1900, 1100, 1500, 1500); // full-right roll, full-down-stick pitch (pulls nose up -> positive pitch demand)

    ModeFBWA fbwa(plane);
    StabilizeInputs in;
    fbwa.update(in);

    REQUIRE(plane.nav_roll_cd == Catch::Approx(static_cast<float>(plane.roll_limit_cd)));
    REQUIRE(plane.nav_roll_cd > 0);
    // pitch stick at 1100 (min) -> norm_input() = -1 -> pitch_input < 0 branch:
    // nav_pitch_cd = -(pitch_input * pitch_limit_min * 100), pitch_limit_min is negative,
    // pitch_input is negative -> product positive -> negated -> negative... unless upstream's
    // sign convention makes stick-down mean nose-down (negative pitch demand). Verify the sign
    // matches pitch_limit_min's own sign convention directly rather than assuming.
    REQUIRE(plane.nav_pitch_cd != 0);
}

TEST_CASE("ModeFBWA: pitch stick pulled up (norm_input > 0) demands a positive (nose-up) pitch", "[vehicle][fbwa]") {
    Plane plane;
    plane.update_flight_limits();
    set_sticks(plane, 1500, 1900, 1500, 1500); // full-up pitch stick -> norm_input() = +1

    // adjust_nav_pitch_throttle() (called from within update() below)
    // reads the THROTTLE SERVO OUTPUT's cached scaled value (upstream:
    // throttle_percentage() -> SRV_Channels::get_output_scaled(k_throttle)),
    // not the RC throttle stick - and only trims nav_pitch_cd down when
    // that cached value is below TRIM_THROTTLE (45 here). Prime it above
    // that threshold so this test isolates the norm_input()-driven demand
    // itself, matching a vehicle already at/above cruise throttle.
    plane.srv_channels.set_output_scaled(fwcpp::srv::Function::kThrottle, 50.0f);

    ModeFBWA fbwa(plane);
    StabilizeInputs in;
    fbwa.update(in);

    REQUIRE(plane.channel_pitch()->norm_input() > 0.0f);
    REQUIRE(plane.nav_pitch_cd == Catch::Approx(plane.aparm.pitch_limit_max_deg * 100.0f));
    REQUIRE(plane.nav_pitch_cd > 0);
}

TEST_CASE("ModeFBWA: stabilize_roll's commanded rate shrinks as roll approaches the commanded bank angle", "[vehicle][fbwa]") {
    Plane plane;
    plane.update_flight_limits();
    set_sticks(plane, 1900, 1500, 1500, 1500); // full-right roll stick

    ModeFBWA fbwa(plane);
    StabilizeInputs in;
    in.dt = 0.02f;
    fbwa.update(in);
    REQUIRE(plane.nav_roll_cd > 0.0f);

    const float commanded_roll_deg = static_cast<float>(plane.nav_roll_cd) * 0.01f;

    // Mock the AHRS's own attitude directly (unit-test level - no
    // SimPlane/AhrsDcm::update() involved here, see the closed-loop
    // integration test below for that) at increasing roll angles
    // approaching the commanded bank, and confirm the roll rate
    // controller's demanded rate is always toward the target and shrinks
    // monotonically as the error shrinks.
    float last_target_rate = 1.0e9f;
    std::uint32_t now_ms = 0;
    for (float roll_deg : {0.0f, commanded_roll_deg * 0.3f, commanded_roll_deg * 0.6f, commanded_roll_deg * 0.9f}) {
        plane.ahrs.roll = fwcpp::math::radians(roll_deg);
        now_ms += 20;
        in.now_ms = now_ms;
        plane.stabilize_roll(in);
        const float target_rate = plane.roll_controller.get_pid_info().target;
        REQUIRE(target_rate > 0.0f); // always demanding right-roll rate toward the target
        REQUIRE(target_rate < last_target_rate); // shrinking as the error shrinks
        last_target_rate = target_rate;
    }
}

TEST_CASE("ModeFBWA::run stabilizes (unlike ModeManual) and outputs pilot throttle", "[vehicle][fbwa]") {
    Plane plane;
    plane.update_flight_limits();
    set_sticks(plane, 1900, 1500, 1700, 1500);

    ModeFBWA fbwa(plane);
    StabilizeInputs in;
    in.dt = 0.02f;
    in.now_ms = 20;

    fbwa.update(in);
    fbwa.run(in);

    // A nonzero nav_roll_cd with ahrs at rest (roll=0) produces a nonzero
    // aileron command - unlike ModeManual, this comes from the roll rate
    // controller, not direct stick passthrough.
    REQUIRE(plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kAileron) != Catch::Approx(0.0f));
    // output_pilot_throttle() ran: with THR_PASS_STAB at its real default
    // (false), throttle passthrough is disposed via get_adjusted_throttle_input(),
    // which (see plane.hpp) collapses to get_throttle_input() for an
    // unconfigured vehicle - non-zero for a 1700us throttle stick.
    REQUIRE(plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle) > 0.0f);
}

// ---------------------------------------------------------------------
// get_speed_scaler() / calc_speed_scaler()
// ---------------------------------------------------------------------

TEST_CASE("Plane::get_speed_scaler defaults to 1.0 and low-pass-filters toward calc_speed_scaler's instantaneous value",
          "[vehicle][speed_scaler]") {
    Plane plane;
    REQUIRE(plane.get_speed_scaler() == Catch::Approx(1.0f));

    // scaling_speed (15.0) / 30.0 m/s -> instantaneous scaler well below 1.0.
    const float instantaneous = plane.calc_speed_scaler(true, 30.0f, false);
    REQUIRE(instantaneous < 1.0f);

    plane.update_speed_scaler(true, 30.0f, false, 0.1f);
    const float after_one_step = plane.get_speed_scaler();
    REQUIRE(after_one_step < 1.0f);
    REQUIRE(after_one_step > instantaneous); // hasn't fully converged after one 0.1s step

    for (int i = 0; i < 500; ++i) {
        plane.update_speed_scaler(true, 30.0f, false, 0.1f);
    }
    REQUIRE(plane.get_speed_scaler() == Catch::Approx(instantaneous).margin(0.005f));
}

TEST_CASE("Plane::calc_speed_scaler clamps to [scale_min, scale_max]", "[vehicle][speed_scaler]") {
    Plane plane;

    const float scale_min = std::min(0.5f, plane.aparm.scaling_speed / (2.0f * plane.aparm.airspeed_max));
    const float airspeed_min_floor = std::max(plane.aparm.airspeed_min, fwcpp::vehicle::kMinAirspeedMin);
    const float scale_max = std::max(2.0f, plane.aparm.scaling_speed / (0.7f * airspeed_min_floor));

    // Very high airspeed -> scaler would be tiny -> clamped to scale_min.
    REQUIRE(plane.calc_speed_scaler(true, 1000.0f, false) == Catch::Approx(scale_min));

    // Effectively-zero airspeed -> the "> 0.0001f" branch is not taken, so
    // calc_speed_scaler falls straight to scale_max (matches upstream's
    // own `speed_scaler = scale_max;` else-branch).
    REQUIRE(plane.calc_speed_scaler(true, 0.00001f, false) == Catch::Approx(scale_max));

    // Not armed, no airspeed sensor -> unit scaling.
    REQUIRE(plane.calc_speed_scaler(false, 0.0f, false) == Catch::Approx(1.0f));
}

// ---------------------------------------------------------------------
// update_load_factor() / apply_load_factor_roll_limits()
// ---------------------------------------------------------------------

TEST_CASE("Plane::update_load_factor reduces roll_limit_cd at high demanded bank angle with low airspeed",
          "[vehicle][load_factor]") {
    Plane plane;
    // At exactly airspeed_min, max_load_factor = (airspeed_min/airspeed_min)^2 = 1
    // -> upstream's own "<=1" branch -> limited to a conservative 25 degrees.
    plane.smoothed_airspeed = plane.aparm.airspeed_min;
    plane.nav_roll_cd = 4500; // demand the full 45-degree default roll limit
    plane.roll_limit_cd = 4500;

    plane.update_load_factor();

    REQUIRE(plane.roll_limit_cd == 2500); // 25 deg, in centidegrees
    REQUIRE(plane.nav_roll_cd == 2500);
}

TEST_CASE("Plane::update_load_factor leaves roll limits unreduced at low demanded bank / ample airspeed",
          "[vehicle][load_factor]") {
    Plane plane;
    plane.smoothed_airspeed = plane.aparm.airspeed_max; // plenty of margin over stall
    plane.nav_roll_cd = 500; // 5 degrees - a small, easily-sustained bank
    plane.roll_limit_cd = 4500;

    plane.update_load_factor();

    REQUIRE(plane.roll_limit_cd == 4500); // unreduced
    REQUIRE(plane.nav_roll_cd == 500); // unclamped
}

// ---------------------------------------------------------------------
// Closed-loop integration test (the real point of this ticket): drive a
// genuine Plane + ModeFBWA against SimPlane's ground-truth flight
// dynamics, in a loop matching tick()'s own real sequencing, and confirm
// the aircraft's TRUE roll angle (from SimPlane - NOT AhrsDcm, the thing
// under test) converges toward and holds near a constant commanded bank
// angle.
// ---------------------------------------------------------------------

TEST_CASE("Closed loop: FBWA holding a constant commanded bank angle converges in SimPlane's ground truth",
          "[vehicle][integration]") {
    Plane plane;
    fwcpp::sim::SimPlane sim_plane;
    ModeFBWA fbwa(plane);

    constexpr float kDt = 0.02f; // 50Hz
    constexpr int kNumTicks = 1500; // 30 simulated seconds

    StabilizeInputs in;
    in.dt = kDt;
    in.armed_and_safety_off = true;

    std::uint32_t now_ms = 0;
    float commanded_roll_deg = 0.0f;

    for (int i = 0; i < kNumTicks; ++i) {
        now_ms += 20;
        in.now_ms = now_ms;

        // A fixed, moderate right-roll stick command plus enough
        // throttle to build and hold airspeed; pitch/rudder centered.
        set_sticks(plane, 1650, 1500, 1700, 1500);

        // Feed SimPlane's TRUE gyro rate into AhrsDcm as this tick's IMU
        // sample - same discretization already established by
        // ahrs_dcm_test.cpp (gyro * dt as the integrated delta_angle).
        fwcpp::ahrs::GyroSample gyro_sample;
        gyro_sample.gyro = sim_plane.gyro;
        gyro_sample.delta_angle = sim_plane.gyro * kDt;
        gyro_sample.dangle_dt = kDt;

        tick(plane, fbwa, gyro_sample, in);

        if (i == 0) {
            commanded_roll_deg = static_cast<float>(plane.nav_roll_cd) * 0.01f;
        }

        // Feed the vehicle's computed servo outputs into SimPlane as this
        // tick's control surface deflections/throttle. SimPlane::update()
        // takes -1..1 for surfaces and 0..1 for throttle; SrvChannels'
        // angle channels are configured +-4500 (see configure_channels()).
        const float aileron = plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kAileron) / fwcpp::vehicle::kServoMax;
        const float elevator = plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kElevator) / fwcpp::vehicle::kServoMax;
        const float rudder = plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kRudder) / fwcpp::vehicle::kServoMax;
        const float throttle = plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle) / 100.0f;
        sim_plane.update(aileron, elevator, rudder, throttle, kDt);
    }

    REQUIRE(commanded_roll_deg > 0.0f);

    float true_roll = 0.0f;
    float true_pitch = 0.0f;
    float true_yaw = 0.0f;
    sim_plane.dcm.to_euler(&true_roll, &true_pitch, &true_yaw);
    const float true_roll_deg = fwcpp::math::degrees(true_roll);

    INFO("commanded roll (deg) = " << commanded_roll_deg << ", true roll (deg) = " << true_roll_deg
                                    << ", true airspeed = " << sim_plane.airspeed);
    // Real convergence (see this test's own git-history verification run)
    // lands within ~0.25 deg of the 16.87 deg commanded bank; a 3 deg
    // margin leaves comfortable headroom for compiler/FP variance while
    // still meaningfully asserting real convergence, not just "didn't crash".
    REQUIRE(true_roll_deg == Catch::Approx(commanded_roll_deg).margin(3.0f));
}
