// Tests for fwcpp::motors::MotorsMatrix (CCP-001) - the core factor
// storage/arithmetic slice of AP_MotorsMatrix. See motors_matrix.hpp's own
// file banner for exactly what upstream behavior this reproduces and what
// is deferred (frame tables, output stage).

#include <cmath>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/motors/motors_matrix.hpp>

using namespace fwcpp::motors;
using Catch::Approx;

// ---------------------------------------------------------------------
// add_motor_raw: basic enable/store, out-of-range motor numbers, and the
// initialised_ok() guard.
// ---------------------------------------------------------------------

TEST_CASE("add_motor_raw: enables the motor and stores all four factors plus test order", "[motors][add_motor_raw]") {
    MotorsMatrix m;
    REQUIRE_FALSE(m.motor_enabled(0));

    m.add_motor_raw(0, 0.25f, -0.5f, 1.0f, 7, 0.75f);

    REQUIRE(m.motor_enabled(0));
    REQUIRE(m.roll_factor(0) == Approx(0.25f));
    REQUIRE(m.pitch_factor(0) == Approx(-0.5f));
    REQUIRE(m.yaw_factor(0) == Approx(1.0f));
    REQUIRE(m.throttle_factor(0) == Approx(0.75f));
    REQUIRE(m.test_order(0) == 7);
}

TEST_CASE("add_motor_raw: throttle_factor defaults to 1.0 when omitted", "[motors][add_motor_raw]") {
    MotorsMatrix m;
    m.add_motor_raw(3, 0.1f, 0.2f, 0.3f, 1);
    REQUIRE(m.throttle_factor(3) == Approx(1.0f));
}

TEST_CASE("add_motor_raw: out-of-range motor numbers are ignored, not crashes", "[motors][add_motor_raw]") {
    MotorsMatrix m;
    m.add_motor_raw(-1, 1.0f, 1.0f, 1.0f, 1);
    m.add_motor_raw(static_cast<std::int8_t>(kMaxNumMotors), 1.0f, 1.0f, 1.0f, 1);
    // No enabled motor exists anywhere in [0, kMaxNumMotors) as a result.
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("add_motor_raw: real max-motor-count boundary is 32 (kMaxNumMotors), not 12", "[motors][kMaxNumMotors]") {
    // AP_MOTORS_MAX_NUM_MOTORS resolves to 32 for this port's SITL target
    // (AP_SCRIPTING_ENABLED branch, unclamped since NUM_SERVO_CHANNELS is
    // also 32 here) - see motors_matrix.hpp's file banner for the full
    // investigation. copter-rust's own COP-005 notes record getting this
    // wrong at 12 first (reading only the #else branch); this asserts the
    // real resolved value directly rather than assuming either number.
    REQUIRE(kMaxNumMotors == 32);

    MotorsMatrix m;
    m.add_motor_raw(31, 1.0f, 1.0f, 1.0f, 1);
    REQUIRE(m.motor_enabled(31));
}

TEST_CASE("add_motor_raw: initialised_ok() guard makes it a no-op once set", "[motors][add_motor_raw][guard]") {
    MotorsMatrix m;
    m.add_motor_raw(0, 0.1f, 0.2f, 0.3f, 1);
    REQUIRE(m.motor_enabled(0));

    m.set_initialised_ok(true);
    // Attempting to add a NEW motor while "initialised" must do nothing -
    // matches upstream's real `if (initialised_ok()) return;` guard.
    m.add_motor_raw(1, 9.0f, 9.0f, 9.0f, 9);
    REQUIRE_FALSE(m.motor_enabled(1));

    // The already-set motor 0 must also be unmodifiable while initialised.
    m.add_motor_raw(0, -9.0f, -9.0f, -9.0f, 99);
    REQUIRE(m.roll_factor(0) == Approx(0.1f));
    REQUIRE(m.pitch_factor(0) == Approx(0.2f));
    REQUIRE(m.yaw_factor(0) == Approx(0.3f));
    REQUIRE(m.test_order(0) == 1);
}

// ---------------------------------------------------------------------
// add_motor: the real +90-on-roll-only asymmetry. Each test below hand-
// computes the roll/pitch factors from the real formula
// (roll = cos(radians(angle_degrees + 90)), pitch = cos(radians(angle_degrees)))
// and is built so a port that applies +90 to BOTH angles, or to NEITHER,
// produces a visibly different (and wrong) result - see the per-test
// comments for the three-way comparison.
// ---------------------------------------------------------------------

TEST_CASE("add_motor (3-arg): angle=0 distinguishes correct/both/neither +90 handling", "[motors][add_motor][angle]") {
    // angle_degrees = 0:
    //   correct: roll = cos(90deg) = 0, pitch = cos(0deg)  = 1
    //   wrong (both +90):    roll = 0,                 pitch = cos(90deg) = 0
    //   wrong (neither +90): roll = cos(0deg) = 1,      pitch = 1
    // All three outcomes are numerically distinct.
    MotorsMatrix m;
    m.add_motor(0, 0.0f, 1.0f, 1);
    REQUIRE(m.roll_factor(0) == Approx(0.0f).margin(1e-6));
    REQUIRE(m.pitch_factor(0) == Approx(1.0f).margin(1e-6));
}

TEST_CASE("add_motor (3-arg): angle=30 matches hand-computed trig, not the +90-on-both or +90-on-neither variants",
          "[motors][add_motor][angle]") {
    // angle_degrees = 30:
    //   correct: roll = cos(120deg) = -0.5,        pitch = cos(30deg) = 0.8660254
    //   wrong (both +90):    roll = cos(120deg) = -0.5,   pitch = cos(120deg) = -0.5
    //   wrong (neither +90): roll = cos(30deg) = 0.8660254, pitch = 0.8660254
    MotorsMatrix m;
    m.add_motor(0, 30.0f, 1.0f, 1);
    const float expected_roll = std::cos(fwcpp::math::radians(120.0f));
    const float expected_pitch = std::cos(fwcpp::math::radians(30.0f));
    REQUIRE(expected_roll == Approx(-0.5f).margin(1e-4));
    REQUIRE(expected_pitch == Approx(0.8660254f).margin(1e-4));
    REQUIRE(m.roll_factor(0) == Approx(expected_roll).margin(1e-4));
    REQUIRE(m.pitch_factor(0) == Approx(expected_pitch).margin(1e-4));
    // Explicitly confirm roll != pitch even though the 3-arg form was
    // called with a single shared angle - the two wrong variants above
    // would both produce roll == pitch here.
    REQUIRE(m.roll_factor(0) != Approx(m.pitch_factor(0)).margin(1e-4));
}

TEST_CASE("add_motor (4-arg): independent roll/pitch angles, still only roll gets +90", "[motors][add_motor][angle]") {
    // roll_factor_in_degrees = 45, pitch_factor_in_degrees = -60:
    //   roll  = cos(radians(45 + 90)) = cos(135deg) = -0.70710678
    //   pitch = cos(radians(-60))     = cos(-60deg) = 0.5
    MotorsMatrix m;
    m.add_motor(0, 45.0f, -60.0f, 1.0f, 1);
    const float expected_roll = std::cos(fwcpp::math::radians(135.0f));
    const float expected_pitch = std::cos(fwcpp::math::radians(-60.0f));
    REQUIRE(m.roll_factor(0) == Approx(expected_roll).margin(1e-4));
    REQUIRE(m.pitch_factor(0) == Approx(expected_pitch).margin(1e-4));
    REQUIRE(m.pitch_factor(0) == Approx(0.5f).margin(1e-4));
}

TEST_CASE("add_motor: yaw_factor and testing_order pass through unchanged", "[motors][add_motor]") {
    MotorsMatrix m;
    m.add_motor(2, 90.0f, -1.0f, 4);
    REQUIRE(m.yaw_factor(2) == Approx(-1.0f));
    REQUIRE(m.test_order(2) == 4);
}

// ---------------------------------------------------------------------
// remove_motor
// ---------------------------------------------------------------------

TEST_CASE("remove_motor: disables the motor and zeros all four factors", "[motors][remove_motor]") {
    MotorsMatrix m;
    m.add_motor_raw(5, 0.4f, -0.3f, 0.9f, 2, 0.6f);
    REQUIRE(m.motor_enabled(5));

    m.remove_motor(5);

    REQUIRE_FALSE(m.motor_enabled(5));
    REQUIRE(m.roll_factor(5) == Approx(0.0f));
    REQUIRE(m.pitch_factor(5) == Approx(0.0f));
    REQUIRE(m.yaw_factor(5) == Approx(0.0f));
    REQUIRE(m.throttle_factor(5) == Approx(0.0f));
}

TEST_CASE("remove_motor: out-of-range motor numbers are ignored", "[motors][remove_motor]") {
    MotorsMatrix m;
    m.remove_motor(-1);
    m.remove_motor(static_cast<std::int8_t>(kMaxNumMotors));
    // No crash, no state anywhere.
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

// ---------------------------------------------------------------------
// add_motors / add_motors_raw: index-order loops.
// ---------------------------------------------------------------------

TEST_CASE("add_motors: populates motors in array order using motor index as motor_num", "[motors][add_motors]") {
    MotorsMatrix m;
    const MotorsMatrix::MotorDef defs[] = {
        {90.0f, 1.0f, 2},
        {-90.0f, -1.0f, 4},
    };
    m.add_motors(defs, 2);

    REQUIRE(m.motor_enabled(0));
    REQUIRE(m.motor_enabled(1));
    REQUIRE(m.yaw_factor(0) == Approx(1.0f));
    REQUIRE(m.yaw_factor(1) == Approx(-1.0f));
    REQUIRE(m.test_order(0) == 2);
    REQUIRE(m.test_order(1) == 4);
    // Both defs used the symmetric add_motor path, so roll != pitch here
    // too (angle=90: roll=cos(180)=-1, pitch=cos(90)=0).
    REQUIRE(m.roll_factor(0) == Approx(-1.0f).margin(1e-4));
    REQUIRE(m.pitch_factor(0) == Approx(0.0f).margin(1e-4));
}

TEST_CASE("add_motors_raw: populates motors in array order with raw factors, no angle conversion", "[motors][add_motors_raw]") {
    MotorsMatrix m;
    const MotorsMatrix::MotorDefRaw defs[] = {
        {0.5f, 0.25f, 1.0f, 1},
        {-0.5f, -0.25f, -1.0f, 2},
    };
    m.add_motors_raw(defs, 2);

    REQUIRE(m.roll_factor(0) == Approx(0.5f));
    REQUIRE(m.pitch_factor(0) == Approx(0.25f));
    REQUIRE(m.roll_factor(1) == Approx(-0.5f));
    REQUIRE(m.pitch_factor(1) == Approx(-0.25f));
}

// ---------------------------------------------------------------------
// normalise_rpy_factors: two-pass max-then-rescale, independent per-axis
// scaling, the zero-range divide-by-zero guard, and the asymmetric
// throttle-floors-at-zero-but-rpy-does-not clamping.
// ---------------------------------------------------------------------

TEST_CASE("normalise_rpy_factors: each axis normalizes independently against its OWN max, not a combined one",
          "[motors][normalise]") {
    // motor 0: much larger yaw authority than roll/pitch.
    // motor 1: smaller-magnitude, opposite-signed factors on every axis.
    MotorsMatrix m;
    m.add_motor_raw(0, 0.5f, 0.25f, 2.0f, 1, 1.0f);
    m.add_motor_raw(1, -0.25f, -0.5f, -1.0f, 2, 0.5f);

    m.normalise_rpy_factors();

    // roll max = 0.5 -> motor0 roll unchanged in magnitude: 0.5*0.5/0.5 = 0.5
    REQUIRE(m.roll_factor(0) == Approx(0.5f).margin(1e-4));
    REQUIRE(m.roll_factor(1) == Approx(-0.25f).margin(1e-4));
    // pitch max = 0.5 -> motor0 pitch: 0.5*0.25/0.5 = 0.25
    REQUIRE(m.pitch_factor(0) == Approx(0.25f).margin(1e-4));
    REQUIRE(m.pitch_factor(1) == Approx(-0.5f).margin(1e-4));
    // yaw max = 2.0 -> motor0 yaw HALVED from 2.0 to 0.5*2.0/2.0 = 0.5,
    // proving yaw scaled against its OWN max (2.0), not roll/pitch's (0.5).
    REQUIRE(m.yaw_factor(0) == Approx(0.5f).margin(1e-4));
    REQUIRE(m.yaw_factor(1) == Approx(-0.25f).margin(1e-4));
    // throttle max = 1.0 (both already non-negative).
    REQUIRE(m.throttle_factor(0) == Approx(1.0f).margin(1e-4));
    REQUIRE(m.throttle_factor(1) == Approx(0.5f).margin(1e-4));
}

TEST_CASE("normalise_rpy_factors: an axis where every enabled motor's factor is exactly zero does not divide by zero",
          "[motors][normalise][zero-guard]") {
    MotorsMatrix m;
    // pitch_fac is 0 for every motor - pitch's max is exactly 0.0f, so
    // the is_zero(pitch_fac) guard must skip rescaling that axis (leaving
    // it at 0, never NaN/inf from a 0/0 division).
    m.add_motor_raw(0, 0.5f, 0.0f, 1.0f, 1, 1.0f);
    m.add_motor_raw(1, -0.5f, 0.0f, -1.0f, 2, 1.0f);

    m.normalise_rpy_factors();

    REQUIRE(m.pitch_factor(0) == Approx(0.0f));
    REQUIRE(m.pitch_factor(1) == Approx(0.0f));
    REQUIRE_FALSE(std::isnan(m.pitch_factor(0)));
    REQUIRE_FALSE(std::isnan(m.pitch_factor(1)));
    // roll/yaw still normalize normally, confirming the guard is
    // per-axis, not an early-return over the whole motor.
    REQUIRE(m.roll_factor(0) == Approx(0.5f).margin(1e-4));
    REQUIRE(m.yaw_factor(0) == Approx(0.5f).margin(1e-4));
}

TEST_CASE("normalise_rpy_factors: throttle floors at zero but roll/pitch/yaw legitimately go negative",
          "[motors][normalise][clamp]") {
    MotorsMatrix m;
    // motor 0: positive throttle, sets the (only) positive throttle max.
    // motor 1: NEGATIVE throttle factor and a negative-dominant roll.
    m.add_motor_raw(0, 0.6f, 0.0f, 0.0f, 1, 1.0f);
    m.add_motor_raw(1, -0.3f, 0.0f, 0.0f, 2, -2.0f);

    m.normalise_rpy_factors();

    // roll max = 0.6 -> motor1 roll = 0.5 * -0.3 / 0.6 = -0.25, a real
    // negative result that must NOT be floored.
    REQUIRE(m.roll_factor(1) == Approx(-0.25f).margin(1e-4));
    // throttle max = MAX(0, MAX(1.0, -2.0-clamped-to-0)) = 1.0. motor1's
    // rescaled throttle = MAX(0, -2.0/1.0) = MAX(0, -2.0) = 0.0 - floored,
    // unlike the equally-negative-going roll factor above.
    REQUIRE(m.throttle_factor(1) == Approx(0.0f).margin(1e-6));
    REQUIRE(m.throttle_factor(0) == Approx(1.0f).margin(1e-4));
}

TEST_CASE("normalise_rpy_factors: iterates all kMaxNumMotors slots safely and leaves never-added motors disabled",
          "[motors][normalise]") {
    // Note: remove_motor() (like upstream) always zeros a motor's factors
    // together with disabling it, so a "disabled but non-zero factor"
    // state is not reachable through this class's public API - matching
    // upstream, where the same is true (nothing else can flip
    // motor_enabled[i] false without also zeroing the factors). This test
    // therefore checks what IS observable: normalise_rpy_factors runs
    // safely across the full kMaxNumMotors range (no out-of-bounds
    // access, no crash under ASan) and never-touched slots stay disabled
    // with zeroed factors, rather than being spuriously enabled.
    MotorsMatrix m;
    m.add_motor_raw(0, 0.2f, 0.3f, 0.4f, 1, 1.0f);

    m.normalise_rpy_factors();

    for (std::uint8_t i = 1; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
        REQUIRE(m.roll_factor(i) == Approx(0.0f));
        REQUIRE(m.pitch_factor(i) == Approx(0.0f));
        REQUIRE(m.yaw_factor(i) == Approx(0.0f));
        REQUIRE(m.throttle_factor(i) == Approx(0.0f));
    }
    // Motor 0 itself normalizes to 0.5 on every axis since it is the only
    // enabled motor (its own factor is trivially the max on every axis).
    REQUIRE(m.roll_factor(0) == Approx(0.5f).margin(1e-4));
    REQUIRE(m.pitch_factor(0) == Approx(0.5f).margin(1e-4));
    REQUIRE(m.yaw_factor(0) == Approx(0.5f).margin(1e-4));
}
