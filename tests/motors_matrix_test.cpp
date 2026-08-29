// Tests for fwcpp::motors::MotorsMatrix (CCP-001) - the core factor
// storage/arithmetic slice of AP_MotorsMatrix. See motors_matrix.hpp's own
// file banner for exactly what upstream behavior this reproduces and what
// is deferred (frame tables, output stage).

#include <array>
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

// =======================================================================
// CCP-002: setup_quad_matrix - every real in-scope frame type, checked
// against hand-computed values from the real upstream angle/factor
// inputs (AP_MotorsMatrix.cpp lines 576-772). Angle-based frames use the
// same cos(radians(angle+90))/cos(radians(angle)) formula CCP-001's own
// add_motor tests above already exercise directly; V and Y4 use direct
// value comparison since their factors are not angle-derived.
// =======================================================================

namespace {

// One angle-based motor definition, in the SAME array order
// setup_quad_matrix's own static MotorDef tables use (array index becomes
// motor_num via add_motors' index-order loop).
struct AngleMotor {
    float angle_degrees;
    float yaw_factor;
    std::uint8_t test_order;
};

// Hand-computes the expected roll/pitch/yaw/test_order for each motor of
// an angle-based frame and checks it against the real MotorsMatrix state -
// reproducing add_motor's own real formula (roll = cos(radians(angle+90)),
// pitch = cos(radians(angle))) independently of motors_matrix.hpp's own
// implementation, per the ticket's explicit test requirement.
void checkAngleFrame(const MotorsMatrix& m, const AngleMotor* motors, std::uint8_t n) {
    for (std::uint8_t i = 0; i < n; ++i) {
        const float expected_roll = std::cos(fwcpp::math::radians(motors[i].angle_degrees + 90.0f));
        const float expected_pitch = std::cos(fwcpp::math::radians(motors[i].angle_degrees));
        INFO("motor index " << static_cast<int>(i) << " angle " << motors[i].angle_degrees);
        REQUIRE(m.motor_enabled(i));
        REQUIRE(m.roll_factor(i) == Approx(expected_roll).margin(1e-4));
        REQUIRE(m.pitch_factor(i) == Approx(expected_pitch).margin(1e-4));
        REQUIRE(m.yaw_factor(i) == Approx(motors[i].yaw_factor).margin(1e-6));
        REQUIRE(m.test_order(i) == motors[i].test_order);
    }
}

} // namespace

TEST_CASE("setup_quad_matrix: PLUS matches hand-computed angle/yaw/test_order", "[motors][setup_quad_matrix][plus]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(m.frame_class_string() == "QUAD");
    REQUIRE(m.frame_type_string() == "PLUS");
    const AngleMotor expected[] = {
        {90.0f, kYawFactorCcw, 2},
        {-90.0f, kYawFactorCcw, 4},
        {0.0f, kYawFactorCw, 1},
        {180.0f, kYawFactorCw, 3},
    };
    checkAngleFrame(m, expected, 4);
}

TEST_CASE("setup_quad_matrix: X matches hand-computed angle/yaw/test_order", "[motors][setup_quad_matrix][x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::X));
    REQUIRE(m.frame_type_string() == "X");
    const AngleMotor expected[] = {
        {45.0f, kYawFactorCcw, 1},
        {-135.0f, kYawFactorCcw, 3},
        {-45.0f, kYawFactorCw, 4},
        {135.0f, kYawFactorCw, 2},
    };
    checkAngleFrame(m, expected, 4);
}

TEST_CASE("setup_quad_matrix: BF_X (betaflight order) matches hand-computed values", "[motors][setup_quad_matrix][bf_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::BfX));
    REQUIRE(m.frame_type_string() == "BF_X");
    const AngleMotor expected[] = {
        {135.0f, kYawFactorCw, 2},
        {45.0f, kYawFactorCcw, 1},
        {-135.0f, kYawFactorCcw, 3},
        {-45.0f, kYawFactorCw, 4},
    };
    checkAngleFrame(m, expected, 4);
}

TEST_CASE("setup_quad_matrix: BF_X_REV (betaflight order, reversed) matches hand-computed values",
          "[motors][setup_quad_matrix][bf_x_rev]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::BfXRev));
    REQUIRE(m.frame_type_string() == "X_REV");
    const AngleMotor expected[] = {
        {135.0f, kYawFactorCcw, 2},
        {45.0f, kYawFactorCw, 1},
        {-135.0f, kYawFactorCw, 3},
        {-45.0f, kYawFactorCcw, 4},
    };
    checkAngleFrame(m, expected, 4);
}

TEST_CASE("setup_quad_matrix: BF_X and BF_X_REV have every yaw factor mutually negated",
          "[motors][setup_quad_matrix][bf_x][bf_x_rev]") {
    // Same angles/test_order, every yaw factor flipped - confirmed
    // directly rather than assumed from the "reversed motors" name.
    MotorsMatrix bf_x;
    MotorsMatrix bf_x_rev;
    REQUIRE(bf_x.setup_quad_matrix(MotorsMatrix::FrameType::BfX));
    REQUIRE(bf_x_rev.setup_quad_matrix(MotorsMatrix::FrameType::BfXRev));
    for (std::uint8_t i = 0; i < 4; ++i) {
        REQUIRE(bf_x.roll_factor(i) == Approx(bf_x_rev.roll_factor(i)).margin(1e-4));
        REQUIRE(bf_x.pitch_factor(i) == Approx(bf_x_rev.pitch_factor(i)).margin(1e-4));
        REQUIRE(bf_x.test_order(i) == bf_x_rev.test_order(i));
        REQUIRE(bf_x.yaw_factor(i) == Approx(-bf_x_rev.yaw_factor(i)).margin(1e-6));
    }
}

TEST_CASE("setup_quad_matrix: DJI_X matches hand-computed angle/yaw/test_order", "[motors][setup_quad_matrix][dji_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::DjiX));
    REQUIRE(m.frame_type_string() == "DJI_X");
    const AngleMotor expected[] = {
        {45.0f, kYawFactorCcw, 1},
        {-45.0f, kYawFactorCw, 4},
        {-135.0f, kYawFactorCcw, 3},
        {135.0f, kYawFactorCw, 2},
    };
    checkAngleFrame(m, expected, 4);
}

TEST_CASE("setup_quad_matrix: CW_X matches hand-computed angle/yaw/test_order", "[motors][setup_quad_matrix][cw_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::CwX));
    REQUIRE(m.frame_type_string() == "CW_X");
    const AngleMotor expected[] = {
        {45.0f, kYawFactorCcw, 1},
        {135.0f, kYawFactorCw, 2},
        {-135.0f, kYawFactorCcw, 3},
        {-45.0f, kYawFactorCw, 4},
    };
    checkAngleFrame(m, expected, 4);
}

TEST_CASE("setup_quad_matrix: H matches hand-computed angle/yaw/test_order (same angles as X, opposite spin)",
          "[motors][setup_quad_matrix][h]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::H));
    REQUIRE(m.frame_type_string() == "H");
    const AngleMotor expected[] = {
        {45.0f, kYawFactorCw, 1},
        {-135.0f, kYawFactorCw, 3},
        {-45.0f, kYawFactorCcw, 4},
        {135.0f, kYawFactorCcw, 2},
    };
    checkAngleFrame(m, expected, 4);
    // H uses the SAME angles as X (so same roll/pitch), but every yaw
    // factor is the opposite sign - confirmed directly, not assumed.
    MotorsMatrix x;
    REQUIRE(x.setup_quad_matrix(MotorsMatrix::FrameType::X));
    for (std::uint8_t i = 0; i < 4; ++i) {
        REQUIRE(m.roll_factor(i) == Approx(x.roll_factor(i)).margin(1e-4));
        REQUIRE(m.pitch_factor(i) == Approx(x.pitch_factor(i)).margin(1e-4));
        REQUIRE(m.yaw_factor(i) == Approx(-x.yaw_factor(i)).margin(1e-6));
    }
}

TEST_CASE("setup_quad_matrix: V uses the real non-angle-derived yaw factors exactly (0.7981/1.0/-0.7981/-1.0)",
          "[motors][setup_quad_matrix][v]") {
    // V's yaw factors are transcribed directly from upstream's own float
    // literals, not derived from a cos/sin formula - so this test checks
    // exact literal values, and separately confirms roll/pitch still come
    // from the ordinary angle formula (V's angles ARE ordinary).
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::V));
    REQUIRE(m.frame_type_string() == "V");

    REQUIRE(m.yaw_factor(0) == Approx(0.7981f).margin(1e-6));
    REQUIRE(m.yaw_factor(1) == Approx(1.0000f).margin(1e-6));
    REQUIRE(m.yaw_factor(2) == Approx(-0.7981f).margin(1e-6));
    REQUIRE(m.yaw_factor(3) == Approx(-1.0000f).margin(1e-6));

    const float angles[] = {45.0f, -135.0f, -45.0f, 135.0f};
    const std::uint8_t test_orders[] = {1, 3, 4, 2};
    for (std::uint8_t i = 0; i < 4; ++i) {
        REQUIRE(m.roll_factor(i) == Approx(std::cos(fwcpp::math::radians(angles[i] + 90.0f))).margin(1e-4));
        REQUIRE(m.pitch_factor(i) == Approx(std::cos(fwcpp::math::radians(angles[i]))).margin(1e-4));
        REQUIRE(m.test_order(i) == test_orders[i]);
    }
}

TEST_CASE("setup_quad_matrix: VTAIL sets roll/pitch/yaw per motor from direct add_motor calls",
          "[motors][setup_quad_matrix][vtail]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::VTail));
    REQUIRE(m.frame_type_string() == "VTAIL");

    // add_motor(0, 60, 60, 0, 1)
    REQUIRE(m.roll_factor(0) == Approx(std::cos(fwcpp::math::radians(60.0f + 90.0f))).margin(1e-4));
    REQUIRE(m.pitch_factor(0) == Approx(std::cos(fwcpp::math::radians(60.0f))).margin(1e-4));
    REQUIRE(m.yaw_factor(0) == Approx(0.0f).margin(1e-6));
    REQUIRE(m.test_order(0) == 1);

    // add_motor(1, 0, -160, CW, 3) - no roll in rear motors (roll angle 0
    // fed through the SAME +90-on-roll-only formula as every other motor).
    REQUIRE(m.roll_factor(1) == Approx(std::cos(fwcpp::math::radians(0.0f + 90.0f))).margin(1e-4));
    REQUIRE(m.pitch_factor(1) == Approx(std::cos(fwcpp::math::radians(-160.0f))).margin(1e-4));
    REQUIRE(m.yaw_factor(1) == Approx(kYawFactorCw).margin(1e-6));
    REQUIRE(m.test_order(1) == 3);

    // add_motor(2, -60, -60, 0, 4)
    REQUIRE(m.roll_factor(2) == Approx(std::cos(fwcpp::math::radians(-60.0f + 90.0f))).margin(1e-4));
    REQUIRE(m.pitch_factor(2) == Approx(std::cos(fwcpp::math::radians(-60.0f))).margin(1e-4));
    REQUIRE(m.yaw_factor(2) == Approx(0.0f).margin(1e-6));
    REQUIRE(m.test_order(2) == 4);

    // add_motor(3, 0, 160, CCW, 2)
    REQUIRE(m.roll_factor(3) == Approx(std::cos(fwcpp::math::radians(0.0f + 90.0f))).margin(1e-4));
    REQUIRE(m.pitch_factor(3) == Approx(std::cos(fwcpp::math::radians(160.0f))).margin(1e-4));
    REQUIRE(m.yaw_factor(3) == Approx(kYawFactorCcw).margin(1e-6));
    REQUIRE(m.test_order(3) == 2);
}

TEST_CASE("setup_quad_matrix: ATAIL vs VTAIL - the ONE real difference is motors 1 and 3's yaw sign",
          "[motors][setup_quad_matrix][atail][vtail]") {
    // This is exactly the "looks like a duplicate but isn't" pair the
    // ticket calls out: a careless port could implement ATAIL as a copy
    // of VTAIL (or vice versa) since every roll/pitch/test_order value is
    // identical between them. This test would fail immediately if that
    // happened, since it requires motors 1 and 3's yaw factors to be
    // OPPOSITE between the two frames while motors 0 and 2 stay equal
    // (both zero - "no yaw in front motors" holds for both).
    MotorsMatrix vtail;
    MotorsMatrix atail;
    REQUIRE(vtail.setup_quad_matrix(MotorsMatrix::FrameType::VTail));
    REQUIRE(atail.setup_quad_matrix(MotorsMatrix::FrameType::ATail));
    REQUIRE(vtail.frame_type_string() == "VTAIL");
    REQUIRE(atail.frame_type_string() == "ATAIL");

    for (std::uint8_t i = 0; i < 4; ++i) {
        // roll/pitch/test_order are pairwise IDENTICAL - same physical
        // motor geometry.
        REQUIRE(vtail.roll_factor(i) == Approx(atail.roll_factor(i)).margin(1e-6));
        REQUIRE(vtail.pitch_factor(i) == Approx(atail.pitch_factor(i)).margin(1e-6));
        REQUIRE(vtail.test_order(i) == atail.test_order(i));
    }

    // Front motors (0, 2): both zero, equal in both frames.
    REQUIRE(vtail.yaw_factor(0) == Approx(0.0f).margin(1e-6));
    REQUIRE(atail.yaw_factor(0) == Approx(0.0f).margin(1e-6));
    REQUIRE(vtail.yaw_factor(2) == Approx(0.0f).margin(1e-6));
    REQUIRE(atail.yaw_factor(2) == Approx(0.0f).margin(1e-6));

    // Rear motors (1, 3): REVERSED between the two frames - the one real
    // asymmetry this pair has.
    REQUIRE(vtail.yaw_factor(1) == Approx(kYawFactorCw).margin(1e-6));
    REQUIRE(atail.yaw_factor(1) == Approx(kYawFactorCcw).margin(1e-6));
    REQUIRE(vtail.yaw_factor(3) == Approx(kYawFactorCcw).margin(1e-6));
    REQUIRE(atail.yaw_factor(3) == Approx(kYawFactorCw).margin(1e-6));

    // Explicitly: they are NOT identical frames.
    REQUIRE(vtail.yaw_factor(1) != Approx(atail.yaw_factor(1)).margin(1e-6));
    REQUIRE(vtail.yaw_factor(3) != Approx(atail.yaw_factor(3)).margin(1e-6));
}

TEST_CASE("setup_quad_matrix: PLUSREV matches hand-computed angle/yaw/test_order", "[motors][setup_quad_matrix][plusrev]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::PlusRev));
    REQUIRE(m.frame_type_string() == "PLUSREV");
    const AngleMotor expected[] = {
        {90.0f, kYawFactorCw, 2},
        {-90.0f, kYawFactorCw, 4},
        {0.0f, kYawFactorCcw, 1},
        {180.0f, kYawFactorCcw, 3},
    };
    checkAngleFrame(m, expected, 4);
}

TEST_CASE("setup_quad_matrix: PLUSREV's yaw factors are the EXACT sign-negation of PLUS's own, not a duplicate",
          "[motors][setup_quad_matrix][plus][plusrev]") {
    // The ticket's other explicit "looks like a duplicate but isn't" pair:
    // a careless port could implement PLUSREV as literally PLUS's own
    // MotorDef table copy-pasted. This test requires PLUSREV's yaw at
    // every motor index to be the exact negation of PLUS's own, with
    // roll/pitch/test_order identical - which a copy-paste would satisfy
    // for roll/pitch/test_order but FAIL for the yaw negation (since a
    // pure copy would leave yaw equal, not negated).
    MotorsMatrix plus;
    MotorsMatrix plus_rev;
    REQUIRE(plus.setup_quad_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(plus_rev.setup_quad_matrix(MotorsMatrix::FrameType::PlusRev));

    for (std::uint8_t i = 0; i < 4; ++i) {
        REQUIRE(plus.roll_factor(i) == Approx(plus_rev.roll_factor(i)).margin(1e-6));
        REQUIRE(plus.pitch_factor(i) == Approx(plus_rev.pitch_factor(i)).margin(1e-6));
        REQUIRE(plus.test_order(i) == plus_rev.test_order(i));
        REQUIRE(plus.yaw_factor(i) == Approx(-plus_rev.yaw_factor(i)).margin(1e-6));
        // And explicitly not equal (CW/CCW are nonzero, so equal-to-negation
        // would require yaw_factor(i) == 0, which none of these are).
        REQUIRE(plus.yaw_factor(i) != Approx(plus_rev.yaw_factor(i)).margin(1e-6));
    }
}

TEST_CASE("setup_quad_matrix: Y4 raw factors match exactly, no angle conversion involved",
          "[motors][setup_quad_matrix][y4]") {
    MotorsMatrix m;
    REQUIRE(m.setup_quad_matrix(MotorsMatrix::FrameType::Y4));
    REQUIRE(m.frame_type_string() == "Y4");

    // Direct value comparison per the ticket's own instruction for Y4 -
    // these are raw MotorDefRaw entries, not angle-derived.
    REQUIRE(m.roll_factor(0) == Approx(-1.0f));
    REQUIRE(m.pitch_factor(0) == Approx(1.0f));
    REQUIRE(m.yaw_factor(0) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(0) == 1);

    REQUIRE(m.roll_factor(1) == Approx(0.0f));
    REQUIRE(m.pitch_factor(1) == Approx(-1.0f));
    REQUIRE(m.yaw_factor(1) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(1) == 2);

    REQUIRE(m.roll_factor(2) == Approx(0.0f));
    REQUIRE(m.pitch_factor(2) == Approx(-1.0f));
    REQUIRE(m.yaw_factor(2) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(2) == 3);

    REQUIRE(m.roll_factor(3) == Approx(1.0f));
    REQUIRE(m.pitch_factor(3) == Approx(1.0f));
    REQUIRE(m.yaw_factor(3) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(3) == 4);
}

TEST_CASE("setup_quad_matrix: returns true for every real in-scope frame type and sets frame_class_string to QUAD",
          "[motors][setup_quad_matrix]") {
    const MotorsMatrix::FrameType all_types[] = {
        MotorsMatrix::FrameType::Plus,   MotorsMatrix::FrameType::X,       MotorsMatrix::FrameType::BfX,
        MotorsMatrix::FrameType::BfXRev, MotorsMatrix::FrameType::DjiX,    MotorsMatrix::FrameType::CwX,
        MotorsMatrix::FrameType::V,      MotorsMatrix::FrameType::H,      MotorsMatrix::FrameType::VTail,
        MotorsMatrix::FrameType::ATail,  MotorsMatrix::FrameType::PlusRev, MotorsMatrix::FrameType::Y4,
    };
    for (const auto ft : all_types) {
        MotorsMatrix m;
        REQUIRE(m.setup_quad_matrix(ft));
        REQUIRE(m.frame_class_string() == "QUAD");
        // Every real frame type populates all four motor slots 0-3.
        for (std::uint8_t i = 0; i < 4; ++i) {
            REQUIRE(m.motor_enabled(i));
        }
    }
}

TEST_CASE("setup_quad_matrix: an out-of-range frame type hits the real SIMPLE default branch and returns false",
          "[motors][setup_quad_matrix][default]") {
    // Confirms the real upstream default case ("quad frame class does not
    // support this frame type; return false;") - the simple kind, not
    // setup_y6_matrix's own productive default (a separate, later-ticket
    // concern). This port's own FrameType enum only ever names real,
    // handled values, so an unsupported value must be reached via an
    // explicit out-of-enum-range cast, matching how upstream would reach
    // its own default from a frame type belonging to another frame class.
    MotorsMatrix m;
    REQUIRE_FALSE(m.setup_quad_matrix(static_cast<MotorsMatrix::FrameType>(255)));
    // No motor should have been enabled - the default branch does
    // nothing before returning false.
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

// =======================================================================
// CCP-003: setup_hexa_matrix - every real in-scope frame type, checked
// against hand-computed values from the real upstream angle/factor
// inputs (AP_MotorsMatrix.cpp lines 775-851). Angle-based frames reuse
// checkAngleFrame() above (same real add_motor formula); H uses raw
// (roll_fac, pitch_fac) pairs, checked by direct value comparison the
// same way setup_quad_matrix's own Y4 test above checks its raw factors.
// All five of hexa's real frame types (PLUS/X/H/DJI_X/CW_X) reuse
// FrameType enumerators setup_quad_matrix's own tests above already
// exercise - see motors_matrix.hpp's file banner "CCP-003 ADDITION" for
// the enum-sharing investigation this reflects.
// =======================================================================

TEST_CASE("setup_hexa_matrix: PLUS matches hand-computed angle/yaw/test_order", "[motors][setup_hexa_matrix][plus]") {
    MotorsMatrix m;
    REQUIRE(m.setup_hexa_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(m.frame_class_string() == "HEXA");
    REQUIRE(m.frame_type_string() == "PLUS");
    const AngleMotor expected[] = {
        {0.0f, kYawFactorCw, 1},   {180.0f, kYawFactorCcw, 4}, {-120.0f, kYawFactorCw, 5},
        {60.0f, kYawFactorCcw, 2}, {-60.0f, kYawFactorCcw, 6}, {120.0f, kYawFactorCw, 3},
    };
    checkAngleFrame(m, expected, 6);
}

TEST_CASE("setup_hexa_matrix: X matches hand-computed angle/yaw/test_order", "[motors][setup_hexa_matrix][x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_hexa_matrix(MotorsMatrix::FrameType::X));
    REQUIRE(m.frame_type_string() == "X");
    const AngleMotor expected[] = {
        {90.0f, kYawFactorCw, 2},  {-90.0f, kYawFactorCcw, 5}, {-30.0f, kYawFactorCw, 6},
        {150.0f, kYawFactorCcw, 3}, {30.0f, kYawFactorCcw, 1}, {-150.0f, kYawFactorCw, 4},
    };
    checkAngleFrame(m, expected, 6);
}

TEST_CASE("setup_hexa_matrix: H uses real explicit raw roll/pitch pairs, not angle degrees, and matches exactly",
          "[motors][setup_hexa_matrix][h]") {
    // Upstream's own comment: "H is same as X except middle motors are
    // closer to center" - confirmed directly this is add_motors_raw with
    // real (roll_fac, pitch_fac) pairs, unlike quad's own angle-based H.
    MotorsMatrix m;
    REQUIRE(m.setup_hexa_matrix(MotorsMatrix::FrameType::H));
    REQUIRE(m.frame_type_string() == "H");

    struct RawMotor {
        float roll_fac;
        float pitch_fac;
        float yaw_fac;
        std::uint8_t test_order;
    };
    const RawMotor expected[] = {
        {-1.0f, 0.0f, kYawFactorCw, 2},  {1.0f, 0.0f, kYawFactorCcw, 5}, {1.0f, 1.0f, kYawFactorCw, 6},
        {-1.0f, -1.0f, kYawFactorCcw, 3}, {-1.0f, 1.0f, kYawFactorCcw, 1}, {1.0f, -1.0f, kYawFactorCw, 4},
    };
    for (std::uint8_t i = 0; i < 6; ++i) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(m.motor_enabled(i));
        REQUIRE(m.roll_factor(i) == Approx(expected[i].roll_fac));
        REQUIRE(m.pitch_factor(i) == Approx(expected[i].pitch_fac));
        REQUIRE(m.yaw_factor(i) == Approx(expected[i].yaw_fac));
        REQUIRE(m.test_order(i) == expected[i].test_order);
    }
}

TEST_CASE("setup_hexa_matrix: DJI_X matches hand-computed angle/yaw/test_order", "[motors][setup_hexa_matrix][dji_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_hexa_matrix(MotorsMatrix::FrameType::DjiX));
    REQUIRE(m.frame_type_string() == "DJI_X");
    const AngleMotor expected[] = {
        {30.0f, kYawFactorCcw, 1},  {-30.0f, kYawFactorCw, 6}, {-90.0f, kYawFactorCcw, 5},
        {-150.0f, kYawFactorCw, 4}, {150.0f, kYawFactorCcw, 3}, {90.0f, kYawFactorCw, 2},
    };
    checkAngleFrame(m, expected, 6);
}

TEST_CASE("setup_hexa_matrix: CW_X matches hand-computed angle/yaw/test_order", "[motors][setup_hexa_matrix][cw_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_hexa_matrix(MotorsMatrix::FrameType::CwX));
    REQUIRE(m.frame_type_string() == "CW_X");
    const AngleMotor expected[] = {
        {30.0f, kYawFactorCcw, 1}, {90.0f, kYawFactorCw, 2},   {150.0f, kYawFactorCcw, 3},
        {-150.0f, kYawFactorCw, 4}, {-90.0f, kYawFactorCcw, 5}, {-30.0f, kYawFactorCw, 6},
    };
    checkAngleFrame(m, expected, 6);
}

TEST_CASE("setup_hexa_matrix: returns true for every real in-scope frame type and sets frame_class_string to HEXA",
          "[motors][setup_hexa_matrix]") {
    const MotorsMatrix::FrameType all_types[] = {
        MotorsMatrix::FrameType::Plus, MotorsMatrix::FrameType::X,    MotorsMatrix::FrameType::H,
        MotorsMatrix::FrameType::DjiX, MotorsMatrix::FrameType::CwX,
    };
    for (const auto ft : all_types) {
        MotorsMatrix m;
        REQUIRE(m.setup_hexa_matrix(ft));
        REQUIRE(m.frame_class_string() == "HEXA");
        // Every real frame type populates all six motor slots 0-5 (hexa
        // has six motors, unlike setup_quad_matrix's own four).
        for (std::uint8_t i = 0; i < 6; ++i) {
            REQUIRE(m.motor_enabled(i));
        }
    }
}

TEST_CASE("setup_hexa_matrix: an out-of-range frame type hits the real SIMPLE default branch and returns false",
          "[motors][setup_hexa_matrix][default]") {
    // Confirms the real upstream default case ("hexa frame class does not
    // support this frame type; return false;") - the simple kind, same
    // as setup_quad_matrix's own, not setup_y6_matrix's own productive
    // default (a separate, later-ticket concern per COP-005).
    MotorsMatrix m;
    REQUIRE_FALSE(m.setup_hexa_matrix(static_cast<MotorsMatrix::FrameType>(255)));
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_hexa_matrix: PLUS reuses the SAME FrameType enumerator as setup_quad_matrix's own PLUS, "
          "but yields hexa-specific values on a six-motor table",
          "[motors][setup_hexa_matrix][setup_quad_matrix][plus]") {
    // Direct evidence for the file banner's enum-sharing investigation:
    // MotorsMatrix::FrameType::Plus is the literal same C++ enumerator
    // passed to both functions (mirroring upstream's real single shared
    // motor_frame_type enum), yet each function's own frame table
    // produces its own real, different numeric results - dispatch is by
    // which FUNCTION is called (i.e. which real motor_frame_class the
    // caller selected), not by a hexa-specific enumerator value.
    MotorsMatrix quad;
    MotorsMatrix hexa;
    REQUIRE(quad.setup_quad_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(hexa.setup_hexa_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(quad.frame_class_string() == "QUAD");
    REQUIRE(hexa.frame_class_string() == "HEXA");
    // Quad's PLUS motor 0 sits at 90 degrees (yaw CCW); hexa's PLUS
    // motor 0 sits at 0 degrees (yaw CW) - genuinely different real
    // tables behind the same enumerator.
    REQUIRE(quad.roll_factor(0) != Approx(hexa.roll_factor(0)));
    REQUIRE(quad.yaw_factor(0) == Approx(kYawFactorCcw));
    REQUIRE(hexa.yaw_factor(0) == Approx(kYawFactorCw));
    // Hexa populates two motor slots quad never touches.
    REQUIRE_FALSE(quad.motor_enabled(4));
    REQUIRE_FALSE(quad.motor_enabled(5));
    REQUIRE(hexa.motor_enabled(4));
    REQUIRE(hexa.motor_enabled(5));
}

// =======================================================================
// CCP-004: setup_octa_matrix - every real in-scope frame type, checked
// against hand-computed values from the real upstream angle/factor
// inputs (AP_MotorsMatrix.cpp lines 854-970). Angle-based frames reuse
// checkAngleFrame() above; V/H/I use real explicit raw (roll_fac,
// pitch_fac) pairs, checked via checkRawFrame() below. V's own non-round
// factors and H's/I's own +-0.333f entries are exactly the values this
// ticket's own acceptance criteria calls out as needing exact
// transcription, not rounding - checkRawFrame() uses plain Approx() with
// no .margin() override (matching setup_quad_matrix's own Y4 test and
// setup_hexa_matrix's own H test above), so Catch2's default ~0.1%
// relative epsilon is tight enough to fail on a rounded/approximated
// value like 0.33 in place of the real 0.333. Six of the seven real
// frame types (PLUS/X/V/H/DJI_X/CW_X) reuse FrameType enumerators
// setup_quad_matrix's own tests above already exercise; `I` is the one
// genuinely new enumerator this ticket adds - see motors_matrix.hpp's
// file banner "CCP-004 ADDITION" for the enum investigation this
// reflects.
// =======================================================================

namespace {

// One raw-factor motor definition, in the SAME array order
// setup_octa_matrix's own static MotorDefRaw tables use for V/H/I.
struct RawMotorDef {
    float roll_fac;
    float pitch_fac;
    float yaw_fac;
    std::uint8_t test_order;
};

// Hand-checks a raw-factor frame's motors against exact expected values,
// independently of motors_matrix.hpp's own implementation - see the
// section comment above for why no .margin() override is used here.
void checkRawFrame(const MotorsMatrix& m, const RawMotorDef* motors, std::uint8_t n) {
    for (std::uint8_t i = 0; i < n; ++i) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(m.motor_enabled(i));
        REQUIRE(m.roll_factor(i) == Approx(motors[i].roll_fac));
        REQUIRE(m.pitch_factor(i) == Approx(motors[i].pitch_fac));
        REQUIRE(m.yaw_factor(i) == Approx(motors[i].yaw_fac));
        REQUIRE(m.test_order(i) == motors[i].test_order);
    }
}

} // namespace

TEST_CASE("setup_octa_matrix: PLUS matches hand-computed angle/yaw/test_order", "[motors][setup_octa_matrix][plus]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octa_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(m.frame_class_string() == "OCTA");
    REQUIRE(m.frame_type_string() == "PLUS");
    const AngleMotor expected[] = {
        {0.0f, kYawFactorCw, 1},    {180.0f, kYawFactorCw, 5},   {45.0f, kYawFactorCcw, 2},
        {135.0f, kYawFactorCcw, 4}, {-45.0f, kYawFactorCcw, 8},  {-135.0f, kYawFactorCcw, 6},
        {-90.0f, kYawFactorCw, 7},  {90.0f, kYawFactorCw, 3},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octa_matrix: X matches hand-computed angle/yaw/test_order", "[motors][setup_octa_matrix][x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octa_matrix(MotorsMatrix::FrameType::X));
    REQUIRE(m.frame_type_string() == "X");
    const AngleMotor expected[] = {
        {22.5f, kYawFactorCw, 1},    {-157.5f, kYawFactorCw, 5},  {67.5f, kYawFactorCcw, 2},
        {157.5f, kYawFactorCcw, 4},  {-22.5f, kYawFactorCcw, 8},  {-112.5f, kYawFactorCcw, 6},
        {-67.5f, kYawFactorCw, 7},   {112.5f, kYawFactorCw, 3},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octa_matrix: V uses the real non-round explicit raw factors exactly, not angle-derived",
          "[motors][setup_octa_matrix][v]") {
    // Ticket's own explicit warning: 0.83f/0.34f/-0.67f/-0.32f/etc are
    // real, non-round upstream literals, not derived from any simple
    // angle formula - a rounded or approximated value here would be a
    // silent bug checkRawFrame's tight tolerance is built to catch.
    MotorsMatrix m;
    REQUIRE(m.setup_octa_matrix(MotorsMatrix::FrameType::V));
    REQUIRE(m.frame_type_string() == "V");
    const RawMotorDef expected[] = {
        {0.83f, 0.34f, kYawFactorCw, 7},   {-0.67f, -0.32f, kYawFactorCw, 3},
        {0.67f, -0.32f, kYawFactorCcw, 6}, {-0.50f, -1.00f, kYawFactorCcw, 4},
        {1.00f, 1.00f, kYawFactorCcw, 8},  {-0.83f, 0.34f, kYawFactorCcw, 2},
        {-1.00f, 1.00f, kYawFactorCw, 1},  {0.50f, -1.00f, kYawFactorCw, 5},
    };
    checkRawFrame(m, expected, 8);
}

TEST_CASE("setup_octa_matrix: H's own two 0.333f pitch entries are transcribed exactly, not rounded to "
          "+-1.0f or +-0.33f",
          "[motors][setup_octa_matrix][h]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octa_matrix(MotorsMatrix::FrameType::H));
    REQUIRE(m.frame_type_string() == "H");
    const RawMotorDef expected[] = {
        {-1.0f, 1.0f, kYawFactorCw, 1},    {1.0f, -1.0f, kYawFactorCw, 5},
        {-1.0f, 0.333f, kYawFactorCcw, 2}, {-1.0f, -1.0f, kYawFactorCcw, 4},
        {1.0f, 1.0f, kYawFactorCcw, 8},    {1.0f, -0.333f, kYawFactorCcw, 6},
        {1.0f, 0.333f, kYawFactorCw, 7},   {-1.0f, -0.333f, kYawFactorCw, 3},
    };
    checkRawFrame(m, expected, 8);
    // Direct, explicit assertions on the two 0.333f-magnitude entries -
    // the exact values the ticket's own acceptance criteria singles out
    // (motor index 2 = testing_order 2, motor index 5 = testing_order 6).
    REQUIRE(m.pitch_factor(2) == Approx(0.333f));
    REQUIRE(m.pitch_factor(5) == Approx(-0.333f));
}

TEST_CASE("setup_octa_matrix: I is the genuinely new frame type - \"(sideways H) octo only\"",
          "[motors][setup_octa_matrix][i]") {
    // Upstream's own enumerator comment (AP_Motors_Class.h line 91,
    // MOTOR_FRAME_TYPE_I = 15): "(sideways H) octo only" - the one
    // enumerator this ticket adds to FrameType (see motors_matrix.hpp's
    // file banner "CCP-004 ADDITION"). Same +-1.0f/+-0.333f value
    // vocabulary as H's own above, but arranged differently - here roll
    // carries the 0.333f-magnitude values, not pitch.
    MotorsMatrix m;
    REQUIRE(m.setup_octa_matrix(MotorsMatrix::FrameType::I));
    REQUIRE(m.frame_class_string() == "OCTA");
    REQUIRE(m.frame_type_string() == "I");
    const RawMotorDef expected[] = {
        {0.333f, -1.0f, kYawFactorCw, 5},    {-0.333f, 1.0f, kYawFactorCw, 1},
        {1.0f, -1.0f, kYawFactorCcw, 6},     {0.333f, 1.0f, kYawFactorCcw, 8},
        {-0.333f, -1.0f, kYawFactorCcw, 4},  {-1.0f, 1.0f, kYawFactorCcw, 2},
        {-1.0f, -1.0f, kYawFactorCw, 3},     {1.0f, 1.0f, kYawFactorCw, 7},
    };
    checkRawFrame(m, expected, 8);
    // Direct, explicit assertions on the 0.333f-magnitude roll entries.
    REQUIRE(m.roll_factor(0) == Approx(0.333f));
    REQUIRE(m.roll_factor(1) == Approx(-0.333f));
}

TEST_CASE("setup_octa_matrix: DJI_X matches hand-computed angle/yaw/test_order", "[motors][setup_octa_matrix][dji_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octa_matrix(MotorsMatrix::FrameType::DjiX));
    REQUIRE(m.frame_type_string() == "DJI_X");
    const AngleMotor expected[] = {
        {22.5f, kYawFactorCcw, 1},   {-22.5f, kYawFactorCw, 8},   {-67.5f, kYawFactorCcw, 7},
        {-112.5f, kYawFactorCw, 6},  {-157.5f, kYawFactorCcw, 5}, {157.5f, kYawFactorCw, 4},
        {112.5f, kYawFactorCcw, 3},  {67.5f, kYawFactorCw, 2},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octa_matrix: CW_X matches hand-computed angle/yaw/test_order", "[motors][setup_octa_matrix][cw_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octa_matrix(MotorsMatrix::FrameType::CwX));
    REQUIRE(m.frame_type_string() == "CW_X");
    const AngleMotor expected[] = {
        {22.5f, kYawFactorCcw, 1},  {67.5f, kYawFactorCw, 2},    {112.5f, kYawFactorCcw, 3},
        {157.5f, kYawFactorCw, 4},  {-157.5f, kYawFactorCcw, 5}, {-112.5f, kYawFactorCw, 6},
        {-67.5f, kYawFactorCcw, 7}, {-22.5f, kYawFactorCw, 8},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octa_matrix: returns true for every real in-scope frame type and sets frame_class_string to OCTA",
          "[motors][setup_octa_matrix]") {
    const MotorsMatrix::FrameType all_types[] = {
        MotorsMatrix::FrameType::Plus, MotorsMatrix::FrameType::X,    MotorsMatrix::FrameType::V,
        MotorsMatrix::FrameType::H,    MotorsMatrix::FrameType::I,    MotorsMatrix::FrameType::DjiX,
        MotorsMatrix::FrameType::CwX,
    };
    for (const auto ft : all_types) {
        MotorsMatrix m;
        REQUIRE(m.setup_octa_matrix(ft));
        REQUIRE(m.frame_class_string() == "OCTA");
        // Every real frame type populates all eight motor slots 0-7
        // (octa has eight motors, unlike setup_hexa_matrix's own six).
        for (std::uint8_t i = 0; i < 8; ++i) {
            REQUIRE(m.motor_enabled(i));
        }
    }
}

TEST_CASE("setup_octa_matrix: an out-of-range frame type hits the real SIMPLE default branch and returns false",
          "[motors][setup_octa_matrix][default]") {
    // Confirms the real upstream default case ("octa frame class does
    // not support this frame type; return false;") - the simple kind,
    // same as setup_quad_matrix's/setup_hexa_matrix's own, not
    // setup_y6_matrix's own productive default (a separate, later-ticket
    // concern per COP-005).
    MotorsMatrix m;
    REQUIRE_FALSE(m.setup_octa_matrix(static_cast<MotorsMatrix::FrameType>(255)));
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_octa_matrix: PLUS reuses the SAME FrameType enumerator as setup_quad_matrix's/setup_hexa_matrix's "
          "own PLUS, but yields octa-specific values on an eight-motor table",
          "[motors][setup_octa_matrix][setup_quad_matrix][plus]") {
    // Direct evidence for the file banner's enum investigation:
    // MotorsMatrix::FrameType::Plus is the literal same C++ enumerator
    // passed to setup_quad_matrix and setup_octa_matrix (mirroring
    // upstream's real single shared motor_frame_type enum), yet each
    // function's own frame table produces its own real, different
    // numeric results - dispatch is by which FUNCTION is called, not by
    // an octa-specific enumerator value.
    MotorsMatrix quad;
    MotorsMatrix octa;
    REQUIRE(quad.setup_quad_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(octa.setup_octa_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(quad.frame_class_string() == "QUAD");
    REQUIRE(octa.frame_class_string() == "OCTA");
    // Quad's PLUS motor 0 sits at 90 degrees (yaw CCW); octa's PLUS
    // motor 0 sits at 0 degrees (yaw CW) - genuinely different real
    // tables behind the same enumerator.
    REQUIRE(quad.roll_factor(0) != Approx(octa.roll_factor(0)));
    REQUIRE(quad.yaw_factor(0) == Approx(kYawFactorCcw));
    REQUIRE(octa.yaw_factor(0) == Approx(kYawFactorCw));
    // Octa populates four motor slots quad never touches.
    for (std::uint8_t i = 4; i < 8; ++i) {
        REQUIRE_FALSE(quad.motor_enabled(i));
        REQUIRE(octa.motor_enabled(i));
    }
}

// =======================================================================
// CCP-005: setup_octaquad_matrix - every real in-scope frame type,
// checked against hand-computed values from the real upstream angle/
// factor inputs (AP_MotorsMatrix.cpp lines 973-1140). All seven "plain"
// frame types (PLUS/X/V/H/CW_X/BF_X/BF_X_REV) are angle-based MotorDef
// tables - including V, which (unlike setup_octa_matrix's own raw-factor
// V) has the SAME shape as setup_quad_matrix's own V: ordinary
// angle-derived roll/pitch with a real, non-+-1 explicit yaw_factor - so
// checkAngleFrame() above (which compares yaw_factor by exact value, not
// just +-1) covers all seven directly. XCor/CwXCor are the real X8
// co-rotating pitfall this ticket's own acceptance criteria singles out:
// each applies a real, separate rescaling step to a genuinely different
// subset of the eight motors AFTER its own add_motors() table is
// applied - tested explicitly below, including throttle_factor (easy to
// silently miss - MotorDef has no throttle field, so the pre-scale value
// comes from add_motor_raw's own default 1.0f) and the real float-vs-
// double pitfall in kOctaquadCorotatingScaleFactor itself.
// =======================================================================

TEST_CASE("setup_octaquad_matrix: PLUS matches hand-computed angle/yaw/test_order", "[motors][setup_octaquad_matrix][plus]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(m.frame_class_string() == "OCTAQUAD");
    REQUIRE(m.frame_type_string() == "PLUS");
    const AngleMotor expected[] = {
        {0.0f, kYawFactorCcw, 1},   {-90.0f, kYawFactorCw, 7},  {180.0f, kYawFactorCcw, 5}, {90.0f, kYawFactorCw, 3},
        {-90.0f, kYawFactorCcw, 8}, {0.0f, kYawFactorCw, 2},    {90.0f, kYawFactorCcw, 4},  {180.0f, kYawFactorCw, 6},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octaquad_matrix: X matches hand-computed angle/yaw/test_order", "[motors][setup_octaquad_matrix][x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::X));
    REQUIRE(m.frame_type_string() == "X");
    const AngleMotor expected[] = {
        {45.0f, kYawFactorCcw, 1},  {-45.0f, kYawFactorCw, 7},   {-135.0f, kYawFactorCcw, 5}, {135.0f, kYawFactorCw, 3},
        {-45.0f, kYawFactorCcw, 8}, {45.0f, kYawFactorCw, 2},    {135.0f, kYawFactorCcw, 4},  {-135.0f, kYawFactorCw, 6},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octaquad_matrix: V uses the real non-angle-derived yaw factors exactly, same MotorDef shape as "
          "setup_quad_matrix's own V (not setup_octa_matrix's own raw-factor V)",
          "[motors][setup_octaquad_matrix][v]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::V));
    REQUIRE(m.frame_type_string() == "V");
    const AngleMotor expected[] = {
        {45.0f, 0.7981f, 1},  {-45.0f, -0.7981f, 7}, {-135.0f, 1.0000f, 5}, {135.0f, -1.0000f, 3},
        {-45.0f, 0.7981f, 8}, {45.0f, -0.7981f, 2},  {135.0f, 1.0000f, 4},  {-135.0f, -1.0000f, 6},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octaquad_matrix: H matches hand-computed angle/yaw/test_order", "[motors][setup_octaquad_matrix][h]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::H));
    REQUIRE(m.frame_type_string() == "H");
    const AngleMotor expected[] = {
        {45.0f, kYawFactorCw, 1},  {-45.0f, kYawFactorCcw, 7},  {-135.0f, kYawFactorCw, 5}, {135.0f, kYawFactorCcw, 3},
        {-45.0f, kYawFactorCw, 8}, {45.0f, kYawFactorCcw, 2},   {135.0f, kYawFactorCw, 4},  {-135.0f, kYawFactorCcw, 6},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octaquad_matrix: CW_X matches hand-computed angle/yaw/test_order", "[motors][setup_octaquad_matrix][cw_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::CwX));
    REQUIRE(m.frame_type_string() == "CW_X");
    const AngleMotor expected[] = {
        {45.0f, kYawFactorCcw, 1},   {45.0f, kYawFactorCw, 2},   {135.0f, kYawFactorCw, 3},  {135.0f, kYawFactorCcw, 4},
        {-135.0f, kYawFactorCcw, 5}, {-135.0f, kYawFactorCw, 6}, {-45.0f, kYawFactorCw, 7},  {-45.0f, kYawFactorCcw, 8},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octaquad_matrix: BF_X matches hand-computed angle/yaw/test_order", "[motors][setup_octaquad_matrix][bf_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::BfX));
    REQUIRE(m.frame_type_string() == "BF_X");
    const AngleMotor expected[] = {
        {135.0f, kYawFactorCw, 3},  {45.0f, kYawFactorCcw, 1},  {-135.0f, kYawFactorCcw, 5}, {-45.0f, kYawFactorCw, 7},
        {135.0f, kYawFactorCcw, 4}, {45.0f, kYawFactorCw, 2},   {-135.0f, kYawFactorCw, 6},  {-45.0f, kYawFactorCcw, 8},
    };
    checkAngleFrame(m, expected, 8);
}

TEST_CASE("setup_octaquad_matrix: BF_X_REV is BF_X with every yaw_factor sign-flipped, matching PLUS/PLUSREV's own "
          "sign-negation relationship",
          "[motors][setup_octaquad_matrix][bf_x_rev]") {
    MotorsMatrix bf_x;
    MotorsMatrix bf_x_rev;
    REQUIRE(bf_x.setup_octaquad_matrix(MotorsMatrix::FrameType::BfX));
    REQUIRE(bf_x_rev.setup_octaquad_matrix(MotorsMatrix::FrameType::BfXRev));
    REQUIRE(bf_x_rev.frame_type_string() == "X_REV");
    for (std::uint8_t i = 0; i < 8; ++i) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(bf_x_rev.roll_factor(i) == Approx(bf_x.roll_factor(i)));
        REQUIRE(bf_x_rev.pitch_factor(i) == Approx(bf_x.pitch_factor(i)));
        REQUIRE(bf_x_rev.yaw_factor(i) == Approx(-bf_x.yaw_factor(i)));
        REQUIRE(bf_x_rev.test_order(i) == bf_x.test_order(i));
    }
}

TEST_CASE("setup_octaquad_matrix: X_COR scales motors 0-3 by the real co-rotating factor and leaves motors 4-7 "
          "genuinely unscaled",
          "[motors][setup_octaquad_matrix][x_cor]") {
    // X_COR's own real table (AP_MotorsMatrix.cpp lines 1104-1113) is
    // angle-based, hand-computed here independently of
    // motors_matrix.hpp's own implementation, then compared BEFORE
    // scaling (motors 4-7) and AFTER scaling (motors 0-3, multiplied by
    // kOctaquadCorotatingScaleFactor) - proving the real SUBSET
    // distinction directly (genuinely different values, not "close to"
    // something), not merely that "some scaling happened somewhere".
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::XCor));
    REQUIRE(m.frame_class_string() == "OCTAQUAD");
    REQUIRE(m.frame_type_string() == "X_COR");

    const float angles[] = {45.0f, -45.0f, -135.0f, 135.0f, -45.0f, 45.0f, 135.0f, -135.0f};
    const float yaws[] = {kYawFactorCcw, kYawFactorCw, kYawFactorCcw, kYawFactorCw,
                           kYawFactorCw,  kYawFactorCcw, kYawFactorCw, kYawFactorCcw};

    for (std::uint8_t i = 0; i < 8; ++i) {
        INFO("motor index " << static_cast<int>(i));
        const float unscaled_roll = std::cos(fwcpp::math::radians(angles[i] + 90.0f));
        const float unscaled_pitch = std::cos(fwcpp::math::radians(angles[i]));
        if (i < 4) {
            // Scaled subset (first four) - genuinely DIFFERENT from the
            // plain angle-derived value, not merely close to it.
            REQUIRE(m.roll_factor(i) == Approx(unscaled_roll * kOctaquadCorotatingScaleFactor).margin(1e-5));
            REQUIRE(m.pitch_factor(i) == Approx(unscaled_pitch * kOctaquadCorotatingScaleFactor).margin(1e-5));
            REQUIRE(m.yaw_factor(i) == Approx(yaws[i] * kOctaquadCorotatingScaleFactor).margin(1e-6));
            REQUIRE(m.roll_factor(i) != Approx(unscaled_roll).margin(1e-4));
            REQUIRE(m.pitch_factor(i) != Approx(unscaled_pitch).margin(1e-4));
        } else {
            // Unscaled subset (last four) - the plain angle-derived value,
            // untouched by the co-rotating rescale loop.
            REQUIRE(m.roll_factor(i) == Approx(unscaled_roll).margin(1e-4));
            REQUIRE(m.pitch_factor(i) == Approx(unscaled_pitch).margin(1e-4));
            REQUIRE(m.yaw_factor(i) == Approx(yaws[i]).margin(1e-6));
        }
    }
}

TEST_CASE("setup_octaquad_matrix: CW_X_COR scales the EVEN-index motors (0,2,4,6) and leaves the ODD-index motors "
          "(1,3,5,7) genuinely unscaled - the mirror-image pattern from X_COR's own first-four",
          "[motors][setup_octaquad_matrix][cw_x_cor]") {
    // CW_X_COR's own real table (AP_MotorsMatrix.cpp lines 1123-1132) -
    // re-verified directly to use a genuinely DIFFERENT motor subset from
    // X_COR's own (every-other index, not first-four).
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::CwXCor));
    REQUIRE(m.frame_class_string() == "OCTAQUAD");
    REQUIRE(m.frame_type_string() == "CW_X_COR");

    const float angles[] = {45.0f, 45.0f, 135.0f, 135.0f, -135.0f, -135.0f, -45.0f, -45.0f};
    const float yaws[] = {kYawFactorCcw, kYawFactorCcw, kYawFactorCw,  kYawFactorCw,
                           kYawFactorCcw, kYawFactorCcw, kYawFactorCw, kYawFactorCw};

    for (std::uint8_t i = 0; i < 8; ++i) {
        INFO("motor index " << static_cast<int>(i));
        const float unscaled_roll = std::cos(fwcpp::math::radians(angles[i] + 90.0f));
        const float unscaled_pitch = std::cos(fwcpp::math::radians(angles[i]));
        if (i % 2 == 0) {
            // Scaled subset (even indices) - genuinely DIFFERENT from the
            // plain angle-derived value.
            REQUIRE(m.roll_factor(i) == Approx(unscaled_roll * kOctaquadCorotatingScaleFactor).margin(1e-5));
            REQUIRE(m.pitch_factor(i) == Approx(unscaled_pitch * kOctaquadCorotatingScaleFactor).margin(1e-5));
            REQUIRE(m.yaw_factor(i) == Approx(yaws[i] * kOctaquadCorotatingScaleFactor).margin(1e-6));
            REQUIRE(m.roll_factor(i) != Approx(unscaled_roll).margin(1e-4));
            REQUIRE(m.pitch_factor(i) != Approx(unscaled_pitch).margin(1e-4));
        } else {
            // Unscaled subset (odd indices) - untouched by the rescale
            // loop, the mirror image of X_COR's own even/first-four split.
            REQUIRE(m.roll_factor(i) == Approx(unscaled_roll).margin(1e-4));
            REQUIRE(m.pitch_factor(i) == Approx(unscaled_pitch).margin(1e-4));
            REQUIRE(m.yaw_factor(i) == Approx(yaws[i]).margin(1e-6));
        }
    }
}

TEST_CASE("setup_octaquad_matrix: X_COR scales throttle_factor too, not just roll/pitch/yaw",
          "[motors][setup_octaquad_matrix][x_cor][throttle]") {
    // Neither X_COR's own MotorDef table nor add_motors() ever sets an
    // explicit throttle_factor (MotorDef has no such field), so the
    // PRE-scale value for every motor is add_motor_raw's own default
    // (1.0f, CCP-001) - scaling it by kOctaquadCorotatingScaleFactor
    // therefore leaves the scaled result EQUAL to the scale factor itself
    // for motors 0-3, and untouched at 1.0f for motors 4-7. A port that
    // forgot to scale throttle (only scaling roll/pitch/yaw) would fail
    // this test by leaving motors 0-3 at 1.0f too - the exact silent bug
    // the ticket's own acceptance criteria warns about.
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::XCor));
    for (std::uint8_t i = 0; i < 4; ++i) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(m.throttle_factor(i) == Approx(kOctaquadCorotatingScaleFactor));
        REQUIRE(m.throttle_factor(i) != Approx(1.0f));
    }
    for (std::uint8_t i = 4; i < 8; ++i) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(m.throttle_factor(i) == Approx(1.0f));
    }
}

TEST_CASE("setup_octaquad_matrix: CW_X_COR scales throttle_factor too, on the even-index subset",
          "[motors][setup_octaquad_matrix][cw_x_cor][throttle]") {
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::CwXCor));
    for (std::uint8_t i = 0; i < 8; i += 2) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(m.throttle_factor(i) == Approx(kOctaquadCorotatingScaleFactor));
        REQUIRE(m.throttle_factor(i) != Approx(1.0f));
    }
    for (std::uint8_t i = 1; i < 8; i += 2) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(m.throttle_factor(i) == Approx(1.0f));
    }
}

TEST_CASE("setup_octaquad_matrix: X_COR's real 0.9 scale factor computes in float precision, not double - the "
          "exact float-vs-double pitfall COP-005 found",
          "[motors][setup_octaquad_matrix][x_cor][float_vs_double]") {
    // Motor 0's real pre-scale pitch_factor is cos(radians(45deg)) as
    // float (~0.70710677f). Independently verified (bit-level, outside
    // this codebase) that multiplying THIS specific value by 0.9
    // diverges by exactly one ULP between float-precision arithmetic
    // (0.9f * value, both operands float) and double-precision
    // arithmetic ((double)0.9 * value, narrowed back to float): the
    // float path yields bit pattern 0x3f22eada, the double path
    // 0x3f22eadb. A port whose scale constant is a bare, unsuffixed
    // `0.9` literal (computing in double, since this module does not
    // link fwcpp_upstream_flags - see motors_matrix.hpp's file banner)
    // would produce the double-path bit pattern here: numerically valid-
    // looking, but one ULP away from upstream's real EFFECTIVE value
    // (upstream's own build applies -fsingle-precision-constant, forcing
    // its bare `0.9` to behave as `float`). This test uses EXACT
    // equality (no Approx()/.margin() - the file's own looser tolerance
    // used for angle-derived trig comparisons elsewhere would be far too
    // loose to catch a single ULP) against a reference computed with an
    // explicitly float-typed `0.9f` multiplication, per the ticket's own
    // explicit test requirement.
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::XCor));

    const float unscaled_pitch = std::cos(fwcpp::math::radians(45.0f));  // motor 0's real pre-scale pitch_factor
    const float float_reference = unscaled_pitch * 0.9f;                 // explicit float literal - the correct behavior
    const float double_reference =
        static_cast<float>(static_cast<double>(unscaled_pitch) * 0.9);  // bare double literal - the bug this test catches

    // Sanity check on the test itself: these two references really are
    // numerically DIFFERENT for this specific value (independently
    // verified at the bit level) - if they were equal, this test would
    // not actually be able to distinguish a correct float-precision
    // implementation from a buggy double-precision one.
    REQUIRE(float_reference != double_reference);

    REQUIRE(m.pitch_factor(0) == float_reference);
    REQUIRE(m.pitch_factor(0) != double_reference);
}

TEST_CASE("setup_octaquad_matrix: CW_X_COR's real 0.9 scale factor also computes in float precision, not double",
          "[motors][setup_octaquad_matrix][cw_x_cor][float_vs_double]") {
    // Same exact-equality technique as X_COR's own test above, applied to
    // CW_X_COR's own motor 0 (also at a 45 degree angle, so the identical
    // pre-scale pitch_factor value and identical one-ULP float-vs-double
    // divergence applies).
    MotorsMatrix m;
    REQUIRE(m.setup_octaquad_matrix(MotorsMatrix::FrameType::CwXCor));

    const float unscaled_pitch = std::cos(fwcpp::math::radians(45.0f));
    const float float_reference = unscaled_pitch * 0.9f;
    const float double_reference = static_cast<float>(static_cast<double>(unscaled_pitch) * 0.9);
    REQUIRE(float_reference != double_reference);

    REQUIRE(m.pitch_factor(0) == float_reference);
    REQUIRE(m.pitch_factor(0) != double_reference);
}

TEST_CASE("setup_octaquad_matrix: returns true for every real in-scope frame type and sets frame_class_string to "
          "OCTAQUAD",
          "[motors][setup_octaquad_matrix]") {
    const MotorsMatrix::FrameType all_types[] = {
        MotorsMatrix::FrameType::Plus, MotorsMatrix::FrameType::X,      MotorsMatrix::FrameType::V,
        MotorsMatrix::FrameType::H,    MotorsMatrix::FrameType::CwX,    MotorsMatrix::FrameType::BfX,
        MotorsMatrix::FrameType::BfXRev, MotorsMatrix::FrameType::XCor, MotorsMatrix::FrameType::CwXCor,
    };
    for (const auto ft : all_types) {
        MotorsMatrix m;
        REQUIRE(m.setup_octaquad_matrix(ft));
        REQUIRE(m.frame_class_string() == "OCTAQUAD");
        // Every real frame type populates all eight motor slots 0-7.
        for (std::uint8_t i = 0; i < 8; ++i) {
            REQUIRE(m.motor_enabled(i));
        }
    }
}

TEST_CASE("setup_octaquad_matrix: an out-of-range frame type hits the real SIMPLE default branch and returns false",
          "[motors][setup_octaquad_matrix][default]") {
    // Confirms the real upstream default case ("octaquad frame class does
    // not support this frame type; return false;") - the simple kind,
    // same as every other already-ported setup_*_matrix's own default,
    // NOT setup_y6_matrix's own productive default (a separate,
    // later-ticket concern per COP-005).
    MotorsMatrix m;
    REQUIRE_FALSE(m.setup_octaquad_matrix(static_cast<MotorsMatrix::FrameType>(255)));
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_octaquad_matrix: PLUS reuses the SAME FrameType enumerator as setup_quad_matrix's/"
          "setup_hexa_matrix's/setup_octa_matrix's own PLUS, but yields octaquad-specific values",
          "[motors][setup_octaquad_matrix][setup_quad_matrix][plus]") {
    MotorsMatrix quad;
    MotorsMatrix octaquad;
    REQUIRE(quad.setup_quad_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(octaquad.setup_octaquad_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(quad.frame_class_string() == "QUAD");
    REQUIRE(octaquad.frame_class_string() == "OCTAQUAD");
    // Quad's PLUS motor 0 sits at 90 degrees (yaw CCW); octaquad's PLUS
    // motor 0 sits at 0 degrees (yaw CCW too, but a different real angle
    // and therefore different roll/pitch) - genuinely different real
    // tables behind the same enumerator.
    REQUIRE(quad.roll_factor(0) != Approx(octaquad.roll_factor(0)));
    for (std::uint8_t i = 4; i < 8; ++i) {
        REQUIRE_FALSE(quad.motor_enabled(i));
        REQUIRE(octaquad.motor_enabled(i));
    }
}

// =======================================================================
// CCP-006: setup_y6_matrix - Y6B/Y6F checked by direct value comparison
// (raw MotorDefRaw entries, same style as setup_quad_matrix's own Y4
// test above), PLUS the real productive `default:` fallback this
// ticket's whole scope is about. See motors_matrix.hpp's file banner
// "CCP-006 ADDITION" for the full case-list/default-branch
// investigation - this function is a genuine structural departure from
// every setup_*_matrix tested above: its own default: case is a real,
// working fallback table (not "unsupported, return false"), and the
// function itself never returns false anywhere.
// =======================================================================

TEST_CASE("setup_y6_matrix: Y6B raw factors match exactly, no angle conversion involved",
          "[motors][setup_y6_matrix][y6b]") {
    MotorsMatrix m;
    REQUIRE(m.setup_y6_matrix(MotorsMatrix::FrameType::Y6B));
    REQUIRE(m.frame_class_string() == "Y6");
    REQUIRE(m.frame_type_string() == "Y6B");

    // "Y6 motor definition with all top motors spinning clockwise, all
    // bottom motors counter clockwise" - upstream's own comment,
    // transcribed exactly (AP_MotorsMatrix.cpp line 1195).
    REQUIRE(m.roll_factor(0) == Approx(-1.0f));
    REQUIRE(m.pitch_factor(0) == Approx(0.500f));
    REQUIRE(m.yaw_factor(0) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(0) == 1);

    REQUIRE(m.roll_factor(1) == Approx(-1.0f));
    REQUIRE(m.pitch_factor(1) == Approx(0.500f));
    REQUIRE(m.yaw_factor(1) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(1) == 2);

    REQUIRE(m.roll_factor(2) == Approx(0.0f));
    REQUIRE(m.pitch_factor(2) == Approx(-1.000f));
    REQUIRE(m.yaw_factor(2) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(2) == 3);

    REQUIRE(m.roll_factor(3) == Approx(0.0f));
    REQUIRE(m.pitch_factor(3) == Approx(-1.000f));
    REQUIRE(m.yaw_factor(3) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(3) == 4);

    REQUIRE(m.roll_factor(4) == Approx(1.0f));
    REQUIRE(m.pitch_factor(4) == Approx(0.500f));
    REQUIRE(m.yaw_factor(4) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(4) == 5);

    REQUIRE(m.roll_factor(5) == Approx(1.0f));
    REQUIRE(m.pitch_factor(5) == Approx(0.500f));
    REQUIRE(m.yaw_factor(5) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(5) == 6);
}

TEST_CASE("setup_y6_matrix: Y6F raw factors match exactly, including the real non-ascending testing_order array "
          "layout",
          "[motors][setup_y6_matrix][y6f]") {
    MotorsMatrix m;
    REQUIRE(m.setup_y6_matrix(MotorsMatrix::FrameType::Y6F));
    REQUIRE(m.frame_class_string() == "Y6");
    REQUIRE(m.frame_type_string() == "Y6F");

    // "Y6 motor layout for FireFlyY6" - upstream's own comment,
    // transcribed exactly (AP_MotorsMatrix.cpp line 1207). Note the
    // array's own testing_order sequence is (3,1,5,4,2,6), NOT ascending -
    // add_motors_raw uses array INDEX as motor_num, not testing_order, so
    // this is re-verified per-slot rather than assumed to be in order.
    REQUIRE(m.roll_factor(0) == Approx(0.0f));
    REQUIRE(m.pitch_factor(0) == Approx(-1.000f));
    REQUIRE(m.yaw_factor(0) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(0) == 3);

    REQUIRE(m.roll_factor(1) == Approx(-1.0f));
    REQUIRE(m.pitch_factor(1) == Approx(0.500f));
    REQUIRE(m.yaw_factor(1) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(1) == 1);

    REQUIRE(m.roll_factor(2) == Approx(1.0f));
    REQUIRE(m.pitch_factor(2) == Approx(0.500f));
    REQUIRE(m.yaw_factor(2) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(2) == 5);

    REQUIRE(m.roll_factor(3) == Approx(0.0f));
    REQUIRE(m.pitch_factor(3) == Approx(-1.000f));
    REQUIRE(m.yaw_factor(3) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(3) == 4);

    REQUIRE(m.roll_factor(4) == Approx(-1.0f));
    REQUIRE(m.pitch_factor(4) == Approx(0.500f));
    REQUIRE(m.yaw_factor(4) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(4) == 2);

    REQUIRE(m.roll_factor(5) == Approx(1.0f));
    REQUIRE(m.pitch_factor(5) == Approx(0.500f));
    REQUIRE(m.yaw_factor(5) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(5) == 6);
}

namespace {
// Shared expected values for the real productive `default:` fallback
// table (AP_MotorsMatrix.cpp lines 1219-1236) - upstream's own case has
// no comment on it, just the table. Factored out so every fallback test
// below checks the identical expected values, rather than each spelling
// out its own copy that could silently drift.
void checkY6DefaultFallback(const MotorsMatrix& m) {
    REQUIRE(m.roll_factor(0) == Approx(-1.0f));
    REQUIRE(m.pitch_factor(0) == Approx(0.666f));
    REQUIRE(m.yaw_factor(0) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(0) == 2);

    REQUIRE(m.roll_factor(1) == Approx(1.0f));
    REQUIRE(m.pitch_factor(1) == Approx(0.666f));
    REQUIRE(m.yaw_factor(1) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(1) == 5);

    REQUIRE(m.roll_factor(2) == Approx(1.0f));
    REQUIRE(m.pitch_factor(2) == Approx(0.666f));
    REQUIRE(m.yaw_factor(2) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(2) == 6);

    REQUIRE(m.roll_factor(3) == Approx(0.0f));
    REQUIRE(m.pitch_factor(3) == Approx(-1.333f));
    REQUIRE(m.yaw_factor(3) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(3) == 4);

    REQUIRE(m.roll_factor(4) == Approx(-1.0f));
    REQUIRE(m.pitch_factor(4) == Approx(0.666f));
    REQUIRE(m.yaw_factor(4) == Approx(kYawFactorCw));
    REQUIRE(m.test_order(4) == 1);

    REQUIRE(m.roll_factor(5) == Approx(0.0f));
    REQUIRE(m.pitch_factor(5) == Approx(-1.333f));
    REQUIRE(m.yaw_factor(5) == Approx(kYawFactorCcw));
    REQUIRE(m.test_order(5) == 3);
}
} // namespace

TEST_CASE("setup_y6_matrix: THE REAL PRODUCTIVE default - several different non-Y6B/Y6F FrameType inputs ALL "
          "produce the exact same fallback table, not a rejection and not different tables per input",
          "[motors][setup_y6_matrix][default][pitfall]") {
    // THE SINGLE MOST IMPORTANT TEST THIS TICKET WRITES (per CCP-006's
    // own explicit acceptance criteria and file banner). Real upstream's
    // setup_y6_matrix has a switch naming only Y6B/Y6F explicitly; every
    // OTHER real, valid motor_frame_type value silently falls through to
    // the SAME productive default: fallback table - it does NOT reject
    // or vary per input. copter-rust's own COP-005 investigation found
    // this first (24 of 64 real upstream frame configurations come from
    // this exact fallback). A careless port would either (a) return
    // false for these inputs, or (b) build a different table depending
    // on which input arrived - this test would fail under EITHER wrong
    // behavior: it checks that Plus/X/V/H/I/XCor (six real, valid
    // FrameType values this port's own enum already defines from earlier
    // tickets, none of which is Y6B/Y6F) each return true, each report
    // frame_type_string() == "default", and each produce IDENTICAL
    // per-motor factors/testing_order to the shared expected values in
    // checkY6DefaultFallback() above - not merely "some" fallback, but
    // the SAME one every time.
    const MotorsMatrix::FrameType non_y6_types[] = {
        MotorsMatrix::FrameType::Plus, MotorsMatrix::FrameType::X,  MotorsMatrix::FrameType::V,
        MotorsMatrix::FrameType::H,    MotorsMatrix::FrameType::I,  MotorsMatrix::FrameType::XCor,
    };
    for (const auto ft : non_y6_types) {
        MotorsMatrix m;
        // Not a rejection: setup_y6_matrix returns true for every one of
        // these, exactly like it does for Y6B/Y6F.
        REQUIRE(m.setup_y6_matrix(ft));
        REQUIRE(m.frame_class_string() == "Y6");
        REQUIRE(m.frame_type_string() == "default");
        checkY6DefaultFallback(m);
    }

    // Cross-check: two DIFFERENT non-Y6B/Y6F inputs produce bit-for-bit
    // identical per-motor state, not merely each independently matching
    // the expected constants above - guards against a checkY6
    // DefaultFallback() copy/paste bug hiding a real per-input
    // difference.
    MotorsMatrix from_plus;
    MotorsMatrix from_h;
    REQUIRE(from_plus.setup_y6_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(from_h.setup_y6_matrix(MotorsMatrix::FrameType::H));
    for (std::uint8_t i = 0; i < 6; ++i) {
        REQUIRE(from_plus.roll_factor(i) == Approx(from_h.roll_factor(i)));
        REQUIRE(from_plus.pitch_factor(i) == Approx(from_h.pitch_factor(i)));
        REQUIRE(from_plus.yaw_factor(i) == Approx(from_h.yaw_factor(i)));
        REQUIRE(from_plus.test_order(i) == from_h.test_order(i));
    }
}

TEST_CASE("setup_y6_matrix: the real productive default is also reached by a truly out-of-range FrameType, and "
          "still returns true",
          "[motors][setup_y6_matrix][default][pitfall]") {
    // Same fallback, reached the same way setup_quad_matrix's/
    // setup_hexa_matrix's/setup_octa_matrix's/setup_octaquad_matrix's own
    // "out-of-range default" tests reach THEIR default - except here the
    // default is productive, not a rejection: this must still return
    // true and populate the exact same fallback table, not false.
    MotorsMatrix m;
    REQUIRE(m.setup_y6_matrix(static_cast<MotorsMatrix::FrameType>(255)));
    REQUIRE(m.frame_type_string() == "default");
    checkY6DefaultFallback(m);
}

TEST_CASE("setup_y6_matrix: never returns false, for every real FrameType this port's own enum currently defines "
          "plus an out-of-range value",
          "[motors][setup_y6_matrix][default][pitfall]") {
    // Confirms the SECOND real, related consequence the file banner
    // discloses: unlike every other setup_*_matrix above (each of which
    // has exactly one real `return false;`, in its own default: case),
    // setup_y6_matrix's own body has NO `return false;` anywhere -
    // re-verified directly against the real source. This test exercises
    // EVERY enumerator this port's own FrameType currently defines
    // (Y6B/Y6F explicitly handled, everything else via the productive
    // default), plus a genuinely out-of-range enum value constructed via
    // an explicit cast (this port's own established idiom for
    // constructing such a value safely, matching every other
    // setup_*_matrix's own "out-of-range default" test above) - the
    // approach taken per the ticket's own explicit instruction to use
    // judgment between "cast an out-of-range value" and "exhaustively
    // test every real enumerator", since C++'s enum class permits
    // constructing an out-of-range value safely for scoped enums with a
    // fixed underlying type (std::uint8_t here), this test does BOTH
    // rather than only the weaker substitute.
    const MotorsMatrix::FrameType all_types[] = {
        MotorsMatrix::FrameType::Plus,    MotorsMatrix::FrameType::X,      MotorsMatrix::FrameType::BfX,
        MotorsMatrix::FrameType::BfXRev,  MotorsMatrix::FrameType::DjiX,   MotorsMatrix::FrameType::CwX,
        MotorsMatrix::FrameType::V,       MotorsMatrix::FrameType::H,      MotorsMatrix::FrameType::VTail,
        MotorsMatrix::FrameType::ATail,   MotorsMatrix::FrameType::PlusRev, MotorsMatrix::FrameType::Y4,
        MotorsMatrix::FrameType::I,       MotorsMatrix::FrameType::XCor,   MotorsMatrix::FrameType::CwXCor,
        MotorsMatrix::FrameType::Y6B,     MotorsMatrix::FrameType::Y6F,
    };
    for (const auto ft : all_types) {
        MotorsMatrix m;
        REQUIRE(m.setup_y6_matrix(ft));
    }
    // The genuinely out-of-range value too - matching the out-of-range
    // idiom every other setup_*_matrix's own default test above uses.
    MotorsMatrix m_oor;
    REQUIRE(m_oor.setup_y6_matrix(static_cast<MotorsMatrix::FrameType>(255)));
}

TEST_CASE("setup_y6_matrix: Y6B and Y6F reuse no FrameType enumerator from any earlier setup_*_matrix - both are "
          "genuinely new",
          "[motors][setup_y6_matrix][y6b][y6f]") {
    // Y6B/Y6F are CCP-006's own two new enumerators (see file banner's
    // "CCP-006 ADDITION" enum investigation) - unlike CCP-003's hexa
    // (zero new enumerators) or CCP-004's octa/CCP-005's octaquad (one
    // and two new enumerators respectively, each reused by a LATER
    // setup_*_matrix), Y6B/Y6F are used ONLY by setup_y6_matrix: passing
    // either one to an earlier setup_*_matrix hits that function's own
    // SIMPLE "not supported, return false" default, not a real frame
    // table - confirming they are genuinely new to the switch dispatch,
    // not silently aliasing an existing case in another frame class.
    MotorsMatrix quad;
    MotorsMatrix hexa;
    MotorsMatrix octa;
    MotorsMatrix octaquad;
    REQUIRE_FALSE(quad.setup_quad_matrix(MotorsMatrix::FrameType::Y6B));
    REQUIRE_FALSE(hexa.setup_hexa_matrix(MotorsMatrix::FrameType::Y6B));
    REQUIRE_FALSE(octa.setup_octa_matrix(MotorsMatrix::FrameType::Y6F));
    REQUIRE_FALSE(octaquad.setup_octaquad_matrix(MotorsMatrix::FrameType::Y6F));
}

// =======================================================================
// CCP-007: setup_dodecahexa_matrix - every real in-scope frame type,
// checked against hand-computed values from the real upstream angle/yaw
// inputs (AP_MotorsMatrix.cpp lines 1140-1188). Angle-based frames reuse
// checkAngleFrame() above (both PLUS and X are plain add_motors() calls
// over a 12-entry MotorDef table - no add_motors_raw()/direct add_motor()
// calls anywhere in this function). A RETURN to the SIMPLE default: shape
// used by setup_quad_matrix/setup_hexa_matrix/setup_octa_matrix/
// setup_octaquad_matrix above, NOT a continuation of setup_y6_matrix's
// own productive-default departure - see motors_matrix.hpp's file banner
// "CCP-007 ADDITION" for the full investigation, including the
// correction of the ticket's own claim about which ticket first added
// zero new FrameType enumerators (CCP-003, not this one).
// =======================================================================

namespace {

// Confirms the real, specific alternating-pair structure the ticket calls
// out as the one pitfall this frame class is exposed to: both PLUS and X
// repeat each of six physical-position angles TWICE (a top/bottom motor
// pair), and the two motors of each pair must have EXACTLY OPPOSITE yaw
// factors (one CW, one CCW) and the SAME roll/pitch (since they share the
// same angle). A copy-paste bug that collapsed a pair into two IDENTICAL
// yaw factors would still populate 12 motors with plausible angles and
// would NOT be caught by checkAngleFrame() alone if the expected-value
// table itself were copy-pasted the same wrong way - so this check
// re-derives the "must be opposite" property structurally (yaw(2i) ==
// -yaw(2i+1)) rather than only comparing against a second hard-coded
// expected table.
void checkAlternatingTopBottomPairs(const MotorsMatrix& m) {
    for (std::uint8_t pair = 0; pair < 6; ++pair) {
        const std::uint8_t top = static_cast<std::uint8_t>(pair * 2);
        const std::uint8_t bottom = static_cast<std::uint8_t>(pair * 2 + 1);
        INFO("physical position " << static_cast<int>(pair) << " (motors " << static_cast<int>(top) << "/"
                                   << static_cast<int>(bottom) << ")");
        REQUIRE(m.motor_enabled(top));
        REQUIRE(m.motor_enabled(bottom));
        // Same angle -> same roll/pitch factors for the pair.
        REQUIRE(m.roll_factor(top) == Approx(m.roll_factor(bottom)).margin(1e-4));
        REQUIRE(m.pitch_factor(top) == Approx(m.pitch_factor(bottom)).margin(1e-4));
        // The real pitfall: yaw factors must be EXACT opposites, never
        // equal - a collapsed/copy-pasted pair would fail this.
        REQUIRE(m.yaw_factor(top) == Approx(-m.yaw_factor(bottom)).margin(1e-6));
        REQUIRE(std::fabs(m.yaw_factor(top)) == Approx(1.0f).margin(1e-6));
        REQUIRE(m.yaw_factor(top) != Approx(m.yaw_factor(bottom)).margin(1e-6));
    }
}

} // namespace

TEST_CASE("setup_dodecahexa_matrix: PLUS matches hand-computed angle/yaw/test_order across all twelve motors",
          "[motors][setup_dodecahexa_matrix][plus]") {
    MotorsMatrix m;
    REQUIRE(m.setup_dodecahexa_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(m.frame_class_string() == "DODECAHEXA");
    REQUIRE(m.frame_type_string() == "PLUS");
    const AngleMotor expected[] = {
        {0.0f, kYawFactorCcw, 1},    // forward-top
        {0.0f, kYawFactorCw, 2},     // forward-bottom
        {60.0f, kYawFactorCw, 3},    // forward-right-top
        {60.0f, kYawFactorCcw, 4},   // forward-right-bottom
        {120.0f, kYawFactorCcw, 5},  // back-right-top
        {120.0f, kYawFactorCw, 6},   // back-right-bottom
        {180.0f, kYawFactorCw, 7},   // back-top
        {180.0f, kYawFactorCcw, 8},  // back-bottom
        {-120.0f, kYawFactorCcw, 9}, // back-left-top
        {-120.0f, kYawFactorCw, 10}, // back-left-bottom
        {-60.0f, kYawFactorCw, 11},  // forward-left-top
        {-60.0f, kYawFactorCcw, 12}, // forward-left-bottom
    };
    checkAngleFrame(m, expected, 12);
    // Motor 12 and beyond must remain untouched - dodecahexa is exactly
    // 12 motors, not more.
    for (std::uint8_t i = 12; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_dodecahexa_matrix: X matches hand-computed angle/yaw/test_order across all twelve motors",
          "[motors][setup_dodecahexa_matrix][x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_dodecahexa_matrix(MotorsMatrix::FrameType::X));
    REQUIRE(m.frame_class_string() == "DODECAHEXA");
    REQUIRE(m.frame_type_string() == "X");
    const AngleMotor expected[] = {
        {30.0f, kYawFactorCcw, 1},   // forward-right-top
        {30.0f, kYawFactorCw, 2},    // forward-right-bottom
        {90.0f, kYawFactorCw, 3},    // right-top
        {90.0f, kYawFactorCcw, 4},   // right-bottom
        {150.0f, kYawFactorCcw, 5},  // back-right-top
        {150.0f, kYawFactorCw, 6},   // back-right-bottom
        {-150.0f, kYawFactorCw, 7},  // back-left-top
        {-150.0f, kYawFactorCcw, 8}, // back-left-bottom
        {-90.0f, kYawFactorCcw, 9},  // left-top
        {-90.0f, kYawFactorCw, 10},  // left-bottom
        {-30.0f, kYawFactorCw, 11},  // forward-left-top
        {-30.0f, kYawFactorCcw, 12}, // forward-left-bottom
    };
    checkAngleFrame(m, expected, 12);
    for (std::uint8_t i = 12; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_dodecahexa_matrix: PLUS's six top/bottom motor pairs alternate yaw factors, not collapsed to "
          "identical values - THE ONE PITFALL THIS TICKET GUARDS AGAINST",
          "[motors][setup_dodecahexa_matrix][plus][pitfall]") {
    // Per the ticket's own explicit instruction: a test that would catch
    // a copy-paste bug collapsing a pair into two identical yaw factors
    // instead of the real alternating CW/CCW pattern.
    MotorsMatrix m;
    REQUIRE(m.setup_dodecahexa_matrix(MotorsMatrix::FrameType::Plus));
    checkAlternatingTopBottomPairs(m);
}

TEST_CASE("setup_dodecahexa_matrix: X's six top/bottom motor pairs alternate yaw factors, not collapsed to "
          "identical values - THE ONE PITFALL THIS TICKET GUARDS AGAINST",
          "[motors][setup_dodecahexa_matrix][x][pitfall]") {
    MotorsMatrix m;
    REQUIRE(m.setup_dodecahexa_matrix(MotorsMatrix::FrameType::X));
    checkAlternatingTopBottomPairs(m);
}

TEST_CASE("setup_dodecahexa_matrix: returns true for every real in-scope frame type and sets frame_class_string "
          "to DODECAHEXA",
          "[motors][setup_dodecahexa_matrix]") {
    const MotorsMatrix::FrameType all_types[] = {
        MotorsMatrix::FrameType::Plus,
        MotorsMatrix::FrameType::X,
    };
    for (const auto ft : all_types) {
        MotorsMatrix m;
        REQUIRE(m.setup_dodecahexa_matrix(ft));
        REQUIRE(m.frame_class_string() == "DODECAHEXA");
        // Every real frame type populates all twelve motor slots 0-11
        // (dodecahexa has twelve motors, unlike setup_quad_matrix's own
        // four or setup_hexa_matrix's/setup_y6_matrix's own six).
        for (std::uint8_t i = 0; i < 12; ++i) {
            REQUIRE(m.motor_enabled(i));
        }
    }
}

TEST_CASE("setup_dodecahexa_matrix: an out-of-range frame type hits the real SIMPLE default branch and returns "
          "false",
          "[motors][setup_dodecahexa_matrix][default]") {
    // Confirms the real upstream default case ("dodeca-hexa frame class
    // does not support this frame type; return false;") - the SIMPLE
    // kind, a return to the shape used by setup_quad_matrix's/
    // setup_hexa_matrix's/setup_octa_matrix's/setup_octaquad_matrix's own
    // defaults, NOT setup_y6_matrix's own productive default (a
    // structural departure specific to that one ticket, per its own
    // banner section).
    MotorsMatrix m;
    REQUIRE_FALSE(m.setup_dodecahexa_matrix(static_cast<MotorsMatrix::FrameType>(255)));
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_dodecahexa_matrix: PLUS reuses the SAME FrameType enumerator as setup_quad_matrix's own PLUS, "
          "but yields dodecahexa-specific values on a twelve-motor table",
          "[motors][setup_dodecahexa_matrix][setup_quad_matrix][plus]") {
    // Direct evidence for the file banner's "CCP-007 ADDITION" enum
    // investigation: MotorsMatrix::FrameType::Plus is the literal same
    // C++ enumerator passed to both functions (mirroring upstream's real
    // single shared motor_frame_type enum), yet each function's own frame
    // table produces its own real, different numeric results - dispatch
    // is by which FUNCTION is called (i.e. which real motor_frame_class
    // the caller selected), not by a dodecahexa-specific enumerator
    // value. This ticket adds ZERO new enumerators to FrameType - see
    // file banner for the correction of the ticket's own claim that this
    // would be the first such ticket (it is the second; CCP-003 was
    // first).
    MotorsMatrix quad;
    MotorsMatrix dodecahexa;
    REQUIRE(quad.setup_quad_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(dodecahexa.setup_dodecahexa_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(quad.frame_class_string() == "QUAD");
    REQUIRE(dodecahexa.frame_class_string() == "DODECAHEXA");
    // Quad's PLUS motor 0 sits at 90 degrees (yaw CCW); dodecahexa's PLUS
    // motor 0 sits at 0 degrees (yaw CCW too, but a different real angle
    // and therefore different roll/pitch) - genuinely different real
    // tables behind the same enumerator.
    REQUIRE(quad.roll_factor(0) != Approx(dodecahexa.roll_factor(0)));
    // Dodecahexa populates eight motor slots quad never touches.
    for (std::uint8_t i = 4; i < 12; ++i) {
        REQUIRE_FALSE(quad.motor_enabled(i));
        REQUIRE(dodecahexa.motor_enabled(i));
    }
}

// =======================================================================
// CCP-008: setup_deca_matrix - THE LAST setup_*_matrix ticket in this arc.
// Every case's angle/yaw/test_order values are checked against
// hand-computed values from the real upstream inputs (AP_MotorsMatrix.cpp
// lines 1242-1287) using checkAngleFrame() above (both PLUS and the
// shared X/CW_X table are plain add_motors() calls over a 10-entry
// MotorDef table). Both real frame types (PLUS, and X/CW_X) reuse
// FrameType enumerators setup_quad_matrix's own tests above already
// exercise - this ticket adds ZERO new enumerators, the THIRD such
// zero-growth ticket in this arc (CCP-003, CCP-007, CCP-008) - see
// motors_matrix.hpp's file banner "CCP-008 ADDITION" for the full
// investigation. The single most important test in this section is the
// X/CW_X fall-through test below, which proves the real shared-table case
// (a genuinely distinct structural pattern from every prior ticket) was
// faithfully reproduced rather than accidentally split into two
// different tables.
// =======================================================================

TEST_CASE("setup_deca_matrix: PLUS matches hand-computed angle/yaw/test_order across all ten motors",
          "[motors][setup_deca_matrix][plus]") {
    MotorsMatrix m;
    REQUIRE(m.setup_deca_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(m.frame_class_string() == "DECA");
    REQUIRE(m.frame_type_string() == "PLUS");
    const AngleMotor expected[] = {
        {0.0f, kYawFactorCcw, 1},   {36.0f, kYawFactorCw, 2},   {72.0f, kYawFactorCcw, 3},
        {108.0f, kYawFactorCw, 4},  {144.0f, kYawFactorCcw, 5}, {180.0f, kYawFactorCw, 6},
        {-144.0f, kYawFactorCcw, 7}, {-108.0f, kYawFactorCw, 8}, {-72.0f, kYawFactorCcw, 9},
        {-36.0f, kYawFactorCw, 10},
    };
    checkAngleFrame(m, expected, 10);
    // Deca is exactly 10 motors - motor 10 and beyond must remain untouched.
    for (std::uint8_t i = 10; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_deca_matrix: X matches hand-computed angle/yaw/test_order across all ten motors",
          "[motors][setup_deca_matrix][x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_deca_matrix(MotorsMatrix::FrameType::X));
    REQUIRE(m.frame_class_string() == "DECA");
    // Real upstream sets a single COMBINED frame_type_string for this
    // shared case - "X/CW_X", not "X" alone - transcribed exactly, not
    // picking one name over the other.
    REQUIRE(m.frame_type_string() == "X/CW_X");
    const AngleMotor expected[] = {
        {18.0f, kYawFactorCcw, 1},  {54.0f, kYawFactorCw, 2},   {90.0f, kYawFactorCcw, 3},
        {126.0f, kYawFactorCw, 4},  {162.0f, kYawFactorCcw, 5}, {-162.0f, kYawFactorCw, 6},
        {-126.0f, kYawFactorCcw, 7}, {-90.0f, kYawFactorCw, 8}, {-54.0f, kYawFactorCcw, 9},
        {-18.0f, kYawFactorCw, 10},
    };
    checkAngleFrame(m, expected, 10);
    for (std::uint8_t i = 10; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_deca_matrix: CW_X matches hand-computed angle/yaw/test_order across all ten motors "
          "(the SAME table as X, not a separate one)",
          "[motors][setup_deca_matrix][cw_x]") {
    MotorsMatrix m;
    REQUIRE(m.setup_deca_matrix(MotorsMatrix::FrameType::CwX));
    REQUIRE(m.frame_class_string() == "DECA");
    REQUIRE(m.frame_type_string() == "X/CW_X");
    const AngleMotor expected[] = {
        {18.0f, kYawFactorCcw, 1},  {54.0f, kYawFactorCw, 2},   {90.0f, kYawFactorCcw, 3},
        {126.0f, kYawFactorCw, 4},  {162.0f, kYawFactorCcw, 5}, {-162.0f, kYawFactorCw, 6},
        {-126.0f, kYawFactorCcw, 7}, {-90.0f, kYawFactorCw, 8}, {-54.0f, kYawFactorCcw, 9},
        {-18.0f, kYawFactorCw, 10},
    };
    checkAngleFrame(m, expected, 10);
    for (std::uint8_t i = 10; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_deca_matrix: X and CW_X are a REAL fall-through sharing the IDENTICAL table - "
          "THE SINGLE MOST IMPORTANT TEST THIS TICKET WRITES",
          "[motors][setup_deca_matrix][x][cw_x][fallthrough]") {
    // This is the exact risk the ticket calls out: a careless port could
    // split upstream's real `case MOTOR_FRAME_TYPE_X: case
    // MOTOR_FRAME_TYPE_CW_X:` fall-through into two separate cases and
    // invent a second, DIFFERENT (wrong) table for CW_X - as every other
    // X/CW_X pair ported so far in this arc genuinely does have two
    // different tables (e.g. setup_octaquad_matrix's own X_COR/CW_X_COR
    // in CCP-005). Constructing two independent MotorsMatrix instances,
    // one via each enumerator, and requiring every one of the ten motors'
    // roll/pitch/yaw/test_order to match exactly (not merely
    // approximately similar) is the direct proof the shared table was
    // faithfully reproduced.
    MotorsMatrix x;
    MotorsMatrix cw_x;
    REQUIRE(x.setup_deca_matrix(MotorsMatrix::FrameType::X));
    REQUIRE(cw_x.setup_deca_matrix(MotorsMatrix::FrameType::CwX));

    REQUIRE(x.frame_class_string() == cw_x.frame_class_string());
    REQUIRE(x.frame_type_string() == cw_x.frame_type_string());
    REQUIRE(x.frame_type_string() == "X/CW_X");

    for (std::uint8_t i = 0; i < 10; ++i) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(x.motor_enabled(i));
        REQUIRE(cw_x.motor_enabled(i));
        REQUIRE(x.roll_factor(i) == Approx(cw_x.roll_factor(i)).margin(1e-6));
        REQUIRE(x.pitch_factor(i) == Approx(cw_x.pitch_factor(i)).margin(1e-6));
        REQUIRE(x.yaw_factor(i) == Approx(cw_x.yaw_factor(i)).margin(1e-6));
        REQUIRE(x.test_order(i) == cw_x.test_order(i));
    }
}

TEST_CASE("setup_deca_matrix: returns true for every real in-scope frame type and sets frame_class_string to DECA",
          "[motors][setup_deca_matrix]") {
    const MotorsMatrix::FrameType all_types[] = {
        MotorsMatrix::FrameType::Plus,
        MotorsMatrix::FrameType::X,
        MotorsMatrix::FrameType::CwX,
    };
    for (const auto ft : all_types) {
        MotorsMatrix m;
        REQUIRE(m.setup_deca_matrix(ft));
        REQUIRE(m.frame_class_string() == "DECA");
        // Every real frame type populates all ten motor slots 0-9 (deca
        // has ten motors, unlike setup_dodecahexa_matrix's own twelve).
        for (std::uint8_t i = 0; i < 10; ++i) {
            REQUIRE(m.motor_enabled(i));
        }
    }
}

TEST_CASE("setup_deca_matrix: an out-of-range frame type hits the real SIMPLE default branch and returns false",
          "[motors][setup_deca_matrix][default]") {
    // Confirms the real upstream default case ("deca frame class does not
    // support this frame type; return false;") - the SIMPLE kind, matching
    // every other setup_*_matrix's own default above except
    // setup_y6_matrix's own real productive fallback (a structural
    // departure specific to that one ticket).
    MotorsMatrix m;
    REQUIRE_FALSE(m.setup_deca_matrix(static_cast<MotorsMatrix::FrameType>(255)));
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
    }
}

TEST_CASE("setup_deca_matrix: PLUS reuses the SAME FrameType enumerator as setup_quad_matrix's own PLUS, "
          "but yields deca-specific values on a ten-motor table",
          "[motors][setup_deca_matrix][setup_quad_matrix][plus]") {
    // Direct evidence for the file banner's "CCP-008 ADDITION" enum
    // investigation: MotorsMatrix::FrameType::Plus is the literal same
    // C++ enumerator passed to both functions (mirroring upstream's real
    // single shared motor_frame_type enum), yet each function's own frame
    // table produces its own real, different numeric results - dispatch
    // is by which FUNCTION is called (i.e. which real motor_frame_class
    // the caller selected), not by a deca-specific enumerator value. This
    // ticket adds ZERO new enumerators to FrameType - the THIRD such
    // zero-growth ticket in this arc (CCP-003, CCP-007, CCP-008).
    MotorsMatrix quad;
    MotorsMatrix deca;
    REQUIRE(quad.setup_quad_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(deca.setup_deca_matrix(MotorsMatrix::FrameType::Plus));
    REQUIRE(quad.frame_class_string() == "QUAD");
    REQUIRE(deca.frame_class_string() == "DECA");
    // Quad's PLUS motor 0 sits at 90 degrees (yaw CCW); deca's PLUS motor
    // 0 sits at 0 degrees (yaw CCW too, but a different real angle and
    // therefore different roll/pitch) - genuinely different real tables
    // behind the same enumerator.
    REQUIRE(quad.roll_factor(0) != Approx(deca.roll_factor(0)));
    // Deca populates six motor slots quad never touches.
    for (std::uint8_t i = 4; i < 10; ++i) {
        REQUIRE_FALSE(quad.motor_enabled(i));
        REQUIRE(deca.motor_enabled(i));
    }
}

// ---------------------------------------------------------------------
// setup_motors (CCP-009) - the dispatcher that routes a (FrameClass,
// FrameType) pair to the correct one of the seven setup_*_matrix
// functions above, and closes out AP_MotorsMatrix's own
// construction-time configuration surface. See motors_matrix.hpp's own
// "CCP-009 ADDITION" file-banner section and setup_motors()'s own
// method-level comment for the full real seven-step structure this
// reproduces (real function lines 1290-1349).
// ---------------------------------------------------------------------

namespace {

// Compares every one of kMaxNumMotors motor slots between two
// MotorsMatrix instances - not just the ones a particular frame table is
// expected to populate - so a dispatch bug that enables/leaks an
// unexpected extra slot (or misses one) is caught just as reliably as a
// wrong factor value on an expected one.
void requireSameMotorState(const MotorsMatrix& a, const MotorsMatrix& b) {
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(a.motor_enabled(i) == b.motor_enabled(i));
        REQUIRE(a.roll_factor(i) == Approx(b.roll_factor(i)).margin(1e-6));
        REQUIRE(a.pitch_factor(i) == Approx(b.pitch_factor(i)).margin(1e-6));
        REQUIRE(a.yaw_factor(i) == Approx(b.yaw_factor(i)).margin(1e-6));
        REQUIRE(a.throttle_factor(i) == Approx(b.throttle_factor(i)).margin(1e-6));
        REQUIRE(a.test_order(i) == b.test_order(i));
    }
}

} // namespace

// NOTE ON THE COMPARISON BELOW: setup_motors's own real structure calls
// normalise_rpy_factors() UNCONDITIONALLY after the switch (see file
// banner) - a step none of the seven setup_*_matrix functions themselves
// perform (each returns immediately after its own add_motors()/
// add_motors_raw() calls, un-normalised). A literal
// `setup_motors(...)` vs `setup_X_matrix(...)` comparison would therefore
// legitimately mismatch on every roll/pitch/yaw/throttle factor (raw
// angle-derived values vs their normalised, magnitude-0.5-rescaled
// equivalents) despite the dispatch itself being entirely correct. Each
// test below therefore also calls normalise_rpy_factors() on the
// directly-constructed reference instance, reproducing setup_motors's
// own real post-switch step exactly - this is the faithful equivalent of
// "the same per-motor values", not a weakening of the test.

TEST_CASE("setup_motors: FrameClass::Quad dispatches to setup_quad_matrix", "[motors][setup_motors][quad]") {
    MotorsMatrix via_dispatch;
    via_dispatch.setup_motors(MotorsMatrix::FrameClass::Quad, MotorsMatrix::FrameType::Plus);

    MotorsMatrix direct;
    REQUIRE(direct.setup_quad_matrix(MotorsMatrix::FrameType::Plus));
    direct.normalise_rpy_factors();

    REQUIRE(via_dispatch.initialised_ok());
    REQUIRE(via_dispatch.frame_class_string() == direct.frame_class_string());
    REQUIRE(via_dispatch.frame_class_string() == "QUAD");
    REQUIRE(via_dispatch.frame_type_string() == direct.frame_type_string());
    requireSameMotorState(via_dispatch, direct);
}

TEST_CASE("setup_motors: FrameClass::Hexa dispatches to setup_hexa_matrix", "[motors][setup_motors][hexa]") {
    MotorsMatrix via_dispatch;
    via_dispatch.setup_motors(MotorsMatrix::FrameClass::Hexa, MotorsMatrix::FrameType::Plus);

    MotorsMatrix direct;
    REQUIRE(direct.setup_hexa_matrix(MotorsMatrix::FrameType::Plus));
    direct.normalise_rpy_factors();

    REQUIRE(via_dispatch.initialised_ok());
    REQUIRE(via_dispatch.frame_class_string() == direct.frame_class_string());
    REQUIRE(via_dispatch.frame_class_string() == "HEXA");
    REQUIRE(via_dispatch.frame_type_string() == direct.frame_type_string());
    requireSameMotorState(via_dispatch, direct);
}

TEST_CASE("setup_motors: FrameClass::Octa dispatches to setup_octa_matrix", "[motors][setup_motors][octa]") {
    MotorsMatrix via_dispatch;
    via_dispatch.setup_motors(MotorsMatrix::FrameClass::Octa, MotorsMatrix::FrameType::Plus);

    MotorsMatrix direct;
    REQUIRE(direct.setup_octa_matrix(MotorsMatrix::FrameType::Plus));
    direct.normalise_rpy_factors();

    REQUIRE(via_dispatch.initialised_ok());
    REQUIRE(via_dispatch.frame_class_string() == direct.frame_class_string());
    REQUIRE(via_dispatch.frame_class_string() == "OCTA");
    REQUIRE(via_dispatch.frame_type_string() == direct.frame_type_string());
    requireSameMotorState(via_dispatch, direct);
}

TEST_CASE("setup_motors: FrameClass::Octaquad dispatches to setup_octaquad_matrix", "[motors][setup_motors][octaquad]") {
    MotorsMatrix via_dispatch;
    via_dispatch.setup_motors(MotorsMatrix::FrameClass::Octaquad, MotorsMatrix::FrameType::Plus);

    MotorsMatrix direct;
    REQUIRE(direct.setup_octaquad_matrix(MotorsMatrix::FrameType::Plus));
    direct.normalise_rpy_factors();

    REQUIRE(via_dispatch.initialised_ok());
    REQUIRE(via_dispatch.frame_class_string() == direct.frame_class_string());
    REQUIRE(via_dispatch.frame_class_string() == "OCTAQUAD");
    REQUIRE(via_dispatch.frame_type_string() == direct.frame_type_string());
    requireSameMotorState(via_dispatch, direct);
}

TEST_CASE("setup_motors: FrameClass::Dodecahexa dispatches to setup_dodecahexa_matrix",
          "[motors][setup_motors][dodecahexa]") {
    MotorsMatrix via_dispatch;
    via_dispatch.setup_motors(MotorsMatrix::FrameClass::Dodecahexa, MotorsMatrix::FrameType::Plus);

    MotorsMatrix direct;
    REQUIRE(direct.setup_dodecahexa_matrix(MotorsMatrix::FrameType::Plus));
    direct.normalise_rpy_factors();

    REQUIRE(via_dispatch.initialised_ok());
    REQUIRE(via_dispatch.frame_class_string() == direct.frame_class_string());
    REQUIRE(via_dispatch.frame_class_string() == "DODECAHEXA");
    REQUIRE(via_dispatch.frame_type_string() == direct.frame_type_string());
    requireSameMotorState(via_dispatch, direct);
}

TEST_CASE("setup_motors: FrameClass::Y6 dispatches to setup_y6_matrix", "[motors][setup_motors][y6]") {
    // Uses FrameType::Y6B (a real explicit case in setup_y6_matrix's own
    // switch, not its productive default) as the representative frame
    // type - either would exercise the same dispatch, but Y6B avoids
    // relying on setup_y6_matrix's own separately-tested fallback
    // behavior here.
    MotorsMatrix via_dispatch;
    via_dispatch.setup_motors(MotorsMatrix::FrameClass::Y6, MotorsMatrix::FrameType::Y6B);

    MotorsMatrix direct;
    REQUIRE(direct.setup_y6_matrix(MotorsMatrix::FrameType::Y6B));
    direct.normalise_rpy_factors();

    REQUIRE(via_dispatch.initialised_ok());
    REQUIRE(via_dispatch.frame_class_string() == direct.frame_class_string());
    REQUIRE(via_dispatch.frame_class_string() == "Y6");
    REQUIRE(via_dispatch.frame_type_string() == direct.frame_type_string());
    requireSameMotorState(via_dispatch, direct);
}

TEST_CASE("setup_motors: FrameClass::Deca dispatches to setup_deca_matrix", "[motors][setup_motors][deca]") {
    MotorsMatrix via_dispatch;
    via_dispatch.setup_motors(MotorsMatrix::FrameClass::Deca, MotorsMatrix::FrameType::Plus);

    MotorsMatrix direct;
    REQUIRE(direct.setup_deca_matrix(MotorsMatrix::FrameType::Plus));
    direct.normalise_rpy_factors();

    REQUIRE(via_dispatch.initialised_ok());
    REQUIRE(via_dispatch.frame_class_string() == direct.frame_class_string());
    REQUIRE(via_dispatch.frame_class_string() == "DECA");
    REQUIRE(via_dispatch.frame_type_string() == direct.frame_type_string());
    requireSameMotorState(via_dispatch, direct);
}

TEST_CASE("setup_motors: an out-of-range FrameClass hits the real default branch, "
          "clears all motors, and sets frame_class_string to UNSUPPORTED",
          "[motors][setup_motors][default][unsupported]") {
    // Confirms the real upstream `default: success = false;` branch (see
    // file banner), followed unconditionally by normalise_rpy_factors()
    // (a harmless no-op here, since step 1 already cleared every motor)
    // and `frame_class_string_ = "UNSUPPORTED";`.
    MotorsMatrix m;
    m.setup_motors(static_cast<MotorsMatrix::FrameClass>(255), MotorsMatrix::FrameType::Plus);

    REQUIRE_FALSE(m.initialised_ok());
    REQUIRE(m.frame_class_string() == "UNSUPPORTED");
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
        REQUIRE(m.roll_factor(i) == Approx(0.0f));
        REQUIRE(m.pitch_factor(i) == Approx(0.0f));
        REQUIRE(m.yaw_factor(i) == Approx(0.0f));
        REQUIRE(m.throttle_factor(i) == Approx(0.0f));
    }
}

TEST_CASE("setup_motors: re-configuring with a DIFFERENT frame class/type leaves no stale motor state "
          "from the first call - THE SINGLE MOST IMPORTANT TEST THIS TICKET WRITES",
          "[motors][setup_motors][reconfigure]") {
    // This is exactly the risk the ticket calls out: the real upstream
    // structure unconditionally removes ALL motor slots before rebuilding
    // (see file banner) specifically so a second, different setup_motors
    // call is a genuine full reset, not an incremental merge on top of
    // the first call's own state. A port that skipped or narrowed the
    // remove-all step (e.g. only clearing slots the NEW frame table is
    // about to populate) would leak the first call's extra motor slots
    // into the second configuration - octa's own 8 populated slots (0-7)
    // vs quad's own 4 (0-3) makes any such leak directly observable on
    // slots 4-7.
    MotorsMatrix m;

    // First call: octa, 8 motors (0-7) all enabled.
    m.setup_motors(MotorsMatrix::FrameClass::Octa, MotorsMatrix::FrameType::Plus);
    REQUIRE(m.initialised_ok());
    for (std::uint8_t i = 0; i < 8; ++i) {
        REQUIRE(m.motor_enabled(i));
    }

    // Second call: a DIFFERENT real frame class AND frame type - quad, X.
    m.setup_motors(MotorsMatrix::FrameClass::Quad, MotorsMatrix::FrameType::X);
    REQUIRE(m.initialised_ok());
    REQUIRE(m.frame_class_string() == "QUAD");
    REQUIRE(m.frame_type_string() == "X");

    // The final state's motor_enabled flag and all four RPYT factor
    // arrays must match a fresh instance configured ONLY via quad/X,
    // across EVERY motor slot - proving no enabled motor or non-zero
    // roll/pitch/yaw/throttle factor survived from the first octa/Plus
    // call.
    MotorsMatrix reference;
    REQUIRE(reference.setup_quad_matrix(MotorsMatrix::FrameType::X));
    reference.normalise_rpy_factors();
    for (std::uint8_t i = 0; i < kMaxNumMotors; ++i) {
        INFO("motor index " << static_cast<int>(i));
        REQUIRE(m.motor_enabled(i) == reference.motor_enabled(i));
        REQUIRE(m.roll_factor(i) == Approx(reference.roll_factor(i)).margin(1e-6));
        REQUIRE(m.pitch_factor(i) == Approx(reference.pitch_factor(i)).margin(1e-6));
        REQUIRE(m.yaw_factor(i) == Approx(reference.yaw_factor(i)).margin(1e-6));
        REQUIRE(m.throttle_factor(i) == Approx(reference.throttle_factor(i)).margin(1e-6));
    }
    // test_order for the four motors quad/X actively rebuilds (0-3) must
    // also match the reference exactly - proving those slots were fully
    // rebuilt, not merely re-enabled with stale ordering.
    for (std::uint8_t i = 0; i < 4; ++i) {
        REQUIRE(m.test_order(i) == reference.test_order(i));
    }

    // Explicitly, motors 4-7 (populated by the first octa call, never
    // touched by quad's own 4-motor table) must now be fully DISABLED
    // with all four RPYT factors zeroed - the direct proof no live motor
    // or non-zero factor leaked across the reconfiguration.
    for (std::uint8_t i = 4; i < 8; ++i) {
        REQUIRE_FALSE(m.motor_enabled(i));
        REQUIRE(m.roll_factor(i) == Approx(0.0f));
        REQUIRE(m.pitch_factor(i) == Approx(0.0f));
        REQUIRE(m.yaw_factor(i) == Approx(0.0f));
        REQUIRE(m.throttle_factor(i) == Approx(0.0f));
    }
    // REAL, RE-VERIFIED QUIRK, discovered by this exact test (confirmed
    // directly against upstream's own real remove_motor, AP_MotorsMatrix.cpp
    // line 545-552): remove_motor's own real body clears motor_enabled and
    // all FOUR RPYT factor arrays, but never touches _test_order - upstream
    // itself has no code path that resets a disabled motor's test_order.
    // This means motors 4-7's test_order values are the REAL, DELIBERATELY
    // STALE leftovers from the first octa/Plus call (see that frame table's
    // own testing_order column above: motor 4=8, 5=6, 6=7, 7=3) - not
    // reset to 0 - even though those motors are now fully disabled with
    // zeroed factors. This is upstream's own real, faithfully-reproduced
    // behavior, not a port bug: test_order is only ever consulted for
    // motor-test sequencing among ENABLED motors, so a stale value on a
    // disabled slot is harmless in practice, but a test asserting it reads
    // 0 here would be asserting behavior upstream itself does not provide.
    REQUIRE(m.test_order(4) == 8);
    REQUIRE(m.test_order(5) == 6);
    REQUIRE(m.test_order(6) == 7);
    REQUIRE(m.test_order(7) == 3);
}

// ---------------------------------------------------------------------
// check_for_failed_motor (CCP-011) - port of upstream
// AP_MotorsMatrix::check_for_failed_motor (real function lines 414-461).
// See motors_matrix.hpp's own file banner ("CCP-011 ADDITION") for the
// full six-step structure, the disclosed motor_lost_index_ bug-fix
// (real upstream's own _motor_lost_index is indeterminate until first
// write - this port gives it a defined 0 instead), and the
// active_frame_type_/thr_lin_/throttle_thrust_max design decisions.
//
// Every test below builds its own enabled-motor set directly via
// add_motor_raw (never via setup_motors/setup_*_matrix) so the motor
// count and per-motor thrust are fully explicit and independent of any
// particular frame table. Most tests pass a deliberately huge dt_s
// (1.0e6f) so alpha = dt_s / (dt_s + 0.5f) is within ~5e-7 of 1.0 -
// this snaps thrust_rpyt_out_filt_ to thrust_rpyt_out_ in a single call,
// isolating the balance/lost-index/thrust-boost logic under test from
// the filter's own convergence behavior (covered separately by the
// first test below).
// ---------------------------------------------------------------------

TEST_CASE("check_for_failed_motor: thrust_boost_ defaults to false and thrust_balanced_ defaults to true, "
          "matching upstream's real AP_Motors_Class.cpp constructor (lines 54-55)",
          "[motors][check_for_failed_motor]") {
    MotorsMatrix m;
    REQUIRE_FALSE(m.thrust_boost());
    REQUIRE(m.thrust_balanced());
}

TEST_CASE("check_for_failed_motor: motor_lost_index_ starts at a defined value (0), not indeterminate - "
          "regression test for the disclosed upstream uninitialized-read bug fix (see file banner)",
          "[motors][check_for_failed_motor][bugfix]") {
    MotorsMatrix m;
    // Fresh instance, before check_for_failed_motor has ever run and
    // before thrust_boost_ has ever been set true. Real upstream's own
    // _motor_lost_index (AP_MotorsMatrix.h line 150) has no in-class
    // initializer and is never assigned in the constructor, so it would
    // read as indeterminate memory here - this port's own
    // motor_lost_index_ must instead read a real, defined 0.
    REQUIRE(m.motor_lost_index() == 0);
}

TEST_CASE("check_for_failed_motor: alpha uses the real dt_s/(dt_s+0.5f) formula, not a generic lowpass "
          "helper, and the filter converges toward a fixed thrust_rpyt_out over repeated calls",
          "[motors][check_for_failed_motor]") {
    MotorsMatrix m;
    m.add_motor_raw(0, 0.0f, 0.0f, 0.0f, 1);
    m.set_thrust_rpyt_out(0, 1.0f);

    const float dt_s = 0.1f;
    const float alpha = dt_s / (dt_s + 0.5f);
    REQUIRE(alpha == Approx(1.0f / 6.0f).margin(1e-6f));

    // filt_n = filt_{n-1} + alpha * (1.0 - filt_{n-1}), starting at 0 -
    // computed independently here (not via a closed form) to mirror the
    // real per-call update exactly.
    float expected_filt = 0.0f;
    for (int call = 0; call < 5; ++call) {
        m.check_for_failed_motor(/*throttle_thrust_best_plus_adj=*/0.0f, /*throttle_thrust_max=*/0.0f, dt_s,
                                  /*air_density_ratio=*/1.0f);
        expected_filt += alpha * (1.0f - expected_filt);
        REQUIRE(m.thrust_rpyt_out_filt(0) == Approx(expected_filt).margin(1e-6f));
    }
    // Monotonically approaching 1.0 from below over these five calls,
    // never overshooting - real exponential-filter convergence behavior.
    REQUIRE(m.thrust_rpyt_out_filt(0) < 1.0f);
    REQUIRE(m.thrust_rpyt_out_filt(0) > 0.5f);
}

TEST_CASE("check_for_failed_motor: 6+ motor imbalance (ratio >= 1.5) trips thrust_balanced_ to false, "
          "and a later balanced call (ratio <= 1.25) recovers it to true",
          "[motors][check_for_failed_motor]") {
    MotorsMatrix m;
    for (std::uint8_t i = 0; i < 6; ++i) {
        m.add_motor_raw(static_cast<std::int8_t>(i), 0.0f, 0.0f, 0.0f, static_cast<std::uint8_t>(i + 1));
    }
    REQUIRE(m.thrust_balanced()); // real upstream default (AP_Motors_Class.cpp line 55)

    // Motor 0 at 2.0, the other five at 1.0: rpyt_high=2.0, rpyt_sum=7.0,
    // number_motors=6 -> thrust_balance = 2.0*6/7.0 ~= 1.714, well above
    // the 1.5 trip threshold.
    const float huge_dt_s = 1.0e6f;
    m.set_thrust_rpyt_out(0, 2.0f);
    for (std::uint8_t i = 1; i < 6; ++i) {
        m.set_thrust_rpyt_out(i, 1.0f);
    }
    m.check_for_failed_motor(0.0f, 0.0f, huge_dt_s, 1.0f);
    REQUIRE_FALSE(m.thrust_balanced());

    // Recovery: all six motors now balanced at 1.0 -> thrust_balance ==
    // 1.0, at/below the 1.25 recovery threshold.
    for (std::uint8_t i = 0; i < 6; ++i) {
        m.set_thrust_rpyt_out(i, 1.0f);
    }
    m.check_for_failed_motor(0.0f, 0.0f, huge_dt_s, 1.0f);
    REQUIRE(m.thrust_balanced());
}

TEST_CASE("check_for_failed_motor: a ratio strictly between 1.25 and 1.5 is a real dead zone - it "
          "changes thrust_balanced_ in NEITHER direction",
          "[motors][check_for_failed_motor]") {
    MotorsMatrix m;
    for (std::uint8_t i = 0; i < 6; ++i) {
        m.add_motor_raw(static_cast<std::int8_t>(i), 0.0f, 0.0f, 0.0f, static_cast<std::uint8_t>(i + 1));
    }
    const float huge_dt_s = 1.0e6f;

    // Motor 0 at 1.45, the other five at 1.0: thrust_balance ==
    // 6*1.45/(1.45+5.0) ~= 1.3488, strictly inside (1.25, 1.5).
    auto set_dead_zone_thrusts = [&m]() {
        m.set_thrust_rpyt_out(0, 1.45f);
        for (std::uint8_t i = 1; i < 6; ++i) {
            m.set_thrust_rpyt_out(i, 1.0f);
        }
    };

    // Starting thrust_balanced_ == true (the default): the dead-zone
    // ratio does not satisfy `thrust_balance >= 1.5f`, so it must stay
    // true.
    set_dead_zone_thrusts();
    REQUIRE(m.thrust_balanced());
    m.check_for_failed_motor(0.0f, 0.0f, huge_dt_s, 1.0f);
    REQUIRE(m.thrust_balanced());

    // Starting thrust_balanced_ == false: the same dead-zone ratio does
    // not satisfy `thrust_balance <= 1.25f` either, so it must stay
    // false.
    m.set_thrust_balanced(false);
    set_dead_zone_thrusts();
    m.check_for_failed_motor(0.0f, 0.0f, huge_dt_s, 1.0f);
    REQUIRE_FALSE(m.thrust_balanced());
}

TEST_CASE("check_for_failed_motor: co-rotating frame types (XCor/CwXCor) suppress the imbalance trip "
          "that would otherwise fire for the exact same inputs",
          "[motors][check_for_failed_motor]") {
    const float huge_dt_s = 1.0e6f;

    MotorsMatrix m_xcor;
    for (std::uint8_t i = 0; i < 6; ++i) {
        m_xcor.add_motor_raw(static_cast<std::int8_t>(i), 0.0f, 0.0f, 0.0f, static_cast<std::uint8_t>(i + 1));
    }
    m_xcor.set_active_frame_type(MotorsMatrix::FrameType::XCor);
    m_xcor.set_thrust_rpyt_out(0, 2.0f); // same imbalanced scenario as the
    for (std::uint8_t i = 1; i < 6; ++i) { // >= 1.5 trip test above
        m_xcor.set_thrust_rpyt_out(i, 1.0f);
    }
    m_xcor.check_for_failed_motor(0.0f, 0.0f, huge_dt_s, 1.0f);
    REQUIRE(m_xcor.thrust_balanced()); // still true - is_corotating suppressed the trip

    MotorsMatrix m_cwxcor;
    for (std::uint8_t i = 0; i < 6; ++i) {
        m_cwxcor.add_motor_raw(static_cast<std::int8_t>(i), 0.0f, 0.0f, 0.0f, static_cast<std::uint8_t>(i + 1));
    }
    m_cwxcor.set_active_frame_type(MotorsMatrix::FrameType::CwXCor);
    m_cwxcor.set_thrust_rpyt_out(0, 2.0f);
    for (std::uint8_t i = 1; i < 6; ++i) {
        m_cwxcor.set_thrust_rpyt_out(i, 1.0f);
    }
    m_cwxcor.check_for_failed_motor(0.0f, 0.0f, huge_dt_s, 1.0f);
    REQUIRE(m_cwxcor.thrust_balanced());
}

TEST_CASE("check_for_failed_motor: fewer than 6 enabled motors never trips thrust_balanced_ to false, "
          "regardless of ratio",
          "[motors][check_for_failed_motor]") {
    MotorsMatrix m;
    for (std::uint8_t i = 0; i < 5; ++i) {
        m.add_motor_raw(static_cast<std::int8_t>(i), 0.0f, 0.0f, 0.0f, static_cast<std::uint8_t>(i + 1));
    }
    // Motor 0 at 2.0, the other four at 1.0: rpyt_high=2.0, rpyt_sum=6.0,
    // number_motors=5 -> thrust_balance = 2.0*5/6.0 ~= 1.667, itself well
    // above the 1.5 threshold - but number_motors < 6 must suppress the
    // trip regardless.
    m.set_thrust_rpyt_out(0, 2.0f);
    for (std::uint8_t i = 1; i < 5; ++i) {
        m.set_thrust_rpyt_out(i, 1.0f);
    }
    m.check_for_failed_motor(0.0f, 0.0f, 1.0e6f, 1.0f);
    REQUIRE(m.thrust_balanced());
}

TEST_CASE("check_for_failed_motor: motor_lost_index_ tracks the highest-filtered motor across calls, "
          "and is frozen while thrust_boost_ is true",
          "[motors][check_for_failed_motor]") {
    MotorsMatrix m;
    for (std::uint8_t i = 0; i < 6; ++i) {
        m.add_motor_raw(static_cast<std::int8_t>(i), 0.0f, 0.0f, 0.0f, static_cast<std::uint8_t>(i + 1));
    }
    const float huge_dt_s = 1.0e6f;

    // Motor 3 is highest.
    for (std::uint8_t i = 0; i < 6; ++i) {
        m.set_thrust_rpyt_out(i, i == 3 ? 5.0f : 1.0f);
    }
    m.check_for_failed_motor(0.0f, 0.0f, huge_dt_s, 1.0f);
    REQUIRE(m.motor_lost_index() == 3);

    // Now motor 5 becomes highest - motor_lost_index_ must move to 5,
    // since thrust_boost_ is still false.
    REQUIRE_FALSE(m.thrust_boost());
    m.set_thrust_rpyt_out(5, 9.0f);
    m.check_for_failed_motor(0.0f, 0.0f, huge_dt_s, 1.0f);
    REQUIRE(m.motor_lost_index() == 5);

    // Freeze: enable thrust_boost_, then make motor 0 the new highest -
    // motor_lost_index_ must NOT move off 5 ("hold motor lost index
    // constant while thrust boost is active" - upstream's own comment).
    m.set_thrust_boost(true);
    m.set_thrust_rpyt_out(0, 20.0f);
    m.check_for_failed_motor(0.0f, 0.0f, huge_dt_s, 1.0f);
    REQUIRE(m.motor_lost_index() == 5);
}

TEST_CASE("check_for_failed_motor: sets thrust_boost_ false when throttle headroom is available, "
          "rpyt_high is below 0.9, and thrust is balanced - and only ever sets it false, never true",
          "[motors][check_for_failed_motor]") {
    MotorsMatrix m;
    for (std::uint8_t i = 0; i < 6; ++i) {
        m.add_motor_raw(static_cast<std::int8_t>(i), 0.0f, 0.0f, 0.0f, static_cast<std::uint8_t>(i + 1));
        m.set_thrust_rpyt_out(i, 0.1f); // rpyt_high stays well below 0.9 after filtering
    }
    m.set_thrust_boost(true);
    REQUIRE(m.thrust_balanced()); // default true, unaffected by this balanced input

    // air_density_ratio=1.0f -> get_compensation_gain() == 1.0f here:
    // lift_max_ defaults to 1.0 (1/lift_max_ == 1.0), and 1.0 is inside
    // the (0.3, 1.5) density gate but constrain_value(1.0, 0.5, 1.25) ==
    // 1.0 is itself a no-op multiplier - so the first condition reduces
    // to throttle_thrust_max > throttle_thrust_best_plus_adj.
    m.check_for_failed_motor(/*throttle_thrust_best_plus_adj=*/0.5f, /*throttle_thrust_max=*/1.0f, 1.0e6f,
                              /*air_density_ratio=*/1.0f);
    REQUIRE_FALSE(m.thrust_boost());

    // Once false, feed the exact same inputs again (every condition this
    // function checks is satisfied) - thrust_boost_ must stay false,
    // since this function never assigns it true anywhere.
    m.check_for_failed_motor(0.5f, 1.0f, 1.0e6f, 1.0f);
    REQUIRE_FALSE(m.thrust_boost());

    // Negative control: starting thrust_boost_ true again, but with the
    // headroom condition FALSE (throttle_thrust_max below
    // throttle_thrust_best_plus_adj) - thrust_boost_ must remain true,
    // proving the three-way AND is not vacuously satisfied.
    MotorsMatrix m2;
    for (std::uint8_t i = 0; i < 6; ++i) {
        m2.add_motor_raw(static_cast<std::int8_t>(i), 0.0f, 0.0f, 0.0f, static_cast<std::uint8_t>(i + 1));
        m2.set_thrust_rpyt_out(i, 0.1f);
    }
    m2.set_thrust_boost(true);
    m2.check_for_failed_motor(/*throttle_thrust_best_plus_adj=*/0.9f, /*throttle_thrust_max=*/0.5f, 1.0e6f, 1.0f);
    REQUIRE(m2.thrust_boost());
}


// ---------------------------------------------------------------------
// set_actuator_with_slew (CCP-012) - upstream AP_MotorsMulticopter's real
// function body, AP_MotorsMulticopter.cpp lines 480-503. See
// motors_matrix.hpp's file banner ("CCP-012 ADDITION") for the full
// formula derivation, the real current-output-vs-destination bounding
// pitfall (COP-004's own finding), and the resolved "no SHUT_DOWN check"
// non-bug.
// ---------------------------------------------------------------------

TEST_CASE("set_actuator_with_slew: real AP_MOTORS_SLEW_TIME_DEFAULT (0.0f, both directions) applies NO "
          "limiting regardless of how large the jump is",
          "[motors][set_actuator_with_slew]") {
    // AP_MOTORS_SLEW_TIME_DEFAULT = 0.0f (AP_MotorsMulticopter.h line 21,
    // re-verified directly) - both MOT_SLEW_UP_TIME/MOT_SLEW_DN_TIME
    // default to this, and is_positive(0.0f) is false, so neither branch
    // ever executes: output_slew_limit_up/_dn stay at their "no limit"
    // defaults (1.0f/0.0f) regardless of dt_s or the jump size.
    const float slew_up_time = 0.0f;
    const float slew_dn_time = 0.0f;
    const float dt_s = 0.02f;

    float actuator_output = 0.0f;
    MotorsMatrix::set_actuator_with_slew(actuator_output, 1.0f, slew_up_time, slew_dn_time, dt_s);
    REQUIRE(actuator_output == Approx(1.0f)); // full jump, instantly

    actuator_output = 1.0f;
    MotorsMatrix::set_actuator_with_slew(actuator_output, 0.0f, slew_up_time, slew_dn_time, dt_s);
    REQUIRE(actuator_output == Approx(0.0f)); // full drop, instantly
}

TEST_CASE("set_actuator_with_slew: limits are computed from the CURRENT actuator_output, not from the "
          "destination input - a multi-iteration ramp toward a large target proves the step-by-step "
          "trajectory (COP-004's own test shape: a destination-relative port would jump straight to the "
          "target on the very first call instead of ramping in equal steps)",
          "[motors][set_actuator_with_slew]") {
    const float slew_up_time = 0.1f;
    const float dt_s = 0.02f;
    // output_delta_up_max = dt_s / constrain_value(slew_up_time, 0, 0.5) = 0.02 / 0.1 = 0.2 per call.
    const float expected_step = 0.2f;

    float actuator_output = 0.0f;
    const float target = 1.0f; // a large target, several steps away

    // Real formula: output_slew_limit_up = constrain_value(actuator_output + step, 0, 1), THEN
    // actuator_output = constrain_value(input, output_slew_limit_dn, output_slew_limit_up). slew_dn_time
    // is left at 0 (unset) here, so only the up-limit ever binds - down movement stays unlimited (see
    // the dedicated up/down-independence test below).
    float expected = 0.0f;
    for (int i = 0; i < 5; ++i) {
        MotorsMatrix::set_actuator_with_slew(actuator_output, target, slew_up_time, /*slew_dn_time=*/0.0f, dt_s);
        expected += expected_step;
        if (expected > 1.0f) {
            expected = 1.0f;
        }
        REQUIRE(actuator_output == Approx(expected).margin(1e-6f));
    }
    // After exactly 5 steps of 0.2 each, the ramp has reached the target exactly (0.2, 0.4, 0.6, 0.8,
    // 1.0) - a genuine multi-call trajectory, not the instantaneous first-call jump to 1.0 a
    // destination-relative port (computing the limit from `input` instead of `actuator_output`) would
    // produce instead.
    REQUIRE(actuator_output == Approx(1.0f).margin(1e-6f));
}

TEST_CASE("set_actuator_with_slew: up-slew and down-slew are independent - setting only one leaves "
          "movement in the OTHER direction completely unlimited, at each direction's own configured rate",
          "[motors][set_actuator_with_slew]") {
    const float dt_s = 0.1f;

    // Only slew_up_time set (0.2s -> delta = 0.1/0.2 = 0.5 per call); slew_dn_time stays 0 (unlimited).
    {
        float actuator_output = 0.0f;
        MotorsMatrix::set_actuator_with_slew(actuator_output, 1.0f, /*slew_up_time=*/0.2f, /*slew_dn_time=*/0.0f, dt_s);
        REQUIRE(actuator_output == Approx(0.5f)); // limited going up

        // Coming back down from 0.5 to 0.0 in one call - fully unlimited since slew_dn_time is 0.
        MotorsMatrix::set_actuator_with_slew(actuator_output, 0.0f, /*slew_up_time=*/0.2f, /*slew_dn_time=*/0.0f, dt_s);
        REQUIRE(actuator_output == Approx(0.0f));
    }

    // Only slew_dn_time set (0.4s -> delta = 0.1/0.4 = 0.25 per call); slew_up_time stays 0 (unlimited).
    {
        float actuator_output = 1.0f;
        MotorsMatrix::set_actuator_with_slew(actuator_output, 0.0f, /*slew_up_time=*/0.0f, /*slew_dn_time=*/0.4f, dt_s);
        REQUIRE(actuator_output == Approx(0.75f)); // limited going down: 1.0 - 0.25

        // Going back up from 0.75 to 1.0 in one call - fully unlimited since slew_up_time is 0.
        MotorsMatrix::set_actuator_with_slew(actuator_output, 1.0f, /*slew_up_time=*/0.0f, /*slew_dn_time=*/0.4f, dt_s);
        REQUIRE(actuator_output == Approx(1.0f));
    }
}

TEST_CASE("set_actuator_with_slew: the real [0, 0.5] clamp on slew_up_time/slew_dn_time themselves - an "
          "absurdly large configured slew time behaves exactly as if clamped to 0.5s",
          "[motors][set_actuator_with_slew]") {
    const float dt_s = 0.1f;

    // slew_up_time = 1000.0f (absurdly large) must behave exactly as slew_up_time = 0.5f: both compute
    // output_delta_up_max = dt_s / 0.5 = 0.2.
    float actuator_output_absurd = 0.0f;
    MotorsMatrix::set_actuator_with_slew(actuator_output_absurd, 1.0f, /*slew_up_time=*/1000.0f, 0.0f, dt_s);

    float actuator_output_clamped = 0.0f;
    MotorsMatrix::set_actuator_with_slew(actuator_output_clamped, 1.0f, /*slew_up_time=*/0.5f, 0.0f, dt_s);

    REQUIRE(actuator_output_absurd == Approx(actuator_output_clamped));
    REQUIRE(actuator_output_absurd == Approx(0.2f));

    // Same for slew_dn_time.
    float dn_absurd = 1.0f;
    MotorsMatrix::set_actuator_with_slew(dn_absurd, 0.0f, 0.0f, /*slew_dn_time=*/1000.0f, dt_s);
    float dn_clamped = 1.0f;
    MotorsMatrix::set_actuator_with_slew(dn_clamped, 0.0f, 0.0f, /*slew_dn_time=*/0.5f, dt_s);
    REQUIRE(dn_absurd == Approx(dn_clamped));
    REQUIRE(dn_absurd == Approx(0.8f)); // 1.0 - (dt_s / 0.5) = 1.0 - 0.2
}

// ---------------------------------------------------------------------
// actuator_spin_up_to_ground_idle (CCP-012) - upstream
// AP_MotorsMulticopter::actuator_spin_up_to_ground_idle, real function
// body AP_MotorsMulticopter.cpp lines 511-513. Takes spin_min directly
// (see motors_matrix.hpp's file banner "spin_min PARAMETER CORRECTION" -
// ThrustLinearization has no get_spin_min() accessor; spin_min is a plain
// public field on the caller-owned ThrustLinParams struct, CCP-010).
// ---------------------------------------------------------------------

TEST_CASE("actuator_spin_up_to_ground_idle: mid-range spin_up_ratio scales spin_min linearly",
          "[motors][actuator_spin_up_to_ground_idle]") {
    REQUIRE(MotorsMatrix::actuator_spin_up_to_ground_idle(0.5f, 0.2f) == Approx(0.1f));
    REQUIRE(MotorsMatrix::actuator_spin_up_to_ground_idle(1.0f, 0.2f) == Approx(0.2f));
    REQUIRE(MotorsMatrix::actuator_spin_up_to_ground_idle(0.0f, 0.2f) == Approx(0.0f));
}

TEST_CASE("actuator_spin_up_to_ground_idle: out-of-range spin_up_ratio is clamped to [0, 1] before scaling",
          "[motors][actuator_spin_up_to_ground_idle]") {
    // Negative clamps to 0.
    REQUIRE(MotorsMatrix::actuator_spin_up_to_ground_idle(-0.5f, 0.3f) == Approx(0.0f));
    // >1 clamps to 1 - matches real upstream's own ramp, which is stepped before it is checked and can
    // be fractionally past 1 for one iteration (COP-004's own note on the identical Rust port).
    REQUIRE(MotorsMatrix::actuator_spin_up_to_ground_idle(1.5f, 0.3f) == Approx(0.3f));
    REQUIRE(MotorsMatrix::actuator_spin_up_to_ground_idle(100.0f, 0.3f) == Approx(0.3f));
}

TEST_CASE("actuator_spin_up_to_ground_idle: depends on the caller-supplied spin_min parameter directly, "
          "not on any ThrustLinearization accessor - varying spin_min scales the output",
          "[motors][actuator_spin_up_to_ground_idle]") {
    // ThrustLinearization (CCP-010) has no get_spin_min() accessor - spin_min is a plain public field on
    // the caller-owned ThrustLinParams, taken directly as this function's own second parameter (see
    // motors_matrix.hpp's file banner, "spin_min PARAMETER CORRECTION"). Varying it here at a fixed
    // spin_up_ratio confirms the real dependency without needing a MotorsMatrix/ThrustLinearization
    // instance at all - this is a static, dependency-free function.
    const float ratio = 0.6f;
    REQUIRE(MotorsMatrix::actuator_spin_up_to_ground_idle(ratio, 0.10f) == Approx(0.06f));
    REQUIRE(MotorsMatrix::actuator_spin_up_to_ground_idle(ratio, 0.15f) == Approx(0.09f));
    REQUIRE(MotorsMatrix::actuator_spin_up_to_ground_idle(ratio, 0.20f) == Approx(0.12f));
}

// ---------------------------------------------------------------------
// output_logic (PART 1 ONLY, CCP-013) - upstream
// AP_MotorsMulticopter::output_logic, real function body lines 591-768
// of the real 591-884 span. See motors_matrix.hpp's own file banner
// "CCP-013 ADDITION" for the full structure, the corrected spool_state_/
// spool_desired_ investigation (suspected bug, found NOT to be one),
// the six-member uninitialized-read bug that WAS confirmed and fixed,
// and the real same-call spoolup_block read-after-write finding.
//
// Per COP-004's own step-by-step verification philosophy (reused here,
// see that file's own tracker notes, 2026-08-25: "A state machine can
// agree at the endpoints and disagree in the middle... only a per-step
// comparison sees that."), these tests drive the machine through many
// successive calls and assert INTERMEDIATE state at each step, not just
// final outcomes.
//
// Signature reminder (see file banner for the full rationale): output_logic(
//   armed, interlock, disarm_disable_pwm, safe_time, spool_up_time&,
//   spool_down_time, spin_arm, idle_time_delay_s, spin_min, spoolup_block,
//   filtered_throttle, current_limit_max_throttle, dt_s, limits_all_engaged&,
//   should_set_spoolup_block&) - CCP-014 appended filtered_throttle/
//   current_limit_max_throttle (see file banner's "CCP-014 ADDITION") just
//   before dt_s. Every Part-1-only test below (none of which reaches
//   SPOOLING_UP/THROTTLE_UNLIMITED/SPOOLING_DOWN) passes the real
//   no-limiting default current_limit_max_throttle=1.0f COP-004 identified,
//   and an arbitrary filtered_throttle=1.0f that no Part-1 code path ever
//   reads.
// ---------------------------------------------------------------------

TEST_CASE("output_logic: disarming forces immediate SHUT_DOWN of both actual and desired state, from any "
          "prior state, with no ramp - the switch then runs the just-forced SHUT_DOWN case in the SAME call",
          "[motors][output_logic][safety]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::GroundIdle);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
    m.set_spin_up_ratio(0.5f);

    float spool_up_time = 0.5f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    m.output_logic(/*armed=*/false, /*interlock=*/true, /*disarm_disable_pwm=*/true, /*safe_time=*/1.0f, spool_up_time,
                    /*spool_down_time=*/0.5f, /*spin_arm=*/0.1f, /*idle_time_delay_s=*/0.5f, /*spin_min=*/0.15f,
                    /*spoolup_block=*/false, /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f,
                    /*dt_s=*/0.02f, limits_all_engaged, should_set_spoolup_block);

    REQUIRE(m.spool_desired() == MotorsMatrix::DesiredSpoolState::ShutDown);
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ShutDown);
    // The safety rule forces spool_state_ to ShutDown BEFORE the outer switch runs, so this same call
    // also executes the ShutDown case body, zeroing the ramps too - not merely flipping the enum.
    REQUIRE(m.spin_up_ratio() == Approx(0.0f));
    REQUIRE(m.disarm_safe_timer() == Approx(0.0f));
    REQUIRE(limits_all_engaged);
}

TEST_CASE("output_logic: an open interlock (while armed) forces the identical immediate SHUT_DOWN as "
          "disarming",
          "[motors][output_logic][safety]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::GroundIdle);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);

    float spool_up_time = 0.5f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    // armed=true but interlock=false, disarm_disable_pwm=false -> the disarm-safe-timer preamble takes
    // its OWN "armed" branch regardless (timer logic only cares about armed(), not interlock) and jumps
    // straight to safe_time since disarm_disable_pwm is false.
    m.output_logic(/*armed=*/true, /*interlock=*/false, /*disarm_disable_pwm=*/false, /*safe_time=*/1.0f, spool_up_time,
                    0.5f, 0.1f, 0.5f, 0.15f, false, /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f,
                    /*dt_s=*/0.02f, limits_all_engaged, should_set_spoolup_block);

    REQUIRE(m.spool_desired() == MotorsMatrix::DesiredSpoolState::ShutDown);
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ShutDown);
    REQUIRE(m.disarm_safe_timer() == Approx(1.0f));
}

TEST_CASE("output_logic: disarm-safe-timer accumulates by dt_s per call while armed with disarm_disable_pwm "
          "set, genuinely OVERSHOOTS past safe_time on the step that crosses it (the pre-increment check "
          "does not prevent overshoot), is clamped down to exactly safe_time on the NEXT call, and resets "
          "to 0 immediately on disarm",
          "[motors][output_logic][disarm_safe_timer]") {
    MotorsMatrix m;
    float spool_up_time = 0.5f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;

    const float safe_time = 0.25f;
    const float dt_s = 0.1f;
    auto step = [&](bool armed) {
        m.output_logic(armed, /*interlock=*/true, /*disarm_disable_pwm=*/true, safe_time, spool_up_time, 0.5f, 0.1f,
                        0.5f, 0.15f, false, /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f, dt_s,
                        limits_all_engaged, should_set_spoolup_block);
    };

    step(true);
    REQUIRE(m.disarm_safe_timer() == Approx(0.1f));
    step(true);
    REQUIRE(m.disarm_safe_timer() == Approx(0.2f));
    // Real overshoot: 0.2 < 0.25 going INTO this call, so the if-branch adds the full dt_s, landing at
    // 0.3 - past safe_time. Upstream's own check is against the PRE-increment value, so it does not
    // prevent a step from crossing the ceiling, only the FOLLOWING call's check catches it.
    step(true);
    REQUIRE(m.disarm_safe_timer() == Approx(0.3f));
    // Now 0.3 is not < 0.25, so the else-branch fires and clamps DOWN to exactly safe_time.
    step(true);
    REQUIRE(m.disarm_safe_timer() == Approx(0.25f));
    step(true);
    REQUIRE(m.disarm_safe_timer() == Approx(0.25f));
    // Disarming resets to 0 immediately, regardless of the accumulated value.
    step(false);
    REQUIRE(m.disarm_safe_timer() == Approx(0.0f));
}

TEST_CASE("output_logic: disarm_disable_pwm=false jumps the disarm-safe-timer straight to safe_time on the "
          "very first armed call - the timer is not exercised at all when PWM is not disabled while "
          "disarmed",
          "[motors][output_logic][disarm_safe_timer]") {
    MotorsMatrix m;
    float spool_up_time = 0.5f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    m.output_logic(true, true, /*disarm_disable_pwm=*/false, /*safe_time=*/0.75f, spool_up_time, 0.5f, 0.1f, 0.5f,
                    0.15f, false, /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.01f,
                    limits_all_engaged, should_set_spoolup_block);
    REQUIRE(m.disarm_safe_timer() == Approx(0.75f));
}

TEST_CASE("output_logic: spool_up_time below the real 0.05s minimum_spool_time floor is clamped UP in the "
          "CALLER's own variable - a genuine write-back, not a local copy",
          "[motors][output_logic][write_back]") {
    MotorsMatrix m;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;

    float spool_up_time_low = 0.01f; // below the 0.05f floor
    m.output_logic(true, true, false, 0.1f, spool_up_time_low, 0.5f, 0.1f, 0.5f, 0.15f, false, 1.0f, 1.0f, 0.02f,
                    limits_all_engaged, should_set_spoolup_block);
    REQUIRE(spool_up_time_low == Approx(0.05f)); // the CALLER's own variable is mutated

    // A value already above the floor is left untouched.
    float spool_up_time_ok = 0.3f;
    m.output_logic(true, true, false, 0.1f, spool_up_time_ok, 0.5f, 0.1f, 0.5f, 0.15f, false, 1.0f, 1.0f, 0.02f,
                    limits_all_engaged, should_set_spoolup_block);
    REQUIRE(spool_up_time_ok == Approx(0.3f));

    // Exactly at the floor is also left untouched - the real condition is strictly `<`.
    float spool_up_time_exact = 0.05f;
    m.output_logic(true, true, false, 0.1f, spool_up_time_exact, 0.5f, 0.1f, 0.5f, 0.15f, false, 1.0f, 1.0f, 0.02f,
                    limits_all_engaged, should_set_spoolup_block);
    REQUIRE(spool_up_time_exact == Approx(0.05f));
}

TEST_CASE("output_logic: SHUT_DOWN transitions to GROUND_IDLE only once BOTH the desired state has changed "
          "AND the disarm-safe-timer has elapsed - each condition tested independently",
          "[motors][output_logic][shutdown][ground_idle]") {
    // Case A: desired changed, but the safe-time delay has not elapsed yet - stays SHUT_DOWN.
    {
        MotorsMatrix m;
        m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
        float spool_up_time = 0.5f;
        bool limits_all_engaged = false;
        bool should_set_spoolup_block = false;
        m.output_logic(true, true, true, /*safe_time=*/1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.15f, false, 1.0f, 1.0f,
                        /*dt_s=*/0.1f, limits_all_engaged, should_set_spoolup_block);
        REQUIRE(m.disarm_safe_timer() == Approx(0.1f));
        REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ShutDown);
    }
    // Case B: the safe-time delay has elapsed, but desired is still SHUT_DOWN - stays SHUT_DOWN.
    {
        MotorsMatrix m;
        float spool_up_time = 0.5f;
        bool limits_all_engaged = false;
        bool should_set_spoolup_block = false;
        // desired stays the default ShutDown. dt_s > safe_time so the timer overshoots straight past it
        // on the very first call.
        m.output_logic(true, true, true, /*safe_time=*/0.1f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.15f, false, 1.0f, 1.0f,
                        /*dt_s=*/0.2f, limits_all_engaged, should_set_spoolup_block);
        REQUIRE(m.disarm_safe_timer() >= 0.1f);
        REQUIRE(m.spool_desired() == MotorsMatrix::DesiredSpoolState::ShutDown);
        REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ShutDown);
    }
    // Case C: both conditions true - transitions to GROUND_IDLE.
    {
        MotorsMatrix m;
        m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
        float spool_up_time = 0.5f;
        bool limits_all_engaged = false;
        bool should_set_spoolup_block = false;
        m.output_logic(true, true, true, /*safe_time=*/0.1f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.15f, false, 1.0f, 1.0f,
                        /*dt_s=*/0.2f, limits_all_engaged, should_set_spoolup_block);
        REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle);
    }
}

TEST_CASE("output_logic: SHUT_DOWN zeroes spin_up_ratio/throttle_thrust_max/idle_time and disables thrust "
          "boost every call, and sets the limits_all_engaged output",
          "[motors][output_logic][shutdown]") {
    MotorsMatrix m;
    m.set_spin_up_ratio(0.7f);
    m.set_throttle_thrust_max(0.4f);
    m.set_idle_time(0.2f);
    m.set_thrust_boost(true);
    m.set_thrust_boost_ratio(0.6f);

    float spool_up_time = 0.5f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.15f, false, 1.0f, 1.0f, 0.02f,
                    limits_all_engaged, should_set_spoolup_block);

    REQUIRE(m.spin_up_ratio() == Approx(0.0f));
    REQUIRE(m.throttle_thrust_max() == Approx(0.0f));
    REQUIRE(m.idle_time() == Approx(0.0f));
    REQUIRE_FALSE(m.thrust_boost());
    REQUIRE(m.thrust_boost_ratio() == Approx(0.0f));
    REQUIRE(limits_all_engaged);
    REQUIRE_FALSE(should_set_spoolup_block);
}

TEST_CASE("output_logic: GROUND_IDLE, desired SHUT_DOWN - down-ramps spin_up_ratio using spool_down_time "
          "and transitions to SHUT_DOWN exactly once the ratio reaches (and clamps to) 0, stepped call by "
          "call",
          "[motors][output_logic][ground_idle][shutdown_desired]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::GroundIdle);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ShutDown);
    m.set_spin_up_ratio(0.5f);

    // spin_arm/spin_min = 0.1/0.2 = 0.5 (only affects the idle-time bookkeeping here, not this branch's
    // own ramp formula). spool_down_time (0.4) is above the 0.05 floor, so it is used directly (no
    // fallback to spool_up_time). spool_step = dt_s / spool_down_time = 0.1 / 0.4 = 0.25 per call.
    float spool_up_time = 1.0f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    auto step = [&] {
        m.output_logic(true, true, false, 1.0f, spool_up_time, /*spool_down_time=*/0.4f, /*spin_arm=*/0.1f,
                        /*idle_time_delay_s=*/1.0f, /*spin_min=*/0.2f, false, 1.0f, 1.0f, /*dt_s=*/0.1f,
                        limits_all_engaged, should_set_spoolup_block);
    };

    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.25f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle);
    // The shared post-inner-switch reset still fires every call, regardless of inner path.
    REQUIRE(m.throttle_thrust_max() == Approx(0.0f));

    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.0f)); // clamped to exactly 0, not left slightly negative
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ShutDown);
}

TEST_CASE("output_logic: GROUND_IDLE, desired SHUT_DOWN - falls back to spool_up_time when spool_down_time "
          "is at or below the 0.05s floor, the real symmetry fallback",
          "[motors][output_logic][ground_idle][shutdown_desired]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::GroundIdle);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ShutDown);
    m.set_spin_up_ratio(0.5f);

    // spool_down_time (0.02) is NOT above the 0.05 floor, so upstream falls back to spool_up_time (0.5).
    // spool_step = dt_s / spool_up_time = 0.1 / 0.5 = 0.2.
    float spool_up_time = 0.5f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    m.output_logic(true, true, false, 1.0f, spool_up_time, /*spool_down_time=*/0.02f, 0.1f, 1.0f, 0.2f, false, 1.0f,
                    1.0f, 0.1f, limits_all_engaged, should_set_spoolup_block);
    REQUIRE(m.spin_up_ratio() == Approx(0.3f));
}

TEST_CASE("output_logic: GROUND_IDLE, desired THROTTLE_UNLIMITED - ramps spin_up_ratio up, HOLDS it at "
          "spin_up_ground_idle_ratio with a genuine early BREAK while the idle-time delay is still "
          "running (not merely a clamp), then completes spin-up and raises should_set_spoolup_block on "
          "exactly the transition call - which itself does NOT advance to SPOOLING_UP in the SAME call "
          "(the real same-call read-after-write), only on a LATER call once the caller reports the block "
          "cleared",
          "[motors][output_logic][ground_idle][throttle_unlimited_desired]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::GroundIdle);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
    m.set_spin_up_ratio(0.0f);
    m.set_idle_time(0.0f);

    // spin_up_ground_idle_ratio = spin_arm/spin_min = 0.1/0.2 = 0.5. spool_up_time = 0.5, dt_s = 0.1 ->
    // spool_step = 0.2 per call. idle_time_delay_s = 0.3.
    float spool_up_time = 0.5f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    auto step = [&](bool spoolup_block) {
        m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, /*spin_arm=*/0.1f, /*idle_time_delay_s=*/0.3f,
                        /*spin_min=*/0.2f, spoolup_block, 1.0f, 1.0f, /*dt_s=*/0.1f, limits_all_engaged,
                        should_set_spoolup_block);
    };

    // Calls 1-3: idle_time's own pre-check uses spin_up_ratio_ as it was BEFORE this call's own
    // increment, and only starts advancing once that pre-increment ratio reaches the 0.5 target - so
    // idle_time stays 0 through these three calls even though spin_up_ratio_ itself reaches 0.5 on call 3
    // (0.5 >= 0.5 is not evaluated until call 4's own pre-check).
    step(false);
    REQUIRE(m.spin_up_ratio() == Approx(0.2f));
    REQUIRE(m.idle_time() == Approx(0.0f));
    REQUIRE_FALSE(should_set_spoolup_block);

    step(false);
    REQUIRE(m.spin_up_ratio() == Approx(0.4f));
    REQUIRE(m.idle_time() == Approx(0.0f));

    step(false);
    // Ramped to 0.6 then held/clamped to the 0.5 ground-idle ratio (idle_time still < delay).
    REQUIRE(m.spin_up_ratio() == Approx(0.5f));
    REQUIRE(m.idle_time() == Approx(0.0f));

    // Call 4: pre-check now sees ratio 0.5 >= target 0.5 - idle_time starts advancing. Ramp still held at
    // 0.5 since idle_time (0.1) is still below the 0.3 delay.
    step(false);
    REQUIRE(m.idle_time() == Approx(0.1f));
    REQUIRE(m.spin_up_ratio() == Approx(0.5f));

    step(false);
    REQUIRE(m.idle_time() == Approx(0.2f));
    REQUIRE(m.spin_up_ratio() == Approx(0.5f));

    // Call 6: idle_time reaches exactly the 0.3 delay (capped, not left to overshoot: MIN(delay,
    // idle_time+dt_s)). The idle-time gate (idle_time < idle_time_delay_s) now reads FALSE - re-verify
    // this uses idle_time AFTER this call's own advance, so the delay clears on the SAME call it reaches
    // it, not one call later - so the hold/break is skipped THIS call: spin_up_ratio_ is left at its
    // ramped 0.7 rather than clamped back to 0.5. It is still below 1.0, though, so completion has NOT
    // happened yet - the ramp needs to keep climbing over several more calls once released from the hold,
    // exactly as it did while held (0.2 per call), before it ever reaches the completion branch.
    step(/*spoolup_block=*/false);
    REQUIRE(m.idle_time() == Approx(0.3f));
    REQUIRE(m.spin_up_ratio() == Approx(0.7f));
    REQUIRE_FALSE(m.spin_up_complete());
    REQUIRE_FALSE(should_set_spoolup_block);

    // Call 7: past the delay, ramping continues normally (idle_time already capped at 0.3).
    step(/*spoolup_block=*/false);
    REQUIRE(m.spin_up_ratio() == Approx(0.9f));
    REQUIRE_FALSE(m.spin_up_complete());

    // Call 8: ratio would ramp to 1.1 - not < 1.0, so it is clamped to exactly 1.0 and spin_up_complete_
    // makes its ONE, real transition to true. should_set_spoolup_block fires exactly on THIS call - but
    // the SPOOLING_UP transition does NOT fire in the same call (the real same-call read-after-write:
    // spoolup_block_now mirrors the just-raised block, so `spin_up_complete_ && !spoolup_block_now` is
    // false here even though the INPUT `spoolup_block` parameter itself is still false).
    step(/*spoolup_block=*/false);
    REQUIRE(m.spin_up_ratio() == Approx(1.0f));
    REQUIRE(m.spin_up_complete());
    REQUIRE(should_set_spoolup_block);
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle); // NOT SpoolingUp yet

    // Call 9: the caller now reports the block cleared (spoolup_block=false, reflecting the real vehicle
    // having lowered it after seeing should_set_spoolup_block from call 8) - now the transition fires.
    step(/*spoolup_block=*/false);
    REQUIRE_FALSE(should_set_spoolup_block); // only ever raised on the ONE transition call
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingUp);
}

TEST_CASE("output_logic: GROUND_IDLE, desired THROTTLE_UNLIMITED - a still-raised spoolup_block input holds "
          "the machine in GROUND_IDLE indefinitely even after spin-up completes",
          "[motors][output_logic][ground_idle][throttle_unlimited_desired]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::GroundIdle);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
    m.set_spin_up_ratio(1.0f);
    m.set_idle_time(1.0f); // already past the delay
    // set_spin_up_complete has no dedicated setter - reach it for real by taking one call to completion
    // first (spin_up_ratio already at 1.0, so the ramp immediately clamps and completes).
    float spool_up_time = 0.5f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.3f, 0.2f, /*spoolup_block=*/true, 1.0f, 1.0f,
                    0.1f, limits_all_engaged, should_set_spoolup_block);
    REQUIRE(m.spin_up_complete());
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle);

    // Several more calls with the block still raised - never advances.
    for (int i = 0; i < 5; ++i) {
        m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.3f, 0.2f, /*spoolup_block=*/true, 1.0f,
                        1.0f, 0.1f, limits_all_engaged, should_set_spoolup_block);
        REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle);
        REQUIRE_FALSE(should_set_spoolup_block); // only ever raised on the ONE transition call, long past now
    }
}

TEST_CASE("output_logic: GROUND_IDLE, desired GROUND_IDLE - asymmetric slew toward spin_up_ground_idle_ratio, "
          "up-limited by spool_up_step, stepped call by call until it settles exactly at the target",
          "[motors][output_logic][ground_idle][ground_idle_desired]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::GroundIdle);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::GroundIdle);
    m.set_spin_up_ratio(0.0f);

    // target = spin_arm/spin_min = 0.45/0.5 = 0.9. spool_up_step = dt_s/spool_up_time = 0.1/0.5 = 0.2.
    float spool_up_time = 0.5f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    auto step = [&] {
        m.output_logic(true, true, false, 1.0f, spool_up_time, /*spool_down_time=*/0.3f, /*spin_arm=*/0.45f,
                        /*idle_time_delay_s=*/1.0f, /*spin_min=*/0.5f, false, 1.0f, 1.0f, /*dt_s=*/0.1f,
                        limits_all_engaged, should_set_spoolup_block);
    };

    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.2f)); // diff 0.9 clamped to the 0.2 up-step
    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.4f)); // diff 0.7, still clamped
    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.6f));
    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.8f));
    step();
    // diff is now 0.1, LESS than the 0.2 up-step - no longer clamped, lands exactly on the target.
    REQUIRE(m.spin_up_ratio() == Approx(0.9f));
    step();
    // Settled: diff 0 stays at the target.
    REQUIRE(m.spin_up_ratio() == Approx(0.9f));
}

TEST_CASE("output_logic: GROUND_IDLE, desired GROUND_IDLE - the DOWN direction is independently bounded by "
          "spool_down_step (from spool_down_time), not the up-step",
          "[motors][output_logic][ground_idle][ground_idle_desired]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::GroundIdle);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::GroundIdle);
    m.set_spin_up_ratio(0.9f);

    // target = spin_arm/spin_min = 0.05/0.5 = 0.1. spool_down_step = dt_s/spool_down_time = 0.1/0.2 = 0.5
    // (spool_up_step, from spool_up_time=1.0, would be 0.1 - deliberately very different from the down
    // step, to prove the down direction uses its OWN bound, not the up one).
    float spool_up_time = 1.0f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    auto step = [&] {
        m.output_logic(true, true, false, 1.0f, spool_up_time, /*spool_down_time=*/0.2f, /*spin_arm=*/0.05f,
                        /*idle_time_delay_s=*/1.0f, /*spin_min=*/0.5f, false, 1.0f, 1.0f, /*dt_s=*/0.1f,
                        limits_all_engaged, should_set_spoolup_block);
    };

    step();
    // diff = 0.1 - 0.9 = -0.8, clamped to -0.5 (the down-step) -> 0.9 - 0.5 = 0.4.
    REQUIRE(m.spin_up_ratio() == Approx(0.4f));
    step();
    // diff = 0.1 - 0.4 = -0.3, within the -0.5 bound (not clamped) -> lands exactly on the target.
    REQUIRE(m.spin_up_ratio() == Approx(0.1f));
}

TEST_CASE("output_logic: GROUND_IDLE's shared post-inner-switch reset (throttle_thrust_max_/thrust_boost_/"
          "thrust_boost_ratio_) fires identically regardless of which of the three inner DesiredSpoolState "
          "paths ran",
          "[motors][output_logic][ground_idle]") {
    using Desired = MotorsMatrix::DesiredSpoolState;
    for (Desired desired : {Desired::ShutDown, Desired::ThrottleUnlimited, Desired::GroundIdle}) {
        MotorsMatrix m;
        m.set_spool_state(MotorsMatrix::SpoolState::GroundIdle);
        m.set_spool_desired(desired);
        m.set_spin_up_ratio(0.5f); // mid-ramp, so ShutDown's own case doesn't finish/transition this call
        m.set_throttle_thrust_max(0.7f);
        m.set_thrust_boost(true);
        m.set_thrust_boost_ratio(0.6f);

        float spool_up_time = 1.0f;
        bool limits_all_engaged = false;
        bool should_set_spoolup_block = false;
        m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 1.0f, 0.2f, false, 1.0f, 1.0f, 0.05f,
                        limits_all_engaged, should_set_spoolup_block);

        REQUIRE(m.throttle_thrust_max() == Approx(0.0f));
        REQUIRE_FALSE(m.thrust_boost());
        REQUIRE(m.thrust_boost_ratio() == Approx(0.0f));
        REQUIRE(limits_all_engaged);
    }
}

TEST_CASE("output_logic: a fresh MotorsMatrix starts with real, DEFINED initial values for every new "
          "CCP-013 member - spool_state_/spool_desired_ genuinely match real upstream's own AP_Motors "
          "constructor default (SHUT_DOWN for both, NOT a bug fix - see file banner), and the six "
          "members with a CONFIRMED upstream uninitialized-read bug (disarm_safe_timer_/spin_up_ratio_/"
          "throttle_thrust_max_/idle_time_/spin_up_complete_/thrust_boost_ratio_) are fixed to defined "
          "values here rather than left indeterminate",
          "[motors][output_logic][regression]") {
    MotorsMatrix m;
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ShutDown);
    REQUIRE(m.spool_desired() == MotorsMatrix::DesiredSpoolState::ShutDown);
    REQUIRE(m.disarm_safe_timer() == Approx(0.0f));
    REQUIRE(m.spin_up_ratio() == Approx(0.0f));
    REQUIRE(m.throttle_thrust_max() == Approx(0.0f));
    REQUIRE(m.idle_time() == Approx(0.0f));
    REQUIRE_FALSE(m.spin_up_complete());
    REQUIRE(m.thrust_boost_ratio() == Approx(0.0f));
}

TEST_CASE("output_logic: the real SpoolState/DesiredSpoolState enumerator values match upstream's own "
          "SCREAMING_CASE enum exactly (SHUT_DOWN=0, GROUND_IDLE=1, SPOOLING_UP=2, THROTTLE_UNLIMITED=3, "
          "SPOOLING_DOWN=4; SHUT_DOWN=0, GROUND_IDLE=1, THROTTLE_UNLIMITED=2)",
          "[motors][output_logic][enum]") {
    using SS = MotorsMatrix::SpoolState;
    using DS = MotorsMatrix::DesiredSpoolState;
    REQUIRE(static_cast<std::uint8_t>(SS::ShutDown) == 0);
    REQUIRE(static_cast<std::uint8_t>(SS::GroundIdle) == 1);
    REQUIRE(static_cast<std::uint8_t>(SS::SpoolingUp) == 2);
    REQUIRE(static_cast<std::uint8_t>(SS::ThrottleUnlimited) == 3);
    REQUIRE(static_cast<std::uint8_t>(SS::SpoolingDown) == 4);
    REQUIRE(static_cast<std::uint8_t>(DS::ShutDown) == 0);
    REQUIRE(static_cast<std::uint8_t>(DS::GroundIdle) == 1);
    REQUIRE(static_cast<std::uint8_t>(DS::ThrottleUnlimited) == 2);
}

// ---------------------------------------------------------------------
// CCP-014: output_logic PART 2 - SPOOLING_UP/THROTTLE_UNLIMITED/
// SPOOLING_DOWN (real lines 769-883, completing the real 591-883 span
// CCP-013 started). Same step-by-step philosophy as CCP-013's own tests
// above (see that section's own banner comment for the full rationale).
// Both new explicit parameters this ticket added - filtered_throttle and
// current_limit_max_throttle - are exercised directly below with a
// range of values, not pinned to a single constant throughout; COP-004's
// own real no-limiting default of current_limit_max_throttle=1.0f is
// used wherever a test does not specifically need a lower ceiling.
// ---------------------------------------------------------------------

TEST_CASE("output_logic: SPOOLING_UP ramps throttle_thrust_max by spool_step each call and transitions to "
          "THROTTLE_UNLIMITED exactly once the ceiling reaches min(filtered_throttle, current_limit_max_throttle), "
          "snapping the ceiling to current_limit_max_throttle itself - NOT to the min() used only for the "
          "comparison - at that moment",
          "[motors][output_logic][spooling_up]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::SpoolingUp);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
    m.set_throttle_thrust_max(0.0f);
    m.set_thrust_boost(true);
    m.set_thrust_boost_ratio(0.5f);

    // spool_step = dt_s/spool_up_time = 0.1/0.5 = 0.2. filtered_throttle (0.6) is deliberately the binding
    // limit here, BELOW current_limit_max_throttle (1.0), so the snap-to-1.0 on transition is visibly NOT a
    // snap to the 0.6 that actually triggered it.
    float spool_up_time = 0.5f;
    bool limits_all_engaged = true; // deliberately seeded true, to prove SpoolingUp sets it false
    bool should_set_spoolup_block = false;
    auto step = [&] {
        m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.2f, false,
                        /*filtered_throttle=*/0.6f, /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.1f,
                        limits_all_engaged, should_set_spoolup_block);
    };

    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.2f));
    REQUIRE(m.spin_up_ratio() == Approx(1.0f));
    REQUIRE_FALSE(limits_all_engaged); // "all limits released" - the opposite of ShutDown/GroundIdle's own true
    REQUIRE_FALSE(m.thrust_boost());
    REQUIRE(m.thrust_boost_ratio() == Approx(0.3f)); // 0.5 - 0.2
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingUp);

    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.4f));
    REQUIRE(m.thrust_boost_ratio() == Approx(0.1f)); // 0.3 - 0.2
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingUp);

    // Third call: 0.4 + 0.2 = 0.6, which meets min(filtered_throttle=0.6, current_limit_max_throttle=1.0)
    // exactly - the transition fires, snapping the ceiling to current_limit_max_throttle (1.0), NOT to the
    // 0.6 value that triggered it.
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(1.0f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ThrottleUnlimited);
    // The thrust-boost-ratio fade still ran this same call: max(0, 0.1 - 0.2) = 0, clamped.
    REQUIRE(m.thrust_boost_ratio() == Approx(0.0f));
}

TEST_CASE("output_logic: SPOOLING_UP's lower-bound guard clamps throttle_thrust_max to exactly 0.0 when "
          "negative, and re-verified to fire ONLY on that else branch, never in addition to the "
          "snap-to-ceiling transition",
          "[motors][output_logic][spooling_up]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::SpoolingUp);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
    // A pathological negative starting ceiling - real upstream's own lower-bound guard exists for exactly
    // this case, reached only when the transition condition itself is false.
    m.set_throttle_thrust_max(-0.5f);

    float spool_up_time = 1.0f; // spool_step = 0.1/1.0 = 0.1, small enough that -0.5+0.1=-0.4 stays negative
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.2f, false,
                    /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.1f, limits_all_engaged,
                    should_set_spoolup_block);
    // -0.5 + 0.1 = -0.4, which is NOT >= min(1.0, 1.0) = 1.0, so the transition does not fire - the else-if
    // guard clamps it to exactly 0.0 instead.
    REQUIRE(m.throttle_thrust_max() == Approx(0.0f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingUp);
}

TEST_CASE("output_logic: SPOOLING_UP's direction-correction is a genuine early BREAK - when desired drops "
          "below THROTTLE_UNLIMITED, it reverses to SPOOLING_DOWN immediately and skips the ramp/thrust-boost "
          "fade entirely for that call",
          "[motors][output_logic][spooling_up][direction_correction]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::SpoolingUp);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::GroundIdle); // anything but ThrottleUnlimited
    m.set_throttle_thrust_max(0.35f);
    m.set_thrust_boost(true);
    m.set_thrust_boost_ratio(0.4f);

    float spool_up_time = 0.5f;
    bool limits_all_engaged = true;
    bool should_set_spoolup_block = false;
    m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.2f, false, /*filtered_throttle=*/1.0f,
                    /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.1f, limits_all_engaged, should_set_spoolup_block);

    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingDown);
    REQUIRE_FALSE(limits_all_engaged); // still set before the break
    // Everything past the break line is untouched this call - not incremented, not faded.
    REQUIRE(m.throttle_thrust_max() == Approx(0.35f));
    REQUIRE(m.thrust_boost());
    REQUIRE(m.thrust_boost_ratio() == Approx(0.4f));
}

TEST_CASE("output_logic: THROTTLE_UNLIMITED's direction-correction is the SAME check as SPOOLING_UP's own - "
          "a separate, textually-duplicated guard in real upstream, not shared code - re-verified "
          "independently here with a current_limit_max_throttle deliberately lower than the pre-call ceiling, "
          "to prove the break really skips the reassignment rather than merely producing the same result",
          "[motors][output_logic][throttle_unlimited][direction_correction]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::ThrottleUnlimited);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::GroundIdle);
    m.set_throttle_thrust_max(0.9f);
    m.set_thrust_boost_ratio(0.4f);

    float spool_up_time = 0.5f;
    bool limits_all_engaged = true;
    bool should_set_spoolup_block = false;
    m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.2f, false, /*filtered_throttle=*/1.0f,
                    /*current_limit_max_throttle=*/0.3f, /*dt_s=*/0.1f, limits_all_engaged, should_set_spoolup_block);

    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingDown);
    REQUIRE_FALSE(limits_all_engaged);
    REQUIRE(m.throttle_thrust_max() == Approx(0.9f)); // NOT overwritten with current_limit_max_throttle (0.3)
    REQUIRE(m.thrust_boost_ratio() == Approx(0.4f));
}

TEST_CASE("output_logic: THROTTLE_UNLIMITED sets throttle_thrust_max to current_limit_max_throttle as a "
          "PLAIN ASSIGNMENT every call, never incremental - re-verified with a ceiling far from the prior "
          "value",
          "[motors][output_logic][throttle_unlimited]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::ThrottleUnlimited);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
    m.set_throttle_thrust_max(0.1f); // deliberately far from the target, to distinguish assignment from ramp

    float spool_up_time = 1.0f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.2f, false, /*filtered_throttle=*/1.0f,
                    /*current_limit_max_throttle=*/0.75f, /*dt_s=*/0.1f, limits_all_engaged, should_set_spoolup_block);

    // A ramp would have moved only by spool_step (0.1) toward 0.75; the real assignment lands on it exactly,
    // in one call.
    REQUIRE(m.throttle_thrust_max() == Approx(0.75f));
    REQUIRE(m.spin_up_ratio() == Approx(1.0f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ThrottleUnlimited);
}

TEST_CASE("output_logic: THROTTLE_UNLIMITED's thrust-boost-ratio management is a genuine if/else, not two "
          "independent ifs - each direction independently triggered and stepped across multiple calls",
          "[motors][output_logic][throttle_unlimited][thrust_boost]") {
    // Case A: thrust_boost_ && !thrust_balanced_ - ramps UP toward 1.0, clamped there.
    {
        MotorsMatrix m;
        m.set_spool_state(MotorsMatrix::SpoolState::ThrottleUnlimited);
        m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
        m.set_thrust_boost(true);
        m.set_thrust_balanced(false);
        m.set_thrust_boost_ratio(0.5f);

        // spool_step = dt_s/spool_up_time = 0.1/0.5 = 0.2 - used here for the boost ramp, NOT for throttle
        // (see this method's own CCP-014 comment on THROTTLE_UNLIMITED's spool_step).
        float spool_up_time = 0.5f;
        bool limits_all_engaged = false;
        bool should_set_spoolup_block = false;
        auto step = [&] {
            m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.2f, false,
                            /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.1f,
                            limits_all_engaged, should_set_spoolup_block);
        };

        step();
        REQUIRE(m.thrust_boost_ratio() == Approx(0.7f));
        step();
        REQUIRE(m.thrust_boost_ratio() == Approx(0.9f));
        // 0.9 + 0.2 = 1.1, clamped down to exactly 1.0.
        step();
        REQUIRE(m.thrust_boost_ratio() == Approx(1.0f));
        step();
        REQUIRE(m.thrust_boost_ratio() == Approx(1.0f)); // settled at the ceiling
    }
    // Case B: NOT (thrust_boost_ && !thrust_balanced_) - ramps DOWN toward 0, clamped there. Exercised via
    // thrust_boost_=false (thrust_balanced_ left at its own true default).
    {
        MotorsMatrix m;
        m.set_spool_state(MotorsMatrix::SpoolState::ThrottleUnlimited);
        m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
        m.set_thrust_boost(false);
        m.set_thrust_boost_ratio(0.5f);

        float spool_up_time = 0.5f;
        bool limits_all_engaged = false;
        bool should_set_spoolup_block = false;
        auto step = [&] {
            m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.2f, false,
                            /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.1f,
                            limits_all_engaged, should_set_spoolup_block);
        };

        step();
        REQUIRE(m.thrust_boost_ratio() == Approx(0.3f));
        step();
        REQUIRE(m.thrust_boost_ratio() == Approx(0.1f));
        // 0.1 - 0.2 = -0.1, clamped up to exactly 0.0.
        step();
        REQUIRE(m.thrust_boost_ratio() == Approx(0.0f));
    }
    // Case B, second flavor: thrust_boost_=true but thrust_balanced_=true still takes the else (down)
    // branch - proving the real condition is a genuine &&, not thrust_boost_ alone.
    {
        MotorsMatrix m;
        m.set_spool_state(MotorsMatrix::SpoolState::ThrottleUnlimited);
        m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
        m.set_thrust_boost(true);
        m.set_thrust_balanced(true);
        m.set_thrust_boost_ratio(0.5f);

        float spool_up_time = 0.5f;
        bool limits_all_engaged = false;
        bool should_set_spoolup_block = false;
        m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.2f, false,
                        /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.1f,
                        limits_all_engaged, should_set_spoolup_block);
        REQUIRE(m.thrust_boost_ratio() == Approx(0.3f)); // down, not up
    }
}

TEST_CASE("output_logic: SPOOLING_DOWN's direction-correction is the REVERSE of SPOOLING_UP/"
          "THROTTLE_UNLIMITED's own - desired returning to THROTTLE_UNLIMITED reverses to SPOOLING_UP "
          "immediately, skipping the down-ramp entirely for that call",
          "[motors][output_logic][spooling_down][direction_correction]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::SpoolingDown);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);
    m.set_throttle_thrust_max(0.6f);
    m.set_thrust_boost_ratio(0.4f);

    float spool_up_time = 0.5f;
    bool limits_all_engaged = true;
    bool should_set_spoolup_block = false;
    m.output_logic(true, true, false, 1.0f, spool_up_time, 0.5f, 0.1f, 0.5f, 0.2f, false, /*filtered_throttle=*/1.0f,
                    /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.1f, limits_all_engaged, should_set_spoolup_block);

    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingUp);
    REQUIRE_FALSE(limits_all_engaged);
    REQUIRE(m.throttle_thrust_max() == Approx(0.6f)); // unchanged - the break skips the down-ramp
    REQUIRE(m.thrust_boost_ratio() == Approx(0.4f));  // unchanged too
}

TEST_CASE("output_logic: SPOOLING_DOWN ramps throttle_thrust_max down by spool_step each call, using the "
          "SAME symmetry-fallback spool_time formula as GROUND_IDLE's own SHUT_DOWN-desired case, and holds "
          "spin_up_ratio at 1.0 throughout (spin reduction happens only in GROUND_IDLE, per real upstream's "
          "own comment)",
          "[motors][output_logic][spooling_down]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::SpoolingDown);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::GroundIdle);
    m.set_throttle_thrust_max(0.8f);
    m.set_thrust_boost_ratio(0.5f);

    // spool_down_time (0.3) is above the 0.05 floor, so it is used directly. spool_step = 0.1/0.3 = 1/3.
    // current_limit_max_throttle (1.0) stays well above the ramp throughout, so the snap-to-ceiling tail
    // branch never fires here - see the dedicated tail tests below for that branch.
    float spool_up_time = 1.0f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    auto step = [&] {
        m.output_logic(true, true, false, 1.0f, spool_up_time, /*spool_down_time=*/0.3f, 0.1f, 0.5f, 0.2f, false,
                        /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.1f,
                        limits_all_engaged, should_set_spoolup_block);
    };

    step();
    REQUIRE(m.spin_up_ratio() == Approx(1.0f));
    REQUIRE(m.throttle_thrust_max() == Approx(0.8f - 1.0f / 3.0f));
    REQUIRE(m.thrust_boost_ratio() == Approx(0.5f - 1.0f / 3.0f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingDown);

    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.8f - 2.0f / 3.0f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingDown);
}

TEST_CASE("output_logic: SPOOLING_DOWN's lower-bound clamp lands throttle_thrust_max at exactly 0.0 rather "
          "than slightly negative, and its two-branch tail's SECOND branch - transitioning to GROUND_IDLE - "
          "fires once the ceiling is exactly zero and the current limit is not itself binding",
          "[motors][output_logic][spooling_down]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::SpoolingDown);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::GroundIdle);
    m.set_throttle_thrust_max(0.4f);

    // current_limit_max_throttle (1.0) is well above the ramp, so the FIRST tail branch (snap-to-ceiling)
    // never fires - only the second (is_zero -> GROUND_IDLE) is exercised here.
    float spool_up_time = 1.0f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    auto step = [&] {
        m.output_logic(true, true, false, 1.0f, spool_up_time, /*spool_down_time=*/0.5f, 0.1f, 0.5f, 0.2f, false,
                        /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/1.0f, /*dt_s=*/0.1f,
                        limits_all_engaged, should_set_spoolup_block);
    };
    // spool_step = dt_s/spool_down_time = 0.1/0.5 = 0.2.
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.2f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingDown);

    step();
    // 0.2 - 0.2 = 0.0 exactly - the clamp fires (harmlessly, already exactly 0) and the is_zero branch
    // transitions to GROUND_IDLE.
    REQUIRE(m.throttle_thrust_max() == Approx(0.0f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle);
}

TEST_CASE("output_logic: SPOOLING_DOWN's tail FIRST branch - snapping down to current_limit_max_throttle "
          "when the ramp is still at/above it - fires independently when the current limit itself drops "
          "below the already-ramping-down ceiling mid-ramp",
          "[motors][output_logic][spooling_down]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::SpoolingDown);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::GroundIdle);
    m.set_throttle_thrust_max(0.8f);

    // current_limit_max_throttle (0.5) has dropped BELOW the already-ramping-down ceiling (0.8) - the real,
    // easy-to-miss case this tail branch exists for.
    float spool_up_time = 1.0f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    // spool_step = dt_s/spool_down_time = 0.1/0.5 = 0.2.
    m.output_logic(true, true, false, 1.0f, spool_up_time, /*spool_down_time=*/0.5f, 0.1f, 0.5f, 0.2f, false,
                    /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/0.5f, /*dt_s=*/0.1f, limits_all_engaged,
                    should_set_spoolup_block);

    // 0.8 - 0.2 = 0.6, which is >= current_limit_max_throttle (0.5) - the FIRST branch fires, snapping DOWN
    // to exactly 0.5, NOT the is_zero branch (0.5 is not zero) - stays SPOOLING_DOWN.
    REQUIRE(m.throttle_thrust_max() == Approx(0.5f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingDown);
}

TEST_CASE("output_logic: SPOOLING_DOWN's if/else-if tail is genuinely mutually exclusive, not two "
          "independent ifs - the one real edge case where BOTH conditions are simultaneously true "
          "(current_limit_max_throttle pinned to exactly 0.0, ramp reaching exactly 0.0 too) exercises this "
          "directly: only the FIRST branch runs (a no-op snap, since both values are 0.0), and the "
          "GROUND_IDLE transition the second branch would have fired is suppressed - a port using two "
          "independent ifs would incorrectly also transition to GROUND_IDLE here",
          "[motors][output_logic][spooling_down][mutual_exclusivity]") {
    MotorsMatrix m;
    m.set_spool_state(MotorsMatrix::SpoolState::SpoolingDown);
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::GroundIdle);
    m.set_throttle_thrust_max(0.2f);

    float spool_up_time = 1.0f;
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;
    // spool_step = dt_s/spool_down_time = 0.1/0.5 = 0.2, so 0.2 - 0.2 = 0.0 exactly.
    m.output_logic(true, true, false, 1.0f, spool_up_time, /*spool_down_time=*/0.5f, 0.1f, 0.5f, 0.2f, false,
                    /*filtered_throttle=*/1.0f, /*current_limit_max_throttle=*/0.0f, /*dt_s=*/0.1f, limits_all_engaged,
                    should_set_spoolup_block);

    REQUIRE(m.throttle_thrust_max() == Approx(0.0f));
    // The if fires first (0.0 >= 0.0), snapping to current_limit_max_throttle (0.0, a no-op value-wise) -
    // the else-if is then skipped, so the state does NOT advance to GROUND_IDLE despite is_zero(0.0) also
    // being true. This is the real structure's actual, disclosed behavior - copter-rust's own COP-004 port
    // notes this exact edge case is unreachable in practice (it needs a vehicle that can draw zero current
    // at all) but still faithfully reproduces the real if/else-if rather than assuming two independent ifs
    // would behave identically.
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingDown);
}

TEST_CASE("output_logic: a full, multi-state, multi-step integration test drives the COMPLETE real state "
          "machine through a realistic sequence - SHUT_DOWN -> GROUND_IDLE -> SPOOLING_UP -> "
          "THROTTLE_UNLIMITED -> SPOOLING_DOWN -> GROUND_IDLE - confirming every transition occurs at the "
          "expected call, not early or late; the first test exercising the whole real function end-to-end "
          "rather than one case in isolation",
          "[motors][output_logic][integration]") {
    MotorsMatrix m;
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::ThrottleUnlimited);

    const float safe_time = 0.2f;
    float spool_up_time = 0.5f;
    const float spool_down_time = 0.5f;
    const float spin_arm = 0.1f;
    const float idle_time_delay_s = 0.1f;
    const float spin_min = 0.5f; // spin_up_ground_idle_ratio = spin_arm/spin_min = 0.2
    const float filtered_throttle = 0.6f;
    const float current_limit_max_throttle = 1.0f; // COP-004's own real no-limiting default
    const float dt_s = 0.1f;
    bool spoolup_block = false; // never externally raised in this scenario
    bool limits_all_engaged = false;
    bool should_set_spoolup_block = false;

    auto step = [&] {
        m.output_logic(/*armed=*/true, /*interlock=*/true, /*disarm_disable_pwm=*/true, safe_time, spool_up_time,
                        spool_down_time, spin_arm, idle_time_delay_s, spin_min, spoolup_block, filtered_throttle,
                        current_limit_max_throttle, dt_s, limits_all_engaged, should_set_spoolup_block);
    };

    // Calls 1-2: SHUT_DOWN, waiting for disarm_safe_timer_ to reach safe_time (0.2s at 0.1s/call = 2 calls).
    step();
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ShutDown);
    step();
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle); // transitions exactly on call 2

    // Calls 3-6: GROUND_IDLE, spin_up_ratio_ ramping by spool_step=dt_s/spool_up_time=0.2/call, held at the
    // 0.2 ground-idle ratio by the idle-time-delay early break until idle_time_ itself reaches the 0.1s
    // delay (one call after spin_up_ratio_ first reaches 0.2, per GROUND_IDLE's own pre-increment idle-time
    // check).
    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.2f)); // held at the ground-idle ratio, idle_time_ still 0
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle);
    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.4f)); // idle_time_ reached 0.1 THIS call, so the ramp continues
    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.6f));
    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.8f));

    // Call 7: spin_up_ratio_ reaches exactly 1.0 - spin_up_complete_ becomes true and
    // should_set_spoolup_block fires, but the real same-call read-after-write means the SPOOLING_UP
    // transition does NOT fire yet.
    step();
    REQUIRE(m.spin_up_ratio() == Approx(1.0f));
    REQUIRE(m.spin_up_complete());
    REQUIRE(should_set_spoolup_block);
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle);

    // Call 8: spoolup_block is still (correctly) false going in, so the transition fires now.
    step();
    REQUIRE_FALSE(should_set_spoolup_block);
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingUp);

    // Calls 9-11: SPOOLING_UP, ramping throttle_thrust_max_ by spool_step=dt_s/spool_up_time=0.2/call
    // toward min(filtered_throttle=0.6, current_limit_max_throttle=1.0)=0.6.
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.2f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingUp);
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.4f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingUp);
    step();
    // 0.4 + 0.2 = 0.6, meets the threshold exactly - transitions to THROTTLE_UNLIMITED, snapping the
    // ceiling to current_limit_max_throttle (1.0), not to the 0.6 that triggered it.
    REQUIRE(m.throttle_thrust_max() == Approx(1.0f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ThrottleUnlimited);

    // Call 12: THROTTLE_UNLIMITED, steady state - ceiling tracks current_limit_max_throttle directly.
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(1.0f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::ThrottleUnlimited);

    // The pilot now requests GROUND_IDLE (e.g. landing) - the machine must spool down before it can settle.
    m.set_spool_desired(MotorsMatrix::DesiredSpoolState::GroundIdle);

    // Call 13: THROTTLE_UNLIMITED's own direction-correction fires immediately - reverses to SPOOLING_DOWN
    // and skips the (would-be) throttle_thrust_max_ reassignment for this call.
    step();
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingDown);
    REQUIRE(m.throttle_thrust_max() == Approx(1.0f)); // unchanged by the break

    // Calls 14-17: SPOOLING_DOWN, ramping throttle_thrust_max_ down by spool_step=dt_s/spool_down_time=0.2/
    // call; current_limit_max_throttle (1.0) stays above the ramp throughout, so the snap-to-ceiling tail
    // branch never fires here.
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.8f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::SpoolingDown);
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.6f));
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.4f));
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.2f));

    // Call 18: throttle_thrust_max_ reaches ~0.0 (a tiny FP residual from five successive subtractions
    // starting at 1.0, not a clean power-of-two relationship like the dedicated tail tests above - hence
    // the explicit .margin() here, matching this file's own established convention for near-zero targets
    // reached by accumulation rather than a single clamp assignment) - is_zero()'s own real FLT_EPSILON
    // threshold (see math::is_zero) is what the real code itself uses to decide the transition, and it
    // does fire: the is_zero tail branch transitions back to GROUND_IDLE, closing the full cycle.
    step();
    REQUIRE(m.throttle_thrust_max() == Approx(0.0f).margin(1e-4f));
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle);

    // Call 19, bonus confirmation: GROUND_IDLE's own desired-GROUND_IDLE path re-engages correctly, slewing
    // spin_up_ratio_ (still 1.0 from the whole SPOOLING_UP/THROTTLE_UNLIMITED/SPOOLING_DOWN run, since none
    // of those states touch it except to hold it at 1.0) back down toward the 0.2 ground-idle ratio, bounded
    // by spool_down_step=dt_s/spool_down_time=0.2.
    step();
    REQUIRE(m.spin_up_ratio() == Approx(0.8f)); // diff (0.2-1.0=-0.8) clamped to the -0.2 down-step
    REQUIRE(m.spool_state() == MotorsMatrix::SpoolState::GroundIdle);
}

// ---------------------------------------------------------------------
// output_to_pwm (CCP-015) - upstream AP_MotorsMulticopter::output_to_pwm,
// real function body lines 457-472. A static, pure function (see
// motors_matrix.hpp's own "CCP-015 ADDITION" comment) - no MotorsMatrix
// instance is needed to call it.
// ---------------------------------------------------------------------

TEST_CASE("output_to_pwm: SHUT_DOWN with disarm_disable_pwm=true and armed=false returns exactly 0, the "
          "real PWM-off case",
          "[motors][output_to_pwm][shut_down]") {
    const auto result = MotorsMatrix::output_to_pwm(/*actuator=*/0.5f, MotorsMatrix::SpoolState::ShutDown,
                                                     /*armed=*/false, /*disarm_disable_pwm=*/true,
                                                     /*pwm_output_min=*/1000, /*pwm_output_max=*/2000);
    REQUIRE(result == 0);
}

TEST_CASE("output_to_pwm: SHUT_DOWN with disarm_disable_pwm=false returns exactly pwm_output_min, not 0, "
          "even while disarmed",
          "[motors][output_to_pwm][shut_down]") {
    const auto result = MotorsMatrix::output_to_pwm(/*actuator=*/0.5f, MotorsMatrix::SpoolState::ShutDown,
                                                     /*armed=*/false, /*disarm_disable_pwm=*/false,
                                                     /*pwm_output_min=*/1000, /*pwm_output_max=*/2000);
    REQUIRE(result == 1000);
}

TEST_CASE("output_to_pwm: SHUT_DOWN with armed=true returns exactly pwm_output_min regardless of "
          "disarm_disable_pwm - the real `!armed()` condition, not `disarm_disable_pwm` alone, gates the "
          "PWM-off case",
          "[motors][output_to_pwm][shut_down]") {
    const auto result = MotorsMatrix::output_to_pwm(/*actuator=*/0.5f, MotorsMatrix::SpoolState::ShutDown,
                                                     /*armed=*/true, /*disarm_disable_pwm=*/true,
                                                     /*pwm_output_min=*/1000, /*pwm_output_max=*/2000);
    REQUIRE(result == 1000);
}

TEST_CASE("output_to_pwm: every non-SHUT_DOWN SpoolState performs the identical real linear remap - "
          "actuator=0.0 returns exactly pwm_output_min and actuator=1.0 returns exactly pwm_output_max, for "
          "GROUND_IDLE, SPOOLING_UP, THROTTLE_UNLIMITED, and SPOOLING_DOWN alike",
          "[motors][output_to_pwm][remap]") {
    const std::array<MotorsMatrix::SpoolState, 4> non_shut_down_states = {
        MotorsMatrix::SpoolState::GroundIdle,
        MotorsMatrix::SpoolState::SpoolingUp,
        MotorsMatrix::SpoolState::ThrottleUnlimited,
        MotorsMatrix::SpoolState::SpoolingDown,
    };
    for (const auto state : non_shut_down_states) {
        REQUIRE(MotorsMatrix::output_to_pwm(/*actuator=*/0.0f, state, /*armed=*/true,
                                             /*disarm_disable_pwm=*/true, /*pwm_output_min=*/1000,
                                             /*pwm_output_max=*/2000) == 1000);
        REQUIRE(MotorsMatrix::output_to_pwm(/*actuator=*/1.0f, state, /*armed=*/true,
                                             /*disarm_disable_pwm=*/true, /*pwm_output_min=*/1000,
                                             /*pwm_output_max=*/2000) == 2000);
    }
}

TEST_CASE("output_to_pwm: an interior actuator value matches the hand-computed real linear remap exactly - "
          "pwm_output_min=1000 + (2000-1000)*0.25 = 1250.0, no fractional remainder to truncate",
          "[motors][output_to_pwm][remap]") {
    const auto result = MotorsMatrix::output_to_pwm(/*actuator=*/0.25f, MotorsMatrix::SpoolState::ThrottleUnlimited,
                                                     /*armed=*/true, /*disarm_disable_pwm=*/true,
                                                     /*pwm_output_min=*/1000, /*pwm_output_max=*/2000);
    REQUIRE(result == 1250);
}

TEST_CASE("output_to_pwm: TRUNCATION, NOT ROUNDING - copter-rust's own COP-004 finding, independently "
          "re-verified directly against the real C++ source: real upstream's own `return pwm_output;` is an "
          "IMPLICIT float-to-int16_t conversion, which truncates toward zero rather than rounding to "
          "nearest. pwm_output_min=1000 + (2000-1000)*0.5007 = 1500.7 (fractional part 0.7, well above the "
          "0.5 rounding boundary) - a faithful truncating port returns exactly 1500. A port that used "
          "std::round()/std::lround() as an 'improvement' would return 1501 here instead and this assertion "
          "would fail, which is the entire point of this test",
          "[motors][output_to_pwm][truncation]") {
    const auto result = MotorsMatrix::output_to_pwm(/*actuator=*/0.5007f, MotorsMatrix::SpoolState::ThrottleUnlimited,
                                                     /*armed=*/true, /*disarm_disable_pwm=*/true,
                                                     /*pwm_output_min=*/1000, /*pwm_output_max=*/2000);
    REQUIRE(result == 1500);
    REQUIRE(result != 1501);
}
