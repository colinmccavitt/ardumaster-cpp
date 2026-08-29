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
