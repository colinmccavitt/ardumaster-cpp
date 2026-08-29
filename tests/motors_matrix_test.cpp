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

