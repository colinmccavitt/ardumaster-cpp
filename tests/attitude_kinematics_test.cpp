// Tests for fwcpp::control's attitude kinematics (CCP-018) - the four
// frame-conversion/limit primitives ported from AC_AttitudeControl.cpp's
// real lines 1206-1226 (ang_vel_limit), 1229-1248 (body_to_euler_limit),
// 1303-1316 (euler_derivative_to_body), and 1323-1342
// (body_to_euler_derivative). See attitude_kinematics.hpp's own file
// banner for the full design writeup, including the gimbal-lock finding
// this file's own tests exist specifically to pin down.
//
// Test cases translate essentially line-for-line from copter-rust's own
// already-merged, already parity-tested COP-007 test file
// (ports/plane-fw-rust/crates/ap-control/tests/attitude_kinematics.rs) -
// its real measured values/tolerances are reused directly here rather
// than re-derived, per this ticket's own explicit instruction.
//
// CCP-020 added the thrust_vector_rotation_angles tests below (real
// lines 1054-1103) - see attitude_kinematics.hpp's own "CCP-020
// ADDENDUM" comment block for the full design writeup: the thrust/
// heading urgency split, the second-quaternion heading decomposition,
// and the degenerate-case fallback.
//
// TEST-CONSTRUCTION PITFALL, deliberately avoided here (copter-rust's
// own COP-007 team fell into this and corrected it - reused directly):
// "pure heading" is NOT the same Euler roll/pitch with a different Euler
// yaw. Those angles are applied relative to the yawed frame, so on a
// leaning aircraft changing Euler yaw moves the thrust vector too - it
// is not heading-only. Every "pure heading" case below is instead built
// as `body * Quaternion::from_axis_angle((0,0,-1), delta)`: a LOCAL
// rotation about the body's own thrust axis (the constant (0,0,-1),
// which is the thrust direction in any body-fixed frame per this file's
// own banner), composed on the right of `body` - heading by
// construction, not by Euler approximation.
//
// Every case below is deliberately neither level nor north-facing
// (nonzero roll/pitch AND nonzero yaw somewhere in the pair), per
// copter-rust's own test philosophy: the leak these tests look for
// (thrust error leaking into yaw, or heading leaking into roll/pitch) is
// invisible at trivial/symmetric attitudes.
//
// The general-case and pure-lean expected thrust_angle_rad/thrust_error_
// angle_rad/attitude_error_rad values below were independently
// recomputed from the real upstream formulas in a from-scratch Python
// reimplementation (float32 arithmetic, matching this port's own
// operator*(Vector3)/from_euler/from_axis_angle/to_axis_angle formulas
// line-for-line but as an independent re-derivation, not copy-pasted
// from this header or run through it) - not merely re-running this
// port's own C++ output back at itself.

// CCP-022 added the attitude_command_model tests below (real lines
// 1108-1130) - see attitude_kinematics.hpp's own "CCP-022 ADDENDUM"
// comment block for the full design writeup: the two fallback defaults
// and why each exists, the jerk-limit direction, the exact argument
// mapping into shape_angle_vel_accel, and the real separate final
// integration step.
//
// Several of these tests use an EQUIVALENT-CALL comparison rather than
// hand-deriving an expected numeric output: one call with a fallback
// parameter (accel_max or input_tc) left invalid so the real internal
// default kicks in, compared bit-for-bit against a second call with
// that same parameter set EXPLICITLY to what the default is claimed to
// be, everything else held identical. shape_angle_vel_accel (and
// everything it calls) is a pure function of its arguments, so if the
// two calls produce bit-identical output, the internal default must
// have resolved to exactly the explicit value used in the second call -
// this is what lets the input_tc test pin "exactly dt * 10.0" without
// having to independently replicate the jerk-limiting arithmetic by
// hand and risk a reordering-induced rounding mismatch of its own.

#include <cmath>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

using namespace fwcpp::control;
using fwcpp::math::Quaternion;
using fwcpp::math::Vector3f;
using Catch::Approx;

namespace {

Quaternion attitude(float roll, float pitch, float yaw) {
    Quaternion q;
    q.from_euler(roll, pitch, yaw);
    return q;
}

} // namespace

// ---------------------------------------------------------------------
// Round trip: euler_derivative_to_body then body_to_euler_derivative
// recovers the original input, across several attitudes. Catches a sign
// error in either direction, and (unlike checking one direction alone)
// catches a sign error present in BOTH directions too, since they are
// not each other's negation.
// ---------------------------------------------------------------------

TEST_CASE("the derivative conversions are inverses across several attitudes", "[control][attitude_kinematics][round_trip]") {
    const Vector3f rate{0.35f, -0.20f, 0.55f};

    const float attitudes[][3] = {
        {0.0f, 0.0f, 0.0f},
        {0.4f, 0.2f, 1.1f},
        {-0.6f, 0.5f, -2.0f},
        {1.2f, -0.9f, 0.3f},
        {0.1f, 1.4f, 2.9f},
    };

    for (const auto& rpy : attitudes) {
        const Quaternion att = attitude(rpy[0], rpy[1], rpy[2]);
        const Vector3f body = euler_derivative_to_body(att, rate);

        Vector3f back{};
        REQUIRE(body_to_euler_derivative(att, body, back));

        REQUIRE(back.x == Approx(rate.x).margin(1e-4));
        REQUIRE(back.y == Approx(rate.y).margin(1e-4));
        REQUIRE(back.z == Approx(rate.z).margin(1e-4));
    }
}

// ---------------------------------------------------------------------
// At a level, wings-level attitude, the Euler and body frames coincide.
// ---------------------------------------------------------------------

TEST_CASE("at level attitude the euler and body frames agree", "[control][attitude_kinematics]") {
    const Quaternion att = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f rate{0.3f, -0.4f, 0.5f};

    const Vector3f body = euler_derivative_to_body(att, rate);
    REQUIRE(body.x == Approx(rate.x).margin(1e-6));
    REQUIRE(body.y == Approx(rate.y).margin(1e-6));
    REQUIRE(body.z == Approx(rate.z).margin(1e-6));
}

// ---------------------------------------------------------------------
// THE SINGLE MOST IMPORTANT TEST IN THIS FILE.
//
// Upstream's own doc comment claims body_to_euler_derivative "returns
// false if the vehicle is pitched 90 degrees up or down (gimbal lock)".
// In practice, at a genuinely-constructed exact 90-degree quaternion,
// this is false: the guard is is_zero(cos_theta), which wants |cos_theta|
// below FLT_EPSILON (~1.19e-7). get_euler_pitch() (an asin, numerically
// flat near its own domain endpoints) reads back a measured 1.570451
// rather than the true pi/2 = 1.5707963, so cos_theta lands around
// 3.45e-4 - roughly three thousand times too large to trip the guard.
// (Measured values reused directly from copter-rust's own COP-007
// investigation - see this file's own header comment.)
//
// The function returns TRUE here, with a large-but-finite result, NOT
// false. A "fixed" reimplementation that widened the threshold to
// actually trigger at 90 degrees would silently diverge from real
// upstream behavior - upstream answers for this attitude, so this port
// must too.
// ---------------------------------------------------------------------

TEST_CASE("near exact 90-degree pitch the guard does not fire: large but finite, not a refusal",
          "[control][attitude_kinematics][gimbal_lock]") {
    const Quaternion att = attitude(0.0f, static_cast<float>(M_PI_2), 0.0f);
    const Vector3f body{0.1f, 0.2f, 0.3f};

    Vector3f euler{};
    const bool ok = body_to_euler_derivative(att, body, euler);

    REQUIRE(ok);
    REQUIRE(std::isfinite(euler.z));
    REQUIRE(std::fabs(euler.z) > 100.0f);

    // The forward direction has no singularity and stays well-behaved,
    // even fed this same large result back in.
    const Vector3f back = euler_derivative_to_body(att, euler);
    REQUIRE(std::isfinite(back.x));
    REQUIRE(std::isfinite(back.y));
    REQUIRE(std::isfinite(back.z));
}

// ---------------------------------------------------------------------
// Companion test: the guard DOES genuinely fire for some input close
// enough to the true singularity that cos_theta lands inside
// FLT_EPSILON. A quaternion round-trip through from_euler/get_euler_pitch
// cannot reach that precision by construction (see the test above), so
// this bisects toward it directly instead - matching COP-007's own
// 40-iteration bisection approach.
// ---------------------------------------------------------------------

TEST_CASE("the guard can fire: bisection finds a pitch where cos_theta is genuinely inside FLT_EPSILON",
          "[control][attitude_kinematics][gimbal_lock]") {
    float pitch = static_cast<float>(M_PI_2);
    bool refused = false;

    for (int i = 0; i < 40; ++i) {
        const Quaternion att = attitude(0.0f, pitch, 0.0f);
        Vector3f euler{};
        if (!body_to_euler_derivative(att, Vector3f{0.1f, 0.2f, 0.3f}, euler)) {
            refused = true;
            break;
        }
        // Walk toward the true singularity in the recovered angle.
        pitch += 1e-4f;
    }

    REQUIRE(refused);
}

// ---------------------------------------------------------------------
// ang_vel_limit: roll/pitch coupled elliptically, not per-axis. A
// diagonal command exceeding both limits is scaled back along its own
// direction, landing exactly on the unit circle in the ratio space - not
// clamped independently, which would let it through at up to root-two of
// the intended magnitude.
// ---------------------------------------------------------------------

TEST_CASE("ang_vel_limit scales a diagonal command onto the unit ellipse, preserving direction",
          "[control][attitude_kinematics][ang_vel_limit]") {
    const float max = 1.0f;

    Vector3f v{1.0f, 1.0f, 0.0f};
    ang_vel_limit(v, max, max, 0.0f);

    const float magnitude = std::sqrt(v.x * v.x + v.y * v.y);
    REQUIRE(magnitude == Approx(1.0f).margin(1e-5));
    REQUIRE(v.x == Approx(v.y).margin(1e-6));
}

TEST_CASE("ang_vel_limit leaves a command inside the ellipse untouched", "[control][attitude_kinematics][ang_vel_limit]") {
    const float max = 1.0f;
    Vector3f inside{0.5f, 0.5f, 0.0f};
    ang_vel_limit(inside, max, max, 0.0f);
    REQUIRE(inside.x == Approx(0.5f).margin(1e-6));
    REQUIRE(inside.y == Approx(0.5f).margin(1e-6));
}

TEST_CASE("ang_vel_limit: a zero limit means unlimited, not held at zero", "[control][attitude_kinematics][ang_vel_limit]") {
    Vector3f v{5.0f, 6.0f, 7.0f};
    ang_vel_limit(v, 0.0f, 0.0f, 0.0f);
    REQUIRE(v.x == Approx(5.0f).margin(1e-6));
    REQUIRE(v.y == Approx(6.0f).margin(1e-6));
    REQUIRE(v.z == Approx(7.0f).margin(1e-6));
}

TEST_CASE("ang_vel_limit: with one of roll/pitch zero, the other still clamps independently",
          "[control][attitude_kinematics][ang_vel_limit]") {
    Vector3f half{5.0f, 6.0f, 7.0f};
    ang_vel_limit(half, 0.0f, 2.0f, 3.0f);
    REQUIRE(half.x == Approx(5.0f).margin(1e-6)); // roll unlimited
    REQUIRE(half.y == Approx(2.0f).margin(1e-6)); // pitch clamped
    REQUIRE(half.z == Approx(3.0f).margin(1e-6)); // yaw clamped
}

TEST_CASE("ang_vel_limit: yaw is never coupled into the roll/pitch ellipse", "[control][attitude_kinematics][ang_vel_limit]") {
    // Roll/pitch both exceed their (equal) limits and get scaled back
    // together; yaw independently exceeds its own, smaller limit and
    // gets clamped on its own, unaffected by the roll/pitch scaling.
    Vector3f v{2.0f, 2.0f, 5.0f};
    ang_vel_limit(v, 1.0f, 1.0f, 1.0f);

    const float magnitude = std::sqrt(v.x * v.x + v.y * v.y);
    REQUIRE(magnitude == Approx(1.0f).margin(1e-5));
    REQUIRE(v.z == Approx(1.0f).margin(1e-6));
}

// ---------------------------------------------------------------------
// body_to_euler_limit: whole-vector passthrough on any single
// non-positive component (single ||-combined guard, not three
// independent per-component checks), tested with different components
// triggering it.
// ---------------------------------------------------------------------

TEST_CASE("body_to_euler_limit: a non-positive y component passes the whole vector through unchanged",
          "[control][attitude_kinematics][body_to_euler_limit]") {
    const Quaternion att = attitude(0.5f, 0.3f, 0.0f);
    const Vector3f limit{1.0f, 0.0f, 2.0f};
    const Vector3f out = body_to_euler_limit(att, limit);

    REQUIRE(out.x == Approx(limit.x).margin(1e-6));
    REQUIRE(out.y == Approx(limit.y).margin(1e-6));
    REQUIRE(out.z == Approx(limit.z).margin(1e-6));
}

TEST_CASE("body_to_euler_limit: a non-positive x component also passes the whole vector through unchanged",
          "[control][attitude_kinematics][body_to_euler_limit]") {
    const Quaternion att = attitude(0.5f, 0.3f, 0.0f);
    const Vector3f limit{-1.0f, 2.0f, 3.0f};
    const Vector3f out = body_to_euler_limit(att, limit);

    REQUIRE(out.x == Approx(limit.x).margin(1e-6));
    REQUIRE(out.y == Approx(limit.y).margin(1e-6));
    REQUIRE(out.z == Approx(limit.z).margin(1e-6));
}

TEST_CASE("body_to_euler_limit: a non-positive z component also passes the whole vector through unchanged",
          "[control][attitude_kinematics][body_to_euler_limit]") {
    const Quaternion att = attitude(0.5f, 0.3f, 0.0f);
    const Vector3f limit{1.0f, 2.0f, 0.0f};
    const Vector3f out = body_to_euler_limit(att, limit);

    REQUIRE(out.x == Approx(limit.x).margin(1e-6));
    REQUIRE(out.y == Approx(limit.y).margin(1e-6));
    REQUIRE(out.z == Approx(limit.z).margin(1e-6));
}

// ---------------------------------------------------------------------
// body_to_euler_limit: the 0.1 trig floor bounds the inflation factor at
// roughly 10x per term (not unboundedly large) even directly at a trig
// singularity.
// ---------------------------------------------------------------------

TEST_CASE("body_to_euler_limit: the 0.1 trig floor caps inflation near a singularity, doesn't let it run away",
          "[control][attitude_kinematics][body_to_euler_limit]") {
    const Vector3f body{1.0f, 1.0f, 1.0f};

    // Straight up: cos(pitch) is exactly zero, so without the floor the
    // z-term's cos_theta divisor would be zero (infinite output).
    const Quaternion steep = attitude(0.0f, static_cast<float>(M_PI_2), 0.0f);
    const Vector3f out = body_to_euler_limit(steep, body);

    REQUIRE(std::isfinite(out.z));
    // Two 0.1-floored terms multiply in the z denominator's
    // sin_phi*cos_theta / cos_phi*cos_theta components; bounded well
    // under a hundred here, nowhere near unboundedly large.
    REQUIRE(out.z <= 100.0f);

    // x is always passed straight through, in every case.
    REQUIRE(out.x == Approx(body.x).margin(1e-6));
}

// =======================================================================
// thrust_vector_rotation_angles (CCP-020) - the quaternion error
// decomposition. See this file's own header comment and attitude_
// kinematics.hpp's own "CCP-020 ADDENDUM" for the full design writeup.
// =======================================================================

namespace {

// The heading_vec_correction_quat.to_axis_angle()'s x and y should be
// zero (upstream's own comment) - re-derived directly here as an actual
// assertion helper, not just trusted, for every test case below that
// checks it.
void require_heading_xy_is_zero(const Quaternion& thrust_vector_correction, const Quaternion& attitude_body,
                                 const Quaternion& attitude_target, float tol) {
    const Quaternion heading_vec_correction_quat = thrust_vector_correction.inverse() * attitude_body.inverse() * attitude_target;
    Vector3f rotation_rad{};
    heading_vec_correction_quat.to_axis_angle(rotation_rad);
    REQUIRE(std::fabs(rotation_rad.x) < tol);
    REQUIRE(std::fabs(rotation_rad.y) < tol);
}

} // namespace

// -----------------------------------------------------------------------
// Matching attitudes: no error of any kind, but the lean angle (thrust_
// angle_rad) still reports the body's own lean rather than reading zero
// just because the error is zero - a leaning-but-matched attitude proves
// the two are not conflated.
// -----------------------------------------------------------------------

TEST_CASE("thrust_vector_rotation_angles: matching attitudes give no error, even while leaning",
          "[control][attitude_kinematics][thrust_vector_rotation_angles]") {
    const Quaternion att = attitude(0.3f, -0.2f, 1.1f);

    Quaternion correction;
    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_vector_rotation_angles(att, att, correction, error, thrust_angle, thrust_error_angle);

    REQUIRE(error.x == Approx(0.0f).margin(1e-5));
    REQUIRE(error.y == Approx(0.0f).margin(1e-5));
    REQUIRE(error.z == Approx(0.0f).margin(1e-5));
    REQUIRE(thrust_error_angle == Approx(0.0f).margin(1e-5));
    // Still reports the real lean, matching att's own 0.3/-0.2 rad tilt -
    // not the (zero) error.
    REQUIRE(thrust_angle == Approx(0.358873f).margin(1e-3));
}

// -----------------------------------------------------------------------
// (1) A pure lean (thrust-direction) error with zero heading difference
// produces zero attitude_error_rad.z.
//
// Built with matching Euler pitch AND matching Euler yaw, only roll
// differs - and the shared yaw (0.9 rad) and shared pitch (0.4 rad) are
// both nonzero, so this is neither level nor north-facing. Expected
// values independently recomputed - see this file's own header comment.
// -----------------------------------------------------------------------

TEST_CASE("thrust_vector_rotation_angles: a pure lean error stays out of yaw",
          "[control][attitude_kinematics][thrust_vector_rotation_angles]") {
    const Quaternion body = attitude(0.1f, 0.4f, 0.9f);
    const Quaternion target = attitude(0.35f, 0.4f, 0.9f);

    Quaternion correction;
    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_vector_rotation_angles(target, body, correction, error, thrust_angle, thrust_error_angle);

    REQUIRE(thrust_angle == Approx(0.411656f).margin(1e-3));
    REQUIRE(thrust_error_angle == Approx(0.25f).margin(1e-3));
    // The roll delta carries the whole correction...
    REQUIRE(error.x == Approx(0.25f).margin(1e-3));
    // ...and yaw is untouched.
    REQUIRE(error.y == Approx(0.0f).margin(1e-4));
    REQUIRE(error.z == Approx(0.0f).margin(1e-4));

    require_heading_xy_is_zero(correction, body, target, 1e-4f);
}

// -----------------------------------------------------------------------
// (2) A pure heading error, constructed the CORRECT way (a rotation
// about the CURRENT attitude's own thrust axis via CCP-019's from_axis_
// angle, composed on the right of body - not an independent Euler-yaw
// delta), produces zero attitude_error_rad.x/.y.
//
// body is deliberately leaning (roll=0.3, pitch=-0.25) AND yawed
// (0.6 rad) - not level, not north-facing. See this file's own header
// comment for why the naive "same roll/pitch, different Euler yaw"
// construction would have been wrong here.
// -----------------------------------------------------------------------

TEST_CASE("thrust_vector_rotation_angles: a pure heading error (rotation about the body's own thrust axis) stays out of "
          "roll/pitch",
          "[control][attitude_kinematics][thrust_vector_rotation_angles]") {
    const Quaternion body = attitude(0.3f, -0.25f, 0.6f);
    const float heading_change = 0.4f;

    // The thrust axis is (0,0,-1) in ANY body-fixed frame (this file's
    // own banner) - so a local rotation about that axis, composed on
    // the right of body, is a rotation about body's OWN current thrust
    // axis, not an independent Euler-yaw delta applied in the yawed
    // frame. This is the corrected construction from copter-rust's own
    // documented test-construction pitfall (see this file's own header
    // comment) - built correctly from the start here.
    Quaternion heading_delta;
    heading_delta.from_axis_angle(Vector3f{0.0f, 0.0f, -1.0f}, heading_change);
    const Quaternion target = body * heading_delta;

    Quaternion correction;
    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_vector_rotation_angles(target, body, correction, error, thrust_angle, thrust_error_angle);

    // The thrust vectors already agree (a pure heading change does not
    // move the thrust axis) - so this also exercises the degenerate
    // zero-error-angle fallback from a genuinely non-trivial attitude,
    // rather than only from the trivial matching-attitudes case above.
    REQUIRE(thrust_error_angle == Approx(0.0f).margin(1e-4));
    REQUIRE(error.x == Approx(0.0f).margin(1e-4));
    REQUIRE(error.y == Approx(0.0f).margin(1e-4));
    REQUIRE(std::fabs(error.z) == Approx(heading_change).margin(1e-3));
}

// -----------------------------------------------------------------------
// (3) The degenerate fallback, exercised explicitly and split into its
// two independently-triggering halves of the real `||` condition (see
// attitude_kinematics.hpp's own banner addendum):
//   - aligned thrust vectors: thrust_error_angle_rad is zero (and the
//     cross product is also zero - both halves true together).
//   - antiparallel thrust vectors: the cross product's length is zero
//     but thrust_error_angle_rad is pi, NOT zero - proving the length
//     check is independently necessary, not redundant with the angle
//     check.
// In both, thrust_vec_cross resets to the real thrust_vector_up CONSTANT
// itself, not a zero vector - visible here via thrust_vector_correction
// coming out finite and well-defined rather than NaN.
// -----------------------------------------------------------------------

TEST_CASE("thrust_vector_rotation_angles: degenerate fallback, aligned thrust vectors (zero error angle)",
          "[control][attitude_kinematics][thrust_vector_rotation_angles][degenerate]") {
    const Quaternion body = attitude(0.3f, -0.2f, 1.1f);

    Quaternion correction;
    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_vector_rotation_angles(body, body, correction, error, thrust_angle, thrust_error_angle);

    REQUIRE(thrust_error_angle == Approx(0.0f).margin(1e-6));
    REQUIRE(error.x == Approx(0.0f).margin(1e-6));
    REQUIRE(error.y == Approx(0.0f).margin(1e-6));
    REQUIRE(error.z == Approx(0.0f).margin(1e-6));
    // The fallback quaternion is the identity, not NaN.
    REQUIRE(std::isfinite(correction.q1));
    REQUIRE(correction.q1 == Approx(1.0f).margin(1e-6));
}

TEST_CASE("thrust_vector_rotation_angles: degenerate fallback, antiparallel thrust vectors (zero cross length, "
          "nonzero angle)",
          "[control][attitude_kinematics][thrust_vector_rotation_angles][degenerate]") {
    const Quaternion level = attitude(0.0f, 0.0f, 0.0f);
    const Quaternion inverted = attitude(static_cast<float>(M_PI), 0.0f, 0.0f);

    Quaternion correction;
    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_vector_rotation_angles(inverted, level, correction, error, thrust_angle, thrust_error_angle);

    REQUIRE(thrust_angle == Approx(0.0f).margin(1e-5));
    REQUIRE(thrust_error_angle == Approx(static_cast<float>(M_PI)).margin(1e-3));
    // Finite, not NaN - the fallback substituted thrust_vector_up
    // rather than dividing a zero-length cross product by itself.
    REQUIRE(std::isfinite(error.x));
    REQUIRE(std::isfinite(error.y));
    REQUIRE(std::isfinite(error.z));
    REQUIRE(std::isfinite(correction.q1));
    REQUIRE(correction.is_unit_length());
}

// -----------------------------------------------------------------------
// (4) A general, non-trivial (neither level nor north-facing) case, with
// thrust_angle_rad/thrust_error_angle_rad/attitude_error_rad
// independently recomputed - see this file's own header comment.
// -----------------------------------------------------------------------

TEST_CASE("thrust_vector_rotation_angles: a general non-trivial case matches an independently computed reference",
          "[control][attitude_kinematics][thrust_vector_rotation_angles]") {
    const Quaternion body = attitude(0.2f, -0.3f, 0.7f);
    const Quaternion target = attitude(-0.15f, 0.25f, 1.3f);

    Quaternion correction;
    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_vector_rotation_angles(target, body, correction, error, thrust_angle, thrust_error_angle);

    REQUIRE(thrust_angle == Approx(0.358873f).margin(1e-3));
    REQUIRE(thrust_error_angle == Approx(0.624906f).margin(1e-3));
    REQUIRE(error.x == Approx(-0.460579f).margin(1e-3));
    REQUIRE(error.y == Approx(0.422345f).margin(1e-3));
    REQUIRE(error.z == Approx(0.616828f).margin(1e-3));

    // (5) The heading_vec_correction_quat's own x/y-should-be-zero
    // invariant, confirmed as an actual assertion for this same
    // non-trivial case.
    require_heading_xy_is_zero(correction, body, target, 1e-4f);
}

// -----------------------------------------------------------------------
// (5) The heading_vec_correction_quat's own "x and y should be zero
// here" invariant (upstream's own comment), confirmed as an actual
// assertion across several of the cases above rather than trusted on
// faith - reusing the helper defined at the top of this section.
// -----------------------------------------------------------------------

TEST_CASE("thrust_vector_rotation_angles: heading_vec_correction_quat's x/y really are zero, across several attitudes",
          "[control][attitude_kinematics][thrust_vector_rotation_angles]") {
    struct Case {
        Quaternion body;
        Quaternion target;
    };
    const Case cases[] = {
        {attitude(0.0f, 0.0f, 0.0f), attitude(0.2f, -0.3f, 0.7f)},
        {attitude(0.1f, 0.4f, 0.9f), attitude(0.35f, 0.4f, 0.9f)},
        {attitude(-0.5f, 0.6f, -1.2f), attitude(0.4f, -0.4f, 0.2f)},
    };

    for (const auto& c : cases) {
        Quaternion correction;
        Vector3f error{};
        float thrust_angle = 0.0f;
        float thrust_error_angle = 0.0f;
        thrust_vector_rotation_angles(c.target, c.body, correction, error, thrust_angle, thrust_error_angle);
        require_heading_xy_is_zero(correction, c.body, c.target, 1e-3f);
    }
}

// -----------------------------------------------------------------------
// Composition property: applying the two corrections in order (thrust,
// then heading) takes body all the way to target - the strongest check
// available without a full controller fixture, and independent of any
// hardcoded reference numbers.
// -----------------------------------------------------------------------

TEST_CASE("thrust_vector_rotation_angles: the two corrections compose back to the target",
          "[control][attitude_kinematics][thrust_vector_rotation_angles]") {
    struct Case {
        Quaternion body;
        Quaternion target;
    };
    const Case cases[] = {
        {attitude(0.0f, 0.0f, 0.0f), attitude(0.2f, -0.3f, 0.7f)},
        {attitude(0.35f, -0.2f, 0.4f), attitude(-0.1f, 0.5f, 1.9f)},
        {attitude(-0.5f, 0.6f, -1.2f), attitude(0.4f, -0.4f, 0.2f)},
    };

    for (const auto& c : cases) {
        Quaternion correction;
        Vector3f error{};
        float thrust_angle = 0.0f;
        float thrust_error_angle = 0.0f;
        thrust_vector_rotation_angles(c.target, c.body, correction, error, thrust_angle, thrust_error_angle);

        const Quaternion heading = correction.inverse() * c.body.inverse() * c.target;
        const Quaternion rebuilt = c.body * correction * heading;

        float rr = 0.0f, rp = 0.0f, ry = 0.0f;
        rebuilt.to_euler(rr, rp, ry);
        float tr = 0.0f, tp = 0.0f, ty = 0.0f;
        c.target.to_euler(tr, tp, ty);

        REQUIRE(rr == Approx(tr).margin(1e-3));
        REQUIRE(rp == Approx(tp).margin(1e-3));
        REQUIRE(ry == Approx(ty).margin(1e-3));
    }
}

// -----------------------------------------------------------------------
// attitude_command_model (CCP-022) - see attitude_kinematics.hpp's own
// "CCP-022 ADDENDUM" banner for the full design writeup this section's
// tests are built against.
// -----------------------------------------------------------------------

// (1) The accel_max fallback: radians(1800.0f), EXACTLY - not a
// separately hand-typed approximation. Driven directly rather than via
// the equivalent-call technique used below for input_tc: input_tc is
// set absurdly small (1e-6f) so the jerk limit (accel_max / input_tc)
// is astronomically large and cannot possibly bind on this first call
// from rest, max_ang_vel is set huge so the velocity clamp inside
// shape_pos_vel_accel cannot bind either, and error_angle (kept inside
// (-pi, pi) so shape_angle_vel_accel's own wrap_PI is a no-op and does
// not complicate the reasoning) is large enough that the sqrt-shaped
// correction comfortably exceeds accel_max before its own clamp. The
// ONLY thing left able to determine target_ang_accel is the real
// accel_min/accel_max clamp inside shape_pos_vel_accel, using the exact
// accel_max this function resolved the fallback to - so the output is
// a direct, bit-exact readout of that resolved value.
TEST_CASE("attitude_command_model: the accel_max fallback is exactly radians(1800.0f), read directly off a "
          "saturated first step",
          "[control][attitude_kinematics][attitude_command_model]") {
    float target_ang_vel = 0.0f;
    float target_ang_accel = 0.0f;
    attitude_command_model(/*error_angle=*/2.5f, /*desired_ang_vel=*/0.0f, target_ang_vel, target_ang_accel,
                            /*max_ang_vel=*/1.0e6f, /*accel_max=*/0.0f, /*input_tc=*/1.0e-6f, /*dt=*/0.0025f);

    // Compared against radians()'s own real output, not a separately
    // hand-typed literal - the precise test that would catch a
    // ULP-level transcription error in the fallback constant.
    REQUIRE(target_ang_accel == fwcpp::math::radians(1800.0f));
}

// (2) The input_tc fallback: exactly dt * 10.0 (upstream's own comment:
// "achieve maximum acceleration in 10 clock cycles"). Uses the
// equivalent-call technique described in this file's own header
// comment above: one call with input_tc invalid (triggering the real
// fallback), one call with input_tc explicitly set to dt * 10.0f,
// everything else identical (including a valid, explicit accel_max, so
// this test is not entangled with test (1) above). If the fallback
// really resolves to dt * 10.0, the two calls are computing the exact
// same jerk_max from the exact same accel_max, and thus a pure
// downstream function of identical inputs - the two results must be
// bit-identical.
TEST_CASE("attitude_command_model: the input_tc fallback is exactly dt * 10.0", "[control][attitude_kinematics]["
                                                                                  "attitude_command_model]") {
    const float error_angle = 0.6f;
    const float desired_ang_vel = 0.1f;
    const float max_ang_vel = 5.0f;
    const float accel_max = fwcpp::math::radians(400.0f);
    const float dt = 0.0025f;

    float fallback_vel = 0.0f;
    float fallback_accel = 0.0f;
    attitude_command_model(error_angle, desired_ang_vel, fallback_vel, fallback_accel, max_ang_vel, accel_max,
                            /*input_tc=*/0.0f, dt);

    float explicit_vel = 0.0f;
    float explicit_accel = 0.0f;
    attitude_command_model(error_angle, desired_ang_vel, explicit_vel, explicit_accel, max_ang_vel, accel_max,
                            /*input_tc=*/dt * 10.0f, dt);

    REQUIRE(fallback_accel == explicit_accel);
    REQUIRE(fallback_vel == explicit_vel);

    // Sanity check the equivalent-call technique is actually exercising
    // something: a wildly different input_tc must NOT agree, or this
    // test would pass no matter what the fallback resolved to.
    float different_vel = 0.0f;
    float different_accel = 0.0f;
    attitude_command_model(error_angle, desired_ang_vel, different_vel, different_accel, max_ang_vel, accel_max,
                            /*input_tc=*/dt * 100.0f, dt);
    REQUIRE(different_accel != fallback_accel);
}

// (3) THE DIRECTION TEST copter-rust's own notes specifically call out:
// a smaller input_tc must produce a SHARPER (larger-magnitude
// target_ang_accel) response than a larger input_tc, all else held
// identical. This is the one test that would fail if accel_max/
// input_tc were accidentally swapped in the accel_max / input_tc jerk-
// limit division - a swap would still compile, still run, and still
// produce SOME finite, plausible jerk limit, just the wrong one.
//
// error_angle (kept inside (-pi, pi) so wrap_PI is a no-op) is large
// enough that the pre-jerk-limit acceleration command saturates against
// accel_max for BOTH input_tc choices (so the comparison below is
// purely about which jerk limit binds, not about two different
// unsaturated sqrt-controller outputs), and both input_tc choices are
// comfortably larger than dt so that jerk_max * dt (the real per-step
// accel change from rest) stays below accel_max for both - i.e. the
// jerk limit, not the accel_max clamp, is what determines the outcome
// of this first step.
TEST_CASE("attitude_command_model: a smaller input_tc produces a sharper (larger-magnitude) response",
          "[control][attitude_kinematics][attitude_command_model]") {
    const float error_angle = 2.5f;
    const float desired_ang_vel = 0.0f;
    const float max_ang_vel = 1.0e6f;
    const float accel_max = fwcpp::math::radians(400.0f);
    const float dt = 0.0025f;

    float sharp_vel = 0.0f;
    float sharp_accel = 0.0f;
    attitude_command_model(error_angle, desired_ang_vel, sharp_vel, sharp_accel, max_ang_vel, accel_max,
                            /*input_tc=*/0.02f, dt);

    float gentle_vel = 0.0f;
    float gentle_accel = 0.0f;
    attitude_command_model(error_angle, desired_ang_vel, gentle_vel, gentle_accel, max_ang_vel, accel_max,
                            /*input_tc=*/0.08f, dt);

    REQUIRE(sharp_accel > 0.0f);
    REQUIRE(gentle_accel > 0.0f);
    REQUIRE(sharp_accel > gentle_accel);
    // Both stay strictly below the acceleration ceiling itself - proving
    // this comparison is really about the jerk limit binding, not two
    // calls that both simply saturated at accel_max.
    REQUIRE(sharp_accel < accel_max);
    REQUIRE(gentle_accel < accel_max);
}

// (4) The dt <= 0 early return: both output parameters left completely
// untouched, not merely "close to" their input values.
TEST_CASE("attitude_command_model: a non-positive dt leaves both outputs completely unchanged",
          "[control][attitude_kinematics][attitude_command_model]") {
    float target_ang_vel = 1.23f;
    float target_ang_accel = 4.56f;
    attitude_command_model(/*error_angle=*/0.5f, /*desired_ang_vel=*/0.2f, target_ang_vel, target_ang_accel,
                            /*max_ang_vel=*/5.0f, /*accel_max=*/fwcpp::math::radians(400.0f), /*input_tc=*/0.05f,
                            /*dt=*/0.0f);
    REQUIRE(target_ang_vel == 1.23f);
    REQUIRE(target_ang_accel == 4.56f);

    // A negative dt takes the exact same early-return branch
    // (is_positive gates it, not merely != 0).
    target_ang_vel = 1.23f;
    target_ang_accel = 4.56f;
    attitude_command_model(0.5f, 0.2f, target_ang_vel, target_ang_accel, 5.0f, fwcpp::math::radians(400.0f), 0.05f,
                            -0.0025f);
    REQUIRE(target_ang_vel == 1.23f);
    REQUIRE(target_ang_accel == 4.56f);
}

// (5) The real, separate final `target_ang_vel += target_ang_accel *
// dt` integration step - genuinely additional to whatever
// shape_angle_vel_accel itself does (that call's own angle_vel
// parameter is by-value input only, so it cannot have written
// target_ang_vel back). Constructed so skipping this step would
// produce a measurably different (and wrong) target_ang_vel: starting
// target_ang_vel is a nonzero, arbitrary value unrelated to the error,
// so shape_angle_vel_accel's own internal treatment of it cannot
// coincidentally reproduce this function's final output on its own.
TEST_CASE("attitude_command_model: target_ang_vel gets a real, separate final += target_ang_accel * dt step",
          "[control][attitude_kinematics][attitude_command_model]") {
    const float starting_target_ang_vel = 3.0f;
    const float dt = 0.0025f;

    float target_ang_vel = starting_target_ang_vel;
    float target_ang_accel = 0.0f;
    attitude_command_model(/*error_angle=*/0.5f, /*desired_ang_vel=*/0.1f, target_ang_vel, target_ang_accel,
                            /*max_ang_vel=*/5.0f, /*accel_max=*/fwcpp::math::radians(400.0f), /*input_tc=*/0.05f,
                            dt);

    // The step must have moved the needle by a measurable amount - not
    // merely be "not exactly zero" due to float noise - or this test
    // would not actually be distinguishing "the step ran" from "the
    // step is missing".
    REQUIRE(std::fabs(target_ang_accel * dt) > 1e-6f);

    // shape_angle_vel_accel's own angle_vel parameter is by-value input
    // only (see attitude_kinematics.hpp's own banner addendum), so
    // target_ang_vel is provably UNCHANGED by that call itself; the
    // final value must equal the starting value plus exactly this
    // function's own separate integration step.
    REQUIRE(target_ang_vel == Approx(starting_target_ang_vel + target_ang_accel * dt).margin(1e-6f));

    // And that really is different from what a port that dropped the
    // final step (leaving target_ang_vel at its pre-call value) would
    // have produced.
    REQUIRE(target_ang_vel != starting_target_ang_vel);
}
