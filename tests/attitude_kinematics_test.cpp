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

// CCP-023 added the thrust_heading_rotation_angles tests below (real
// lines 1033-1050) - see attitude_kinematics.hpp's own "CCP-023
// ADDENDUM" comment block for the full design writeup: the corrected
// CCP-020 deferral, the real two-level nested guard, the 1.0f/
// rate_yaw_kp reciprocal, and the three named radians()-computed
// constants.
//
// Every case below reuses the same "pure heading" construction
// established above for thrust_vector_rotation_angles - `body *
// Quaternion::from_axis_angle((0,0,-1), delta)` - specifically because
// it drives the CCP-020 thrust correction to its own degenerate
// (near-identity) fallback, isolating attitude_error_rad.z as (up to
// sign) the whole real heading delta with no roll/pitch coupling to
// account for by hand. The RAW (pre-clamp) attitude_error_rad.z for a
// given body/delta pair is obtained directly from an independent call
// to thrust_vector_rotation_angles - CCP-020's own already-verified
// function - not hand-derived from quaternion algebra, so these tests
// stay focused on this ticket's own new clamping/recomposition logic
// rather than re-litigating CCP-020's own already-tested decomposition.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/math/control.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

using namespace fwcpp::control;
using fwcpp::math::Quaternion;
using fwcpp::math::Vector2f;
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

// =======================================================================
// thrust_heading_rotation_angles (CCP-023) - the yaw-error-limiting
// wrapper around thrust_vector_rotation_angles. See this file's own
// header comment and attitude_kinematics.hpp's own "CCP-023 ADDENDUM"
// for the full design writeup, including the corrected CCP-020 deferral.
// =======================================================================

// (1) The real outer guard: rate_yaw_kp == 0 leaves attitude_target
// COMPLETELY UNCHANGED - a real, enclosing `if`, not a subsequent early
// `return` (attitude_error_rad.x/.y/.z are still computed by the CCP-020
// call underneath, which runs unconditionally BEFORE this guard).
TEST_CASE("thrust_heading_rotation_angles: rate_yaw_kp == 0 leaves attitude_target completely unchanged",
          "[control][attitude_kinematics][thrust_heading_rotation_angles]") {
    const Quaternion body = attitude(0.3f, -0.25f, 0.6f);
    const float heading_change = 0.65f; // large enough it would clamp if the guard did not block it
    Quaternion heading_delta;
    heading_delta.from_axis_angle(Vector3f{0.0f, 0.0f, -1.0f}, heading_change);
    Quaternion target = body * heading_delta;
    const Quaternion original_target = target;

    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_heading_rotation_angles(target, body, error, thrust_angle, thrust_error_angle,
                                    /*rate_yaw_kp=*/0.0f, /*angle_yaw_kp=*/1.0f,
                                    /*accel_yaw_max_radss=*/fwcpp::math::radians(270.0f));

    // Bit-exact, not merely "close" - attitude_target is never touched
    // at all when this guard fires.
    REQUIRE(target.q1 == original_target.q1);
    REQUIRE(target.q2 == original_target.q2);
    REQUIRE(target.q3 == original_target.q3);
    REQUIRE(target.q4 == original_target.q4);
    // The CCP-020 decomposition underneath still ran and populated a
    // real, finite heading error - proving this guard skips only the
    // clamp/recomposition, not the whole function.
    REQUIRE(std::isfinite(error.z));
    REQUIRE(std::fabs(error.z) > 1e-3f);
}

// (2a) The second guard's own first half, tested independently:
// angle_yaw_kp == 0 skips clamping even though the actual yaw error is
// large enough that it otherwise would have clamped.
TEST_CASE("thrust_heading_rotation_angles: angle_yaw_kp == 0 skips clamping despite a large yaw error",
          "[control][attitude_kinematics][thrust_heading_rotation_angles]") {
    const Quaternion body = attitude(0.3f, -0.25f, 0.6f);
    const float heading_change = 1.2f; // well beyond AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD (45 deg) on its own
    Quaternion heading_delta;
    heading_delta.from_axis_angle(Vector3f{0.0f, 0.0f, -1.0f}, heading_change);
    Quaternion target = body * heading_delta;
    const Quaternion original_target = target;

    // Ground truth for the raw (unclamped) error, from CCP-020's own
    // already-verified thrust_vector_rotation_angles.
    Quaternion unused_correction;
    Vector3f raw_error{};
    float unused_thrust_angle = 0.0f;
    float unused_thrust_error_angle = 0.0f;
    thrust_vector_rotation_angles(target, body, unused_correction, raw_error, unused_thrust_angle,
                                   unused_thrust_error_angle);
    REQUIRE(std::fabs(raw_error.z) > kYawMaxErrorAngleRad); // sanity: this really would clamp otherwise

    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_heading_rotation_angles(target, body, error, thrust_angle, thrust_error_angle,
                                    /*rate_yaw_kp=*/2.0f, /*angle_yaw_kp=*/0.0f,
                                    /*accel_yaw_max_radss=*/fwcpp::math::radians(270.0f));

    // Unclamped: error.z is exactly the raw value, and attitude_target
    // is completely untouched (the mutating line sits inside this same
    // guard).
    REQUIRE(error.z == raw_error.z);
    REQUIRE(target.q1 == original_target.q1);
    REQUIRE(target.q2 == original_target.q2);
    REQUIRE(target.q3 == original_target.q3);
    REQUIRE(target.q4 == original_target.q4);
}

// (2b) The second guard's own other half, tested independently: a yaw
// error already within heading_error_max skips clamping despite a
// genuinely non-zero angle_yaw_kp.
TEST_CASE("thrust_heading_rotation_angles: a yaw error already within heading_error_max skips clamping",
          "[control][attitude_kinematics][thrust_heading_rotation_angles]") {
    const Quaternion body = attitude(0.3f, -0.25f, 0.6f);
    const float heading_change = 0.1f; // small, comfortably inside any real heading_error_max
    Quaternion heading_delta;
    heading_delta.from_axis_angle(Vector3f{0.0f, 0.0f, -1.0f}, heading_change);
    Quaternion target = body * heading_delta;
    const Quaternion original_target = target;

    Quaternion unused_correction;
    Vector3f raw_error{};
    float unused_thrust_angle = 0.0f;
    float unused_thrust_error_angle = 0.0f;
    thrust_vector_rotation_angles(target, body, unused_correction, raw_error, unused_thrust_angle,
                                   unused_thrust_error_angle);

    const float rate_yaw_kp = 2.0f;
    const float angle_yaw_kp = 1.0f;
    const float accel_yaw_max_radss = fwcpp::math::radians(270.0f);
    const float heading_accel_max_ref = fwcpp::math::constrain_value(
        accel_yaw_max_radss / 2.0f, kAccelYControllerMinRadss, kAccelYControllerMaxRadss);
    const float heading_error_max_ref =
        std::min(fwcpp::math::inv_sqrt_controller(1.0f / rate_yaw_kp, angle_yaw_kp, heading_accel_max_ref),
                 kYawMaxErrorAngleRad);
    REQUIRE(std::fabs(raw_error.z) < heading_error_max_ref); // sanity: really is within limit

    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_heading_rotation_angles(target, body, error, thrust_angle, thrust_error_angle, rate_yaw_kp, angle_yaw_kp,
                                    accel_yaw_max_radss);

    REQUIRE(error.z == raw_error.z);
    REQUIRE(target.q1 == original_target.q1);
    REQUIRE(target.q2 == original_target.q2);
    REQUIRE(target.q3 == original_target.q3);
    REQUIRE(target.q4 == original_target.q4);
}

// (3) Both guard conditions satisfied: attitude_error_rad.z genuinely
// clamps to +-heading_error_max AND attitude_target is genuinely
// reassigned to the real three-way composition
// (attitude_body * thrust_vector_correction * heading_vec_correction_quat),
// verified against an independently-computed reference quaternion built
// from CCP-020's own already-verified thrust_vector_rotation_angles
// output plus a hand-built heading correction quaternion - not merely
// "some quaternion changed value".
TEST_CASE("thrust_heading_rotation_angles: both guards satisfied clamps attitude_error_rad.z and reassigns "
          "attitude_target via the real three-way composition",
          "[control][attitude_kinematics][thrust_heading_rotation_angles]") {
    const Quaternion body = attitude(0.3f, -0.25f, 0.6f);
    const float heading_change = 0.65f;
    Quaternion heading_delta;
    heading_delta.from_axis_angle(Vector3f{0.0f, 0.0f, -1.0f}, heading_change);
    const Quaternion original_target = body * heading_delta;

    // Ground truth for the raw (unclamped) error and the thrust
    // correction quaternion, from CCP-020's own already-verified
    // thrust_vector_rotation_angles - independent of anything this
    // ticket's own new clamping/recomposition logic does.
    Quaternion thrust_vector_correction_ref;
    Vector3f raw_error{};
    float ref_thrust_angle = 0.0f;
    float ref_thrust_error_angle = 0.0f;
    thrust_vector_rotation_angles(original_target, body, thrust_vector_correction_ref, raw_error, ref_thrust_angle,
                                   ref_thrust_error_angle);

    const float rate_yaw_kp = 2.0f;
    const float angle_yaw_kp = 1.0f;
    const float accel_yaw_max_radss = fwcpp::math::radians(270.0f);

    // Independently-derived heading_error_max, computed by hand from the
    // real formula rather than trusted from the function under test.
    const float heading_accel_max_ref = fwcpp::math::constrain_value(
        accel_yaw_max_radss / 2.0f, kAccelYControllerMinRadss, kAccelYControllerMaxRadss);
    const float heading_error_max_ref =
        std::min(fwcpp::math::inv_sqrt_controller(1.0f / rate_yaw_kp, angle_yaw_kp, heading_accel_max_ref),
                 kYawMaxErrorAngleRad);
    REQUIRE(std::fabs(raw_error.z) > heading_error_max_ref); // sanity: this case really does need clamping

    const float clamped_z = fwcpp::math::constrain_value(fwcpp::math::wrap_PI(raw_error.z), -heading_error_max_ref,
                                                           heading_error_max_ref);

    Quaternion heading_vec_correction_ref;
    heading_vec_correction_ref.from_axis_angle(Vector3f{0.0f, 0.0f, clamped_z});
    const Quaternion expected_target = body * thrust_vector_correction_ref * heading_vec_correction_ref;

    Quaternion target = original_target;
    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_heading_rotation_angles(target, body, error, thrust_angle, thrust_error_angle, rate_yaw_kp, angle_yaw_kp,
                                    accel_yaw_max_radss);

    REQUIRE(error.z == Approx(clamped_z).margin(1e-6f));
    REQUIRE(std::fabs(error.z) == Approx(heading_error_max_ref).margin(1e-6f));
    REQUIRE(target.q1 == Approx(expected_target.q1).margin(1e-6f));
    REQUIRE(target.q2 == Approx(expected_target.q2).margin(1e-6f));
    REQUIRE(target.q3 == Approx(expected_target.q3).margin(1e-6f));
    REQUIRE(target.q4 == Approx(expected_target.q4).margin(1e-6f));
    // Genuinely reassigned, not merely still valid - differs from the
    // unclamped original target.
    REQUIRE(target.q1 != original_target.q1);
}

// (4) The real 1.0f / rate_yaw_kp reciprocal, specifically: constructed
// so that passing rate_yaw_kp itself (2.0) instead of its reciprocal
// (0.5) into inv_sqrt_controller would resolve heading_error_max to a
// DIFFERENT value (the AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD ceiling, since
// the un-reciprocated computation overshoots it) than the real, correct
// computation does - and a yaw error picked strictly between the two
// candidate limits clamps under the correct reciprocal but would NOT
// clamp at all under the reciprocal-omitted bug, making attitude_
// target's own mutation status itself the discriminator.
TEST_CASE("thrust_heading_rotation_angles: the 1.0f / rate_yaw_kp reciprocal is exercised, not the raw gain",
          "[control][attitude_kinematics][thrust_heading_rotation_angles]") {
    const Quaternion body = attitude(0.3f, -0.25f, 0.6f);
    const float heading_change = 0.65f;
    Quaternion heading_delta;
    heading_delta.from_axis_angle(Vector3f{0.0f, 0.0f, -1.0f}, heading_change);
    Quaternion target = body * heading_delta;
    const Quaternion original_target = target;

    const float rate_yaw_kp = 2.0f; // reciprocal = 0.5
    const float angle_yaw_kp = 1.0f;
    const float accel_yaw_max_radss = fwcpp::math::radians(270.0f);

    const float heading_accel_max_ref = fwcpp::math::constrain_value(
        accel_yaw_max_radss / 2.0f, kAccelYControllerMinRadss, kAccelYControllerMaxRadss);
    const float correct_heading_error_max =
        std::min(fwcpp::math::inv_sqrt_controller(1.0f / rate_yaw_kp, angle_yaw_kp, heading_accel_max_ref),
                 kYawMaxErrorAngleRad);
    // What heading_error_max would resolve to if the reciprocal were
    // accidentally omitted (rate_yaw_kp passed directly).
    const float buggy_heading_error_max =
        std::min(fwcpp::math::inv_sqrt_controller(rate_yaw_kp, angle_yaw_kp, heading_accel_max_ref),
                 kYawMaxErrorAngleRad);
    REQUIRE(correct_heading_error_max != buggy_heading_error_max);
    REQUIRE(heading_change > correct_heading_error_max);
    REQUIRE(heading_change < buggy_heading_error_max);

    Vector3f error{};
    float thrust_angle = 0.0f;
    float thrust_error_angle = 0.0f;
    thrust_heading_rotation_angles(target, body, error, thrust_angle, thrust_error_angle, rate_yaw_kp, angle_yaw_kp,
                                    accel_yaw_max_radss);

    // The real implementation clamps (proving the reciprocal, not the
    // raw gain, was used) - a reciprocal-omitted port would leave
    // attitude_target completely unchanged here instead, since
    // heading_change (0.65) sits below buggy_heading_error_max.
    REQUIRE(std::fabs(error.z) == Approx(correct_heading_error_max).margin(1e-6f));
    REQUIRE(target.q1 != original_target.q1);
}

// (5) The three named constants, each confirmed against radians()'s own
// real output directly - not a separately hand-typed approximation,
// matching CCP-022's own established ULP-precision test discipline for
// its radians(1800.0f) fallback.
TEST_CASE("thrust_heading_rotation_angles: the three named constants are exactly radians()'s own output",
          "[control][attitude_kinematics][thrust_heading_rotation_angles]") {
    REQUIRE(kAccelYControllerMinRadss == fwcpp::math::radians(10.0f));
    REQUIRE(kAccelYControllerMaxRadss == fwcpp::math::radians(120.0f));
    REQUIRE(kYawMaxErrorAngleRad == fwcpp::math::radians(45.0f));
}

// =======================================================================
// attitude_from_thrust_vector + update_ang_vel_target_from_att_error
// (CCP-024). See attitude_kinematics.hpp's own "CCP-024 ADDENDUM" comment
// block for the full design writeup: the load-bearing composition order,
// the opposite Z signs, the round-trip test methodology reused directly
// from copter-rust's own COP-007, the real per-axis (not per-vehicle)
// strategy choice, the acceleration-halving, and the axis-different clamp
// bounds.
// =======================================================================

// (1) The real round-trip test methodology, reused directly from
// copter-rust's own COP-007 rather than invented independently: build an
// attitude from a 0.3 rad tilt, run it back through CCP-020's own
// already-verified thrust_vector_rotation_angles against level, and
// require the recovered lean to come out at exactly 0.3. A nonzero
// heading is included on the construction side specifically to also
// prove heading does not leak into the recovered lean (either direction
// alone - the construction or the decomposition - could be
// self-consistently wrong; tying them together via round-trip catches
// what an isolated check can't).
TEST_CASE("attitude_from_thrust_vector: round-trips through thrust_vector_rotation_angles decomposition",
          "[control][attitude_kinematics][attitude_from_thrust_vector]") {
    const float tilt = 0.3f;
    const float heading = 0.4f; // nonzero: proves heading does not leak into the recovered lean
    const Vector3f thrust_vector{std::sin(tilt), 0.0f, -std::cos(tilt)};

    const Quaternion attitude_target = attitude_from_thrust_vector(thrust_vector, heading);
    const Quaternion level; // identity - level, per this test's own methodology

    Quaternion thrust_vector_correction;
    Vector3f attitude_error{};
    float thrust_angle_rad = 0.0f;
    float thrust_error_angle_rad = 0.0f;
    thrust_vector_rotation_angles(attitude_target, level, thrust_vector_correction, attitude_error, thrust_angle_rad,
                                   thrust_error_angle_rad);

    // thrust_error_angle_rad is the angle between the two thrust vectors
    // (target vs. level/body) - the recovered lean.
    REQUIRE(thrust_error_angle_rad == Approx(tilt).margin(1e-5f));
}

// (2) The real, load-bearing composition order (thrust_vec_quat on the
// LEFT, yaw_quat on the RIGHT) - a non-trivial tilt AND a non-zero
// heading together, confirming the result matches the real forward order
// exactly, and genuinely differs from the reversed order and from either
// rotation taken alone.
TEST_CASE("attitude_from_thrust_vector: composes as thrust_quat * yaw_quat, and the order is load-bearing",
          "[control][attitude_kinematics][attitude_from_thrust_vector]") {
    const float tilt = 0.3f;
    const float heading = 0.6f;
    const Vector3f thrust_vector{std::sin(tilt), 0.0f, -std::cos(tilt)};

    // Independently-built reference quaternions, matching the real
    // formula's own construction by hand (the cross product of
    // (0,0,-1) and thrust_vector is (0,-sin(tilt),0), normalizing to
    // (0,-1,0)) rather than calling the function under test for either
    // half.
    Quaternion thrust_vec_quat_ref;
    thrust_vec_quat_ref.from_axis_angle(Vector3f{0.0f, -1.0f, 0.0f}, tilt);
    Quaternion yaw_quat_ref;
    yaw_quat_ref.from_axis_angle(Vector3f{0.0f, 0.0f, 1.0f}, heading);

    const Quaternion expected_forward = thrust_vec_quat_ref * yaw_quat_ref;
    const Quaternion reversed = yaw_quat_ref * thrust_vec_quat_ref;
    // Sanity: the two orders really do differ (the axes are not
    // parallel), so a port that silently swapped them would be caught.
    REQUIRE(std::fabs(expected_forward.q2 - reversed.q2) > 1e-3f);

    const Quaternion result = attitude_from_thrust_vector(thrust_vector, heading);

    // Matches the real forward order exactly.
    REQUIRE(result.q1 == Approx(expected_forward.q1).margin(1e-6f));
    REQUIRE(result.q2 == Approx(expected_forward.q2).margin(1e-6f));
    REQUIRE(result.q3 == Approx(expected_forward.q3).margin(1e-6f));
    REQUIRE(result.q4 == Approx(expected_forward.q4).margin(1e-6f));

    // And genuinely differs from the reversed composition order.
    REQUIRE(std::fabs(result.q2 - reversed.q2) > 1e-3f);

    // And differs from either rotation taken alone (pure tilt, no
    // heading; pure heading, no tilt).
    const Quaternion tilt_only = attitude_from_thrust_vector(thrust_vector, 0.0f);
    const Quaternion heading_only = attitude_from_thrust_vector(Vector3f{0.0f, 0.0f, -1.0f}, heading);
    REQUIRE(std::fabs(result.q2 - tilt_only.q2) > 1e-3f);
    REQUIRE(std::fabs(result.q4 - heading_only.q4) > 1e-6f);
}

// (2b) A dedicated, direct test of the opposite-Z-sign asymmetry itself:
// thrust rotates about the cross-product axis derived from the real -Z
// "up" convention, but heading rotates about +Z - a genuinely different
// axis, not a sign that happens to cancel out. Constructed so that
// swapping the heading axis to -Z (the same sign as thrust's own up
// vector) would produce a measurably different, wrong result.
TEST_CASE("attitude_from_thrust_vector: heading rotates about +Z, the opposite sign from thrust's own -Z convention",
          "[control][attitude_kinematics][attitude_from_thrust_vector]") {
    const float tilt = 0.3f;
    const float heading = 0.6f;
    const Vector3f thrust_vector{std::sin(tilt), 0.0f, -std::cos(tilt)};

    Quaternion thrust_vec_quat_ref;
    thrust_vec_quat_ref.from_axis_angle(Vector3f{0.0f, -1.0f, 0.0f}, tilt);

    Quaternion yaw_quat_correct;
    yaw_quat_correct.from_axis_angle(Vector3f{0.0f, 0.0f, 1.0f}, heading);
    Quaternion yaw_quat_wrong_sign;
    yaw_quat_wrong_sign.from_axis_angle(Vector3f{0.0f, 0.0f, -1.0f}, heading);

    const Quaternion expected_correct = thrust_vec_quat_ref * yaw_quat_correct;
    const Quaternion expected_wrong_sign = thrust_vec_quat_ref * yaw_quat_wrong_sign;
    // Sanity: the two sign conventions really do produce different
    // results for this non-trivial tilt+heading pair.
    REQUIRE(std::fabs(expected_correct.q4 - expected_wrong_sign.q4) > 1e-3f);

    const Quaternion result = attitude_from_thrust_vector(thrust_vector, heading);
    REQUIRE(result.q1 == Approx(expected_correct.q1).margin(1e-6f));
    REQUIRE(result.q2 == Approx(expected_correct.q2).margin(1e-6f));
    REQUIRE(result.q3 == Approx(expected_correct.q3).margin(1e-6f));
    REQUIRE(result.q4 == Approx(expected_correct.q4).margin(1e-6f));
    REQUIRE(std::fabs(result.q4 - expected_wrong_sign.q4) > 1e-3f);
}

// (3) The real per-axis (not per-vehicle) sqrt-vs-proportional strategy
// choice - a MIXED case in one call: roll/pitch have a non-zero
// acceleration max (sqrt-eligible), yaw has a zero acceleration max
// (forced proportional regardless of use_sqrt_controller). error.x/.y are
// picked large enough relative to their own linear_dist to land in the
// sqrt branch, not merely the linear region of the hybrid model, so the
// sqrt-vs-proportional distinction is actually exercised.
TEST_CASE("update_ang_vel_target_from_att_error: chooses sqrt vs proportional per axis, not per vehicle",
          "[control][attitude_kinematics][update_ang_vel_target_from_att_error]") {
    const Vector3f error{0.2f, -0.25f, 0.1f};
    const float angle_kp_roll = 4.5f;
    const float angle_kp_pitch = 4.5f;
    const float angle_kp_yaw = 4.5f;
    const Vector3f angle_p_scale{1.0f, 1.0f, 1.0f};
    const float accel_roll_max = fwcpp::math::radians(400.0f);  // non-zero: sqrt-eligible
    const float accel_pitch_max = fwcpp::math::radians(400.0f); // non-zero: sqrt-eligible
    const float accel_yaw_max = 0.0f;                           // zero: forces proportional regardless of the flag
    const float dt = 0.0025f;

    const Vector3f result = update_ang_vel_target_from_att_error(error, angle_kp_roll, angle_kp_pitch, angle_kp_yaw,
                                                                   angle_p_scale, accel_roll_max, accel_pitch_max,
                                                                   accel_yaw_max, /*use_sqrt_controller=*/true, dt);

    const float expected_roll = fwcpp::math::sqrt_controller(
        error.x, angle_kp_roll * angle_p_scale.x,
        fwcpp::math::constrain_value(accel_roll_max / 2.0f, kAccelRpControllerMinRadss, kAccelRpControllerMaxRadss),
        dt);
    const float expected_pitch = fwcpp::math::sqrt_controller(
        error.y, angle_kp_pitch * angle_p_scale.y,
        fwcpp::math::constrain_value(accel_pitch_max / 2.0f, kAccelRpControllerMinRadss, kAccelRpControllerMaxRadss),
        dt);
    const float expected_yaw = (angle_kp_yaw * angle_p_scale.z) * error.z; // plain proportional, forced by zero accel max

    REQUIRE(result.x == Approx(expected_roll).margin(1e-6f));
    REQUIRE(result.y == Approx(expected_pitch).margin(1e-6f));
    REQUIRE(result.z == Approx(expected_yaw).margin(1e-6f));

    // Sanity: roll/pitch's sqrt-branch results are genuinely different
    // from what plain proportional would have given on those same axes -
    // proving the sqrt branch actually ran there, not merely compiled.
    const float plain_roll = (angle_kp_roll * angle_p_scale.x) * error.x;
    const float plain_pitch = (angle_kp_pitch * angle_p_scale.y) * error.y;
    REQUIRE(result.x != Approx(plain_roll).margin(1e-9f));
    REQUIRE(result.y != Approx(plain_pitch).margin(1e-9f));
}

// (3b) The per-axis choice also holds with use_sqrt_controller == false:
// every axis falls back to plain proportional regardless of its own
// acceleration max, since the shared flag gates ALL axes off together
// (only the acceleration-max half of the gate is per-axis).
TEST_CASE("update_ang_vel_target_from_att_error: use_sqrt_controller == false forces plain proportional on every axis",
          "[control][attitude_kinematics][update_ang_vel_target_from_att_error]") {
    const Vector3f error{0.2f, -0.25f, 0.1f};
    const float angle_kp = 4.5f;
    const Vector3f angle_p_scale{1.0f, 1.0f, 1.0f};
    const float accel_max = fwcpp::math::radians(400.0f); // non-zero on every axis
    const float dt = 0.0025f;

    const Vector3f result = update_ang_vel_target_from_att_error(
        error, angle_kp, angle_kp, angle_kp, angle_p_scale, accel_max, accel_max, accel_max,
        /*use_sqrt_controller=*/false, dt);

    REQUIRE(result.x == Approx(angle_kp * error.x).margin(1e-6f));
    REQUIRE(result.y == Approx(angle_kp * error.y).margin(1e-6f));
    REQUIRE(result.z == Approx(angle_kp * error.z).margin(1e-6f));
}

// (4) The real acceleration-halving fed to sqrt_controller, confirmed via
// a hand-computed expected sqrt_controller output on the pitch axis - the
// halved value (100 deg/s^2) sits comfortably inside [40,720], so this
// test isolates the /2.0f itself, not the clamp bounds (those get their
// own dedicated tests below).
TEST_CASE("update_ang_vel_target_from_att_error: the acceleration fed to sqrt_controller is HALF the axis max",
          "[control][attitude_kinematics][update_ang_vel_target_from_att_error]") {
    const Vector3f error{0.0f, 0.3f, 0.0f};
    const float angle_kp_pitch = 4.5f;
    const float accel_pitch_max = fwcpp::math::radians(200.0f); // /2 = 100 deg/s^2, inside [40,720]
    const float dt = 0.0025f;

    const Vector3f result = update_ang_vel_target_from_att_error(
        error, /*angle_kp_roll=*/0.0f, angle_kp_pitch, /*angle_kp_yaw=*/0.0f, Vector3f{1.0f, 1.0f, 1.0f},
        /*accel_roll_max_radss=*/0.0f, accel_pitch_max, /*accel_yaw_max_radss=*/0.0f,
        /*use_sqrt_controller=*/true, dt);

    const float expected_pitch = fwcpp::math::sqrt_controller(
        error.y, angle_kp_pitch,
        fwcpp::math::constrain_value(accel_pitch_max / 2.0f, kAccelRpControllerMinRadss, kAccelRpControllerMaxRadss),
        dt);
    REQUIRE(result.y == Approx(expected_pitch).margin(1e-6f));

    // And that really is different from what the UN-halved (full) axis
    // maximum would have produced, proving the /2.0f genuinely ran.
    const float unhalved = fwcpp::math::sqrt_controller(
        error.y, angle_kp_pitch,
        fwcpp::math::constrain_value(accel_pitch_max, kAccelRpControllerMinRadss, kAccelRpControllerMaxRadss), dt);
    REQUIRE(expected_pitch != Approx(unhalved).margin(1e-6f));
}

// (5a) Roll/pitch's own clamp UPPER bound (720 deg/s^2) - accel_roll_max
// chosen so its own /2.0f genuinely exceeds the ceiling, confirmed both
// by a sanity check on the pre-clamp arithmetic and by comparing the
// actual result against a reference built from the clamped constant.
TEST_CASE("update_ang_vel_target_from_att_error: roll/pitch clamp binds at its own 720 deg/s^2 upper bound",
          "[control][attitude_kinematics][update_ang_vel_target_from_att_error]") {
    const Vector3f error{0.3f, 0.0f, 0.0f};
    const float angle_kp_roll = 4.5f;
    const float accel_roll_max = fwcpp::math::radians(2000.0f); // /2 = 1000 deg/s^2, above the 720 ceiling
    const float dt = 0.0025f;
    REQUIRE(accel_roll_max / 2.0f > kAccelRpControllerMaxRadss); // sanity: genuinely exceeds the ceiling

    const Vector3f result = update_ang_vel_target_from_att_error(
        error, angle_kp_roll, 0.0f, 0.0f, Vector3f{1.0f, 1.0f, 1.0f}, accel_roll_max, 0.0f, 0.0f,
        /*use_sqrt_controller=*/true, dt);

    const float expected = fwcpp::math::sqrt_controller(error.x, angle_kp_roll, kAccelRpControllerMaxRadss, dt);
    REQUIRE(result.x == Approx(expected).margin(1e-6f));
}

// (5b) Roll/pitch's own clamp LOWER bound (40 deg/s^2) - accel_pitch_max
// chosen so its own /2.0f genuinely falls below the floor.
TEST_CASE("update_ang_vel_target_from_att_error: roll/pitch clamp binds at its own 40 deg/s^2 lower bound",
          "[control][attitude_kinematics][update_ang_vel_target_from_att_error]") {
    const Vector3f error{0.0f, 0.3f, 0.0f};
    const float angle_kp_pitch = 4.5f;
    const float accel_pitch_max = fwcpp::math::radians(2.0f); // /2 = 1 deg/s^2, below the 40 deg/s^2 floor
    const float dt = 0.0025f;
    REQUIRE(accel_pitch_max / 2.0f < kAccelRpControllerMinRadss); // sanity: genuinely below the floor

    const Vector3f result = update_ang_vel_target_from_att_error(
        error, 0.0f, angle_kp_pitch, 0.0f, Vector3f{1.0f, 1.0f, 1.0f}, 0.0f, accel_pitch_max, 0.0f,
        /*use_sqrt_controller=*/true, dt);

    const float expected = fwcpp::math::sqrt_controller(error.y, angle_kp_pitch, kAccelRpControllerMinRadss, dt);
    REQUIRE(result.y == Approx(expected).margin(1e-6f));
}

// (5c) Yaw's own, DIFFERENT clamp UPPER bound (120 deg/s^2, not roll/
// pitch's 720) - accel_yaw_max chosen so its own /2.0f exceeds yaw's own
// ceiling while staying nowhere near roll/pitch's much wider range,
// proving the axis-different bounds are genuinely independent constants,
// not the same pair reused for every axis.
TEST_CASE("update_ang_vel_target_from_att_error: yaw clamps at its OWN 120 deg/s^2 upper bound, not roll/pitch's 720",
          "[control][attitude_kinematics][update_ang_vel_target_from_att_error]") {
    const Vector3f error{0.0f, 0.0f, 0.3f};
    const float angle_kp_yaw = 4.5f;
    const float accel_yaw_max = fwcpp::math::radians(300.0f); // /2 = 150 deg/s^2: above yaw's 120 ceiling
    const float dt = 0.0025f;
    REQUIRE(accel_yaw_max / 2.0f > kAccelYControllerMaxRadss);      // sanity: exceeds yaw's own ceiling
    REQUIRE(accel_yaw_max / 2.0f < kAccelRpControllerMaxRadss);     // and stays well inside roll/pitch's own range

    const Vector3f result = update_ang_vel_target_from_att_error(
        error, 0.0f, 0.0f, angle_kp_yaw, Vector3f{1.0f, 1.0f, 1.0f}, 0.0f, 0.0f, accel_yaw_max,
        /*use_sqrt_controller=*/true, dt);

    const float expected = fwcpp::math::sqrt_controller(error.z, angle_kp_yaw, kAccelYControllerMaxRadss, dt);
    REQUIRE(result.z == Approx(expected).margin(1e-6f));
}

// (5d) Yaw's own, DIFFERENT clamp LOWER bound (10 deg/s^2, not roll/
// pitch's 40).
TEST_CASE("update_ang_vel_target_from_att_error: yaw clamps at its OWN 10 deg/s^2 lower bound, not roll/pitch's 40",
          "[control][attitude_kinematics][update_ang_vel_target_from_att_error]") {
    const Vector3f error{0.0f, 0.0f, 0.3f};
    const float angle_kp_yaw = 4.5f;
    const float accel_yaw_max = fwcpp::math::radians(4.0f); // /2 = 2 deg/s^2: below yaw's own 10 deg/s^2 floor
    const float dt = 0.0025f;
    REQUIRE(accel_yaw_max / 2.0f < kAccelYControllerMinRadss);  // sanity: below yaw's own floor
    REQUIRE(accel_yaw_max / 2.0f < kAccelRpControllerMinRadss); // and also below roll/pitch's own floor

    const Vector3f result = update_ang_vel_target_from_att_error(
        error, 0.0f, 0.0f, angle_kp_yaw, Vector3f{1.0f, 1.0f, 1.0f}, 0.0f, 0.0f, accel_yaw_max,
        /*use_sqrt_controller=*/true, dt);

    const float expected = fwcpp::math::sqrt_controller(error.z, angle_kp_yaw, kAccelYControllerMinRadss, dt);
    REQUIRE(result.z == Approx(expected).margin(1e-6f));
}

// (6) The two new roll/pitch acceleration-clamp constants are exactly
// radians()'s own output - matching CCP-022/023's own established
// ULP-precision test discipline, not a separately hand-typed
// approximation.
TEST_CASE("update_ang_vel_target_from_att_error: the two new roll/pitch clamp constants are exactly radians()'s own output",
          "[control][attitude_kinematics][update_ang_vel_target_from_att_error]") {
    REQUIRE(kAccelRpControllerMinRadss == fwcpp::math::radians(40.0f));
    REQUIRE(kAccelRpControllerMaxRadss == fwcpp::math::radians(720.0f));
}

// ---------------------------------------------------------------------
// CCP-025: update_attitude_target (real lines 979-986) and
// attitude_controller_run_quat (real lines 989-1027) - the real control
// loop itself. See attitude_kinematics.hpp's own "CCP-025 ADDENDUM"
// comment block for the full design writeup: the real three-way
// thrust-error branch, the single most important yaw-not-double-scaled
// asymmetry, and the architectural decision to expose every real
// persistent piece of state this function touches as an explicit
// output parameter.
//
// Shared fixture gains below are deliberately unremarkable (matching
// copter-rust's own COP-007 test fixture values, reused directly where
// convenient) - the tests exist to pin the real control-flow structure
// (composition order, branch thresholds, the yaw asymmetry), not to
// re-litigate CCP-018/023/024's own already-tested formulas.
// ---------------------------------------------------------------------

namespace {

struct ControllerGains {
    float rate_yaw_kp = 2.0f;
    float angle_yaw_kp = 1.0f;
    float angle_kp_roll = 6.0f;
    float angle_kp_pitch = 6.0f;
    float angle_kp_yaw = 4.0f;
    Vector3f angle_p_scale{1.0f, 1.0f, 1.0f};
    float accel_roll_max_radss = fwcpp::math::radians(400.0f);
    float accel_pitch_max_radss = fwcpp::math::radians(400.0f);
    float accel_yaw_max_radss = fwcpp::math::radians(200.0f);
    bool use_sqrt_controller = false;
    float ang_vel_roll_max_degs = 220.0f;
    float ang_vel_pitch_max_degs = 220.0f;
    float ang_vel_yaw_max_degs = 200.0f;
    float rate_wp_yaw_max_degs = 45.0f;
};

// Reconstructs the pre-branch quantities (steps 1-6 of
// attitude_controller_run_quat) independently, using this module's own
// already-verified CCP-023/024/018 building blocks directly, so tests
// below can hand-apply the real (or a deliberately wrong) branch
// formula on top without re-deriving thrust_heading_rotation_angles or
// update_ang_vel_target_from_att_error's own arithmetic.
struct PreBranchState {
    Quaternion target; // possibly mutated by thrust_heading_rotation_angles
    float thrust_angle_rad = 0.0f;
    float thrust_error_angle_rad = 0.0f;
    Vector3f base_ang_vel_body_rads; // after steps 2+3, before the branch
    Vector3f feedforward;            // ang_vel_body_feedforward, step 5
};

PreBranchState compute_pre_branch_state(Quaternion target, const Quaternion& body,
                                         const Vector3f& ang_vel_target_rads, const ControllerGains& g, float dt) {
    PreBranchState s;
    Vector3f attitude_error{};
    thrust_heading_rotation_angles(target, body, attitude_error, s.thrust_angle_rad, s.thrust_error_angle_rad,
                                    g.rate_yaw_kp, g.angle_yaw_kp, g.accel_yaw_max_radss);
    s.target = target;

    s.base_ang_vel_body_rads = update_ang_vel_target_from_att_error(
        attitude_error, g.angle_kp_roll, g.angle_kp_pitch, g.angle_kp_yaw, g.angle_p_scale, g.accel_roll_max_radss,
        g.accel_pitch_max_radss, g.accel_yaw_max_radss, g.use_sqrt_controller, dt);
    ang_vel_limit(s.base_ang_vel_body_rads, fwcpp::math::radians(g.ang_vel_roll_max_degs),
                  fwcpp::math::radians(g.ang_vel_pitch_max_degs), fwcpp::math::radians(g.ang_vel_yaw_max_degs));

    const Quaternion rotation_target_to_body = body.inverse() * s.target;
    s.feedforward = rotation_target_to_body * ang_vel_target_rads;
    return s;
}

} // namespace

// ---------------------------------------------------------------------
// update_attitude_target
// ---------------------------------------------------------------------

TEST_CASE("update_attitude_target advances the target by the commanded rate", "[control][attitude_kinematics][update_attitude_target]") {
    Quaternion target = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f rate{0.0f, 0.0f, 1.0f}; // 1 rad/s of yaw
    const float dt = 0.1f;

    update_attitude_target(target, rate, dt);

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    target.to_euler(roll, pitch, yaw);
    REQUIRE(yaw == Approx(0.1f).margin(1e-4f));
    REQUIRE(roll == Approx(0.0f).margin(1e-4f));
    REQUIRE(pitch == Approx(0.0f).margin(1e-4f));
}

// THE SINGLE MOST IMPORTANT TEST FOR update_attitude_target - see this
// file's own header comment and attitude_kinematics.hpp's own "CCP-025
// ADDENDUM" banner for why explicit normalize() matters: composing a
// small rotation onto a quaternion every iteration accumulates drift,
// and at 400 Hz it is measurable within seconds. 4000 iterations,
// matching copter-rust's own COP-007 test exactly.
TEST_CASE("update_attitude_target: the quaternion stays normalised over thousands of iterations",
          "[control][attitude_kinematics][update_attitude_target]") {
    Quaternion q = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f rate{0.3f, -0.2f, 0.5f};
    const float dt = 0.0025f; // 400 Hz

    for (int i = 0; i < 4000; ++i) {
        update_attitude_target(q, rate, dt);
    }

    REQUIRE(q.length() == Approx(1.0f).margin(1e-4f));
    REQUIRE(q.is_unit_length());
}

// ---------------------------------------------------------------------
// attitude_controller_run_quat: the real three-way branch
// ---------------------------------------------------------------------

// (1) Under 30 degrees of thrust error: full feedforward on all three
// axes, feedforward_scalar stays at its initial 1.0f (unused, per the
// real "else" branch), and the result is exactly base + feedforward on
// every axis - a plain whole-vector add, not a per-axis scaled one.
TEST_CASE("attitude_controller_run_quat: under 30 degrees of thrust error, full feedforward on all three axes",
          "[control][attitude_kinematics][attitude_controller_run_quat]") {
    const ControllerGains g;
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Quaternion original_target = attitude(0.15f, 0.0f, 0.0f); // ~8.6 degrees of lean, well under 30
    const Vector3f ang_vel_target_rads{0.0f, 0.0f, 1.0f};
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    const float dt = 0.0025f;

    const PreBranchState ref = compute_pre_branch_state(original_target, body, ang_vel_target_rads, g, dt);
    REQUIRE(ref.thrust_error_angle_rad < kAttitudeThrustErrorAngleRad); // sanity: genuinely under 30 degrees

    Quaternion target = original_target;
    float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
    Quaternion attitude_ang_error;
    Vector3f ang_vel_body_rads;
    attitude_controller_run_quat(target, body, ang_vel_target_rads, gyro, g.rate_yaw_kp, g.angle_yaw_kp,
                                  g.angle_kp_roll, g.angle_kp_pitch, g.angle_kp_yaw, g.angle_p_scale,
                                  g.accel_roll_max_radss, g.accel_pitch_max_radss, g.accel_yaw_max_radss,
                                  g.use_sqrt_controller, g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs,
                                  g.ang_vel_yaw_max_degs, dt, thrust_angle, thrust_error_angle, feedforward_scalar,
                                  attitude_ang_error, ang_vel_body_rads);

    REQUIRE(feedforward_scalar == 1.0f);
    REQUIRE(ang_vel_body_rads.x == Approx(ref.base_ang_vel_body_rads.x + ref.feedforward.x).margin(1e-6f));
    REQUIRE(ang_vel_body_rads.y == Approx(ref.base_ang_vel_body_rads.y + ref.feedforward.y).margin(1e-6f));
    REQUIRE(ang_vel_body_rads.z == Approx(ref.base_ang_vel_body_rads.z + ref.feedforward.z).margin(1e-6f));

    // Sanity: the feedforward genuinely contributes something
    // non-negligible on at least one axis, proving this isn't a
    // vacuously-true near-zero comparison.
    REQUIRE(ref.feedforward.length() > 1e-3f);
}

// (2) Over 60 degrees: yaw is a direct, EXACT (bit-for-bit) overwrite
// with gyro.z, and roll/pitch receive LITERALLY ZERO feedforward - not
// merely heavily reduced - meaning they are bit-for-bit identical to
// whatever update_ang_vel_target_from_att_error + ang_vel_limit alone
// already produced.
TEST_CASE("attitude_controller_run_quat: over 60 degrees, yaw is overwritten by the gyro and roll/pitch get "
          "literally zero feedforward",
          "[control][attitude_kinematics][attitude_controller_run_quat]") {
    const ControllerGains g;
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    // 80 degrees of lean plus a heading component, matching copter-
    // rust's own equivalent case shape.
    const Quaternion original_target = attitude(fwcpp::math::radians(80.0f), 0.0f, 1.0f);
    const Vector3f ang_vel_target_rads{0.0f, 0.0f, 1.0f};
    const float measured_yaw_rate = 0.42f;
    const Vector3f gyro{0.0f, 0.0f, measured_yaw_rate};
    const float dt = 0.0025f;

    const PreBranchState ref = compute_pre_branch_state(original_target, body, ang_vel_target_rads, g, dt);
    REQUIRE(ref.thrust_error_angle_rad > 2.0f * kAttitudeThrustErrorAngleRad); // sanity: genuinely over 60 degrees

    Quaternion target = original_target;
    float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
    Quaternion attitude_ang_error;
    Vector3f ang_vel_body_rads;
    attitude_controller_run_quat(target, body, ang_vel_target_rads, gyro, g.rate_yaw_kp, g.angle_yaw_kp,
                                  g.angle_kp_roll, g.angle_kp_pitch, g.angle_kp_yaw, g.angle_p_scale,
                                  g.accel_roll_max_radss, g.accel_pitch_max_radss, g.accel_yaw_max_radss,
                                  g.use_sqrt_controller, g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs,
                                  g.ang_vel_yaw_max_degs, dt, thrust_angle, thrust_error_angle, feedforward_scalar,
                                  attitude_ang_error, ang_vel_body_rads);

    // Yaw: exact overwrite, bit-for-bit.
    REQUIRE(ang_vel_body_rads.z == measured_yaw_rate);

    // Roll/pitch: bit-for-bit identical to the pre-branch base value -
    // proving zero feedforward was added, not merely a small amount.
    REQUIRE(ang_vel_body_rads.x == ref.base_ang_vel_body_rads.x);
    REQUIRE(ang_vel_body_rads.y == ref.base_ang_vel_body_rads.y);

    // Sanity: the feedforward that was withheld from roll/pitch really
    // was non-negligible, so "zero feedforward" is a meaningful claim
    // here rather than there being nothing to withhold in the first
    // place.
    REQUIRE(std::fabs(ref.feedforward.x) + std::fabs(ref.feedforward.y) > 1e-3f);
}

// (3) The fade-scalar formula, sampled at several points across the
// 30-60 degree band rather than just the midpoint - a scalar that
// happened to be right at 45 degrees could still be wrong elsewhere in
// the band. Each sample is checked against the exact real formula, and
// the sequence is confirmed monotonically decreasing.
TEST_CASE("attitude_controller_run_quat: the fade scalar matches the real linear formula across the whole 30-60 "
          "degree band",
          "[control][attitude_kinematics][attitude_controller_run_quat]") {
    const ControllerGains g;
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f ang_vel_target_rads{0.0f, 0.0f, 0.0f};
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    const float dt = 0.0025f;

    float previous_scalar = 1.1f; // above any real feedforward_scalar, so the first comparison always passes
    for (float degrees : {31.0f, 35.0f, 40.0f, 45.0f, 50.0f, 55.0f, 59.0f}) {
        Quaternion target = attitude(fwcpp::math::radians(degrees), 0.0f, 0.0f);

        float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
        Quaternion attitude_ang_error;
        Vector3f ang_vel_body_rads;
        attitude_controller_run_quat(target, body, ang_vel_target_rads, gyro, g.rate_yaw_kp, g.angle_yaw_kp,
                                      g.angle_kp_roll, g.angle_kp_pitch, g.angle_kp_yaw, g.angle_p_scale,
                                      g.accel_roll_max_radss, g.accel_pitch_max_radss, g.accel_yaw_max_radss,
                                      g.use_sqrt_controller, g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs,
                                      g.ang_vel_yaw_max_degs, dt, thrust_angle, thrust_error_angle,
                                      feedforward_scalar, attitude_ang_error, ang_vel_body_rads);

        REQUIRE(thrust_error_angle > kAttitudeThrustErrorAngleRad); // sanity: genuinely inside the fade band
        REQUIRE(thrust_error_angle < 2.0f * kAttitudeThrustErrorAngleRad);

        const float expected_scalar =
            1.0f - (thrust_error_angle - kAttitudeThrustErrorAngleRad) / kAttitudeThrustErrorAngleRad;
        REQUIRE(feedforward_scalar == Approx(expected_scalar).margin(1e-5f));

        REQUIRE(feedforward_scalar < previous_scalar);
        previous_scalar = feedforward_scalar;
    }

    // Should not have reached zero before the far (60-degree) threshold.
    REQUIRE(previous_scalar > 0.0f);
}

// (4) THE CRITICAL YAW-NOT-DOUBLE-SCALED TEST - see this file's own
// header comment and attitude_kinematics.hpp's own "CCP-025 ADDENDUM"
// banner for the full writeup of this asymmetry. Constructs a fade-band
// case, computes the real (correct) yaw result via the actual function
// under test, then independently reconstructs BOTH the correct formula
// and the naive "scale yaw at the point of addition too" formula from
// the same underlying pre-branch quantities, and confirms: the real
// implementation matches the correct formula, and differs from the
// naive one by a large, non-negligible margin (not merely a rounding-
// sized discrepancy).
TEST_CASE("attitude_controller_run_quat: the yaw feedforward is not double-scaled in the fade band",
          "[control][attitude_kinematics][attitude_controller_run_quat][yaw_asymmetry]") {
    const ControllerGains g;
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    // A pure 45-degree lean: dead center of the 30-60 degree band, so
    // feedforward_scalar should land at exactly 0.5.
    const Quaternion original_target = attitude(fwcpp::math::radians(45.0f), 0.0f, 0.0f);
    // A deliberately large target yaw rate so the frame-rotation
    // coupling produces a large, unmistakable ang_vel_body_feedforward.z
    // component even though the target itself has no heading error.
    const Vector3f ang_vel_target_rads{0.0f, 0.0f, 1.0f};
    const float measured_yaw_rate = 0.05f;
    const Vector3f gyro{0.0f, 0.0f, measured_yaw_rate};
    const float dt = 0.0025f;

    const PreBranchState ref = compute_pre_branch_state(original_target, body, ang_vel_target_rads, g, dt);
    REQUIRE(ref.thrust_error_angle_rad > kAttitudeThrustErrorAngleRad); // sanity: genuinely in the fade band
    REQUIRE(ref.thrust_error_angle_rad < 2.0f * kAttitudeThrustErrorAngleRad);
    // Sanity: the feedforward.z the branch will operate on is genuinely
    // large, not a near-zero value that would make any formula
    // difference invisible.
    REQUIRE(std::fabs(ref.feedforward.z) > 0.3f);

    const float expected_scalar =
        1.0f - (ref.thrust_error_angle_rad - kAttitudeThrustErrorAngleRad) / kAttitudeThrustErrorAngleRad;
    REQUIRE(expected_scalar == Approx(0.5f).margin(0.05f)); // dead center of the band

    // The REAL (correct) formula: unscaled add, then blend the
    // already-summed value.
    const float correct_z_sum = ref.base_ang_vel_body_rads.z + ref.feedforward.z;
    const float correct_z = gyro.z * (1.0f - expected_scalar) + correct_z_sum * expected_scalar;

    // The NAIVE (incorrect) formula: scale the feedforward at the point
    // of addition too, "for symmetry" with roll/pitch, then blend -
    // this is what a port that "cleaned up" the yaw branch to look
    // parallel to roll/pitch would produce.
    const float naive_z_sum = ref.base_ang_vel_body_rads.z + ref.feedforward.z * expected_scalar;
    const float naive_z = gyro.z * (1.0f - expected_scalar) + naive_z_sum * expected_scalar;

    // The two formulas must genuinely differ for this test to prove
    // anything - a large, non-negligible margin, not a rounding-sized
    // discrepancy.
    REQUIRE(std::fabs(correct_z - naive_z) > 0.05f);

    Quaternion target = original_target;
    float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
    Quaternion attitude_ang_error;
    Vector3f ang_vel_body_rads;
    attitude_controller_run_quat(target, body, ang_vel_target_rads, gyro, g.rate_yaw_kp, g.angle_yaw_kp,
                                  g.angle_kp_roll, g.angle_kp_pitch, g.angle_kp_yaw, g.angle_p_scale,
                                  g.accel_roll_max_radss, g.accel_pitch_max_radss, g.accel_yaw_max_radss,
                                  g.use_sqrt_controller, g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs,
                                  g.ang_vel_yaw_max_degs, dt, thrust_angle, thrust_error_angle, feedforward_scalar,
                                  attitude_ang_error, ang_vel_body_rads);

    // The actual implementation matches the correct (asymmetric)
    // formula...
    REQUIRE(ang_vel_body_rads.z == Approx(correct_z).margin(1e-5f));
    // ...and genuinely differs from the naive (double-scaled) one, by
    // the same large margin established above - proving this is the
    // real, structurally different two-step yaw treatment, not the
    // "obvious" symmetric one.
    REQUIRE(ang_vel_body_rads.z != Approx(naive_z).margin(1e-5f));
    REQUIRE(std::fabs(ang_vel_body_rads.z - naive_z) > 0.05f);
}

// (5) attitude_ang_error is recomputed after step 1's potential
// mutation of attitude_target, using the real body.inverse() *
// attitude_target composition - confirmed directly against an
// independently-built reference using the (possibly mutated) target
// this same call actually produced.
TEST_CASE("attitude_controller_run_quat: attitude_ang_error uses the post-mutation attitude_target",
          "[control][attitude_kinematics][attitude_controller_run_quat]") {
    const ControllerGains g;
    const Quaternion body = attitude(0.3f, -0.25f, 0.6f);
    // A large heading delta, chosen (matching the established
    // thrust_heading_rotation_angles test convention above) so the
    // yaw-clamp guard genuinely fires and attitude_target is genuinely
    // reassigned.
    Quaternion heading_delta;
    heading_delta.from_axis_angle(Vector3f{0.0f, 0.0f, -1.0f}, 1.2f); // well beyond kYawMaxErrorAngleRad (45 deg) on its own, guaranteeing the clamp fires
    const Quaternion original_target = body * heading_delta;
    const Vector3f ang_vel_target_rads{0.1f, -0.1f, 0.2f};
    const Vector3f gyro{0.01f, 0.02f, 0.03f};
    const float dt = 0.0025f;

    Quaternion target = original_target;
    float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
    Quaternion attitude_ang_error;
    Vector3f ang_vel_body_rads;
    attitude_controller_run_quat(target, body, ang_vel_target_rads, gyro, g.rate_yaw_kp, g.angle_yaw_kp,
                                  g.angle_kp_roll, g.angle_kp_pitch, g.angle_kp_yaw, g.angle_p_scale,
                                  g.accel_roll_max_radss, g.accel_pitch_max_radss, g.accel_yaw_max_radss,
                                  g.use_sqrt_controller, g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs,
                                  g.ang_vel_yaw_max_degs, dt, thrust_angle, thrust_error_angle, feedforward_scalar,
                                  attitude_ang_error, ang_vel_body_rads);

    // Sanity: the mutation genuinely happened.
    REQUIRE(target.q1 != original_target.q1);

    const Quaternion expected = body.inverse() * target;
    REQUIRE(attitude_ang_error.q1 == Approx(expected.q1).margin(1e-6f));
    REQUIRE(attitude_ang_error.q2 == Approx(expected.q2).margin(1e-6f));
    REQUIRE(attitude_ang_error.q3 == Approx(expected.q3).margin(1e-6f));
    REQUIRE(attitude_ang_error.q4 == Approx(expected.q4).margin(1e-6f));

    // And it is NOT the same as body.inverse() * original_target (the
    // pre-mutation value) - proving this really did recompute using the
    // post-mutation target, not reuse a stale rotation_target_to_body.
    const Quaternion stale = body.inverse() * original_target;
    const bool differs_from_stale = std::fabs(attitude_ang_error.q1 - stale.q1) > 1e-4f ||
                                     std::fabs(attitude_ang_error.q2 - stale.q2) > 1e-4f ||
                                     std::fabs(attitude_ang_error.q3 - stale.q3) > 1e-4f ||
                                     std::fabs(attitude_ang_error.q4 - stale.q4) > 1e-4f;
    REQUIRE(differs_from_stale);
}

// (6) kAttitudeThrustErrorAngleRad is exactly radians()'s own output -
// matching CCP-022/023/024's own established ULP-precision test
// discipline.
TEST_CASE("attitude_controller_run_quat: kAttitudeThrustErrorAngleRad is exactly radians(30.0f)",
          "[control][attitude_kinematics][attitude_controller_run_quat]") {
    REQUIRE(kAttitudeThrustErrorAngleRad == fwcpp::math::radians(30.0f));
}

// =======================================================================
// command_model_rate_predictor (CCP-026) - see attitude_kinematics.hpp's
// own "CCP-026 ADDENDUM" comment block for the full design writeup: the
// corrected CCP-022/023 deferral reasoning (this function needed no new
// per-axis STATE, only plain explicit parameters, exactly like CCP-023
// already found for thrust_heading_rotation_angles), and the
// independently re-verified D-025 dt-parameter quirk with this port's
// chosen resolution (a single dt_s parameter, not a second unused one
// matching real upstream's own).
// =======================================================================

namespace {
// Default gains for command_model_rate_predictor tests. Roll/pitch
// values are deliberately DIFFERENT from each other everywhere (max
// rates, accel maxima, kP gains) so a test that accidentally swapped
// the two axes, or reused one axis's constant for the other, would
// produce a detectably wrong result rather than an equivalent one.
struct PredictorGains {
    float angle_kp_roll = 5.0f;
    float angle_kp_pitch = 3.0f;
    Vector3f angle_p_scale{1.1f, 0.8f, 1.0f}; // .z deliberately set too, to prove it is never read
    float ang_vel_roll_max_degs = 90.0f;
    float ang_vel_pitch_max_degs = 60.0f;
    float accel_roll_max_radss = fwcpp::math::radians(400.0f);
    float accel_pitch_max_radss = fwcpp::math::radians(250.0f);
    float input_tc = 0.15f;
    float dt_s = 0.0025f;
};
} // namespace

// (1) rate_bf_ff_enabled == true: both axes must delegate to
// attitude_command_model with the EXACT real argument mapping. Ground
// truth is built by independently calling this file's own already-
// verified attitude_command_model with the hand-traced real mapping
// (wrap_PI(error), literal 0.0 desired_ang_vel, the per-axis max/accel/
// input_tc/dt_s), then applying the same unconditional final ang_vel_
// limit re-clamp command_model_rate_predictor itself always applies -
// not a "plausible-looking" independently-derived number, but the exact
// same real computation, reproduced call-for-call.
TEST_CASE("command_model_rate_predictor: rate_bf_ff_enabled == true delegates to attitude_command_model with the "
          "exact real argument mapping, including its own in/out target_ang_vel_rads/target_ang_accel_rads state",
          "[control][attitude_kinematics][command_model_rate_predictor]") {
    const PredictorGains g;
    // Chosen outside (-pi, pi) on both axes so a port that forgot the
    // real wrap_PI() call would diverge from one that applied it.
    const Vector2f error_angle_rad{3.6f, -3.5f};

    // IMPORTANT, independently re-derived while writing this test (see
    // attitude_kinematics.hpp's own "CCP-026 ADDENDUM" banner's "IMPORTANT"
    // paragraph): target_ang_vel_rads/target_ang_accel_rads are genuine
    // in/out STATE in this branch, not fresh outputs - attitude_command_
    // model's own shape_angle_vel_accel call reads the INCOMING target_
    // ang_vel as its current-velocity input, and its own final "+=" step
    // depends on the incoming target_ang_vel too. A first attempt at this
    // test seeded these with an arbitrary sentinel expecting a full
    // overwrite - that assumption was WRONG and the test caught its own
    // author's mistake immediately (a large, easily-diagnosed mismatch,
    // not a subtle one). Nonzero, axis-distinct starting values below,
    // matched identically between the "expected" computation and the
    // real call, so this test exercises the real argument mapping without
    // repeating that mistake.
    const Vector2f starting_vel{0.05f, -0.03f};
    const Vector2f starting_accel{0.2f, -0.1f};

    float expected_vel_x = starting_vel.x, expected_accel_x = starting_accel.x;
    attitude_command_model(fwcpp::math::wrap_PI(error_angle_rad.x), 0.0f, expected_vel_x, expected_accel_x,
                            fwcpp::math::radians(g.ang_vel_roll_max_degs), g.accel_roll_max_radss, g.input_tc,
                            g.dt_s);
    float expected_vel_y = starting_vel.y, expected_accel_y = starting_accel.y;
    attitude_command_model(fwcpp::math::wrap_PI(error_angle_rad.y), 0.0f, expected_vel_y, expected_accel_y,
                            fwcpp::math::radians(g.ang_vel_pitch_max_degs), g.accel_pitch_max_radss, g.input_tc,
                            g.dt_s);
    Vector3f expected_ang_vel(expected_vel_x, expected_vel_y, 0.0f);
    ang_vel_limit(expected_ang_vel, fwcpp::math::radians(g.ang_vel_roll_max_degs),
                   fwcpp::math::radians(g.ang_vel_pitch_max_degs), 0.0f);

    Vector2f target_ang_vel_rads = starting_vel;
    Vector2f target_ang_accel_rads = starting_accel;
    command_model_rate_predictor(error_angle_rad, target_ang_vel_rads, target_ang_accel_rads,
                                  /*rate_bf_ff_enabled=*/true, g.angle_kp_roll, g.angle_kp_pitch, g.angle_p_scale,
                                  g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs, g.accel_roll_max_radss,
                                  g.accel_pitch_max_radss, g.input_tc, g.dt_s);

    REQUIRE(target_ang_vel_rads.x == expected_ang_vel.x);
    REQUIRE(target_ang_vel_rads.y == expected_ang_vel.y);
    REQUIRE(target_ang_accel_rads.x == expected_accel_x);
    REQUIRE(target_ang_accel_rads.y == expected_accel_y);

    // Sanity: the two axes really did produce different numbers here -
    // if this test would pass with roll/pitch's own distinct constants
    // silently swapped, it would not actually be testing the mapping.
    REQUIRE(target_ang_vel_rads.x != target_ang_vel_rads.y);

    // And a genuinely different starting state produces a genuinely
    // different result, proving the incoming target_ang_vel_rads/
    // target_ang_accel_rads values really do flow into the computation
    // rather than being discarded and recomputed from scratch.
    Vector2f zero_start_vel{0.0f, 0.0f};
    Vector2f zero_start_accel{0.0f, 0.0f};
    command_model_rate_predictor(error_angle_rad, zero_start_vel, zero_start_accel, /*rate_bf_ff_enabled=*/true,
                                  g.angle_kp_roll, g.angle_kp_pitch, g.angle_p_scale, g.ang_vel_roll_max_degs,
                                  g.ang_vel_pitch_max_degs, g.accel_roll_max_radss, g.accel_pitch_max_radss,
                                  g.input_tc, g.dt_s);
    REQUIRE(zero_start_vel.x != target_ang_vel_rads.x);
    REQUIRE(zero_start_accel.x != target_ang_accel_rads.x);
}

// (2) rate_bf_ff_enabled == false: the plain proportional formula on
// both axes, AND target_ang_accel_rads left genuinely untouched by this
// branch - seeded with a deliberate sentinel beforehand, confirmed to
// survive the call unmodified. Max rates are set generously large here
// specifically so the final ang_vel_limit re-clamp does not bind,
// isolating this test to the proportional formula itself (the clamp
// binding/pass-through cases are tested separately below).
TEST_CASE("command_model_rate_predictor: rate_bf_ff_enabled == false computes the plain proportional formula on "
          "both axes and leaves target_ang_accel_rads completely unchanged",
          "[control][attitude_kinematics][command_model_rate_predictor]") {
    PredictorGains g;
    g.ang_vel_roll_max_degs = 100000.0f;  // large enough the final
    g.ang_vel_pitch_max_degs = 100000.0f; // re-clamp never binds here.
    const Vector2f error_angle_rad{0.2f, -0.35f};

    const float expected_vel_x = (g.angle_kp_roll * g.angle_p_scale.x) * fwcpp::math::wrap_PI(error_angle_rad.x);
    const float expected_vel_y = (g.angle_kp_pitch * g.angle_p_scale.y) * fwcpp::math::wrap_PI(error_angle_rad.y);

    Vector2f target_ang_vel_rads{0.0f, 0.0f};
    Vector2f target_ang_accel_rads{-777.25f, 888.5f}; // deliberate sentinel
    command_model_rate_predictor(error_angle_rad, target_ang_vel_rads, target_ang_accel_rads,
                                  /*rate_bf_ff_enabled=*/false, g.angle_kp_roll, g.angle_kp_pitch, g.angle_p_scale,
                                  g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs, g.accel_roll_max_radss,
                                  g.accel_pitch_max_radss, g.input_tc, g.dt_s);

    REQUIRE(target_ang_vel_rads.x == Approx(expected_vel_x).margin(1e-6f));
    REQUIRE(target_ang_vel_rads.y == Approx(expected_vel_y).margin(1e-6f));

    // The sentinel must survive bit-for-bit - this branch never writes
    // target_ang_accel_rads at all, and neither does the unconditional
    // final ang_vel_limit step (it only ever touches target_ang_vel_rads).
    REQUIRE(target_ang_accel_rads.x == -777.25f);
    REQUIRE(target_ang_accel_rads.y == 888.5f);
}

// (3) The final ang_vel_limit re-clamp DOES bind when the pre-limit
// proportional output genuinely exceeds the configured max. Uses the
// rate_bf_ff_enabled == false path (the simplest way to construct a
// large pre-limit value deterministically), with gains chosen so the
// pre-limit values are confirmed - by an explicit sanity assertion,
// not merely by construction - to exceed the configured per-axis max
// before the clamp is even considered.
TEST_CASE("command_model_rate_predictor: the final ang_vel_limit re-clamp binds when the pre-limit values exceed "
          "the configured max",
          "[control][attitude_kinematics][command_model_rate_predictor]") {
    PredictorGains g;
    g.angle_kp_roll = 100.0f;
    g.angle_kp_pitch = 80.0f;
    g.angle_p_scale = Vector3f{1.0f, 1.0f, 1.0f};
    g.ang_vel_roll_max_degs = 45.0f;
    g.ang_vel_pitch_max_degs = 45.0f;
    const Vector2f error_angle_rad{1.0f, 0.9f};

    const float pre_vel_x = g.angle_kp_roll * fwcpp::math::wrap_PI(error_angle_rad.x);
    const float pre_vel_y = g.angle_kp_pitch * fwcpp::math::wrap_PI(error_angle_rad.y);
    // Sanity: this case really does exceed the configured max on both
    // axes, or the clamp below would not actually be exercised.
    REQUIRE(std::fabs(pre_vel_x) > fwcpp::math::radians(g.ang_vel_roll_max_degs));
    REQUIRE(std::fabs(pre_vel_y) > fwcpp::math::radians(g.ang_vel_pitch_max_degs));

    Vector3f expected_ang_vel(pre_vel_x, pre_vel_y, 0.0f);
    ang_vel_limit(expected_ang_vel, fwcpp::math::radians(g.ang_vel_roll_max_degs),
                   fwcpp::math::radians(g.ang_vel_pitch_max_degs), 0.0f);
    // The clamp must have actually changed something, or this test
    // would not distinguish "clamped" from "coincidentally identical".
    REQUIRE(expected_ang_vel.x != Approx(pre_vel_x));

    Vector2f target_ang_vel_rads{0.0f, 0.0f};
    Vector2f target_ang_accel_rads{0.0f, 0.0f};
    command_model_rate_predictor(error_angle_rad, target_ang_vel_rads, target_ang_accel_rads,
                                  /*rate_bf_ff_enabled=*/false, g.angle_kp_roll, g.angle_kp_pitch, g.angle_p_scale,
                                  g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs, g.accel_roll_max_radss,
                                  g.accel_pitch_max_radss, g.input_tc, g.dt_s);

    REQUIRE(target_ang_vel_rads.x == Approx(expected_ang_vel.x).margin(1e-6f));
    REQUIRE(target_ang_vel_rads.y == Approx(expected_ang_vel.y).margin(1e-6f));
}

// (4) The mirror case: pre-limit values that do NOT exceed the
// configured max pass through completely unchanged (not merely "close
// to" the pre-limit value).
TEST_CASE("command_model_rate_predictor: the final ang_vel_limit re-clamp passes pre-limit values through "
          "unchanged when they do not exceed the configured max",
          "[control][attitude_kinematics][command_model_rate_predictor]") {
    PredictorGains g;
    g.angle_kp_roll = 0.5f;
    g.angle_kp_pitch = 0.4f;
    g.angle_p_scale = Vector3f{1.0f, 1.0f, 1.0f};
    g.ang_vel_roll_max_degs = 90.0f;
    g.ang_vel_pitch_max_degs = 90.0f;
    const Vector2f error_angle_rad{0.1f, -0.08f};

    const float pre_vel_x = g.angle_kp_roll * fwcpp::math::wrap_PI(error_angle_rad.x);
    const float pre_vel_y = g.angle_kp_pitch * fwcpp::math::wrap_PI(error_angle_rad.y);
    REQUIRE(std::fabs(pre_vel_x) < fwcpp::math::radians(g.ang_vel_roll_max_degs));
    REQUIRE(std::fabs(pre_vel_y) < fwcpp::math::radians(g.ang_vel_pitch_max_degs));

    Vector2f target_ang_vel_rads{0.0f, 0.0f};
    Vector2f target_ang_accel_rads{0.0f, 0.0f};
    command_model_rate_predictor(error_angle_rad, target_ang_vel_rads, target_ang_accel_rads,
                                  /*rate_bf_ff_enabled=*/false, g.angle_kp_roll, g.angle_kp_pitch, g.angle_p_scale,
                                  g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs, g.accel_roll_max_radss,
                                  g.accel_pitch_max_radss, g.input_tc, g.dt_s);

    REQUIRE(target_ang_vel_rads.x == Approx(pre_vel_x).margin(1e-6f));
    REQUIRE(target_ang_vel_rads.y == Approx(pre_vel_y).margin(1e-6f));
}

// (5) Yaw is never referenced anywhere in this function - a STRUCTURAL
// confirmation, not merely a numerical one. error_angle_rad, target_
// ang_vel_rads, and target_ang_accel_rads are all math::Vector2f - a
// type with only x/y members, not x/y/z - so there is no yaw-shaped
// slot anywhere in this function's own signature for a value to flow
// through in the first place. This is a stronger claim than "yaw
// happens to be 0.0f in every test case here": there is no third
// component to BE anything.
TEST_CASE("command_model_rate_predictor: yaw cannot be referenced anywhere - a structural, not numerical, "
          "confirmation",
          "[control][attitude_kinematics][command_model_rate_predictor]") {
    // Vector2f is exactly two floats - no z member exists for a yaw
    // component to occupy. (A hypothetical `error_angle_rad.z` would
    // simply fail to compile - there is nothing to assert about a
    // member that structurally does not exist.)
    static_assert(sizeof(Vector2f) == 2 * sizeof(float),
                  "Vector2f must be exactly {x, y} - no z/yaw slot - for this structural claim to hold.");

    // The function's own real signature (see command_model_rate_
    // predictor's declaration in attitude_kinematics.hpp) likewise has
    // no ang_vel_yaw_max_degs, accel_yaw_max_radss, or any other
    // yaw-suffixed parameter at all, unlike attitude_controller_run_quat
    // above which genuinely does take yaw parameters throughout. There
    // is nothing further to assert at runtime here - the absence is in
    // the type signature itself, confirmed at compile time by the
    // static_assert above and by this function simply having no such
    // parameter to pass one through.
    SUCCEED("no yaw-shaped parameter exists anywhere in this function's signature");
}

// (6) THE dt_s QUIRK RESOLUTION: this port's command_model_rate_
// predictor takes exactly ONE dt-shaped parameter (dt_s), deliberately
// NOT a second, unused one matching real upstream's own D-025-flagged
// `dt` parameter (see attitude_kinematics.hpp's own "CCP-026 ADDENDUM"
// banner, "RESOLUTION CHOSEN"). This test proves dt_s is genuinely
// LIVE, not a phantom unused parameter reintroducing the same hazard
// under a different name: two otherwise-identical calls differing only
// in dt_s must produce different outputs, since dt_s is the exact value
// forwarded into attitude_command_model's own dt parameter (which does
// read it - see attitude_command_model's own already-established dt
// tests above) whenever rate_bf_ff_enabled is true.
TEST_CASE("command_model_rate_predictor: dt_s is the only dt-shaped parameter, and it is genuinely used, not a "
          "phantom unused one",
          "[control][attitude_kinematics][command_model_rate_predictor]") {
    const PredictorGains g;
    const Vector2f error_angle_rad{0.5f, -0.4f};

    Vector2f vel_a{0.0f, 0.0f}, accel_a{0.0f, 0.0f};
    command_model_rate_predictor(error_angle_rad, vel_a, accel_a, /*rate_bf_ff_enabled=*/true, g.angle_kp_roll,
                                  g.angle_kp_pitch, g.angle_p_scale, g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs,
                                  g.accel_roll_max_radss, g.accel_pitch_max_radss, g.input_tc, /*dt_s=*/0.0025f);

    Vector2f vel_b{0.0f, 0.0f}, accel_b{0.0f, 0.0f};
    command_model_rate_predictor(error_angle_rad, vel_b, accel_b, /*rate_bf_ff_enabled=*/true, g.angle_kp_roll,
                                  g.angle_kp_pitch, g.angle_p_scale, g.ang_vel_roll_max_degs, g.ang_vel_pitch_max_degs,
                                  g.accel_roll_max_radss, g.accel_pitch_max_radss, g.input_tc, /*dt_s=*/0.01f);

    REQUIRE(accel_a.x != accel_b.x);
    REQUIRE(accel_a.y != accel_b.y);
    REQUIRE(vel_a.x != vel_b.x);
    REQUIRE(vel_a.y != vel_b.y);
}

// =======================================================================
// input_euler_angle_roll_pitch_euler_rate_yaw_rad (+ its trivial _cd
// wrapper) - CCP-029, the stabilised-flight entry point. See
// attitude_kinematics.hpp's own "CCP-029 ADDENDUM" comment block for the
// full design writeup: the real conceptual framing this whole input_*
// family exists for (a pilot's stick position is not the attitude
// target - it is what the target is shaped TOWARD), the real frame-
// conversion rationale, the real roll/pitch-vs-yaw argument-shape
// asymmetry, the get_roll_trim_rad() multirotor simplification, and the
// architectural decision behind the AttitudeTargetState/
// EulerAngleRateShapingGains structs these tests construct directly.
//
// "The entry point is stateful, so one call proves almost nothing" -
// copter-rust's own COP-007 finding, reused directly. The shaped
// (rate_bf_ff_enabled == true) branch is therefore tested below with a
// real, 300-iteration scripted sequence at 400 Hz (matching this port's
// own established dt convention elsewhere in this file), not a single
// call - a step in roll, a ramp in pitch, and a yaw rate that reverses
// sign partway through, copter-rust's own real script shape, reused
// directly. The unshaped (rate_bf_ff_enabled == false) branch, by
// contrast, IS legitimately tested with a single call: re-verified
// directly against attitude_kinematics.hpp's own real structure above,
// that branch's own output is a direct, one-step function of its own
// inputs, with no iterative shaping involved.
// =======================================================================

namespace {

// A representative set of shaping gains for these tests. input_tc and
// rate_y_tc are DELIBERATELY UNEQUAL (matching COP-007's own established
// "rate_rp_tc 0.15, rate_y_tc 0.25" convention for the analogous rate-
// entry-point gains, reused here for the identical reason: if the two
// were equal, a port that used the wrong one everywhere would be
// numerically indistinguishable from a correct one).
struct EntryPointGains {
    EulerAngleRateShapingGains gains() const {
        EulerAngleRateShapingGains g;
        g.rate_bf_ff_enabled = true;
        g.input_tc = input_tc;
        g.rate_y_tc = rate_y_tc;
        g.rate_rp_tc = rate_rp_tc;
        g.ang_vel_roll_max_degs = ang_vel_roll_max_degs;
        g.ang_vel_pitch_max_degs = ang_vel_pitch_max_degs;
        g.ang_vel_yaw_max_degs = ang_vel_yaw_max_degs;
        g.rate_wp_yaw_max_degs = rate_wp_yaw_max_degs;
        g.accel_roll_max_radss = accel_roll_max_radss;
        g.accel_pitch_max_radss = accel_pitch_max_radss;
        g.accel_yaw_max_radss = accel_yaw_max_radss;
        g.rate_yaw_kp = rate_yaw_kp;
        g.angle_yaw_kp = angle_yaw_kp;
        g.angle_kp_roll = angle_kp_roll;
        g.angle_kp_pitch = angle_kp_pitch;
        g.angle_kp_yaw = angle_kp_yaw;
        g.angle_p_scale = angle_p_scale;
        g.use_sqrt_controller = use_sqrt_controller;
        return g;
    }

    float input_tc = 0.15f;
    float rate_y_tc = 0.2f;
    float rate_rp_tc = 0.15f;
    float ang_vel_roll_max_degs = 220.0f;
    float ang_vel_pitch_max_degs = 220.0f;
    float ang_vel_yaw_max_degs = 200.0f;
    float rate_wp_yaw_max_degs = 45.0f;
    float accel_roll_max_radss = fwcpp::math::radians(400.0f);
    float accel_pitch_max_radss = fwcpp::math::radians(400.0f);
    float accel_yaw_max_radss = fwcpp::math::radians(200.0f);
    float rate_yaw_kp = 2.0f;
    float angle_yaw_kp = 1.0f;
    float angle_kp_roll = 6.0f;
    float angle_kp_pitch = 6.0f;
    float angle_kp_yaw = 4.0f;
    Vector3f angle_p_scale{1.0f, 1.0f, 1.0f};
    bool use_sqrt_controller = false;
};

AttitudeTargetState fresh_state() {
    AttitudeTargetState s;
    s.attitude_target = attitude(0.0f, 0.0f, 0.0f);
    s.euler_angle_target_rad = Vector3f{0.0f, 0.0f, 0.0f};
    s.euler_rate_target_rads = Vector3f{0.0f, 0.0f, 0.0f};
    s.ang_vel_target_rads = Vector3f{0.0f, 0.0f, 0.0f};
    s.ang_accel_target_rads = Vector3f{0.0f, 0.0f, 0.0f};
    return s;
}

// One step of the shaped entry point against a fixed, level body and
// zero gyro - these tests are about the TARGET's own shaping dynamics,
// not about an aircraft actually tracking it, so attitude_body/gyro are
// held constant throughout every sequence below.
void step(float roll_rad, float pitch_rad, float yaw_rate_rads, AttitudeTargetState& state,
          const EulerAngleRateShapingGains& gains, float dt) {
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
    Quaternion attitude_ang_error;
    Vector3f ang_vel_body_rads;
    input_euler_angle_roll_pitch_euler_rate_yaw_rad(roll_rad, pitch_rad, yaw_rate_rads, state, body, gyro, gains, dt,
                                                     thrust_angle, thrust_error_angle, feedforward_scalar,
                                                     attitude_ang_error, ang_vel_body_rads);
}

} // namespace

// (1) THE REAL, MULTI-STEP SCRIPTED TEST for the shaped branch - 600
// iterations at 400 Hz (1.5 s). A step in roll (settling), a ramp in
// pitch (tracking), and a yaw rate that reverses sign partway through
// (turning around) - copter-rust's own real script shape, reused
// directly. Asserts convergence/tracking at multiple stages, not just
// the final value.
//
// Checkpoint margins below are set from measured behavior of this jerk-
// limited shaper at these gains (probed directly before writing this
// test, matching this file's own established "tolerance set from the
// measured value" discipline elsewhere) rather than chosen defensively:
// the roll step takes on the order of a second to fully settle at these
// accel/tc values, and a ramped pitch input settles into a STEADY
// tracking lag (measured ~0.15 rad here) rather than negligible
// near-zero lag - both real, measured properties of the shaper, not
// arbitrary constants.
TEST_CASE("input_euler_angle_roll_pitch_euler_rate_yaw_rad: a scripted stick sequence settles, tracks and turns "
          "around",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_euler_rate_yaw][sequence]") {
    const EntryPointGains eg;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f; // 400 Hz
    const int kSteps = 600;   // 1.5 s

    const float roll_step_rad = fwcpp::math::radians(15.0f);
    const float pitch_ramp_rads = 0.5f; // rad/s, linear ramp
    const float yaw_rate_rads = 0.4f;   // reverses sign at the midpoint
    const int kReversalStep = kSteps / 2;

    AttitudeTargetState state = fresh_state();

    float roll_at_step10 = 0.0f;
    float pitch_at_step300 = 0.0f, pitch_at_step450 = 0.0f, pitch_at_step600 = 0.0f;
    float roll_at_step600 = 0.0f;
    float yaw_rate_at_step10 = 0.0f, yaw_rate_at_step300 = 0.0f;
    float yaw_rate_at_step350 = 0.0f, yaw_rate_at_step600 = 0.0f;

    for (int i = 1; i <= kSteps; ++i) {
        const float t = static_cast<float>(i) * dt;
        const float pitch_cmd = pitch_ramp_rads * t;
        const float yaw_rate_cmd = (i <= kReversalStep) ? yaw_rate_rads : -yaw_rate_rads;

        step(roll_step_rad, pitch_cmd, yaw_rate_cmd, state, gains, dt);

        if (i == 10) {
            roll_at_step10 = state.euler_angle_target_rad.x;
            yaw_rate_at_step10 = state.euler_rate_target_rads.z;
        }
        if (i == 300) { // last step of the +yaw_rate_rads half
            pitch_at_step300 = state.euler_angle_target_rad.y;
            yaw_rate_at_step300 = state.euler_rate_target_rads.z;
        }
        if (i == 350) { // 50 steps (125 ms) past the reversal
            yaw_rate_at_step350 = state.euler_rate_target_rads.z;
        }
        if (i == 450) {
            pitch_at_step450 = state.euler_angle_target_rad.y;
        }
        if (i == 600) {
            roll_at_step600 = state.euler_angle_target_rad.x;
            pitch_at_step600 = state.euler_angle_target_rad.y;
            yaw_rate_at_step600 = state.euler_rate_target_rads.z;
        }
    }

    // ROLL SETTLES: at step 10 (25 ms in) the target has moved toward
    // the step but has genuinely not arrived yet - proving real shaping
    // motion, not an instant snap. By step 600 (1.5 s) it has settled
    // close to the commanded step (measured residual ~2e-4 rad; margin
    // set generously above that).
    REQUIRE(roll_at_step10 > 0.0f);
    REQUIRE(roll_at_step10 < roll_step_rad * 0.9f);
    REQUIRE(roll_at_step600 == Approx(roll_step_rad).margin(0.01f));

    // PITCH TRACKS: sampled at three points well past the initial
    // transient, the target trails the ramp's own commanded value by a
    // measured, roughly constant lag (~0.15 rad at these gains) rather
    // than diverging or catching up exactly - genuine continuous
    // tracking, not merely eventual convergence at the very end.
    const float pitch_cmd_at_300 = pitch_ramp_rads * (300.0f * dt);
    const float pitch_cmd_at_450 = pitch_ramp_rads * (450.0f * dt);
    const float pitch_cmd_at_600 = pitch_ramp_rads * (600.0f * dt);
    const float measured_lag_rad = 0.15f;
    REQUIRE((pitch_cmd_at_300 - pitch_at_step300) == Approx(measured_lag_rad).margin(0.05f));
    REQUIRE((pitch_cmd_at_450 - pitch_at_step450) == Approx(measured_lag_rad).margin(0.05f));
    REQUIRE((pitch_cmd_at_600 - pitch_at_step600) == Approx(measured_lag_rad).margin(0.05f));
    // The ramp is genuinely still climbing throughout - not a case where
    // pitch happened to already saturate at some limit.
    REQUIRE(pitch_at_step450 > pitch_at_step300);
    REQUIRE(pitch_at_step600 > pitch_at_step450);

    // YAW TURNS AROUND: settles toward +yaw_rate_rads by the end of the
    // first half, then genuinely turns around and settles toward
    // -yaw_rate_rads by the end of the second half - sampled once very
    // early (partial progress, proving real shaping rather than an
    // instant snap), once settled just before the reversal, once
    // shortly after (genuinely decreasing), and once settled at the end.
    REQUIRE(yaw_rate_at_step10 > 0.0f);
    REQUIRE(yaw_rate_at_step10 < yaw_rate_rads * 0.5f);
    REQUIRE(yaw_rate_at_step300 == Approx(yaw_rate_rads).margin(0.05f));
    REQUIRE(yaw_rate_at_step350 < yaw_rate_at_step300 - 0.05f); // genuinely turning around right after the reversal
    REQUIRE(yaw_rate_at_step600 == Approx(-yaw_rate_rads).margin(0.05f));
}

// (2) The unshaped (rate_bf_ff_enabled == false) branch - a single call
// IS legitimate here, re-verified directly: this branch's own output is
// a direct, one-step function of its own inputs, unlike the shaped
// branch above. Split into two calls: (2a) proves the direct-set/
// integrate-from-old-value/rebuild behavior with a clean, uncorrupted
// starting state, and confirms step 5 still runs unconditionally; (2b)
// proves every feedforward field is genuinely ZEROED, seeded with
// obvious sentinels first.
TEST_CASE("input_euler_angle_roll_pitch_euler_rate_yaw_rad: rate_bf_ff_enabled == false sets targets directly and "
          "zeros every feedforward",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_euler_rate_yaw]") {
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    EntryPointGains eg;
    EulerAngleRateShapingGains gains = eg.gains();
    gains.rate_bf_ff_enabled = false;

    // (2a) A clean starting state with a pre-existing yaw of 0.1 rad,
    // encoded in attitude_target itself (NOT written directly into
    // euler_angle_target_rad, since step 2 of this function
    // unconditionally recomputes euler_angle_target_rad FROM
    // attitude_target via to_euler before the branch ever runs - a
    // direct write there would simply be overwritten and prove
    // nothing). ang_vel_target_rads is left at zero so step 1
    // (update_attitude_target, which also runs unconditionally) is a
    // no-op and does not perturb this starting attitude.
    {
        AttitudeTargetState state = fresh_state();
        state.attitude_target = attitude(0.0f, 0.0f, 0.1f);

        const float roll_cmd = 0.3f;
        const float pitch_cmd = -0.2f;
        const float yaw_rate_cmd = 0.5f;
        const float dt = 0.01f;

        float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
        Quaternion attitude_ang_error;
        Vector3f ang_vel_body_rads;
        input_euler_angle_roll_pitch_euler_rate_yaw_rad(roll_cmd, pitch_cmd, yaw_rate_cmd, state, body, gyro, gains,
                                                          dt, thrust_angle, thrust_error_angle, feedforward_scalar,
                                                          attitude_ang_error, ang_vel_body_rads);

        // Roll/pitch targets set DIRECTLY from the input angles.
        REQUIRE(state.euler_angle_target_rad.x == roll_cmd);
        REQUIRE(state.euler_angle_target_rad.y == pitch_cmd);
        // Yaw target plain-Euler-INTEGRATED from its own prior value
        // (0.1, carried via attitude_target/to_euler at step 2) - NOT
        // shaped, NOT reset to the input.
        REQUIRE(state.euler_angle_target_rad.z == Approx(0.1f + yaw_rate_cmd * dt).margin(1e-5f));

        // attitude_target rebuilt fresh via from_euler from the result.
        Vector3f rebuilt_euler;
        state.attitude_target.to_euler(rebuilt_euler);
        REQUIRE(rebuilt_euler.x == Approx(roll_cmd).margin(1e-4f));
        REQUIRE(rebuilt_euler.y == Approx(pitch_cmd).margin(1e-4f));
        REQUIRE(rebuilt_euler.z == Approx(0.1f + yaw_rate_cmd * dt).margin(1e-4f));

        // Step 5 still ran UNCONDITIONALLY even on this branch:
        // reproducing attitude_controller_run_quat directly against the
        // resulting (post-branch) state must reproduce the same
        // ang_vel_body_rads this call already produced - proving the
        // final step is not skipped.
        Quaternion reference_target = state.attitude_target;
        float ref_thrust_angle = 0.0f, ref_thrust_error_angle = 0.0f, ref_feedforward_scalar = 0.0f;
        Quaternion ref_attitude_ang_error;
        Vector3f ref_ang_vel_body_rads;
        attitude_controller_run_quat(reference_target, body, state.ang_vel_target_rads, gyro, gains.rate_yaw_kp,
                                      gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch,
                                      gains.angle_kp_yaw, gains.angle_p_scale, gains.accel_roll_max_radss,
                                      gains.accel_pitch_max_radss, gains.accel_yaw_max_radss,
                                      gains.use_sqrt_controller, gains.ang_vel_roll_max_degs,
                                      gains.ang_vel_pitch_max_degs, gains.ang_vel_yaw_max_degs, dt, ref_thrust_angle,
                                      ref_thrust_error_angle, ref_feedforward_scalar, ref_attitude_ang_error,
                                      ref_ang_vel_body_rads);
        REQUIRE(ang_vel_body_rads.x == Approx(ref_ang_vel_body_rads.x).margin(1e-6f));
        REQUIRE(ang_vel_body_rads.y == Approx(ref_ang_vel_body_rads.y).margin(1e-6f));
        REQUIRE(ang_vel_body_rads.z == Approx(ref_ang_vel_body_rads.z).margin(1e-6f));
    }

    // (2b) Every feedforward field seeded with an obviously-nonzero
    // sentinel, so a genuine zero afterward proves the branch actually
    // zeroed them rather than them starting at zero already. (The exact
    // resulting angle/attitude values are not asserted here - (2a)
    // above already covers that precisely; this call is only about the
    // zeroing guarantee.)
    {
        AttitudeTargetState state = fresh_state();
        state.euler_rate_target_rads = Vector3f{1.0f, -1.0f, 1.0f};
        state.ang_vel_target_rads = Vector3f{2.0f, -2.0f, 2.0f};
        state.ang_accel_target_rads = Vector3f{3.0f, -3.0f, 3.0f};

        float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
        Quaternion attitude_ang_error;
        Vector3f ang_vel_body_rads;
        input_euler_angle_roll_pitch_euler_rate_yaw_rad(0.1f, -0.1f, 0.2f, state, body, gyro, gains, 0.01f,
                                                          thrust_angle, thrust_error_angle, feedforward_scalar,
                                                          attitude_ang_error, ang_vel_body_rads);

        REQUIRE(state.euler_rate_target_rads.x == 0.0f);
        REQUIRE(state.euler_rate_target_rads.y == 0.0f);
        REQUIRE(state.euler_rate_target_rads.z == 0.0f);
        REQUIRE(state.ang_vel_target_rads.x == 0.0f);
        REQUIRE(state.ang_vel_target_rads.y == 0.0f);
        REQUIRE(state.ang_vel_target_rads.z == 0.0f);
        REQUIRE(state.ang_accel_target_rads.x == 0.0f);
        REQUIRE(state.ang_accel_target_rads.y == 0.0f);
        REQUIRE(state.ang_accel_target_rads.z == 0.0f);
    }
}

// (3) THE REAL ROLL/PITCH-VS-YAW ARGUMENT-SHAPE ASYMMETRY - a case that
// would produce a DIFFERENT result under the "obvious" (but wrong)
// assumption that all three attitude_command_model calls share one
// argument shape. Drives the REAL entry point for 150 iterations with a
// constant commanded yaw RATE (roll/pitch held at zero), and
// independently reconstructs, over the SAME 150 iterations, both the
// CORRECT per-call yaw shape (0.0 error_angle, the real input rate as
// desired_ang_vel) and the NAIVE (wrong) shape that would result from
// assuming yaw is shaped exactly like roll/pitch (the commanded rate
// treated as an angle error against a fixed zero target, 0.0
// desired_ang_vel) - using attitude_command_model directly, already
// independently verified by its own dedicated tests above.
//
// A SINGLE call is not enough to catch this: probed directly before
// writing this test, the two shapes are numerically IDENTICAL on the
// first call from a zero state (both are, at that instant, simply
// "ramp at max jerk toward a distant target" with no dependence yet on
// which of the two channels carries the target). The divergence is real
// but only becomes large after enough iterations that the naive
// (angle-hold) shape's own steady-state behavior differs from the
// correct (rate-hold) shape's - confirmed directly by probing before
// choosing 150 iterations here, comfortably past that point.
TEST_CASE("input_euler_angle_roll_pitch_euler_rate_yaw_rad: yaw is shaped as a rate command, not an angle command "
          "like roll/pitch",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_euler_rate_yaw][asymmetry]") {
    const EntryPointGains eg;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f;
    const int kSteps = 150;
    const float yaw_rate_cmd = 0.3f; // a RATE, not an angle

    // The actual function under test.
    AttitudeTargetState state = fresh_state();
    for (int i = 0; i < kSteps; ++i) {
        step(0.0f, 0.0f, yaw_rate_cmd, state, gains, dt);
    }

    // Independently reconstructed CORRECT yaw shape, iterated the same
    // number of times: error_angle == 0.0, desired_ang_vel ==
    // yaw_rate_cmd, using rate_y_tc - matching what the real function's
    // own third attitude_command_model call does every iteration (at
    // level attitude the Euler rate/accel limits it reads reduce
    // exactly to the plain yaw max, confirmed directly, so this
    // reconstruction uses the same limit values).
    float correct_rate = 0.0f, correct_accel = 0.0f;
    for (int i = 0; i < kSteps; ++i) {
        attitude_command_model(0.0f, yaw_rate_cmd, correct_rate, correct_accel,
                                std::fabs(fwcpp::math::radians(gains.ang_vel_yaw_max_degs)), gains.accel_yaw_max_radss,
                                gains.rate_y_tc, dt);
    }

    // Independently reconstructed NAIVE (wrong) yaw shape: treating the
    // commanded rate as if it were a fixed angle error against a zero
    // target, with 0.0 desired_ang_vel - the "all three calls share
    // roll/pitch's own shape" assumption.
    float naive_rate = 0.0f, naive_accel = 0.0f;
    for (int i = 0; i < kSteps; ++i) {
        attitude_command_model(fwcpp::math::wrap_PI(yaw_rate_cmd - 0.0f), 0.0f, naive_rate, naive_accel,
                                std::fabs(fwcpp::math::radians(gains.ang_vel_yaw_max_degs)), gains.accel_yaw_max_radss,
                                gains.rate_y_tc, dt);
    }

    // The two must genuinely differ, by a large margin, for this test
    // to prove anything (measured difference at 150 iterations is well
    // over 0.3 rad/s).
    REQUIRE(std::fabs(correct_rate - naive_rate) > 0.2f);

    // The actual function matches the correct (rate) shape...
    REQUIRE(state.euler_rate_target_rads.z == Approx(correct_rate).margin(1e-4f));
    // ...and genuinely differs from the naive (angle) shape by the same
    // large margin.
    REQUIRE(std::fabs(state.euler_rate_target_rads.z - naive_rate) > 0.2f);

    // Sanity: roll genuinely used the angle-error shape too (a nonzero
    // commanded roll angle here would produce a nonzero target rate),
    // proving roll/pitch's own shape is exercised, not merely yaw's -
    // re-run with a nonzero roll to confirm.
    AttitudeTargetState roll_state = fresh_state();
    step(0.2f, 0.0f, 0.0f, roll_state, gains, dt);
    REQUIRE(roll_state.euler_rate_target_rads.x > 0.0f);
}

// (4) THE REAL SEPARATE rate_y_tc VS input_tc DISTINCTION - two
// otherwise-identical short sequences, differing only in rate_y_tc,
// produce a measurably different yaw response timing. Reuses CCP-022's
// own already-verified property ("a smaller input_tc/time-constant
// produces a sharper, faster-converging response") in the yaw-specific
// time constant specifically, proving rate_y_tc is genuinely read and
// genuinely separate from input_tc (which is held IDENTICAL, and
// nonzero, in both variants below - a port that accidentally shaped yaw
// with input_tc instead of rate_y_tc would produce IDENTICAL results in
// both variants, since input_tc never changes between them). Margin
// below (0.08) is set from a measured difference of ~0.13 rad/s at
// these gains and step count.
TEST_CASE("input_euler_angle_roll_pitch_euler_rate_yaw_rad: rate_y_tc, not input_tc, sets the yaw response's own "
          "timing",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_euler_rate_yaw][rate_y_tc]") {
    EntryPointGains eg_sharp;
    eg_sharp.input_tc = 0.3f;   // held identical in both variants
    eg_sharp.rate_y_tc = 0.02f; // much sharper than input_tc

    EntryPointGains eg_slow;
    eg_slow.input_tc = 0.3f;  // identical to eg_sharp's own input_tc
    eg_slow.rate_y_tc = 0.3f; // deliberately equal to input_tc - "as if yaw used input_tc"

    const EulerAngleRateShapingGains gains_sharp = eg_sharp.gains();
    const EulerAngleRateShapingGains gains_slow = eg_slow.gains();
    const float dt = 0.0025f;
    const float yaw_rate_cmd = 1.0f;
    const int kSteps = 20; // early in the transient, where the difference is largest

    AttitudeTargetState state_sharp = fresh_state();
    AttitudeTargetState state_slow = fresh_state();

    for (int i = 0; i < kSteps; ++i) {
        step(0.0f, 0.0f, yaw_rate_cmd, state_sharp, gains_sharp, dt);
        step(0.0f, 0.0f, yaw_rate_cmd, state_slow, gains_slow, dt);
    }

    // Both are still short of the full commanded rate (neither has
    // fully settled yet - otherwise the comparison below would be
    // vacuous, both pinned at the same ceiling).
    REQUIRE(state_sharp.euler_rate_target_rads.z < yaw_rate_cmd);
    REQUIRE(state_slow.euler_rate_target_rads.z < yaw_rate_cmd);

    // The sharper rate_y_tc has converged measurably further toward the
    // commanded rate than the slow one, with input_tc held identical
    // between the two - proving rate_y_tc, not input_tc, is what
    // actually governs yaw's own timing here.
    REQUIRE(state_sharp.euler_rate_target_rads.z > state_slow.euler_rate_target_rads.z + 0.08f);
}

// (5) The trivial _cd wrapper: converts all three inputs via cd_to_rad
// and otherwise behaves identically to a direct _rad call with the
// already-converted values.
TEST_CASE("input_euler_angle_roll_pitch_euler_rate_yaw_cd: converts centidegrees and forwards to the _rad entry "
          "point",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_euler_rate_yaw]") {
    const EntryPointGains eg;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f;
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};

    const float roll_cd = 500.0f;      // 5 degrees
    const float pitch_cd = -300.0f;    // -3 degrees
    const float yaw_rate_cds = 200.0f; // 2 deg/s

    AttitudeTargetState state_cd = fresh_state();
    float thrust_angle_cd = 0.0f, thrust_error_angle_cd = 0.0f, feedforward_scalar_cd = 0.0f;
    Quaternion attitude_ang_error_cd;
    Vector3f ang_vel_body_rads_cd;
    input_euler_angle_roll_pitch_euler_rate_yaw_cd(roll_cd, pitch_cd, yaw_rate_cds, state_cd, body, gyro, gains, dt,
                                                    thrust_angle_cd, thrust_error_angle_cd, feedforward_scalar_cd,
                                                    attitude_ang_error_cd, ang_vel_body_rads_cd);

    AttitudeTargetState state_rad = fresh_state();
    float thrust_angle_rad = 0.0f, thrust_error_angle_rad = 0.0f, feedforward_scalar_rad = 0.0f;
    Quaternion attitude_ang_error_rad;
    Vector3f ang_vel_body_rads_rad;
    input_euler_angle_roll_pitch_euler_rate_yaw_rad(fwcpp::math::cd_to_rad(roll_cd), fwcpp::math::cd_to_rad(pitch_cd),
                                                     fwcpp::math::cd_to_rad(yaw_rate_cds), state_rad, body, gyro,
                                                     gains, dt, thrust_angle_rad, thrust_error_angle_rad,
                                                     feedforward_scalar_rad, attitude_ang_error_rad,
                                                     ang_vel_body_rads_rad);

    REQUIRE(state_cd.euler_angle_target_rad.x == state_rad.euler_angle_target_rad.x);
    REQUIRE(state_cd.euler_angle_target_rad.y == state_rad.euler_angle_target_rad.y);
    REQUIRE(state_cd.euler_angle_target_rad.z == state_rad.euler_angle_target_rad.z);
    REQUIRE(ang_vel_body_rads_cd.x == ang_vel_body_rads_rad.x);
    REQUIRE(ang_vel_body_rads_cd.y == ang_vel_body_rads_rad.y);
    REQUIRE(ang_vel_body_rads_cd.z == ang_vel_body_rads_rad.z);
}


// =======================================================================
// CCP-030: get_slew_yaw_max_rads, input_euler_angle_roll_pitch_yaw_rad (+ _cd)
// =======================================================================

void step_yaw_angle(float roll_rad, float pitch_rad, float yaw_rad, bool slew_yaw, AttitudeTargetState& state,
                    const EulerAngleRateShapingGains& gains, float dt) {
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
    Quaternion attitude_ang_error;
    Vector3f ang_vel_body_rads;
    input_euler_angle_roll_pitch_yaw_rad(roll_rad, pitch_rad, yaw_rad, slew_yaw, state, body, gyro, gains, dt,
                                          thrust_angle, thrust_error_angle, feedforward_scalar, attitude_ang_error,
                                          ang_vel_body_rads);
}

AttitudeTargetState state_with_roll_offset(float roll_rad) {
    AttitudeTargetState s = fresh_state();
    s.euler_angle_target_rad.x = roll_rad;
    s.attitude_target.from_euler(s.euler_angle_target_rad);
    return s;
}

TEST_CASE("get_slew_yaw_max_rads: non-positive ang_vel_yaw_max uses wp yaw rate only",
          "[control][attitude_kinematics][get_slew_yaw_max_rads][CCP-030]") {
    REQUIRE(get_slew_yaw_max_rads(0.0f, 720.0f) == Approx(fwcpp::math::radians(720.0f)).margin(1e-6f));
    REQUIRE(get_slew_yaw_max_rads(-10.0f, 45.0f) == Approx(fwcpp::math::radians(45.0f)).margin(1e-6f));
}

TEST_CASE("get_slew_yaw_max_rads: positive ang_vel_yaw_max returns min with wp yaw rate",
          "[control][attitude_kinematics][get_slew_yaw_max_rads][CCP-030]") {
    const float slow = get_slew_yaw_max_rads(200.0f, 45.0f);
    REQUIRE(slow == Approx(fwcpp::math::radians(45.0f)).margin(1e-6f));
    const float fast = get_slew_yaw_max_rads(360.0f, 720.0f);
    REQUIRE(fast == Approx(fwcpp::math::radians(360.0f)).margin(1e-6f));
}

TEST_CASE("input_euler_angle_roll_pitch_yaw_rad: yaw uses angle shaping with input_tc, not CCP-029 rate-y_tc asymmetry",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_yaw][CCP-030][symmetry]") {
    EntryPointGains eg;
    EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f;
    const int kSteps = 120;
    const float yaw_angle_cmd = 0.35f;

    AttitudeTargetState state = fresh_state();
    for (int i = 0; i < kSteps; ++i) {
        step_yaw_angle(0.0f, 0.0f, yaw_angle_cmd, false, state, gains, dt);
    }

    float correct_rate = 0.0f, correct_accel = 0.0f;
    float target_z = 0.0f;
    for (int i = 0; i < kSteps; ++i) {
        attitude_command_model(fwcpp::math::wrap_PI(yaw_angle_cmd - target_z), 0.0f, correct_rate, correct_accel,
                                std::fabs(fwcpp::math::radians(gains.ang_vel_yaw_max_degs)), gains.accel_yaw_max_radss,
                                gains.input_tc, dt);
        target_z += correct_rate * dt;
    }

    float wrong_rate = 0.0f, wrong_accel = 0.0f;
    for (int i = 0; i < kSteps; ++i) {
        attitude_command_model(0.0f, yaw_angle_cmd, wrong_rate, wrong_accel,
                                std::fabs(fwcpp::math::radians(gains.ang_vel_yaw_max_degs)), gains.accel_yaw_max_radss,
                                gains.rate_y_tc, dt);
    }

    REQUIRE(std::fabs(correct_rate - wrong_rate) > 0.15f);
    REQUIRE(state.euler_rate_target_rads.z == Approx(correct_rate).margin(1e-4f));
}

TEST_CASE("input_euler_angle_roll_pitch_yaw_rad: slew_yaw selects slower yaw rate limit in shaped branch",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_yaw][CCP-030][slew_yaw]") {
    EntryPointGains eg;
    eg.ang_vel_yaw_max_degs = 360.0f;
    eg.rate_wp_yaw_max_degs = 30.0f;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f;
    const float yaw_cmd = 0.5f;

    AttitudeTargetState fast_state = fresh_state();
    AttitudeTargetState slow_state = fresh_state();
    for (int i = 0; i < 80; ++i) {
        step_yaw_angle(0.0f, 0.0f, yaw_cmd, false, fast_state, gains, dt);
        step_yaw_angle(0.0f, 0.0f, yaw_cmd, true, slow_state, gains, dt);
    }

    REQUIRE(std::fabs(slow_state.euler_rate_target_rads.z) < std::fabs(fast_state.euler_rate_target_rads.z));
}

TEST_CASE("input_euler_angle_roll_pitch_yaw_rad: unshaped branch slews yaw target by yaw_rate_max*dt",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_yaw][CCP-030][unshaped]") {
    EntryPointGains eg;
    EulerAngleRateShapingGains gains = eg.gains();
    gains.rate_bf_ff_enabled = false;
    gains.ang_vel_yaw_max_degs = 100.0f;
    const float dt = 0.05f;
    const float yaw_rate_max = fwcpp::math::radians(gains.ang_vel_yaw_max_degs);
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};

    AttitudeTargetState clamped = fresh_state();
    float t1 = 0.0f, te1 = 0.0f, ff1 = 0.0f;
    Quaternion err1;
    Vector3f av1;
    input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, 1.0f, false, clamped, body, gyro, gains, dt, t1, te1, ff1, err1,
                                          av1);
    REQUIRE(clamped.euler_angle_target_rad.z == Approx(yaw_rate_max * dt).margin(1e-5f));

    AttitudeTargetState passthrough = fresh_state();
    float t2 = 0.0f, te2 = 0.0f, ff2 = 0.0f;
    Quaternion err2;
    Vector3f av2;
    input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, 0.02f, false, passthrough, body, gyro, gains, dt, t2, te2, ff2,
                                          err2, av2);
    REQUIRE(passthrough.euler_angle_target_rad.z == Approx(0.02f).margin(1e-5f));

    gains.ang_vel_yaw_max_degs = 0.0f;
    AttitudeTargetState direct = fresh_state();
    float t3 = 0.0f, te3 = 0.0f, ff3 = 0.0f;
    Quaternion err3;
    Vector3f av3;
    input_euler_angle_roll_pitch_yaw_rad(0.0f, 0.0f, 0.75f, false, direct, body, gyro, gains, dt, t3, te3, ff3, err3,
                                          av3);
    REQUIRE(direct.euler_angle_target_rad.z == Approx(0.75f).margin(1e-5f));
}

TEST_CASE("input_euler_angle_roll_pitch_yaw_rad: scripted heading sequence stays clear of acos cliff",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_yaw][CCP-030][sequence]") {
    // Start ~0.15 rad off level (>> 3.2e-4 acos cliff documented in COP-007).
    EntryPointGains eg;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f;
    AttitudeTargetState state = state_with_roll_offset(0.15f);

    for (int i = 0; i < 300; ++i) {
        const float roll = (i < 100) ? 0.20f : 0.10f;
        const float pitch = (i < 200) ? -0.12f : 0.05f;
        const float yaw = (i < 150) ? 0.40f : -0.25f;
        step_yaw_angle(roll, pitch, yaw, true, state, gains, dt);
    }

    REQUIRE(std::fabs(state.euler_angle_target_rad.x) > 0.05f);
    REQUIRE(std::fabs(state.euler_rate_target_rads.z) < fwcpp::math::radians(eg.ang_vel_yaw_max_degs));
}

TEST_CASE("input_euler_angle_roll_pitch_yaw_cd: converts centidegrees and forwards",
          "[control][attitude_kinematics][input_euler_angle_roll_pitch_yaw][CCP-030]") {
    EntryPointGains eg;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f;
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    const float roll_cd = 800.0f;
    const float pitch_cd = -400.0f;
    const float yaw_cd = 1200.0f;

    AttitudeTargetState state_cd = fresh_state();
    float t_cd = 0.0f, te_cd = 0.0f, ff_cd = 0.0f;
    Quaternion err_cd;
    Vector3f av_cd;
    input_euler_angle_roll_pitch_yaw_cd(roll_cd, pitch_cd, yaw_cd, false, state_cd, body, gyro, gains, dt, t_cd,
                                         te_cd, ff_cd, err_cd, av_cd);

    AttitudeTargetState state_rad = fresh_state();
    float t_rad = 0.0f, te_rad = 0.0f, ff_rad = 0.0f;
    Quaternion err_rad;
    Vector3f av_rad;
    input_euler_angle_roll_pitch_yaw_rad(fwcpp::math::cd_to_rad(roll_cd), fwcpp::math::cd_to_rad(pitch_cd),
                                          fwcpp::math::cd_to_rad(yaw_cd), false, state_rad, body, gyro, gains, dt,
                                          t_rad, te_rad, ff_rad, err_rad, av_rad);

    REQUIRE(state_cd.euler_rate_target_rads.z == Approx(state_rad.euler_rate_target_rads.z).margin(1e-6f));
}


// =======================================================================
// CCP-031: input_euler_rate_roll_pitch_yaw_rads
// Acro-style entry: every axis is a RATE. No _cd wrapper in upstream.
// =======================================================================

void step_euler_rate(float roll_rate_rads, float pitch_rate_rads, float yaw_rate_rads, AttitudeTargetState& state,
                     const EulerAngleRateShapingGains& gains, float dt) {
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
    Quaternion attitude_ang_error;
    Vector3f ang_vel_body_rads;
    input_euler_rate_roll_pitch_yaw_rads(roll_rate_rads, pitch_rate_rads, yaw_rate_rads, state, body, gyro, gains, dt,
                                          thrust_angle, thrust_error_angle, feedforward_scalar, attitude_ang_error,
                                          ang_vel_body_rads);
}

TEST_CASE("input_euler_rate_roll_pitch_yaw_rads: shaped sequence settles, tracks and turns around",
          "[control][attitude_kinematics][input_euler_rate_roll_pitch_yaw][CCP-031][sequence]") {
    EntryPointGains eg;
    eg.rate_rp_tc = 0.15f;
    eg.rate_y_tc = 0.25f;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f; // 400 Hz
    const int kSteps = 600;   // 1.5 s

    const float roll_rate_rads = 0.4f;
    const float pitch_rate_rads = 0.3f;
    const float yaw_rate_rads = 0.5f;
    const int kReversalStep = kSteps / 2;

    AttitudeTargetState state = fresh_state();

    float roll_rate_at_10 = 0.0f, pitch_rate_at_10 = 0.0f, yaw_rate_at_10 = 0.0f;
    float roll_rate_at_300 = 0.0f, pitch_rate_at_300 = 0.0f, yaw_rate_at_300 = 0.0f;
    float roll_rate_at_350 = 0.0f, pitch_rate_at_350 = 0.0f, yaw_rate_at_350 = 0.0f;
    float roll_rate_at_600 = 0.0f, pitch_rate_at_600 = 0.0f, yaw_rate_at_600 = 0.0f;

    for (int i = 1; i <= kSteps; ++i) {
        const float roll_cmd = (i <= kReversalStep) ? roll_rate_rads : -roll_rate_rads;
        const float pitch_cmd = (i <= kReversalStep) ? pitch_rate_rads : -pitch_rate_rads;
        const float yaw_cmd = (i <= kReversalStep) ? yaw_rate_rads : -yaw_rate_rads;

        step_euler_rate(roll_cmd, pitch_cmd, yaw_cmd, state, gains, dt);

        if (i == 10) {
            roll_rate_at_10 = state.euler_rate_target_rads.x;
            pitch_rate_at_10 = state.euler_rate_target_rads.y;
            yaw_rate_at_10 = state.euler_rate_target_rads.z;
        }
        if (i == 300) {
            roll_rate_at_300 = state.euler_rate_target_rads.x;
            pitch_rate_at_300 = state.euler_rate_target_rads.y;
            yaw_rate_at_300 = state.euler_rate_target_rads.z;
        }
        if (i == 350) {
            roll_rate_at_350 = state.euler_rate_target_rads.x;
            pitch_rate_at_350 = state.euler_rate_target_rads.y;
            yaw_rate_at_350 = state.euler_rate_target_rads.z;
        }
        if (i == 600) {
            roll_rate_at_600 = state.euler_rate_target_rads.x;
            pitch_rate_at_600 = state.euler_rate_target_rads.y;
            yaw_rate_at_600 = state.euler_rate_target_rads.z;
        }
    }

    // Early: motion has started but has not snapped to the command.
    REQUIRE(roll_rate_at_10 > 0.0f);
    REQUIRE(roll_rate_at_10 < roll_rate_rads * 0.5f);
    REQUIRE(pitch_rate_at_10 > 0.0f);
    REQUIRE(pitch_rate_at_10 < pitch_rate_rads * 0.5f);
    REQUIRE(yaw_rate_at_10 > 0.0f);
    REQUIRE(yaw_rate_at_10 < yaw_rate_rads * 0.5f);

    // Mid: all three rate targets have settled toward the command.
    REQUIRE(roll_rate_at_300 == Approx(roll_rate_rads).margin(0.05f));
    REQUIRE(pitch_rate_at_300 == Approx(pitch_rate_rads).margin(0.05f));
    REQUIRE(yaw_rate_at_300 == Approx(yaw_rate_rads).margin(0.05f));

    // Shortly after reversal: all three axes are genuinely turning around.
    REQUIRE(roll_rate_at_350 < roll_rate_at_300 - 0.05f);
    REQUIRE(pitch_rate_at_350 < pitch_rate_at_300 - 0.05f);
    REQUIRE(yaw_rate_at_350 < yaw_rate_at_300 - 0.05f);

    // End: reversed rates settled. Yaw uses the slower rate_y_tc (0.25)
    // and a smaller accel limit, so it is given a slightly wider settle
    // band than roll/pitch.
    REQUIRE(roll_rate_at_600 == Approx(-roll_rate_rads).margin(0.05f));
    REQUIRE(pitch_rate_at_600 == Approx(-pitch_rate_rads).margin(0.05f));
    REQUIRE(yaw_rate_at_600 == Approx(-yaw_rate_rads).margin(0.10f));
}

TEST_CASE("input_euler_rate_roll_pitch_yaw_rads: unshaped wrap_2PI yaw, wrap_PI roll, 85deg pitch clamp",
          "[control][attitude_kinematics][input_euler_rate_roll_pitch_yaw][CCP-031][unshaped]") {
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    EntryPointGains eg;
    EulerAngleRateShapingGains gains = eg.gains();
    gains.rate_bf_ff_enabled = false;

    // Yaw: start at 3.0 rad (still in to_euler's [-pi, pi] range) and
    // step +0.5 so the result is 3.5, which is > pi. wrap_2PI keeps 3.5;
    // wrap_PI would fold it to ~-2.78. Pitch/roll held at zero.
    {
        AttitudeTargetState state = fresh_state();
        state.attitude_target = attitude(0.0f, 0.0f, 3.0f);

        float t = 0.0f, te = 0.0f, ff = 0.0f;
        Quaternion err;
        Vector3f av;
        input_euler_rate_roll_pitch_yaw_rads(0.0f, 0.0f, 10.0f, state, body, gyro, gains, 0.05f, t, te, ff, err, av);

        const float expected_yaw = fwcpp::math::wrap_2PI(3.0f + 10.0f * 0.05f);
        REQUIRE(state.euler_angle_target_rad.z == Approx(expected_yaw).margin(1e-5f));
        REQUIRE(expected_yaw == Approx(3.5f).margin(1e-5f));
        REQUIRE(std::fabs(state.euler_angle_target_rad.z - fwcpp::math::wrap_PI(3.5f)) > 1.0f);
    }

    // Pitch clamp: a 2 rad/s step of 1 s would be 2 rad (~114 deg); the
    // unshaped branch clamps to ±radians(85).
    {
        AttitudeTargetState pos = fresh_state();
        AttitudeTargetState neg = fresh_state();
        float t = 0.0f, te = 0.0f, ff = 0.0f;
        Quaternion err;
        Vector3f av;
        input_euler_rate_roll_pitch_yaw_rads(0.0f, 2.0f, 0.0f, pos, body, gyro, gains, 1.0f, t, te, ff, err, av);
        input_euler_rate_roll_pitch_yaw_rads(0.0f, -2.0f, 0.0f, neg, body, gyro, gains, 1.0f, t, te, ff, err, av);

        const float pitch_lim = fwcpp::math::radians(85.0f);
        REQUIRE(pos.euler_angle_target_rad.y == Approx(pitch_lim).margin(1e-5f));
        REQUIRE(neg.euler_angle_target_rad.y == Approx(-pitch_lim).margin(1e-5f));
        REQUIRE(pos.euler_angle_target_rad.y < 2.0f);
    }

    // Roll: wrap_PI, not wrap_2PI. A 4 rad step from 0 is ~-2.28 under
    // wrap_PI and 4.0 under wrap_2PI.
    {
        AttitudeTargetState state = fresh_state();
        float t = 0.0f, te = 0.0f, ff = 0.0f;
        Quaternion err;
        Vector3f av;
        input_euler_rate_roll_pitch_yaw_rads(4.0f, 0.0f, 0.0f, state, body, gyro, gains, 1.0f, t, te, ff, err, av);

        REQUIRE(state.euler_angle_target_rad.x == Approx(fwcpp::math::wrap_PI(4.0f)).margin(1e-5f));
        REQUIRE(std::fabs(state.euler_angle_target_rad.x - fwcpp::math::wrap_2PI(4.0f)) > 1.0f);
    }

    // Feedforward targets are zeroed; run_quat still runs.
    {
        AttitudeTargetState state = fresh_state();
        state.euler_rate_target_rads = Vector3f{1.0f, -1.0f, 1.0f};
        state.ang_vel_target_rads = Vector3f{2.0f, -2.0f, 2.0f};
        state.ang_accel_target_rads = Vector3f{3.0f, -3.0f, 3.0f};

        float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
        Quaternion attitude_ang_error;
        Vector3f ang_vel_body_rads;
        input_euler_rate_roll_pitch_yaw_rads(0.1f, -0.1f, 0.2f, state, body, gyro, gains, 0.01f, thrust_angle,
                                              thrust_error_angle, feedforward_scalar, attitude_ang_error,
                                              ang_vel_body_rads);

        REQUIRE(state.euler_rate_target_rads.x == 0.0f);
        REQUIRE(state.euler_rate_target_rads.y == 0.0f);
        REQUIRE(state.euler_rate_target_rads.z == 0.0f);
        REQUIRE(state.ang_vel_target_rads.x == 0.0f);
        REQUIRE(state.ang_vel_target_rads.y == 0.0f);
        REQUIRE(state.ang_vel_target_rads.z == 0.0f);
        REQUIRE(state.ang_accel_target_rads.x == 0.0f);
        REQUIRE(state.ang_accel_target_rads.y == 0.0f);
        REQUIRE(state.ang_accel_target_rads.z == 0.0f);

        Quaternion reference_target = state.attitude_target;
        float ref_thrust_angle = 0.0f, ref_thrust_error_angle = 0.0f, ref_feedforward_scalar = 0.0f;
        Quaternion ref_attitude_ang_error;
        Vector3f ref_ang_vel_body_rads;
        attitude_controller_run_quat(reference_target, body, state.ang_vel_target_rads, gyro, gains.rate_yaw_kp,
                                      gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch,
                                      gains.angle_kp_yaw, gains.angle_p_scale, gains.accel_roll_max_radss,
                                      gains.accel_pitch_max_radss, gains.accel_yaw_max_radss,
                                      gains.use_sqrt_controller, gains.ang_vel_roll_max_degs,
                                      gains.ang_vel_pitch_max_degs, gains.ang_vel_yaw_max_degs, 0.01f,
                                      ref_thrust_angle, ref_thrust_error_angle, ref_feedforward_scalar,
                                      ref_attitude_ang_error, ref_ang_vel_body_rads);
        REQUIRE(ang_vel_body_rads.x == Approx(ref_ang_vel_body_rads.x).margin(1e-6f));
        REQUIRE(ang_vel_body_rads.y == Approx(ref_ang_vel_body_rads.y).margin(1e-6f));
        REQUIRE(ang_vel_body_rads.z == Approx(ref_ang_vel_body_rads.z).margin(1e-6f));
    }
}

TEST_CASE("input_euler_rate_roll_pitch_yaw_rads: rate_rp_tc shapes roll+pitch, rate_y_tc shapes yaw, not input_tc",
          "[control][attitude_kinematics][input_euler_rate_roll_pitch_yaw][CCP-031][rate_rp_tc]") {
    const float dt = 0.0025f;
    const float cmd = 1.0f;
    const int kSteps = 20;

    auto run = [&](float rate_rp_tc, float rate_y_tc, float input_tc, float roll_cmd, float pitch_cmd,
                   float yaw_cmd) {
        EntryPointGains eg;
        eg.rate_rp_tc = rate_rp_tc;
        eg.rate_y_tc = rate_y_tc;
        eg.input_tc = input_tc;
        const EulerAngleRateShapingGains gains = eg.gains();
        AttitudeTargetState state = fresh_state();
        for (int i = 0; i < kSteps; ++i) {
            step_euler_rate(roll_cmd, pitch_cmd, yaw_cmd, state, gains, dt);
        }
        return state;
    };

    // Roll-only: different rate_rp_tc, same rate_y_tc and input_tc.
    const AttitudeTargetState sharp_rp = run(0.02f, 0.30f, 0.40f, cmd, 0.0f, 0.0f);
    const AttitudeTargetState slow_rp = run(0.30f, 0.30f, 0.40f, cmd, 0.0f, 0.0f);
    REQUIRE(sharp_rp.euler_rate_target_rads.x < cmd);
    REQUIRE(slow_rp.euler_rate_target_rads.x < cmd);
    REQUIRE(sharp_rp.euler_rate_target_rads.x > slow_rp.euler_rate_target_rads.x + 0.08f);

    // Yaw-only, same rate_y_tc, different rate_rp_tc: yaw must match. A
    // port that reused rate_rp_tc for yaw would diverge. Commanding
    // yaw-only keeps body_to_euler_limit stable (heading does not
    // change phi/theta), so this is not an attitude-coupling artifact.
    const AttitudeTargetState yaw_rp_a = run(0.02f, 0.30f, 0.40f, 0.0f, 0.0f, cmd);
    const AttitudeTargetState yaw_rp_b = run(0.30f, 0.30f, 0.40f, 0.0f, 0.0f, cmd);
    REQUIRE(yaw_rp_a.euler_rate_target_rads.z == Approx(yaw_rp_b.euler_rate_target_rads.z).margin(1e-5f));

    // Yaw-only: sharper rate_y_tc converges further. Roll-only with the
    // same rate_rp_tc is unaffected by rate_y_tc.
    const AttitudeTargetState sharp_y = run(0.30f, 0.02f, 0.40f, 0.0f, 0.0f, cmd);
    const AttitudeTargetState slow_y = run(0.30f, 0.30f, 0.40f, 0.0f, 0.0f, cmd);
    REQUIRE(sharp_y.euler_rate_target_rads.z > slow_y.euler_rate_target_rads.z + 0.08f);
    const AttitudeTargetState roll_y_a = run(0.30f, 0.02f, 0.40f, cmd, 0.0f, 0.0f);
    const AttitudeTargetState roll_y_b = run(0.30f, 0.30f, 0.40f, cmd, 0.0f, 0.0f);
    REQUIRE(roll_y_a.euler_rate_target_rads.x == Approx(roll_y_b.euler_rate_target_rads.x).margin(1e-5f));

    // input_tc is unused on this path: swapping it must not change any axis.
    const AttitudeTargetState input_sharp = run(0.15f, 0.25f, 0.02f, cmd, cmd, cmd);
    const AttitudeTargetState input_slow = run(0.15f, 0.25f, 0.40f, cmd, cmd, cmd);
    REQUIRE(input_sharp.euler_rate_target_rads.x == Approx(input_slow.euler_rate_target_rads.x).margin(1e-5f));
    REQUIRE(input_sharp.euler_rate_target_rads.y == Approx(input_slow.euler_rate_target_rads.y).margin(1e-5f));
    REQUIRE(input_sharp.euler_rate_target_rads.z == Approx(input_slow.euler_rate_target_rads.z).margin(1e-5f));
}

TEST_CASE("input_euler_rate_roll_pitch_yaw_rads: all three axes are rate-shaped, not angle-error shaped",
          "[control][attitude_kinematics][input_euler_rate_roll_pitch_yaw][CCP-031][rate-shape]") {
    EntryPointGains eg;
    eg.rate_rp_tc = 0.15f;
    eg.rate_y_tc = 0.25f;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f;
    const float rate_cmd = 0.3f;

    // Yaw-only: heading does not change body_to_euler_limit, so an
    // independent attitude_command_model loop with rate_y_tc and the
    // yaw accel limit must match the entry point. The naive
    // angle-error shape (CCP-029/030 roll/pitch argument order) must
    // not.
    const int kYawSteps = 150;
    AttitudeTargetState yaw_state = fresh_state();
    for (int i = 0; i < kYawSteps; ++i) {
        step_euler_rate(0.0f, 0.0f, rate_cmd, yaw_state, gains, dt);
    }

    float correct_y = 0.0f, correct_y_accel = 0.0f;
    for (int i = 0; i < kYawSteps; ++i) {
        attitude_command_model(0.0f, rate_cmd, correct_y, correct_y_accel, 0.0f, gains.accel_yaw_max_radss,
                                gains.rate_y_tc, dt);
    }

    float naive_rate = 0.0f, naive_accel = 0.0f;
    for (int i = 0; i < kYawSteps; ++i) {
        attitude_command_model(fwcpp::math::wrap_PI(rate_cmd - 0.0f), 0.0f, naive_rate, naive_accel, 0.0f,
                                gains.accel_yaw_max_radss, gains.rate_y_tc, dt);
    }

    REQUIRE(std::fabs(correct_y - naive_rate) > 0.2f);
    REQUIRE(yaw_state.euler_rate_target_rads.z == Approx(correct_y).margin(1e-4f));
    REQUIRE(std::fabs(yaw_state.euler_rate_target_rads.z - naive_rate) > 0.2f);

    // Roll-only, short enough that |phi| stays inside body_to_euler_limit's
    // 0.1 sin-floor, so the roll accel limit is still accel_roll_max.
    const int kRpSteps = 20;
    AttitudeTargetState roll_state = fresh_state();
    for (int i = 0; i < kRpSteps; ++i) {
        step_euler_rate(rate_cmd, 0.0f, 0.0f, roll_state, gains, dt);
    }
    float correct_r = 0.0f, correct_r_accel = 0.0f;
    for (int i = 0; i < kRpSteps; ++i) {
        attitude_command_model(0.0f, rate_cmd, correct_r, correct_r_accel, 0.0f, gains.accel_roll_max_radss,
                                gains.rate_rp_tc, dt);
    }
    REQUIRE(roll_state.euler_rate_target_rads.x == Approx(correct_r).margin(1e-4f));
    REQUIRE(roll_state.euler_rate_target_rads.x > 0.0f);
}


// =======================================================================
// CCP-032: input_rate_bf_roll_pitch_yaw family
// Four distinct body-frame rate laws + each _cds wrapper.
// =======================================================================

void step_rate_bf(float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads,
                  AttitudeTargetState& state, const EulerAngleRateShapingGains& gains, float dt) {
    const Quaternion body = attitude(0.0f, 0.0f, 0.0f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
    Quaternion attitude_ang_error;
    Vector3f ang_vel_body_rads;
    input_rate_bf_roll_pitch_yaw_rads(roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads, state, body, gyro, gains,
                                      dt, thrust_angle, thrust_error_angle, feedforward_scalar, attitude_ang_error,
                                      ang_vel_body_rads);
}

TEST_CASE("input_rate_bf_roll_pitch_yaw_rads: unshaped right-multiplies from_axis_angle(rates*dt) and zeros ff",
          "[control][attitude_kinematics][input_rate_bf][CCP-032][v1-unshaped]") {
    const Quaternion body = attitude(0.15f, -0.10f, 0.40f);
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    EntryPointGains eg;
    EulerAngleRateShapingGains gains = eg.gains();
    gains.rate_bf_ff_enabled = false;

    const float dt = 0.01f;
    const float roll_rate = 0.8f;
    const float pitch_rate = -0.5f;
    const float yaw_rate = 1.2f;

    AttitudeTargetState state = fresh_state();
    state.attitude_target = attitude(0.20f, 0.10f, 0.30f);
    state.euler_rate_target_rads = Vector3f{1.0f, -1.0f, 1.0f};
    state.ang_vel_target_rads = Vector3f{0.0f, 0.0f, 0.0f};
    state.ang_accel_target_rads = Vector3f{3.0f, -3.0f, 3.0f};

    Quaternion expected_multiply = state.attitude_target;
    Quaternion update;
    update.from_axis_angle(Vector3f{roll_rate, pitch_rate, yaw_rate} * dt);
    expected_multiply = expected_multiply * update;
    expected_multiply.normalize();

    float thrust_angle = 0.0f, thrust_error_angle = 0.0f, feedforward_scalar = 0.0f;
    Quaternion attitude_ang_error;
    Vector3f ang_vel_body_rads;
    input_rate_bf_roll_pitch_yaw_rads(roll_rate, pitch_rate, yaw_rate, state, body, gyro, gains, dt, thrust_angle,
                                      thrust_error_angle, feedforward_scalar, attitude_ang_error, ang_vel_body_rads);

    // run_quat may rewrite attitude_target; the multiply is the input to it.
    Quaternion expected_target = expected_multiply;
    float ref_t = 0.0f, ref_te = 0.0f, ref_ff = 0.0f;
    Quaternion ref_err;
    Vector3f ref_av;
    attitude_controller_run_quat(expected_target, body, Vector3f{0.0f, 0.0f, 0.0f}, gyro, gains.rate_yaw_kp,
                                  gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch, gains.angle_kp_yaw,
                                  gains.angle_p_scale, gains.accel_roll_max_radss, gains.accel_pitch_max_radss,
                                  gains.accel_yaw_max_radss, gains.use_sqrt_controller, gains.ang_vel_roll_max_degs,
                                  gains.ang_vel_pitch_max_degs, gains.ang_vel_yaw_max_degs, dt, ref_t, ref_te, ref_ff,
                                  ref_err, ref_av);
    REQUIRE(state.attitude_target.q1 == Approx(expected_target.q1).margin(1e-6f));
    REQUIRE(state.attitude_target.q2 == Approx(expected_target.q2).margin(1e-6f));
    REQUIRE(state.attitude_target.q3 == Approx(expected_target.q3).margin(1e-6f));
    REQUIRE(state.attitude_target.q4 == Approx(expected_target.q4).margin(1e-6f));

    REQUIRE(state.euler_rate_target_rads.x == 0.0f);
    REQUIRE(state.euler_rate_target_rads.y == 0.0f);
    REQUIRE(state.euler_rate_target_rads.z == 0.0f);
    REQUIRE(state.ang_vel_target_rads.x == 0.0f);
    REQUIRE(state.ang_vel_target_rads.y == 0.0f);
    REQUIRE(state.ang_vel_target_rads.z == 0.0f);
    REQUIRE(state.ang_accel_target_rads.x == 0.0f);
    REQUIRE(state.ang_accel_target_rads.y == 0.0f);
    REQUIRE(state.ang_accel_target_rads.z == 0.0f);

    // Body-frame axis-angle, not CCP-031's Euler wrap/clamp. Unshaped v1
    // leaves euler_angle_target at the pre-multiply to_euler (here 0)
    // rather than writing constrain(pitch, ±85deg). The multiply itself
    // is identity * from_axis_angle((0,2,0)), which is not from_euler of
    // the 85 deg clamp.
    {
        AttitudeTargetState pitch_state = fresh_state();
        pitch_state.ang_vel_target_rads.zero();
        float t = 0.0f, te = 0.0f, ff = 0.0f;
        Quaternion err;
        Vector3f av;
        input_rate_bf_roll_pitch_yaw_rads(0.0f, 2.0f, 0.0f, pitch_state, body, gyro, gains, 1.0f, t, te, ff, err, av);
        REQUIRE(std::fabs(pitch_state.euler_angle_target_rad.y) < 0.01f);

        Quaternion axis;
        axis.from_axis_angle(Vector3f{0.0f, 2.0f, 0.0f});
        Quaternion clamped;
        clamped.from_euler(0.0f, fwcpp::math::radians(85.0f), 0.0f);
        REQUIRE(std::fabs(axis.q1 - clamped.q1) + std::fabs(axis.q2 - clamped.q2) + std::fabs(axis.q3 - clamped.q3) +
                    std::fabs(axis.q4 - clamped.q4) >
                0.05f);

        Quaternion after_run = axis;
        float rt = 0.0f, rte = 0.0f, rff = 0.0f;
        Quaternion rerr;
        Vector3f rav;
        attitude_controller_run_quat(after_run, body, Vector3f{0.0f, 0.0f, 0.0f}, gyro, gains.rate_yaw_kp,
                                      gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch,
                                      gains.angle_kp_yaw, gains.angle_p_scale, gains.accel_roll_max_radss,
                                      gains.accel_pitch_max_radss, gains.accel_yaw_max_radss,
                                      gains.use_sqrt_controller, gains.ang_vel_roll_max_degs,
                                      gains.ang_vel_pitch_max_degs, gains.ang_vel_yaw_max_degs, 1.0f, rt, rte, rff,
                                      rerr, rav);
        REQUIRE(pitch_state.attitude_target.q1 == Approx(after_run.q1).margin(1e-5f));
        REQUIRE(pitch_state.attitude_target.q2 == Approx(after_run.q2).margin(1e-5f));
        REQUIRE(pitch_state.attitude_target.q3 == Approx(after_run.q3).margin(1e-5f));
        REQUIRE(pitch_state.attitude_target.q4 == Approx(after_run.q4).margin(1e-5f));
    }

    // run_quat still runs on the post-multiply target with zeroed ff.
    Quaternion reference_target = state.attitude_target;
    float ref_thrust_angle = 0.0f, ref_thrust_error_angle = 0.0f, ref_feedforward_scalar = 0.0f;
    Quaternion ref_attitude_ang_error;
    Vector3f ref_ang_vel_body_rads;
    attitude_controller_run_quat(reference_target, body, state.ang_vel_target_rads, gyro, gains.rate_yaw_kp,
                                  gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch, gains.angle_kp_yaw,
                                  gains.angle_p_scale, gains.accel_roll_max_radss, gains.accel_pitch_max_radss,
                                  gains.accel_yaw_max_radss, gains.use_sqrt_controller, gains.ang_vel_roll_max_degs,
                                  gains.ang_vel_pitch_max_degs, gains.ang_vel_yaw_max_degs, dt, ref_thrust_angle,
                                  ref_thrust_error_angle, ref_feedforward_scalar, ref_attitude_ang_error,
                                  ref_ang_vel_body_rads);
    REQUIRE(ang_vel_body_rads.x == Approx(ref_ang_vel_body_rads.x).margin(1e-6f));
    REQUIRE(ang_vel_body_rads.y == Approx(ref_ang_vel_body_rads.y).margin(1e-6f));
    REQUIRE(ang_vel_body_rads.z == Approx(ref_ang_vel_body_rads.z).margin(1e-6f));
}

TEST_CASE("input_rate_bf_roll_pitch_yaw_rads: shaped path is body-frame command_model, then body_to_euler_derivative",
          "[control][attitude_kinematics][input_rate_bf][CCP-032][v1-shaped]") {
    EntryPointGains eg;
    eg.rate_rp_tc = 0.15f;
    eg.rate_y_tc = 0.25f;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f;
    const float cmd = 0.8f;
    const int kSteps = 40;

    AttitudeTargetState state = fresh_state();
    for (int i = 0; i < kSteps; ++i) {
        step_rate_bf(cmd, 0.0f, cmd, state, gains, dt);
    }

    float body_roll = 0.0f, body_roll_accel = 0.0f;
    float body_yaw = 0.0f, body_yaw_accel = 0.0f;
    for (int i = 0; i < kSteps; ++i) {
        attitude_command_model(0.0f, cmd, body_roll, body_roll_accel, 0.0f, gains.accel_roll_max_radss,
                                gains.rate_rp_tc, dt);
        attitude_command_model(0.0f, cmd, body_yaw, body_yaw_accel, 0.0f, gains.accel_yaw_max_radss, gains.rate_y_tc,
                                dt);
    }

    REQUIRE(state.ang_vel_target_rads.x == Approx(body_roll).margin(1e-5f));
    REQUIRE(state.ang_vel_target_rads.z == Approx(body_yaw).margin(1e-5f));
    REQUIRE(state.ang_vel_target_rads.x > state.ang_vel_target_rads.z + 0.02f);

    Vector3f expected_euler;
    REQUIRE(body_to_euler_derivative(state.attitude_target, state.ang_vel_target_rads, expected_euler));
    REQUIRE(state.euler_rate_target_rads.x == Approx(expected_euler.x).margin(1e-5f));
    REQUIRE(state.euler_rate_target_rads.y == Approx(expected_euler.y).margin(1e-5f));
    REQUIRE(state.euler_rate_target_rads.z == Approx(expected_euler.z).margin(1e-5f));
}

TEST_CASE("input_rate_bf_roll_pitch_yaw_2_rads: always shapes, copies AHRS, no run_quat",
          "[control][attitude_kinematics][input_rate_bf][CCP-032][v2]") {
    EntryPointGains eg;
    eg.rate_rp_tc = 0.15f;
    eg.rate_y_tc = 0.25f;
    EulerAngleRateShapingGains gains = eg.gains();
    gains.rate_bf_ff_enabled = false;

    const Quaternion ahrs = attitude(0.25f, -0.15f, 0.80f);
    const float dt = 0.0025f;
    const float cmd = 1.0f;

    AttitudeTargetState state = fresh_state();
    state.attitude_target = attitude(0.05f, 0.05f, 0.05f);
    state.ang_vel_target_rads = Vector3f{0.1f, -0.1f, 0.2f};

    Vector3f ang_vel_body;
    input_rate_bf_roll_pitch_yaw_2_rads(cmd, 0.0f, 0.0f, state, ahrs, gains, dt, ang_vel_body);

    // AHRS copy, not the previous target and not a run_quat rewrite.
    REQUIRE(state.attitude_target.q1 == Approx(ahrs.q1).margin(1e-6f));
    REQUIRE(state.attitude_target.q2 == Approx(ahrs.q2).margin(1e-6f));
    REQUIRE(state.attitude_target.q3 == Approx(ahrs.q3).margin(1e-6f));
    REQUIRE(state.attitude_target.q4 == Approx(ahrs.q4).margin(1e-6f));

    Vector3f euler_from_ahrs;
    ahrs.to_euler(euler_from_ahrs);
    REQUIRE(state.euler_angle_target_rad.x == Approx(euler_from_ahrs.x).margin(1e-6f));
    REQUIRE(state.euler_angle_target_rad.y == Approx(euler_from_ahrs.y).margin(1e-6f));
    REQUIRE(state.euler_angle_target_rad.z == Approx(euler_from_ahrs.z).margin(1e-6f));

    // Always shapes: first-step body rate is not the raw command, even
    // with rate_bf_ff_enabled false.
    REQUIRE(state.ang_vel_target_rads.x > 0.1f);
    REQUIRE(state.ang_vel_target_rads.x < cmd * 0.5f);

    float shaped = 0.1f, shaped_accel = 0.0f;
    attitude_command_model(0.0f, cmd, shaped, shaped_accel, 0.0f, gains.accel_roll_max_radss, gains.rate_rp_tc, dt);
    REQUIRE(state.ang_vel_target_rads.x == Approx(shaped).margin(1e-6f));

    // No run_quat: body rate is the shaped target, not the error-based
    // run_quat correction that a mismatched AHRS/target pair would produce.
    REQUIRE(ang_vel_body.x == Approx(state.ang_vel_target_rads.x).margin(1e-6f));
    REQUIRE(ang_vel_body.y == Approx(state.ang_vel_target_rads.y).margin(1e-6f));
    REQUIRE(ang_vel_body.z == Approx(state.ang_vel_target_rads.z).margin(1e-6f));

    AttitudeTargetState run_state = fresh_state();
    run_state.attitude_target = attitude(0.05f, 0.05f, 0.05f);
    float t = 0.0f, te = 0.0f, ff = 0.0f;
    Quaternion err;
    Vector3f run_quat_body;
    const Vector3f gyro{0.0f, 0.0f, 0.0f};
    attitude_controller_run_quat(run_state.attitude_target, ahrs, state.ang_vel_target_rads, gyro, gains.rate_yaw_kp,
                                  gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch, gains.angle_kp_yaw,
                                  gains.angle_p_scale, gains.accel_roll_max_radss, gains.accel_pitch_max_radss,
                                  gains.accel_yaw_max_radss, gains.use_sqrt_controller, gains.ang_vel_roll_max_degs,
                                  gains.ang_vel_pitch_max_degs, gains.ang_vel_yaw_max_degs, dt, t, te, ff, err,
                                  run_quat_body);
    REQUIRE(std::fabs(ang_vel_body.x - run_quat_body.x) + std::fabs(ang_vel_body.y - run_quat_body.y) +
                std::fabs(ang_vel_body.z - run_quat_body.z) >
            0.05f);

    // rate_bf_ff_enabled true vs false is identical on this path.
    EulerAngleRateShapingGains gains_ff = gains;
    gains_ff.rate_bf_ff_enabled = true;
    AttitudeTargetState state_ff = fresh_state();
    state_ff.ang_vel_target_rads = Vector3f{0.1f, -0.1f, 0.2f};
    Vector3f ang_vel_ff;
    input_rate_bf_roll_pitch_yaw_2_rads(cmd, 0.0f, 0.0f, state_ff, ahrs, gains_ff, dt, ang_vel_ff);
    REQUIRE(state_ff.ang_vel_target_rads.x == Approx(state.ang_vel_target_rads.x).margin(1e-6f));
    REQUIRE(ang_vel_ff.x == Approx(ang_vel_body.x).margin(1e-6f));
}

TEST_CASE("input_rate_bf_roll_pitch_yaw_3_rads: integrator path differs from v1",
          "[control][attitude_kinematics][input_rate_bf][CCP-032][v3]") {
    EntryPointGains eg;
    eg.rate_rp_tc = 0.15f;
    eg.rate_y_tc = 0.25f;
    const EulerAngleRateShapingGains gains = eg.gains();
    const Quaternion ahrs = attitude(0.20f, -0.10f, 0.50f);
    const Vector3f gyro{0.3f, -0.2f, 0.1f};
    const float dt = 0.01f;
    const float cmd = 0.6f;

    AttitudeTargetState v3_state = fresh_state();
    v3_state.ang_vel_target_rads = Vector3f{0.4f, 0.0f, 0.0f};
    Quaternion v3_error;
    v3_error.from_axis_angle(Vector3f{0.12f, -0.08f, 0.05f});
    Vector3f v3_body;
    input_rate_bf_roll_pitch_yaw_3_rads(cmd, 0.0f, 0.0f, v3_state, ahrs, gyro, gains, dt, v3_error, v3_body);

    AttitudeTargetState v1_state = fresh_state();
    v1_state.ang_vel_target_rads = Vector3f{0.4f, 0.0f, 0.0f};
    float t = 0.0f, te = 0.0f, ff = 0.0f;
    Quaternion v1_error;
    Vector3f v1_body;
    input_rate_bf_roll_pitch_yaw_rads(cmd, 0.0f, 0.0f, v1_state, ahrs, gyro, gains, dt, t, te, ff, v1_error, v1_body);

    REQUIRE(std::fabs(v3_state.attitude_target.q1 - v1_state.attitude_target.q1) +
                std::fabs(v3_state.attitude_target.q2 - v1_state.attitude_target.q2) +
                std::fabs(v3_state.attitude_target.q3 - v1_state.attitude_target.q3) +
                std::fabs(v3_state.attitude_target.q4 - v1_state.attitude_target.q4) >
            0.01f);
    REQUIRE(std::fabs(v3_body.x - v1_body.x) + std::fabs(v3_body.y - v1_body.y) + std::fabs(v3_body.z - v1_body.z) >
            0.05f);

    // Reconstruct the integrator: clamp, then left-multiply
    // from_axis_angle((pre_shape_target - gyro) * dt).
    Quaternion expected_error;
    expected_error.from_axis_angle(Vector3f{0.12f, -0.08f, 0.05f});
    Vector3f err_vec;
    expected_error.to_axis_angle(err_vec);
    const float err_mag = err_vec.length();
    if (err_mag > kAttitudeThrustErrorAngleRad) {
        err_vec *= kAttitudeThrustErrorAngleRad / err_mag;
        expected_error.from_axis_angle(err_vec);
    }
    Quaternion update;
    update.from_axis_angle((Vector3f{0.4f, 0.0f, 0.0f} - gyro) * dt);
    expected_error = update * expected_error;
    expected_error.normalize();

    REQUIRE(v3_error.q1 == Approx(expected_error.q1).margin(1e-6f));
    REQUIRE(v3_error.q2 == Approx(expected_error.q2).margin(1e-6f));
    REQUIRE(v3_error.q3 == Approx(expected_error.q3).margin(1e-6f));
    REQUIRE(v3_error.q4 == Approx(expected_error.q4).margin(1e-6f));

    Quaternion expected_target = ahrs * expected_error;
    expected_target.normalize();
    REQUIRE(v3_state.attitude_target.q1 == Approx(expected_target.q1).margin(1e-6f));
    REQUIRE(v3_state.attitude_target.q2 == Approx(expected_target.q2).margin(1e-6f));
    REQUIRE(v3_state.attitude_target.q3 == Approx(expected_target.q3).margin(1e-6f));
    REQUIRE(v3_state.attitude_target.q4 == Approx(expected_target.q4).margin(1e-6f));

    // Left multiply is load-bearing: right multiply does not commute.
    Quaternion right = expected_error;
    // Rebuild the pre-compose expected_error path's update against the
    // original (post-clamp) error, then compose the other way.
    Quaternion original;
    original.from_axis_angle(Vector3f{0.12f, -0.08f, 0.05f});
    Vector3f orig_vec;
    original.to_axis_angle(orig_vec);
    const float orig_mag = orig_vec.length();
    if (orig_mag > kAttitudeThrustErrorAngleRad) {
        orig_vec *= kAttitudeThrustErrorAngleRad / orig_mag;
        original.from_axis_angle(orig_vec);
    }
    right = original * update;
    right.normalize();
    REQUIRE(std::fabs(v3_error.q2 - right.q2) + std::fabs(v3_error.q3 - right.q3) +
                std::fabs(v3_error.q4 - right.q4) >
            1e-5f);

    // ang_vel_body = P(error) + shaped ang_vel_target, not run_quat.
    Vector3f post_err;
    expected_error.to_axis_angle(post_err);
    const Vector3f p_term = update_ang_vel_target_from_att_error(
        post_err, gains.angle_kp_roll, gains.angle_kp_pitch, gains.angle_kp_yaw, gains.angle_p_scale,
        gains.accel_roll_max_radss, gains.accel_pitch_max_radss, gains.accel_yaw_max_radss, gains.use_sqrt_controller,
        dt);
    REQUIRE(v3_body.x == Approx(p_term.x + v3_state.ang_vel_target_rads.x).margin(1e-5f));
    REQUIRE(v3_body.y == Approx(p_term.y + v3_state.ang_vel_target_rads.y).margin(1e-5f));
    REQUIRE(v3_body.z == Approx(p_term.z + v3_state.ang_vel_target_rads.z).margin(1e-5f));

    // Windup clamp: a 1.2 rad error is pulled back to 30 deg before integrate.
    {
        AttitudeTargetState clamp_state = fresh_state();
        Quaternion huge;
        huge.from_axis_angle(Vector3f{1.2f, 0.0f, 0.0f});
        Vector3f body_out;
        const Vector3f zero_gyro{0.0f, 0.0f, 0.0f};
        input_rate_bf_roll_pitch_yaw_3_rads(0.0f, 0.0f, 0.0f, clamp_state, ahrs, zero_gyro, gains, dt, huge, body_out);
        Vector3f clamped;
        huge.to_axis_angle(clamped);
        REQUIRE(clamped.length() == Approx(kAttitudeThrustErrorAngleRad).margin(1e-4f));
        REQUIRE(clamped.length() < 1.0f);
    }
}

TEST_CASE("input_rate_bf_roll_pitch_yaw_no_shaping_rads: writes rates directly, copies AHRS, no run_quat",
          "[control][attitude_kinematics][input_rate_bf][CCP-032][no-shaping]") {
    const Quaternion ahrs = attitude(-0.18f, 0.22f, 1.10f);
    const float roll = 1.5f;
    const float pitch = -0.7f;
    const float yaw = 0.4f;

    AttitudeTargetState state = fresh_state();
    state.attitude_target = attitude(0.3f, -0.2f, 0.1f);
    state.ang_vel_target_rads = Vector3f{9.0f, 9.0f, 9.0f};
    state.ang_accel_target_rads = Vector3f{4.0f, 4.0f, 4.0f};

    Vector3f ang_vel_body;
    input_rate_bf_roll_pitch_yaw_no_shaping_rads(roll, pitch, yaw, state, ahrs, ang_vel_body);

    REQUIRE(state.ang_vel_target_rads.x == roll);
    REQUIRE(state.ang_vel_target_rads.y == pitch);
    REQUIRE(state.ang_vel_target_rads.z == yaw);
    REQUIRE(ang_vel_body.x == roll);
    REQUIRE(ang_vel_body.y == pitch);
    REQUIRE(ang_vel_body.z == yaw);

    REQUIRE(state.attitude_target.q1 == Approx(ahrs.q1).margin(1e-6f));
    REQUIRE(state.attitude_target.q2 == Approx(ahrs.q2).margin(1e-6f));
    REQUIRE(state.attitude_target.q3 == Approx(ahrs.q3).margin(1e-6f));
    REQUIRE(state.attitude_target.q4 == Approx(ahrs.q4).margin(1e-6f));

    Vector3f expected_euler;
    REQUIRE(body_to_euler_derivative(ahrs, Vector3f{roll, pitch, yaw}, expected_euler));
    REQUIRE(state.euler_rate_target_rads.x == Approx(expected_euler.x).margin(1e-6f));
    REQUIRE(state.euler_rate_target_rads.y == Approx(expected_euler.y).margin(1e-6f));
    REQUIRE(state.euler_rate_target_rads.z == Approx(expected_euler.z).margin(1e-6f));

    // Distinct from v2: first-step v2 would shape 1.5 rad/s down.
    EntryPointGains eg;
    const EulerAngleRateShapingGains gains = eg.gains();
    AttitudeTargetState v2_state = fresh_state();
    Vector3f v2_body;
    input_rate_bf_roll_pitch_yaw_2_rads(roll, pitch, yaw, v2_state, ahrs, gains, 0.0025f, v2_body);
    REQUIRE(v2_state.ang_vel_target_rads.x < roll * 0.5f);
    REQUIRE(state.ang_vel_target_rads.x == roll);

    // ang_accel is not zeroed (v1 unshaped would zero it).
    REQUIRE(state.ang_accel_target_rads.x == 4.0f);
}

TEST_CASE("input_rate_bf _cds wrappers convert via cd_to_rad and call the rads entry points",
          "[control][attitude_kinematics][input_rate_bf][CCP-032][cds]") {
    EntryPointGains eg;
    const EulerAngleRateShapingGains gains = eg.gains();
    const float dt = 0.0025f;
    const Quaternion body = attitude(0.10f, -0.05f, 0.20f);
    const Vector3f gyro{0.05f, -0.02f, 0.01f};
    const float roll_cds = 800.0f;
    const float pitch_cds = -400.0f;
    const float yaw_cds = 250.0f;

    // Variant 1.
    {
        AttitudeTargetState cd_state = fresh_state();
        AttitudeTargetState rad_state = fresh_state();
        float t_cd = 0.0f, te_cd = 0.0f, ff_cd = 0.0f;
        float t_rad = 0.0f, te_rad = 0.0f, ff_rad = 0.0f;
        Quaternion err_cd, err_rad;
        Vector3f av_cd, av_rad;
        input_rate_bf_roll_pitch_yaw_cds(roll_cds, pitch_cds, yaw_cds, cd_state, body, gyro, gains, dt, t_cd, te_cd,
                                          ff_cd, err_cd, av_cd);
        input_rate_bf_roll_pitch_yaw_rads(fwcpp::math::cd_to_rad(roll_cds), fwcpp::math::cd_to_rad(pitch_cds),
                                          fwcpp::math::cd_to_rad(yaw_cds), rad_state, body, gyro, gains, dt, t_rad,
                                          te_rad, ff_rad, err_rad, av_rad);
        REQUIRE(cd_state.ang_vel_target_rads.x == Approx(rad_state.ang_vel_target_rads.x).margin(1e-6f));
        REQUIRE(cd_state.ang_vel_target_rads.y == Approx(rad_state.ang_vel_target_rads.y).margin(1e-6f));
        REQUIRE(cd_state.ang_vel_target_rads.z == Approx(rad_state.ang_vel_target_rads.z).margin(1e-6f));
        REQUIRE(av_cd.x == Approx(av_rad.x).margin(1e-6f));
    }

    // Variant 2.
    {
        AttitudeTargetState cd_state = fresh_state();
        AttitudeTargetState rad_state = fresh_state();
        Vector3f av_cd, av_rad;
        input_rate_bf_roll_pitch_yaw_2_cds(roll_cds, pitch_cds, yaw_cds, cd_state, body, gains, dt, av_cd);
        input_rate_bf_roll_pitch_yaw_2_rads(fwcpp::math::cd_to_rad(roll_cds), fwcpp::math::cd_to_rad(pitch_cds),
                                            fwcpp::math::cd_to_rad(yaw_cds), rad_state, body, gains, dt, av_rad);
        REQUIRE(cd_state.ang_vel_target_rads.x == Approx(rad_state.ang_vel_target_rads.x).margin(1e-6f));
        REQUIRE(cd_state.attitude_target.q1 == Approx(rad_state.attitude_target.q1).margin(1e-6f));
        REQUIRE(av_cd.x == Approx(av_rad.x).margin(1e-6f));
    }

    // Variant 3.
    {
        AttitudeTargetState cd_state = fresh_state();
        AttitudeTargetState rad_state = fresh_state();
        cd_state.ang_vel_target_rads = Vector3f{0.2f, 0.0f, 0.0f};
        rad_state.ang_vel_target_rads = Vector3f{0.2f, 0.0f, 0.0f};
        Quaternion err_cd, err_rad;
        err_cd.from_axis_angle(Vector3f{0.05f, 0.0f, 0.0f});
        err_rad.from_axis_angle(Vector3f{0.05f, 0.0f, 0.0f});
        Vector3f av_cd, av_rad;
        input_rate_bf_roll_pitch_yaw_3_cds(roll_cds, pitch_cds, yaw_cds, cd_state, body, gyro, gains, dt, err_cd,
                                            av_cd);
        input_rate_bf_roll_pitch_yaw_3_rads(fwcpp::math::cd_to_rad(roll_cds), fwcpp::math::cd_to_rad(pitch_cds),
                                            fwcpp::math::cd_to_rad(yaw_cds), rad_state, body, gyro, gains, dt, err_rad,
                                            av_rad);
        REQUIRE(cd_state.ang_vel_target_rads.x == Approx(rad_state.ang_vel_target_rads.x).margin(1e-6f));
        REQUIRE(err_cd.q1 == Approx(err_rad.q1).margin(1e-6f));
        REQUIRE(av_cd.x == Approx(av_rad.x).margin(1e-6f));
        REQUIRE(cd_state.attitude_target.q2 == Approx(rad_state.attitude_target.q2).margin(1e-6f));
    }

    // no_shaping.
    {
        AttitudeTargetState cd_state = fresh_state();
        AttitudeTargetState rad_state = fresh_state();
        Vector3f av_cd, av_rad;
        input_rate_bf_roll_pitch_yaw_no_shaping_cds(roll_cds, pitch_cds, yaw_cds, cd_state, body, av_cd);
        input_rate_bf_roll_pitch_yaw_no_shaping_rads(fwcpp::math::cd_to_rad(roll_cds), fwcpp::math::cd_to_rad(pitch_cds),
                                                     fwcpp::math::cd_to_rad(yaw_cds), rad_state, body, av_rad);
        REQUIRE(cd_state.ang_vel_target_rads.x == Approx(rad_state.ang_vel_target_rads.x).margin(1e-6f));
        REQUIRE(cd_state.ang_vel_target_rads.y == Approx(rad_state.ang_vel_target_rads.y).margin(1e-6f));
        REQUIRE(cd_state.ang_vel_target_rads.z == Approx(rad_state.ang_vel_target_rads.z).margin(1e-6f));
        REQUIRE(av_cd.z == Approx(av_rad.z).margin(1e-6f));
    }
}

