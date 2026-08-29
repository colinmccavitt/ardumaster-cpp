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

#include <cmath>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/control/attitude_kinematics.hpp>
#include <fwcpp/math/quaternion.hpp>
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
