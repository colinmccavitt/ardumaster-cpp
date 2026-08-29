// Tests for QuaternionT<T> (CPP-009 slice 1; CCP-019 added the
// from_axis_angle/to_axis_angle/rotate(Vector3) axis-angle group below).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/quaternion.hpp>

using namespace fwcpp::math;

namespace {
// Upstream QuaternionT has no operator== (unlike Vector2/Vector3/Matrix3),
// so this port doesn't invent one either (ADR-0012 decision 9: keep
// upstream's API surface for diffability). Tests compare components.
void require_quat_approx(const Quaternion& a, const Quaternion& b, float margin = 1e-6f) {
    REQUIRE(a.q1 == Catch::Approx(b.q1).margin(margin));
    REQUIRE(a.q2 == Catch::Approx(b.q2).margin(margin));
    REQUIRE(a.q3 == Catch::Approx(b.q3).margin(margin));
    REQUIRE(a.q4 == Catch::Approx(b.q4).margin(margin));
}
} // namespace

TEST_CASE("Quaternion default constructor is the identity rotation", "[quaternion]") {
    Quaternion q;
    REQUIRE(q.q1 == 1.0f);
    REQUIRE(q.q2 == 0.0f);
    REQUIRE(q.q3 == 0.0f);
    REQUIRE(q.q4 == 0.0f);
}

TEST_CASE("Quaternion initialise resets to the identity rotation", "[quaternion]") {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    q.initialise();
    REQUIRE(q.q1 == 1.0f);
    REQUIRE(q.q2 == 0.0f);
}

TEST_CASE("Quaternion is_nan and is_zero", "[quaternion]") {
    REQUIRE_FALSE(Quaternion().is_nan());
    REQUIRE(Quaternion(std::nanf(""), 0, 0, 0).is_nan());

    Quaternion z;
    z.zero();
    REQUIRE(z.is_zero());
    REQUIRE_FALSE(Quaternion().is_zero()); // identity is not zero
}

TEST_CASE("Quaternion zero() gives an invalid (not identity) quaternion", "[quaternion]") {
    Quaternion q;
    q.zero();
    REQUIRE(q.q1 == 0.0f); // NOT 1 - zero() is the invalid quaternion, not identity
}

TEST_CASE("Quaternion length of the identity is 1", "[quaternion]") {
    REQUIRE(Quaternion().length() == Catch::Approx(1.0f));
    REQUIRE(Quaternion().length_squared() == Catch::Approx(1.0f));
}

TEST_CASE("Quaternion normalize produces a unit quaternion", "[quaternion]") {
    Quaternion q(2.0f, 0.0f, 0.0f, 0.0f);
    q.normalize();
    REQUIRE(q.length() == Catch::Approx(1.0f));
    REQUIRE(q.q1 == Catch::Approx(1.0f));
}

TEST_CASE("Quaternion normalize on a zero quaternion leaves it unchanged and reports via InternalError", "[quaternion]") {
    Quaternion q;
    q.zero();
    fwcpp::InternalError err;
    q.normalize(&err, 99);
    REQUIRE(q.is_zero()); // still zero - normalize can't invent a direction
    REQUIRE(err.has_error(fwcpp::InternalErrorCode::flow_of_control));
    REQUIRE(err.last_error_line() == 99);
}

TEST_CASE("Quaternion normalize with a null InternalError does not crash", "[quaternion]") {
    Quaternion q;
    q.zero();
    q.normalize(); // default nullptr
    REQUIRE(q.is_zero());
}

TEST_CASE("Quaternion is_unit_length", "[quaternion]") {
    REQUIRE(Quaternion().is_unit_length());
    Quaternion not_unit(2.0f, 0.0f, 0.0f, 0.0f);
    REQUIRE_FALSE(not_unit.is_unit_length());
}

TEST_CASE("Quaternion inverse negates the vector part only", "[quaternion]") {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    Quaternion inv = q.inverse();
    require_quat_approx(inv, Quaternion(0.5f, -0.5f, -0.5f, -0.5f));
}

TEST_CASE("Quaternion invert matches inverse but mutates in place", "[quaternion]") {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    Quaternion expected = q.inverse();
    q.invert();
    require_quat_approx(q, expected);
}

TEST_CASE("Quaternion composition with the identity is a no-op", "[quaternion]") {
    Quaternion id;
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    require_quat_approx(id * q, q);
    require_quat_approx(q * id, q);
}

TEST_CASE("Quaternion composed with its inverse is the identity", "[quaternion]") {
    Quaternion q;
    q.from_euler(0.3f, -0.2f, 1.1f);
    Quaternion product = q * q.inverse();
    REQUIRE(product.q1 == Catch::Approx(1.0f));
    REQUIRE(product.q2 == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(product.q3 == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(product.q4 == Catch::Approx(0.0f).margin(1e-6));
}

TEST_CASE("Quaternion from_euler / get_euler_roll,pitch,yaw round-trip", "[quaternion]") {
    const float roll = 0.3f, pitch = 0.2f, yaw = 1.0f;
    Quaternion q;
    q.from_euler(roll, pitch, yaw);
    REQUIRE(q.get_euler_roll() == Catch::Approx(roll).margin(1e-5));
    REQUIRE(q.get_euler_pitch() == Catch::Approx(pitch).margin(1e-5));
    REQUIRE(q.get_euler_yaw() == Catch::Approx(yaw).margin(1e-5));
}

TEST_CASE("Quaternion to_euler matches the individual get_euler_* accessors", "[quaternion]") {
    Quaternion q;
    q.from_euler(0.3f, 0.2f, 1.0f);
    float r, p, y;
    q.to_euler(r, p, y);
    REQUIRE(r == Catch::Approx(q.get_euler_roll()));
    REQUIRE(p == Catch::Approx(q.get_euler_pitch()));
    REQUIRE(y == Catch::Approx(q.get_euler_yaw()));
}

TEST_CASE("Quaternion from_rotation_matrix / rotation_matrix round-trip", "[quaternion]") {
    Quaternion q;
    q.from_euler(0.3f, 0.2f, 1.0f);

    Matrix3f m;
    q.rotation_matrix(m);

    Quaternion q2;
    q2.from_rotation_matrix(m);

    // q and q2 may differ by sign (q and -q represent the same rotation),
    // so compare via the rotation they produce, not the raw components.
    Vector3f v(1.0f, 2.0f, 3.0f);
    Vector3f from_q = q * v;
    Vector3f from_q2 = q2 * v;
    REQUIRE(from_q.x == Catch::Approx(from_q2.x).margin(1e-4));
    REQUIRE(from_q.y == Catch::Approx(from_q2.y).margin(1e-4));
    REQUIRE(from_q.z == Catch::Approx(from_q2.z).margin(1e-4));
}

TEST_CASE("Quaternion*Vector3 rotation agrees with Matrix3 rotation for the same angles", "[quaternion]") {
    // Cross-check between two independently-implemented rotation paths:
    // quaternion's optimized operator*(Vector3) vs going through
    // Matrix3::from_euler and multiplying. If these two disagree,
    // one of the two ports has a sign or index error.
    const float roll = 0.4f, pitch = -0.3f, yaw = 0.7f;
    Vector3f v(1.0f, 2.0f, 3.0f);

    Quaternion q;
    q.from_euler(roll, pitch, yaw);
    Vector3f via_quat = q * v;

    Matrix3f m;
    m.from_euler(roll, pitch, yaw);
    Vector3f via_matrix = m * v;

    REQUIRE(via_quat.x == Catch::Approx(via_matrix.x).margin(1e-4));
    REQUIRE(via_quat.y == Catch::Approx(via_matrix.y).margin(1e-4));
    REQUIRE(via_quat.z == Catch::Approx(via_matrix.z).margin(1e-4));
}

TEST_CASE("Quaternion todouble/tofloat convert every component", "[quaternion]") {
    QuaternionD d(1.0, 0.5, 0.25, 0.125);
    Quaternion f = d.tofloat();
    REQUIRE(f.q2 == Catch::Approx(0.5f));
    QuaternionD back = f.todouble();
    REQUIRE(back.q2 == Catch::Approx(0.5));
}

// CCP-019: from_axis_angle (both overloads) / to_axis_angle / rotate(Vector3).

TEST_CASE("Quaternion from_axis_angle/to_axis_angle round-trip for angles well within the wrap boundary", "[quaternion][axis_angle]") {
    // Several axes and angles, all comfortably inside (-pi, pi] so the
    // wrap in to_axis_angle never engages - isolates the round-trip itself
    // from the wrap behavior tested separately below.
    struct Case { Vector3f axis; float angle; };
    const Case cases[] = {
        {Vector3f(0.0f, 0.0f, 1.0f), 0.1f},
        {Vector3f(1.0f, 0.0f, 0.0f), 0.7f},
        {Vector3f(0.0f, 1.0f, 0.0f), 1.5f},
        {Vector3f(0.0f, 0.0f, 1.0f), 3.0f},
        {Vector3f(0.267261f, 0.534522f, 0.801784f), 2.0f}, // non-axis-aligned unit axis
    };
    for (const auto& c : cases) {
        Quaternion q;
        q.from_axis_angle(c.axis, c.angle);

        Vector3f back;
        q.to_axis_angle(back);

        // back's direction should match c.axis and its length should
        // match c.angle (to_axis_angle re-derives both from the
        // quaternion, independent of how from_axis_angle built it).
        REQUIRE(back.length() == Catch::Approx(c.angle).margin(1e-5f));
        Vector3f back_dir = back / back.length();
        REQUIRE(back_dir.x == Catch::Approx(c.axis.x).margin(1e-5f));
        REQUIRE(back_dir.y == Catch::Approx(c.axis.y).margin(1e-5f));
        REQUIRE(back_dir.z == Catch::Approx(c.axis.z).margin(1e-5f));
    }
}

TEST_CASE("Quaternion from_axis_angle(Vector3) rotation-vector overload round-trips through to_axis_angle", "[quaternion][axis_angle]") {
    // The single-argument "rotation vector" overload: direction is axis,
    // length is angle, all in one Vector3 - self-normalizing internally.
    Vector3f v(0.0f, 0.3f, 0.4f); // length 0.5
    Quaternion q;
    q.from_axis_angle(v);

    Vector3f back;
    q.to_axis_angle(back);

    REQUIRE(back.x == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(back.y == Catch::Approx(0.3f).margin(1e-5f));
    REQUIRE(back.z == Catch::Approx(0.4f).margin(1e-5f));
}

TEST_CASE("Quaternion to_axis_angle wraps a 350 degree rotation to -10 degrees", "[quaternion][axis_angle]") {
    // The real, load-bearing quirk: to_axis_angle wraps its angle to
    // (-pi, pi], reused directly from copter-rust's own COP-007
    // investigation (and independently re-verified against the real
    // upstream `wrap_PI(2*atan2(l,q1))` formula here). Every real caller
    // in the attitude controller treats this as an error to drive to
    // zero, so the unwrapped +350 degrees would command a nearly-full
    // turn instead of a small -10 degree correction.
    const float long_way = radians(350.0f);
    Quaternion q;
    q.from_axis_angle(Vector3f(0.0f, 0.0f, 1.0f), long_way);

    Vector3f back;
    q.to_axis_angle(back);

    const float expected = radians(-10.0f);
    REQUIRE(back.z == Catch::Approx(expected).margin(1e-4f));
    REQUIRE(back.x == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(back.y == Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("Quaternion built via from_axis_angle rotates a vector matching an independent rotation-matrix reference", "[quaternion][axis_angle]") {
    // rotate() and operator*(Vector3) are both written as upstream writes
    // them - inlined cross products, not a rotation matrix - so they are
    // algebraically equal to a matrix-based rotation but not bit-equal to
    // one. This constructs quaternions via the new from_axis_angle and
    // checks operator*(Vector3) (the pre-existing body-vector-rotation
    // operator this ticket depends on, not something it builds) agrees
    // with an INDEPENDENT reference: this same quaternion's own
    // rotation_matrix() (a completely different formula) applied to the
    // vector. Reused methodology from copter-rust's own COP-007
    // (`rotating_a_vector_agrees_with_the_matrix`), which found this catches
    // a sign error in the inlined cross-product form.
    struct Case { Vector3f axis; float angle; };
    const Case cases[] = {
        {Vector3f(0.0f, 0.0f, 1.0f), 0.4f},
        {Vector3f(1.0f, 0.0f, 0.0f), 1.1f},
        {Vector3f(0.267261f, 0.534522f, 0.801784f), -0.9f},
        {Vector3f(0.0f, 1.0f, 0.0f), 2.6f},
    };
    const Vector3f vectors[] = {
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
        Vector3f(0.0f, 0.0f, -1.0f),
        Vector3f(0.3f, -0.7f, 0.2f),
    };
    for (const auto& c : cases) {
        Quaternion q;
        q.from_axis_angle(c.axis, c.angle);

        Matrix3f m;
        q.rotation_matrix(m);

        for (const auto& v : vectors) {
            Vector3f by_quat = q * v;
            Vector3f by_matrix = m * v;
            REQUIRE(by_quat.x == Catch::Approx(by_matrix.x).margin(1e-5f));
            REQUIRE(by_quat.y == Catch::Approx(by_matrix.y).margin(1e-5f));
            REQUIRE(by_quat.z == Catch::Approx(by_matrix.z).margin(1e-5f));
        }
    }
}

TEST_CASE("Quaternion from_axis_angle(axis, theta) does NOT normalize a deliberately non-unit axis", "[quaternion][axis_angle]") {
    // Real, disclosed upstream quirk: this overload trusts the caller
    // entirely ("axis must be a unit vector as there is no check for
    // length"). A non-unit axis must produce the literal unnormalized
    // arithmetic (q2=axis.x*st2 etc with the RAW axis components), not an
    // auto-normalized result - so this deliberately passes a length-2
    // axis and checks the answer against the literal formula rather than
    // against what a normalized axis would give.
    const Vector3f axis(0.0f, 0.0f, 2.0f); // deliberately non-unit (length 2)
    const float theta = 1.0f;
    Quaternion q;
    q.from_axis_angle(axis, theta);

    const float st2 = std::sin(0.5f * theta);
    const float expected_q1 = std::cos(0.5f * theta);
    const float expected_q4 = axis.z * st2; // 2 * st2, NOT normalized to 1 * st2

    REQUIRE(q.q1 == Catch::Approx(expected_q1).margin(1e-6f));
    REQUIRE(q.q2 == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(q.q3 == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(q.q4 == Catch::Approx(expected_q4).margin(1e-6f));
    // Sanity: this is NOT what the normalized axis would have produced.
    REQUIRE(q.q4 != Catch::Approx(st2).margin(1e-6f));
    // And the resulting quaternion is consequently not unit length either -
    // a direct, visible symptom of the non-normalized input.
    REQUIRE_FALSE(q.is_unit_length());
}

TEST_CASE("Quaternion from_axis_angle resets to the identity for a zero angle, both overloads", "[quaternion][axis_angle]") {
    Quaternion from_axis;
    from_axis = Quaternion(0.5f, 0.5f, 0.5f, 0.5f); // start non-identity
    from_axis.from_axis_angle(Vector3f(0.0f, 0.0f, 1.0f), 0.0f);
    REQUIRE(from_axis.q1 == 1.0f);
    REQUIRE(from_axis.q2 == 0.0f);
    REQUIRE(from_axis.q3 == 0.0f);
    REQUIRE(from_axis.q4 == 0.0f);

    Quaternion from_vector(0.5f, 0.5f, 0.5f, 0.5f);
    from_vector.from_axis_angle(Vector3f(0.0f, 0.0f, 0.0f)); // zero-length rotation vector
    REQUIRE(from_vector.q1 == 1.0f);
    REQUIRE(from_vector.q2 == 0.0f);
    REQUIRE(from_vector.q3 == 0.0f);
    REQUIRE(from_vector.q4 == 0.0f);
}

TEST_CASE("Quaternion rotate(Vector3) and operator*(Vector3) are genuinely distinct operations", "[quaternion][axis_angle]") {
    // rotate(v) composes THIS quaternion with the delta rotation for v
    // (this = this * from_axis_angle(v)); operator*(Vector3) rotates an
    // EXTERNAL vector by this quaternion and leaves this quaternion
    // untouched. A test that conflated the two - e.g. by comparing
    // q.rotate(v) as though it returned a rotated vector - would either
    // fail to compile (rotate returns void) or, if adapted to compare
    // component-by-component against a Vector3, would fail outright since
    // rotate mutates a QUATERNION's four components while operator*
    // produces a rotated VECTOR's three. This test instead demonstrates
    // the two really do different jobs on the same delta: rotate(v)
    // advances q's own orientation, while q * v rotates v through q's
    // *original* orientation and never changes q.
    Quaternion q; // identity
    const Vector3f delta(0.0f, 0.0f, radians(90.0f)); // 90 degrees about +Z, as a rotation vector
    const Vector3f probe(1.0f, 0.0f, 0.0f);

    // operator*(Vector3) with the ORIGINAL identity q: no rotation at all.
    Vector3f unrotated_probe = q * probe;
    REQUIRE(unrotated_probe.x == Catch::Approx(probe.x).margin(1e-6f));
    REQUIRE(unrotated_probe.y == Catch::Approx(probe.y).margin(1e-6f));
    REQUIRE(unrotated_probe.z == Catch::Approx(probe.z).margin(1e-6f));

    // rotate(delta) advances q ITSELF by 90 degrees about +Z - q is no
    // longer the identity, and no vector was returned or rotated.
    q.rotate(delta);
    REQUIRE_FALSE(q.q1 == Catch::Approx(1.0f).margin(1e-6f));

    // NOW operator*(Vector3) on the SAME probe, through the UPDATED q,
    // gives the actual 90-degree-about-Z rotation of probe - confirmed
    // against the independent rotation-matrix reference, same methodology
    // as the earlier agreement test.
    Vector3f rotated_probe = q * probe;
    Matrix3f m;
    q.rotation_matrix(m);
    Vector3f expected = m * probe;
    REQUIRE(rotated_probe.x == Catch::Approx(expected.x).margin(1e-5f));
    REQUIRE(rotated_probe.y == Catch::Approx(expected.y).margin(1e-5f));
    REQUIRE(rotated_probe.z == Catch::Approx(expected.z).margin(1e-5f));
    // And this is NOT the same as calling operator*(Vector3) with the
    // ORIGINAL (pre-rotate) q would have given - the two really are
    // different operations, not two names for the same one.
    REQUIRE_FALSE(rotated_probe.x == Catch::Approx(unrotated_probe.x).margin(1e-3f));
}
