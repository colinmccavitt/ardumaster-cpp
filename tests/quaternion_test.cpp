// Tests for QuaternionT<T> (CPP-009 slice 1).

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

TEST_CASE("Quaternion normalize on a zero quaternion leaves it unchanged (no crash)", "[quaternion]") {
    Quaternion q;
    q.zero();
    q.normalize();
    REQUIRE(q.is_zero()); // still zero - no report wired yet (CPP-005), but no crash either
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
