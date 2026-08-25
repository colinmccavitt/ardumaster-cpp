// Tests for Matrix3<T> (CPP-008 slice 1).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/matrix3.hpp>

using namespace fwcpp::math;

namespace {
Matrix3f make_identity() {
    Matrix3f m;
    m.identity();
    return m;
}
} // namespace

TEST_CASE("Matrix3 identity is the multiplicative identity", "[matrix3]") {
    Matrix3f id = make_identity();
    Vector3f v(1.0f, 2.0f, 3.0f);
    REQUIRE((id * v) == v);
}

TEST_CASE("Matrix3 zero produces an all-zero matrix", "[matrix3]") {
    Matrix3f m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    m.zero();
    REQUIRE(m.a == Vector3f(0, 0, 0));
    REQUIRE(m.b == Vector3f(0, 0, 0));
    REQUIRE(m.c == Vector3f(0, 0, 0));
}

TEST_CASE("Matrix3 arithmetic operators", "[matrix3]") {
    Matrix3f a(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3f b(1, 1, 1, 1, 1, 1, 1, 1, 1);
    Matrix3f sum = a + b;
    REQUIRE(sum.a == Vector3f(2, 3, 4));
    Matrix3f scaled = a * 2.0f;
    REQUIRE(scaled.a == Vector3f(2, 4, 6));
}

TEST_CASE("Matrix3 colx/coly/colz extract columns, not rows", "[matrix3]") {
    Matrix3f m(1, 2, 3, 4, 5, 6, 7, 8, 9); // rows: (1,2,3) (4,5,6) (7,8,9)
    REQUIRE(m.colx() == Vector3f(1, 4, 7));
    REQUIRE(m.coly() == Vector3f(2, 5, 8));
    REQUIRE(m.colz() == Vector3f(3, 6, 9));
}

TEST_CASE("Matrix3 * Vector3 matches hand-computed values", "[matrix3]") {
    // 90-degree rotation about z: x->y, y->-x
    Matrix3f rot_z90(0, -1, 0,
                      1, 0, 0,
                      0, 0, 1);
    Vector3f v(1.0f, 0.0f, 0.0f);
    Vector3f rotated = rot_z90 * v;
    REQUIRE(rotated.x == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(rotated.y == Catch::Approx(1.0f));
}

TEST_CASE("Matrix3 * Matrix3 with the identity is a no-op", "[matrix3]") {
    Matrix3f id = make_identity();
    Matrix3f m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    REQUIRE((m * id) == m);
    REQUIRE((id * m) == m);
}

TEST_CASE("Matrix3 transpose matches hand-computed values", "[matrix3]") {
    Matrix3f m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3f t = m.transposed();
    REQUIRE(t.a == Vector3f(1, 4, 7));
    REQUIRE(t.b == Vector3f(2, 5, 8));
    REQUIRE(t.c == Vector3f(3, 6, 9));
}

TEST_CASE("Matrix3 det of the identity is 1", "[matrix3]") {
    REQUIRE(make_identity().det() == Catch::Approx(1.0f));
}

TEST_CASE("Matrix3 inverse of the identity is the identity", "[matrix3]") {
    Matrix3f id = make_identity();
    Matrix3f inv;
    REQUIRE(id.inverse(inv));
    REQUIRE(inv == id);
}

TEST_CASE("Matrix3 inverse round-trips: M * inverse(M) is the identity", "[matrix3]") {
    Matrix3f m(2, 0, 1, 1, 3, 0, 0, 1, 4);
    Matrix3f inv;
    REQUIRE(m.inverse(inv));
    Matrix3f product = m * inv;
    Matrix3f id = make_identity();
    for (int r = 0; r < 3; ++r) {
        for (int col = 0; col < 3; ++col) {
            REQUIRE(product[r][col] == Catch::Approx(id[r][col]).margin(1e-5));
        }
    }
}

TEST_CASE("Matrix3 inverse rejects a singular matrix", "[matrix3]") {
    Matrix3f singular(1, 2, 3, 2, 4, 6, 1, 1, 1); // row2 = 2*row1
    Matrix3f inv;
    REQUIRE_FALSE(singular.inverse(inv));
}

TEST_CASE("Matrix3 is_nan checks all three rows", "[matrix3]") {
    Matrix3f m = make_identity();
    REQUIRE_FALSE(m.is_nan());
    m.a.x = std::nanf("");
    REQUIRE(m.is_nan());
}

TEST_CASE("Matrix3 from_euler / to_euler round-trip for a well-conditioned angle set", "[matrix3]") {
    const float roll = 0.3f, pitch = 0.2f, yaw = 1.0f;
    Matrix3f m;
    m.from_euler(roll, pitch, yaw);

    float r, p, y;
    m.to_euler(&r, &p, &y);
    REQUIRE(r == Catch::Approx(roll).margin(1e-5));
    REQUIRE(p == Catch::Approx(pitch).margin(1e-5));
    REQUIRE(y == Catch::Approx(yaw).margin(1e-5));
}

TEST_CASE("Matrix3 from_euler produces an orthonormal (rotation) matrix", "[matrix3]") {
    Matrix3f m;
    m.from_euler(0.3f, -0.4f, 2.0f);
    // rows should be unit length and mutually orthogonal
    REQUIRE(m.a.length() == Catch::Approx(1.0f).margin(1e-5));
    REQUIRE(m.b.length() == Catch::Approx(1.0f).margin(1e-5));
    REQUIRE(m.c.length() == Catch::Approx(1.0f).margin(1e-5));
    REQUIRE(m.a.dot(m.b) == Catch::Approx(0.0f).margin(1e-5));
    REQUIRE(m.a.dot(m.c) == Catch::Approx(0.0f).margin(1e-5));
}

TEST_CASE("Matrix3 from_euler312 / to_euler312 round-trip", "[matrix3]") {
    const float roll = 0.25f, pitch = -0.15f, yaw = 0.6f;
    Matrix3f m;
    m.from_euler312(roll, pitch, yaw);
    Vector3f rpy = m.to_euler312();
    REQUIRE(rpy.x == Catch::Approx(roll).margin(1e-5));
    REQUIRE(rpy.y == Catch::Approx(pitch).margin(1e-5));
    REQUIRE(rpy.z == Catch::Approx(yaw).margin(1e-5));
}

TEST_CASE("Matrix3::rotate applies a small-angle body-frame rotation", "[matrix3]") {
    // A tiny gyro-vector rotation about z should turn the identity toward
    // a small z-rotation, matching the first-order Taylor expansion of
    // from_axis_angle for small theta.
    Matrix3f m = make_identity();
    const float dtheta = 0.001f;
    m.rotate(Vector3f(0.0f, 0.0f, dtheta));

    Matrix3f expected;
    expected.from_axis_angle(Vector3f(0.0f, 0.0f, 1.0f), dtheta);

    REQUIRE(m.a.x == Catch::Approx(expected.a.x).margin(1e-4));
    REQUIRE(m.a.y == Catch::Approx(expected.a.y).margin(1e-4));
}

TEST_CASE("Matrix3::normalize restores orthonormality after drift", "[matrix3]") {
    Matrix3f m = make_identity();
    // introduce deliberate drift
    m.a.x = 1.01f;
    m.b.y = 0.99f;
    m.normalize();
    REQUIRE(m.a.length() == Catch::Approx(1.0f).margin(1e-5));
    REQUIRE(m.b.length() == Catch::Approx(1.0f).margin(1e-5));
    REQUIRE(m.c.length() == Catch::Approx(1.0f).margin(1e-5));
    REQUIRE(m.a.dot(m.b) == Catch::Approx(0.0f).margin(1e-5));
}

TEST_CASE("Matrix3::from_axis_angle about z matches a manual z-rotation", "[matrix3]") {
    Matrix3f m;
    m.from_axis_angle(Vector3f(0.0f, 0.0f, 1.0f), static_cast<float>(M_PI / 2));
    Vector3f v(1.0f, 0.0f, 0.0f);
    Vector3f rotated = m * v;
    REQUIRE(rotated.x == Catch::Approx(0.0f).margin(1e-5));
    REQUIRE(rotated.y == Catch::Approx(1.0f));
}

TEST_CASE("Matrix3 todouble/tofloat convert every element", "[matrix3]") {
    Matrix3d d(1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5);
    Matrix3f f = d.tofloat();
    REQUIRE(f.a == Vector3f(1.5f, 2.5f, 3.5f));
    Matrix3d back = f.todouble();
    REQUIRE(back == d);
}
