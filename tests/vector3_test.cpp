// Tests for Vector3<T> core algebra (CPP-007 slice 1).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/vector3.hpp>

using namespace fwcpp::math;

TEST_CASE("Vector3 arithmetic operators", "[vector3]") {
    Vector3f a(1.0f, 2.0f, 3.0f);
    Vector3f b(4.0f, 5.0f, 6.0f);
    REQUIRE((a + b) == Vector3f(5.0f, 7.0f, 9.0f));
    REQUIRE((a - b) == Vector3f(-3.0f, -3.0f, -3.0f));
    REQUIRE((-a) == Vector3f(-1.0f, -2.0f, -3.0f));
    REQUIRE((a * 2.0f) == Vector3f(2.0f, 4.0f, 6.0f));
    REQUIRE((b / 2.0f) == Vector3f(2.0f, 2.5f, 3.0f));
}

TEST_CASE("Vector3 from Vector2 plus z", "[vector3]") {
    Vector2f xy(1.0f, 2.0f);
    Vector3f v(xy, 3.0f);
    REQUIRE(v == Vector3f(1.0f, 2.0f, 3.0f));
}

TEST_CASE("Vector3 non-uniform scaling via operator*=(Vector3)", "[vector3]") {
    Vector3f a(2.0f, 3.0f, 4.0f);
    a *= Vector3f(2.0f, 0.5f, 1.0f);
    REQUIRE(a == Vector3f(4.0f, 1.5f, 4.0f));
}

TEST_CASE("Vector3 dot and cross products match hand-computed values", "[vector3]") {
    Vector3f a(1.0f, 0.0f, 0.0f);
    Vector3f b(0.0f, 1.0f, 0.0f);
    REQUIRE(a.dot(b) == Catch::Approx(0.0f)); // perpendicular unit vectors
    Vector3f c = a.cross(b);
    REQUIRE(c == Vector3f(0.0f, 0.0f, 1.0f)); // right-hand rule: x cross y = z

    Vector3f d(1.0f, 2.0f, 3.0f);
    Vector3f e(4.0f, 5.0f, 6.0f);
    REQUIRE(d.dot(e) == Catch::Approx(32.0f)); // 4+10+18
}

TEST_CASE("Vector3 length and length_squared", "[vector3]") {
    Vector3f v(2.0f, 3.0f, 6.0f); // classic 2-3-6-7 Pythagorean-like example
    REQUIRE(v.length_squared() == Catch::Approx(49.0f));
    REQUIRE(v.length() == Catch::Approx(7.0f));
}

TEST_CASE("Vector3 limit_length_xy only scales x,y and leaves z untouched", "[vector3]") {
    Vector3f v(3.0f, 4.0f, 100.0f); // xy length 5, huge z
    REQUIRE(v.limit_length_xy(2.5f));
    REQUIRE(v.z == Catch::Approx(100.0f)); // untouched
    REQUIRE(std::sqrt(v.x * v.x + v.y * v.y) == Catch::Approx(2.5f));

    Vector3f under(1.0f, 0.0f, 999.0f);
    REQUIRE_FALSE(under.limit_length_xy(10.0f));
}

TEST_CASE("Vector3 normalize produces a unit vector", "[vector3]") {
    Vector3f v(2.0f, 3.0f, 6.0f);
    Vector3f n = v.normalized();
    REQUIRE(n.length() == Catch::Approx(1.0f));
}

TEST_CASE("Vector3 is_zero: float/double epsilon vs integral exact", "[vector3]") {
    REQUIRE(Vector3f(0.0f, 0.0f, 0.0f).is_zero());
    REQUIRE_FALSE(Vector3f(0.1f, 0.0f, 0.0f).is_zero());
    REQUIRE(Vector3f(1e-10f, 0.0f, 0.0f).is_zero()); // D-003 territory, same as Vector2/scalar

    REQUIRE(Vector3i(0, 0, 0).is_zero());
    REQUIRE_FALSE(Vector3i(0, 0, 1).is_zero());
}

TEST_CASE("Vector3 is_nan and is_inf check every component", "[vector3]") {
    REQUIRE(Vector3f(0.0f, std::nanf(""), 0.0f).is_nan());
    REQUIRE_FALSE(Vector3f(0.0f, 0.0f, 0.0f).is_nan());
    REQUIRE(Vector3f(0.0f, 0.0f, INFINITY).is_inf());
}

TEST_CASE("Vector3::angle returns 0 for both parallel and antiparallel vectors", "[vector3]") {
    // Documents the upstream inconsistency with Vector2::angle (which
    // returns pi for antiparallel) - see vector3.hpp's file banner.
    Vector3f a(1.0f, 0.0f, 0.0f);
    Vector3f parallel(2.0f, 0.0f, 0.0f);
    Vector3f antiparallel(-1.0f, 0.0f, 0.0f);
    Vector3f perp(0.0f, 1.0f, 0.0f);

    REQUIRE(a.angle(parallel) == Catch::Approx(0.0f));
    REQUIRE(a.angle(antiparallel) == Catch::Approx(0.0f)); // NOT pi, unlike Vector2
    REQUIRE(a.angle(perp) == Catch::Approx(static_cast<float>(M_PI / 2)));
}

TEST_CASE("Vector3::rotate_xy rotates in the xy plane, leaving z untouched", "[vector3]") {
    Vector3f v(1.0f, 0.0f, 42.0f);
    v.rotate_xy(static_cast<float>(M_PI / 2));
    REQUIRE(v.x == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(v.y == Catch::Approx(1.0f));
    REQUIRE(v.z == Catch::Approx(42.0f));
}

TEST_CASE("Vector3::offset_bearing moves along bearing/pitch by distance", "[vector3]") {
    Vector3f v(0.0f, 0.0f, 0.0f);
    // bearing 0, pitch 0: pure +x
    v.offset_bearing(0.0f, 0.0f, 10.0f);
    REQUIRE(v.x == Catch::Approx(10.0f));
    REQUIRE(v.y == Catch::Approx(0.0f).margin(1e-4));
    REQUIRE(v.z == Catch::Approx(0.0f).margin(1e-4));

    Vector3f v2(0.0f, 0.0f, 0.0f);
    // pitch 90: straight down (+z in NED convention)
    v2.offset_bearing(0.0f, 90.0f, 5.0f);
    REQUIRE(v2.z == Catch::Approx(5.0f));
}

TEST_CASE("Vector3::rfu_to_frd swaps x/y and negates z", "[vector3]") {
    Vector3f v(1.0f, 2.0f, 3.0f);
    REQUIRE(v.rfu_to_frd() == Vector3f(2.0f, 1.0f, -3.0f));
}

TEST_CASE("Vector3::perpendicular returns the component of p1 orthogonal to v1", "[vector3]") {
    Vector3f p1(3.0f, 4.0f, 0.0f);
    Vector3f v1(1.0f, 0.0f, 0.0f); // x-axis
    Vector3f perp = Vector3f::perpendicular(p1, v1);
    // parallel component (along x) removed, leaving only the y component
    REQUIRE(perp == Vector3f(0.0f, 4.0f, 0.0f));
}

TEST_CASE("Vector3::perpendicular returns p1 unchanged when already perpendicular to v1", "[vector3]") {
    Vector3f p1(0.0f, 4.0f, 0.0f);
    Vector3f v1(1.0f, 0.0f, 0.0f);
    REQUIRE(Vector3f::perpendicular(p1, v1) == p1);
}

TEST_CASE("Vector3 tofloat/todouble convert precision without changing value", "[vector3]") {
    Vector3d d(1.5, 2.5, 3.5);
    Vector3f f = d.tofloat();
    REQUIRE(f == Vector3f(1.5f, 2.5f, 3.5f));
    Vector3d back = f.todouble();
    REQUIRE(back == Vector3d(1.5, 2.5, 3.5));
}
