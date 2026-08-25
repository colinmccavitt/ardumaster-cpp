// Tests for Vector2<T> core algebra (CPP-006 slice 1).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/vector2.hpp>

using namespace fwcpp::math;

TEST_CASE("Vector2 arithmetic operators", "[vector2]") {
    Vector2f a(1.0f, 2.0f);
    Vector2f b(3.0f, 4.0f);
    REQUIRE((a + b) == Vector2f(4.0f, 6.0f));
    REQUIRE((a - b) == Vector2f(-2.0f, -2.0f));
    REQUIRE((-a) == Vector2f(-1.0f, -2.0f));
    REQUIRE((a * 2.0f) == Vector2f(2.0f, 4.0f));
    REQUIRE((b / 2.0f) == Vector2f(1.5f, 2.0f));
}

TEST_CASE("Vector2 compound assignment matches the non-mutating equivalents", "[vector2]") {
    Vector2f a(1.0f, 2.0f);
    a += Vector2f(1.0f, 1.0f);
    REQUIRE(a == Vector2f(2.0f, 3.0f));
    a -= Vector2f(1.0f, 1.0f);
    REQUIRE(a == Vector2f(1.0f, 2.0f));
    a *= 3.0f;
    REQUIRE(a == Vector2f(3.0f, 6.0f));
    a /= 3.0f;
    REQUIRE(a == Vector2f(1.0f, 2.0f));
}

TEST_CASE("Vector2 dot and cross products match hand-computed values", "[vector2]") {
    Vector2f a(1.0f, 2.0f);
    Vector2f b(3.0f, 4.0f);
    REQUIRE(a.dot(b) == Catch::Approx(11.0f)); // 1*3 + 2*4
    REQUIRE((a * b) == Catch::Approx(11.0f));  // operator* is also dot
    REQUIRE((a % b) == Catch::Approx(-2.0f));  // 1*4 - 2*3
}

TEST_CASE("Vector2 length and length_squared", "[vector2]") {
    Vector2f v(3.0f, 4.0f);
    REQUIRE(v.length_squared() == Catch::Approx(25.0f));
    REQUIRE(v.length() == Catch::Approx(5.0f));
}

TEST_CASE("Vector2 limit_length only scales when over the limit", "[vector2]") {
    Vector2f v(3.0f, 4.0f); // length 5
    Vector2f under = v;
    REQUIRE_FALSE(under.limit_length(10.0f));
    REQUIRE(under == v);

    Vector2f over = v;
    REQUIRE(over.limit_length(2.5f));
    REQUIRE(over.length() == Catch::Approx(2.5f));
    // direction preserved
    REQUIRE(over.x / over.y == Catch::Approx(v.x / v.y));
}

TEST_CASE("Vector2 normalize produces a unit vector in the same direction", "[vector2]") {
    Vector2f v(3.0f, 4.0f);
    Vector2f n = v.normalized();
    REQUIRE(n.length() == Catch::Approx(1.0f));
    REQUIRE(n.x == Catch::Approx(0.6f));
    REQUIRE(n.y == Catch::Approx(0.8f));

    Vector2f v2(3.0f, 4.0f);
    v2.normalize();
    REQUIRE(v2 == n);
}

TEST_CASE("Vector2 is_zero: float/double use epsilon, integral types use exact comparison", "[vector2]") {
    REQUIRE(Vector2f(0.0f, 0.0f).is_zero());
    REQUIRE_FALSE(Vector2f(0.1f, 0.0f).is_zero());
    REQUIRE(Vector2f(1e-10f, 0.0f).is_zero()); // below FLT_EPSILON, D-003 territory again

    REQUIRE(Vector2i(0, 0).is_zero());
    REQUIRE_FALSE(Vector2i(1, 0).is_zero());
}

TEST_CASE("Vector2 is_nan and is_inf", "[vector2]") {
    REQUIRE(Vector2f(std::nanf(""), 0.0f).is_nan());
    REQUIRE_FALSE(Vector2f(0.0f, 0.0f).is_nan());
    REQUIRE(Vector2f(INFINITY, 0.0f).is_inf());
    REQUIRE_FALSE(Vector2f(0.0f, 0.0f).is_inf());
}

TEST_CASE("Vector2::angle(void) is the vector's own angle from (1,0)", "[vector2]") {
    REQUIRE(Vector2f(1.0f, 0.0f).angle() == Catch::Approx(0.0f));
    REQUIRE(Vector2f(0.0f, 1.0f).angle() == Catch::Approx(static_cast<float>(M_PI / 2)));
    REQUIRE(Vector2f(1.0f, 1.0f).angle() == Catch::Approx(static_cast<float>(M_PI / 4)));
}

TEST_CASE("Vector2::angle(other) is 0 for parallel and pi for antiparallel vectors", "[vector2]") {
    Vector2f a(1.0f, 0.0f);
    Vector2f parallel(2.0f, 0.0f);
    Vector2f antiparallel(-1.0f, 0.0f);
    Vector2f perpendicular(0.0f, 1.0f);

    REQUIRE(a.angle(parallel) == Catch::Approx(0.0f));
    REQUIRE(a.angle(antiparallel) == Catch::Approx(static_cast<float>(M_PI)));
    REQUIRE(a.angle(perpendicular) == Catch::Approx(static_cast<float>(M_PI / 2)));
}

TEST_CASE("Vector2::rotate by 90 degrees matches a hand-computed rotation", "[vector2]") {
    Vector2f v(1.0f, 0.0f);
    v.rotate(static_cast<float>(M_PI / 2));
    REQUIRE(v.x == Catch::Approx(0.0f).margin(1e-6));
    REQUIRE(v.y == Catch::Approx(1.0f));
}

TEST_CASE("Vector2::offset_bearing moves by distance along the given bearing", "[vector2]") {
    Vector2f v(0.0f, 0.0f);
    v.offset_bearing(90.0f, 10.0f); // bearing 90 = east = +x in this convention (cos(radians(90))=0, sin=1... check axis)
    // offset_bearing: x += cos(radians(bearing))*distance, y += sin(radians(bearing))*distance
    // bearing 0 -> x += distance (pure +x); bearing 90 -> y += distance (pure +y)
    REQUIRE(v.x == Catch::Approx(0.0f).margin(1e-4));
    REQUIRE(v.y == Catch::Approx(10.0f));
}

TEST_CASE("Vector2::project projects onto another vector", "[vector2]") {
    Vector2f v(3.0f, 4.0f);
    Vector2f onto_x(1.0f, 0.0f);
    Vector2f p = v.projected(onto_x);
    REQUIRE(p == Vector2f(3.0f, 0.0f));
}

TEST_CASE("Vector2::reflect reflects about the given normal", "[vector2]") {
    // Reflecting (1,1) about the x-axis normal (1,0) gives (1,-1)... actually
    // this reflect() implementation reflects ABOUT the projection axis (n),
    // not about the surface with normal n - verify against the formula
    // directly: reflect = 2*project(n) - original.
    Vector2f v(1.0f, 1.0f);
    Vector2f n(1.0f, 0.0f);
    Vector2f expected = v.projected(n) * 2.0f - v;
    Vector2f r = v;
    r.reflect(n);
    REQUIRE(r == expected);
}
