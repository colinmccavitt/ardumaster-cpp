// Tests for Vector2<T>'s geometry helper family (CPP-006 completion pass).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/vector2.hpp>

using namespace fwcpp::math;

TEST_CASE("Vector2::perpendicular picks the side maximizing distance from origin", "[vector2][geometry]") {
    Vector2f v1(1.0f, 0.0f); // along x-axis
    Vector2f pos_delta(0.0f, 5.0f); // offset toward +y
    Vector2f p = Vector2f::perpendicular(pos_delta, v1);
    // Both (0,1) and (0,-1) are perpendicular to (1,0); the one maximizing
    // dot with pos_delta=(0,5) is (0,1).
    REQUIRE(p.y > 0.0f);
}

TEST_CASE("Vector2::closest_point (v,w) clamps to segment endpoints outside [0,1]", "[vector2][geometry]") {
    Vector2f v(0.0f, 0.0f);
    Vector2f w(10.0f, 0.0f);
    REQUIRE(Vector2f::closest_point(Vector2f(-5.0f, 3.0f), v, w) == v); // t<0 -> v
    REQUIRE(Vector2f::closest_point(Vector2f(15.0f, 3.0f), v, w) == w); // t>1 -> w
    REQUIRE(Vector2f::closest_point(Vector2f(5.0f, 3.0f), v, w) == Vector2f(5.0f, 0.0f)); // midpoint projection
}

TEST_CASE("Vector2::closest_point (p,w) is the simplification with v=(0,0)", "[vector2][geometry]") {
    Vector2f w(10.0f, 0.0f);
    REQUIRE(Vector2f::closest_point(Vector2f(5.0f, 3.0f), w) == Vector2f(5.0f, 0.0f));
    REQUIRE(Vector2f::closest_point(Vector2f(-5.0f, 3.0f), w) == Vector2f(0.0f, 0.0f));
}

TEST_CASE("Vector2::closest_distance_between_line_and_point matches the perpendicular distance", "[vector2][geometry]") {
    Vector2f w1(0.0f, 0.0f);
    Vector2f w2(10.0f, 0.0f);
    float d = Vector2f::closest_distance_between_line_and_point(w1, w2, Vector2f(5.0f, 3.0f));
    REQUIRE(d == Catch::Approx(3.0f));
}

TEST_CASE("closest_distance_between_lines_squared is an endpoint-only approximation, not true segment distance", "[vector2][geometry]") {
    // Upstream's own comment calls this an approximation: it checks only
    // the 4 (endpoint, opposite-segment) distances, never the segments'
    // actual interior crossing point. For two segments that cross well
    // away from any endpoint, this DELIBERATELY does not report 0, even
    // though the true minimum distance between the segments is 0 at the
    // crossing point. Verified by hand: each of a1/a2/b1/b2 sits exactly
    // 5 units from the OTHER segment here (perpendicular crossing at the
    // origin, endpoints all 5 units out), so MIN of the four is 25
    // (squared) - not zero. Confirmed this is upstream's actual formula
    // by re-reading the source directly rather than trusting an initial
    // (wrong) assumption that it returns the true geometric minimum.
    Vector2f a1(-5.0f, 0.0f), a2(5.0f, 0.0f);
    Vector2f b1(0.0f, -5.0f), b2(0.0f, 5.0f);
    float d2 = Vector2f::closest_distance_between_lines_squared(a1, a2, b1, b2);
    REQUIRE(d2 == Catch::Approx(25.0f).margin(0.01f));
}

TEST_CASE("closest_distance_between_lines_squared is genuinely zero when an endpoint touches the other segment", "[vector2][geometry]") {
    // This is the case the approximation DOES handle correctly: one
    // segment's endpoint lying exactly on the other segment.
    Vector2f a1(-5.0f, 0.0f), a2(5.0f, 0.0f);
    Vector2f b1(0.0f, 0.0f), b2(0.0f, 5.0f); // b1 lies exactly on segment a1-a2
    float d2 = Vector2f::closest_distance_between_lines_squared(a1, a2, b1, b2);
    REQUIRE(d2 == Catch::Approx(0.0f).margin(0.01f));
}

TEST_CASE("Vector2::segment_intersection finds the crossing point of two segments", "[vector2][geometry]") {
    Vector2f intersection;
    bool hit = Vector2f::segment_intersection(
        Vector2f(-5.0f, 0.0f), Vector2f(5.0f, 0.0f),
        Vector2f(0.0f, -5.0f), Vector2f(0.0f, 5.0f),
        intersection);
    REQUIRE(hit);
    REQUIRE(intersection.x == Catch::Approx(0.0f));
    REQUIRE(intersection.y == Catch::Approx(0.0f));
}

TEST_CASE("Vector2::segment_intersection returns false for non-intersecting segments", "[vector2][geometry]") {
    Vector2f intersection;
    bool hit = Vector2f::segment_intersection(
        Vector2f(0.0f, 0.0f), Vector2f(1.0f, 0.0f),
        Vector2f(0.0f, 5.0f), Vector2f(1.0f, 5.0f),
        intersection);
    REQUIRE_FALSE(hit);
}

TEST_CASE("Vector2::segment_intersection returns false for parallel segments", "[vector2][geometry]") {
    Vector2f intersection;
    bool hit = Vector2f::segment_intersection(
        Vector2f(0.0f, 0.0f), Vector2f(10.0f, 0.0f),
        Vector2f(0.0f, 1.0f), Vector2f(10.0f, 1.0f),
        intersection);
    REQUIRE_FALSE(hit);
}

TEST_CASE("Vector2::circle_segment_intersection for a pass-through segment - upstream's own doc/code mismatch", "[vector2][geometry]") {
    // Upstream's function-level comment claims this returns the
    // "intersection closest to seg_start", and an inline comment at the
    // t1 check claims "t1 is the intersection, and it is closer than t2
    // (since t1 uses -b - discriminant)". Neither claim matches the
    // actual formula: t1 = (-b + sqrt(delta))/2a, t2 = (-b - sqrt(delta))/2a.
    // Since a > 0 always (sum of squares) and sqrt(delta) >= 0, t1 >= t2
    // ALWAYS - t1 is the LARGER parameter, i.e. the FARTHER point from
    // seg_start, not the closer one the docs describe. The code checks t1
    // first, so for a segment that fully passes through the circle (both
    // roots in [0,1]) it returns the FAR/exit intersection, not the near
    // one - confirmed by hand-computing this exact geometry (a=400,
    // b=-400, c=91, delta=14400, t1=0.65 -> x=3.0, t2=0.35 -> x=-3.0) and
    // by re-reading the upstream source line by line rather than trusting
    // its own comments. Reproduced as upstream actually behaves (ADR-0007:
    // fix bugs, not documentation that happens to disagree with shipped
    // behavior - other code may already depend on the real behavior).
    Vector2f intersection;
    bool hit = Vector2f::circle_segment_intersection(
        Vector2f(-10.0f, 0.0f), Vector2f(10.0f, 0.0f),
        Vector2f(0.0f, 0.0f), 3.0f, intersection);
    REQUIRE(hit);
    REQUIRE(intersection.x == Catch::Approx(3.0f).margin(0.01f)); // the FAR point, not -3.0
}

TEST_CASE("Vector2::circle_segment_intersection finds the only root when the segment ends inside the circle", "[vector2][geometry]") {
    // Poke case (upstream's own terminology): t1 in range, t2 not - no
    // ambiguity between near/far roots since only one is a valid hit.
    Vector2f intersection;
    bool hit = Vector2f::circle_segment_intersection(
        Vector2f(-10.0f, 0.0f), Vector2f(0.0f, 0.0f), // ends exactly at the circle center
        Vector2f(0.0f, 0.0f), 3.0f, intersection);
    REQUIRE(hit);
    REQUIRE(intersection.x == Catch::Approx(-3.0f).margin(0.01f)); // the only valid entry point
}

TEST_CASE("Vector2::circle_segment_intersection returns false when the segment misses the circle", "[vector2][geometry]") {
    Vector2f intersection;
    bool hit = Vector2f::circle_segment_intersection(
        Vector2f(-10.0f, 100.0f), Vector2f(10.0f, 100.0f), // far from the circle
        Vector2f(0.0f, 0.0f), 3.0f, intersection);
    REQUIRE_FALSE(hit);
}

TEST_CASE("Vector2::point_on_segment recognizes a collinear point within bounds", "[vector2][geometry]") {
    REQUIRE(Vector2f::point_on_segment(Vector2f(5.0f, 0.0f), Vector2f(0.0f, 0.0f), Vector2f(10.0f, 0.0f)));
    REQUIRE_FALSE(Vector2f::point_on_segment(Vector2f(15.0f, 0.0f), Vector2f(0.0f, 0.0f), Vector2f(10.0f, 0.0f))); // out of bounds
    REQUIRE_FALSE(Vector2f::point_on_segment(Vector2f(5.0f, 1.0f), Vector2f(0.0f, 0.0f), Vector2f(10.0f, 0.0f))); // off the line
}

TEST_CASE("Vector2::point_on_segment handles a vertical segment (zero run)", "[vector2][geometry]") {
    REQUIRE(Vector2f::point_on_segment(Vector2f(0.0f, 5.0f), Vector2f(0.0f, 0.0f), Vector2f(0.0f, 10.0f)));
    REQUIRE_FALSE(Vector2f::point_on_segment(Vector2f(1.0f, 5.0f), Vector2f(0.0f, 0.0f), Vector2f(0.0f, 10.0f)));
}

TEST_CASE("Vector2 tofloat/todouble convert precision without changing value", "[vector2][geometry]") {
    Vector2d d(1.5, 2.5);
    Vector2f f = d.tofloat();
    REQUIRE(f == Vector2f(1.5f, 2.5f));
    Vector2d back = f.todouble();
    REQUIRE(back == Vector2d(1.5, 2.5));
}
