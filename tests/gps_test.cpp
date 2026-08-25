// Tests for fwcpp::gps::Gps (CPP-033) - the minimal SITL GPS backend port.
// See gps.hpp's own file banner for exactly what upstream behavior this
// reproduces (AP_GPS_SITL::read() + GPS_Backend::velocity_to_speed_course())
// and what's excluded.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/gps/gps.hpp>

using namespace fwcpp::gps;
using fwcpp::math::Vector3f;

// ---------------------------------------------------------------------
// 200ms rate limit
// ---------------------------------------------------------------------

TEST_CASE("Gps::update is a no-op before the first 200ms elapse (matches upstream's cold-start behavior)", "[gps]") {
    Gps gps;
    // Default GpsSample has has_fix=false - confirm it starts that way.
    REQUIRE_FALSE(gps.sample().has_fix);

    gps.update(Vector3f(5.0f, 0.0f, 0.0f), 0);
    REQUIRE_FALSE(gps.sample().has_fix); // now_ms(0) - last_update_ms_(0) = 0 < 200 -> gated

    gps.update(Vector3f(5.0f, 0.0f, 0.0f), 199);
    REQUIRE_FALSE(gps.sample().has_fix); // still gated, exactly matching upstream's strict "<" comparison
}

TEST_CASE("Gps::update fires exactly at the 200ms boundary, not before", "[gps]") {
    Gps gps;
    gps.update(Vector3f(5.0f, 0.0f, 0.0f), 199);
    REQUIRE_FALSE(gps.sample().has_fix);

    gps.update(Vector3f(5.0f, 0.0f, 0.0f), 200);
    REQUIRE(gps.sample().has_fix); // 200 - 0 = 200, not < 200 -> fires
    REQUIRE(gps.sample().last_fix_time_ms == 200);
}

TEST_CASE("Gps::update actually gates repeat calls within 200ms of the last real update", "[gps]") {
    Gps gps;
    gps.update(Vector3f(10.0f, 0.0f, 0.0f), 200);
    REQUIRE(gps.sample().last_fix_time_ms == 200);
    REQUIRE(gps.sample().ground_speed_ms == Catch::Approx(10.0f));

    // A call 150ms later (< 200ms since the last real update) must be a
    // complete no-op - even though the velocity input changed, nothing in
    // sample() should move.
    gps.update(Vector3f(999.0f, 999.0f, 999.0f), 350);
    REQUIRE(gps.sample().last_fix_time_ms == 200);
    REQUIRE(gps.sample().ground_speed_ms == Catch::Approx(10.0f));
    REQUIRE(gps.sample().velocity_ned.x == Catch::Approx(10.0f));

    // Exactly 200ms after the last REAL update (200 + 200 = 400) fires again.
    gps.update(Vector3f(999.0f, 999.0f, 999.0f), 400);
    REQUIRE(gps.sample().last_fix_time_ms == 400);
    REQUIRE(gps.sample().velocity_ned.x == Catch::Approx(999.0f));
}

TEST_CASE("Gps::update fires again every 200ms on a steady tick stream", "[gps]") {
    Gps gps;
    int fixes = 0;
    std::uint32_t last_seen = 0xFFFFFFFFU;
    for (std::uint32_t t = 0; t <= 1000; t += 20) {
        gps.update(Vector3f(1.0f, 0.0f, 0.0f), t);
        if (gps.sample().has_fix && gps.sample().last_fix_time_ms != last_seen) {
            last_seen = gps.sample().last_fix_time_ms;
            ++fixes;
        }
    }
    // Fixes land at t=200,400,600,800,1000 - exactly 5 over a 1000ms span of
    // 20ms ticks, matching the real 5Hz GPS update rate.
    REQUIRE(fixes == 5);
}

// ---------------------------------------------------------------------
// ground_speed / ground_course - hand-computed atan2/length values
// ---------------------------------------------------------------------

TEST_CASE("velocity_to_speed_course: due north velocity -> course 0deg", "[gps]") {
    Gps gps;
    gps.update(Vector3f(10.0f, 0.0f, 0.0f), 200); // N=10, E=0
    REQUIRE(gps.sample().ground_course_deg == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(gps.sample().ground_speed_ms == Catch::Approx(10.0f));
}

TEST_CASE("velocity_to_speed_course: due east velocity -> course 90deg", "[gps]") {
    Gps gps;
    gps.update(Vector3f(0.0f, 10.0f, 0.0f), 200); // N=0, E=10
    REQUIRE(gps.sample().ground_course_deg == Catch::Approx(90.0f).margin(1e-3f));
    REQUIRE(gps.sample().ground_speed_ms == Catch::Approx(10.0f));
}

TEST_CASE("velocity_to_speed_course: due south velocity -> course 180deg", "[gps]") {
    Gps gps;
    gps.update(Vector3f(-10.0f, 0.0f, 0.0f), 200);
    REQUIRE(gps.sample().ground_course_deg == Catch::Approx(180.0f).margin(1e-3f));
    REQUIRE(gps.sample().ground_speed_ms == Catch::Approx(10.0f));
}

TEST_CASE("velocity_to_speed_course: due west velocity -> course 270deg (wrap_360 of a negative atan2)", "[gps]") {
    Gps gps;
    gps.update(Vector3f(0.0f, -10.0f, 0.0f), 200); // N=0, E=-10 -> atan2(-10,0) = -90deg -> wrap_360 -> 270deg
    REQUIRE(gps.sample().ground_course_deg == Catch::Approx(270.0f).margin(1e-3f));
    REQUIRE(gps.sample().ground_speed_ms == Catch::Approx(10.0f));
}

TEST_CASE("velocity_to_speed_course: diagonal NE velocity -> 45deg course, hypotenuse speed", "[gps]") {
    Gps gps;
    gps.update(Vector3f(10.0f, 10.0f, 0.0f), 200);
    REQUIRE(gps.sample().ground_course_deg == Catch::Approx(45.0f).margin(1e-3f));
    REQUIRE(gps.sample().ground_speed_ms == Catch::Approx(std::sqrt(200.0f)));
}

TEST_CASE("velocity_to_speed_course: ground_speed ignores the down component entirely (xy-only)", "[gps]") {
    Gps gps;
    // A large climb/descent rate (z) must not leak into ground_speed_ms -
    // velocity_to_speed_course() only ever looks at velocity.xy().
    gps.update(Vector3f(3.0f, 4.0f, 500.0f), 200);
    REQUIRE(gps.sample().ground_speed_ms == Catch::Approx(5.0f)); // 3-4-5 triangle, z excluded
}

// ---------------------------------------------------------------------
// Always-good-fix behavior (SITL's real, unconditional behavior)
// ---------------------------------------------------------------------

TEST_CASE("A successful update always reports num_sats=15, has_3d_fix=true, has_fix=true unconditionally", "[gps]") {
    Gps gps;
    // Even with a zero velocity (no motion at all) and no special setup,
    // upstream's SITL backend still reports a perfect fix - it never
    // simulates degraded fix quality.
    gps.update(Vector3f(0.0f, 0.0f, 0.0f), 200);
    REQUIRE(gps.sample().num_sats == 15);
    REQUIRE(gps.sample().has_3d_fix);
    REQUIRE(gps.sample().has_fix);
}

TEST_CASE("Repeated real updates keep reporting the same unconditional good fix", "[gps]") {
    Gps gps;
    for (std::uint32_t t = 200; t <= 1000; t += 200) {
        gps.update(Vector3f(1.0f, 1.0f, 0.0f), t);
        REQUIRE(gps.sample().num_sats == 15);
        REQUIRE(gps.sample().has_3d_fix);
        REQUIRE(gps.sample().has_fix);
    }
}

// ---------------------------------------------------------------------
// velocity_ned passthrough
// ---------------------------------------------------------------------

TEST_CASE("velocity_ned is passed through verbatim, all three axes, no transformation", "[gps]") {
    Gps gps;
    const Vector3f v(1.5f, -2.5f, 3.5f);
    gps.update(v, 200);
    REQUIRE(gps.sample().velocity_ned.x == Catch::Approx(1.5f));
    REQUIRE(gps.sample().velocity_ned.y == Catch::Approx(-2.5f));
    REQUIRE(gps.sample().velocity_ned.z == Catch::Approx(3.5f));
}

TEST_CASE("velocity_ned updates on each new real fix, not just the first", "[gps]") {
    Gps gps;
    gps.update(Vector3f(1.0f, 0.0f, 0.0f), 200);
    REQUIRE(gps.sample().velocity_ned.x == Catch::Approx(1.0f));

    gps.update(Vector3f(2.0f, 0.0f, 0.0f), 400);
    REQUIRE(gps.sample().velocity_ned.x == Catch::Approx(2.0f));
}
