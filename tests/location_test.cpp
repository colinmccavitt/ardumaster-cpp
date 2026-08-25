// Tests for fwcpp::Location core geometry (CPP-011 slice 1).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/location.hpp>

using fwcpp::Location;

TEST_CASE("Location default constructor is all-zero", "[location]") {
    Location loc;
    REQUIRE(loc.lat == 0);
    REQUIRE(loc.lng == 0);
    REQUIRE(loc.alt == 0);
    REQUIRE(loc.get_alt_frame() == Location::AltFrame::ABSOLUTE);
}

TEST_CASE("Location constructor sets lat/lng/alt and the frame flags", "[location]") {
    Location loc(-353632621, 1491652374, 1000, Location::AltFrame::ABOVE_HOME);
    REQUIRE(loc.lat == -353632621);
    REQUIRE(loc.lng == 1491652374);
    REQUIRE(loc.alt == 1000);
    REQUIRE(loc.get_alt_frame() == Location::AltFrame::ABOVE_HOME);
    REQUIRE(loc.relative_alt == 1);
}

TEST_CASE("Location::get_alt_frame round-trips through set_alt_cm for every frame", "[location]") {
    Location loc;
    loc.set_alt_cm(500, Location::AltFrame::ABSOLUTE);
    REQUIRE(loc.get_alt_frame() == Location::AltFrame::ABSOLUTE);
    loc.set_alt_cm(500, Location::AltFrame::ABOVE_HOME);
    REQUIRE(loc.get_alt_frame() == Location::AltFrame::ABOVE_HOME);
    loc.set_alt_cm(500, Location::AltFrame::ABOVE_ORIGIN);
    REQUIRE(loc.get_alt_frame() == Location::AltFrame::ABOVE_ORIGIN);
    loc.set_alt_cm(500, Location::AltFrame::ABOVE_TERRAIN);
    REQUIRE(loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN);
    REQUIRE(loc.relative_alt == 1); // terrain rides on relative_alt, per upstream
}

TEST_CASE("Location::diff_longitude handles the common same-sign case", "[location]") {
    REQUIRE(Location::diff_longitude(100, 50) == 50);
    REQUIRE(Location::diff_longitude(-100, -50) == -50);
}

TEST_CASE("Location::diff_longitude wraps across the +-180e7 antimeridian", "[location]") {
    // lon1 near +180e7, lon2 near -180e7: the short way around is small,
    // not the naive (huge) subtraction.
    const std::int32_t near_pos_180 = 1799999999;
    const std::int32_t near_neg_180 = -1799999999;
    std::int32_t diff = Location::diff_longitude(near_pos_180, near_neg_180);
    REQUIRE(diff == Catch::Approx(-2).margin(1)); // wraps to a tiny difference, not ~3.6e9
}

TEST_CASE("Location::wrap_longitude keeps values in range unchanged", "[location]") {
    REQUIRE(Location::wrap_longitude(1000000000LL) == 1000000000);
    REQUIRE(Location::wrap_longitude(-1000000000LL) == -1000000000);
}

TEST_CASE("Location::wrap_longitude wraps values outside +-180e7", "[location]") {
    REQUIRE(Location::wrap_longitude(2000000000LL) == 2000000000 - 3600000000LL);
    REQUIRE(Location::wrap_longitude(-2000000000LL) == -2000000000 + 3600000000LL);
}

TEST_CASE("Location::limit_lattitude clamps by reflection past the poles", "[location]") {
    REQUIRE(Location::limit_lattitude(1000000000) == 1800000000LL - 1000000000);
    REQUIRE(Location::limit_lattitude(-1000000000) == -(1800000000LL - 1000000000));
    REQUIRE(Location::limit_lattitude(500000000) == 500000000); // in range, unchanged
}

TEST_CASE("Location::longitude_scale is 1 at the equator and shrinks toward the poles", "[location]") {
    REQUIRE(Location::longitude_scale(0) == Catch::Approx(1.0f).margin(1e-4));
    float scale_60deg = Location::longitude_scale(600000000); // 60 degrees N
    REQUIRE(scale_60deg == Catch::Approx(0.5f).margin(1e-3));  // cos(60deg) = 0.5
    REQUIRE(scale_60deg < 1.0f);
}

TEST_CASE("Location::offset moving north increases latitude, east increases longitude", "[location]") {
    Location loc(0, 0, 0, Location::AltFrame::ABSOLUTE);
    loc.offset(100.0f, 0.0f); // 100m north
    REQUIRE(loc.lat > 0);
    REQUIRE(loc.lng == 0);

    Location loc2(0, 0, 0, Location::AltFrame::ABSOLUTE);
    loc2.offset(0.0f, 100.0f); // 100m east
    REQUIRE(loc2.lat == 0);
    REQUIRE(loc2.lng > 0);
}

TEST_CASE("Location::offset then get_distance round-trips to approximately the offset distance", "[location]") {
    Location origin(-353632621, 1491652374, 0, Location::AltFrame::ABSOLUTE);
    Location moved = origin;
    moved.offset(200.0f, 150.0f); // north 200m, east 150m -> distance sqrt(200^2+150^2) = 250m
    float dist = origin.get_distance(moved);
    REQUIRE(dist == Catch::Approx(250.0f).margin(0.5f));
}

TEST_CASE("Location::offset_bearing moves in the expected compass direction", "[location]") {
    Location loc(0, 0, 0, Location::AltFrame::ABSOLUTE);
    loc.offset_bearing(0.0f, 100.0f); // bearing 0 = due north
    REQUIRE(loc.lat > 0);
    REQUIRE(loc.lng == 0);

    Location loc2(0, 0, 0, Location::AltFrame::ABSOLUTE);
    loc2.offset_bearing(90.0f, 100.0f); // bearing 90 = due east
    REQUIRE(loc2.lat == 0);
    REQUIRE(loc2.lng > 0);
}

TEST_CASE("Location::offset_bearing_and_pitch also adjusts altitude", "[location]") {
    Location loc(0, 0, 0, Location::AltFrame::ABSOLUTE);
    loc.offset_bearing_and_pitch(0.0f, 90.0f, 100.0f); // straight up (positive pitch = up in this call's convention)
    REQUIRE(loc.alt != 0);
}

TEST_CASE("Location::get_distance_NE gives north and east components separately", "[location]") {
    Location origin(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location moved = origin;
    moved.offset(200.0f, 150.0f);
    fwcpp::math::Vector2f ne = origin.get_distance_NE(moved);
    REQUIRE(ne.x == Catch::Approx(200.0f).margin(0.5f));
    REQUIRE(ne.y == Catch::Approx(150.0f).margin(0.5f));
}

TEST_CASE("Location::get_distance_NED includes the altitude difference as Down", "[location]") {
    Location a(0, 0, 1000, Location::AltFrame::ABSOLUTE); // 10m alt
    Location b(0, 0, 500, Location::AltFrame::ABSOLUTE);  // 5m alt (lower)
    fwcpp::math::Vector3f ned = a.get_distance_NED(b);
    REQUIRE(ned.x == Catch::Approx(0.0f));
    REQUIRE(ned.y == Catch::Approx(0.0f));
    // a is higher than b, so "down to reach b" is positive
    REQUIRE(ned.z == Catch::Approx(5.0f)); // (1000-500)*0.01 = 5.0m
}

TEST_CASE("Location::get_bearing_to matches the cardinal directions", "[location][bearing]") {
    Location origin(0, 0, 0, Location::AltFrame::ABSOLUTE);

    Location north = origin;
    north.offset(100.0f, 0.0f);
    REQUIRE(origin.get_bearing_to(north) == Catch::Approx(0).margin(200)); // ~0 cd

    Location east = origin;
    east.offset(0.0f, 100.0f);
    REQUIRE(origin.get_bearing_to(east) == Catch::Approx(9000).margin(200)); // ~90 deg = 9000 cd

    Location south = origin;
    south.offset(-100.0f, 0.0f);
    REQUIRE(origin.get_bearing_to(south) == Catch::Approx(18000).margin(200)); // ~180 deg

    Location west = origin;
    west.offset(0.0f, -100.0f);
    REQUIRE(origin.get_bearing_to(west) == Catch::Approx(27000).margin(200)); // ~270 deg
}

TEST_CASE("Location::get_bearing_to is always in [0, 35999]", "[location][bearing]") {
    Location origin(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location target = origin;
    target.offset_bearing(200.0f, 500.0f); // bearing 200 degrees, arbitrary
    std::int32_t bearing = origin.get_bearing_to(target);
    REQUIRE(bearing >= 0);
    REQUIRE(bearing <= 35999);
}

TEST_CASE("same_latlon_as is exact lat/lng equality, alt-independent", "[location][same_loc]") {
    Location a(100, 200, 1000, Location::AltFrame::ABSOLUTE);
    Location b(100, 200, 5000, Location::AltFrame::ABOVE_HOME); // different alt+frame
    REQUIRE(a.same_latlon_as(b));
    Location c(100, 201, 1000, Location::AltFrame::ABSOLUTE);
    REQUIRE_FALSE(a.same_latlon_as(c));
}

TEST_CASE("same_alt_as compares alt directly when frames match", "[location][same_loc]") {
    Location a(0, 0, 1000, Location::AltFrame::ABOVE_HOME);
    Location b(0, 0, 1000, Location::AltFrame::ABOVE_HOME);
    Location c(0, 0, 2000, Location::AltFrame::ABOVE_HOME);
    REQUIRE(a.same_alt_as(b));
    REQUIRE_FALSE(a.same_alt_as(c));
}

TEST_CASE("same_alt_as returns false across differing alt frames (fast path only - see file banner)", "[location][same_loc]") {
    Location a(0, 0, 1000, Location::AltFrame::ABOVE_HOME);
    Location b(0, 0, 1000, Location::AltFrame::ABSOLUTE); // same numeric alt, different frame
    REQUIRE_FALSE(a.same_alt_as(b));
}

TEST_CASE("same_loc_as requires both same_latlon_as and same_alt_as", "[location][same_loc]") {
    Location a(100, 200, 1000, Location::AltFrame::ABSOLUTE);
    Location b(100, 200, 1000, Location::AltFrame::ABSOLUTE);
    REQUIRE(a.same_loc_as(b));

    Location c(100, 200, 2000, Location::AltFrame::ABSOLUTE); // same latlon, different alt
    REQUIRE_FALSE(a.same_loc_as(c));

    Location d(101, 200, 1000, Location::AltFrame::ABSOLUTE); // different latlon, same alt
    REQUIRE_FALSE(a.same_loc_as(d));
}

TEST_CASE("is_zero is true only for a default-constructed (all-zero) Location", "[location][is_zero]") {
    Location a;
    REQUIRE(a.is_zero());

    Location b(1, 0, 0, Location::AltFrame::ABSOLUTE);
    REQUIRE_FALSE(b.is_zero());

    Location c(0, 0, 0, Location::AltFrame::ABOVE_HOME); // sets relative_alt=1 even with alt=0
    REQUIRE_FALSE(c.is_zero());
}

TEST_CASE("check_latlng accepts in-range and rejects out-of-range lat/lng", "[location][check_latlng]") {
    Location a(900000000, 1800000000, 0, Location::AltFrame::ABSOLUTE); // exactly at the poles/antimeridian
    REQUIRE(a.check_latlng());

    Location b(900000001, 0, 0, Location::AltFrame::ABSOLUTE); // just past the pole
    REQUIRE_FALSE(b.check_latlng());

    Location c(0, 1800000001, 0, Location::AltFrame::ABSOLUTE); // just past the antimeridian
    REQUIRE_FALSE(c.check_latlng());

    Location d(-900000000, -1800000000, 0, Location::AltFrame::ABSOLUTE);
    REQUIRE(d.check_latlng());
}

TEST_CASE("line_path_proportion is 0 at point1, 1 at point2, and interpolates between", "[location][line_path_proportion]") {
    Location point1(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location point2 = point1;
    point2.offset(1000.0f, 0.0f); // 1000m north

    Location at_start = point1;
    REQUIRE(at_start.line_path_proportion(point1, point2) == Catch::Approx(0.0f).margin(0.01f));

    Location at_end = point2;
    REQUIRE(at_end.line_path_proportion(point1, point2) == Catch::Approx(1.0f).margin(0.01f));

    Location halfway = point1;
    halfway.offset(500.0f, 0.0f);
    REQUIRE(halfway.line_path_proportion(point1, point2) == Catch::Approx(0.5f).margin(0.01f));

    Location past_end = point1;
    past_end.offset(1500.0f, 0.0f);
    REQUIRE(past_end.line_path_proportion(point1, point2) == Catch::Approx(1.5f).margin(0.01f));
}

TEST_CASE("line_path_proportion returns 1.0 when point1 and point2 are coincident", "[location][line_path_proportion]") {
    Location point1(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location point2 = point1; // same location - zero-length segment
    Location anywhere = point1;
    anywhere.offset(100.0f, 0.0f);
    REQUIRE(anywhere.line_path_proportion(point1, point2) == 1.0f);
}

TEST_CASE("past_interval_finish_line is true only once line_path_proportion reaches 1", "[location][past_interval_finish_line]") {
    Location point1(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location point2 = point1;
    point2.offset(1000.0f, 0.0f);

    Location before = point1;
    before.offset(500.0f, 0.0f);
    REQUIRE_FALSE(before.past_interval_finish_line(point1, point2));

    Location after = point1;
    after.offset(1500.0f, 0.0f);
    REQUIRE(after.past_interval_finish_line(point1, point2));

    Location at = point2;
    REQUIRE(at.past_interval_finish_line(point1, point2));
}
