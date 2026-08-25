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
