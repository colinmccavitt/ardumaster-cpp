// Tests for fwcpp::Location core geometry (CPP-011 slice 1).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/location.hpp>

#include <cstdint>

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

TEST_CASE("initialised is false only for lat=lng=alt=0, regardless of alt frame flags", "[location][initialised]") {
    Location zero;
    REQUIRE_FALSE(zero.initialised());

    Location nonzero_lat(1, 0, 0, Location::AltFrame::ABSOLUTE);
    REQUIRE(nonzero_lat.initialised());

    // lat=lng=0 but a nonzero alt in a non-ABSOLUTE frame - looks exotic
    // but upstream's own definition only checks lat/lng/alt, not the
    // frame flags, so this counts as initialised.
    Location nonzero_alt(0, 0, 500, Location::AltFrame::ABOVE_HOME);
    REQUIRE(nonzero_alt.initialised());
}

TEST_CASE("copy_alt_from copies alt and every frame flag, leaving lat/lng untouched", "[location][copy_alt_from]") {
    Location a(100, 200, 1000, Location::AltFrame::ABSOLUTE);
    Location b(999, 888, 5000, Location::AltFrame::ABOVE_TERRAIN);
    a.copy_alt_from(b);
    REQUIRE(a.lat == 100);
    REQUIRE(a.lng == 200);
    REQUIRE(a.alt == 5000);
    REQUIRE(a.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN);
}

TEST_CASE("get_alt_cm same-frame shortcut succeeds with no context at all", "[location][get_alt_cm]") {
    Location loc(0, 0, 1234, Location::AltFrame::ABSOLUTE);
    fwcpp::AltitudeContext ctx; // home/origin both unset
    std::int32_t out = 0;
    REQUIRE(loc.get_alt_cm(Location::AltFrame::ABSOLUTE, ctx, out));
    REQUIRE(out == 1234);
}

TEST_CASE("get_alt_cm converts ABOVE_HOME to ABSOLUTE using the context's home altitude", "[location][get_alt_cm]") {
    Location loc(0, 0, 500, Location::AltFrame::ABOVE_HOME); // 500cm above home
    fwcpp::AltitudeContext ctx;
    ctx.home_is_set = true;
    ctx.home = Location(0, 0, 10000, Location::AltFrame::ABSOLUTE); // home at 100m AMSL

    std::int32_t out = 0;
    REQUIRE(loc.get_alt_cm(Location::AltFrame::ABSOLUTE, ctx, out));
    REQUIRE(out == 10500); // 100m home + 5m relative
}

TEST_CASE("get_alt_cm fails when the required context (home/origin) is not set", "[location][get_alt_cm]") {
    Location loc(0, 0, 500, Location::AltFrame::ABOVE_HOME);
    fwcpp::AltitudeContext ctx; // home_is_set left false
    std::int32_t out = 999;
    REQUIRE_FALSE(loc.get_alt_cm(Location::AltFrame::ABSOLUTE, ctx, out));
    REQUIRE(out == 999); // untouched on failure, matching upstream's contract
}

TEST_CASE("get_alt_cm round-trips ABOVE_HOME -> ABSOLUTE -> ABOVE_ORIGIN correctly", "[location][get_alt_cm]") {
    Location loc(0, 0, 500, Location::AltFrame::ABOVE_HOME);
    fwcpp::AltitudeContext ctx;
    ctx.home_is_set = true;
    ctx.home = Location(0, 0, 10000, Location::AltFrame::ABSOLUTE); // home at 100m AMSL
    ctx.origin_is_set = true;
    ctx.ekf_origin = Location(0, 0, 8000, Location::AltFrame::ABSOLUTE); // origin at 80m AMSL

    std::int32_t out = 0;
    REQUIRE(loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, ctx, out));
    // absolute = 100m + 5m = 105m; above origin (80m) = 25m = 2500cm
    REQUIRE(out == 2500);
}

TEST_CASE("get_alt_cm fails whenever a terrain frame is involved - no terrain database in this port", "[location][get_alt_cm]") {
    Location loc(0, 0, 500, Location::AltFrame::ABOVE_TERRAIN);
    fwcpp::AltitudeContext ctx;
    ctx.home_is_set = true;
    ctx.home = Location(0, 0, 10000, Location::AltFrame::ABSOLUTE);
    std::int32_t out = 0;
    REQUIRE_FALSE(loc.get_alt_cm(Location::AltFrame::ABSOLUTE, ctx, out));

    Location loc2(0, 0, 500, Location::AltFrame::ABSOLUTE);
    REQUIRE_FALSE(loc2.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, ctx, out));
}

TEST_CASE("get_alt_cm with matching source/desired terrain frames succeeds via the same-frame shortcut", "[location][get_alt_cm]") {
    // The one terrain-involving case this port CAN honor: no actual
    // conversion is happening, so no terrain database is needed.
    Location loc(0, 0, 500, Location::AltFrame::ABOVE_TERRAIN);
    fwcpp::AltitudeContext ctx;
    std::int32_t out = 0;
    REQUIRE(loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, ctx, out));
    REQUIRE(out == 500);
}

TEST_CASE("get_alt_m matches get_alt_cm scaled to meters", "[location][get_alt_m]") {
    Location loc(0, 0, 500, Location::AltFrame::ABOVE_HOME);
    fwcpp::AltitudeContext ctx;
    ctx.home_is_set = true;
    ctx.home = Location(0, 0, 10000, Location::AltFrame::ABSOLUTE);

    float out = 0.0f;
    REQUIRE(loc.get_alt_m(Location::AltFrame::ABSOLUTE, ctx, out));
    REQUIRE(out == Catch::Approx(105.0f));
}

TEST_CASE("change_alt_frame converts this Location's own altitude and frame in place", "[location][change_alt_frame]") {
    Location loc(0, 0, 500, Location::AltFrame::ABOVE_HOME);
    fwcpp::AltitudeContext ctx;
    ctx.home_is_set = true;
    ctx.home = Location(0, 0, 10000, Location::AltFrame::ABSOLUTE);

    REQUIRE(loc.change_alt_frame(Location::AltFrame::ABSOLUTE, ctx));
    REQUIRE(loc.alt == 10500);
    REQUIRE(loc.get_alt_frame() == Location::AltFrame::ABSOLUTE);
}

TEST_CASE("change_alt_frame leaves the Location unchanged when the conversion fails", "[location][change_alt_frame]") {
    Location loc(0, 0, 500, Location::AltFrame::ABOVE_HOME);
    fwcpp::AltitudeContext ctx; // home not set
    REQUIRE_FALSE(loc.change_alt_frame(Location::AltFrame::ABSOLUTE, ctx));
    REQUIRE(loc.alt == 500);
    REQUIRE(loc.get_alt_frame() == Location::AltFrame::ABOVE_HOME);
}

TEST_CASE("sanitize fills in lat/lng from default_loc when both are zero", "[location][sanitize]") {
    Location loc(0, 0, 1000, Location::AltFrame::ABSOLUTE);
    Location default_loc(100, 200, 5000, Location::AltFrame::ABSOLUTE);
    fwcpp::AltitudeContext ctx;

    REQUIRE(loc.sanitize(default_loc, ctx));
    REQUIRE(loc.lat == 100);
    REQUIRE(loc.lng == 200);
    REQUIRE(loc.alt == 1000); // alt untouched: not relative, so the alt branch doesn't apply
}

TEST_CASE("sanitize leaves a nonzero lat/lng untouched", "[location][sanitize]") {
    Location loc(50, 60, 1000, Location::AltFrame::ABSOLUTE);
    Location default_loc(100, 200, 5000, Location::AltFrame::ABSOLUTE);
    fwcpp::AltitudeContext ctx;

    REQUIRE_FALSE(loc.sanitize(default_loc, ctx));
    REQUIRE(loc.lat == 50);
    REQUIRE(loc.lng == 60);
}

TEST_CASE("sanitize fills in a zero relative altitude from default_loc when the context resolves it", "[location][sanitize]") {
    Location loc(50, 60, 0, Location::AltFrame::ABOVE_HOME); // alt=0, relative -> "use current alt"
    Location default_loc(50, 60, 1500, Location::AltFrame::ABOVE_HOME); // default_loc is 15m above home
    fwcpp::AltitudeContext ctx; // home not needed: get_alt_frame() == default_loc's frame -> same-frame shortcut

    REQUIRE(loc.sanitize(default_loc, ctx));
    REQUIRE(loc.alt == 1500);
}

TEST_CASE("sanitize's alt branch is skipped (not a crash) when get_alt_cm can't resolve it", "[location][sanitize]") {
    // loc's frame is ABOVE_HOME but default_loc is ABSOLUTE with no home
    // set in ctx - get_alt_cm has no way to convert, matching upstream's
    // own `if (...) { alt = ...; }` with no else: alt is simply left at 0.
    Location loc(50, 60, 0, Location::AltFrame::ABOVE_HOME);
    Location default_loc(50, 60, 5000, Location::AltFrame::ABSOLUTE);
    fwcpp::AltitudeContext ctx; // home_is_set left false

    REQUIRE_FALSE(loc.sanitize(default_loc, ctx)); // lat/lng already matched, alt branch failed silently
    REQUIRE(loc.alt == 0);
}

TEST_CASE("sanitize corrects an out-of-range lat/lng to default_loc's", "[location][sanitize]") {
    Location loc(950000000, 60, 1000, Location::AltFrame::ABSOLUTE); // lat out of range
    Location default_loc(100, 200, 5000, Location::AltFrame::ABSOLUTE);
    fwcpp::AltitudeContext ctx;

    REQUIRE(loc.sanitize(default_loc, ctx));
    REQUIRE(loc.lat == 100);
    REQUIRE(loc.lng == 200);
}

TEST_CASE("get_distance_NE_postype matches get_distance_NE within float precision", "[location][postype]") {
    Location a(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location b = a;
    b.offset(1000.0f, 500.0f);
    fwcpp::math::Vector2f d_float = a.get_distance_NE(b);
    fwcpp::math::Vector2p d_post = a.get_distance_NE_postype(b);
    REQUIRE(static_cast<float>(d_post.x) == Catch::Approx(d_float.x).margin(0.01f));
    REQUIRE(static_cast<float>(d_post.y) == Catch::Approx(d_float.y).margin(0.01f));
}

TEST_CASE("get_distance_NED_postype matches get_distance_NED bit-for-bit (no genuine precision gain - see comment)", "[location][postype]") {
    // Unlike get_distance_NE_postype, get_distance_NED_postype does NOT
    // cast LOCATION_SCALING_FACTOR to double before multiplying - matches
    // upstream's own inconsistency, so this should be exactly as precise
    // as (not better than) get_distance_NED, not just approximately equal.
    Location a(0, 0, 1000, Location::AltFrame::ABSOLUTE);
    Location b = a;
    b.offset(500.0f, 0.0f);
    b.alt = 2000;
    fwcpp::math::Vector3f d_float = a.get_distance_NED(b);
    fwcpp::math::Vector3p d_post = a.get_distance_NED_postype(b);
    REQUIRE(static_cast<float>(d_post.x) == d_float.x);
    REQUIRE(static_cast<float>(d_post.y) == d_float.y);
    REQUIRE(static_cast<float>(d_post.z) == d_float.z);
}

TEST_CASE("get_vector_xy_from_origin_NE_cm returns the NE offset from the EKF origin, in cm", "[location][origin]") {
    Location origin(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location loc = origin;
    loc.offset(100.0f, 0.0f); // 100m north of origin

    fwcpp::AltitudeContext ctx;
    ctx.origin_is_set = true;
    ctx.ekf_origin = origin;

    fwcpp::math::Vector2f vec;
    REQUIRE(loc.get_vector_xy_from_origin_NE_cm(vec, ctx));
    REQUIRE(vec.x == Catch::Approx(10000.0f).margin(50.0f)); // 100m = 10000cm
    REQUIRE(vec.y == Catch::Approx(0.0f).margin(50.0f));
}

TEST_CASE("get_vector_xy_from_origin_NE_cm fails when the context has no EKF origin", "[location][origin]") {
    Location loc(0, 0, 0, Location::AltFrame::ABSOLUTE);
    fwcpp::AltitudeContext ctx; // origin_is_set left false
    fwcpp::math::Vector2f vec(9.0f, 9.0f);
    REQUIRE_FALSE(loc.get_vector_xy_from_origin_NE_cm(vec, ctx));
    REQUIRE(vec.x == 9.0f); // untouched on failure
}

TEST_CASE("get_vector_xy_from_origin_NE_cm works with Vector2p (postype), touching only x/y", "[location][origin]") {
    Location origin(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location loc = origin;
    loc.offset(0.0f, 50.0f); // 50m east

    fwcpp::AltitudeContext ctx;
    ctx.origin_is_set = true;
    ctx.ekf_origin = origin;

    fwcpp::math::Vector2p vec;
    REQUIRE(loc.get_vector_xy_from_origin_NE_cm(vec, ctx));
    REQUIRE(vec.y == Catch::Approx(5000.0).margin(50.0));
}

TEST_CASE("get_vector_from_origin_NEU_cm combines the NE vector with altitude above origin", "[location][origin]") {
    Location origin(0, 0, 10000, Location::AltFrame::ABSOLUTE); // origin at 100m AMSL
    Location loc = origin;
    loc.offset(10.0f, 0.0f); // 10m north
    loc.alt = 15000; // 150m AMSL absolute

    fwcpp::AltitudeContext ctx;
    ctx.origin_is_set = true;
    ctx.ekf_origin = origin;

    fwcpp::math::Vector3f vec;
    REQUIRE(loc.get_vector_from_origin_NEU_cm(vec, ctx));
    REQUIRE(vec.x == Catch::Approx(1000.0f).margin(50.0f)); // 10m north = 1000cm
    REQUIRE(vec.z == Catch::Approx(5000.0f)); // 150m - 100m = 50m above origin = 5000cm
}

TEST_CASE("get_vector_from_origin_NEU_cm fails when altitude cannot be resolved above origin", "[location][origin]") {
    Location origin(0, 0, 10000, Location::AltFrame::ABSOLUTE);
    Location loc(0, 0, 500, Location::AltFrame::ABOVE_TERRAIN); // no terrain database in this port
    fwcpp::AltitudeContext ctx;
    ctx.origin_is_set = true;
    ctx.ekf_origin = origin;

    fwcpp::math::Vector3f vec;
    REQUIRE_FALSE(loc.get_vector_from_origin_NEU_cm(vec, ctx));
}

TEST_CASE("get_vector_from_origin_NEU is an alias for get_vector_from_origin_NEU_cm", "[location][origin]") {
    Location origin(0, 0, 10000, Location::AltFrame::ABSOLUTE);
    Location loc = origin;
    loc.offset(10.0f, 0.0f);
    loc.alt = 15000;
    fwcpp::AltitudeContext ctx;
    ctx.origin_is_set = true;
    ctx.ekf_origin = origin;

    fwcpp::math::Vector3f a, b;
    REQUIRE(loc.get_vector_from_origin_NEU_cm(a, ctx));
    REQUIRE(loc.get_vector_from_origin_NEU(b, ctx));
    REQUIRE(a.x == b.x);
    REQUIRE(a.y == b.y);
    REQUIRE(a.z == b.z);
}

TEST_CASE("get_vector_xy_from_origin_NE_m scales the cm result down by 100", "[location][origin]") {
    Location origin(0, 0, 0, Location::AltFrame::ABSOLUTE);
    Location loc = origin;
    loc.offset(100.0f, 0.0f);
    fwcpp::AltitudeContext ctx;
    ctx.origin_is_set = true;
    ctx.ekf_origin = origin;

    fwcpp::math::Vector2f vec;
    REQUIRE(loc.get_vector_xy_from_origin_NE_m(vec, ctx));
    REQUIRE(vec.x == Catch::Approx(100.0f).margin(0.5f));
}

TEST_CASE("get_vector_from_origin_NED_m negates z relative to get_vector_from_origin_NEU_m", "[location][origin]") {
    Location origin(0, 0, 10000, Location::AltFrame::ABSOLUTE);
    Location loc = origin;
    loc.alt = 15000;
    fwcpp::AltitudeContext ctx;
    ctx.origin_is_set = true;
    ctx.ekf_origin = origin;

    fwcpp::math::Vector3f neu;
    fwcpp::math::Vector3f ned;
    REQUIRE(loc.get_vector_from_origin_NEU_m(neu, ctx));
    REQUIRE(loc.get_vector_from_origin_NED_m(ned, ctx));
    REQUIRE(ned.z == Catch::Approx(-neu.z));
    REQUIRE(ned.x == Catch::Approx(neu.x));
    REQUIRE(ned.y == Catch::Approx(neu.y));
}

TEST_CASE("linearly_interpolate_alt sets altitude proportionally along the point1->point2 track", "[location][interpolate]") {
    Location point1(0, 0, 1000, Location::AltFrame::ABSOLUTE); // 10m
    Location point2 = point1;
    point2.offset(1000.0f, 0.0f); // 1000m north
    point2.alt = 3000; // 30m

    Location halfway = point1;
    halfway.offset(500.0f, 0.0f);
    halfway.linearly_interpolate_alt(point1, point2);
    REQUIRE(halfway.alt == Catch::Approx(2000).margin(20)); // halfway between 10m and 30m -> 20m
    REQUIRE(halfway.get_alt_frame() == Location::AltFrame::ABSOLUTE); // takes point2's frame
}

TEST_CASE("linearly_interpolate_alt clamps to point1's altitude before the start of the track", "[location][interpolate]") {
    Location point1(0, 0, 1000, Location::AltFrame::ABSOLUTE);
    Location point2 = point1;
    point2.offset(1000.0f, 0.0f);
    point2.alt = 3000;

    Location before_start = point1;
    before_start.offset(-500.0f, 0.0f); // 500m before point1
    before_start.linearly_interpolate_alt(point1, point2);
    REQUIRE(before_start.alt == Catch::Approx(1000).margin(20)); // clamped to point1's alt
}

TEST_CASE("linearly_interpolate_alt clamps to point2's altitude past the end of the track", "[location][interpolate]") {
    Location point1(0, 0, 1000, Location::AltFrame::ABSOLUTE);
    Location point2 = point1;
    point2.offset(1000.0f, 0.0f);
    point2.alt = 3000;

    Location past_end = point1;
    past_end.offset(1500.0f, 0.0f);
    past_end.linearly_interpolate_alt(point1, point2);
    REQUIRE(past_end.alt == Catch::Approx(3000).margin(20)); // clamped to point2's alt
}

TEST_CASE("from_ekf_offset_NEU_cm sets alt/frame always, lat/lng only when the context has an EKF origin", "[location][ekf_offset]") {
    fwcpp::math::Vector3f offset_neu_cm(1000.0f, 500.0f, 2000.0f); // 10m north, 5m east, 20m up
    fwcpp::AltitudeContext ctx; // origin not set

    Location out;
    REQUIRE_FALSE(Location::from_ekf_offset_NEU_cm(offset_neu_cm, Location::AltFrame::ABOVE_HOME, ctx, out));
    REQUIRE(out.lat == 0); // zero()'d, never got a lat/lng
    REQUIRE(out.lng == 0);
    REQUIRE(out.alt == 2000); // alt/frame ARE always set, matching upstream's constructor exactly
    REQUIRE(out.get_alt_frame() == Location::AltFrame::ABOVE_HOME);
}

TEST_CASE("from_ekf_offset_NEU_cm offsets from ekf_origin when the context has one", "[location][ekf_offset]") {
    Location origin(0, 0, 0, Location::AltFrame::ABSOLUTE);
    fwcpp::AltitudeContext ctx;
    ctx.origin_is_set = true;
    ctx.ekf_origin = origin;

    fwcpp::math::Vector3f offset_neu_cm(10000.0f, 0.0f, 500.0f); // 100m north
    Location out;
    REQUIRE(Location::from_ekf_offset_NEU_cm(offset_neu_cm, Location::AltFrame::ABSOLUTE, ctx, out));

    Location expected = origin;
    expected.offset(100.0f, 0.0f);
    REQUIRE(out.lat == Catch::Approx(static_cast<double>(expected.lat)).margin(2000)); // within ~2e-4 deg
    REQUIRE(out.alt == 500);
}

TEST_CASE("from_ekf_offset_NED_m matches from_ekf_offset_NEU_cm after unit/sign conversion", "[location][ekf_offset]") {
    Location origin(0, 0, 0, Location::AltFrame::ABSOLUTE);
    fwcpp::AltitudeContext ctx;
    ctx.origin_is_set = true;
    ctx.ekf_origin = origin;

    fwcpp::math::Vector3f offset_ned_m(50.0f, 25.0f, -10.0f); // 50m north, 25m east, 10m up (NED: negative down = up)
    fwcpp::math::Vector3f offset_neu_cm(5000.0f, 2500.0f, 1000.0f); // same offset, NEU-cm

    Location via_ned;
    Location via_neu;
    REQUIRE(Location::from_ekf_offset_NED_m(offset_ned_m, Location::AltFrame::ABSOLUTE, ctx, via_ned));
    REQUIRE(Location::from_ekf_offset_NEU_cm(offset_neu_cm, Location::AltFrame::ABSOLUTE, ctx, via_neu));

    REQUIRE(via_ned.lat == via_neu.lat);
    REQUIRE(via_ned.lng == via_neu.lng);
    REQUIRE(via_ned.alt == via_neu.alt);
}
