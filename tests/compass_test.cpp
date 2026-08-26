// Tests for fwcpp::compass::Compass (CPP-035) - the fixed-earth-field
// compass model. See compass.hpp's own file banner for exactly what
// upstream behavior this reproduces (Aircraft::update_mag_field_bf()'s
// earth-field construction and body-frame rotation, minus the permanent
// real-geodesy scope boundary) and what's excluded.

#include <cmath>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/compass/compass.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

using namespace fwcpp::compass;
using fwcpp::math::Matrix3f;
using fwcpp::math::Vector3f;

// ---------------------------------------------------------------------
// Earth-frame field construction (declination/inclination/intensity ->
// Vector3f), upstream's exact
// `from_euler(0, -inclination, declination) * Vector3f(intensity, 0, 0)`.
// ---------------------------------------------------------------------

TEST_CASE("Compass: zero declination/inclination reduces to a pure-North horizontal field", "[compass]") {
    Compass c(0.0f, 0.0f, 500.0f);
    const Vector3f& ef = c.earth_field();
    REQUIRE(ef.x == Catch::Approx(500.0f));
    REQUIRE(ef.y == Catch::Approx(0.0f).margin(1e-4));
    REQUIRE(ef.z == Catch::Approx(0.0f).margin(1e-4));
}

TEST_CASE("Compass: 90 degree declination (no inclination) rotates the field to pure East", "[compass]") {
    Compass c(fwcpp::math::radians(90.0f), 0.0f, 500.0f);
    const Vector3f& ef = c.earth_field();
    REQUIRE(ef.x == Catch::Approx(0.0f).margin(1e-3));
    REQUIRE(ef.y == Catch::Approx(500.0f));
    REQUIRE(ef.z == Catch::Approx(0.0f).margin(1e-3));
}

TEST_CASE("Compass: 90 degree inclination (no declination) points the field straight Down", "[compass]") {
    Compass c(0.0f, fwcpp::math::radians(90.0f), 500.0f);
    const Vector3f& ef = c.earth_field();
    REQUIRE(ef.x == Catch::Approx(0.0f).margin(1e-3));
    REQUIRE(ef.y == Catch::Approx(0.0f).margin(1e-3));
    REQUIRE(ef.z == Catch::Approx(500.0f));
}

TEST_CASE("Compass: a mid-latitude declination+inclination combination matches the hand-computed vector", "[compass]") {
    // declination = 30 deg, inclination = 60 deg, intensity = 500 mGauss.
    // Hand-derived from upstream's own construction (see compass.hpp's
    // file banner + this ticket's own from_euler(0,-incl,decl) trace):
    // mag_ef = intensity * (cos(incl)*cos(decl), cos(incl)*sin(decl), sin(incl)).
    const float decl = fwcpp::math::radians(30.0f);
    const float incl = fwcpp::math::radians(60.0f);
    const float intensity = 500.0f;
    Compass c(decl, incl, intensity);
    const Vector3f& ef = c.earth_field();
    const float expected_x = intensity * std::cos(incl) * std::cos(decl);
    const float expected_y = intensity * std::cos(incl) * std::sin(decl);
    const float expected_z = intensity * std::sin(incl);
    REQUIRE(ef.x == Catch::Approx(expected_x).margin(1e-3));
    REQUIRE(ef.y == Catch::Approx(expected_y).margin(1e-3));
    REQUIRE(ef.z == Catch::Approx(expected_z).margin(1e-3));
}

TEST_CASE("Compass: default constructor uses the cited real-world Halifax, NS reference value", "[compass]") {
    // See compass.hpp's own "FIXED EARTH-FIELD DEFAULT" note: Halifax, NS
    // (44.65N, 63.6W), declination -16.1deg, inclination 66.3deg,
    // intensity 510.11 mGauss.
    Compass c;
    REQUIRE(c.declination_rad() == Catch::Approx(fwcpp::math::radians(-16.1f)));
    // Rotation preserves vector length - the earth field's magnitude
    // should still be the cited 510.11 mGauss intensity regardless of the
    // declination/inclination rotation applied to build it.
    REQUIRE(c.earth_field().length() == Catch::Approx(510.11f).margin(0.05f));
}

// ---------------------------------------------------------------------
// Body-frame rotation for known attitudes -
// rotate_earth_field_to_body(true_dcm) = true_dcm.transposed() * mag_ef.
// ---------------------------------------------------------------------

TEST_CASE("Compass: rotate_earth_field_to_body with a level (identity) attitude leaves the field unchanged", "[compass]") {
    Compass c(0.0f, 0.0f, 500.0f); // pure North earth field
    Matrix3f level;
    level.identity();
    const Vector3f body = c.rotate_earth_field_to_body(level);
    REQUIRE(body.x == Catch::Approx(500.0f));
    REQUIRE(body.y == Catch::Approx(0.0f).margin(1e-3));
    REQUIRE(body.z == Catch::Approx(0.0f).margin(1e-3));
}

TEST_CASE("Compass: rotate_earth_field_to_body with a 90 degree yaw (nose East) rotates a pure-North field to -body-Y",
          "[compass]") {
    // Hand-derived (see this ticket's own trace of from_euler(0,0,pi/2)
    // and its transpose): nose pointing East, a field that truly points
    // North (earth frame) appears along the aircraft's OWN -Y (left) axis,
    // not +X (forward) or +Z (down).
    Compass c(0.0f, 0.0f, 500.0f);
    Matrix3f yawed_east;
    yawed_east.from_euler(0.0f, 0.0f, fwcpp::math::radians(90.0f));
    const Vector3f body = c.rotate_earth_field_to_body(yawed_east);
    REQUIRE(body.x == Catch::Approx(0.0f).margin(1e-3));
    REQUIRE(body.y == Catch::Approx(-500.0f));
    REQUIRE(body.z == Catch::Approx(0.0f).margin(1e-3));
}

TEST_CASE("Compass: rotate_earth_field_to_body with a 180 degree yaw reverses North/East horizontal components",
          "[compass]") {
    Compass c(0.0f, 0.0f, 500.0f);
    Matrix3f yawed_180;
    yawed_180.from_euler(0.0f, 0.0f, fwcpp::math::radians(180.0f));
    const Vector3f body = c.rotate_earth_field_to_body(yawed_180);
    REQUIRE(body.x == Catch::Approx(-500.0f));
    REQUIRE(body.y == Catch::Approx(0.0f).margin(1e-3));
    REQUIRE(body.z == Catch::Approx(0.0f).margin(1e-3));
}

TEST_CASE("Compass: rotate_earth_field_to_body preserves field magnitude under any attitude", "[compass]") {
    Compass c; // default (cited) earth field
    Matrix3f tilted;
    tilted.from_euler(fwcpp::math::radians(15.0f), fwcpp::math::radians(-25.0f), fwcpp::math::radians(70.0f));
    const Vector3f body = c.rotate_earth_field_to_body(tilted);
    REQUIRE(body.length() == Catch::Approx(c.earth_field().length()).margin(1e-3));
}

// ---------------------------------------------------------------------
// update()/sample() - CompassSample population.
// ---------------------------------------------------------------------

TEST_CASE("Compass: sample() defaults to unhealthy/zero before any update()", "[compass]") {
    Compass c;
    REQUIRE_FALSE(c.sample().healthy);
    REQUIRE(c.sample().field.is_zero());
    REQUIRE(c.sample().declination_rad == Catch::Approx(0.0f));
    REQUIRE(c.sample().last_update_usec == 0);
}

TEST_CASE("Compass: update() populates field/declination_rad/last_update_usec and marks the sample healthy",
          "[compass]") {
    const float decl = fwcpp::math::radians(12.0f);
    Compass c(decl, fwcpp::math::radians(50.0f), 500.0f);
    const Vector3f field_bf(100.0f, -200.0f, 300.0f);

    c.update(field_bf, 123456789ULL);

    const auto& s = c.sample();
    REQUIRE(s.healthy);
    REQUIRE(s.field.x == Catch::Approx(field_bf.x));
    REQUIRE(s.field.y == Catch::Approx(field_bf.y));
    REQUIRE(s.field.z == Catch::Approx(field_bf.z));
    REQUIRE(s.declination_rad == Catch::Approx(decl));
    REQUIRE(s.last_update_usec == 123456789ULL);
}

TEST_CASE("Compass: repeated update() calls overwrite the previous sample", "[compass]") {
    Compass c;
    c.update(Vector3f(1.0f, 2.0f, 3.0f), 1000);
    c.update(Vector3f(4.0f, 5.0f, 6.0f), 2000);

    const auto& s = c.sample();
    REQUIRE(s.field.x == Catch::Approx(4.0f));
    REQUIRE(s.field.y == Catch::Approx(5.0f));
    REQUIRE(s.field.z == Catch::Approx(6.0f));
    REQUIRE(s.last_update_usec == 2000);
    REQUIRE(s.healthy);
}

TEST_CASE("Compass: update() never touches attitude - the same field_bf input always yields the same sample regardless "
          "of any Compass-external state",
          "[compass]") {
    // Confirms Compass::update() is a pure function of (field_bf, now_usec)
    // plus its own fixed earth field/declination - it does not, and
    // cannot, read attitude at all (no such parameter exists) - see
    // compass.hpp's own "WHO COMPUTES..." file banner note.
    Compass a(fwcpp::math::radians(10.0f), fwcpp::math::radians(40.0f), 500.0f);
    Compass b(fwcpp::math::radians(10.0f), fwcpp::math::radians(40.0f), 500.0f);
    const Vector3f field_bf(50.0f, -25.0f, 10.0f);
    a.update(field_bf, 42);
    b.update(field_bf, 42);
    REQUIRE(a.sample().field.x == Catch::Approx(b.sample().field.x));
    REQUIRE(a.sample().field.y == Catch::Approx(b.sample().field.y));
    REQUIRE(a.sample().field.z == Catch::Approx(b.sample().field.z));
    REQUIRE(a.sample().declination_rad == Catch::Approx(b.sample().declination_rad));
}
