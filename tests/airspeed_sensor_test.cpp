// Tests for fwcpp::airspeed::AirspeedSensor (CPP-082 phase 1) - the
// single-instance pressure-to-EAS pipeline. See airspeed_sensor.hpp's own
// file banner for exactly what real upstream behavior (AP_Airspeed::
// read(), PITOT_TUBE_ORDER_AUTO branch) this reproduces and what's
// excluded.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/airspeed/airspeed_sensor.hpp>

using namespace fwcpp::airspeed;

// ---------------------------------------------------------------------
// Real upstream defaults (AP_Airspeed_Params.cpp) - verified directly,
// see file banner.
// ---------------------------------------------------------------------

TEST_CASE("AirspeedSensor: default constructor uses upstream's real ARSPD_RATIO=2/ARSPD_OFFSET=0", "[airspeed]") {
    AirspeedSensor s;
    REQUIRE(s.ratio() == Catch::Approx(2.0f));
    REQUIRE(s.offset() == Catch::Approx(0.0f));
    REQUIRE(kDefaultRatio == Catch::Approx(2.0f));
    REQUIRE(kDefaultOffset == Catch::Approx(0.0f));
}

// Before any update() call, this port matches Compass's own "healthy
// means update() was called" precedent - default-constructed, unhealthy,
// zero everywhere.
TEST_CASE("AirspeedSensor: default-constructed is unhealthy with zero readings", "[airspeed]") {
    AirspeedSensor s;
    REQUIRE_FALSE(s.healthy());
    REQUIRE_FALSE(s.use());
    REQUIRE(s.airspeed() == Catch::Approx(0.0f));
    REQUIRE(s.raw_airspeed() == Catch::Approx(0.0f));
}

// ---------------------------------------------------------------------
// read()-formula hand-computed reference: offset subtraction, then
// (first call - see IIR section below) sqrt(|filtered_pressure| * ratio)
// with filtered_pressure == airspeed_pressure on the very first call.
// ---------------------------------------------------------------------

TEST_CASE("AirspeedSensor: first update() reproduces sqrt(|raw-offset|*ratio) exactly (ratio=2, offset=0)",
          "[airspeed]") {
    AirspeedSensor s(2.0f, 0.0f);
    // True airspeed 15 m/s, ratio 2 -> diff_pressure = 15^2/2 = 112.5 Pa
    // (matching sim_plane.hpp's own formula with zero noise) - feed that
    // pressure straight in.
    constexpr float raw_pressure = 112.5f;
    s.update(raw_pressure);
    REQUIRE(s.healthy());
    REQUIRE(s.use());
    REQUIRE(s.corrected_pressure() == Catch::Approx(112.5f));
    REQUIRE(s.filtered_pressure() == Catch::Approx(112.5f)); // reset-to-raw on the first (unhealthy->healthy) call
    const float expected_airspeed = std::sqrt(std::fabs(112.5f) * 2.0f);
    REQUIRE(expected_airspeed == Catch::Approx(15.0f).margin(1e-3)); // sanity: recovers the 15 m/s we started from
    REQUIRE(s.airspeed() == Catch::Approx(expected_airspeed));
    REQUIRE(s.raw_airspeed() == Catch::Approx(expected_airspeed));
}

TEST_CASE("AirspeedSensor: offset is subtracted before ratio/filter, matching real read()'s corrected_pressure",
          "[airspeed]") {
    AirspeedSensor s(2.0f, 10.0f); // ARSPD_OFFSET = 10 Pa
    s.update(122.5f);              // raw_pressure = 122.5 -> corrected = 112.5, same as the offset=0 case above
    REQUIRE(s.corrected_pressure() == Catch::Approx(112.5f));
    REQUIRE(s.airspeed() == Catch::Approx(std::sqrt(112.5f * 2.0f)));
}

TEST_CASE("AirspeedSensor: a hand-computed ratio=4 case matches sqrt(pressure*ratio) directly", "[airspeed]") {
    AirspeedSensor s(4.0f, 0.0f);
    s.update(50.0f);
    REQUIRE(s.airspeed() == Catch::Approx(std::sqrt(50.0f * 4.0f)));
    REQUIRE(s.airspeed() == Catch::Approx(std::sqrt(200.0f)));
}

TEST_CASE("AirspeedSensor: PITOT_TUBE_ORDER_AUTO takes the absolute value of a negative corrected pressure",
          "[airspeed]") {
    // upstream's AUTO branch: `sqrtf(fabsf(filtered_pressure) * ratio)` -
    // a negative corrected pressure (e.g. offset larger than raw_pressure,
    // or a reversed pitot connection this port doesn't model - see file
    // banner's "signflip" exclusion) still yields a real, non-NaN
    // airspeed via fabsf(), not a domain error.
    AirspeedSensor s(2.0f, 100.0f);
    s.update(50.0f); // corrected_pressure = 50 - 100 = -50
    REQUIRE(s.corrected_pressure() == Catch::Approx(-50.0f));
    REQUIRE(s.airspeed() == Catch::Approx(std::sqrt(50.0f * 2.0f)));
}

// ---------------------------------------------------------------------
// The 0.7/0.3 IIR filter and its unhealthy->healthy reset edge case -
// the ticket's own explicit acceptance criterion.
// ---------------------------------------------------------------------

TEST_CASE("AirspeedSensor: IIR filter blends 0.7*prev + 0.3*new on consecutive healthy updates", "[airspeed]") {
    AirspeedSensor s(2.0f, 0.0f);
    s.update(100.0f); // first call: reset to raw -> filtered = 100
    REQUIRE(s.filtered_pressure() == Catch::Approx(100.0f));

    s.update(200.0f); // second call: prev_healthy is now true -> real IIR blend
    const float expected_second = 0.7f * 100.0f + 0.3f * 200.0f;
    REQUIRE(s.filtered_pressure() == Catch::Approx(expected_second)); // == 130.0
    REQUIRE(s.airspeed() == Catch::Approx(std::sqrt(expected_second * 2.0f)));

    s.update(200.0f); // third call: same input, filter keeps converging toward it
    const float expected_third = 0.7f * expected_second + 0.3f * 200.0f;
    REQUIRE(s.filtered_pressure() == Catch::Approx(expected_third));
    REQUIRE(s.filtered_pressure() > expected_second); // monotonically approaching 200 from below
}

TEST_CASE("AirspeedSensor: filter converges toward a steady input pressure over many updates", "[airspeed]") {
    AirspeedSensor s(2.0f, 0.0f);
    s.update(0.0f); // reset branch
    for (int i = 0; i < 200; ++i) {
        s.update(112.5f);
    }
    REQUIRE(s.filtered_pressure() == Catch::Approx(112.5f).margin(1e-3));
    REQUIRE(s.airspeed() == Catch::Approx(15.0f).margin(1e-3));
}

TEST_CASE("AirspeedSensor: raw_airspeed always tracks the UNFILTERED corrected pressure, even mid-filter-transient",
          "[airspeed]") {
    AirspeedSensor s(2.0f, 0.0f);
    s.update(100.0f);
    s.update(400.0f); // filtered lags behind (0.7*100+0.3*400=190), raw does not
    REQUIRE(s.filtered_pressure() == Catch::Approx(190.0f));
    REQUIRE(s.raw_airspeed() == Catch::Approx(std::sqrt(400.0f * 2.0f))); // unfiltered - tracks THIS call's input exactly
    REQUIRE(s.airspeed() == Catch::Approx(std::sqrt(190.0f * 2.0f)));     // filtered - lags
    REQUIRE(s.raw_airspeed() != s.airspeed());
}

// This is the ticket's own explicit "unhealthy->healthy transition"
// acceptance criterion: this phase's healthy() is a near-no-op that is
// always true once update() has been called (see file banner's
// "HEALTHY()" note - SITL's own get_differential_pressure() never fails
// without fail-injection this phase doesn't build), so the ONLY
// unhealthy->healthy transition reachable in phase 1 is the very first
// update() call on a freshly-constructed sensor - verified directly
// above (every "first update()" case resets to raw rather than blending
// with the zero-initialized filtered_pressure_). This test makes that
// transition explicit and contrasts it with a hypothetical "blended"
// result to prove the reset branch, not the blend branch, actually ran.
TEST_CASE("AirspeedSensor: the very first update() is itself the unhealthy->healthy transition, and resets rather "
          "than blends",
          "[airspeed]") {
    AirspeedSensor s(2.0f, 0.0f);
    REQUIRE_FALSE(s.healthy()); // unhealthy before any update()

    s.update(300.0f);
    REQUIRE(s.healthy()); // healthy immediately after - this WAS the unhealthy->healthy transition

    // If this had incorrectly blended with a zero-initialized
    // filtered_pressure_ (0.7*0 + 0.3*300 = 90) instead of resetting
    // (filtered_pressure_ = 300), the two would differ sharply - assert
    // the RESET value, not the hypothetical blended one.
    const float blended_if_bug = 0.7f * 0.0f + 0.3f * 300.0f;
    REQUIRE(s.filtered_pressure() == Catch::Approx(300.0f));
    REQUIRE(s.filtered_pressure() != Catch::Approx(blended_if_bug));
}
