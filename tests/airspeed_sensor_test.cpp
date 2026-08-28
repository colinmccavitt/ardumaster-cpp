// Tests for fwcpp::airspeed::AirspeedSensor (CPP-082 phase 1) - the
// single-instance pressure-to-EAS pipeline. See airspeed_sensor.hpp's own
// file banner for exactly what real upstream behavior (AP_Airspeed::
// read(), PITOT_TUBE_ORDER_AUTO branch) this reproduces and what's
// excluded.

#include <cmath>
#include <cstdint>

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


// ---------------------------------------------------------------------
// CPP-083: the real boot-time zero-offset calibration routine
// (AP_Airspeed::calibrate()/update_calibration(), AP_Airspeed.cpp ~line
// 528/574) - see airspeed_sensor.hpp's file banner "CALIBRATION" section
// for the full transcription, upstream line references, and what's
// explicitly deferred (SkipCalType/SKIP_CAL, the watchdog-reset check,
// the airspeed_min-based warning check, persistence across a restart,
// the health-check state machine, and the interactive ratio auto-cal).
// ---------------------------------------------------------------------

TEST_CASE("AirspeedSensor calibration: default state is NotStarted; start_calibration() begins InProgress",
          "[airspeed][calibration]") {
    AirspeedSensor s;
    REQUIRE(s.calibration_state() == CalibrationState::NotStarted);

    s.start_calibration(1000);
    REQUIRE(s.calibration_state() == CalibrationState::InProgress);
    REQUIRE(s.calibration_read_count() == 0);
    REQUIRE(s.calibration_sample_count() == 0);
}

TEST_CASE("AirspeedSensor calibration: update() never touches calibration state when never started (NotStarted)",
          "[airspeed][calibration]") {
    // Zero regression check: a caller that never calls start_calibration()
    // (e.g. every existing CPP-082 test/call site above) gets bit-for-bit
    // the pre-CPP-083 read()-formula behavior - the calibration branch in
    // update() is gated on calibration_state() == InProgress, which never
    // becomes true without an explicit start_calibration() call.
    AirspeedSensor s(2.0f, 5.0f); // pre-set, non-zero offset
    for (int i = 0; i < 30; ++i) {
        s.update(100.0f, static_cast<std::uint32_t>(i * 100));
    }
    REQUIRE(s.calibration_state() == CalibrationState::NotStarted);
    REQUIRE(s.calibration_read_count() == 0);
    REQUIRE(s.offset() == Catch::Approx(5.0f)); // untouched - calibration never ran
}

// The ticket's own explicit acceptance criterion: still InProgress at 14
// reads, even with plenty of elapsed time - `read_count > 15` genuinely
// requires MORE than 15 completed (non-finalizing) calls before the
// finalize check can ever pass, verified directly against
// AP_Airspeed.cpp's own `state[i].cal.read_count > 15`.
TEST_CASE("AirspeedSensor calibration: still InProgress at 14 reads despite elapsed time far exceeding 1000ms",
          "[airspeed][calibration]") {
    AirspeedSensor s;
    s.start_calibration(0);
    std::uint32_t now_ms = 0;
    for (int i = 0; i < 14; ++i) {
        now_ms += 100; // 1400ms elapsed after 14 calls - well past the 1000ms time gate
        s.update(50.0f, now_ms);
    }
    REQUIRE(s.calibration_read_count() == 14);
    REQUIRE(s.calibration_state() == CalibrationState::InProgress); // time threshold cleared, read-count (14, not > 15) isn't
}

// The mirror image: read-count threshold cleared well before the 1000ms
// time threshold - `now_ms - cal.start_ms >= 1000` genuinely gates
// finalization on elapsed WALL TIME too, not just sample count, verified
// directly against AP_Airspeed.cpp's own real condition.
TEST_CASE("AirspeedSensor calibration: still InProgress past 16 reads if under 1000ms elapsed", "[airspeed][calibration]") {
    AirspeedSensor s;
    s.start_calibration(0);
    std::uint32_t now_ms = 0;
    // 20 calls at 10ms apart = 200ms elapsed - clears the read-count gate
    // (>15) many times over, nowhere near the 1000ms time gate.
    for (int i = 0; i < 20; ++i) {
        now_ms += 10;
        s.update(50.0f, now_ms);
    }
    REQUIRE(now_ms == 200);
    REQUIRE(s.calibration_read_count() >= 16);
    REQUIRE(s.calibration_state() == CalibrationState::InProgress);
}

// A REAL, VERIFIED SURPRISE (not the ticket's own paraphrase, and not
// this port's invention - traced directly against AP_Airspeed.cpp's real
// source, line 610-618): the comment immediately above the discard gate
// literally says "we discard the first 5 samples", but the gate itself
// is `state[i].healthy && state[i].cal.read_count > 5`, checked BEFORE
// this same call's own `state[i].cal.read_count++`, with read_count
// starting at 0 from calibrate(). Tracing calls 1..N with read_count
// going into call k equal to (k-1): the gate (k-1 > 5, i.e. k > 6) is
// false for k=1..6 and only becomes true starting at k=7 - i.e. the code
// actually discards SIX calls, not five. This port transcribes the real
// CODE exactly (per this port's own "port fixes bugs, not upstream"
// convention - a divergence from upstream's own comment is not a bug in
// upstream's CALIBRATION BEHAVIOR to "fix", it's upstream's own
// comment/code mismatch, faithfully reproduced here since the CODE is
// upstream's real behavior).
TEST_CASE("AirspeedSensor calibration: the real gate discards SIX calls (not the five upstream's own comment claims)",
          "[airspeed][calibration]") {
    AirspeedSensor s;
    s.start_calibration(0);
    std::uint32_t now_ms = 0;
    for (int i = 0; i < 6; ++i) {
        now_ms += 100;
        s.update(1000.0f, now_ms); // an outlier that would obviously skew sample_count()'s average if wrongly accumulated
        REQUIRE(s.calibration_sample_count() == 0);
    }
    REQUIRE(s.calibration_read_count() == 6);

    now_ms += 100;
    s.update(1000.0f, now_ms); // the 7th call - the first one that DOES accumulate
    REQUIRE(s.calibration_sample_count() == 1);
    REQUIRE(s.calibration_read_count() == 7);
}

// The ticket's own explicit, most-important acceptance criterion: a
// real, deterministic test proving AirspeedSensor genuinely RECOVERS an
// injected offset bias via the real averaging/discard/threshold logic -
// not just that the code compiles.
TEST_CASE("AirspeedSensor calibration: recovers a known injected zero-offset bias via real averaging",
          "[airspeed][calibration]") {
    constexpr float kBias = 37.5f;       // injected pitot zero-offset bias, Pa - unknown to the sensor a priori
    constexpr float kOutlier = 9999.0f;  // fed ONLY during the discarded window - must NOT affect the recovered offset
    AirspeedSensor s(2.0f, 0.0f);        // starts uncalibrated: ARSPD_OFFSET default of 0

    s.start_calibration(0);
    REQUIRE(s.calibration_state() == CalibrationState::InProgress);

    std::uint32_t now_ms = 0;

    // Calls 1-6: the real discarded window (see the dedicated
    // "discards SIX calls" test above for why it's six, not the
    // upstream comment's "five") - an extreme outlier here proves it is
    // truly excluded from the final average, not just small enough not
    // to matter.
    for (int i = 0; i < 6; ++i) {
        now_ms += 100;
        s.update(kOutlier, now_ms);
        REQUIRE(s.calibration_state() == CalibrationState::InProgress);
        REQUIRE(s.calibration_sample_count() == 0);
    }
    REQUIRE(s.calibration_read_count() == 6);

    // Calls 7-16: the real accumulation window (10 samples, the minimum
    // needed to reach read_count==16 without finalizing - see file
    // banner). True (stationary, zero-airspeed ground calibration)
    // pressure is 0 Pa, so raw_pressure = 0 + bias +/- a zero-mean
    // alternating perturbation (5 of +2, 5 of -2, cancelling exactly) -
    // proving this is a genuine average over real varying samples, not
    // just the same constant repeated 10 times.
    for (int i = 0; i < 10; ++i) {
        now_ms += 100;
        const float perturbation = (i % 2 == 0) ? 2.0f : -2.0f;
        s.update(kBias + perturbation, now_ms);
    }
    REQUIRE(s.calibration_read_count() == 16);
    REQUIRE(s.calibration_sample_count() == 10);
    // 16 reads is not "> 15" simultaneously satisfiable with itself as
    // the finalizing call - finalization needs a 17th call to observe
    // read_count==16 (i.e. > 15) at entry. Real upstream requires this
    // exact off-by-one (`read_count > 15`, checked BEFORE increment).
    REQUIRE(s.calibration_state() == CalibrationState::InProgress);

    // Call 17: elapsed = 1700ms (>= 1000) AND read_count entering this
    // call == 16 (> 15) - BOTH real thresholds now hold, so this call
    // finalizes rather than accumulating.
    now_ms += 100;
    s.update(kBias, now_ms);
    REQUIRE(s.calibration_state() == CalibrationState::Success);
    REQUIRE(s.calibration_read_count() == 16);  // frozen - the finalizing call returns before its own increment
    REQUIRE(s.calibration_sample_count() == 10);
    REQUIRE(s.offset() == Catch::Approx(kBias).margin(1e-3f)); // <-- the ticket's own core acceptance assertion

    // Same-tick ordering guarantee (see file banner's "CALIBRATION"
    // section): THIS SAME call's corrected_pressure_ was computed with
    // the OLD offset_ (0.0), matching upstream's own real ordering
    // (airspeed_pressure computed from get_offset(i) BEFORE
    // update_calibration() can write a new offset) - a same-tick
    // Success does not retroactively change this tick's own reading.
    REQUIRE(s.corrected_pressure() == Catch::Approx(kBias));

    // The NEXT tick is the first to observe the newly-calibrated offset_
    // applied - proving CPP-083's own "apply to offset_ directly" step
    // genuinely took effect for future readings, not just that offset()
    // reports the right number in isolation.
    now_ms += 100;
    s.update(kBias, now_ms); // raw_pressure == offset_ now -> corrected_pressure should collapse to ~0
    REQUIRE(s.corrected_pressure() == Catch::Approx(0.0f).margin(1e-3f));
}

// Mirrors the main recovery test above but with a different bias
// (including a case where the true stationary reading isn't exactly
// zero-mean-cancelling to a different value) to guard against the main
// test's specific numbers accidentally hiding an implementation bug
// (e.g. an accumulator that happens to work only for one magnitude).
TEST_CASE("AirspeedSensor calibration: recovers a second, differently-signed injected offset bias",
          "[airspeed][calibration]") {
    constexpr float kBias = -12.0f; // a negative offset bias this time
    AirspeedSensor s(2.0f, 0.0f);
    s.start_calibration(500); // a non-zero calibration start time, matching a real non-zero boot millis()

    std::uint32_t now_ms = 500;
    for (int i = 0; i < 6; ++i) {
        now_ms += 50;
        s.update(-4000.0f, now_ms); // discarded-window outlier
    }
    for (int i = 0; i < 10; ++i) {
        now_ms += 50;
        s.update(kBias, now_ms); // constant true-zero-airspeed reading + bias, no perturbation this time
    }
    // now_ms is 1300 here (500 start + 16*50); elapsed = 800ms, still
    // under the 1000ms gate, so still InProgress regardless of read_count.
    REQUIRE(s.calibration_state() == CalibrationState::InProgress);
    // Keep ticking until BOTH real thresholds clear, exactly like a real
    // caller would - this loop does not assume a specific finalizing
    // call index, unlike the main recovery test above.
    int guard = 0;
    while (s.calibration_state() == CalibrationState::InProgress && guard < 100) {
        now_ms += 50;
        s.update(kBias, now_ms);
        ++guard;
    }
    REQUIRE(s.calibration_state() == CalibrationState::Success);
    REQUIRE(s.offset() == Catch::Approx(kBias).margin(1e-3f));
}
