// Tests for fwcpp::motors::ThrustLinearization (CCP-010) - the thrust
// curve linearization + battery-voltage/air-density compensation slice of
// AP_Motors_Thrust_Linearization. See thrust_linearization.hpp's own file
// banner for exactly what upstream behavior this reproduces (including
// the real unclamped-zero-expo asymmetry and the explicit-parameter
// battery/air-density inputs) and what is deferred.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/motors/thrust_linearization.hpp>

using namespace fwcpp::motors;
using Catch::Approx;

namespace {

BatteryVoltage resting(float voltage) {
    // raw is unused by update_lift_max_from_batt_voltage (see file
    // banner's has_option(BATT_RAW_VOLTAGE) simplification) - set equal
    // to resting_estimate so a test that accidentally read the wrong
    // field would still be exercising a plausible voltage, not silently
    // passing on a zero.
    return BatteryVoltage{voltage, voltage};
}

} // namespace

// ---------------------------------------------------------------------
// Round-trip: thrust_to_actuator / actuator_to_thrust are real inverses
// of one another, upstream's own documented intent ("Inverse of above"),
// tested (upstream's own comment) against AP_Motors/examples/expo_inverse_test.
// ---------------------------------------------------------------------

TEST_CASE("thrust/actuator round-trip holds across expo and spin range, non-zero expo",
          "[motors][thrust_linearization][round_trip]") {
    ThrustLinearization tl;
    const float expos[] = {-1.0f, -0.5f, 0.3f, 0.65f, 1.0f};
    const float spin_pairs[][2] = {{0.0f, 1.0f}, {0.15f, 0.95f}, {0.1f, 0.8f}};
    const float thrusts[] = {0.0f, 0.05f, 0.25f, 0.5f, 0.75f, 0.95f, 1.0f};

    for (float expo : expos) {
        for (const auto& sp : spin_pairs) {
            ThrustLinParams params;
            params.curve_expo = expo;
            params.spin_min = sp[0];
            params.spin_max = sp[1];
            for (float thrust : thrusts) {
                const float actuator = tl.thrust_to_actuator(params, thrust);
                const float recovered = tl.actuator_to_thrust(params, actuator);
                REQUIRE(recovered == Approx(thrust).margin(1e-4));
            }
        }
    }
}

TEST_CASE("thrust/actuator round-trip holds for the zero-expo (linear) special case",
          "[motors][thrust_linearization][round_trip][zero_expo]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.curve_expo = 0.0f;
    params.spin_min = 0.15f;
    params.spin_max = 0.95f;

    for (float thrust : {0.0f, 0.1f, 0.33f, 0.5f, 0.7f, 0.9f, 1.0f}) {
        const float actuator = tl.thrust_to_actuator(params, thrust);
        const float recovered = tl.actuator_to_thrust(params, actuator);
        REQUIRE(recovered == Approx(thrust).margin(1e-5));
    }

    // And directly, without going through the spin range at all.
    const float applied = tl.apply_thrust_curve_and_volt_scaling(params, 0.42f);
    const float removed = tl.remove_thrust_curve_and_volt_scaling(params, applied);
    REQUIRE(removed == Approx(0.42f).margin(1e-5));
}

TEST_CASE("actuator/thrust round-trip holds the other way (actuator_to_thrust first)",
          "[motors][thrust_linearization][round_trip]") {
    ThrustLinearization tl;
    for (float expo : {-0.7f, 0.0f, 0.4f, 1.0f}) {
        ThrustLinParams params;
        params.curve_expo = expo;
        params.spin_min = 0.15f;
        params.spin_max = 0.95f;
        // Actuator values must stay within [spin_min, spin_max] to be
        // reachable from some thrust in [0, 1] in the first place.
        for (float actuator : {0.15f, 0.3f, 0.5f, 0.7f, 0.95f}) {
            const float thrust = tl.actuator_to_thrust(params, actuator);
            const float recovered = tl.thrust_to_actuator(params, thrust);
            REQUIRE(recovered == Approx(actuator).margin(1e-4));
        }
    }
}

TEST_CASE("round-trip still holds under non-trivial battery compensation",
          "[motors][thrust_linearization][round_trip][battery]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.curve_expo = 0.65f;
    params.spin_min = 0.15f;
    params.spin_max = 0.95f;
    params.batt_voltage_max = 16.8f;
    params.batt_voltage_min = 10.5f;

    // A single non-saturating update: pack sitting at a mid-range
    // voltage. dt large relative to the 0.5 Hz cutoff so the filter has
    // mostly converged rather than testing the filter itself here.
    tl.update_lift_max_from_batt_voltage(params, resting(14.0f), 10.0f);
    REQUIRE(tl.lift_max() < 1.0f);

    for (float thrust : {0.0f, 0.2f, 0.5f, 0.8f, 1.0f}) {
        const float actuator = tl.thrust_to_actuator(params, thrust);
        const float recovered = tl.actuator_to_thrust(params, actuator);
        REQUIRE(recovered == Approx(thrust).margin(1e-4));
    }
}

// ---------------------------------------------------------------------
// The real unclamped-zero-expo asymmetry: for the SAME out-of-range
// input, the zero-expo branch returns unclamped while the non-zero-expo
// branch clamps to [0, 1]. A port that "corrected" the zero-expo branch
// to also clamp would fail these.
// ---------------------------------------------------------------------

TEST_CASE("apply_thrust_curve_and_volt_scaling: zero-expo branch is genuinely unclamped above 1, "
          "non-zero-expo branch clamps for the same thrust",
          "[motors][thrust_linearization][asymmetry]") {
    ThrustLinearization tl; // lift_max()==1.0, battery_scale==1.0 (fresh filter)
    ThrustLinParams zero_expo;
    zero_expo.curve_expo = 0.0f;
    ThrustLinParams nonzero_expo;
    nonzero_expo.curve_expo = 0.65f;

    const float thrust = 2.0f; // deliberately outside [0, 1] - both apply_*
                                // functions take a raw thrust with no
                                // clamp of their own (only thrust_to_actuator
                                // clamps its input before calling in).

    const float zero_result = tl.apply_thrust_curve_and_volt_scaling(zero_expo, thrust);
    // lift_max * thrust * battery_scale = 1.0 * 2.0 * 1.0 = 2.0, exactly -
    // NOT clamped to 1.0.
    REQUIRE(zero_result == Approx(2.0f).margin(1e-5));
    REQUIRE(zero_result > 1.0f);

    const float nonzero_result = tl.apply_thrust_curve_and_volt_scaling(nonzero_expo, thrust);
    // The general quadratic branch DOES clamp - this same thrust drives
    // its throttle_ratio well above 1.0 before the clamp.
    REQUIRE(nonzero_result == Approx(1.0f).margin(1e-6));
}

TEST_CASE("apply_thrust_curve_and_volt_scaling: zero-expo branch is genuinely unclamped below 0",
          "[motors][thrust_linearization][asymmetry]") {
    ThrustLinearization tl;
    ThrustLinParams zero_expo;
    zero_expo.curve_expo = 0.0f;
    ThrustLinParams nonzero_expo;
    nonzero_expo.curve_expo = 0.65f;

    const float thrust = -1.0f; // deliberately negative.

    const float zero_result = tl.apply_thrust_curve_and_volt_scaling(zero_expo, thrust);
    REQUIRE(zero_result == Approx(-1.0f).margin(1e-5));
    REQUIRE(zero_result < 0.0f);

    const float nonzero_result = tl.apply_thrust_curve_and_volt_scaling(nonzero_expo, thrust);
    REQUIRE(nonzero_result == Approx(0.0f).margin(1e-6));
}

TEST_CASE("remove_thrust_curve_and_volt_scaling: zero-expo branch is genuinely unclamped above 1, "
          "non-zero-expo branch clamps for the same throttle",
          "[motors][thrust_linearization][asymmetry]") {
    ThrustLinearization tl; // lift_max()==1.0, battery_scale==1.0
    ThrustLinParams zero_expo;
    zero_expo.curve_expo = 0.0f;
    ThrustLinParams nonzero_expo;
    nonzero_expo.curve_expo = 0.65f;

    const float throttle = 2.0f;

    const float zero_result = tl.remove_thrust_curve_and_volt_scaling(zero_expo, throttle);
    // throttle / (lift_max * battery_scale) = 2.0 / (1.0 * 1.0) = 2.0,
    // exactly - NOT clamped to 1.0.
    REQUIRE(zero_result == Approx(2.0f).margin(1e-5));
    REQUIRE(zero_result > 1.0f);

    const float nonzero_result = tl.remove_thrust_curve_and_volt_scaling(nonzero_expo, throttle);
    REQUIRE(nonzero_result == Approx(1.0f).margin(1e-6));
}

TEST_CASE("apply/remove asymmetry survives round-trip framing: actuator_to_thrust clamps "
          "even though remove_thrust_curve_and_volt_scaling's own zero-expo branch would not",
          "[motors][thrust_linearization][asymmetry]") {
    // actuator_to_thrust always clamps its own final result (real
    // upstream `constrain_float(remove_thrust_curve_and_volt_scaling(...), 0, 1)`),
    // regardless of which branch remove_thrust_curve_and_volt_scaling took
    // internally - the asymmetry is only visible when calling
    // remove_thrust_curve_and_volt_scaling directly, as the tests above do.
    ThrustLinearization tl;
    ThrustLinParams zero_expo;
    zero_expo.curve_expo = 0.0f;
    zero_expo.spin_min = 0.0f;
    zero_expo.spin_max = 1.0f;

    const float unclamped_direct = tl.remove_thrust_curve_and_volt_scaling(zero_expo, 2.0f);
    REQUIRE(unclamped_direct > 1.0f);

    const float through_actuator_to_thrust = tl.actuator_to_thrust(zero_expo, 2.0f);
    REQUIRE(through_actuator_to_thrust == Approx(1.0f).margin(1e-6));
}

// ---------------------------------------------------------------------
// update_lift_max_from_batt_voltage: the real misconfiguration
// early-return, and the real batt_voltage_min write-back.
// ---------------------------------------------------------------------

TEST_CASE("update_lift_max_from_batt_voltage: batt_voltage_max <= 0 bails to lift_max == 1.0",
          "[motors][thrust_linearization][battery]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.batt_voltage_max = 16.8f;
    params.batt_voltage_min = 10.5f;
    tl.update_lift_max_from_batt_voltage(params, resting(14.0f), 10.0f);
    REQUIRE(tl.lift_max() < 1.0f); // setup perturbed lift_max away from 1.0

    ThrustLinParams misconfigured;
    misconfigured.batt_voltage_max = 0.0f; // disabled
    misconfigured.batt_voltage_min = 10.5f;
    tl.update_lift_max_from_batt_voltage(misconfigured, resting(14.0f), 0.02f);
    REQUIRE(tl.lift_max() == Approx(1.0f).margin(1e-6));
}

TEST_CASE("update_lift_max_from_batt_voltage: batt_voltage_min >= batt_voltage_max bails to lift_max == 1.0",
          "[motors][thrust_linearization][battery]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.batt_voltage_max = 16.8f;
    params.batt_voltage_min = 10.5f;
    tl.update_lift_max_from_batt_voltage(params, resting(14.0f), 10.0f);
    REQUIRE(tl.lift_max() < 1.0f);

    ThrustLinParams misconfigured;
    misconfigured.batt_voltage_max = 12.0f;
    misconfigured.batt_voltage_min = 12.0f; // min == max
    tl.update_lift_max_from_batt_voltage(misconfigured, resting(14.0f), 0.02f);
    REQUIRE(tl.lift_max() == Approx(1.0f).margin(1e-6));
}

TEST_CASE("update_lift_max_from_batt_voltage: implausibly low voltage (< 0.25 * batt_voltage_min) "
          "bails to lift_max == 1.0",
          "[motors][thrust_linearization][battery]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.batt_voltage_max = 16.8f;
    params.batt_voltage_min = 10.5f;
    // 0.25 * 10.5 = 2.625: 1.0V is deliberately far below even that (e.g.
    // a disconnected battery reporting near zero).
    tl.update_lift_max_from_batt_voltage(params, resting(1.0f), 0.02f);
    REQUIRE(tl.lift_max() == Approx(1.0f).margin(1e-6));
}

TEST_CASE("update_lift_max_from_batt_voltage: the misconfiguration bail-out also resets the "
          "voltage filter to 1.0, restoring battery_scale == 1.0",
          "[motors][thrust_linearization][battery]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.batt_voltage_max = 16.8f;
    params.batt_voltage_min = 10.5f;
    tl.update_lift_max_from_batt_voltage(params, resting(10.6f), 10.0f);
    REQUIRE(tl.lift_max() < 1.0f);

    ThrustLinParams misconfigured;
    misconfigured.batt_voltage_max = 0.0f;
    misconfigured.batt_voltage_min = 0.0f;
    tl.update_lift_max_from_batt_voltage(misconfigured, resting(10.6f), 0.02f);

    // battery_scale is 1/batt_voltage_filt.get() once the filter is reset
    // to 1.0, so a zero-expo apply() with thrust=1 should read back
    // exactly lift_max (1.0) * 1.0 * 1.0.
    ThrustLinParams zero_expo;
    zero_expo.curve_expo = 0.0f;
    REQUIRE(tl.apply_thrust_curve_and_volt_scaling(zero_expo, 1.0f) == Approx(1.0f).margin(1e-6));
}

TEST_CASE("update_lift_max_from_batt_voltage: a too-small batt_voltage_min is permanently raised "
          "to 0.6 * batt_voltage_max",
          "[motors][thrust_linearization][battery][write_back]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.batt_voltage_max = 16.8f;
    params.batt_voltage_min = 1.0f; // far below 0.6 * 16.8 = 10.08
    tl.update_lift_max_from_batt_voltage(params, resting(14.0f), 10.0f);
    REQUIRE(params.batt_voltage_min == Approx(16.8f * 0.6f).margin(1e-6));
}

TEST_CASE("update_lift_max_from_batt_voltage: a batt_voltage_min already above the 0.6 floor is left alone",
          "[motors][thrust_linearization][battery][write_back]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.batt_voltage_max = 16.8f;
    params.batt_voltage_min = 12.0f; // already above 0.6 * 16.8 = 10.08
    tl.update_lift_max_from_batt_voltage(params, resting(14.0f), 10.0f);
    REQUIRE(params.batt_voltage_min == Approx(12.0f).margin(1e-6));
}

TEST_CASE("update_lift_max_from_batt_voltage: only battery.resting_estimate is read, "
          "battery.raw is ignored (has_option(BATT_RAW_VOLTAGE) simplification)",
          "[motors][thrust_linearization][battery]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.batt_voltage_max = 16.8f;
    params.batt_voltage_min = 10.5f;
    // raw is set to an implausibly low value that would fail the sanity
    // check on its own (< 0.25 * 10.5 = 2.625) - if it were read instead
    // of resting_estimate, lift_max would bail to 1.0. resting_estimate
    // is a plausible mid-range voltage.
    BatteryVoltage battery{0.5f, 14.0f};
    tl.update_lift_max_from_batt_voltage(params, battery, 10.0f);
    REQUIRE(tl.lift_max() < 1.0f);
    REQUIRE(tl.lift_max() != Approx(1.0f).margin(1e-6));
}

// ---------------------------------------------------------------------
// get_compensation_gain: the real air-density gate (0.3, 1.5), both
// bounds exclusive, plus the inner (0.5, 1.25) clamp on the ratio itself.
// ---------------------------------------------------------------------

TEST_CASE("get_compensation_gain: ratio inside the gate applies 1/ratio scaling",
          "[motors][thrust_linearization][compensation_gain]") {
    ThrustLinearization tl; // lift_max() == 1.0
    // 1.0 is inside both (0.3, 1.5) and the inner (0.5, 1.25) clamp, so it
    // applies unscaled: gain = 1/lift_max * 1/1.0 = 1.0.
    REQUIRE(tl.get_compensation_gain(1.0f) == Approx(1.0f).margin(1e-6));
}

TEST_CASE("get_compensation_gain: ratio below the inner clamp floor is scaled by the clamped "
          "value (0.5), not the true ratio",
          "[motors][thrust_linearization][compensation_gain]") {
    ThrustLinearization tl; // lift_max() == 1.0
    // 0.4 is inside the gate (0.3, 1.5) but below the clamp floor 0.5.
    const float gain = tl.get_compensation_gain(0.4f);
    REQUIRE(gain == Approx(1.0f / 0.5f).margin(1e-5)); // == 2.0
}

TEST_CASE("get_compensation_gain: ratio above the inner clamp ceiling is scaled by the clamped "
          "value (1.25), not the true ratio",
          "[motors][thrust_linearization][compensation_gain]") {
    ThrustLinearization tl;
    // 1.4 is inside the gate (0.3, 1.5) but above the clamp ceiling 1.25.
    const float gain = tl.get_compensation_gain(1.4f);
    REQUIRE(gain == Approx(1.0f / 1.25f).margin(1e-5)); // == 0.8
}

TEST_CASE("get_compensation_gain: ratio at or below the gate's lower bound (0.3) applies no "
          "density scaling - bound is exclusive",
          "[motors][thrust_linearization][compensation_gain][boundary]") {
    ThrustLinearization tl; // lift_max() == 1.0, so gain == 1.0 with no density term
    REQUIRE(tl.get_compensation_gain(0.3f) == Approx(1.0f).margin(1e-6)); // exactly at bound: excluded
    REQUIRE(tl.get_compensation_gain(0.2f) == Approx(1.0f).margin(1e-6)); // below bound: excluded
}

TEST_CASE("get_compensation_gain: ratio at or above the gate's upper bound (1.5) applies no "
          "density scaling - bound is exclusive",
          "[motors][thrust_linearization][compensation_gain][boundary]") {
    ThrustLinearization tl;
    REQUIRE(tl.get_compensation_gain(1.5f) == Approx(1.0f).margin(1e-6)); // exactly at bound: excluded
    REQUIRE(tl.get_compensation_gain(1.6f) == Approx(1.0f).margin(1e-6)); // above bound: excluded
}

TEST_CASE("get_compensation_gain: just inside the gate bounds does apply density scaling",
          "[motors][thrust_linearization][compensation_gain][boundary]") {
    ThrustLinearization tl;
    // Just above 0.3 (still below the 0.5 clamp floor): scaled by 0.5.
    REQUIRE(tl.get_compensation_gain(0.30001f) == Approx(1.0f / 0.5f).margin(1e-4));
    // Just below 1.5 (still above the 1.25 clamp ceiling): scaled by 1.25.
    REQUIRE(tl.get_compensation_gain(1.49999f) == Approx(1.0f / 1.25f).margin(1e-4));
}

TEST_CASE("get_compensation_gain: incorporates 1/lift_max together with the density term",
          "[motors][thrust_linearization][compensation_gain]") {
    ThrustLinearization tl;
    ThrustLinParams params;
    params.batt_voltage_max = 16.8f;
    params.batt_voltage_min = 10.5f;
    tl.update_lift_max_from_batt_voltage(params, resting(12.0f), 10.0f);
    REQUIRE(tl.lift_max() < 1.0f);

    const float expected_no_density = 1.0f / tl.lift_max();
    REQUIRE(tl.get_compensation_gain(0.2f) == Approx(expected_no_density).margin(1e-5)); // outside gate

    const float expected_with_density = (1.0f / tl.lift_max()) * (1.0f / 1.0f); // ratio 1.0, unclamped
    REQUIRE(tl.get_compensation_gain(1.0f) == Approx(expected_with_density).margin(1e-5));
}
