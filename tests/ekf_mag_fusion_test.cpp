// Tests for fwcpp::ekf::EkfCore's CPP-059 phase 5 addition: 3-axis
// magnetometer fusion (fuse_magnetometer()). See fwcpp/ekf/ekf_core.hpp's
// "CPP-059, PHASE 5" banner for the full scope/exclusions/corrections
// discussion this ticket's own ticket file asked for.
//
// CPP-060 phase 6 ADDENDUM (see fwcpp/ekf/ekf_core.hpp's "CPP-060, PHASE
// 6" banner): tests 7/8 below cover the new mag_test_ratio()/magHealth
// innovation-consistency gate. Test 7 exploits the gate's real,
// verified-directly per-axis (NOT combined) structure: a single-axis-
// dominant inconsistency large enough to fail ONLY that one axis's
// magTestRatio[i] < 1.0 check still fails the whole gate (magHealth
// requires ALL three), and - the critical distinction this phase exists
// to get right - the rejection is a bare upstream `return;` with NO
// covariance_init() call, so state/P must be byte-for-byte untouched,
// NOT reset the way tests 4/5's bad-conditioning/healthyFusion failures
// are. Test 8 locates the real gate boundary by bisection (rather than
// hand-deriving the exact closed-form varInnovMag[0] under covariance_
// init()'s real, un-engineered P) and confirms a reading just inside it
// still fuses normally - regression coverage against phase 5's existing
// fusion behavior.
//
// Test strategy (per the ticket's own acceptance criteria):
//   1. A mag reading exactly consistent with the current attitude/field
//      state produces zero innovation on all 3 axes, so the state is left
//      EXACTLY unchanged (a correction that is a Kalman gain times an
//      exactly-zero innovation is exactly zero, regardless of the gain
//      magnitude) - the textbook "nothing to correct" case.
//   2. Regression coverage for the banner's own claim ("A REAL, NOTABLE
//      CONSEQUENCE OF inhibit_mag_states DEFAULTING TO true"): with
//      default settings, an inconsistent reading corrects attitude
//      (quat) but leaves earth_magfield/body_magfield EXACTLY untouched -
//      because kalman_mask never sets bits 16-21 while inhibit_mag_states
//      is true, verified directly here, not just asserted in the banner.
//   3. With inhibit_mag_states explicitly cleared, an inconsistent
//      reading measurably corrects earth_magfield toward the true field
//      over repeated fusion calls - the mag-field-learning path the
//      banner's item 2 above shows is masked off by default.
//   4. The "badly conditioned" abort path (varInnovMag[i] < R_MAG) is
//      engineered directly (a deliberately negative, non-physical
//      covariance entry - the ticket's own suggested "singular/
//      degenerate P at entry") and confirmed to fully reset P to a fresh
//      covariance_init() result, not merely leave it corrupted or
//      partially updated.
//   5. The second, independent abort path (the healthyFusion KHP[i][i] >
//      P[i][i] guard) is engineered SEPARATELY from #4 - a non-physical
//      cross-correlation term chosen (via the same SH_MAG-simplifies-to-
//      mostly-zero algebra worked out for test 4/5's fixture, see the
//      comment below) to leave varInnovMag[0] untouched while blowing up
//      one Kalman-gain entry - confirming this port reproduces BOTH real
//      upstream abort conditions distinctly, both landing on the same
//      CovarianceInit()-and-abort behavior.
//   6. A closed-loop test analogous to CPP-056's GPS drift-correction
//      test: an unmodeled constant gyro-z bias drives real yaw drift
//      under pure dead reckoning; an identical instance additionally
//      fusing a constant, true-earth-field magnetometer reading (at
//      DEFAULT inhibit_mag_states=true - i.e. relying ONLY on the
//      attitude-correction path test #2 confirms exists) shows measurably
//      less yaw drift over the same run.
//
// Fixture note for tests 4/5: with an identity quaternion (q0=1,
// q1=q2=q3=0) and a zero earth_magfield, SH_MAG collapses to
// {0,0,0,0,0,0,1,0,0} (verified by hand from the real SH_MAG formulas in
// ekf_core.cpp/upstream) - which in turn collapses the X-axis
// varInnovMag/H_MAG/SK_MX/Kfusion formulas down to:
//   varInnovMag[0] = P[19][19] + R_MAG + 2*P[16][19] + P[16][16]
//   Kfusion[i]     = (P[i][19] + P[i][16]) / varInnovMag[0]
// (both re-derived independently from the real transcribed formulas in
// ekf_core.cpp, not assumed) - which is what makes it possible to
// engineer #4 and #5 as clean, ISOLATED single-variable perturbations
// rather than fighting the full dense algebra.

#include <array>
#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/ekf/ekf_core.hpp>

using namespace fwcpp::ekf;

namespace {
constexpr ftype kGravity = static_cast<ftype>(9.80665);

// A valid, non-degenerate GyroSample for R_MAG's angular-rate-scaling
// term - delta_angle_dt must be nonzero (it is a divisor in R_MAG's real
// formula) for every test below.
GyroSample make_gyro(ftype dt) {
    GyroSample gyro;
    gyro.delta_angle = Vector3F(ftype(0), ftype(0), ftype(0));
    gyro.delta_angle_dt = dt;
    return gyro;
}
} // namespace

TEST_CASE("fuse_magnetometer: a reading consistent with the current state leaves state exactly unchanged",
          "[ekf_core][mag_fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));  // identity - DCM = I
    ekf.state.earth_magfield = Vector3F(ftype(0.2), ftype(0.1), ftype(0.4));
    ekf.state.body_magfield = Vector3F(ftype(0.01), ftype(-0.02), ftype(0.03));
    ekf.covariance_init(ftype(0.01));

    const StateVector state_before = ekf.state;

    // With DCM = I, MagPred = earth_magfield + body_magfield exactly -
    // feed exactly that as the "measurement" so innovMag = 0 on all 3
    // axes (upstream: innovMag = MagPred - magDataDelayed.mag).
    MagSample mag;
    mag.mag = ekf.state.earth_magfield + ekf.state.body_magfield;

    const bool applied = ekf.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01));

    REQUIRE(applied);
    REQUIRE(static_cast<double>(ekf.innov_mag.x) == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(static_cast<double>(ekf.innov_mag.y) == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(static_cast<double>(ekf.innov_mag.z) == Catch::Approx(0.0).margin(1e-9));

    // Correction = Kfusion * innovation; innovation is exactly 0 on every
    // axis, so every field except quat (which goes through an
    // unconditional renormalize even when nothing moved it) must be
    // bit-for-bit unchanged.
    REQUIRE(ekf.state.velocity.x == state_before.velocity.x);
    REQUIRE(ekf.state.velocity.y == state_before.velocity.y);
    REQUIRE(ekf.state.velocity.z == state_before.velocity.z);
    REQUIRE(ekf.state.position.x == state_before.position.x);
    REQUIRE(ekf.state.gyro_bias.x == state_before.gyro_bias.x);
    REQUIRE(ekf.state.accel_bias.x == state_before.accel_bias.x);
    REQUIRE(ekf.state.earth_magfield.x == state_before.earth_magfield.x);
    REQUIRE(ekf.state.earth_magfield.y == state_before.earth_magfield.y);
    REQUIRE(ekf.state.earth_magfield.z == state_before.earth_magfield.z);
    REQUIRE(ekf.state.body_magfield.x == state_before.body_magfield.x);
    REQUIRE(ekf.state.body_magfield.y == state_before.body_magfield.y);
    REQUIRE(ekf.state.body_magfield.z == state_before.body_magfield.z);
    REQUIRE(static_cast<double>(ekf.state.quat.q1) == Catch::Approx(static_cast<double>(state_before.quat.q1)).margin(1e-9));
    REQUIRE(static_cast<double>(ekf.state.quat.q2) == Catch::Approx(static_cast<double>(state_before.quat.q2)).margin(1e-9));
    REQUIRE(ekf.state.quat.is_unit_length());
}

TEST_CASE("fuse_magnetometer: at default settings, an inconsistent reading corrects attitude but never "
          "touches earth_magfield/body_magfield",
          "[ekf_core][mag_fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    // A nonzero earth_magfield is required for the attitude Jacobian to
    // be observable at all: with earth_magfield == 0 (a fresh EkfCore's
    // real default), rotating a zero vector by any DCM is still zero, so
    // SH_MAG[0]/[1]/[2]/[7]/[8] (and therefore H_MAG[0..3]) are all
    // exactly 0 regardless of attitude - a mathematically genuine
    // observability gap, not a bug, verified by hand from the real
    // SH_MAG formula. A plausible "already has a reasonable earth-field
    // estimate" starting point (as a real system would have from an
    // initial yaw-alignment step - out of this ticket's scope, see
    // banner) is what makes attitude observable from a mag reading at
    // all, exactly like a real magnetometer.
    ekf.state.earth_magfield = Vector3F(ftype(0.25), ftype(0.0), ftype(0.4));
    REQUIRE(ekf.inhibit_mag_states);  // default, unchanged since phase 2 - the condition under test
    ekf.covariance_init(ftype(0.01));

    // Predicted reading (DCM = I) is exactly earth_magfield +
    // body_magfield(=0) = (0.25, 0, 0.4); feed a measurement that
    // disagrees only in Y - exactly what a real magnetometer would read
    // if the true heading were slightly off from what the filter
    // currently believes - so there is real innovation to attribute to
    // attitude specifically.
    MagSample mag;
    mag.mag = Vector3F(ftype(0.25), ftype(0.05), ftype(0.4));

    const bool applied = ekf.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01));

    REQUIRE(applied);
    // The real, banner-documented consequence: kalman_mask never sets
    // bits 16-21 while inhibit_mag_states is true, so Kfusion[16..21] is
    // exactly 0 on every axis - these must be bit-for-bit untouched.
    REQUIRE(ekf.state.earth_magfield.x == ftype(0.25));
    REQUIRE(ekf.state.earth_magfield.y == ftype(0.0));
    REQUIRE(ekf.state.earth_magfield.z == ftype(0.4));
    REQUIRE(ekf.state.body_magfield.x == ftype(0));
    REQUIRE(ekf.state.body_magfield.y == ftype(0));
    REQUIRE(ekf.state.body_magfield.z == ftype(0));
    // But attitude (bits 0-3, never masked) DOES move - a nonzero
    // innovation with a nonzero Kalman gain on the always-active
    // quaternion block must correct something.
    const bool quat_moved = std::abs(static_cast<double>(ekf.state.quat.q2)) > 1e-9 ||
                             std::abs(static_cast<double>(ekf.state.quat.q3)) > 1e-9 ||
                             std::abs(static_cast<double>(ekf.state.quat.q4)) > 1e-9;
    REQUIRE(quat_moved);
    REQUIRE(ekf.state.quat.is_unit_length());
}

TEST_CASE("fuse_magnetometer: with inhibit_mag_states cleared, earth_magfield learning is no longer capped "
          "at one call - CPP-065 phase 11 fixed the constrain_variances() gap this test used to document",
          "[ekf_core][mag_fusion]") {
    // HISTORY / CPP-065 UPDATE: this test used to document a real,
    // disclosed gap (CPP-059 phase 5): constrain_variances() unconditionally
    // zeroed P's mag/wind rows/cols 16-23 EVERY call regardless of
    // inhibit_mag_states's actual value, so clearing inhibit_mag_states
    // only ever unlocked ONE call's worth of earth-field learning (a
    // second call's Kalman gain for those states was provably ~0). CPP-065
    // phase 11 fixed this directly: constrain_variances() now clamps
    // (rather than zeros) P[16..21]'s diagonal to [0, 0.01] whenever
    // inhibit_mag_states is clear (see ekf_core.cpp's own "CPP-065 phase
    // 11" banner in constrain_variances()) - re-verified empirically here,
    // not assumed: the second call now moves earth_magfield by a REAL,
    // material amount (~20% of the first call's movement, not the old
    // provably-negligible ~0), and learning continues with diminishing
    // returns over further calls (expected Kalman behavior - see the
    // in-test comments below for the exact measured trajectory). Note this
    // fixture never calls covariance_prediction() between fuse_magnetometer()
    // calls, so P[16..18] only ever shrinks here (no fresh process noise
    // between calls) - a real closed-loop run (interleaving
    // covariance_prediction(), which now DOES add mag process noise per
    // CPP-065) can behave differently; this test is specifically isolating
    // the constrain_variances() fix, not re-measuring the full closed-loop
    // picture (see ekf_closed_loop_test.cpp for that).
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.inhibit_mag_states = false;  // unlock mag-field learning for this test
    ekf.covariance_init(ftype(0.01));

    // CPP-060 phase 6 ADDENDUM: this fixture's true_field was originally
    // (0.25, 0.0, 0.4) - with a zero-initialized earth_magfield (MagPred
    // = 0), that put the Z-axis innovation (-0.4) past the real,
    // newly-enforced MAG_I_GATE_DEFAULT=300 gate (verified: with this
    // fixture's default P, var_innov_mag is ~0.0075 on every axis - see
    // this file's own tests 7/8 above for how that's derived - so the
    // real per-axis gate boundary here is |innovation| < sqrt(9*0.0075)
    // ~= 0.26; 0.4 fails it, 0.25 barely passes). Shrunk to (0.2, 0.0,
    // 0.15), comfortably inside the real gate on every axis, while still
    // leaving initial_err well above the sanity floor below - this test
    // is about the field-LEARNING behavior once a reading passes the
    // gate, not the gate itself (see tests 7/8 above for that).
    const Vector3F true_field(ftype(0.2), ftype(0.0), ftype(0.15));
    MagSample mag;
    mag.mag = true_field;  // body_magfield stays 0 throughout - isolate earth-field learning

    const double initial_err = static_cast<double>((true_field - ekf.state.earth_magfield).length());
    REQUIRE(initial_err > 0.1);  // sanity: the test starts genuinely wrong, not vacuously close

    REQUIRE(ekf.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01)));
    const double err_after_one = static_cast<double>((true_field - ekf.state.earth_magfield).length());
    // Real, measurable movement toward the true field after the one call
    // that can actually use covariance_init()'s nonzero P[16..21] prior -
    // not just "it ran without crashing".
    REQUIRE(err_after_one < initial_err * ftype(0.95));

    // Confirm the disclosed gap directly: by now P[16..21]'s diagonal has
    // been zeroed by constrain_variances(), so a second call moves
    // earth_magfield by a provably negligible amount versus the first.
    // CPP-065 UPDATE: the old assertion here ("second_call_movement <
    // first_call_movement * 0.05") is now FALSE - empirically re-measured
    // after this ticket's fix (constrain_variances() no longer
    // unconditionally zeros P[16..21] when inhibit_mag_states is clear):
    // second-call movement is ~0.0166 vs. first-call's ~0.0833 (about 20%,
    // not <5%) - a real, measurable SECOND call, not a negligible one.
    // Learning continues over many more calls too (verified directly by
    // probing 10 further calls: error goes 0.1667 -> 0.1501 -> 0.1429 ->
    // 0.1389 -> ... -> 0.1305 by call 11, converging with diminishing
    // returns - expected Kalman behavior since no covariance_prediction()
    // runs between these back-to-back fuse_magnetometer() calls in this
    // fixture, so P[16..18]'s diagonal only ever shrinks here, it never
    // gets fresh process noise to grow from again; a real closed-loop run
    // interleaving covariance_prediction() would behave differently - see
    // the CPP-064-derived closed-loop test in ekf_closed_loop_test.cpp for
    // that scenario). This test now asserts the real, sustained-learning
    // shape: call 2 must move earth_magfield by a MATERIAL fraction of
    // call 1's movement (not the old near-zero cap), and repeated calls
    // must keep reducing the error, converging toward (not fully
    // reaching, in just a few calls) the true field.
    const Vector3F earth_after_one = ekf.state.earth_magfield;
    REQUIRE(ekf.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01)));
    const double err_after_two = static_cast<double>((true_field - ekf.state.earth_magfield).length());
    const double second_call_movement =
        static_cast<double>((ekf.state.earth_magfield - earth_after_one).length());
    const double first_call_movement = initial_err - err_after_one;

    // THE CENTRAL FINDING: the second call is no longer capped to a
    // provably negligible movement - it moves earth_magfield by a real,
    // material fraction (empirically ~20%) of the first call's movement.
    REQUIRE(second_call_movement > first_call_movement * 0.05);
    REQUIRE(err_after_two < err_after_one);

    // Continued learning over several more calls (diminishing returns,
    // per the derivation above, but genuinely still converging - not
    // flatlined at err_after_two the way it would have flatlined at
    // err_after_one before this ticket).
    double err = err_after_two;
    for (int i = 0; i < 5; ++i) {
        REQUIRE(ekf.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01)));
        const double new_err = static_cast<double>((true_field - ekf.state.earth_magfield).length());
        REQUIRE(new_err < err);
        err = new_err;
    }
    // After 7 total calls, error is notably smaller than the ONE-call
    // result this port used to be stuck at (empirically ~0.133 vs.
    // ~0.167) - real, sustained progress, not a one-shot correction.
    REQUIRE(err < err_after_one * 0.85);
}

TEST_CASE("fuse_magnetometer: a badly-conditioned axis resets the covariance matrix and aborts",
          "[ekf_core][mag_fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));

    // Reference: an independent EkfCore with the SAME quat that just
    // calls covariance_init() directly - covariance_init() is a pure
    // function of state.quat/dt_ekf_avg/this object's own noise-parameter
    // fields, so this must be bit-for-bit what fuse_magnetometer()'s
    // internal CovarianceInit()-equivalent call produces too.
    EkfCore reference;
    reference.state.quat = ekf.state.quat;
    reference.covariance_init(ftype(0.01));

    // Engineer a non-physical, deliberately degenerate P[19][19] (the
    // ticket's own suggested "singular/degenerate P at entry") - using
    // this fixture's simplified varInnovMag[0] = P[19][19] + R_MAG +
    // 2*P[16][19] + P[16][16] (see file banner), a large negative
    // P[19][19] alone drives varInnovMag[0] far below R_MAG.
    ekf.P[19][19] = ftype(-1000.0);

    MagSample mag;
    mag.mag = Vector3F(ftype(0), ftype(0), ftype(0));

    const bool applied = ekf.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01));

    REQUIRE_FALSE(applied);
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    reference.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
}

TEST_CASE("fuse_magnetometer: a failed healthyFusion guard (distinct from bad conditioning) also resets P "
          "and aborts",
          "[ekf_core][mag_fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));

    EkfCore reference;
    reference.state.quat = ekf.state.quat;
    reference.covariance_init(ftype(0.01));

    // Using this fixture's simplified formulas (see file banner):
    // varInnovMag[0] = P[19][19] + R_MAG + 2*P[16][19] + P[16][16] does
    // NOT depend on P[2][19] at all, while Kfusion[2] = (P[2][19] +
    // P[2][16]) / varInnovMag[0] DOES. A wildly non-physical P[2][19]
    // (Cauchy-Schwarz would cap |P[2][19]| at sqrt(P[2][2]*P[19][19]),
    // nowhere near 1000) therefore leaves the bad-conditioning check
    // (test above) completely unaffected while blowing up Kfusion[2] -
    // and via KHP[2][2] = Kfusion[2]*(P[16][2] + P[19][2]), driving
    // KHP[2][2] far above P[2][2], tripping the SECOND, independent
    // healthyFusion guard on the very first (X) axis.
    ekf.P[19][2] = ekf.P[2][19] = ftype(1000.0);

    MagSample mag;
    mag.mag = Vector3F(ftype(0), ftype(0), ftype(0));

    const bool applied = ekf.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01));

    REQUIRE_FALSE(applied);
    // Confirm this genuinely was the SECOND failure path, not secretly
    // the first: varInnovMag[0] (computed and stored before this axis's
    // Kalman gain/healthyFusion check ever runs) must still show as
    // "not badly conditioned" by the same R_MAG this call used.
    const ftype r_mag_used = sq(fwcpp::math::constrain_value(ekf.mag_noise, ftype(0.01), ftype(0.5)));
    REQUIRE(static_cast<double>(ekf.var_innov_mag.x) >= static_cast<double>(r_mag_used));
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    reference.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
}

TEST_CASE("EkfCore: magnetometer fusion measurably constrains yaw drift versus pure gyro integration",
          "[ekf_core][mag_fusion]") {
    const ftype dt = ftype(0.01);              // 100 Hz IMU
    const ftype gyro_bias_z_true = ftype(0.01);  // unmodeled constant gyro-z bias, rad/s
    const int total_steps = 4000;              // 40 s total run
    const int mag_period_steps = 50;           // magnetometer sample every 0.5 s
    const Vector3F earth_field_true(ftype(0.25), ftype(0.0), ftype(0.4));

    auto make_ekf = [&]() {
        EkfCore ekf;
        ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
        // Both instances start already knowing the true earth field - a
        // real system would normally have this from an initial yaw-
        // alignment step (out of this ticket's scope, see banner), and
        // the test above already establishes separately that LEARNING
        // the field from a zero cold start is a different, more limited
        // capability (capped at one call by a real, disclosed gap). This
        // test isolates the actual claim under test: given a known
        // field, does fuse_magnetometer() measurably fight gyro-driven
        // yaw drift - exactly the real attitude-Jacobian-observability
        // precondition the test above also establishes (SH_MAG's
        // attitude terms are exactly 0 when earth_magfield is 0).
        ekf.state.earth_magfield = earth_field_true;
        // Default inhibit_mag_states (true) is left UNCHANGED on purpose
        // - this test demonstrates the attitude-only correction path the
        // test above confirms exists even with mag-field learning masked
        // off, matching real-world behavior before a future health/mode
        // phase ever clears inhibitMagStates.
        ekf.covariance_init(dt);
        return ekf;
    };

    EkfCore fused = make_ekf();
    EkfCore unfused = make_ekf();

    GyroSample gyro;
    // Real physical rotation is zero throughout (the vehicle truly stays
    // level and north-pointing) - but the gyro SENSOR reads a constant,
    // unmodeled z-axis bias every step, a textbook attitude-drift source
    // exactly analogous to CPP-056's own unmodeled-accel-bias velocity/
    // position drift test.
    gyro.delta_angle = Vector3F(ftype(0), ftype(0), gyro_bias_z_true * dt);
    gyro.delta_angle_dt = dt;
    AccelSample accel;
    accel.delta_velocity = Vector3F(ftype(0), ftype(0), -kGravity * dt);  // level, gravity-cancelling
    accel.delta_velocity_dt = dt;

    for (int step = 1; step <= total_steps; ++step) {
        fused.update_strapdown_equations_ned(gyro, accel, dt);
        fused.covariance_prediction(gyro, accel, dt);
        unfused.update_strapdown_equations_ned(gyro, accel, dt);
        unfused.covariance_prediction(gyro, accel, dt);

        if (step % mag_period_steps == 0) {
            // Ground truth: the real vehicle never actually rotated, so a
            // real magnetometer on it always reads exactly the true earth
            // field rotated by the TRUE (identity) attitude - i.e. the
            // true field itself, constant every time.
            MagSample mag;
            mag.mag = earth_field_true;
            fused.fuse_magnetometer(mag, gyro, dt);
        }
    }

    const double fused_yaw_err = std::abs(static_cast<double>(fused.state.quat.get_euler_yaw()));
    const double unfused_yaw_err = std::abs(static_cast<double>(unfused.state.quat.get_euler_yaw()));

    // Sanity check the test itself is not vacuous: pure dead reckoning
    // must show real, substantial yaw drift. Expected analytically:
    // yaw error ~= gyro_bias_z_true * T = 0.01*40 = 0.4 rad (~23 deg).
    REQUIRE(unfused_yaw_err > 0.1);

    // The actual point of this ticket: mag fusion must measurably
    // outperform pure prediction over the identical run, using ONLY the
    // attitude-correction path (inhibit_mag_states was never cleared).
    // The improvement is real but genuinely modest (empirically ~28%,
    // stable across a 10x change in fusion frequency during this test's
    // own development - not a fluke of one particular period), not
    // dramatic: with inhibit_mag_states left at its real default, this
    // path only ever couples through the attitude block's OWN covariance
    // (P[0..3][0..3], via nonzero SH_MAG-weighted terms), never through a
    // learned earth/body-field correlation (masked off, see the test
    // above) - a materially weaker feedback loop than a real system
    // would have once a future health/mode phase clears inhibitMagStates.
    // The threshold below reflects that real, verified magnitude, not a
    // rounder-looking number picked without checking what this code
    // actually produces.
    REQUIRE(fused_yaw_err < unfused_yaw_err * 0.85);
}

TEST_CASE("fuse_magnetometer: a single-axis-dominant inconsistency fails the per-axis magTestRatio gate "
          "and leaves state/covariance byte-for-byte untouched (NOT a covariance_init() reset)",
          "[ekf_core][mag_fusion]") {
    // Identity attitude, zero earth_magfield/body_magfield (a fresh
    // EkfCore's real defaults) - collapses SH_MAG to {0,0,0,0,0,0,1,0,0}
    // (see file banner above tests 4/5), so MagPred = (0,0,0) exactly and
    // innov_mag = -mag.mag on every axis, with var_innov_mag depending
    // only on P (from covariance_init(), never hand-engineered here) and
    // NOT on the injected reading at all - i.e. the bad-conditioning
    // checks above the new gate are completely unaffected by how large
    // the injected reading is, only the NEW gate is exercised here.
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));

    const StateVector state_before = ekf.state;
    const Matrix24 P_before = ekf.P;

    // A large error concentrated on ONE axis (X), with Y/Z exactly
    // consistent (0 innovation) - exploiting the real, verified-directly
    // per-axis (not combined sum-of-squares, unlike CPP-057's GPS gate)
    // magHealth structure: a single bad axis must fail the whole gate on
    // its own, per the ticket's own instruction.
    MagSample mag;
    mag.mag = Vector3F(ftype(1000), ftype(0), ftype(0));

    const bool applied = ekf.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01));

    REQUIRE_FALSE(applied);

    // Confirm this is genuinely the NEW gate, not one of CPP-059's
    // existing abort paths: var_innov_mag.x must still show as "not
    // badly conditioned" (>= R_MAG, matching test 4's own check), and
    // the per-axis test ratio itself must show X failing while Y/Z pass
    // comfortably - the single-axis-dominant structure this test is
    // built to exploit.
    const ftype r_mag_used = sq(fwcpp::math::constrain_value(ekf.mag_noise, ftype(0.01), ftype(0.5)));
    REQUIRE(static_cast<double>(ekf.var_innov_mag.x) >= static_cast<double>(r_mag_used));
    const Vector3F ratio = ekf.mag_test_ratio();
    REQUIRE(static_cast<double>(ratio.x) >= 1.0);
    REQUIRE(static_cast<double>(ratio.y) < 1.0);
    REQUIRE(static_cast<double>(ratio.z) < 1.0);

    // THE critical distinction this phase exists to get right: upstream
    // is a bare `return;` here, with NO CovarianceInit() call - unlike
    // tests 4/5's covariance-reset abort paths, state/P must be
    // EXACTLY, byte-for-byte what they were at entry, not merely
    // "changed less" or "reset to a fresh covariance_init() result".
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    P_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
    REQUIRE(ekf.state.quat.q1 == state_before.quat.q1);
    REQUIRE(ekf.state.quat.q2 == state_before.quat.q2);
    REQUIRE(ekf.state.quat.q3 == state_before.quat.q3);
    REQUIRE(ekf.state.quat.q4 == state_before.quat.q4);
    REQUIRE(ekf.state.velocity.x == state_before.velocity.x);
    REQUIRE(ekf.state.velocity.y == state_before.velocity.y);
    REQUIRE(ekf.state.velocity.z == state_before.velocity.z);
    REQUIRE(ekf.state.position.x == state_before.position.x);
    REQUIRE(ekf.state.position.y == state_before.position.y);
    REQUIRE(ekf.state.position.z == state_before.position.z);
    REQUIRE(ekf.state.gyro_bias.x == state_before.gyro_bias.x);
    REQUIRE(ekf.state.gyro_bias.y == state_before.gyro_bias.y);
    REQUIRE(ekf.state.gyro_bias.z == state_before.gyro_bias.z);
    REQUIRE(ekf.state.accel_bias.x == state_before.accel_bias.x);
    REQUIRE(ekf.state.accel_bias.y == state_before.accel_bias.y);
    REQUIRE(ekf.state.accel_bias.z == state_before.accel_bias.z);
    REQUIRE(ekf.state.earth_magfield.x == state_before.earth_magfield.x);
    REQUIRE(ekf.state.earth_magfield.y == state_before.earth_magfield.y);
    REQUIRE(ekf.state.earth_magfield.z == state_before.earth_magfield.z);
    REQUIRE(ekf.state.body_magfield.x == state_before.body_magfield.x);
    REQUIRE(ekf.state.body_magfield.y == state_before.body_magfield.y);
    REQUIRE(ekf.state.body_magfield.z == state_before.body_magfield.z);
}

TEST_CASE("fuse_magnetometer: a borderline-inside-gate reading still fuses normally (regression vs. "
          "phase 5's existing fusion behavior)",
          "[ekf_core][mag_fusion]") {
    // Locate the real gate boundary along a single-axis (X) injected
    // reading by bisection, against covariance_init()'s real,
    // un-engineered P - rather than hand-deriving the exact closed-form
    // varInnovMag[0] value, which is legitimate for tests 4/5's
    // hand-engineered P but would be circular here (this test's whole
    // point is to confirm the real formula end to end, boundary
    // included). Each probe uses a completely fresh EkfCore so a
    // rejected (state-untouched, per the test above) or accepted
    // (state-mutated) trial never contaminates the next one.
    auto make_fixture = [] {
        EkfCore ekf;
        ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
        ekf.covariance_init(ftype(0.01));
        return ekf;
    };

    auto ratio_x_for = [&](ftype mag_x) {
        EkfCore probe = make_fixture();
        MagSample mag;
        mag.mag = Vector3F(mag_x, ftype(0), ftype(0));
        probe.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01));
        return probe.mag_test_ratio().x;
    };

    REQUIRE(static_cast<double>(ratio_x_for(ftype(0))) < 1.0);  // zero innovation: sanity floor
    ftype lo = ftype(0);       // ratio(lo) < 1.0, invariant maintained throughout
    ftype hi = ftype(1000);    // must be comfortably >= 1.0 - checked below, matches test above
    REQUIRE(static_cast<double>(ratio_x_for(hi)) >= 1.0);
    for (int iter = 0; iter < 60; ++iter) {
        const ftype mid = (lo + hi) / ftype(2);
        if (static_cast<double>(ratio_x_for(mid)) < 1.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    // `lo` is now a reading whose per-axis test ratio is just inside the
    // gate (< 1.0, converged to within 2^-60 of the real boundary) - the
    // exact "borderline-inside-gate" case the ticket's acceptance
    // criteria ask for. It must still fuse exactly as phase 5 already
    // established for a smaller, non-borderline inconsistent reading.
    EkfCore ekf = make_fixture();
    MagSample mag;
    mag.mag = Vector3F(lo, ftype(0), ftype(0));

    const bool applied = ekf.fuse_magnetometer(mag, make_gyro(ftype(0.01)), ftype(0.01));

    REQUIRE(applied);
    REQUIRE(static_cast<double>(ekf.mag_test_ratio().x) < 1.0);
}


// ============================================================================
// CPP-068 PHASE 14: time-correct magnetometer sample recall via ObsBuffer -
// extends CPP-067 (phase 13)'s identical GPS pattern to magnetometer
// fusion. See ekf_core.hpp's "CPP-068, PHASE 14" banner (above
// EkfCore::mag_buffer) for the full scope/reasoning: the deliberately
// narrower recall-against-caller's-own-now_s design (same simplification
// CPP-067 already disclosed for GPS), the buffer-size justification
// (independently re-derived for magnetometer's own real 10Hz closed-loop-
// test rate, not copied from GPS's N=4 unexamined), the verified-moot
// second storedMag.recall() call site (learnMagBiasFromGPS(), already an
// established CPP-059 exclusion - this port has no such function, so
// recall_mag_sample() is the ONLY consumer of mag_buffer), and why
// fuse_magnetometer() above is kept completely unchanged - the new
// capability is delivered entirely by the two additions exercised here:
// push_mag_sample()/recall_mag_sample().
//
// Test strategy (mirrors CPP-067's own GPS buffer test shape exactly):
//   1. MagSample::set_time_s()/time_s() round-trip at millisecond
//      resolution (the disclosed ObsElement::time_ms quantization,
//      identical convention to GpsSample's own).
//   2. recall_mag_sample() on an empty buffer returns false and leaves
//      `out` untouched.
//   3. THE REAL, NEW CAPABILITY: push several magnetometer samples at
//      DIFFERENT, non-tick-aligned timestamps (simulating a real compass
//      driver reading arriving asynchronously relative to the EKF's own
//      scheduling, exactly as upstream's readMagData()/SelectMagFusion()
//      are two independently-scheduled functions), then confirm fusion
//      at each 50Hz tick correctly recalls the most-recently-available
//      sample and successfully fuses it against a valid attitude/field
//      fixture - the test never calls push/recall at matching times, and
//      never hand-feeds fuse_magnetometer() "the right sample" the way
//      every other test above this section does.
// ============================================================================

TEST_CASE("MagSample::set_time_s/time_s round-trips at millisecond resolution", "[ekf_core][mag_fusion][mag_buffer]") {
    MagSample mag;
    REQUIRE(static_cast<double>(mag.time_s()) == Catch::Approx(0.0));  // zero-initialized ObsElement::time_ms

    mag.set_time_s(ftype(1.234));
    REQUIRE(static_cast<double>(mag.time_s()) == Catch::Approx(1.234).margin(1e-6));

    mag.set_time_s(ftype(0.0));
    REQUIRE(static_cast<double>(mag.time_s()) == Catch::Approx(0.0));

    // Defensive negative clamp (see MagSample::set_time_s()'s own doc
    // comment - now_s is never legitimately negative in this port's own
    // convention, but the clamp avoids UB rather than assuming callers
    // never pass one) - identical convention to GpsSample's own.
    mag.set_time_s(ftype(-5.0));
    REQUIRE(static_cast<double>(mag.time_s()) == Catch::Approx(0.0));
}

TEST_CASE("recall_mag_sample: returns false on an empty buffer", "[ekf_core][mag_fusion][mag_buffer]") {
    EkfCore ekf;
    MagSample out;
    out.mag = Vector3F(ftype(99.0), ftype(99.0), ftype(99.0));  // sentinel - must be untouched

    REQUIRE_FALSE(ekf.recall_mag_sample(out, ftype(0.05)));
    REQUIRE(ekf.mag_buffer.empty());
    // Untouched on failure, matching ObsBuffer::recall()'s own documented
    // contract (ekf_buffer.hpp).
    REQUIRE(static_cast<double>(out.mag.x) == Catch::Approx(99.0));
}

TEST_CASE("push_mag_sample/recall_mag_sample: recalls the correct sample under realistic "
          "asynchronous (non-tick-aligned) magnetometer arrival, without the caller hand-feeding "
          "exactly the right sample synchronously",
          "[ekf_core][mag_fusion][mag_buffer]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));  // identity - DCM = I
    ekf.state.earth_magfield = Vector3F(ftype(0.2), ftype(0.1), ftype(0.4));
    ekf.state.body_magfield = Vector3F(ftype(0.01), ftype(-0.02), ftype(0.03));
    ekf.covariance_init(ftype(0.01));

    constexpr ftype kDt = ftype(0.02);  // 50Hz IMU tick, matching this port's own
                                         // closed-loop-test precedent (ekf_closed_loop_test.cpp).

    // Four magnetometer readings, each carrying its own TRUE arrival
    // timestamp - none of them a multiple of kDt (0.02s), i.e. none would
    // ever land exactly on an IMU tick boundary (verified: 91, 233, 456,
    // 719 mod 20 are all nonzero). Each carries a distinct, tagged
    // `mag.x` offset from the exactly-consistent reading (see
    // ekf.state.earth_magfield/body_magfield above and this file's own
    // "consistent with current state" fixture pattern) so a successful
    // recall can be matched back to exactly which reading it returned.
    struct Fix {
        ftype arrival_s;
        ftype tag;
        bool pushed = false;
    };
    std::array<Fix, 4> fixes{{
        {ftype(0.091), ftype(1.0)},
        {ftype(0.233), ftype(2.0)},
        {ftype(0.456), ftype(3.0)},
        {ftype(0.719), ftype(4.0)},
    }};

    const Vector3F consistent_mag = ekf.state.earth_magfield + ekf.state.body_magfield;

    std::vector<ftype> recalled_tags;
    std::vector<ftype> recall_latency_s;  // now_s at recall time - the sample's own arrival_s

    for (int tick = 1; tick <= 60; ++tick) {  // 60 * 20ms = 1.2s, comfortably past the last reading
        const ftype now_s = static_cast<ftype>(tick) * kDt;

        // --- Sample arrival: modelled as an independently-scheduled event
        // (upstream: readMagData(), its own function, called on its own
        // terms) that pushes into mag_buffer AS SOON AS it has arrived,
        // stamped with its OWN true arrival time - NOT now_s, and NOT
        // aligned to the tick grid at all. ---
        for (Fix& fix : fixes) {
            if (!fix.pushed && now_s >= fix.arrival_s) {
                MagSample mag;
                mag.set_time_s(fix.arrival_s);
                mag.mag = Vector3F(consistent_mag.x + fix.tag, consistent_mag.y, consistent_mag.z);
                ekf.push_mag_sample(mag);
                fix.pushed = true;
            }
        }

        // --- Fusion attempted EVERY tick (upstream: SelectMagFusion()
        // runs every EKF cycle) - recall_mag_sample() decides, purely
        // from timestamps, whether a time-eligible sample exists yet. THE
        // TEST NEVER TELLS IT WHICH TICK TO EXPECT A MATCH ON - that is
        // exactly the capability this ticket adds (contrast with every
        // fuse_magnetometer(mag, ...) call above this section, which
        // hands over a hand-built MagSample directly, on the exact call
        // the test chooses). ---
        MagSample recalled;
        if (ekf.recall_mag_sample(recalled, now_s)) {
            recalled_tags.push_back(recalled.mag.x - consistent_mag.x);
            recall_latency_s.push_back(now_s - recalled.time_s());

            // The full pipeline: fuse the recalled sample exactly like
            // any other MagSample - fuse_magnetometer() itself is
            // completely unchanged by this ticket.
            ekf.fuse_magnetometer(recalled, make_gyro(ftype(0.01)), kDt);
        }
    }

    // All 4 asynchronously-arriving readings were recalled exactly once
    // each, in arrival order, none lost and none double-counted - the
    // real acceptance criterion: fusion recalled the right sample at the
    // right (later, tick-quantized) time without ever being handed it
    // directly.
    REQUIRE(recalled_tags.size() == 4);
    REQUIRE(static_cast<double>(recalled_tags[0]) == Catch::Approx(1.0));
    REQUIRE(static_cast<double>(recalled_tags[1]) == Catch::Approx(2.0));
    REQUIRE(static_cast<double>(recalled_tags[2]) == Catch::Approx(3.0));
    REQUIRE(static_cast<double>(recalled_tags[3]) == Catch::Approx(4.0));

    // Each recall happened PROMPTLY after its sample's true arrival - at
    // most one tick (kDt) of latency, and never before arrival.
    for (const ftype latency : recall_latency_s) {
        REQUIRE(static_cast<double>(latency) >= 0.0);
        REQUIRE(static_cast<double>(latency) < static_cast<double>(kDt) + 1e-9);
    }

    // The buffer is drained back to empty - every pushed sample was
    // consumed by exactly one recall, none left stranded.
    REQUIRE(ekf.mag_buffer.empty());
}
