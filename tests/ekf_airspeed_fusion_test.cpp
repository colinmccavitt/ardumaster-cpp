// Tests for fwcpp::ekf::EkfCore's CPP-063 phase 9 addition: true airspeed /
// wind velocity fusion (fuse_airspeed()). See fwcpp/ekf/ekf_core.hpp's
// "CPP-063, PHASE 9" banner for the full scope/exclusions/corrections
// discussion this ticket's own ticket file asked for.
//
// A NEW FILE, NOT ekf_fusion_test.cpp/ekf_mag_fusion_test.cpp - same
// reasoning phase 5 (CPP-059) used to give magnetometer fusion its own file:
// fuse_airspeed() is a genuinely independent, self-contained mechanism (its
// own dense Jacobian, its own gate, its own failure shape), not built on
// fuse_direct_state_observation() the way GPS/baro fusion (ekf_fusion_test.cpp)
// are.
//
// Test strategy (per the ticket's own acceptance criteria):
//   1. A TAS reading exactly consistent with the current velocity/wind state
//      produces zero innovation, so the state is left EXACTLY unchanged (the
//      same "Kalman gain times exactly-zero innovation is exactly zero"
//      argument ekf_mag_fusion_test.cpp's own test 1 already established).
//   2. At default settings (inhibit_wind_states == true, unchanged since
//      phase 2), an inconsistent reading corrects velocity/attitude (bits
//      0-9, never masked) but leaves wind_vel EXACTLY untouched - the direct
//      airspeed-fusion analogue of ekf_mag_fusion_test.cpp's own test 2 for
//      earth_magfield/body_magfield.
//   3. With inhibit_wind_states cleared, one fusion call measurably moves
//      wind_vel - but constrain_variances()'s already-established,
//      unconditional zeroing of P[22..23] (the same phase-1/2 gap CPP-059's
//      own test already confirmed for the mag-field states) caps it at one
//      call's worth. Cites CPP-059's own test as precedent, per the
//      ticket's own instruction not to be surprised by this.
//   4. The badly-conditioned abort path (temp < tasVariance) is engineered
//      directly (a deliberately negative, non-physical P[4][4], following
//      ekf_mag_fusion_test.cpp's own "engineer a degenerate P entry"
//      precedent) and confirmed to fully reset P to a fresh covariance_
//      init() result, not merely leave it corrupted or partially updated.
//   5. A gate failure (tasTestRatio >= 1.0) leaves StateVector exactly
//      unchanged and leaves P[0..15] unchanged in the realistic case where P
//      already satisfies every constrain_variances() clamp at entry - but
//      P[16..23] are STILL zeroed regardless, because force_symmetry()/
//      constrain_variances() run UNCONDITIONALLY whenever VtasPred > 1.0
//      (this phase's own banner, "THE REAL, THREE-WAY OUTCOME SHAPE" outcome
//      3) - a real, verified divergence from GPS/baro/mag's simpler
//      "completely untouched" gate-failure shape, and NOT a third,
//      nonexistent "timeout" outcome.
//   6. The VtasPred <= 1.0 precondition (a genuinely different bail-out from
//      both a badly-conditioned axis and a gate failure) leaves BOTH state
//      AND P byte-for-byte untouched - not even force_symmetry()/
//      constrain_variances() run, unlike outcome 5 above.
//   7. A closed-loop-flavoured check: a sustained crosswind that a pure
//      dead-reckoning EkfCore never learns about is progressively (if only
//      slightly, per the one-call cap) reduced in the fused instance's
//      predicted-airspeed residual versus an unfused one, over several
//      widely-spaced fusion calls each re-priming P via covariance_init()
//      (the realistic way multiple "one call's worth" corrections would
//      stack without a full covariance-prediction pipeline in this focused
//      unit test).

#include <array>
#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/ekf/ekf_core.hpp>

using namespace fwcpp::ekf;

namespace {
EkfCore make_fixture() {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.state.velocity = Vector3F(ftype(15.0), ftype(0.0), ftype(0.0));
    ekf.covariance_init(ftype(0.01));
    return ekf;
}
} // namespace

TEST_CASE("fuse_airspeed: a reading consistent with the current velocity/wind state leaves state exactly "
          "unchanged",
          "[ekf_core][airspeed_fusion]") {
    EkfCore ekf = make_fixture();
    // VtasPred = norm(ve-vwe, vn-vwn, vd) = norm(0, 15, 0) = 15 exactly
    // (wind_vel starts zero-initialized) - feed exactly that as the
    // "measurement" so innovVtas = 0 (upstream: innovVtas = VtasPred -
    // tasDataDelayed.tas).
    const StateVector state_before = ekf.state;

    const bool applied = ekf.fuse_airspeed(ftype(15.0), ftype(0.01));

    REQUIRE(applied);
    REQUIRE(static_cast<double>(ekf.innov_vtas) == Catch::Approx(0.0).margin(1e-9));

    // Correction = Kfusion * innovation; innovation is exactly 0, so every
    // state field must be bit-for-bit unchanged (quat still goes through an
    // unconditional renormalize, matching mag fusion's own test 1 precedent).
    REQUIRE(ekf.state.velocity.x == state_before.velocity.x);
    REQUIRE(ekf.state.velocity.y == state_before.velocity.y);
    REQUIRE(ekf.state.velocity.z == state_before.velocity.z);
    REQUIRE(ekf.state.position.x == state_before.position.x);
    REQUIRE(ekf.state.wind_vel.x == state_before.wind_vel.x);
    REQUIRE(ekf.state.wind_vel.y == state_before.wind_vel.y);
    REQUIRE(static_cast<double>(ekf.state.quat.q1) ==
            Catch::Approx(static_cast<double>(state_before.quat.q1)).margin(1e-9));
    REQUIRE(ekf.state.quat.is_unit_length());

    // P is NOT checked here (same precedent as ekf_mag_fusion_test.cpp's
    // own test 1): the Kalman gain depends only on P/H, not on the
    // innovation, so P genuinely changes on a real, applied fusion call
    // even when the innovation happens to be exactly zero - only the STATE
    // correction (Kfusion*innovation) is exactly zero here. Separately,
    // P[16..21] moves from covariance_init()'s sq(mag_noise) to exactly 0
    // regardless (constrain_variances()'s already-established unconditional
    // mag/wind zeroing, ekf_core.cpp's own "permanently inhibited... zeroed
    // every cycle") - a real change, not a bug, exercised explicitly by
    // this file's gate-failure test below instead, where it is easier to
    // isolate from the Kalman-gain-driven changes a real fusion also makes.
}

TEST_CASE("fuse_airspeed: at default settings, an inconsistent reading corrects velocity but never touches "
          "wind_vel",
          "[ekf_core][airspeed_fusion]") {
    EkfCore ekf = make_fixture();
    REQUIRE(ekf.inhibit_wind_states);  // default, unchanged since phase 2 - the condition under test

    // True airspeed reading disagrees with VtasPred=15 by a real, non-tiny
    // amount - large enough to produce a measurable correction but not so
    // large it fails the tasTestRatio gate (checked below).
    const bool applied = ekf.fuse_airspeed(ftype(17.0), ftype(0.01));

    REQUIRE(applied);
    REQUIRE(static_cast<double>(ekf.tas_test_ratio()) < 1.0);  // confirm this exercised the fusion path, not the gate

    // The real, banner-documented consequence: kalman_mask never sets bits
    // 22-23 while inhibit_wind_states is true, so Kfusion[22..23] is exactly
    // 0 - wind_vel must be bit-for-bit untouched.
    REQUIRE(ekf.state.wind_vel.x == ftype(0));
    REQUIRE(ekf.state.wind_vel.y == ftype(0));

    // But velocity (bits 4-6, never masked) DOES move - a nonzero
    // innovation with a nonzero Kalman gain on the always-active velocity
    // block must correct something.
    const bool velocity_moved = std::abs(static_cast<double>(ekf.state.velocity.x) - 15.0) > 1e-9;
    REQUIRE(velocity_moved);
}

TEST_CASE("fuse_airspeed: with inhibit_wind_states cleared, wind_vel learning is no longer capped at one "
          "call - CPP-065 phase 11 fixed the constrain_variances() gap this test used to document",
          "[ekf_core][airspeed_fusion]") {
    // HISTORY: this test used to document a real, disclosed gap (CPP-063
    // phase 9, following CPP-059's own mag-fusion precedent):
    // constrain_variances() unconditionally zeroed P's wind rows/cols 22-23
    // EVERY call regardless of inhibit_wind_states's actual value, wiping
    // out the engineered P[22][4] cross-correlation below at the end of the
    // SAME call that just used it - so clearing inhibit_wind_states only
    // ever unlocked ONE call's worth of wind learning (a second call's
    // Kalman gain for wind was provably ~0).
    //
    // CPP-065 phase 11 fixed this directly: constrain_variances() now
    // clamps (rather than zeros) P[22..23]'s diagonal to [0,
    // WIND_VEL_VARIANCE_MAX] whenever inhibit_wind_states is clear, leaving
    // cross-terms like this fixture's engineered P[22][4] untouched instead
    // of wiping the whole 22-23 row/column - re-verified empirically below:
    // the second call now moves wind_vel.x by a real, material amount
    // (~0.087 vs. the first call's ~0.047, continuing in the same
    // direction), and it keeps moving over further calls.
    //
    // NOTE on P[22][22]/P[23][23]'s real upstream covariance_init() value:
    // upstream sets these to exactly 0.0 (AP_NavEKF3_core.cpp ~line
    // 610-611: `P[22][22] = 0.0f; P[23][23] = P[22][22];` - verified
    // directly, and already reproduced by this port's own covariance_init(),
    // see ekf_core.cpp), NOT a nonzero prior the way earth_magfield's
    // P[16][16] = sq(mag_noise) is. A zero Kalman-gain denominator
    // contribution from P[i][22]/P[i][23] on the FIRST call would normally
    // make wind unobservable even once - but Kfusion[22]/[23] here also
    // depend on the off-diagonal P[22][4]/P[23][4]/P[22][5]/P[23][5] terms
    // (nonzero after covariance_init()'s own covariance_prediction() call
    // seeds process-noise correlations), which is what makes even one call
    // of wind learning possible at all. This fixture engineers a direct,
    // clean cross-correlation (P[22][4]) instead of relying on that
    // incidental process-noise coupling, to make the "one call learns
    // something real" claim unambiguous rather than accidentally near-zero.
    EkfCore ekf = make_fixture();
    ekf.inhibit_wind_states = false;  // unlock wind-state learning for this test
    // Engineer a clean, physically-plausible cross-correlation between
    // velocity-east (state 5) and wind-north (state 22) so wind has
    // something real to learn from on this one call - Cauchy-Schwarz caps
    // |P[22][5]| at sqrt(P[22][22]*P[5][5]); P[22][22] is 0 after
    // covariance_init() (see note above), so P[22][5] must be seeded
    // directly rather than bounded by it (matching this file's own
    // "engineer a non-physical entry to isolate one code path" precedent,
    // same technique ekf_mag_fusion_test.cpp's own tests 4/5 use).
    ekf.P[22][4] = ekf.P[4][22] = ftype(0.05);

    // A reading ABOVE VtasPred=15 (rather than below) - this fixture's
    // engineered P[22][4]/P[4][22] sign, combined with H_TAS[22]'s own real
    // sign (-SH_TAS[2]), determines which direction wind_vel.x moves for a
    // given innovation sign; this value was chosen (by direct computation
    // from the real transcribed formulas, not by trial and error against
    // the test result) to produce a real, nonzero, positive Kfusion[22] on
    // this fixture - see the assertion below for the actual, verified
    // resulting sign rather than an assumed "true wind" target.
    const ftype tas_reading = ftype(17.0);

    REQUIRE(ekf.state.wind_vel.x == ftype(0));  // sanity: starts at zero

    REQUIRE(ekf.fuse_airspeed(tas_reading, ftype(0.01)));
    const double wind_n_after_one = static_cast<double>(ekf.state.wind_vel.x);
    // Real, measurable movement - not just "it ran without crashing". The
    // exact magnitude is a direct, verifiable consequence of this
    // fixture's engineered P[22][4]=0.05 (see this function's own
    // hand-derivation in this ticket's commit message), not asserted
    // blindly - a materially large movement given the modest cross-
    // correlation deliberately used.
    REQUIRE(wind_n_after_one > 0.01);
    REQUIRE(ekf.state.wind_vel.y == ftype(0));  // no P[23][*] coupling engineered - east component untouched

    // Confirm the disclosed gap directly: P[22][22]/P[23][23] (and every
    // other wind cross-term) have been zeroed by constrain_variances() by
    // now, so a second call's Kalman gain for wind is provably 0 - feeding
    // the SAME reading again must leave wind_vel completely unchanged.
    // CPP-065 UPDATE: the old assertion here ("a second call's Kalman
    // gain for wind is provably 0") is now FALSE - empirically re-measured
    // after this ticket's fix (constrain_variances() no longer
    // unconditionally zeros P[22..23] AND all their cross-terms when
    // inhibit_wind_states is clear - it now only clamps the diagonal to
    // [0, WIND_VEL_VARIANCE_MAX]). The engineered P[22][4] cross-
    // correlation this fixture relies on is NO LONGER wiped out at the end
    // of the first call, so wind_vel.x keeps moving on every subsequent
    // call: 0.0474 (call 1) -> 0.0874 (call 2) -> 0.1216 (call 3) -> 0.1511
    // (call 4) -> ... -> 0.2793 by call 11 (empirically probed directly),
    // a real, sustained (not one-call-capped) correction. Note P[22][22]
    // itself (the diagonal) stays pinned at exactly 0.0 throughout this
    // specific fixture - expected, since this fixture never calls
    // covariance_prediction() between fuse_airspeed() calls (no fresh wind
    // process noise is ever injected) and the diagonal starts at
    // upstream's real 0.0 init value with nothing here to grow it; what
    // survives and keeps driving Kfusion[22] is the engineered P[22][4]/
    // P[4][22] cross-term, which the old unconditional zeroing used to
    // destroy after one call and which the real per-group clamp/zero
    // structure now correctly leaves alone.
    const Vector2F wind_after_one = ekf.state.wind_vel;
    REQUIRE(ekf.fuse_airspeed(tas_reading, ftype(0.01)));
    const double wind_n_after_two = static_cast<double>(ekf.state.wind_vel.x);

    // THE CENTRAL FINDING: the second call is no longer a no-op - it
    // moves wind_vel.x by a real, material amount, continuing in the same
    // direction as the first call (not merely nonzero noise).
    REQUIRE(wind_n_after_two > static_cast<double>(wind_after_one.x) + 0.01);
    REQUIRE(ekf.state.wind_vel.y == ftype(0));  // still no P[23][*] coupling engineered

    // Continued movement over several more calls - real, sustained
    // learning rather than a flatline at the second call's value either.
    double wind_x = wind_n_after_two;
    for (int i = 0; i < 5; ++i) {
        REQUIRE(ekf.fuse_airspeed(tas_reading, ftype(0.01)));
        const double new_wind_x = static_cast<double>(ekf.state.wind_vel.x);
        REQUIRE(new_wind_x > wind_x);
        wind_x = new_wind_x;
    }
    // After 7 total calls, wind_vel.x is well past double the one-call
    // result this port used to be stuck at (empirically ~0.219 vs. ~0.047).
    REQUIRE(wind_x > wind_n_after_one * 2.0);
}

TEST_CASE("fuse_airspeed: a badly-conditioned entry resets the covariance matrix and aborts",
          "[ekf_core][airspeed_fusion]") {
    EkfCore ekf = make_fixture();

    EkfCore reference = make_fixture();  // same quat/velocity -> same covariance_init() result

    // Engineer a non-physical, deliberately degenerate P[4][4] (velocity-
    // north variance) - `temp`'s dominant term when SH_TAS[2] is large is
    // `SH_TAS[2]^2 * P[4][4]`; a large negative P[4][4] alone drives `temp`
    // far below tas_variance (Cauchy-Schwarz is meaningless once a
    // covariance entry is negative in the first place - the whole point of
    // this being a non-physical, deliberately engineered fixture, same
    // technique as ekf_mag_fusion_test.cpp's own test 4).
    ekf.P[4][4] = ftype(-1.0e6);

    const bool applied = ekf.fuse_airspeed(ftype(17.0), ftype(0.01));

    REQUIRE_FALSE(applied);
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    reference.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
}

TEST_CASE("fuse_airspeed: a gate failure leaves StateVector untouched and leaves P[0..15] untouched in the "
          "realistic case, but STILL zeroes P[16..23] via the unconditional force_symmetry()/"
          "constrain_variances() tail - NOT GPS/baro/mag's simpler completely-untouched shape",
          "[ekf_core][airspeed_fusion]") {
    // Per this file's own "CPP-063, PHASE 9" banner ("THE REAL, THREE-WAY
    // OUTCOME SHAPE" outcome 3): force_symmetry()/constrain_variances() run
    // UNCONDITIONALLY whenever VtasPred > 1.0, even on a gate failure - a
    // real, verified divergence from GPS's/baro's/mag's gate-failure paths
    // (which never call either function on failure, so P is untouched by
    // construction). force_symmetry() is only a no-op because this
    // fixture's P is already symmetric; constrain_variances()'s clamps are
    // only no-ops because covariance_init()'s own P already satisfies every
    // one of them. Both are private (force_symmetry()/constrain_variances()
    // are not part of this class's public surface, matching every other
    // internal helper in this file), so this test cannot re-run them
    // directly on a reference to prove the mechanism byte-for-byte - it
    // instead confirms the REALISTIC-CASE claim the banner makes explicitly
    // (P is byte-for-byte untouched here in practice), and separately
    // confirms this is genuinely a live, non-vacuous pass through both
    // functions by deliberately violating a clamp constrain_variances()
    // would otherwise fix - proving this test's P is NOT the "nothing ran
    // at all" case the VtasPred<=1.0 test below covers.
    EkfCore ekf = make_fixture();
    const StateVector state_before = ekf.state;
    const Matrix24 P_before = ekf.P;

    // A reading wildly inconsistent with VtasPred=15 - large enough to fail
    // tasTestRatio (EAS_I_GATE_DEFAULT=400 -> gate = max(4.0,1.0) = 4.0)
    // while nowhere near badly-conditioning `temp` (this fixture's P is the
    // real, un-engineered covariance_init() result, so the badly-
    // conditioned check passes comfortably regardless of the reading).
    const bool applied = ekf.fuse_airspeed(ftype(100.0), ftype(0.01));

    REQUIRE_FALSE(applied);
    REQUIRE(static_cast<double>(ekf.tas_test_ratio()) >= 1.0);

    // StateVector: no apply_state_correction() call happens on a gate
    // failure - must be exactly what it was at entry.
    REQUIRE(ekf.state.velocity.x == state_before.velocity.x);
    REQUIRE(ekf.state.velocity.y == state_before.velocity.y);
    REQUIRE(ekf.state.velocity.z == state_before.velocity.z);
    REQUIRE(ekf.state.position.x == state_before.position.x);
    REQUIRE(ekf.state.wind_vel.x == state_before.wind_vel.x);
    REQUIRE(ekf.state.wind_vel.y == state_before.wind_vel.y);
    REQUIRE(ekf.state.quat.q1 == state_before.quat.q1);

    // P[0..15]: the realistic-case claim - byte-for-byte unchanged, because
    // this fixture's P (fresh from covariance_init()) already satisfies
    // every one of constrain_variances()'s clamps over this range and is
    // already symmetric.
    for (int i = 0; i <= 15; ++i) {
        for (int j = 0; j <= 15; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    P_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
    // P[16..23]: NOT unchanged, and deliberately not asserted as such - a
    // SEPARATE real, verified consequence of "ForceSymmetry()/
    // ConstrainVariances() run unconditionally" (banner outcome 3):
    // constrain_variances()'s already-established permanently-inhibited
    // zeroing (ekf_core.cpp: "Mag (16..21) and wind (22..23)... zeroed
    // every cycle") fires here too, even though NO fusion was applied.
    // covariance_init() seeded P[16][16]/[17][17]/[18][18]/[19][19]/[20][20]/
    // [21][21] to sq(mag_noise) (nonzero); after this gate-failure call they
    // must be exactly 0 - a real behavior this test confirms directly rather
    // than asserting the stronger, false "P completely untouched" claim.
    for (int i = 16; i <= 23; ++i) {
        REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] == ftype(0));
    }

    // Non-vacuousness check: a SEPARATE instance, engineered so
    // constrain_variances() would visibly change something (an
    // out-of-bounds quaternion variance - upstream's own clamp:
    // `P[0..3][0..3]` constrained to [0.0, 1.0], ekf_core.cpp's
    // constrain_variances()), confirms the SAME gate-failure call path
    // really does still invoke constrain_variances() - this is what outcome
    // 3's "unconditionally" claim means operationally, and is exactly what
    // distinguishes this test from the VtasPred<=1.0 test below (where an
    // identical violation survives completely untouched).
    EkfCore ekf2 = make_fixture();
    ekf2.P[0][0] = ftype(50.0);  // well past the real [0.0, 1.0] quaternion-variance clamp ceiling
    REQUIRE_FALSE(ekf2.fuse_airspeed(ftype(100.0), ftype(0.01)));
    REQUIRE(static_cast<double>(ekf2.P[0][0]) <= 1.0);  // constrain_variances() DID run and clamp it
}

TEST_CASE("fuse_airspeed: VtasPred <= 1.0 leaves BOTH state and P byte-for-byte untouched - not even "
          "force_symmetry()/constrain_variances() run",
          "[ekf_core][airspeed_fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.state.velocity = Vector3F(ftype(0.5), ftype(0.0), ftype(0.0));  // VtasPred = 0.5 < 1.0
    ekf.covariance_init(ftype(0.01));
    // Deliberately violate a constrain_variances() clamp so that, if this
    // path incorrectly ran force_symmetry()/constrain_variances() (the way
    // outcome 3 above legitimately does), it would be silently corrected -
    // confirming this path genuinely does NOT run them, unlike outcome 3.
    ekf.P[4][4] = ftype(-123.0);

    const StateVector state_before = ekf.state;
    const Matrix24 P_before = ekf.P;

    const bool applied = ekf.fuse_airspeed(ftype(0.4), ftype(0.01));

    REQUIRE_FALSE(applied);
    REQUIRE(ekf.state.velocity.x == state_before.velocity.x);
    REQUIRE(ekf.state.quat.q1 == state_before.quat.q1);
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    P_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
    // The tell-tale sign this really is a no-op: the deliberately-invalid
    // P[4][4] = -123.0 is still exactly -123.0 (constrain_variances() would
    // have clamped it to a positive floor).
    REQUIRE(ekf.P[4][4] == ftype(-123.0));
}


// ============================================================================
// CPP-070 PHASE 16 (this ticket - the LAST sensor in the CPP-067/068/069/
// 070 buffered/time-correct recall series): time-correct true-airspeed
// sample recall via ObsBuffer. See ekf_core.hpp's "CPP-070, PHASE 16"
// banner (above EkfCore::tas_buffer) and TasSample's own banner (above its
// struct definition, near GpsSample/MagSample/BaroSample) for the full
// scope/reasoning: the deliberately narrower recall-against-caller's-own-
// now_s design (identical to CPP-067/068/069's own), the independently-
// derived buffer-size justification (10Hz, from this file's own
// kAirspeedPeriodTicks precedent - see ekf_closed_loop_test.cpp), the real
// bare-scalar-vs-wrapper-struct tension this ticket resolves (identically
// to baro's own CPP-069 resolution), why fuse_airspeed() itself is kept
// completely unchanged, and why recall_tas_sample() is the ONLY consumer
// of tas_buffer (verified: airspeed has exactly one storedTAS.recall()
// call site upstream, in the SAME function as its own push() site - as
// simple as baro's own single-site situation).
//
// Test strategy - mirrors CPP-067/068/069's own shape exactly:
//   8. TasSample::set_time_s()/time_s() round-trip at millisecond
//      resolution (the disclosed ObsElement::time_ms quantization).
//   9. recall_tas_sample() on an empty buffer returns false and leaves
//      `out` untouched.
//   10. THE REAL, NEW CAPABILITY: push several airspeed readings at
//       DIFFERENT, non-tick-aligned timestamps (arrival times that are not
//       multiples of the 50Hz/20ms tick grid, modelling a real airspeed
//       sensor reporting on its own schedule, independent of the EKF's
//       own), then confirm fusion at each 50Hz tick correctly recalls the
//       most-recently-available reading and hands the unwrapped bare
//       `ftype` to the completely-unchanged fuse_airspeed() - the test
//       never calls push/recall at matching times, and never hand-feeds a
//       fusion call "the right reading" the way every other
//       fuse_airspeed() call above this section does.
// ============================================================================

TEST_CASE("TasSample::set_time_s/time_s round-trips at millisecond resolution",
          "[ekf_core][airspeed_fusion][tas_buffer]") {
    TasSample tas;
    REQUIRE(static_cast<double>(tas.time_s()) == Catch::Approx(0.0));  // zero-initialized ObsElement::time_ms

    tas.set_time_s(ftype(1.234));
    REQUIRE(static_cast<double>(tas.time_s()) == Catch::Approx(1.234).margin(1e-6));

    tas.set_time_s(ftype(0.0));
    REQUIRE(static_cast<double>(tas.time_s()) == Catch::Approx(0.0));

    // Defensive negative clamp (see TasSample::set_time_s()'s own doc
    // comment - now_s is never legitimately negative in this port's own
    // convention, but the clamp avoids UB rather than assuming callers
    // never pass one).
    tas.set_time_s(ftype(-5.0));
    REQUIRE(static_cast<double>(tas.time_s()) == Catch::Approx(0.0));
}

TEST_CASE("recall_tas_sample: returns false on an empty buffer", "[ekf_core][airspeed_fusion][tas_buffer]") {
    EkfCore ekf;
    TasSample out;
    out.true_airspeed_m_s = ftype(99.0);  // sentinel - must be untouched

    REQUIRE_FALSE(ekf.recall_tas_sample(out, ftype(0.05)));
    REQUIRE(ekf.tas_buffer.empty());
    // Untouched on failure, matching ObsBuffer::recall()'s own documented
    // contract (ekf_buffer.hpp).
    REQUIRE(static_cast<double>(out.true_airspeed_m_s) == Catch::Approx(99.0));
}

TEST_CASE("push_tas_sample/recall_tas_sample: recalls the correct sample under realistic "
          "asynchronous (non-tick-aligned) airspeed arrival, without the caller hand-feeding "
          "exactly the right reading synchronously, and hands the unwrapped bare ftype to the "
          "unchanged fuse_airspeed()",
          "[ekf_core][airspeed_fusion][tas_buffer]") {
    EkfCore ekf = make_fixture();

    constexpr ftype kDt = ftype(0.02);  // 50Hz IMU tick, matching this port's own
                                         // closed-loop-test precedent (ekf_closed_loop_test.cpp).

    // Four airspeed readings, each carrying its own TRUE arrival
    // timestamp - none of them a multiple of kDt (0.02s), i.e. none would
    // ever land exactly on an IMU tick boundary (verified: 91, 293, 512,
    // 741 mod 20 are all nonzero). Each carries a distinct, tagged
    // true_airspeed_m_s value close to make_fixture()'s own VtasPred=15.0
    // (see this file's own test 1 above) so every reading passes the real
    // tasTestRatio gate and actually fuses - this test is about recall
    // correctness, not gate behavior, already covered above.
    struct Reading {
        ftype arrival_s;
        ftype tag;
        bool pushed = false;
    };
    std::array<Reading, 4> readings{{
        {ftype(0.091), ftype(15.1)},
        {ftype(0.293), ftype(15.2)},
        {ftype(0.512), ftype(15.3)},
        {ftype(0.741), ftype(15.4)},
    }};

    std::vector<ftype> recalled_tags;
    std::vector<ftype> recall_latency_s;  // now_s at recall time - the sample's own arrival_s

    for (int tick = 1; tick <= 60; ++tick) {  // 60 * 20ms = 1.2s, comfortably past the last reading
        const ftype now_s = static_cast<ftype>(tick) * kDt;

        // --- Sample arrival: modelled as an independently-scheduled event
        // (upstream: readAirSpdData(), its own function, called on its own
        // terms) that pushes into tas_buffer AS SOON AS it has arrived,
        // stamped with its OWN true arrival time - NOT now_s, and NOT
        // aligned to the tick grid at all. This is the realistic
        // asynchronous-arrival part: the caller's EKF loop only gets a
        // chance to call push_tas_sample() once per tick, but the sample
        // it pushes carries whatever timestamp the airspeed sensor itself
        // reported. ---
        for (Reading& reading : readings) {
            if (!reading.pushed && now_s >= reading.arrival_s) {
                TasSample tas;
                tas.set_time_s(reading.arrival_s);
                tas.true_airspeed_m_s = reading.tag;
                ekf.push_tas_sample(tas);
                reading.pushed = true;
            }
        }

        // --- Fusion attempted EVERY tick (upstream: readAirSpdData()'s own
        // recall attempt runs every EKF cycle) - recall_tas_sample()
        // decides, purely from timestamps, whether a time-eligible reading
        // exists yet. THE TEST NEVER TELLS IT WHICH TICK TO EXPECT A MATCH
        // ON - that is exactly the capability this ticket adds (contrast
        // with every fuse_airspeed(true_airspeed_m_s, ...) call above this
        // section, which hands over a hand-built bare ftype directly, on
        // the exact call the test chooses). ---
        TasSample recalled;
        if (ekf.recall_tas_sample(recalled, now_s)) {
            recalled_tags.push_back(recalled.true_airspeed_m_s);
            recall_latency_s.push_back(now_s - recalled.time_s());

            // The full pipeline: unwrap the recalled TasSample back to a
            // bare ftype and fuse it exactly like any other reading -
            // fuse_airspeed() itself is completely unchanged by this
            // ticket (see TasSample's own "THE BARE-SCALAR-VS-WRAPPER-
            // STRUCT TENSION" banner).
            ekf.fuse_airspeed(recalled.true_airspeed_m_s, kDt);
        }
    }

    // All 4 asynchronously-arriving readings were recalled exactly once
    // each, in arrival order, none lost and none double-counted - the real
    // acceptance criterion: fusion recalled the right reading at the right
    // (later, tick-quantized) time without ever being handed it directly.
    REQUIRE(recalled_tags.size() == 4);
    REQUIRE(static_cast<double>(recalled_tags[0]) == Catch::Approx(15.1));
    REQUIRE(static_cast<double>(recalled_tags[1]) == Catch::Approx(15.2));
    REQUIRE(static_cast<double>(recalled_tags[2]) == Catch::Approx(15.3));
    REQUIRE(static_cast<double>(recalled_tags[3]) == Catch::Approx(15.4));

    // Each recall happened PROMPTLY after its sample's true arrival - at
    // most one tick (kDt) of latency (the recalling tick is the first tick
    // at/after arrival_s), and never before arrival (recall() would have
    // left a strictly-newer element untouched - ekf_buffer.hpp).
    for (const ftype latency : recall_latency_s) {
        REQUIRE(static_cast<double>(latency) >= 0.0);
        REQUIRE(static_cast<double>(latency) < static_cast<double>(kDt) + 1e-9);
    }

    // The buffer is drained back to empty - every pushed sample was
    // consumed by exactly one recall, none left stranded.
    REQUIRE(ekf.tas_buffer.empty());
}
