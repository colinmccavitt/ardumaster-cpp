// Tests for fwcpp::ekf::EkfCore's CPP-056 phase 2 additions: the
// fuse_direct_state_observation() primitive and GPS velocity/position
// fusion built on it. See fwcpp/ekf/ekf_core.hpp's "CPP-056, PHASE 2"
// banner section for the full scope/exclusions discussion.
//
// Test strategy (per the ticket's own acceptance criteria):
//   1. A perfect, zero-noise observation drives the observed state's
//      variance to exactly zero and the state exactly to the
//      observation - textbook Kalman-filter behavior.
//   2. A very noisy observation barely moves the state or its variance -
//      the opposite textbook extreme.
//   3. The healthyFusion negative-variance guard genuinely skips an
//      update engineered to corrupt P, leaving P and state untouched.
//   4. The real GPS "no reported accuracy" R_OBS formula is verified
//      against hand-computed expected values, not just "runs without
//      error".
//   5. A closed-loop test: an EkfCore fed a constant, unmodeled
//      accelerometer bias (a classic INS drift source) with periodic
//      GPS velocity+position fusion is compared against an identical
//      instance receiving no fusion at all (pure dead reckoning) over
//      the same 20-second run. This is the first test in the whole
//      EkfCore port that demonstrates fusion actually outperforming
//      pure prediction, even in this narrow, delay-free, ungated form.
//
// CPP-057 phase 3 adds (see ekf_core.hpp's "CPP-057, PHASE 3" banner for
// the full scope/exclusions discussion):
//   6. gps_vel_test_ratio()/gps_pos_test_ratio() are verified against
//      hand-computed expected values, the same way the R_OBS formula is
//      verified above - not just "runs without error".
//   7. A wild GPS glitch, engineered to genuinely exceed the real gate
//      at the real default VEL_I_GATE_DEFAULT/POS_I_GATE_DEFAULT = 500,
//      is rejected: fuse_gps_velocity()/fuse_gps_position() return 0 and
//      P/state are left byte-for-byte untouched (the ungated-form test
//      above, #5, would have corrupted the estimate with this same
//      sample - see #9 below for a direct side-by-side demonstration).
//   8. A borderline-inside-gate sample (test ratio genuinely close to
//      but under 1.0, not trivially inside) still fuses exactly like the
//      pre-CPP-057 fusion did - regression coverage against #3/#5 above.
//   9. #5's own closed-loop drift-correction scenario, extended with one
//      fabricated wild GPS glitch injected mid-run, run twice: once
//      through this ticket's real gate (gated), and once replicating
//      CPP-056's literal pre-gate behavior by calling
//      fuse_direct_state_observation() directly, bypassing the gate
//      (ungated) - on the exact same glitch. This is the real,
//      demonstrable point of the whole ticket: the gated instance's
//      estimate does not jump toward the glitch while the ungated one's
//      does, side by side, not merely asserted in isolation.
//
// CPP-058 phase 4 adds (see ekf_core.hpp's "CPP-058, PHASE 4" banner for
// the full scope/exclusions/corrections discussion):
//   10. Regression coverage: a gate-failing sample within the 10.0s
//       posRetryTimeUseVel_ms window still behaves exactly like #7 above
//       - P/state byte-for-byte untouched, no reset - confirming the new
//       `now_s`/timeout wiring does not change phase-3 behavior when the
//       timeout hasn't elapsed.
//   11. The real acceptance-criterion scenario: a closed-loop run under
//       CLEAN dynamics (no unmodeled bias - state stays accurate, P
//       grows only through ordinary process noise) is periodically
//       checked against a GPS receiver stuck reporting one fixed, wildly
//       wrong fix (the same magnitude as the wild-glitch tests above)
//       across a >10s span. Every check before the 10.0s mark must leave
//       P/state untouched (matching #10 and phase 3). The first check
//       at/after the 10.0s mark must instead produce a HARD reset: state
//       jumps directly to the (still-wrong-looking, now-trusted) GPS
//       sample (not a gradual blend) and P is re-seeded to the real
//       reduced-scope formula (sq(gps_horiz_vel_noise)/
//       sq(gps_horiz_pos_noise)) - verified as exact values, not just
//       "moved in the right direction". One further check afterward,
//       once GPS reports a value matching where the vehicle actually is,
//       confirms normal CPP-057 gated fusion resumes (n_fused > 0
//       again).
//
// CPP-062 phase 8 adds (see ekf_core.hpp's "CPP-062, PHASE 8" banner for the
// full scope/exclusions/corrections discussion) - baro height fusion at
// state_index=9, mirroring the GPS position tests above at a single axis:
//   12. baro_hgt_obs_variance()/hgt_test_ratio() are verified against
//       hand-computed expected values, the same treatment as the GPS/mag
//       formula tests above.
//   13. A baro reading EXACTLY consistent with the current position.z state
//       (zero innovation) leaves state.position.z exactly unchanged -
//       mirrors ekf_mag_fusion_test.cpp's own "a reading consistent with the
//       current state leaves state exactly unchanged" test shape.
//   14. A baro reading inconsistent-but-not-extreme (comfortably inside the
//       real HGT_I_GATE_DEFAULT=500 gate) measurably corrects
//       state.position.z toward the reading over repeated fusion calls -
//       the ticket's own explicit "(b)" acceptance criterion.
//   15. A wild baro glitch, engineered to genuinely exceed the real gate,
//       is rejected: fuse_baro_height() returns false and P/state are left
//       byte-for-byte untouched, with NO reset triggered (elapsed time
//       since the last pass is well under the 10.0s hgtRetryTimeMode0_ms
//       threshold) - the gate-failure-vs-reset distinction the ticket
//       explicitly asks not to conflate.
//   16. A borderline-inside-gate sample still fuses normally - regression
//       coverage against #14, same shape as the GPS borderline tests.
//   17. Regression coverage: a gate-failing sample within the 10.0s
//       hgtRetryTimeMode0_ms window still behaves exactly like #15 above -
//       P/state byte-for-byte untouched, no reset.
//   18. The real acceptance-criterion scenario: a sustained baro "outage"
//       (a stuck sensor reporting one fixed, wildly wrong altitude) across
//       a >10s span under clean dynamics. Every check before the 10.0s mark
//       leaves P/state untouched. The first check at/after the 10.0s mark
//       produces a HARD reset: state.position.z jumps directly to
//       -baro_altitude_m (not a gradual blend) and P[9][9] is re-seeded to
//       baro_hgt_obs_variance() exactly. One further check afterward, once
//       the baro reading matches the vehicle's actual altitude, confirms
//       normal gated fusion resumes.

#include <array>
#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/ekf/ekf_core.hpp>

using namespace fwcpp::ekf;

namespace {
constexpr ftype kGravity = static_cast<ftype>(9.80665);
}

TEST_CASE("fuse_direct_state_observation: zero-noise observation drives variance to zero and state to the observation",
          "[ekf_core][fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[4][4] = ftype(4.0);  // arbitrary nonzero prior variance, velocity.x (state index 4)
    ekf.state.velocity.x = ftype(10.0);

    const ftype observed_vel = ftype(7.0);
    // upstream's sign convention: innovation = state - observation (see
    // ekf_core.hpp's fuse_direct_state_observation() declaration comment).
    const ftype innovation = ekf.state.velocity.x - observed_vel;

    const bool applied = ekf.fuse_direct_state_observation(4, innovation, ftype(0.0), ftype(0.01));

    REQUIRE(applied);
    // obs_variance == 0 => SK = 1/P[4][4] => Kfusion[4] = 1 exactly =>
    // full correction to the observation.
    REQUIRE(static_cast<double>(ekf.state.velocity.x) == Catch::Approx(7.0).margin(1e-6));
    // Textbook zero-noise fusion drives P[4][4] = (1-K)*prior = 0
    // BEFORE constrain_variances() runs - but fuse_direct_state_observation()
    // calls constrain_variances() as its very last covariance step (see
    // ekf_core.hpp), which enforces upstream's own real VEL_STATE_MIN_VARIANCE
    // floor (AP_NavEKF3_core.h, 1e-4) on NE velocity variance
    // unconditionally - so the real, faithfully-reproduced result is
    // "collapses to upstream's own floor", not literally zero. This IS
    // upstream's real behavior (verified: ConstrainVariances() clamps
    // P[4][4]/P[5][5] to >= VEL_STATE_MIN_VARIANCE with no exception for
    // a just-fused value), not a bug in this port.
    REQUIRE(static_cast<double>(ekf.P[4][4]) == Catch::Approx(1e-4).margin(1e-9));
}

TEST_CASE("fuse_direct_state_observation: a very noisy observation barely moves the state", "[ekf_core][fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[4][4] = ftype(0.01);  // small, confident prior variance
    ekf.state.velocity.x = ftype(10.0);
    const ftype prior_var = ekf.P[4][4];

    const ftype observed_vel = ftype(100.0);  // wildly different from the confident prior
    const ftype innovation = ekf.state.velocity.x - observed_vel;
    const ftype huge_obs_variance = ftype(1e9);

    const bool applied = ekf.fuse_direct_state_observation(4, innovation, huge_obs_variance, ftype(0.01));

    REQUIRE(applied);
    // Kalman gain ~= prior_var / huge_obs_variance ~= 1e-11 - state
    // should not have moved meaningfully off its prior value.
    REQUIRE(std::abs(static_cast<double>(ekf.state.velocity.x) - 10.0) < 1e-6);
    // Essentially no information gained: posterior variance ~= prior.
    REQUIRE(static_cast<double>(ekf.P[4][4]) == Catch::Approx(static_cast<double>(prior_var)).margin(1e-9));
}

TEST_CASE("fuse_direct_state_observation: the healthyFusion negative-variance guard skips a corrupting update",
          "[ekf_core][fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    // Fabricate a non-physical covariance with a cross-correlation
    // P[0][4] far larger than sqrt(P[0][0]*P[4][4]) could ever validly
    // be - engineered specifically to drive KHP[0][0] above P[0][0],
    // exercising upstream's own negative-variance guard (the ONLY
    // fusion-health check in this phase's scope - see ekf_core.hpp
    // banner: no innovation-consistency gating exists yet).
    ekf.P[0][0] = ftype(1.0);
    ekf.P[4][4] = ftype(1.0);
    ekf.P[0][4] = ekf.P[4][0] = ftype(10.0);
    ekf.state.velocity.x = ftype(10.0);

    const Matrix24 p_before = ekf.P;
    const StateVector state_before = ekf.state;
    const ftype innovation = ekf.state.velocity.x - ftype(7.0);

    const bool applied = ekf.fuse_direct_state_observation(4, innovation, ftype(0.0), ftype(0.01));

    REQUIRE_FALSE(applied);
    // P must be byte-for-byte untouched - upstream's own "skip the
    // update entirely" behavior when healthyFusion is false.
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    p_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
    REQUIRE(ekf.state.velocity.x == state_before.velocity.x);
    REQUIRE(ekf.state.quat.q1 == state_before.quat.q1);
}

TEST_CASE("gps_*_obs_variance: matches upstream's real 'no reported accuracy' R_OBS formula", "[ekf_core][fusion]") {
    EkfCore ekf;  // gps_horiz_vel_noise=0.5, gps_vert_vel_noise=0.7, gps_horiz_pos_noise=0.5 (phase-1 defaults)
    ekf.vel_dot_ned_filt = Vector3F(ftype(1.0), ftype(0), ftype(0));  // accNavMag = velDotNEDfilt.length() = 1.0

    // upstream: sq(constrain_ftype(0.5, 0.05, 5.0)) + sq(0.05*1.0) = 0.25 + 0.0025
    REQUIRE(static_cast<double>(ekf.gps_horiz_vel_obs_variance()) == Catch::Approx(0.2525));
    // upstream: sq(constrain_ftype(0.7, 0.05, 5.0)) + sq(0.07*1.0) = 0.49 + 0.0049
    REQUIRE(static_cast<double>(ekf.gps_vert_vel_obs_variance()) == Catch::Approx(0.4949));
    // upstream: sq(constrain_ftype(0.5, 0.1, 10.0)) + sq(0.05*1.0) = 0.25 + 0.0025
    REQUIRE(static_cast<double>(ekf.gps_horiz_pos_obs_variance()) == Catch::Approx(0.2525));
}

TEST_CASE("fuse_gps_velocity/fuse_gps_position: fuse all axes under normal conditions", "[ekf_core][fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));

    GpsSample gps;
    gps.velocity_ned = Vector3F(ftype(5.0), ftype(1.0), ftype(0.1));
    // CPP-057: position offset shrunk from the pre-gate test's
    // (100.0, -50.0) - covariance_init() seeds P[7][7]=P[8][8]=
    // sq(gps_horiz_pos_noise)=0.25 (upstream's own real init value, ~line
    // 236-237), so a 100m offset now genuinely (and correctly) fails the
    // real posTestRatio gate this ticket adds - it was never a realistic
    // "normal conditions" GPS sample, just a convenient pre-gate value.
    // (3.0, -2.0) is a small, physically-plausible offset that passes
    // the real gate comfortably (posTestRatio ~= 0.52) while still moving
    // the state measurably.
    gps.position_ne = Vector2F(ftype(3.0), ftype(-2.0));

    REQUIRE(ekf.fuse_gps_velocity(gps, ftype(0.01)) == 3);
    REQUIRE(ekf.fuse_gps_position(gps, ftype(0.01)) == 2);

    // State should have moved measurably toward the observation (not a
    // no-op) - P[4][4]/P[7][7] etc from covariance_init() are well above
    // zero, so Kalman gain is meaningfully nonzero.
    REQUIRE(static_cast<double>(ekf.state.velocity.x) > 0.0);
    REQUIRE(static_cast<double>(ekf.state.position.x) > 0.0);
}

TEST_CASE("gps_vel_test_ratio/gps_pos_test_ratio: match upstream's real velTestRatio/posTestRatio formulas",
          "[ekf_core][fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    // Manually controlled diagonal covariance (not covariance_init())
    // for a hand-computable expected ratio, same convention as the
    // fuse_direct_state_observation() tests above.
    ekf.P[4][4] = ftype(1.0);
    ekf.P[5][5] = ftype(1.0);
    ekf.P[6][6] = ftype(1.0);
    ekf.P[7][7] = ftype(1.0);
    ekf.P[8][8] = ftype(1.0);
    // vel_dot_ned_filt defaults to zero => accNavMag=0 => r_obs_horiz =
    // gps_horiz_vel_obs_variance() = 0.25, r_obs_vert =
    // gps_vert_vel_obs_variance() = 0.49, r_obs_pos =
    // gps_horiz_pos_obs_variance() = 0.25 (all verified by the "gps_*
    // _obs_variance" test above at accNavMag=1.0; at accNavMag=0 the
    // accel-scale term drops out entirely, leaving just the clamped
    // noise term squared: sq(0.5)=0.25, sq(0.7)=0.49, sq(0.5)=0.25).

    GpsSample gps;
    // innov = state - gps (upstream's own sign convention). state
    // velocity/position are both zero here, so innov = -gps.
    gps.velocity_ned = Vector3F(ftype(-4.0), ftype(-3.0), ftype(0.0));  // innov = (4, 3, 0)
    gps.position_ne = Vector2F(ftype(-6.0), ftype(0.0));                // innov = (6, 0)

    // upstream: velTestRatio = innovVelSumSq / (varVelSum *
    // sq(MAX(0.01*_gpsVelInnovGate, 1.0))). innovSumSq = 4^2+3^2+0^2=25.
    // varVelSum = (1+0.25)+(1+0.25)+(1+0.49) = 3.99. gate =
    // MAX(0.01*500,1.0) = 5.0, sq(gate) = 25. ratio = 25/(3.99*25).
    const double expected_vel_ratio = 25.0 / (3.99 * 25.0);
    REQUIRE(static_cast<double>(ekf.gps_vel_test_ratio(gps)) == Catch::Approx(expected_vel_ratio).margin(1e-6));

    // upstream: posTestRatio = (innovN^2+innovE^2) /
    // (sq(MAX(0.01*_gpsPosInnovGate,1.0)) * (varInnovN+varInnovE)).
    // innovSumSq = 6^2+0^2 = 36. varSum = (1+0.25)+(1+0.25) = 2.5.
    // maxPosInnov2 = 25*2.5 = 62.5. ratio = 36/62.5.
    const double expected_pos_ratio = 36.0 / 62.5;
    REQUIRE(static_cast<double>(ekf.gps_pos_test_ratio(gps)) == Catch::Approx(expected_pos_ratio).margin(1e-6));
}

TEST_CASE("fuse_gps_velocity: a wild GPS glitch fails the real velTestRatio gate and leaves state/P untouched",
          "[ekf_core][fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[4][4] = ekf.P[5][5] = ekf.P[6][6] = ftype(1.0);
    // varVelSum = (1+0.25)*2 + (1+0.49) = 3.99; pass threshold is
    // innovSumSq < 3.99*25 = 99.75 (see the formula test above).

    GpsSample gps;
    gps.velocity_ned = Vector3F(ftype(50.0), ftype(0.0), ftype(0.0));  // innovSumSq = 2500, way over 99.75
    gps.position_ne = Vector2F(ftype(0.0), ftype(0.0));

    REQUIRE(static_cast<double>(ekf.gps_vel_test_ratio(gps)) > 1.0);

    const Matrix24 p_before = ekf.P;
    const StateVector state_before = ekf.state;

    const int n_fused = ekf.fuse_gps_velocity(gps, ftype(0.01));

    REQUIRE(n_fused == 0);
    // P and state must be byte-for-byte untouched, matching upstream's
    // own `else { fuseVelData = false; }` and this port's existing
    // untouched-on-skip convention from the healthyFusion guard test
    // above.
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    p_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
    REQUIRE(ekf.state.velocity.x == state_before.velocity.x);
    REQUIRE(ekf.state.velocity.y == state_before.velocity.y);
    REQUIRE(ekf.state.velocity.z == state_before.velocity.z);
    REQUIRE(ekf.state.quat.q1 == state_before.quat.q1);
}

TEST_CASE("fuse_gps_velocity: a borderline-inside-gate sample still fuses normally", "[ekf_core][fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[4][4] = ekf.P[5][5] = ekf.P[6][6] = ftype(1.0);

    GpsSample gps;
    gps.velocity_ned = Vector3F(ftype(9.9), ftype(0.0), ftype(0.0));  // innovSumSq = 98.01, just under 99.75
    gps.position_ne = Vector2F(ftype(0.0), ftype(0.0));

    const ftype ratio = ekf.gps_vel_test_ratio(gps);
    REQUIRE(static_cast<double>(ratio) < 1.0);
    REQUIRE(static_cast<double>(ratio) > 0.9);  // genuinely borderline, not trivially inside the gate

    const int n_fused = ekf.fuse_gps_velocity(gps, ftype(0.01));

    REQUIRE(n_fused == 3);
    // Regression coverage against CPP-056: a passing sample must still
    // move the state measurably toward the observation, exactly as it
    // did before this ticket's gate existed.
    REQUIRE(static_cast<double>(ekf.state.velocity.x) > 0.0);
}

TEST_CASE("fuse_gps_position: a wild GPS glitch fails the real posTestRatio gate and leaves state/P untouched",
          "[ekf_core][fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[7][7] = ekf.P[8][8] = ftype(1.0);
    // varSum = (1+0.25)*2 = 2.5; pass threshold is innovSumSq < 2.5*25 = 62.5.

    GpsSample gps;
    gps.velocity_ned = Vector3F(ftype(0.0), ftype(0.0), ftype(0.0));
    gps.position_ne = Vector2F(ftype(50.0), ftype(0.0));  // innovSumSq = 2500, way over 62.5

    REQUIRE(static_cast<double>(ekf.gps_pos_test_ratio(gps)) > 1.0);

    const Matrix24 p_before = ekf.P;
    const StateVector state_before = ekf.state;

    const int n_fused = ekf.fuse_gps_position(gps, ftype(0.01));

    REQUIRE(n_fused == 0);
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    p_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
    REQUIRE(ekf.state.position.x == state_before.position.x);
    REQUIRE(ekf.state.position.y == state_before.position.y);
}

TEST_CASE("fuse_gps_position: a borderline-inside-gate sample still fuses normally", "[ekf_core][fusion]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[7][7] = ekf.P[8][8] = ftype(1.0);

    GpsSample gps;
    gps.velocity_ned = Vector3F(ftype(0.0), ftype(0.0), ftype(0.0));
    gps.position_ne = Vector2F(ftype(7.8), ftype(0.0));  // innovSumSq = 60.84, just under 62.5

    const ftype ratio = ekf.gps_pos_test_ratio(gps);
    REQUIRE(static_cast<double>(ratio) < 1.0);
    REQUIRE(static_cast<double>(ratio) > 0.9);  // genuinely borderline, not trivially inside the gate

    const int n_fused = ekf.fuse_gps_position(gps, ftype(0.01));

    REQUIRE(n_fused == 2);
    REQUIRE(static_cast<double>(ekf.state.position.x) > 0.0);
}

TEST_CASE("EkfCore: GPS fusion measurably corrects INS drift versus pure prediction", "[ekf_core][fusion]") {
    const ftype dt = ftype(0.01);                    // 100 Hz IMU
    const ftype accel_bias_x_true = ftype(0.05);      // unmodeled accelerometer bias, m/s^2
    const ftype true_vx = ftype(5.0);                 // true constant NED velocity, m/s
    const int total_steps = 2000;                     // 20 s total run
    const int gps_period_steps = 100;                 // GPS sample every 1.0 s

    auto make_ekf = [&]() {
        EkfCore ekf;
        ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
        ekf.state.velocity = Vector3F(true_vx, ftype(0), ftype(0));
        ekf.covariance_init(dt);
        return ekf;
    };

    EkfCore fused = make_ekf();
    EkfCore unfused = make_ekf();

    GyroSample gyro;
    gyro.delta_angle_dt = dt;  // zero rotation - stays level throughout
    AccelSample accel;
    // Real specific force for level, zero-horizontal-acceleration flight
    // is pure gravity-cancelling; this accelerometer additionally reads a
    // constant, unmodeled bias in x (accel_bias state starts at zero and
    // is never given a chance to learn it in the unfused case) - a
    // textbook INS drift source: unmodeled accel bias -> velocity error
    // growing linearly with time -> position error growing quadratically.
    accel.delta_velocity = Vector3F(accel_bias_x_true * dt, ftype(0), -kGravity * dt);
    accel.delta_velocity_dt = dt;

    ftype true_x = ftype(0.0);
    for (int step = 1; step <= total_steps; ++step) {
        fused.update_strapdown_equations_ned(gyro, accel, dt);
        fused.covariance_prediction(gyro, accel, dt);
        unfused.update_strapdown_equations_ned(gyro, accel, dt);
        unfused.covariance_prediction(gyro, accel, dt);

        true_x += true_vx * dt;

        if (step % gps_period_steps == 0) {
            GpsSample gps;
            gps.velocity_ned = Vector3F(true_vx, ftype(0), ftype(0));
            gps.position_ne = Vector2F(true_x, ftype(0));
            fused.fuse_gps_velocity(gps, dt);
            fused.fuse_gps_position(gps, dt);
        }
    }

    const double fused_vel_err = std::abs(static_cast<double>(fused.state.velocity.x) - static_cast<double>(true_vx));
    const double unfused_vel_err = std::abs(static_cast<double>(unfused.state.velocity.x) - static_cast<double>(true_vx));
    const double fused_pos_err = std::abs(static_cast<double>(fused.state.position.x) - static_cast<double>(true_x));
    const double unfused_pos_err = std::abs(static_cast<double>(unfused.state.position.x) - static_cast<double>(true_x));

    // Sanity check the test itself is not vacuous: pure dead reckoning
    // must show real, substantial drift. Expected analytically:
    // velocity error ~= accel_bias_x_true * T = 0.05*20 = 1.0 m/s;
    // position error ~= 0.5*accel_bias_x_true*T^2 = 0.5*0.05*400 = 10 m.
    REQUIRE(unfused_vel_err > 0.5);
    REQUIRE(unfused_pos_err > 3.0);

    // The actual point of this ticket: fusion must measurably outperform
    // pure prediction over the identical run.
    REQUIRE(fused_vel_err < unfused_vel_err / 3.0);
    REQUIRE(fused_pos_err < unfused_pos_err / 3.0);

    // Bonus: the underlying Kalman gain (via the real transcribed
    // velocity/position <-> accel-bias covariance correlation from
    // covariance_prediction()) should have started learning the true
    // accel bias too, not just repeatedly resetting position/velocity -
    // a real EKF behavior, not achievable by naive GPS-only averaging.
    const double accel_bias_err =
        std::abs(static_cast<double>(fused.state.accel_bias.x) - static_cast<double>(accel_bias_x_true));
    REQUIRE(accel_bias_err < static_cast<double>(accel_bias_x_true));
}

TEST_CASE("EkfCore: an injected GPS glitch mid-run is rejected without corrupting the fused estimate "
          "(gated vs CPP-056's ungated fusion, side by side)",
          "[ekf_core][fusion]") {
    // Same closed-loop scenario as the test directly above, extended per
    // the ticket's own acceptance criterion: inject one fabricated wild
    // GPS glitch mid-run and confirm the filter does NOT jump toward it.
    // Run the identical scenario through two instances that see the
    // EXACT same IMU stream and the EXACT same (including glitch) GPS
    // samples - one through this ticket's real gate ("gated"), one
    // bypassing it entirely by calling fuse_direct_state_observation()
    // directly per axis, replicating CPP-056's literal pre-gate fusion
    // ("ungated") - so the value of this ticket is demonstrated by
    // contrast, not asserted in isolation.
    const ftype dt = ftype(0.01);
    const ftype accel_bias_x_true = ftype(0.05);
    const ftype true_vx = ftype(5.0);
    const int total_steps = 2000;
    const int gps_period_steps = 100;
    const int glitch_step = 1000;              // one bad fix, 10s into the run
    const ftype glitch_vel_x = ftype(500.0);   // wildly wrong reported velocity, m/s
    const ftype glitch_pos_x = ftype(5000.0);  // wildly wrong reported position, m

    auto make_ekf = [&]() {
        EkfCore ekf;
        ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
        ekf.state.velocity = Vector3F(true_vx, ftype(0), ftype(0));
        ekf.covariance_init(dt);
        return ekf;
    };

    EkfCore gated = make_ekf();
    EkfCore ungated = make_ekf();

    GyroSample gyro;
    gyro.delta_angle_dt = dt;
    AccelSample accel;
    accel.delta_velocity = Vector3F(accel_bias_x_true * dt, ftype(0), -kGravity * dt);
    accel.delta_velocity_dt = dt;

    ftype true_x = ftype(0.0);
    double gated_vel_jump = 0.0, ungated_vel_jump = 0.0;
    double gated_pos_jump = 0.0, ungated_pos_jump = 0.0;
    int gated_n_vel_fused_at_glitch = -1, gated_n_pos_fused_at_glitch = -1;

    for (int step = 1; step <= total_steps; ++step) {
        gated.update_strapdown_equations_ned(gyro, accel, dt);
        gated.covariance_prediction(gyro, accel, dt);
        ungated.update_strapdown_equations_ned(gyro, accel, dt);
        ungated.covariance_prediction(gyro, accel, dt);

        true_x += true_vx * dt;

        if (step % gps_period_steps == 0) {
            GpsSample gps;
            if (step == glitch_step) {
                gps.velocity_ned = Vector3F(glitch_vel_x, ftype(0), ftype(0));
                gps.position_ne = Vector2F(glitch_pos_x, ftype(0));
            } else {
                gps.velocity_ned = Vector3F(true_vx, ftype(0), ftype(0));
                gps.position_ne = Vector2F(true_x, ftype(0));
            }

            // gated: this ticket's real fuse_gps_velocity()/fuse_gps_position().
            const int n_vel_fused = gated.fuse_gps_velocity(gps, dt);
            const int n_pos_fused = gated.fuse_gps_position(gps, dt);

            // ungated: reproduce CPP-056's literal pre-CPP-057 behavior -
            // call fuse_direct_state_observation() directly per axis,
            // exactly as fuse_gps_velocity()/fuse_gps_position() did
            // before this ticket added the gate check at their top.
            const ftype r_obs_horiz = ungated.gps_horiz_vel_obs_variance();
            const ftype r_obs_vert = ungated.gps_vert_vel_obs_variance();
            ungated.fuse_direct_state_observation(4, ungated.state.velocity.x - gps.velocity_ned.x, r_obs_horiz, dt);
            ungated.fuse_direct_state_observation(5, ungated.state.velocity.y - gps.velocity_ned.y, r_obs_horiz, dt);
            ungated.fuse_direct_state_observation(6, ungated.state.velocity.z - gps.velocity_ned.z, r_obs_vert, dt);
            const ftype r_obs_pos = ungated.gps_horiz_pos_obs_variance();
            ungated.fuse_direct_state_observation(7, ungated.state.position.x - gps.position_ne.x, r_obs_pos, dt);
            ungated.fuse_direct_state_observation(8, ungated.state.position.y - gps.position_ne.y, r_obs_pos, dt);

            if (step == glitch_step) {
                gated_n_vel_fused_at_glitch = n_vel_fused;
                gated_n_pos_fused_at_glitch = n_pos_fused;
                gated_vel_jump = std::abs(static_cast<double>(gated.state.velocity.x) - static_cast<double>(true_vx));
                ungated_vel_jump =
                    std::abs(static_cast<double>(ungated.state.velocity.x) - static_cast<double>(true_vx));
                gated_pos_jump = std::abs(static_cast<double>(gated.state.position.x) - static_cast<double>(true_x));
                ungated_pos_jump =
                    std::abs(static_cast<double>(ungated.state.position.x) - static_cast<double>(true_x));
            }
        }
    }

    // At the glitch cycle itself: this ticket's real gate must have
    // rejected the sample outright (0 axes fused for both vel and pos).
    REQUIRE(gated_n_vel_fused_at_glitch == 0);
    REQUIRE(gated_n_pos_fused_at_glitch == 0);

    // The real, tangible value of this ticket: right after the glitch
    // cycle, the gated estimate is still close to truth (normal INS
    // drift over one GPS period is small), while the ungated estimate -
    // CPP-056's own literal pre-gate behavior - has jumped drastically
    // toward the glitch, exactly the corruption CPP-056's own commit
    // named as a disclosed gap.
    REQUIRE(gated_vel_jump < 1.0);
    REQUIRE(ungated_vel_jump > 50.0);
    REQUIRE(gated_pos_jump < 1.0);
    REQUIRE(ungated_pos_jump > 500.0);

    // Run to completion: the gated filter should finish the 20s run
    // tracking truth about as well as the un-glitched closed-loop test
    // above (the one rejected sample cost it nothing), while the ungated
    // filter's own final error should still show it was permanently set
    // back by the one glitch it was defenseless against.
    const double gated_final_vel_err =
        std::abs(static_cast<double>(gated.state.velocity.x) - static_cast<double>(true_vx));
    const double gated_final_pos_err =
        std::abs(static_cast<double>(gated.state.position.x) - static_cast<double>(true_x));
    const double ungated_final_vel_err =
        std::abs(static_cast<double>(ungated.state.velocity.x) - static_cast<double>(true_vx));
    const double ungated_final_pos_err =
        std::abs(static_cast<double>(ungated.state.position.x) - static_cast<double>(true_x));

    REQUIRE(gated_final_vel_err < 0.5);
    REQUIRE(gated_final_pos_err < 3.0);
    REQUIRE(gated_final_vel_err < ungated_final_vel_err);
    REQUIRE(gated_final_pos_err < ungated_final_pos_err);
}

TEST_CASE("fuse_gps_velocity/fuse_gps_position: a gap under the 10.0s posRetryTimeUseVel_ms timeout "
          "does not trigger a reset (regression)",
          "[ekf_core][fusion][cpp058]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));

    GpsSample good_gps;
    good_gps.velocity_ned = Vector3F(ftype(5.0), ftype(0.0), ftype(0.0));
    good_gps.position_ne = Vector2F(ftype(0.0), ftype(0.0));
    ekf.state.velocity = good_gps.velocity_ned;

    // Establish a successful pass at now_s = 0 - this anchors
    // last_vel_pass_time_s/last_pos_pass_time_s = 0.
    REQUIRE(ekf.fuse_gps_velocity(good_gps, ftype(0.01), ftype(0.0)) == 3);
    REQUIRE(ekf.fuse_gps_position(good_gps, ftype(0.01), ftype(0.0)) == 2);

    // The same wild-glitch magnitude as the CPP-057 gate tests above,
    // genuinely failing gps_vel_test_ratio()/gps_pos_test_ratio() - but
    // at now_s = 9.0, i.e. elapsed = 9.0 - 0 = 9.0s, still inside the
    // 10.0s posRetryTimeUseVel_ms window.
    GpsSample glitch;
    glitch.velocity_ned = Vector3F(ftype(50.0), ftype(0.0), ftype(0.0));
    glitch.position_ne = Vector2F(ftype(50.0), ftype(0.0));
    REQUIRE(static_cast<double>(ekf.gps_vel_test_ratio(glitch)) > 1.0);
    REQUIRE(static_cast<double>(ekf.gps_pos_test_ratio(glitch)) > 1.0);

    const Matrix24 p_before = ekf.P;
    const StateVector state_before = ekf.state;

    REQUIRE(ekf.fuse_gps_velocity(glitch, ftype(0.01), ftype(9.0)) == 0);
    REQUIRE(ekf.fuse_gps_position(glitch, ftype(0.01), ftype(9.0)) == 0);

    // No reset: P/state byte-for-byte untouched, exactly matching
    // CPP-057's existing gate-rejection behavior - the timeout window
    // has not yet elapsed, so the gate failure alone (not a timeout) is
    // still the only thing that happened.
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    p_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
    REQUIRE(ekf.state.velocity.x == state_before.velocity.x);
    REQUIRE(ekf.state.velocity.y == state_before.velocity.y);
    REQUIRE(ekf.state.position.x == state_before.position.x);
    REQUIRE(ekf.state.position.y == state_before.position.y);
}

TEST_CASE("EkfCore: a sustained GPS outage past the 10.0s timeout triggers a hard reset to the resumed sample, "
          "and gated fusion resumes normally afterward",
          "[ekf_core][fusion][cpp058]") {
    // Simulates a GPS receiver stuck reporting one fixed, wildly wrong
    // fix throughout the outage - the same order of magnitude already
    // proven above to fail the real gate (the "wild GPS glitch" tests),
    // just sustained repeatedly instead of injected once. Strapdown
    // dynamics are kept CLEAN (no unmodeled bias) so the EKF's own state
    // stays accurate and P grows only through ordinary process noise -
    // this isolates the thing actually under test (the timeout/reset
    // wiring) from the already-covered drift-vs-fusion behavior of
    // tests #5/#9 above. An earlier version of this test tried to force
    // the gate failure by injecting a huge (50 m/s^2) unmodeled accel
    // bias instead of a fixed bad GPS value - that backfired: upstream's
    // own (correctly transcribed) covariance-prediction Jacobian grows
    // velocity variance proportionally to the SQUARE of the actual
    // specific force being integrated (real physics: attitude
    // uncertainty converts to velocity error proportional to specific
    // force), so a 50 m/s^2 fake accel made P explode even faster than
    // the deterministic bias-driven error, and the gate never actually
    // failed. A fixed bad GPS value against small, stable P has no such
    // self-defeating feedback loop.
    const ftype dt = ftype(0.01);
    const ftype true_vx = ftype(5.0);
    const ftype outage_duration_s = ftype(13.0);  // comfortably past the 10.0s posRetryTimeUseVel_ms threshold
    const int total_steps = static_cast<int>(outage_duration_s / dt);
    const int gps_check_period_steps = 100;  // attempt a GPS check every 1.0s throughout the outage

    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.state.velocity = Vector3F(true_vx, ftype(0), ftype(0));
    ekf.covariance_init(dt);

    GyroSample gyro;
    gyro.delta_angle_dt = dt;  // zero rotation throughout
    AccelSample accel;
    accel.delta_velocity = Vector3F(ftype(0), ftype(0), -kGravity * dt);  // clean: gravity-cancelling only
    accel.delta_velocity_dt = dt;

    // Establish a healthy pass at t = 0, right before the outage begins.
    GpsSample gps0;
    gps0.velocity_ned = Vector3F(true_vx, ftype(0), ftype(0));
    gps0.position_ne = Vector2F(ftype(0), ftype(0));
    REQUIRE(ekf.fuse_gps_velocity(gps0, dt, ftype(0.0)) == 3);
    REQUIRE(ekf.fuse_gps_position(gps0, dt, ftype(0.0)) == 2);

    // The "stuck receiver" fix - same magnitude as test #9's closed-loop
    // glitch injection above (velocity off by 500 m/s, position off by
    // 5000 m), reused rather than re-derived since it was already proven
    // there to genuinely exceed the real VEL_I_GATE_DEFAULT/
    // POS_I_GATE_DEFAULT=500 gate against covariance that had grown over
    // a real multi-second closed-loop run (a plain +-50 offset, matching
    // this file's earlier single-shot glitch tests, turned out to be too
    // close to the real gate once P[7][7]/P[8][8] grow over this test's
    // own 13s outage span - position variance grows faster under
    // sustained pure prediction than those single-instant tests'
    // manually-set P implied).
    GpsSample stuck_gps;
    stuck_gps.velocity_ned = Vector3F(true_vx + ftype(500.0), ftype(0.0), ftype(0.0));
    stuck_gps.position_ne = Vector2F(ftype(5000.0), ftype(0.0));

    ftype elapsed_s = ftype(0.0);
    bool reset_seen = false;
    ftype reset_at_s = ftype(0.0);

    for (int step = 1; step <= total_steps; ++step) {
        ekf.update_strapdown_equations_ned(gyro, accel, dt);
        ekf.covariance_prediction(gyro, accel, dt);
        elapsed_s += dt;

        if (step % gps_check_period_steps != 0) {
            continue;
        }

        if (!reset_seen) {
            // Confirm this sample genuinely fails the real gate at every
            // attempt throughout the outage - state/P stay small and
            // stable under clean dynamics, so this fixed glitch keeps
            // failing exactly like the one-shot version already proven
            // above, not just on the first attempt.
            REQUIRE(static_cast<double>(ekf.gps_vel_test_ratio(stuck_gps)) >= 1.0);
            REQUIRE(static_cast<double>(ekf.gps_pos_test_ratio(stuck_gps)) >= 1.0);
        }

        const Matrix24 p_before = ekf.P;
        const StateVector state_before = ekf.state;

        const int n_vel = ekf.fuse_gps_velocity(stuck_gps, dt, elapsed_s);
        const int n_pos = ekf.fuse_gps_position(stuck_gps, dt, elapsed_s);

        if (!reset_seen && static_cast<double>(elapsed_s) < 10.0) {
            // Still inside the timeout window: a gate failure alone,
            // exactly CPP-057's existing behavior - P/state left
            // completely untouched.
            REQUIRE(n_vel == 0);
            REQUIRE(n_pos == 0);
            for (int i = 0; i < 24; ++i) {
                for (int j = 0; j < 24; ++j) {
                    REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                            p_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
                }
            }
            REQUIRE(ekf.state.velocity.x == state_before.velocity.x);
            REQUIRE(ekf.state.position.x == state_before.position.x);
        } else if (!reset_seen) {
            // The 10.0s timeout has now elapsed: this call must be a
            // HARD reset, not a gradual Kalman blend - state jumps
            // DIRECTLY to the (still wrong-looking, but now-trusted)
            // resumed GPS sample, and P is re-seeded to the real
            // reduced-scope formula (exact values, not merely "moved in
            // the right direction"). Upstream's real behavior at this
            // point is exactly "trust whatever GPS is available now,
            // right or wrong" (see ekf_core.hpp's "CPP-058, PHASE 4"
            // banner) - this stuck_gps value stands in for "GPS finally
            // reporting SOMETHING again", not necessarily truth.
            reset_seen = true;
            reset_at_s = elapsed_s;
            REQUIRE(n_vel == 0);  // a reset does not count as a fusion
            REQUIRE(n_pos == 0);
            REQUIRE(static_cast<double>(ekf.state.velocity.x) ==
                    Catch::Approx(static_cast<double>(stuck_gps.velocity_ned.x)).margin(1e-6));
            REQUIRE(static_cast<double>(ekf.state.velocity.y) == Catch::Approx(0.0).margin(1e-9));
            REQUIRE(static_cast<double>(ekf.state.position.x) ==
                    Catch::Approx(static_cast<double>(stuck_gps.position_ne.x)).margin(1e-6));
            REQUIRE(static_cast<double>(ekf.state.position.y) == Catch::Approx(0.0).margin(1e-9));
            REQUIRE(static_cast<double>(ekf.P[4][4]) ==
                    Catch::Approx(static_cast<double>(sq(ekf.gps_horiz_vel_noise))).margin(1e-9));
            REQUIRE(static_cast<double>(ekf.P[5][5]) ==
                    Catch::Approx(static_cast<double>(sq(ekf.gps_horiz_vel_noise))).margin(1e-9));
            REQUIRE(static_cast<double>(ekf.P[7][7]) ==
                    Catch::Approx(static_cast<double>(sq(ekf.gps_horiz_pos_noise))).margin(1e-9));
            REQUIRE(static_cast<double>(ekf.P[8][8]) ==
                    Catch::Approx(static_cast<double>(sq(ekf.gps_horiz_pos_noise))).margin(1e-9));
            break;
        }
    }

    REQUIRE(reset_seen);
    REQUIRE(static_cast<double>(reset_at_s) >= 10.0);

    // After the reset, normal CPP-057 gated fusion must resume working
    // against the new baseline: one more second of clean dead reckoning
    // from the just-reset state (unchanged dynamics - state tracks
    // itself essentially exactly, since there is no unmodeled bias here
    // to drift away from), followed by a GPS sample matching wherever
    // the vehicle now actually is, should PASS the gate and fuse
    // normally (n_fused > 0), not fail again immediately the way the
    // stuck receiver's fixed bad value would.
    for (int step = 0; step < gps_check_period_steps; ++step) {
        ekf.update_strapdown_equations_ned(gyro, accel, dt);
        ekf.covariance_prediction(gyro, accel, dt);
        elapsed_s += dt;
    }
    GpsSample gps_after;
    gps_after.velocity_ned = ekf.state.velocity;
    gps_after.position_ne = Vector2F(ekf.state.position.x, ekf.state.position.y);

    const int n_vel_after = ekf.fuse_gps_velocity(gps_after, dt, elapsed_s);
    const int n_pos_after = ekf.fuse_gps_position(gps_after, dt, elapsed_s);
    REQUIRE(n_vel_after > 0);
    REQUIRE(n_pos_after > 0);
}

// ============================================================================
// CPP-062 PHASE 8: baro height fusion. See ekf_core.hpp's "CPP-062, PHASE 8"
// banner for the full scope/exclusions/corrections discussion.
// ============================================================================

TEST_CASE("baro_hgt_obs_variance/hgt_test_ratio: match upstream's real posDownObsNoise/hgtTestRatio formulas",
          "[ekf_core][fusion][cpp062]") {
    EkfCore ekf;  // baro_alt_noise=3.0 (ALT_M_NSE_DEFAULT), hgt_innov_gate_pct=500 (HGT_I_GATE_DEFAULT), phase-1 defaults
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));

    // upstream: posDownObsNoise = sq(constrain_ftype(3.0, 0.1, 100.0)) = 9.0
    REQUIRE(static_cast<double>(ekf.baro_hgt_obs_variance()) == Catch::Approx(9.0));

    // Manually controlled P[9][9] (not covariance_init()) for a
    // hand-computable expected ratio, same convention as the GPS/mag
    // formula tests above.
    ekf.P[9][9] = ftype(1.0);
    ekf.state.position.z = ftype(0.0);
    const ftype baro_altitude_m = ftype(10.0);  // positive-up reading, 10m above origin

    // upstream: innovVelPos[5] = position.z - velPosObs[5] = position.z -
    // (-hgtMea) = position.z + hgtMea = 0 + 10 = 10. varInnovVelPos[5] =
    // P[9][9] + R_OBS_DATA_CHECKS[5] = 1.0 + 9.0 = 10.0. gate =
    // MAX(0.01*500,1.0) = 5.0, sq(gate) = 25. ratio = 100/(25*10) = 0.4.
    const double expected_ratio = 100.0 / (25.0 * 10.0);
    REQUIRE(static_cast<double>(ekf.hgt_test_ratio(baro_altitude_m)) == Catch::Approx(expected_ratio).margin(1e-6));
}

TEST_CASE("fuse_baro_height: a baro reading exactly consistent with the current altitude leaves state unchanged",
          "[ekf_core][fusion][cpp062]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));
    ekf.state.position.z = ftype(-100.0);  // 100m above origin, NED-down convention

    const ftype baro_altitude_m = ftype(100.0);  // exactly consistent positive-up reading
    const ftype p99_before = ekf.P[9][9];

    const bool applied = ekf.fuse_baro_height(baro_altitude_m, ftype(0.01));

    REQUIRE(applied);
    // Zero innovation -> zero correction regardless of Kalman gain -
    // state.position.z must be exactly unchanged (same convention as
    // ekf_mag_fusion_test.cpp's own "consistent reading leaves state
    // exactly unchanged" test).
    REQUIRE(static_cast<double>(ekf.state.position.z) == Catch::Approx(-100.0).margin(1e-9));
    // P[9][9] still shrinks (information is gained even from a
    // zero-innovation update - the Kalman gain is nonzero, only the
    // innovation itself is zero).
    REQUIRE(static_cast<double>(ekf.P[9][9]) < static_cast<double>(p99_before));
}

TEST_CASE("fuse_baro_height: an inconsistent-but-not-extreme reading measurably corrects state.position.z "
          "over repeated fusion",
          "[ekf_core][fusion][cpp062]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));
    ekf.state.position.z = ftype(-100.0);  // starts at 100m above origin

    // Comfortably inside the real HGT_I_GATE_DEFAULT=500 gate: P[9][9]=9.0
    // (covariance_init()'s own real baro_alt_noise-derived value), r_obs=9.0,
    // varInnov=18.0, gate_sq=25 -> pass threshold is innov^2 < 450, i.e.
    // |innov| < 21.2 - a 5m offset (innov=5) is nowhere near that boundary,
    // "inconsistent but not extreme" per the ticket's own wording.
    const ftype baro_altitude_m = ftype(105.0);  // sensor reads 105m - 5m high
    REQUIRE(static_cast<double>(ekf.hgt_test_ratio(baro_altitude_m)) < 1.0);

    double prev_err = std::abs(static_cast<double>(ekf.state.position.z) - (-105.0));
    for (int i = 0; i < 10; ++i) {
        const bool applied = ekf.fuse_baro_height(baro_altitude_m, ftype(0.01));
        REQUIRE(applied);
        const double err = std::abs(static_cast<double>(ekf.state.position.z) - (-105.0));
        // Each repeated fusion against the same consistent reading must
        // move state.position.z monotonically closer to -105 (textbook
        // convergent Kalman behavior against a fixed observation).
        REQUIRE(err <= prev_err);
        prev_err = err;
    }
    // After repeated fusion the state must have moved measurably (not a
    // no-op) toward the observation - the ticket's own explicit
    // acceptance criterion.
    REQUIRE(prev_err < 0.5);
}

TEST_CASE("fuse_baro_height: a wild baro glitch fails the real hgtTestRatio gate and leaves state/P untouched, "
          "with no reset",
          "[ekf_core][fusion][cpp062]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[9][9] = ftype(1.0);
    // varInnov = 1.0+9.0 = 10.0; pass threshold is innov^2 < 250 (see the
    // formula test above), i.e. |innov| < 15.81.
    ekf.state.position.z = ftype(0.0);

    const ftype baro_altitude_m = ftype(100.0);  // innov = 100, way over 15.81
    REQUIRE(static_cast<double>(ekf.hgt_test_ratio(baro_altitude_m)) > 1.0);

    const Matrix24 p_before = ekf.P;
    const StateVector state_before = ekf.state;

    // now_s defaults to 0, last_hgt_pass_time_s defaults to 0 -> elapsed
    // time is 0, well under the 10.0s hgtRetryTimeMode0_ms threshold, so
    // this must be a plain gate failure, NOT a timeout-triggered reset -
    // the distinction the ticket explicitly asks not to conflate.
    const bool applied = ekf.fuse_baro_height(baro_altitude_m, ftype(0.01));

    REQUIRE_FALSE(applied);
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    p_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
    REQUIRE(ekf.state.position.z == state_before.position.z);
    REQUIRE(ekf.state.velocity.z == state_before.velocity.z);
    REQUIRE(ekf.state.quat.q1 == state_before.quat.q1);
}

TEST_CASE("fuse_baro_height: a borderline-inside-gate sample still fuses normally", "[ekf_core][fusion][cpp062]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.P[9][9] = ftype(1.0);
    ekf.state.position.z = ftype(0.0);

    const ftype baro_altitude_m = ftype(15.7);  // innov = 15.7, just under 15.81
    const ftype ratio = ekf.hgt_test_ratio(baro_altitude_m);
    REQUIRE(static_cast<double>(ratio) < 1.0);
    REQUIRE(static_cast<double>(ratio) > 0.9);  // genuinely borderline, not trivially inside the gate

    const bool applied = ekf.fuse_baro_height(baro_altitude_m, ftype(0.01));

    REQUIRE(applied);
    REQUIRE(static_cast<double>(ekf.state.position.z) < 0.0);  // moved toward -15.7
}

TEST_CASE("fuse_baro_height: a gap under the 10.0s hgtRetryTimeMode0_ms timeout does not trigger a reset "
          "(regression)",
          "[ekf_core][fusion][cpp062]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));
    ekf.state.position.z = ftype(-50.0);

    // Establish a successful pass at now_s = 0 - anchors last_hgt_pass_time_s = 0.
    REQUIRE(ekf.fuse_baro_height(ftype(50.0), ftype(0.01), ftype(0.0)));

    // Same wild-glitch magnitude class as the gate test above, genuinely
    // failing hgt_test_ratio() - but at now_s = 9.0, i.e. elapsed = 9.0s,
    // still inside the 10.0s hgtRetryTimeMode0_ms window.
    const ftype glitch_baro_altitude_m = ftype(500.0);
    REQUIRE(static_cast<double>(ekf.hgt_test_ratio(glitch_baro_altitude_m)) > 1.0);

    const Matrix24 p_before = ekf.P;
    const StateVector state_before = ekf.state;

    const bool applied = ekf.fuse_baro_height(glitch_baro_altitude_m, ftype(0.01), ftype(9.0));

    REQUIRE_FALSE(applied);
    // No reset: P/state byte-for-byte untouched, exactly matching the
    // gate-rejection behavior above - the timeout window has not yet
    // elapsed, so the gate failure alone is still the only thing that
    // happened.
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 24; ++j) {
            REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                    p_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
    }
    REQUIRE(ekf.state.position.z == state_before.position.z);
}

TEST_CASE("EkfCore: a sustained baro outage past the 10.0s hgtRetryTimeMode0_ms timeout triggers reset_height(), "
          "and gated fusion resumes normally afterward",
          "[ekf_core][fusion][cpp062]") {
    // Same methodology as the analogous GPS sustained-outage test above:
    // CLEAN dynamics (gravity-cancelling accel, zero rotation - state stays
    // near its initial altitude, P grows only through ordinary process
    // noise) so this isolates the timeout/reset wiring itself from the
    // already-covered drift-vs-fusion behavior of the earlier tests.
    const ftype dt = ftype(0.01);
    const ftype outage_duration_s = ftype(13.0);  // comfortably past the 10.0s hgtRetryTimeMode0_ms threshold
    const int total_steps = static_cast<int>(outage_duration_s / dt);
    const int baro_check_period_steps = 100;  // attempt a baro check every 1.0s throughout the outage

    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.state.position.z = ftype(0.0);
    ekf.covariance_init(dt);

    GyroSample gyro;
    gyro.delta_angle_dt = dt;  // zero rotation throughout
    AccelSample accel;
    accel.delta_velocity = Vector3F(ftype(0), ftype(0), -kGravity * dt);  // clean: gravity-cancelling only
    accel.delta_velocity_dt = dt;

    // Establish a healthy pass at t = 0, right before the outage begins.
    REQUIRE(ekf.fuse_baro_height(ftype(0.0), dt, ftype(0.0)));

    // The "stuck sensor" reading - a wildly wrong altitude (-1000m,
    // i.e. reporting the vehicle is 1000m BELOW its actual position),
    // comfortably exceeding the real HGT_I_GATE_DEFAULT=500 gate against
    // covariance that has grown over a real multi-second run.
    const ftype stuck_baro_altitude_m = ftype(-1000.0);

    ftype elapsed_s = ftype(0.0);
    bool reset_seen = false;
    ftype reset_at_s = ftype(0.0);

    for (int step = 1; step <= total_steps; ++step) {
        ekf.update_strapdown_equations_ned(gyro, accel, dt);
        ekf.covariance_prediction(gyro, accel, dt);
        elapsed_s += dt;

        if (step % baro_check_period_steps != 0) {
            continue;
        }

        if (!reset_seen) {
            // Confirm this sample genuinely fails the real gate at every
            // attempt throughout the outage.
            REQUIRE(static_cast<double>(ekf.hgt_test_ratio(stuck_baro_altitude_m)) >= 1.0);
        }

        const Matrix24 p_before = ekf.P;
        const StateVector state_before = ekf.state;

        const bool applied = ekf.fuse_baro_height(stuck_baro_altitude_m, dt, elapsed_s);

        if (!reset_seen && static_cast<double>(elapsed_s) < 10.0) {
            // Still inside the timeout window: a gate failure alone -
            // P/state left completely untouched.
            REQUIRE_FALSE(applied);
            for (int i = 0; i < 24; ++i) {
                for (int j = 0; j < 24; ++j) {
                    REQUIRE(ekf.P[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] ==
                            p_before[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
                }
            }
            REQUIRE(ekf.state.position.z == state_before.position.z);
        } else if (!reset_seen) {
            // The 10.0s timeout has now elapsed: this call must be a HARD
            // reset, not a gradual Kalman blend - state.position.z jumps
            // DIRECTLY to -stuck_baro_altitude_m and P[9][9] is re-seeded
            // to baro_hgt_obs_variance() exactly.
            reset_seen = true;
            reset_at_s = elapsed_s;
            REQUIRE_FALSE(applied);  // a reset does not count as a fusion
            REQUIRE(static_cast<double>(ekf.state.position.z) ==
                    Catch::Approx(-static_cast<double>(stuck_baro_altitude_m)).margin(1e-6));
            REQUIRE(static_cast<double>(ekf.P[9][9]) ==
                    Catch::Approx(static_cast<double>(ekf.baro_hgt_obs_variance())).margin(1e-9));
            break;
        }
    }

    REQUIRE(reset_seen);
    REQUIRE(static_cast<double>(reset_at_s) >= 10.0);

    // After the reset, normal gated fusion must resume working against the
    // new baseline: one more second of clean dead reckoning from the
    // just-reset state, followed by a baro reading matching wherever the
    // vehicle now actually is, should PASS the gate and fuse normally.
    for (int step = 0; step < baro_check_period_steps; ++step) {
        ekf.update_strapdown_equations_ned(gyro, accel, dt);
        ekf.covariance_prediction(gyro, accel, dt);
        elapsed_s += dt;
    }
    const ftype baro_altitude_after = -ekf.state.position.z;  // matches current altitude exactly

    const bool applied_after = ekf.fuse_baro_height(baro_altitude_after, dt, elapsed_s);
    REQUIRE(applied_after);
}


// ============================================================================
// CPP-067 PHASE 13: time-correct GPS sample recall via ObsBuffer. See
// ekf_core.hpp's "CPP-067, PHASE 13" banner for the full scope/reasoning
// (the deliberately-narrower recall-against-caller's-own-now_s design,
// the buffer-size justification, and why fuse_gps_velocity()/
// fuse_gps_position() above are kept completely unchanged - the new
// capability is delivered entirely by the two additions exercised here:
// push_gps_sample()/recall_gps_sample()).
//
// Test strategy (per the ticket's own acceptance criteria):
//   19. GpsSample::set_time_s()/time_s() round-trip at millisecond
//       resolution (the disclosed ObsElement::time_ms quantization).
//   20. recall_gps_sample() on an empty buffer returns false and leaves
//       `out` untouched.
//   21. THE REAL, NEW CAPABILITY: push several GPS samples at DIFFERENT,
//       non-tick-aligned timestamps (arrival times that are not multiples
//       of the 50Hz/20ms tick grid - simulating a real GPS driver
//       callback firing asynchronously relative to the EKF's own
//       scheduling, exactly as upstream's readGpsData()/
//       SelectVelPosFusion() are two independently-scheduled functions),
//       then confirm fusion at each 50Hz tick correctly recalls the
//       most-recently-available sample - the test never calls
//       push/recall at matching times, and never hand-feeds a fusion
//       call "the right sample" the way every fuse_gps_velocity()/
//       fuse_gps_position() call above this section does.
// ============================================================================

TEST_CASE("GpsSample::set_time_s/time_s round-trips at millisecond resolution", "[ekf_core][fusion][gps_buffer]") {
    GpsSample gps;
    REQUIRE(static_cast<double>(gps.time_s()) == Catch::Approx(0.0));  // zero-initialized ObsElement::time_ms

    gps.set_time_s(ftype(1.234));
    REQUIRE(static_cast<double>(gps.time_s()) == Catch::Approx(1.234).margin(1e-6));

    gps.set_time_s(ftype(0.0));
    REQUIRE(static_cast<double>(gps.time_s()) == Catch::Approx(0.0));

    // Defensive negative clamp (see GpsSample::set_time_s()'s own doc
    // comment - now_s is never legitimately negative in this port's own
    // convention, but the clamp avoids UB rather than assuming callers
    // never pass one).
    gps.set_time_s(ftype(-5.0));
    REQUIRE(static_cast<double>(gps.time_s()) == Catch::Approx(0.0));
}

TEST_CASE("recall_gps_sample: returns false on an empty buffer", "[ekf_core][fusion][gps_buffer]") {
    EkfCore ekf;
    GpsSample out;
    out.velocity_ned = Vector3F(ftype(99.0), ftype(99.0), ftype(99.0));  // sentinel - must be untouched

    REQUIRE_FALSE(ekf.recall_gps_sample(out, ftype(0.05)));
    REQUIRE(ekf.gps_buffer.empty());
    // Untouched on failure, matching ObsBuffer::recall()'s own documented
    // contract (ekf_buffer.hpp).
    REQUIRE(static_cast<double>(out.velocity_ned.x) == Catch::Approx(99.0));
}

TEST_CASE("push_gps_sample/recall_gps_sample: recalls the correct sample under realistic "
          "asynchronous (non-tick-aligned) GPS arrival, without the caller hand-feeding "
          "exactly the right sample synchronously",
          "[ekf_core][fusion][gps_buffer]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));

    constexpr ftype kDt = ftype(0.02);  // 50Hz IMU tick, matching this port's own
                                         // closed-loop-test precedent (ekf_closed_loop_test.cpp).

    // Four GPS fixes, each carrying its own TRUE arrival timestamp - none
    // of them a multiple of kDt (0.02s), i.e. none would ever land exactly
    // on an IMU tick boundary (verified: 83, 287, 501, 734 mod 20 are all
    // nonzero). Each carries a distinct, tagged velocity_ned.x/position_ne.x
    // value so a successful recall can be matched back to exactly which
    // fix it returned, not merely "some fix or other".
    struct Fix {
        ftype arrival_s;
        ftype tag;
        bool pushed = false;
    };
    std::array<Fix, 4> fixes{{
        {ftype(0.083), ftype(1.0)},
        {ftype(0.287), ftype(2.0)},
        {ftype(0.501), ftype(3.0)},
        {ftype(0.734), ftype(4.0)},
    }};

    std::vector<ftype> recalled_tags;
    std::vector<ftype> recall_latency_s;  // now_s at recall time - the sample's own arrival_s

    for (int tick = 1; tick <= 60; ++tick) {  // 60 * 20ms = 1.2s, comfortably past the last fix
        const ftype now_s = static_cast<ftype>(tick) * kDt;

        // --- Sample arrival: modelled as an independently-scheduled event
        // (upstream: readGpsData(), its own function, called on its own
        // terms) that pushes into gps_buffer AS SOON AS it has arrived,
        // stamped with its OWN true arrival time - NOT now_s, and NOT
        // aligned to the tick grid at all. This is the realistic
        // asynchronous-arrival part: the caller's EKF loop only gets a
        // chance to call push_gps_sample() once per tick, but the sample
        // it pushes carries whatever timestamp the GPS itself reported. ---
        for (Fix& fix : fixes) {
            if (!fix.pushed && now_s >= fix.arrival_s) {
                GpsSample gps;
                gps.set_time_s(fix.arrival_s);
                gps.velocity_ned = Vector3F(fix.tag, ftype(0), ftype(0));
                gps.position_ne = Vector2F(fix.tag, ftype(0));
                ekf.push_gps_sample(gps);
                fix.pushed = true;
            }
        }

        // --- Fusion attempted EVERY tick (upstream: SelectVelPosFusion()
        // runs every EKF cycle) - recall_gps_sample() decides, purely from
        // timestamps, whether a time-eligible sample exists yet. THE TEST
        // NEVER TELLS IT WHICH TICK TO EXPECT A MATCH ON - that is exactly
        // the capability this ticket adds (contrast with every
        // fuse_gps_velocity(gps, ...)/fuse_gps_position(gps, ...) call
        // above this section, which hands over a hand-built GpsSample
        // directly, on the exact call the test chooses). ---
        GpsSample recalled;
        if (ekf.recall_gps_sample(recalled, now_s)) {
            recalled_tags.push_back(recalled.velocity_ned.x);
            recall_latency_s.push_back(now_s - recalled.time_s());

            // The full pipeline: fuse the recalled sample exactly like any
            // other GpsSample - fuse_gps_velocity()/fuse_gps_position()
            // themselves are completely unchanged by this ticket.
            ekf.fuse_gps_velocity(recalled, kDt, now_s);
            ekf.fuse_gps_position(recalled, kDt, now_s);
        }
    }

    // All 4 asynchronously-arriving fixes were recalled exactly once each,
    // in arrival order, none lost and none double-counted - the real
    // acceptance criterion: fusion recalled the right sample at the right
    // (later, tick-quantized) time without ever being handed it directly.
    REQUIRE(recalled_tags.size() == 4);
    REQUIRE(static_cast<double>(recalled_tags[0]) == Catch::Approx(1.0));
    REQUIRE(static_cast<double>(recalled_tags[1]) == Catch::Approx(2.0));
    REQUIRE(static_cast<double>(recalled_tags[2]) == Catch::Approx(3.0));
    REQUIRE(static_cast<double>(recalled_tags[3]) == Catch::Approx(4.0));

    // Each recall happened PROMPTLY after its sample's true arrival - at
    // most one tick (kDt) of latency (the recalling tick is the first
    // tick at/after arrival_s), and never before arrival (recall() would
    // have left a strictly-newer element untouched - ekf_buffer.hpp).
    for (const ftype latency : recall_latency_s) {
        REQUIRE(static_cast<double>(latency) >= 0.0);
        REQUIRE(static_cast<double>(latency) < static_cast<double>(kDt) + 1e-9);
    }

    // The buffer is drained back to empty - every pushed sample was
    // consumed by exactly one recall, none left stranded.
    REQUIRE(ekf.gps_buffer.empty());
}

// ============================================================================
// CPP-069 PHASE 15: time-correct baro sample recall via ObsBuffer. See
// ekf_core.hpp's "CPP-069, PHASE 15" banner (above EkfCore::baro_buffer)
// and BaroSample's own banner (above its struct definition, near
// GpsSample/MagSample) for the full scope/reasoning: the deliberately
// narrower recall-against-caller's-own-now_s design (identical to CPP-067/
// 068's own), the independently-derived buffer-size justification, the
// real bare-scalar-vs-wrapper-struct tension this ticket resolves (unlike
// GPS/mag, which already had a real struct to extend), why
// fuse_baro_height() itself is kept completely unchanged, and why
// recall_baro_sample() is the ONLY consumer of baro_buffer (verified: baro
// has exactly one storedBaro.recall() call site upstream, simpler even
// than mag's two-site situation).
//
// Test strategy - mirrors CPP-067/068's own shape exactly:
//   22. BaroSample::set_time_s()/time_s() round-trip at millisecond
//       resolution (the disclosed ObsElement::time_ms quantization).
//   23. recall_baro_sample() on an empty buffer returns false and leaves
//       `out` untouched.
//   24. THE REAL, NEW CAPABILITY: push several baro readings at DIFFERENT,
//       non-tick-aligned timestamps (arrival times that are not multiples
//       of the 50Hz/20ms tick grid, modelling a real baro driver
//       reporting on its own schedule, independent of the EKF's own),
//       then confirm fusion at each 50Hz tick correctly recalls the
//       most-recently-available reading and hands the unwrapped bare
//       `ftype` to the completely-unchanged fuse_baro_height() - the test
//       never calls push/recall at matching times, and never hand-feeds a
//       fusion call "the right reading" the way every other
//       fuse_baro_height() call above this section does.
// ============================================================================

TEST_CASE("BaroSample::set_time_s/time_s round-trips at millisecond resolution", "[ekf_core][fusion][baro_buffer]") {
    BaroSample baro;
    REQUIRE(static_cast<double>(baro.time_s()) == Catch::Approx(0.0));  // zero-initialized ObsElement::time_ms

    baro.set_time_s(ftype(1.234));
    REQUIRE(static_cast<double>(baro.time_s()) == Catch::Approx(1.234).margin(1e-6));

    baro.set_time_s(ftype(0.0));
    REQUIRE(static_cast<double>(baro.time_s()) == Catch::Approx(0.0));

    // Defensive negative clamp (see BaroSample::set_time_s()'s own doc
    // comment - now_s is never legitimately negative in this port's own
    // convention, but the clamp avoids UB rather than assuming callers
    // never pass one).
    baro.set_time_s(ftype(-5.0));
    REQUIRE(static_cast<double>(baro.time_s()) == Catch::Approx(0.0));
}

TEST_CASE("recall_baro_sample: returns false on an empty buffer", "[ekf_core][fusion][baro_buffer]") {
    EkfCore ekf;
    BaroSample out;
    out.altitude_m = ftype(99.0);  // sentinel - must be untouched

    REQUIRE_FALSE(ekf.recall_baro_sample(out, ftype(0.05)));
    REQUIRE(ekf.baro_buffer.empty());
    // Untouched on failure, matching ObsBuffer::recall()'s own documented
    // contract (ekf_buffer.hpp).
    REQUIRE(static_cast<double>(out.altitude_m) == Catch::Approx(99.0));
}

TEST_CASE("push_baro_sample/recall_baro_sample: recalls the correct sample under realistic "
          "asynchronous (non-tick-aligned) baro arrival, without the caller hand-feeding "
          "exactly the right reading synchronously, and hands the unwrapped bare ftype to the "
          "unchanged fuse_baro_height()",
          "[ekf_core][fusion][baro_buffer]") {
    EkfCore ekf;
    ekf.state.quat = QuaternionF(ftype(1), ftype(0), ftype(0), ftype(0));
    ekf.covariance_init(ftype(0.01));

    constexpr ftype kDt = ftype(0.02);  // 50Hz IMU tick, matching this port's own
                                         // closed-loop-test precedent (ekf_closed_loop_test.cpp).

    // Four baro readings, each carrying its own TRUE arrival timestamp -
    // none of them a multiple of kDt (0.02s), i.e. none would ever land
    // exactly on an IMU tick boundary (verified: 91, 293, 512, 741 mod 20
    // are all nonzero). Each carries a distinct, tagged altitude_m value
    // (deliberately close to the state's own initial altitude of 0, so
    // every reading passes the real hgtTestRatio gate and actually fuses -
    // this test is about recall correctness, not gate behavior, already
    // covered above) so a successful recall can be matched back to exactly
    // which reading it returned, not merely "some reading or other".
    struct Reading {
        ftype arrival_s;
        ftype tag;
        bool pushed = false;
    };
    std::array<Reading, 4> readings{{
        {ftype(0.091), ftype(0.10)},
        {ftype(0.293), ftype(0.20)},
        {ftype(0.512), ftype(0.30)},
        {ftype(0.741), ftype(0.40)},
    }};

    std::vector<ftype> recalled_tags;
    std::vector<ftype> recall_latency_s;  // now_s at recall time - the sample's own arrival_s

    for (int tick = 1; tick <= 60; ++tick) {  // 60 * 20ms = 1.2s, comfortably past the last reading
        const ftype now_s = static_cast<ftype>(tick) * kDt;

        // --- Sample arrival: modelled as an independently-scheduled event
        // (upstream: readBaroData(), its own function, called on its own
        // terms) that pushes into baro_buffer AS SOON AS it has arrived,
        // stamped with its OWN true arrival time - NOT now_s, and NOT
        // aligned to the tick grid at all. This is the realistic
        // asynchronous-arrival part: the caller's EKF loop only gets a
        // chance to call push_baro_sample() once per tick, but the sample
        // it pushes carries whatever timestamp the baro driver itself
        // reported. ---
        for (Reading& reading : readings) {
            if (!reading.pushed && now_s >= reading.arrival_s) {
                BaroSample baro;
                baro.set_time_s(reading.arrival_s);
                baro.altitude_m = reading.tag;
                ekf.push_baro_sample(baro);
                reading.pushed = true;
            }
        }

        // --- Fusion attempted EVERY tick (upstream: selectHeightForFusion()
        // runs every EKF cycle) - recall_baro_sample() decides, purely from
        // timestamps, whether a time-eligible reading exists yet. THE TEST
        // NEVER TELLS IT WHICH TICK TO EXPECT A MATCH ON - that is exactly
        // the capability this ticket adds (contrast with every
        // fuse_baro_height(baro_altitude_m, ...) call above this section,
        // which hands over a hand-built bare ftype directly, on the exact
        // call the test chooses). ---
        BaroSample recalled;
        if (ekf.recall_baro_sample(recalled, now_s)) {
            recalled_tags.push_back(recalled.altitude_m);
            recall_latency_s.push_back(now_s - recalled.time_s());

            // The full pipeline: unwrap the recalled BaroSample back to a
            // bare ftype and fuse it exactly like any other reading -
            // fuse_baro_height() itself is completely unchanged by this
            // ticket (see BaroSample's own "THE BARE-SCALAR-VS-WRAPPER-
            // STRUCT TENSION" banner).
            ekf.fuse_baro_height(recalled.altitude_m, kDt, now_s);
        }
    }

    // All 4 asynchronously-arriving readings were recalled exactly once
    // each, in arrival order, none lost and none double-counted - the real
    // acceptance criterion: fusion recalled the right reading at the right
    // (later, tick-quantized) time without ever being handed it directly.
    REQUIRE(recalled_tags.size() == 4);
    REQUIRE(static_cast<double>(recalled_tags[0]) == Catch::Approx(0.10));
    REQUIRE(static_cast<double>(recalled_tags[1]) == Catch::Approx(0.20));
    REQUIRE(static_cast<double>(recalled_tags[2]) == Catch::Approx(0.30));
    REQUIRE(static_cast<double>(recalled_tags[3]) == Catch::Approx(0.40));

    // Each recall happened PROMPTLY after its sample's true arrival - at
    // most one tick (kDt) of latency (the recalling tick is the first
    // tick at/after arrival_s), and never before arrival (recall() would
    // have left a strictly-newer element untouched - ekf_buffer.hpp).
    for (const ftype latency : recall_latency_s) {
        REQUIRE(static_cast<double>(latency) >= 0.0);
        REQUIRE(static_cast<double>(latency) < static_cast<double>(kDt) + 1e-9);
    }

    // The buffer is drained back to empty - every pushed sample was
    // consumed by exactly one recall, none left stranded.
    REQUIRE(ekf.baro_buffer.empty());
}
