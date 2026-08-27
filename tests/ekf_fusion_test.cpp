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

#include <array>
#include <cmath>

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
