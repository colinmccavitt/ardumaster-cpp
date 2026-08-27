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
    gps.position_ne = Vector2F(ftype(100.0), ftype(-50.0));

    REQUIRE(ekf.fuse_gps_velocity(gps, ftype(0.01)) == 3);
    REQUIRE(ekf.fuse_gps_position(gps, ftype(0.01)) == 2);

    // State should have moved measurably toward the observation (not a
    // no-op) - P[4][4]/P[7][7] etc from covariance_init() are well above
    // zero, so Kalman gain is meaningfully nonzero.
    REQUIRE(static_cast<double>(ekf.state.velocity.x) > 0.0);
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
