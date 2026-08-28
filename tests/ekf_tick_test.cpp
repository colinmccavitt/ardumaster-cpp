// Tests for fwcpp::ekf::EkfCore::tick() - CPP-071, PHASE 17: IMU history
// buffering and delayed-state mechanization. See ekf_core.hpp's own
// "CPP-071, PHASE 17" banner (above EkfCore::tick()'s declaration) for
// the full upstream-verification, delay-depth-derivation, and
// pre-fill-strategy discussion this file's tests exercise.
//
// A NEW, standalone test file - touches nothing in ekf_core_test.cpp (or
// any other existing test file). update_strapdown_equations_ned()'s and
// covariance_prediction()'s own direct-call API, and every test that
// exercises them directly, is completely unaffected by tick()'s
// existence - this file exists specifically to test the NEW, separate
// entry point additively.
//
// Test strategy:
//   1. "tick() pre-fill is a true no-op, not garbage": for the first
//      kImuBufferCapacity-1 calls (before the buffer has been pushed to
//      even once for real "oldest" data), tick() must read back the
//      seeded stationary/level no-op sample (see hpp banner) regardless
//      of what real, DISTINGUISHABLE (varying angular rate) gyro/accel
//      the caller is actually feeding it - i.e. the real input is
//      genuinely buffered/ignored-for-now, not silently applied. Checked
//      by exact agreement against an independent reference EkfCore
//      stepped directly (via update_strapdown_equations_ned()/
//      covariance_prediction()) with the SAME seed sample, and checked
//      to DIFFER from a third EkfCore fed the real input directly.
//   2. "tick() genuinely produces an exactly N-1-tick-delayed state once
//      filled": continues the same distinguishable input sequence well
//      past the buffer's capacity and shows, at every subsequent tick k,
//      ekf.state (via tick()) is bit-for-bit identical to an
//      independent reference fed the manually-shifted (seed x(N-1), then
//      real sample 1, 2, ...) sequence directly - i.e. tick()'s internal
//      bookkeeping reproduces the delay exactly, not approximately - AND
//      differs from a fourth EkfCore fed the real sequence directly and
//      immediately (proving the delay is real, not a no-op passthrough
//      that merely happens to look delayed).
//   3. delayed_time_s advances by dt_ekf_avg every tick() call,
//      regardless of pre-fill/filled state (upstream: imuDataDelayed's
//      own timestamp concept - see hpp banner).
//   4. imu_buffer_seeded flips true after exactly the first tick() call,
//      never again.
//   5. CPP-073: under a REALISTIC non-zero starting velocity (unlike
//      every test above, which - like CPP-071's own original tests -
//      starts from a stationary, zero-velocity vehicle), state.position
//      must NOT drift during the pre-fill window (the bug CPP-072's own
//      closed-loop test found and disclosed, now fixed by tick()'s
//      snapshot/restore of state.position while ticks_since_seed <
//      kImuBufferCapacity-1 - see ekf_core.hpp's "CPP-073 ADDENDUM" and
//      ekf_core.cpp's EkfCore::tick() implementation), while velocity and
//      attitude continue to behave exactly as tests 1-4 above already
//      prove (unaffected by this fix - those tests are unchanged by this
//      ticket and still pass bit-for-bit identically).

#include <cmath>
#include <cstddef>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/ekf/ekf_core.hpp>

using namespace fwcpp::ekf;

namespace {
constexpr ftype kGravity = static_cast<ftype>(9.80665);
constexpr ftype kDt = static_cast<ftype>(0.02); // this port's real 50Hz tick rate

// Same StateVector-relevant fields the existing ekf_core_test.cpp
// comparisons use - a small helper so multi-field REQUIREs below stay
// readable.
void require_state_equal(const EkfCore& a, const EkfCore& b) {
    REQUIRE(a.state.quat.q1 == b.state.quat.q1);
    REQUIRE(a.state.quat.q2 == b.state.quat.q2);
    REQUIRE(a.state.quat.q3 == b.state.quat.q3);
    REQUIRE(a.state.quat.q4 == b.state.quat.q4);
    REQUIRE(a.state.velocity.x == b.state.velocity.x);
    REQUIRE(a.state.velocity.y == b.state.velocity.y);
    REQUIRE(a.state.velocity.z == b.state.velocity.z);
    REQUIRE(a.state.position.x == b.state.position.x);
    REQUIRE(a.state.position.y == b.state.position.y);
    REQUIRE(a.state.position.z == b.state.position.z);
    REQUIRE(a.P[16][16] == b.P[16][16]);
}

bool state_differs(const EkfCore& a, const EkfCore& b) {
    return a.state.quat.q2 != b.state.quat.q2 || a.state.quat.q3 != b.state.quat.q3 ||
           a.state.quat.q4 != b.state.quat.q4;
}

// A distinguishable (varying, not repeating) gyro/accel sample sequence:
// angular rate about the body z-axis increases every tick, so each
// sample produces a genuinely different rotation than the last -
// distinguishing "which sample was actually mechanized" is possible by
// comparing states, not just by comparing inputs.
GyroSample real_gyro(int tick_index) {
    GyroSample g;
    const ftype omega_z = static_cast<ftype>(0.05) * static_cast<ftype>(tick_index); // rad/s, grows each tick
    g.delta_angle = Vector3F(ftype(0), ftype(0), omega_z * kDt);
    g.delta_angle_dt = kDt;
    return g;
}

AccelSample real_accel(int tick_index) {
    AccelSample a;
    // Also distinguishable in the vertical channel: a small, growing net
    // vertical specific-force perturbation on top of the gravity-cancelling
    // baseline, so translation state is exercised too, not just attitude.
    const ftype perturbation = static_cast<ftype>(0.01) * static_cast<ftype>(tick_index);
    a.delta_velocity = Vector3F(ftype(0), ftype(0), (-kGravity + perturbation) * kDt);
    a.delta_velocity_dt = kDt;
    return a;
}

// The stationary/level no-op sample tick()'s own pre-fill seeding uses -
// see ekf_core.hpp's "CPP-071, PHASE 17" banner's "PRE-FILL STRATEGY"
// section and ekf_core.cpp's EkfCore::tick() implementation. Reproduced
// independently here (not read back from EkfCore) so this test doesn't
// just tautologically agree with the implementation's own seed.
GyroSample seed_gyro() {
    GyroSample g;
    g.delta_angle_dt = kDt;
    return g;
}

AccelSample seed_accel() {
    AccelSample a;
    a.delta_velocity = Vector3F(ftype(0), ftype(0), -kGravity * kDt);
    a.delta_velocity_dt = kDt;
    return a;
}

} // namespace

TEST_CASE("EkfCore::tick pre-fill reads are a true stationary no-op, not the real input",
          "[ekf_core][tick]") {
    EkfCore ticked; // driven via tick()
    EkfCore ref_seed; // independent reference, manually fed the SAME seed
    EkfCore ref_real; // independent reference, manually fed the REAL input directly

    const std::size_t n = EkfCore::kImuBufferCapacity;
    REQUIRE(n == 11); // pins this test's own tick-count reasoning below to the real, chosen N

    // For tick indices 1..N-1, get_oldest_element() must still be
    // reading an UNWRITTEN (seeded) slot - see hpp banner's "VERIFIED,
    // EXACT DELAY-VS-CAPACITY RELATIONSHIP". Verify this for every one
    // of those ticks, not just the last.
    for (std::size_t k = 1; k < n; ++k) {
        const int tick_index = static_cast<int>(k);
        const GyroSample g = real_gyro(tick_index);
        const AccelSample a = real_accel(tick_index);

        ticked.tick(g, a, kDt);
        ref_seed.update_strapdown_equations_ned(seed_gyro(), seed_accel(), kDt);
        ref_seed.covariance_prediction(seed_gyro(), seed_accel(), kDt);
        ref_real.update_strapdown_equations_ned(g, a, kDt);
        ref_real.covariance_prediction(g, a, kDt);

        REQUIRE_FALSE(ticked.imu_buffer.is_filled());
        require_state_equal(ticked, ref_seed);
        // Sanity: the real input, if it HAD been applied instead, would
        // have produced a visibly different attitude by now - confirms
        // ref_real is actually a meaningfully different trajectory, so
        // require_state_equal(ticked, ref_seed) above is a real
        // assertion, not a vacuous one.
        if (k >= 5) {
            REQUIRE(state_differs(ticked, ref_real));
        }
    }

    // delayed_time_s advances by dt_ekf_avg every call, pre-fill included.
    REQUIRE(static_cast<double>(ticked.delayed_time_s) ==
            Catch::Approx(static_cast<double>(kDt) * static_cast<double>(n - 1)));
}

TEST_CASE("EkfCore::tick seeds imu_buffer_seeded exactly once, on the first call",
          "[ekf_core][tick]") {
    EkfCore ekf;
    REQUIRE_FALSE(ekf.imu_buffer_seeded);
    ekf.tick(real_gyro(1), real_accel(1), kDt);
    REQUIRE(ekf.imu_buffer_seeded);

    // A second call must not re-seed (reset_history() would silently
    // overwrite already-real buffered history if it fired again) -
    // checked indirectly: after enough ticks to fill the buffer, the
    // delayed sample traces back to real tick 1's own input, not some
    // later re-seed. Full delay-correctness is exercised by the
    // "genuinely produces a delayed state" test below; this test only
    // pins the seed-once bookkeeping itself.
    for (int k = 2; k <= 20; ++k) {
        ekf.tick(real_gyro(k), real_accel(k), kDt);
    }
    REQUIRE(ekf.imu_buffer_seeded);
}

TEST_CASE("EkfCore::tick genuinely produces a state delayed by exactly kImuBufferCapacity-1 "
          "ticks once the buffer is filled",
          "[ekf_core][tick]") {
    EkfCore ticked; // driven via tick() - the thing under test
    EkfCore ref_shifted; // independent reference: manually fed the EXACT
                         // sequence tick() should internally be mechanizing
                         // (seed x(N-1), then real sample 1, 2, 3, ...)
    EkfCore ref_immediate; // independent reference: manually fed the REAL
                           // sequence directly and immediately, no delay -
                           // represents "what direct/undelayed mechanization
                           // would give" at each tick.

    const std::size_t n = EkfCore::kImuBufferCapacity; // 11
    const int total_ticks = 40; // well past N, exercises steady-state wraparound too

    // Snapshots of ref_immediate's state after each of its own ticks, so
    // "ticked.state at tick k equals ref_immediate.state at tick k-(N-1)"
    // can be checked without re-deriving ref_immediate's history.
    std::vector<EkfCore> immediate_snapshots;
    immediate_snapshots.reserve(static_cast<std::size_t>(total_ticks) + 1);

    for (int k = 1; k <= total_ticks; ++k) {
        const GyroSample g = real_gyro(k);
        const AccelSample a = real_accel(k);

        ticked.tick(g, a, kDt);

        // ref_shifted: for k <= n-1, mechanize the seed (matching
        // tick()'s own pre-fill window exactly); for k >= n, mechanize
        // real sample (k-(n-1)) - i.e. the manually-shifted sequence.
        if (static_cast<std::size_t>(k) < n) {
            ref_shifted.update_strapdown_equations_ned(seed_gyro(), seed_accel(), kDt);
            ref_shifted.covariance_prediction(seed_gyro(), seed_accel(), kDt);
        } else {
            const int delayed_index = k - (static_cast<int>(n) - 1);
            ref_shifted.update_strapdown_equations_ned(real_gyro(delayed_index), real_accel(delayed_index), kDt);
            ref_shifted.covariance_prediction(real_gyro(delayed_index), real_accel(delayed_index), kDt);
        }

        ref_immediate.update_strapdown_equations_ned(g, a, kDt);
        ref_immediate.covariance_prediction(g, a, kDt);
        immediate_snapshots.push_back(ref_immediate);

        // Exact agreement with the manually-shifted reference at EVERY
        // tick, pre-fill and post-fill alike - proves tick()'s internal
        // delay bookkeeping is precise, not just "eventually close".
        require_state_equal(ticked, ref_shifted);

        // Once filled (k >= n), ticked's state must equal ref_immediate's
        // state from exactly n-1 ticks ago - the actual "lags by exactly
        // the buffer's configured depth" property this ticket exists to
        // demonstrate. immediate_snapshots is 0-indexed by (tick-1), so
        // "after tick (k-(n-1))" is at index (k-(n-1))-1.
        if (static_cast<std::size_t>(k) >= n) {
            const int delayed_index = k - (static_cast<int>(n) - 1); // 1-based tick number
            require_state_equal(ticked, immediate_snapshots[static_cast<std::size_t>(delayed_index - 1)]);
        }

        // And ticked must NOT equal ref_immediate's CURRENT (undelayed)
        // state, confirming genuine lag rather than a coincidental
        // no-delay match, once the signal has grown enough to matter.
        if (k >= 5) {
            REQUIRE(state_differs(ticked, ref_immediate));
        }
    }

    REQUIRE(ticked.imu_buffer.is_filled());
    REQUIRE(static_cast<double>(ticked.delayed_time_s) ==
            Catch::Approx(static_cast<double>(kDt) * static_cast<double>(total_ticks)));
}

TEST_CASE("EkfCore::tick does not let position drift during pre-fill under a "
          "realistic non-zero starting velocity (CPP-073)",
          "[ekf_core][tick]") {
    // CPP-072's own closed-loop test first surfaced this bug by starting
    // from a realistic 18 m/s cruise velocity rather than every prior
    // test's stationary zero-velocity convention - reproduced here in
    // isolation, directly against tick(), rather than via a full
    // closed-loop scenario.
    constexpr ftype kInitialSpeed = static_cast<ftype>(18.0);

    EkfCore ekf;
    ekf.state.velocity = Vector3F(kInitialSpeed, ftype(0), ftype(0));
    const Vector3F initial_position = ekf.state.position;
    REQUIRE(initial_position.x == ftype(0));
    REQUIRE(initial_position.y == ftype(0));
    REQUIRE(initial_position.z == ftype(0));

    const std::size_t n = EkfCore::kImuBufferCapacity;
    REQUIRE(n == 11);

    // Independent reference EkfCore, same non-zero starting velocity,
    // fed the SAME seed sample directly (bypassing tick()'s fix) - this
    // is what tick()'s pre-fill window would produce WITHOUT this
    // ticket's snapshot/restore, i.e. the bug reproduced directly. Used
    // below to confirm this test is actually exercising the bug's real
    // mechanism, not something already coincidentally zero.
    EkfCore ref_unfixed;
    ref_unfixed.state.velocity = Vector3F(kInitialSpeed, ftype(0), ftype(0));

    for (std::size_t k = 1; k < n; ++k) {
        const int tick_index = static_cast<int>(k);
        ekf.tick(real_gyro(tick_index), real_accel(tick_index), kDt);
        ref_unfixed.update_strapdown_equations_ned(seed_gyro(), seed_accel(), kDt);
        ref_unfixed.covariance_prediction(seed_gyro(), seed_accel(), kDt);

        REQUIRE_FALSE(ekf.imu_buffer.is_filled());

        // The actual fix under test: position must stay EXACTLY at its
        // starting value throughout the whole pre-fill window, tick by
        // tick, not just "eventually" - a non-zero velocity would
        // otherwise advance it by velocity*dt every single call (the bug).
        REQUIRE(static_cast<double>(ekf.state.position.x) == Catch::Approx(static_cast<double>(initial_position.x)));
        REQUIRE(static_cast<double>(ekf.state.position.y) == Catch::Approx(static_cast<double>(initial_position.y)));
        REQUIRE(static_cast<double>(ekf.state.position.z) == Catch::Approx(static_cast<double>(initial_position.z)));

        // Velocity and attitude are unaffected by this fix - continue to
        // behave exactly as the true no-op pre-fill already guarantees
        // (tests 1/3 above): velocity stays at its initial value (the
        // seeded sample's delta-velocity exactly cancels gravity, and
        // there is no horizontal specific force), attitude stays level.
        REQUIRE(static_cast<double>(ekf.state.velocity.x) == Catch::Approx(static_cast<double>(kInitialSpeed)));
        REQUIRE(static_cast<double>(ekf.state.velocity.y) == Catch::Approx(0.0));
        REQUIRE(static_cast<double>(ekf.state.velocity.z) == Catch::Approx(0.0));
    }

    // Sanity: confirm the bug is real and this test would have caught it
    // pre-fix - the UNFIXED reference (same starting velocity, same seed
    // sample, no snapshot/restore) DOES drift in position by
    // approximately initial_velocity * (n-1) * dt, matching CPP-072's own
    // measured ~3.6m-at-18m/s finding scaled to this test's own n/dt.
    const ftype expected_unfixed_drift = kInitialSpeed * static_cast<ftype>(n - 1) * kDt;
    REQUIRE(static_cast<double>(ref_unfixed.state.position.x) ==
            Catch::Approx(static_cast<double>(expected_unfixed_drift)).margin(1e-6));
    REQUIRE(ref_unfixed.state.position.x != ftype(0)); // genuinely drifted, not coincidentally zero

    // Once the buffer fills and real samples start flowing through, the
    // fix's window has closed and normal (correct) integration resumes -
    // exercised past n to confirm the fix doesn't leak into or otherwise
    // disturb post-fill behavior.
    for (int k = static_cast<int>(n); k <= 20; ++k) {
        ekf.tick(real_gyro(k), real_accel(k), kDt);
    }
    REQUIRE(ekf.imu_buffer.is_filled());
    REQUIRE(ekf.ticks_since_seed == 20);
}
