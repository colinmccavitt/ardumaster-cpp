// Tests for fwcpp::math's control.hpp (CCP-021) - the scalar half of
// AP_Math/control.cpp: update_vel_accel, update_pos_vel_accel, shape_accel,
// shape_vel_accel, shape_pos_vel_accel, shape_angle_vel_accel,
// sqrt_controller, inv_sqrt_controller, sqrt_controller_accel and
// stopping_distance. See control.hpp's own file banner for real upstream
// line ranges and the full design writeup, including why the "vector half"
// (*_xy variants, kinematic_limit, etc.) is a separate, deferred ticket.
//
// Bar: copter-rust's own COP-001 ticket bit-exact-verified this exact
// scalar half against upstream with 30,586 values, including a 1000-step
// closed-loop test of shape_pos_vel_accel feeding update_pos_vel_accel and
// a 500-step angular one. This port has no equivalent binary-parity
// harness, so the same methodology (closed-loop, many-step, convergence
// under compounding state) is reproduced here as a Catch2 suite instead,
// on top of hand-computed single-call cases for every branch and guard.
//
// Every hand-computed expected value below was derived by tracing this
// header's own real upstream formulas by hand, not by running the code and
// copying its output back into the test.

#include <cmath>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/internal_error.hpp>
#include <fwcpp/math/control.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/scalar.hpp>

using namespace fwcpp::math;
using fwcpp::InternalError;
using fwcpp::InternalErrorCode;
using Catch::Approx;

// ---------------------------------------------------------------------
// update_vel_accel / update_pos_vel_accel - no INTERNAL_ERROR guard on
// either (verified directly against the source, see control.hpp's banner).
// ---------------------------------------------------------------------

TEST_CASE("update_vel_accel applies delta_vel unconstrained when limit is zero", "[control][update_vel_accel]") {
    float vel = 1.0f;
    update_vel_accel(vel, /*accel=*/2.0f, /*dt=*/0.5f, /*limit=*/0.0f, /*vel_error=*/1.0f);
    // limit == 0 -> is_positive(delta_vel * 0) is always false, so the
    // guard never engages regardless of vel_error.
    REQUIRE(vel == Approx(2.0f)); // 1.0 + 2.0*0.5
}

TEST_CASE("update_vel_accel refuses a step that would worsen the limited-direction error", "[control][update_vel_accel]") {
    // limit and vel_error both positive, delta_vel positive (would worsen
    // the error), and vel itself is NOT opposing the limit direction
    // (is_negative(vel*limit) is false) -> the whole step is discarded.
    float vel = 0.5f;
    update_vel_accel(vel, /*accel=*/2.0f, /*dt=*/0.5f, /*limit=*/1.0f, /*vel_error=*/1.0f);
    REQUIRE(vel == Approx(0.5f)); // completely unchanged
}

TEST_CASE("update_vel_accel clips an opposing-limit step so it cannot cross zero", "[control][update_vel_accel]") {
    // Same limit/vel_error as above, but vel now opposes the limit
    // direction (vel*limit < 0) -> delta_vel is clipped to
    // [-|vel|, |vel|] rather than discarded outright, so the vehicle may
    // unwind toward zero but not overshoot through it.
    float vel = -0.3f;
    update_vel_accel(vel, /*accel=*/2.0f, /*dt=*/0.5f, /*limit=*/1.0f, /*vel_error=*/1.0f);
    // raw delta_vel = 1.0, clipped to +-0.3 -> +0.3; -0.3 + 0.3 == 0.0 exactly
    REQUIRE(vel == Approx(0.0f).margin(1e-6));
}

TEST_CASE("update_pos_vel_accel skips the position step when it would worsen the limited-direction position error", "[control][update_pos_vel_accel]") {
    postype_t pos = 0.0;
    float vel = 1.0f;
    // delta_pos = vel*dt + accel*0.5*dt^2 = 1.0, limit and pos_error both
    // positive -> is_positive(delta_pos*limit) && is_positive(pos_error*limit)
    // is true, so delta_pos is zeroed even though vel is nonzero.
    update_pos_vel_accel(pos, vel, /*accel=*/0.0f, /*dt=*/1.0f, /*limit=*/1.0f,
                          /*pos_error=*/1.0f, /*vel_error=*/0.0f);
    REQUIRE(pos == Approx(0.0));
    REQUIRE(vel == Approx(1.0f)); // vel_error*limit == 0, update_vel_accel is a plain pass-through
}

// ---------------------------------------------------------------------
// sqrt_controller - three real branches plus the universal final-timestep
// overshoot clamp.
// ---------------------------------------------------------------------

TEST_CASE("sqrt_controller: no second-order limit is pure linear", "[control][sqrt_controller]") {
    // second_ord_lim <= 0 -> correction_rate = error * p unconditionally.
    REQUIRE(sqrt_controller(3.0f, 2.0f, /*second_ord_lim=*/0.0f, /*dt=*/0.0f) == Approx(6.0f));
    REQUIRE(sqrt_controller(3.0f, 2.0f, /*second_ord_lim=*/-5.0f, /*dt=*/0.0f) == Approx(6.0f));
}

TEST_CASE("sqrt_controller: zero P-gain is pure sqrt-shaped, with a three-way sign check on error", "[control][sqrt_controller]") {
    // p == 0, second_ord_lim == 8: sqrt(2*8*|error|), signed by error, and
    // exactly 0.0 (not a computed near-zero) when error is zero.
    REQUIRE(sqrt_controller(1.0f, 0.0f, 8.0f, 0.0f) == Approx(4.0f)); // sqrt(16)
    REQUIRE(sqrt_controller(-1.0f, 0.0f, 8.0f, 0.0f) == Approx(-4.0f));
    REQUIRE(sqrt_controller(0.0f, 0.0f, 8.0f, 0.0f) == 0.0f);
}

TEST_CASE("sqrt_controller: hybrid model splits linear-inner from sqrt-outer on both sides", "[control][sqrt_controller]") {
    // p = 2, second_ord_lim = 8 -> linear_dist = second_ord_lim/p^2 = 2.0.
    // Inside +-2.0: linear (error*p). Outside: sqrt(2*lim*(|error|-linear_dist/2)).
    REQUIRE(sqrt_controller(1.0f, 2.0f, 8.0f, 0.0f) == Approx(2.0f)); // inside linear region
    REQUIRE(sqrt_controller(3.0f, 2.0f, 8.0f, 0.0f) == Approx(std::sqrt(32.0f))); // sqrt(2*8*(3-1))
    REQUIRE(sqrt_controller(-3.0f, 2.0f, 8.0f, 0.0f) == Approx(-std::sqrt(32.0f)));
}

TEST_CASE("sqrt_controller: the final overshoot clamp applies uniformly across all three branches whenever dt > 0", "[control][sqrt_controller]") {
    // Linear branch, deliberately huge p so the raw correction_rate (500)
    // vastly exceeds |error|/dt (10) -> clamped to +-10.
    REQUIRE(sqrt_controller(10.0f, 50.0f, 0.0f, 1.0f) == Approx(10.0f));
    REQUIRE(sqrt_controller(-10.0f, 50.0f, 0.0f, 1.0f) == Approx(-10.0f));

    // Pure-sqrt branch (p == 0), same idea: raw sqrt(200000) ~= 447 vastly
    // exceeds |error|/dt = 1/0.1 = 10 -> clamped to +-10.
    REQUIRE(sqrt_controller(1.0f, 0.0f, 100000.0f, 0.1f) == Approx(10.0f));
    REQUIRE(sqrt_controller(-1.0f, 0.0f, 100000.0f, 0.1f) == Approx(-10.0f));
}

// ---------------------------------------------------------------------
// inv_sqrt_controller - three degenerate cases, the main linear/sqrt
// branch, and an approximate round trip against sqrt_controller.
// ---------------------------------------------------------------------

TEST_CASE("inv_sqrt_controller: degenerate cases", "[control][inv_sqrt_controller]") {
    // D_max > 0, p == 0: pure sqrt-inverse, output^2 / (2*D_max).
    REQUIRE(inv_sqrt_controller(4.0f, 0.0f, 8.0f) == Approx(1.0f)); // 16/16

    // D_max <= 0, p != 0: pure linear-inverse, output/p.
    REQUIRE(inv_sqrt_controller(6.0f, 2.0f, 0.0f) == Approx(3.0f));
    REQUIRE(inv_sqrt_controller(6.0f, 2.0f, -1.0f) == Approx(3.0f));

    // D_max <= 0, p == 0: no useful model, exactly 0.0.
    REQUIRE(inv_sqrt_controller(5.0f, 0.0f, 0.0f) == 0.0f);
    REQUIRE(inv_sqrt_controller(5.0f, 0.0f, -1.0f) == 0.0f);
}

TEST_CASE("inv_sqrt_controller: main linear-vs-sqrt-region branch", "[control][inv_sqrt_controller]") {
    // D_max = 8, p = 2 -> linear_velocity = D_max/p = 4.0.
    REQUIRE(inv_sqrt_controller(2.0f, 2.0f, 8.0f) == Approx(1.0f)); // linear: output/p
    // sqrt region: linear_dist = D_max/p^2 = 2.0, stopping_dist =
    // 1.0 + output^2/(2*D_max) = 1.0 + 36/16 = 3.25.
    REQUIRE(inv_sqrt_controller(6.0f, 2.0f, 8.0f) == Approx(3.25f));
    REQUIRE(inv_sqrt_controller(-6.0f, 2.0f, 8.0f) == Approx(-3.25f));
}

TEST_CASE("inv_sqrt_controller approximately round-trips sqrt_controller (dt == 0, no overshoot clamp)", "[control][inv_sqrt_controller][round_trip]") {
    // dt == 0 makes is_positive(dt) false, so sqrt_controller's own final
    // clamp is skipped entirely - the cleanest way to get a value the
    // inverse can recover exactly for a well-behaved (error, p, lim).
    const float p = 2.0f;
    const float lim = 8.0f;

    SECTION("linear region") {
        const float error = 1.5f;
        const float output = sqrt_controller(error, p, lim, 0.0f);
        REQUIRE(inv_sqrt_controller(output, p, lim) == Approx(error));
    }
    SECTION("sqrt region, negative error") {
        const float error = -5.0f;
        const float output = sqrt_controller(error, p, lim, 0.0f);
        REQUIRE(inv_sqrt_controller(output, p, lim) == Approx(error));
    }
}

// ---------------------------------------------------------------------
// shape_accel - jerk limiting in both directions, and the
// INTERNAL_ERROR-and-unchanged-output guard.
// ---------------------------------------------------------------------

TEST_CASE("shape_accel jerk-limits toward accel_desired in both directions", "[control][shape_accel]") {
    SECTION("positive delta clamped to +jerk_max*dt") {
        float accel = 0.0f;
        shape_accel(/*accel_desired=*/10.0f, accel, /*jerk_max=*/2.0f, /*dt=*/1.0f);
        REQUIRE(accel == Approx(2.0f));
    }
    SECTION("negative delta clamped to -jerk_max*dt") {
        float accel = 0.0f;
        shape_accel(/*accel_desired=*/-10.0f, accel, /*jerk_max=*/2.0f, /*dt=*/1.0f);
        REQUIRE(accel == Approx(-2.0f));
    }
    SECTION("delta within the jerk budget passes through unclamped") {
        float accel = 0.0f;
        shape_accel(/*accel_desired=*/1.0f, accel, /*jerk_max=*/5.0f, /*dt=*/1.0f);
        REQUIRE(accel == Approx(1.0f));
    }
}

TEST_CASE("shape_accel: invalid jerk_max reports INTERNAL_ERROR and leaves accel completely unchanged", "[control][shape_accel][internal_error]") {
    InternalError err;
    float accel = 42.0f; // deliberately not zero, to prove no clamp/zero happens either
    shape_accel(/*accel_desired=*/10.0f, accel, /*jerk_max=*/0.0f, /*dt=*/1.0f, &err, /*line=*/123);
    REQUIRE(accel == 42.0f); // bitwise unchanged, not zeroed, not clamped
    REQUIRE(err.has_error(InternalErrorCode::invalid_arg_or_result));
    REQUIRE(err.count() == 1);
    REQUIRE(err.last_error_line() == 123);

    // Negative jerk_max triggers the same guard.
    float accel2 = -7.5f;
    InternalError err2;
    shape_accel(/*accel_desired=*/10.0f, accel2, /*jerk_max=*/-1.0f, /*dt=*/1.0f, &err2);
    REQUIRE(accel2 == -7.5f);
    REQUIRE(err2.has_error(InternalErrorCode::invalid_arg_or_result));
}

// ---------------------------------------------------------------------
// shape_vel_accel - the asymmetric KPa selection, tested with both signs
// of vel_error, and the INTERNAL_ERROR guard.
// ---------------------------------------------------------------------

TEST_CASE("shape_vel_accel selects KPa by the sign of vel_error, using accel_max vs -accel_min", "[control][shape_vel_accel]") {
    // accel_max (4.0) and -accel_min (2.0) are deliberately asymmetric, and
    // every number below was chosen so nothing saturates against
    // accel_min/accel_max or the jerk budget - the final `accel` therefore
    // reflects exactly which KPa (and so which limit) sqrt_controller used.
    // See control_test.cpp's own design notes: with the limits swapped by
    // mistake, both cases below land on a visibly different accel.
    const float accel_min = -2.0f;
    const float accel_max = 4.0f;
    const float jerk_max = 1.0f;
    const float dt = 1.0f;

    SECTION("vel_error positive -> KPa = jerk_max / accel_max") {
        float accel = 0.0f;
        // vel_desired - vel = 2.0 - 0.0 = +2.0
        shape_vel_accel(/*vel_desired=*/2.0f, /*accel_desired=*/0.0f, /*vel=*/0.0f, accel,
                         accel_min, accel_max, jerk_max, dt, /*limit_total_accel=*/false);
        REQUIRE(accel == Approx(0.5f));
    }
    SECTION("vel_error negative -> KPa = jerk_max / (-accel_min)") {
        float accel = 0.0f;
        // vel_desired - vel = -2.0 - 0.0 = -2.0
        shape_vel_accel(/*vel_desired=*/-2.0f, /*accel_desired=*/0.0f, /*vel=*/0.0f, accel,
                         accel_min, accel_max, jerk_max, dt, /*limit_total_accel=*/false);
        REQUIRE(accel == Approx(-1.0f));
    }
}

TEST_CASE("shape_vel_accel: malformed accel limits report INTERNAL_ERROR and leave accel completely unchanged", "[control][shape_vel_accel][internal_error]") {
    InternalError err;
    float accel = 123.456f;
    // accel_min must be negative; 1.0 is not.
    shape_vel_accel(/*vel_desired=*/5.0f, /*accel_desired=*/0.0f, /*vel=*/0.0f, accel,
                     /*accel_min=*/1.0f, /*accel_max=*/4.0f, /*jerk_max=*/1.0f, /*dt=*/1.0f,
                     /*limit_total_accel=*/false, &err, 55);
    REQUIRE(accel == 123.456f);
    REQUIRE(err.has_error(InternalErrorCode::invalid_arg_or_result));
    REQUIRE(err.last_error_line() == 55);
}

// ---------------------------------------------------------------------
// shape_pos_vel_accel - correction-frame bias, the two-stage optional
// limiting, and a multi-step closed-loop convergence test.
// ---------------------------------------------------------------------

TEST_CASE("shape_pos_vel_accel's correction-frame bias uses (vel - vel_desired), not raw vel", "[control][shape_pos_vel_accel]") {
    // Hand-traced end to end (see this file's own design notes): with
    // pos_desired=5, pos=0, vel_desired=1, vel=3, accel_min=-100,
    // accel_max=100, jerk_max=100 (all deliberately large enough that
    // nothing here saturates against an outer limit or the jerk budget),
    // and vel_min=vel_max=0 (no velocity limiting), the correct
    // correction-frame formula (rate_state = vel - vel_desired = 2.0 fed
    // into sqrt_controller_accel) yields a final accel of exactly 1.0.
    // Using raw vel (3.0) instead of vel_corr (2.0) there - the bug this
    // test exists to catch - yields exactly 0.0 instead: a different,
    // clearly distinguishable answer.
    float accel = 0.0f;
    shape_pos_vel_accel(/*pos_desired=*/postype_t(5.0), /*vel_desired=*/1.0f, /*accel_desired=*/0.0f,
                         /*pos=*/postype_t(0.0), /*vel=*/3.0f, accel,
                         /*vel_min=*/0.0f, /*vel_max=*/0.0f,
                         /*accel_min=*/-100.0f, /*accel_max=*/100.0f,
                         /*jerk_max=*/100.0f, /*dt=*/1.0f, /*limit_total=*/false);
    REQUIRE(accel == Approx(1.0f));
}

TEST_CASE("shape_pos_vel_accel's limit_total optionally re-clamps vel_target/accel_target after the correction-velocity clamp", "[control][shape_pos_vel_accel]") {
    // Hand-traced end to end: pos_desired=5, pos=0 (pos_error=5, so
    // accel_lim=-accel_min=100, k_v=1.0). vel_desired=2, vel=3
    // (vel_corr=1.0). The correction-velocity clamp (vel_min=-1,
    // vel_max=1) always fires and pulls vel_corr_cmd down to 1.0
    // regardless of limit_total - but vel_target = vel_desired +
    // vel_corr_cmd = 3.0 still exceeds vel_max on its own, which is
    // exactly why the SEPARATE, optional limit_total re-clamp exists.
    const postype_t pos_desired(5.0);
    const postype_t pos(0.0);
    const float vel_desired = 2.0f;
    const float vel = 3.0f;
    const float accel_min = -100.0f;
    const float accel_max = 100.0f;
    const float vel_min = -1.0f;
    const float vel_max = 1.0f;
    const float jerk_max = 100.0f;
    const float dt = 1.0f;

    SECTION("limit_total == false: vel_target/accel_target pass through unclamped") {
        float accel = 0.0f;
        shape_pos_vel_accel(pos_desired, vel_desired, /*accel_desired=*/0.0f, pos, vel, accel,
                             vel_min, vel_max, accel_min, accel_max, jerk_max, dt,
                             /*limit_total=*/false);
        REQUIRE(accel == Approx(0.0f).margin(1e-5));
    }
    SECTION("limit_total == true: vel_target is re-clamped to vel_max, changing the result") {
        float accel = 0.0f;
        shape_pos_vel_accel(pos_desired, vel_desired, /*accel_desired=*/0.0f, pos, vel, accel,
                             vel_min, vel_max, accel_min, accel_max, jerk_max, dt,
                             /*limit_total=*/true);
        REQUIRE(accel == Approx(-2.0f));
    }
}

TEST_CASE("shape_pos_vel_accel: malformed limits report INTERNAL_ERROR and leave accel completely unchanged", "[control][shape_pos_vel_accel][internal_error]") {
    InternalError err;
    float accel = -99.0f;
    // vel_min must not be positive; 3.0 is.
    shape_pos_vel_accel(postype_t(5.0), 0.0f, 0.0f, postype_t(0.0), 0.0f, accel,
                         /*vel_min=*/3.0f, /*vel_max=*/5.0f, /*accel_min=*/-1.0f, /*accel_max=*/1.0f,
                         /*jerk_max=*/1.0f, /*dt=*/1.0f, /*limit_total=*/false, &err, 7);
    REQUIRE(accel == -99.0f);
    REQUIRE(err.has_error(InternalErrorCode::invalid_arg_or_result));
    REQUIRE(err.last_error_line() == 7);
}

TEST_CASE("shape_pos_vel_accel + update_pos_vel_accel converge to a fixed target over 1000 steps without diverging or oscillating", "[control][shape_pos_vel_accel][closed_loop]") {
    // Matches copter-rust's own COP-001 methodology directly: drive the
    // shaper and the integrator together over many steps so any per-step
    // formula error has room to compound into a visible divergence.
    postype_t pos = 0.0;
    float vel = 0.0f;
    float accel = 0.0f;
    const postype_t target = 100.0;
    const float accel_min = -5.0f;
    const float accel_max = 5.0f;
    const float vel_min = -20.0f;
    const float vel_max = 20.0f;
    const float jerk_max = 10.0f;
    const float dt = 0.02f; // 50 Hz, 1000 steps == 20 simulated seconds

    double max_pos = 0.0;
    bool got_moving = false;

    for (int i = 0; i < 1000; ++i) {
        shape_pos_vel_accel(target, /*vel_desired=*/0.0f, /*accel_desired=*/0.0f, pos, vel, accel,
                             vel_min, vel_max, accel_min, accel_max, jerk_max, dt,
                             /*limit_total=*/true);
        // limit == 0 disables update_pos_vel_accel's own directional-limit
        // logic entirely, so pos_error/vel_error are irrelevant here.
        update_pos_vel_accel(pos, vel, accel, dt, /*limit=*/0.0f, /*pos_error=*/0.0f, /*vel_error=*/0.0f);

        max_pos = std::max(max_pos, static_cast<double>(pos));
        if (pos > 50.0) {
            got_moving = true;
        }
    }

    REQUIRE(got_moving); // genuine progress, not a no-op
    REQUIRE(max_pos < 105.0); // bounded overshoot - a diverging/oscillating loop would blow well past this
    REQUIRE(static_cast<double>(pos) == Approx(100.0).margin(1.0));
    REQUIRE(vel == Approx(0.0f).margin(0.5f));
    REQUIRE(accel == Approx(0.0f).margin(0.5f));
}

// ---------------------------------------------------------------------
// shape_angle_vel_accel - wraps toward the SHORT way around, and a
// multi-step angular closed-loop convergence test.
// ---------------------------------------------------------------------

TEST_CASE("shape_angle_vel_accel wraps the target to the short way around the boundary", "[control][shape_angle_vel_accel]") {
    // angle = 3.0 rad (~171.9 deg), angle_desired = -3.0 rad (~-171.9 deg).
    // The raw difference is -6.0 rad; wrap_PI(-6.0) = -6.0 + 2*pi ~=
    // 0.283185, so the wrapped target is angle + 0.283185 ~= 3.283185 -
    // just PAST +pi, continuing in the SAME (increasing) direction rather
    // than reversing all the way back through zero. A naive
    // (non-wrapping) implementation would instead see raw_target(-3.0) <
    // angle(3.0) and shape toward a NEGATIVE (decreasing) angle_accel -
    // the long way around - so the sign of the initial angle_accel
    // distinguishes the two.
    float angle = 3.0f;
    float angle_vel = 0.0f;
    float angle_accel = 0.0f;
    shape_angle_vel_accel(/*angle_desired=*/-3.0f, /*angle_vel_desired=*/0.0f,
                           /*angle_accel_desired=*/0.0f, angle, angle_vel, angle_accel,
                           /*angle_vel_min=*/-5.0f, /*angle_vel_max=*/5.0f,
                           /*angle_accel_max=*/10.0f, /*angle_jerk_max=*/20.0f, /*dt=*/0.01f,
                           /*limit_total=*/true);
    REQUIRE(angle_accel > 0.0f); // shapes toward increasing angle: the short way
}

TEST_CASE("shape_angle_vel_accel + update_pos_vel_accel converge across the wrap boundary over 500 steps", "[control][shape_angle_vel_accel][closed_loop]") {
    // Matches copter-rust's own COP-001 "500-step angular" closed loop.
    // angle starts at 3.0 rad and the external target is -3.0 rad; the
    // short way (see the single-step test above) crosses +pi/-pi rather
    // than passing back through zero.
    float angle = 3.0f;
    float angle_vel = 0.0f;
    float angle_accel = 0.0f;
    const float target = -3.0f;
    const float dt = 0.01f; // 500 steps == 5 simulated seconds

    bool crossed_boundary = false;

    for (int i = 0; i < 500; ++i) {
        shape_angle_vel_accel(target, /*angle_vel_desired=*/0.0f, /*angle_accel_desired=*/0.0f,
                               angle, angle_vel, angle_accel,
                               /*angle_vel_min=*/-5.0f, /*angle_vel_max=*/5.0f,
                               /*angle_accel_max=*/10.0f, /*angle_jerk_max=*/20.0f, dt,
                               /*limit_total=*/true);

        postype_t angle_pos = static_cast<postype_t>(angle);
        update_pos_vel_accel(angle_pos, angle_vel, angle_accel, dt, /*limit=*/0.0f,
                              /*pos_error=*/0.0f, /*vel_error=*/0.0f);
        angle = static_cast<float>(angle_pos);

        if (angle > 3.05f) {
            // Confirms it went the short way: briefly past +pi's
            // neighborhood on the way to wrapping, never dipping back down
            // toward zero first.
            crossed_boundary = true;
        }
    }

    REQUIRE(crossed_boundary);
    // Converged to the target modulo 2*pi, whichever sign the float
    // settled on.
    REQUIRE(wrap_PI(angle - target) == Approx(0.0f).margin(0.05f));
    REQUIRE(angle_vel == Approx(0.0f).margin(0.5f));
}
