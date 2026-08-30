#pragma once

// Port of AP_Math/control.cpp's SCALAR half - CCP-021, the first ticket of
// the copter-cpp effort's own AC_AttitudeControl phase to touch a NEW,
// foundational, SHARED AP_Math utility rather than AC_AttitudeControl
// itself. copter-rust's own COP-001 investigation confirms directly: "The
// file is identical across vehicles" - Rover, Plane and Copter all build
// the exact same real control.cpp, none of it copter-specific - so this is
// tracked under the multirotor effort only because AC_PosControl (Copter's
// position controller) is its main real upstream user, not because any of
// it is multirotor-specific.
//
// Real upstream source: libraries/AP_Math/control.cpp (Plane-4.7.0, pinned
// worktree upstream/plane-4.7.0 - Copter and Plane 4.7.0 share the exact
// same upstream commit for this file). Ten functions, real line ranges
// re-verified directly via `grep -n` before writing anything here (not
// trusted from the ticket's own summary):
//   - update_vel_accel         (real lines 36-49)
//   - update_pos_vel_accel     (real lines 57-68)
//   - shape_accel              (real lines 117-132)
//   - shape_vel_accel          (real lines 172-212)
//   - shape_pos_vel_accel      (real lines 259-336)
//   - shape_angle_vel_accel    (real lines 423-431)
//   - sqrt_controller          (real lines 544-579)
//   - inv_sqrt_controller      (real lines 599-628)
//   - sqrt_controller_accel    (real lines 642-675)
//   - stopping_distance        (real lines 681-685)
//
// DELIBERATELY EXCLUDED, named explicitly as a separate, deferred future
// ticket, matching copter-rust's own identical scalar/vector split: the
// real "vector half" of this same file - shape_accel_xy (both Vector2f and
// Vector3f forms), shape_vel_accel_xy, shape_pos_vel_accel_xy, the
// Vector2f-argument sqrt_controller overload, limit_accel_xy,
// limit_accel_corner_xy, and kinematic_limit (both the scalar/xyz and
// Vector3f forms). Same ideas applied to a direction rather than a sign.
//
// This unblocks TWO separate, real AC_AttitudeControl functions this port
// has not yet ported, both explicitly deferred by CCP-020 pending this
// exact ticket - NOT started here, this ticket is scoped to the shared
// AP_Math utility layer only:
//   - attitude_command_model (needs shape_angle_vel_accel below)
//   - thrust_heading_rotation_angles (needs inv_sqrt_controller below)
//
// REUSED INVESTIGATION: copter-rust's own COP-001 ticket ported this exact
// real "scalar half" first (ports/ardumaster-rust/crates/ap-math/src/
// control.rs), with exceptional rigor reused here as this ticket's own
// bar: "30,586 values 100.00% bit-exact against upstream - including a
// 1000-step closed loop of shape_pos_vel_accel feeding update_pos_vel_accel
// and a 500-step angular one, where compounding state would expose any
// per-step difference." This port has no equivalent binary-parity harness
// infrastructure, so the bar is met instead with as thorough a Catch2 unit
// suite as this ticket can produce (see tests/control_test.cpp), while
// every formula/sign/branch below was independently re-verified against
// the real C++ source directly - not merely transliterated from the Rust.
//
// THE REAL INTERNAL_ERROR-AND-UNCHANGED-OUTPUT PATTERN, shared across
// shape_accel/shape_vel_accel/shape_pos_vel_accel: each begins with a real
// sanity check on its own input limits (the EXACT condition differs per
// function, verified individually below) that, on failure, calls real
// upstream's own INTERNAL_ERROR(AP_InternalError::error_t::
// invalid_arg_or_result) and returns IMMEDIATELY, leaving the output
// parameter (accel) COMPLETELY UNCHANGED - not zeroed, not clamped, nothing
// happens at all. Reproduced here via this port's own existing, explicit
// fwcpp::InternalError mechanism (CPP-005) - the SAME optional trailing
// `InternalError* err = nullptr, std::uint16_t line = 0` convention already
// established by scalar.hpp's own constrain_value and quaternion.hpp's own
// normalize(): a null err is the same as a build with AP_INTERNALERROR_
// ENABLED off upstream (reporting becomes a no-op), and the function's own
// early-return-without-mutation behavior is unaffected either way.
// update_vel_accel/update_pos_vel_accel have NO such guard - verified
// directly, upstream has no INTERNAL_ERROR call anywhere in either body.
//
// shape_angle_vel_accel has no guard of its OWN - it delegates entirely to
// shape_pos_vel_accel (passing err/line straight through), which supplies
// the only real sanity check in that call chain.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/internal_error.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::math {

// No `sq(x)` helper exists anywhere in this port (verified directly - not
// in scalar.hpp, not anywhere else in ap-math) - upstream's own `sq()` is
// a tiny `x*x` macro from AP_Math.h, so every real `sq(...)` call site
// below is written out as `x * x` directly rather than introducing a new
// helper outside this ticket's own scope.

// Projects velocity forward in time using acceleration, constrained by
// directional limit - upstream `update_vel_accel`, real lines 36-49.
// - If `limit` is non-zero, it defines a direction in which acceleration is
//   constrained.
// - `vel_error`'s SIGN (not magnitude) gives the direction of velocity
//   error.
// - When `limit` is active, velocity is only updated if doing so would not
//   increase the error in the limited direction.
// - If velocity currently opposes the limit direction, the update is
//   clipped so it cannot cross zero (it may unwind, but not overshoot).
// No INTERNAL_ERROR guard - verified directly, upstream has none here.
inline void update_vel_accel(float& vel, float accel, float dt, float limit, float vel_error) {
    float delta_vel = accel * dt;
    // do not add delta_vel if it will increase the velocity error in the
    // direction of limit, unless adding delta_vel will reduce vel towards
    // zero
    if (is_positive(delta_vel * limit) && is_positive(vel_error * limit)) {
        if (is_negative(vel * limit)) {
            delta_vel = constrain_value(delta_vel, -std::fabs(vel), std::fabs(vel));
        } else {
            delta_vel = 0.0f;
        }
    }
    vel += delta_vel;
}

// Projects position and velocity forward in time using acceleration,
// constrained by directional limit - upstream `update_pos_vel_accel`, real
// lines 57-68.
// - `limit` defines the constrained direction of motion.
// - `pos_error`/`vel_error`'s SIGNS (magnitude ignored) give the direction
//   of error in position/velocity respectively.
// - If the position update would increase position error in the
//   constrained direction, it is skipped entirely.
// - The velocity update then proceeds exactly as update_vel_accel().
// No INTERNAL_ERROR guard - verified directly, upstream has none here.
inline void update_pos_vel_accel(postype_t& pos, float& vel, float accel, float dt, float limit,
                                  float pos_error, float vel_error) {
    // move position and velocity forward by dt if it does not increase
    // error when limited.
    float delta_pos = vel * dt + accel * 0.5f * (dt * dt);
    // do not add delta_pos if it will increase the position error in the
    // direction of limit
    if (is_positive(delta_pos * limit) && is_positive(pos_error * limit)) {
        delta_pos = 0.0f;
    }
    pos += delta_pos;

    update_vel_accel(vel, accel, dt, limit, vel_error);
}

// Applies jerk-limited shaping to the acceleration value to gradually
// approach a new target - upstream `shape_accel`, real lines 117-132. The
// foundational function every other shape_* function here eventually calls.
// - Constrains the rate of change of acceleration to within
//   +-jerk_max*dt over this one call.
// - `accel` is modified in-place.
//
// INTERNAL_ERROR guard: `!is_positive(jerk_max)` - if it fires, `accel` is
// left COMPLETELY UNCHANGED (verified directly: upstream's own early
// `return;` happens before anything touches `accel`).
inline void shape_accel(float accel_desired, float& accel, float jerk_max, float dt,
                         InternalError* err = nullptr, std::uint16_t line = 0) {
    // sanity check jerk_max
    if (!is_positive(jerk_max)) {
        if (err != nullptr) {
            err->record(InternalErrorCode::invalid_arg_or_result, line);
        }
        return;
    }

    // jerk limit acceleration change
    if (is_positive(dt)) {
        float accel_delta = accel_desired - accel;
        accel_delta = constrain_value(accel_delta, -jerk_max * dt, jerk_max * dt);
        accel += accel_delta;
    }
}

// Piecewise square-root + linear controller that limits second-order
// response (acceleration) - upstream `sqrt_controller`, real lines
// 544-579. Defined here, ahead of shape_vel_accel/shape_pos_vel_accel
// below (both call it), unlike upstream's own file order (which relies on
// AP_Math.h's forward declarations) - this is a single header with no
// separate declarations, so the real dependency order applies directly.
// - Behaves like a plain P controller near the setpoint.
// - Switches to sqrt(2*second_ord_lim*dx) shaping beyond a threshold to
//   limit acceleration.
// - Three real branches (verified directly):
//    1. `second_ord_lim` non-positive: no second-order limit at all - pure
//       linear (error * p).
//    2. `p` exactly zero (with a positive second_ord_lim): pure
//       sqrt-shaped response, with a real three-way sign check on `error`
//       itself (positive/negative/zero handled separately - the zero case
//       returns EXACTLY 0.0, not a computed near-zero value).
//    3. Both defined: hybrid model. `linear_dist = second_ord_lim / p^2`
//       splits a linear inner region from a sqrt-shaped outer region on
//       BOTH sides (the `linear_dist / 2.0` term inside the sqrt is what
//       makes the two regions join continuously, not merely meet).
// - A final "do not overshoot the error in one timestep" clamp is applied
//   UNIFORMLY across all three branches' results whenever `dt > 0`
//   (verified directly: the clamp sits after the if/else-if/else chain,
//   not inside any one branch).
[[nodiscard]] inline float sqrt_controller(float error, float p, float second_ord_lim, float dt) {
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // No second-order limit: use pure linear controller
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // No P gain, but with acceleration limit - use sqrt-shaped
        // response only
        if (is_positive(error)) {
            correction_rate = safe_sqrt(2.0f * second_ord_lim * error);
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0f;
        }
    } else {
        // Both P and second-order limits defined - use hybrid model
        const float linear_dist = second_ord_lim / (p * p);
        if (error > linear_dist) {
            // Positive error beyond linear region - use sqrt branch
            correction_rate = safe_sqrt(2.0f * second_ord_lim * (error - (linear_dist / 2.0f)));
        } else if (error < -linear_dist) {
            // Negative error beyond linear region - use sqrt branch
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error - (linear_dist / 2.0f)));
        } else {
            // Inside linear region
            correction_rate = error * p;
        }
    }
    if (is_positive(dt)) {
        // Clamp to ensure we do not overshoot the error in the last time
        // step
        return constrain_value(correction_rate, -std::fabs(error) / dt, std::fabs(error) / dt);
    }
    return correction_rate;
}

// Inverts the output of sqrt_controller() to recover the input error that
// would produce a given output - upstream `inv_sqrt_controller`, real
// lines 599-628.
// - THREE real degenerate cases before the main linear-vs-sqrt-region
//   branch, parallel in structure to sqrt_controller()'s own three
//   branches (verified directly):
//    1. D_max positive AND p exactly zero: pure sqrt-inverse,
//       output^2 / (2*D_max).
//    2. D_max non-positive AND p non-zero: pure linear-inverse,
//       output / p.
//    3. D_max non-positive AND p exactly zero: no useful model at all,
//       returns exactly 0.0.
// - Main case (D_max positive, p non-zero): computes the transition
//   threshold linear_velocity = D_max / p; below it, linear inverse
//   (output / p); at or above it, the sqrt-region inverse
//   (linear_dist/2 + output^2/(2*D_max)), signed by output's own sign.
[[nodiscard]] inline float inv_sqrt_controller(float output, float p, float D_max) {
    // Degenerate case: second-order limit (D_max) is positive, but P gain
    // is zero
    if (is_positive(D_max) && is_zero(p)) {
        return (output * output) / (2.0f * D_max);
    }

    // Degenerate case: no D_max, but P gain is non-zero -> use linear model
    if ((is_negative(D_max) || is_zero(D_max)) && !is_zero(p)) {
        return output / p;
    }

    // Degenerate case: both gains are zero - no useful model
    if ((is_negative(D_max) || is_zero(D_max)) && is_zero(p)) {
        return 0.0f;
    }

    // Compute transition threshold between linear and sqrt regions
    const float linear_velocity = D_max / p;

    if (std::fabs(output) < linear_velocity) {
        // Linear region: below transition threshold
        return output / p;
    }

    // Square-root region: above transition threshold
    const float linear_dist = D_max / (p * p);
    const float stopping_dist = (linear_dist * 0.5f) + (output * output) / (2.0f * D_max);
    return is_positive(output) ? stopping_dist : -stopping_dist;
}

// Computes the rate-of-change implied by sqrt_controller() for the
// commanded correction rate - upstream `sqrt_controller_accel`, real lines
// 642-675.
// - Chain-rule derived: uses the CORRECTION-FRAME CLOSING RATE
//   (`rate_state`, e.g. shape_pos_vel_accel's vel_corr) rather than raw
//   velocity, because chasing a receding target shrinks the error more
//   slowly than ground speed alone would suggest.
// - If moving away from the target (rate_cmd and rate_state have opposing
//   or zero-crossing signs), returns exactly 0.0 - there is no braking
//   profile to differentiate.
// - Linear region (no second_ord_lim, or |error| inside linear_dist):
//   rate_cmd_dot = -p * rate_state.
// - Sqrt region: rate_cmd_dot = -(second_ord_lim / |rate_cmd|) *
//   rate_state, guarded against rate_cmd == 0 (returns 0.0 there too).
[[nodiscard]] inline float sqrt_controller_accel(float error, float rate_cmd, float rate_state,
                                                  float p, float second_ord_lim) {
    // If we are moving away from the target return zero.
    if (!is_positive(rate_cmd * rate_state)) {
        return 0.0f;
    }

    // If no second-order limit, controller is linear everywhere
    // (rate_cmd ~ p*error).
    if (!is_positive(second_ord_lim)) {
        return -p * rate_state;
    }

    // If no P gain but second-order limit exists, controller is pure sqrt
    // everywhere.
    if (!is_positive(p)) {
        if (is_zero(rate_cmd)) {
            return 0.0f;
        }
        return -(second_ord_lim / std::fabs(rate_cmd)) * rate_state;
    }

    // Both P and second-order limit defined - match sqrt_controller()'s
    // own region selection.
    const float linear_dist = second_ord_lim / (p * p);

    if (std::fabs(error) <= linear_dist) {
        // Inside linear region.
        return -p * rate_state;
    }

    // Outside linear region (sqrt branch). Guard divide-by-zero on
    // rate_cmd.
    if (is_zero(rate_cmd)) {
        return 0.0f;
    }
    return -(second_ord_lim / std::fabs(rate_cmd)) * rate_state;
}

// Calculates the stopping distance required to reduce a velocity to zero
// using a square-root controller - upstream `stopping_distance`, real
// lines 681-685. Literally inv_sqrt_controller() under another name: the
// distance at which the controller would command exactly this speed is
// the distance it needs to shed it.
[[nodiscard]] inline float stopping_distance(float velocity, float p, float accel_max) {
    return inv_sqrt_controller(velocity, p, accel_max);
}

// Shapes velocity and acceleration using jerk-limited control - upstream
// `shape_vel_accel`, real lines 172-212.
// - Computes correction acceleration needed to reach `vel_desired` from the
//   current `vel` via sqrt_controller(), gained by KPa.
// - REAL, ASYMMETRIC accel-limit selection, keyed on the SIGN of
//   `vel_error` specifically (verified directly, real upstream comment
//   reproduced): "The direction of acceleration limit is the same as the
//   velocity error... because the velocity error is negative when slowing
//   down while closing a positive position error." KPa = jerk_max /
//   accel_max when vel_error is positive, else KPa = jerk_max /
//   (-accel_min).
// - Correction is clamped to [accel_min, accel_max], then the feedforward
//   accel_desired is added.
// - If `limit_total_accel` is true, the TOTAL (post-feedforward)
//   acceleration is optionally re-clamped to the same [accel_min,
//   accel_max] window.
// - The result is jerk-limited via shape_accel() - actual mutation of
//   `accel` happens there, not here.
//
// INTERNAL_ERROR guard: `!is_negative(accel_min) || !is_positive(accel_max)
// || !is_positive(jerk_max)` - if it fires, `accel` is left COMPLETELY
// UNCHANGED (the early `return;` happens before shape_accel() is ever
// called).
inline void shape_vel_accel(float vel_desired, float accel_desired, float vel, float& accel,
                             float accel_min, float accel_max, float jerk_max, float dt,
                             bool limit_total_accel, InternalError* err = nullptr,
                             std::uint16_t line = 0) {
    // sanity check accel_min, accel_max and jerk_max.
    if (!is_negative(accel_min) || !is_positive(accel_max) || !is_positive(jerk_max)) {
        if (err != nullptr) {
            err->record(InternalErrorCode::invalid_arg_or_result, line);
        }
        return;
    }

    // velocity error to be corrected
    const float vel_error = vel_desired - vel;

    // Calculate time constants and limits to ensure stable operation.
    // The direction of acceleration limit is the same as the velocity
    // error, keyed on vel_error's SIGN specifically (not accel_desired,
    // not anything else).
    float KPa;
    if (is_positive(vel_error)) {
        KPa = jerk_max / accel_max;
    } else {
        KPa = jerk_max / (-accel_min);
    }

    // acceleration to correct velocity
    float accel_target = sqrt_controller(vel_error, KPa, jerk_max, dt);

    // constrain correction acceleration from accel_min to accel_max
    accel_target = constrain_value(accel_target, accel_min, accel_max);

    // velocity correction with input velocity
    accel_target += accel_desired;

    // Constrain total acceleration if limiting is enabled
    if (limit_total_accel) {
        accel_target = constrain_value(accel_target, accel_min, accel_max);
    }

    shape_accel(accel_target, accel, jerk_max, dt, err, line);
}

// Shapes position, velocity and acceleration using a jerk-limited
// square-root command model - upstream `shape_pos_vel_accel`, real lines
// 259-336. The most complex function in this set.
// - Position error `pos_error = pos_desired - pos`.
// - REAL, sign-based selection of accel_lim/k_v (mirroring
//   shape_vel_accel's own asymmetric pattern, but keyed on `pos_error`'s
//   sign here): pos_error positive -> accel_lim = -accel_min; otherwise
//   accel_lim = accel_max. k_v = jerk_max / accel_lim in both cases.
// - REAL "correction-frame" velocity bias: vel_corr = vel - vel_desired
//   (verified directly, real upstream comment reproduced: "vel_corr is the
//   correction-frame velocity, so pos_error_dot = vel_desired - vel =
//   -vel_corr").
// - vel_corr_cmd from sqrt_controller(pos_error, k_v, accel_lim, dt), then
//   biased by a sqrt_controller_accel()-derived term:
//   vel_corr_cmd += accel_corr_cmd / k_v.
// - SEPARATE correction-velocity-limit step, each side gated independently
//   on is_negative(vel_min)/is_positive(vel_max) - zero on either side
//   means "no limit" on that side (the same convention already seen in
//   earlier AP_Motors/AC_AttitudeControl tickets) - applied BEFORE forming
//   vel_target.
// - vel_target = vel_desired + vel_corr_cmd, with a SEPARATE, OPTIONAL
//   re-clamp of vel_target itself (same vel_min/vel_max gating) only if
//   `limit_total`.
// - accel_target = (vel_target - vel) * k_v.
// - REAL double-clamp: accel_target constrained to [accel_min, accel_max]
//   BEFORE accel_desired is added, then OPTIONALLY re-clamped to the same
//   window AFTER, only if `limit_total`.
// - Finally delegates jerk-limiting to shape_accel().
//
// INTERNAL_ERROR guard: `is_positive(vel_min) || is_negative(vel_max) ||
// !is_negative(accel_min) || !is_positive(accel_max) ||
// !is_positive(jerk_max)` - if it fires, `accel` is left COMPLETELY
// UNCHANGED.
inline void shape_pos_vel_accel(postype_t pos_desired, float vel_desired, float accel_desired,
                                 postype_t pos, float vel, float& accel, float vel_min,
                                 float vel_max, float accel_min, float accel_max, float jerk_max,
                                 float dt, bool limit_total, InternalError* err = nullptr,
                                 std::uint16_t line = 0) {
    // Sanity check limits and jerk_max.
    if (is_positive(vel_min) || is_negative(vel_max) || !is_negative(accel_min) ||
        !is_positive(accel_max) || !is_positive(jerk_max)) {
        if (err != nullptr) {
            err->record(InternalErrorCode::invalid_arg_or_result, line);
        }
        return;
    }

    // Position error to be corrected.
    const float pos_error = static_cast<float>(pos_desired - pos);

    // Select sqrt_controller parameters based on error sign so the
    // second-order limit (acceleration allowance) matches the direction of
    // motion.
    float accel_lim;
    float k_v;
    if (is_positive(pos_error)) {
        accel_lim = -accel_min; // acceleration limit magnitude (positive)
        k_v = jerk_max / accel_lim; // inner velocity-loop gain (1/s)
    } else {
        accel_lim = accel_max; // acceleration limit magnitude (positive)
        k_v = jerk_max / accel_lim; // inner velocity-loop gain (1/s)
    }

    // Work in the correction frame by removing the feedforward velocity.
    // vel_corr is the correction-frame velocity, so
    // pos_error_dot = vel_desired - vel = -vel_corr.
    const float vel_corr = vel - vel_desired;

    // Velocity correction command derived from position error
    // (second-order limited).
    float vel_corr_cmd = sqrt_controller(pos_error, k_v, accel_lim, dt);

    // Rate-of-change implied by the shaped velocity correction, using
    // correction-frame closing rate.
    const float accel_corr_cmd = sqrt_controller_accel(pos_error, vel_corr_cmd, vel_corr, k_v, accel_lim);

    // Convert the implied rate-of-change term into an equivalent velocity
    // correction bias.
    vel_corr_cmd += accel_corr_cmd / k_v;

    // Limit correction velocity magnitude if velocity limiting is enabled
    // (non-zero limits).
    if (is_negative(vel_min)) {
        vel_corr_cmd = std::max(vel_corr_cmd, vel_min);
    }
    if (is_positive(vel_max)) {
        vel_corr_cmd = std::min(vel_corr_cmd, vel_max);
    }

    // Total velocity target = feedforward + correction.
    float vel_target = vel_desired + vel_corr_cmd;

    // Constrain total velocity if limiting is enabled and velocity limits
    // are enabled (non-zero).
    if (limit_total) {
        if (is_negative(vel_min)) {
            vel_target = std::max(vel_target, vel_min);
        }
        if (is_positive(vel_max)) {
            vel_target = std::min(vel_target, vel_max);
        }
    }

    // Acceleration demand from velocity error.
    float accel_target = (vel_target - vel) * k_v;

    // Bound acceleration command (before external feedforward is added).
    accel_target = constrain_value(accel_target, accel_min, accel_max);

    // Add external acceleration feedforward.
    accel_target += accel_desired;

    // Constrain total acceleration if limiting is enabled.
    if (limit_total) {
        accel_target = constrain_value(accel_target, accel_min, accel_max);
    }

    // Jerk-limit acceleration toward accel_target.
    shape_accel(accel_target, accel, jerk_max, dt, err, line);
}

// Shapes angular position, velocity and acceleration using a jerk-limited
// square-root command model - upstream `shape_angle_vel_accel`, real lines
// 423-431. A thin wrapper, no guard of its own.
// - Wraps the desired angle to the nearest equivalent relative to the
//   current angle via `angle + wrap_PI(angle_desired - angle)` (this
//   port's own existing wrap_PI, CCP-019's dependency).
// - Delegates entirely to shape_pos_vel_accel(), passing `-angle_accel_max`
//   / `angle_accel_max` as accel_min/accel_max (verified directly: only
//   the LOW side is negated, the high side is passed through as-is) and
//   `err`/`line` straight through - shape_pos_vel_accel() is where the
//   real sanity check and any INTERNAL_ERROR report actually happen.
inline void shape_angle_vel_accel(float angle_desired, float angle_vel_desired,
                                   float angle_accel_desired, float angle, float angle_vel,
                                   float& angle_accel, float angle_vel_min, float angle_vel_max,
                                   float angle_accel_max, float angle_jerk_max, float dt,
                                   bool limit_total, InternalError* err = nullptr,
                                   std::uint16_t line = 0) {
    // Wrap desired angle to the nearest equivalent setpoint relative to the
    // current angle.
    const float angle_desired_wrapped = angle + wrap_PI(angle_desired - angle);
    shape_pos_vel_accel(static_cast<postype_t>(angle_desired_wrapped), angle_vel_desired,
                         angle_accel_desired, static_cast<postype_t>(angle), angle_vel, angle_accel,
                         angle_vel_min, angle_vel_max, -angle_accel_max, angle_accel_max,
                         angle_jerk_max, dt, limit_total, err, line);
}

} // namespace fwcpp::math
