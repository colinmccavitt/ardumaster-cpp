#pragma once

// Port of AC_AttitudeControl's frame-conversion and rate/angle-limiting
// primitives - CCP-018, the FIRST ticket of the copter-cpp effort's own
// AC_AttitudeControl phase (see /srv/ardumaster/tracker/efforts/
// copter-cpp.md), following directly after AP_Motors's completion
// (CCP-001 through CCP-017). No AC_AttitudeControl-equivalent existed
// anywhere in this port before this ticket - this is the new ap-control
// module's first file.
//
// Real upstream source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
// (Copter-4.7.0, pinned worktree upstream/plane-4.7.0 - Copter and Plane
// 4.7.0 share the exact same upstream commit, already confirmed by
// copter-rust's own charter). Four functions, real line ranges re-verified
// directly against the pinned tree via `grep -n` before writing anything
// here (not trusted from the ticket's own summary):
//   - ang_vel_limit             (real lines 1206-1226)
//   - body_to_euler_limit       (real lines 1229-1248)
//   - euler_derivative_to_body  (real lines 1303-1316)
//   - body_to_euler_derivative  (real lines 1323-1342)
//
// CCP-020 added a fifth function to this same file/module, once CCP-019
// (below) unblocked it - see this file's own "CCP-020 ADDENDUM" comment
// block further down for its full writeup:
//   - thrust_vector_rotation_angles (real lines 1054-1103)
//
// These are the frame-conversion and limiting primitives every other
// AC_AttitudeControl function is built on, with zero dependency on AC_PID/
// AC_P (not yet ported into this port's own modules/) or on quaternion
// axis-angle math (also not yet built - see quaternion.hpp's own file
// banner, "Deliberately NOT in this slice") - a genuinely self-contained
// first slice.
//
// REUSED INVESTIGATION: copter-rust's own COP-007 ticket ported this exact
// same four-function group, against this exact same upstream file, as its
// own first sub-ticket. Its real, merged, already parity-tested files
// (ports/plane-fw-rust/crates/ap-control/src/attitude_kinematics.rs and
// its own tests/attitude_kinematics.rs) were read in full before writing
// this header, and every real measured floating-point value quoted below
// (1.570451 vs true pi/2 = 1.5707963, cos_theta ~= 3.45e-4) is reused
// directly from that investigation, not re-derived - though every formula,
// branch, and constant below was still independently re-verified against
// the real C++ upstream source directly, per this whole project's own
// standing discipline of not trusting a cross-effort or cross-language
// note on faith.
//
// FREE FUNCTIONS, NOT MEMBER METHODS - a deliberate departure from
// upstream's own AC_AttitudeControl::-scoped signatures, and the
// conclusion of this ticket's own explicit investigation:
//   - euler_derivative_to_body, body_to_euler_derivative, and
//     body_to_euler_limit are upstream static/non-const-instance-free
//     already (euler_derivative_to_body/body_to_euler_derivative are
//     plain instance methods that touch no member state at all in their
//     own bodies - re-verified directly by reading both in full - and
//     body_to_euler_limit is a plain, non-static, non-const instance
//     method whose own body also touches no member state - re-verified
//     directly: AC_AttitudeControl.h line 444 declares it with neither a
//     static nor a const qualifier).
//   - ang_vel_limit is upstream's only real `const` INSTANCE method of
//     the four. Read in full (lines 1206-1226): its body references
//     nothing but its own by-reference/by-value parameters
//     (euler_rad, ang_vel_roll_max_rads, ang_vel_pitch_max_rads,
//     ang_vel_yaw_max_rads) - no `_` -prefixed member variable, no other
//     method call, nothing under `this`. There is no real state-dependent
//     reason for it to be a member function; upstream's choice reads as
//     grouping-by-class-namespace convention, not genuine state coupling.
// Ported as four free functions in fwcpp::control, matching this port's
// own ADR-0012 decision 6 (no singletons; explicit parameter passing over
// implicit/hidden state) and this ticket's own COP-007 precedent (all four
// are free functions in ap_control::attitude_kinematics on the Rust side
// too, for the identical reason). No AC_AttitudeControl-equivalent class
// exists in this port yet for these to be methods OF regardless.
//
// bool + OUT-PARAMETER, NOT fwcpp::Result<T,E> - a deliberate departure
// from ADR-0012 decision 3's stated general preference, reconciled against
// this port's own ACTUAL, repeatedly-applied precedent rather than that
// ADR's aspirational text in isolation. Checked directly before writing
// body_to_euler_derivative below: fwcpp::Result<T,E> (modules/fwcpp-result)
// is not referenced anywhere in this port's own already-merged modules
// outside its own unit test - every real bool-plus-out-reference success
// signal already shipped (AhrsBackend::get_relative_position_NE_home/
// _D_home in modules/ap-ahrs/include/fwcpp/ahrs/ahrs_backend.hpp, and
// their ahrs_dcm.hpp overrides; EkfCoreBackend's analogous methods; etc.)
// uses a plain `[[nodiscard]] bool` next to a `T&` out-parameter, matching
// upstream's own shape directly rather than wrapping it in Result. This
// header follows that real, established precedent: body_to_euler_derivative
// below is `[[nodiscard]] bool body_to_euler_derivative(attitude, body,
// Vector3f& euler_out)`, mirroring upstream's own
// `bool body_to_euler_derivative(const Quaternion&, const Vector3f&,
// Vector3f&)` signature exactly rather than switching representations.
//
// THEY ARE NOT ROTATIONS - re-verify this conceptually before reading the
// formulas below. Converting an angular derivative (rate, acceleration -
// "the same transformation applies regardless of derivative order", a
// real upstream comment on both conversion functions) between the Euler
// and body frames is NOT the same operation as rotating a vector. Euler
// rates are measured about three different, non-orthogonal axes at a
// tilted attitude - yaw about the earth-vertical axis, pitch about an
// intermediate axis, roll about the body's own axis - so the transform
// below is not an orthonormal (rotation) matrix and its inverse is not a
// transpose; it has explicit sin/cos ratio (division) terms instead. That
// asymmetry is also the real, disclosed root cause of the gimbal-lock
// finding below: at 90 degrees of pitch the yaw and roll axes coincide,
// the Euler description stops being invertible, and only the body-to-
// euler direction (the one that must invert this relationship) can fail.
//
// THE SINGLE MOST IMPORTANT FINDING IN THIS FILE - reproduced exactly as
// upstream has it, NOT "fixed": body_to_euler_derivative's own real doc
// comment claims it "returns false if the vehicle is pitched 90 degrees up
// or down (gimbal lock)", guarded by the real `if (is_zero(cos_theta))
// return false;` (re-verified directly at real line 1336-1338). This guard
// essentially NEVER fires at a genuinely-constructed 90-degree attitude:
// fwcpp::math::is_zero requires |cos_theta| < FLT_EPSILON (~1.19e-7), but
// building an exact 90-degree-pitch quaternion and reading its pitch back
// via get_euler_pitch() (an asin, numerically flat near its own domain
// endpoints - COP-007's own measured finding, reused directly here) yields
// a measured 1.570451 rather than the true pi/2 = 1.5707963 - a ~3.5e-4
// error. That error makes the resulting cos_theta land around 3.45e-4,
// roughly three thousand times too large to trip the FLT_EPSILON guard.
// The function returns TRUE with a large-but-finite result at a real
// 90-degree attitude in virtually every practical case, not false.
//
// Ported EXACTLY as upstream has it - the guard is NOT widened to actually
// catch this case. Widening it would make this port refuse attitudes real
// upstream still answers for, which is a real behavioral divergence in the
// other, more dangerous direction (a caller relying on the port's own
// refusal where upstream would have kept running would diverge from real
// flight behavior). What the guard's real job is - and does correctly -
// is preventing a literal division by zero when cos_theta is truly,
// exactly (or near-exactly) zero; it was never a usable early-warning
// safety net for "approaching gimbal lock" in general, and this file's own
// tests document that measured fact directly rather than trusting the doc
// comment's own claim at face value. See attitude_kinematics_test.cpp's
// two dedicated tests: one proving the guard does NOT fire at a real,
// exactly-constructed 90-degree attitude (large-but-finite, not false),
// and a companion bisection-based test proving the guard CAN fire for some
// input close enough that cos_theta genuinely lands inside FLT_EPSILON.
//
// TRANSCRIPTION TRAP, explicitly avoided: body_to_euler_derivative's real
// upstream body writes its tangent terms as explicit `sin_theta/cos_theta`
// DIVISION, never as a single `tanf()` call, even though the two are
// mathematically "the same" quantity - `sinf(x)/cosf(x)` and `tanf(x)` are
// NOT guaranteed bit-identical in IEEE float arithmetic (different libm
// code paths, different rounding of the intermediate). This port
// transcribes upstream's real, exact division form below, not a
// "simplified" tanf() call.
//
// ang_vel_limit's REAL CONDITIONAL STRUCTURE - re-verified directly, this
// is NOT a pure ellipse in all cases. If EITHER ang_vel_roll_max_rads or
// ang_vel_pitch_max_rads is exactly zero (upstream's own convention: zero
// means "no limit configured" for that axis, NOT "clamp to zero" - is_zero
// gates this, not a plain `== 0.0f`), each of roll/pitch is clamped
// INDEPENDENTLY via a plain constrain_value, but ONLY for whichever of the
// two is itself non-zero; the zero one is left COMPLETELY UNCLAMPED. ONLY
// when BOTH roll and pitch limits are non-zero does the real elliptical/
// proportional scaling apply: treat (roll/roll_max, pitch/pitch_max) as a
// 2D vector, and if its length exceeds 1.0, scale roll and pitch back
// along that SAME direction, preserving their ratio - a per-axis
// independent clamp here would let a diagonal command through at up to
// root-two (~1.41x, 41% loose) of the intended magnitude, which upstream's
// own real elliptical form exists specifically to avoid. Yaw is ALWAYS
// clamped independently via a plain constrain_value whenever its own max
// is non-zero, regardless of what roll/pitch did - yaw is never coupled
// into the roll/pitch ellipse.
//
// body_to_euler_limit's REAL "WHOLE-VECTOR PASSTHROUGH" - re-verified
// directly, this is a single `||`-combined guard over the WHOLE input, not
// three independent per-component passthroughs. If ANY ONE of the three
// input body_limit components (x/y/z) is non-positive (upstream:
// `!is_positive(...)`), the function returns the ENTIRE INPUT VECTOR
// UNCHANGED - a port that passed through only the specific non-positive
// component while still computing the other two via the real formula
// below would be wrong. Otherwise, sin/cos of roll (phi) and pitch (theta)
// are each independently clamped via constrain_value(fabs(...), 0.1f,
// 1.0f) - this real 0.1 floor is not merely avoiding a literal
// divide-by-zero, it caps the real inflation factor of the division below
// at 10x, rather than letting an attitude near a trig singularity produce
// an effectively unbounded Euler limit. The real output formula (transcribe
// exactly, it is NOT a simple one-to-one component remap):
//   out.x = body_limit.x                                          (unchanged)
//   out.y = MIN(body_limit.y / cos_phi, body_limit.z / sin_phi)
//   out.z = MIN(MIN(body_limit.x / sin_theta,
//                    body_limit.y / (sin_phi * cos_theta)),
//                    body_limit.z / (cos_phi * cos_theta))
//
// DEFERRED, explicitly, NOT started here (as of CCP-018): everything else
// in AC_AttitudeControl.cpp's own ~1550 lines (the attitude error
// decomposition, the command model, the rate target, entry points,
// relax/reset paths, etc.), AC_AttitudeControl_Multi, AC_PosControl,
// AC_WPNav, and the quaternion axis-angle math (from_axis_angle/
// to_axis_angle/from_rotation_vector/rotate_fast) copter-rust's own
// COP-007 needed for its next sub-ticket and which quaternion.hpp's own
// file banner explicitly excludes from this port's current slice. See
// CCP-020's own addendum below for what came next.
//
// ---------------------------------------------------------------------
// CCP-020 ADDENDUM: thrust_vector_rotation_angles, the quaternion error
// decomposition (real lines 1054-1103, re-verified directly via `grep -n`
// against the pinned upstream tree - matches this ticket's own claimed
// range exactly).
//
// Both of this file's own dependencies now exist: CCP-018 built this
// module, and CCP-019 extended quaternion.hpp with from_axis_angle/
// to_axis_angle. This is the FIRST function in this module that needs
// either.
//
// REUSED INVESTIGATION, same discipline as CCP-018's own addendum above:
// copter-rust's own COP-007 ticket ported this exact function first
// (ports/plane-fw-rust/crates/ap-control/src/attitude_error.rs, tests/
// attitude_error.rs), with unusually rich investigation reused directly
// below, independently re-verified against the real C++ source rather
// than trusted on faith.
//
// WHY THE SPLIT EXISTS AT ALL (copter-rust's own words, reproduced
// directly - understanding this is essential to porting the formula
// correctly, not just transcribing it): "The split exists because the
// two are not equally urgent. Thrust pointed the wrong way is a position
// error in the making; heading pointed the wrong way means the aircraft
// is going exactly where it should while facing elsewhere. So the
// controller runs them on different gains and limits and can sacrifice
// heading to keep thrust when it runs out of authority - which a single
// combined error makes impossible to express, because there is nothing
// to give up." And: "the yaw error comes out of a SECOND quaternion
// rather than from an Euler decomposition of the first: after the thrust
// correction, whatever rotation remains is about the body's own thrust
// axis, so it is heading by construction rather than by approximation."
//
// attitude_target and attitude_body are passive rotations from
// target/body frames to the NED frame (upstream's own real framing
// comment, reproduced verbatim). Re-verified this port's own Quaternion
// follows the identical convention before assuming operator ordering
// transfers directly: operator*(Vector3) above is documented "Rotate a
// vector by this quaternion", and CCP-018's own euler_derivative_to_body/
// body_to_euler_derivative already rotate body-fixed quantities into the
// inertial frame this same way - same passive-rotation convention real
// upstream uses, confirmed directly rather than assumed.
//
// THE REAL MIXED-FRAME DOT PRODUCT (thrust_angle_rad) - re-verified this
// is intentional, not a bug to "fix": thrust_vector_up is the BODY-frame
// constant (0,0,-1); att_body_thrust_vec is already the INERTIAL-frame
// view of that same vector (rotated there by attitude_body one line
// above). The dot product mixes the two frames. This is exactly what
// real upstream's own source does, transcribed as-is.
//
// THE REAL DEGENERATE-CASE FALLBACK, re-verified exactly: if the cross
// product's own length is zero OR thrust_error_angle_rad is zero (a
// real `||`, either alone triggers it), thrust_vec_cross resets to the
// real thrust_vector_up CONSTANT ITSELF - not a zero vector, not left as
// whatever the degenerate cross product computed. This is the aligned
// case (parallel thrust vectors: both conditions trigger together,
// error angle zero and cross length zero) and the antiparallel case
// (opposed thrust vectors: cross length is zero but error angle is pi,
// nonzero - proving the length-zero half of the `||` is independently
// necessary, not redundant with the angle-zero half).
//
// THE REAL FRAME-TRANSFORM DIRECTION, re-verified exactly: thrust_vec_
// cross was computed from two INERTIAL-frame vectors, but thrust_vector_
// correction is defined relative to the BODY frame, so it is rotated by
// the INVERSE of attitude_body (upstream's own comment: "First rotate it
// by the inverse of attitude_body to express it back in the body
// frame") - not a forward rotation by attitude_body itself.
//
// THE REAL SECOND-QUATERNION COMPOSITION for heading, re-verified this
// exact three-way operand order: heading_vec_correction_quat =
// thrust_vector_correction.inverse() * attitude_body.inverse() *
// attitude_target - BOTH thrust_vector_correction and attitude_body are
// inverted, attitude_target is not. Only its z axis-angle component is
// taken (upstream's own comment: "x and y should be zero here" - a real,
// testable invariant, pinned directly by this module's own test file
// rather than trusted from the comment alone).
//
// DEFERRED, explicitly, as a separate future ticket: thrust_heading_
// rotation_angles (real lines 1033-1050), the wrapper that adds
// yaw-error limiting on top of this function. It needs real rate-PID
// gain accessors (get_rate_yaw_pid().kP(), _p_angle_yaw.kP()) and an
// inv_sqrt_controller helper, neither of which exists anywhere in this
// port yet - matching copter-rust's own COP-007 identical, still-open
// deferral of the same function for the same reason.
// ---------------------------------------------------------------------
//
// ---------------------------------------------------------------------
// CCP-022 ADDENDUM: attitude_command_model, the jerk-limited angle-error
// shaping (real lines 1108-1130, re-verified directly via `grep -n`
// against the pinned upstream tree - matches this ticket's own claimed
// range exactly). CCP-021 just unblocked this by adding
// fwcpp::math::shape_angle_vel_accel (modules/ap-math/include/fwcpp/
// math/control.hpp) - the one real dependency this function needed that
// this port did not already have.
//
// REUSED INVESTIGATION: copter-rust's own COP-007 ticket already ported
// this exact function (ports/plane-fw-rust/crates/ap-control/src/
// attitude_error.rs - CommandModel, DEFAULT_ACCEL_MAX_DEGSS,
// DEFAULT_INPUT_TC_CYCLES, attitude_command_model) - read in full before
// writing anything here, two of its own findings reused directly below,
// independently re-verified against the real C++ source rather than
// trusted on faith.
//
// TWO REAL FALLBACK DEFAULTS, both exist "to keep the arithmetic finite
// rather than to describe a vehicle" (copter-rust's own exact words,
// reused verbatim) - re-verified directly against the real source:
//   - if accel_max is not positive, it resets (LOCALLY - see below) to
//     radians(1800.0f). Computed via this port's own real radians()
//     function, NOT a hand-typed radian literal: the value feeds a
//     division below, and a hand-converted constant would be one ULP
//     off from radians()'s own output as often as not, propagating
//     error rather than letting it stay put. Pinned by a dedicated test
//     comparing against radians(1800.0f)'s own computed output, not a
//     separately hand-typed approximation.
//   - if input_tc is not positive, it resets (LOCALLY) to dt * 10.0f -
//     upstream's own comment, reproduced verbatim: "no acceleration set
//     so default to achieve maximum acceleration in 10 clock cycles".
// BY VALUE, NOT BY REFERENCE - already confirmed directly by reading
// AC_AttitudeControl.h line 432: accel_max and input_tc are plain float
// parameters, not float&, so upstream's own reassignment of either
// inside the function body is purely local to that call, invisible to
// the caller - it only affects the shape_angle_vel_accel call below.
// This port's own accel_max/input_tc parameters are likewise plain
// float (by value), matching that exactly.
//
// THE JERK LIMIT, re-verified directly at the shape_angle_vel_accel call
// site below: accel_max / input_tc - a SMALLER input_tc produces a
// LARGER (sharper) jerk limit, i.e. a sharper response. Pinned by a
// dedicated direction test (two otherwise-identical calls differing
// only in input_tc, the smaller one producing the larger-magnitude
// target_ang_accel) precisely because a port that accidentally swapped
// accel_max/input_tc in this division would still compile, still run,
// and still produce SOME finite, plausible-looking jerk limit - only a
// test checking the actual DIRECTION of the relationship would catch
// that swap.
//
// THE ARGUMENT MAPPING into shape_angle_vel_accel is NOT a 1:1
// forwarding of this function's own parameters in argument order -
// re-verified directly against both real signatures side by side:
//   angle_desired        = error_angle
//   angle_vel_desired    = desired_ang_vel
//   angle_accel_desired  = 0.0f  (literal)
//   angle                = 0.0f  (literal)
//   angle_vel            = target_ang_vel  (BY VALUE into that
//                          parameter - shape_angle_vel_accel's own
//                          angle_vel is a plain float, not float&, so
//                          this call does NOT write target_ang_vel
//                          back through it; angle_accel below is the
//                          only real reference output of the call)
//   angle_accel (float&) = target_ang_accel  (the sole real output of
//                          this call)
//   angle_vel_min        = -max_ang_vel
//   angle_vel_max        = max_ang_vel
//   angle_accel_max      = accel_max
//   angle_jerk_max       = accel_max / input_tc
//   dt                   = dt
//   limit_total          = true
//
// THE REAL, EASY-TO-MISS FINAL STEP, re-verified directly at real line
// 1129: target_ang_vel += target_ang_accel * dt - an ADDITIONAL,
// explicit Euler integration step AFTER the shape_angle_vel_accel call
// returns, using the JUST-COMPUTED target_ang_accel and the ORIGINAL
// dt. This is genuinely separate from anything shape_angle_vel_accel
// itself does internally (its own angle_vel parameter is by-value
// input only, as noted above - it never writes target_ang_vel), not a
// redundant double-integration. copter-rust's own COP-007 notes exactly
// why this matters: dropping it leaves the rate target one iteration
// behind, which at 400 Hz is small and constant - exactly the kind of
// error that reads as a slightly sluggish airframe rather than as a
// bug. Pinned by a dedicated test constructing a case where skipping
// this step would produce a measurably different target_ang_vel than
// this function actually returns.
//
// DELIBERATELY EXCLUDED, named explicitly as a separate, deferred
// future ticket: command_model_rate_predictor (real lines 1134-1152,
// the function immediately after this one). It needs real AC_P-style
// proportional-gain accessors (_p_angle_roll.kP()/_p_angle_pitch.kP()),
// a real _rate_bf_ff_enabled flag, get_accel_roll_max_radss()/get_
// accel_pitch_max_radss() accessors, and a real _angle_P_scale member -
// none of which this port has wired into AC_AttitudeControl yet.
// NOTE, corrected by CCP-023 below: this sentence originally claimed
// this was "the same real, disclosed gap CCP-020 already found and
// deferred for thrust_heading_rotation_angles above" - CCP-023's own
// addendum below explains why that comparison no longer holds:
// thrust_heading_rotation_angles turned out to need only plain gain
// VALUES (unblocked by explicit float parameters), while command_model_
// rate_predictor genuinely needs real per-axis STATE
// (_rate_bf_ff_enabled, _angle_P_scale) this port has nowhere to source
// from yet - a different, still-real kind of gap.
//
// A REAL, ALREADY-CONFIRMED LATENT HAZARD in that DEFERRED function,
// noted here (without acting on it, since it is out of this ticket's
// own scope) so it is not lost before that future ticket starts:
// command_model_rate_predictor takes its own dt PARAMETER but its own
// real body never reads it - all three internal calls it makes
// (attitude_command_model x2, ang_vel_limit x1) pass the MEMBER _dt_s
// instead, never the dt parameter. This is copter-rust's own documented
// finding (its own D-025): "not an active defect... but a latent
// hazard, since any future caller asking about a different interval
// would be answered about the controller's own step [interval]."
// Re-verified directly this ticket's OWN function has no such mismatch:
// attitude_command_model's own dt parameter IS genuinely used, both in
// the is_positive(dt) guard and passed straight through to
// shape_angle_vel_accel below.
// ---------------------------------------------------------------------
//
// ---------------------------------------------------------------------
// CCP-023 ADDENDUM: thrust_heading_rotation_angles, the yaw-error-limiting
// wrapper around CCP-020's own thrust_vector_rotation_angles (real lines
// 1033-1050, re-verified directly via `grep -n` against the pinned
// upstream tree - it is the real function immediately BEFORE thrust_
// vector_rotation_angles at real line 1056, i.e. upstream declares these
// two in the opposite order from this port's own file).
//
// A REAL, DISCLOSED CORRECTION TO THIS EFFORT'S OWN EARLIER SCOPING
// DECISION: CCP-020's own addendum above (see its "DEFERRED, explicitly"
// paragraph) deferred this exact function, reasoning that it needed real,
// stateful rate-PID/angle-P gain-OWNING objects (get_rate_yaw_pid(),
// _p_angle_yaw) wired into a real AC_AttitudeControl class before it
// could be ported at all. Re-reading the real function this round shows
// that reasoning over-scoped the dependency: the function's own body
// only ever calls `.kP()` on those objects - it needs their current
// scalar GAIN VALUES, not the objects themselves, and no stateful
// integrator/filter machinery of theirs is touched anywhere in this
// function. Per ADR-0012 (explicit context over singletons/hidden
// state) - the same convention already used successfully throughout this
// whole effort (CCP-011's air_density_ratio, CCP-013's spool-machine
// parameters, CCP-016's roll_in/pitch_in/etc.) - each needed gain is
// simply taken as an explicit `float` parameter instead:
// `rate_yaw_kp` (upstream `get_rate_yaw_pid().kP()`), `angle_yaw_kp`
// (upstream `_p_angle_yaw.kP()`), and `accel_yaw_max_radss` (upstream
// `get_accel_yaw_max_radss()`). CCP-021 already finished inv_sqrt_
// controller (modules/ap-math/include/fwcpp/math/control.hpp) as a pure
// function of plain floats, needing no PID object either. This function
// was therefore genuinely fully unblocked already once CCP-021 landed -
// no new "PID wiring" ticket was actually needed first.
//
// THE THREE NAMED CONSTANTS - AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS/
// _MAX_RADSS and AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD (AC_AttitudeControl.h
// real lines 19-20, 30, re-verified directly) - are each real upstream
// `#define`s computed via upstream's own `radians()`, not hand-typed
// degree-to-radian literals. Reproduced here the same way CCP-022 already
// flagged for its own `radians(1800.0f)` fallback: computed via this
// port's own real `fwcpp::math::radians()`, pinned by a dedicated test
// comparing directly against `radians()`'s own output, not a separately
// hand-typed approximation. `radians()` itself is NOT constexpr-callable
// here (declared in scalar.hpp, defined out-of-line in scalar.cpp -
// re-verified directly), so unlike ap-motors's own kMaxNumMotors-style
// pure-literal `inline constexpr` constants, these three are real,
// runtime-initialized `inline const float` constants (computed once, at
// static-initialization time) - the same k-prefixed naming convention,
// just not constexpr, because the value genuinely cannot be produced at
// compile time from a non-constexpr function.
//
// THE REAL TWO-LEVEL NESTED GUARD, re-verified directly - NOT a single
// combined condition, and NOT an early `return` at either level:
//   - OUTER: `if (!is_zero(rate_yaw_kp))` wraps the function's ENTIRE
//     remaining body. If the rate-yaw P-gain is exactly zero,
//     `attitude_target` is left COMPLETELY UNCHANGED - the thrust
//     correction computed just above this guard is discarded, not
//     applied on its own.
//   - INNER, independent second condition: `if (!is_zero(angle_yaw_kp)
//     && fabs(attitude_error_rad.z) > heading_error_max)` - both the
//     angle-yaw gain must be non-zero AND the actual yaw error must
//     already exceed the computed limit before the clamp/recomposition
//     below executes.
//
// THE REAL `1.0f / rate_yaw_kp` RECIPROCAL, re-verified directly against
// inv_sqrt_controller's own real parameter names (CCP-021): `heading_
// error_max = min(inv_sqrt_controller(1.0f / rate_yaw_kp, angle_yaw_kp,
// heading_accel_max), AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD)` - the
// RECIPROCAL of the rate-yaw gain is inv_sqrt_controller's own `output`
// argument, `angle_yaw_kp` is its `p`, `heading_accel_max` is its
// `D_max`. Passing `rate_yaw_kp` itself here instead of its reciprocal
// would still compile and still produce some finite result - only a test
// exercising a value where the reciprocal and the raw gain diverge
// numerically (this file's own test file picks rate_yaw_kp != 1) can
// catch that swap.
//
// THE REAL MUTATION SHAPE: real upstream takes `attitude_target` as a
// non-const `Quaternion&` specifically so this function can overwrite it
// (`attitude_target = attitude_body * thrust_vector_correction *
// heading_vec_correction_quat`) - the exact three-way composition order,
// `attitude_body` leftmost, `heading_vec_correction_quat` rightmost, only
// when the inner guard fires. This port's own signature keeps that same
// non-const `Quaternion&` in/out parameter shape (matching thrust_vector_
// rotation_angles's own out-parameter convention directly above, rather
// than inventing a separate output parameter) - `attitude_target` is
// both read (as an input to the CCP-020 call) and, conditionally,
// reassigned by this same function.
//
// DEFERRED, explicitly, still NOT started here, and NOT retroactively
// unblocked by this ticket's own correction above: command_model_rate_
// predictor (real lines 1134-1152) - see this file's own CCP-022
// addendum's "DELIBERATELY EXCLUDED" paragraph, now corrected in place
// just above it. That function genuinely still needs real per-axis
// STATE this port has nowhere to source from yet (`_rate_bf_ff_enabled`,
// `_angle_P_scale`), not merely a scalar gain value - a materially
// different, still-real gap from this ticket's own now-corrected
// understanding of thrust_heading_rotation_angles's own dependencies.
// ---------------------------------------------------------------------
//
// ---------------------------------------------------------------------
// CCP-024 ADDENDUM: attitude_from_thrust_vector (real lines 947-973) and
// update_ang_vel_target_from_att_error (real lines 1345-1371), both
// re-verified directly via `grep -n` against the pinned upstream tree -
// matches this ticket's own claimed ranges exactly. Two real,
// mutually-independent functions, bundled into one ticket since both are
// small and self-contained now that CCP-019/020/021 exist.
//
// REUSED INVESTIGATION: copter-rust's own COP-007 ticket already ported
// this exact real function pair together (ports/plane-fw-rust/crates/
// ap-control/src/attitude_error.rs - attitude_from_thrust_vector,
// update_ang_vel_target_from_att_error, ACCEL_RP_MIN_DEGSS/MAX_DEGSS) -
// read in full before writing anything here, every finding below
// independently re-verified against the real C++ source rather than
// trusted on faith.
//
// THE REAL COMPOSITION ORDER IS LOAD-BEARING, reused directly from
// copter-rust's own words: "composes as thrust * yaw, and the order is
// load-bearing: reversed, it would yaw in the earth frame before leaning,
// which puts the lean on the wrong axis for any non-zero heading."
// Re-verified this exact real multiplication order directly at real line
// 973: `return thrust_vec_quat*yaw_quat;` - thrust quaternion on the
// LEFT, yaw quaternion on the RIGHT. This module's own test file proves
// this two ways: a round-trip test (below) and a direct composition-order
// comparison against the reversed product.
//
// THE TWO ROTATIONS USE OPPOSITE Z SIGNS, reused directly: "thrust about
// -Z, heading about +Z - because one is the thrust direction and the
// other is earth-frame down." Re-verified directly: step 1 builds
// thrust_vector_up as the same real (0,0,-1) body-frame-thrust-direction
// constant CCP-020's own thrust_vector_rotation_angles already
// establishes (real line 949); the SEPARATE yaw_quat at real line 972 is
// built from a real, different (0,0,1) axis - genuinely opposite signs,
// not a transcription slip.
//
// THE REAL ROUND-TRIP TEST METHODOLOGY, reused directly rather than
// invented independently: "A test ties this to the decomposition rather
// than checking it alone: build an attitude from a 0.3 rad tilt, run it
// back through thrust_vector_rotation_angles against level, and require
// the lean to come out at 0.3. Either direction alone can be
// self-consistently wrong; together they cannot." This module's own test
// file includes exactly this round-trip, through CCP-020's own
// already-merged thrust_vector_rotation_angles, not merely an isolated
// formula check.
//
// update_ang_vel_target_from_att_error's REAL PER-AXIS (NOT PER-VEHICLE)
// STRATEGY CHOICE, reused directly: "chooses between a proportional gain
// and the square-root controller PER AXIS, not per vehicle - the choice
// depends on whether that axis has an acceleration limit, so a vehicle
// can legitimately run sqrt on roll and pitch and proportional on yaw."
// Re-verified directly at real lines 1349-1369: each of the three axes
// gets its own independent `if (_use_sqrt_controller &&
// !is_zero(get_accel_<axis>_max_radss()))` - a single shared
// use_sqrt_controller bool (taken here as one explicit bool parameter)
// combined with a PER-AXIS zero-check on that axis's own acceleration-max
// value, not a single combined per-vehicle branch. This module's own test
// file exercises this directly with a MIXED case in one call (roll/pitch
// sqrt, yaw proportional).
//
// THE REAL ACCELERATION-HALVING FED TO sqrt_controller, reused directly:
// "The acceleration handed to the square-root controller is HALF the
// axis maximum, clamped. The halving leaves headroom for the rate
// controller underneath, which needs authority of its own to track the
// target this produces; giving it the full limit would let the attitude
// loop consume all of it." Re-verified directly: all three axes divide
// their own accel_<axis>_max_radss by exactly 2.0f before clamping (real
// lines 1350, 1357, 1364). This module's own test file confirms this via
// a hand-computed expected sqrt_controller output for at least one axis,
// not merely a plausible-looking result.
//
// THE REAL, AXIS-DIFFERENT CLAMP BOUNDS, reused directly: "The clamp
// bounds differ by axis - roll and pitch get 40 to 720 deg/s^2 against
// yaw's 10 to 120 - because thrust vectoring gives large roll and pitch
// authority while yaw comes only from rotor drag differences."
// Re-verified directly against AC_AttitudeControl.h real lines 17-18:
// AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS = radians(40.0f),
// AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS = radians(720.0f) - both NEW
// this ticket, named below kAccelRpControllerMinRadss/
// kAccelRpControllerMaxRadss, matching CCP-023's own established
// k-prefixed naming/computation convention for the analogous yaw
// constants. Yaw continues to reuse CCP-023's own already-merged
// kAccelYControllerMinRadss/kAccelYControllerMaxRadss directly (defined
// above) - NOT redeclared here, matching real upstream's own reuse of
// AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS/_MAX_RADSS at real line 1368.
// Both new constants are computed via this port's own real
// math::radians(), not hand-typed radian literals - same ULP-precision
// discipline CCP-022/023 already established, and for the identical
// reason: math::radians() is not constexpr-callable (declared in
// scalar.hpp, defined out-of-line in scalar.cpp), so these are real,
// runtime-initialized `inline const float`, not `inline constexpr`.
//
// attitude_from_thrust_vector TAKES thrust_vector BY VALUE, not by
// reference - re-verified directly against the real signature at line
// 947: `Quaternion AC_AttitudeControl::attitude_from_thrust_vector
// (Vector3f thrust_vector, float heading_angle_rad) const` - the
// function's own body reassigns/normalizes a LOCAL copy, invisible to
// the caller. This port's own parameter is likewise a plain math::Vector3f
// by value, matching that exactly (not a `const Vector3f&`, which would
// be a plausible-looking but real signature mismatch since it would make
// the in-place normalize/reset a compile error rather than a silent
// local-only mutation).
//
// update_ang_vel_target_from_att_error's `_angle_P_scale` is taken here as
// a single explicit math::Vector3f parameter, not three separate float
// parameters - the natural, non-arbitrary choice since real upstream
// itself stores it as one Vector3f member (AC_AttitudeControl.h), and
// this same file already takes other naturally-vector-shaped quantities
// (attitude_error_rad, thrust_vector_up) as Vector3f rather than
// decomposed floats elsewhere. angle_kp_roll/angle_kp_pitch/angle_kp_yaw
// remain three separate explicit float parameters (matching
// thrust_heading_rotation_angles's own established precedent just above
// of taking each axis's own real upstream `.kP()` gain as its own
// explicit float, per ADR-0012), since real upstream reads each from a
// genuinely separate _p_angle_roll/_p_angle_pitch/_p_angle_yaw
// gain-owning object with no shared vector-shaped counterpart.
//
// DEFERRED, explicitly, still NOT started here: command_model_rate_
// predictor (real lines 1134-1152 - still needs real per-axis STATE this
// port has nowhere to source from, per this file's own CCP-022/023
// addenda above), the input_* entry points (input_thrust_vector_rate_
// heading, input_thrust_vector_heading, input_quaternion, etc.),
// update_attitude_target/attitude_controller_run_quat, and the
// relax/reset paths (relax, reset_target_and_rate,
// reset_yaw_target_and_rate, inertial_frame_reset) - all separate,
// deliberately deferred future phases, none retroactively unblocked by
// this ticket's own two functions.
// ---------------------------------------------------------------------

#include <algorithm>
#include <cmath>

#include <fwcpp/math/control.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::control {

// euler_derivative_to_body - upstream AC_AttitudeControl::
// euler_derivative_to_body (real lines 1303-1316). Converts an
// Euler-frame derivative (rate, acceleration - order-independent, see
// this file's own banner) to the body frame, for the 321
// (yaw-pitch-roll) Euler sequence. Always succeeds - this direction has
// no singularity; it is only the inverse (body_to_euler_derivative
// below) that cannot be taken at gimbal lock.
[[nodiscard]] inline math::Vector3f euler_derivative_to_body(const math::Quaternion& att,
                                                               const math::Vector3f& euler_derivative_rads) {
    const float theta = att.get_euler_pitch();
    const float phi = att.get_euler_roll();

    const float sin_theta = std::sin(theta);
    const float cos_theta = std::cos(theta);
    const float sin_phi = std::sin(phi);
    const float cos_phi = std::cos(phi);

    return math::Vector3f{
        euler_derivative_rads.x - sin_theta * euler_derivative_rads.z,
        cos_phi * euler_derivative_rads.y + sin_phi * cos_theta * euler_derivative_rads.z,
        -sin_phi * euler_derivative_rads.y + cos_theta * cos_phi * euler_derivative_rads.z,
    };
}

// body_to_euler_derivative - upstream AC_AttitudeControl::
// body_to_euler_derivative (real lines 1323-1342). Converts a body-frame
// derivative to the Euler frame - the real inverse direction, which DOES
// have a singularity at 90 degrees of pitch (gimbal lock). Returns false
// there, matching upstream's own bool-return/out-parameter shape (see
// this file's own banner for why this port keeps that shape here rather
// than switching to fwcpp::Result).
//
// See this file's own banner - THE SINGLE MOST IMPORTANT FINDING - for
// why the real is_zero(cos_theta) guard below essentially never fires at
// a genuinely-constructed 90-degree attitude, and why it is nonetheless
// reproduced EXACTLY as upstream has it, not widened.
[[nodiscard]] inline bool body_to_euler_derivative(const math::Quaternion& att, const math::Vector3f& body_derivative_rads,
                                                    math::Vector3f& euler_derivative_rads) {
    const float theta = att.get_euler_pitch();
    const float phi = att.get_euler_roll();

    const float sin_theta = std::sin(theta);
    const float cos_theta = std::cos(theta);
    const float sin_phi = std::sin(phi);
    const float cos_phi = std::cos(phi);

    // When the vehicle pitches all the way up or all the way down, the
    // euler angles become discontinuous. In this case, we just return
    // false. (Upstream's own comment, reproduced verbatim - see this
    // file's own banner for the measured reality of when this actually
    // triggers.)
    if (math::is_zero(cos_theta)) {
        return false;
    }

    // Written as explicit sin/cos division, NOT tanf() - see this file's
    // own banner's "TRANSCRIPTION TRAP" section; the two are not
    // guaranteed bit-identical in IEEE float arithmetic.
    euler_derivative_rads.x = body_derivative_rads.x + sin_phi * (sin_theta / cos_theta) * body_derivative_rads.y +
                               cos_phi * (sin_theta / cos_theta) * body_derivative_rads.z;
    euler_derivative_rads.y = cos_phi * body_derivative_rads.y - sin_phi * body_derivative_rads.z;
    euler_derivative_rads.z = (sin_phi / cos_theta) * body_derivative_rads.y + (cos_phi / cos_theta) * body_derivative_rads.z;
    return true;
}

// ang_vel_limit - upstream AC_AttitudeControl::ang_vel_limit (real lines
// 1206-1226, upstream's own `const` instance method - see this file's
// own banner for why this port makes it a free function instead).
// Limits an Euler-frame angular-velocity (or acceleration) vector's
// roll/pitch/yaw components against real per-axis maxima.
//
// See this file's own banner's "REAL CONDITIONAL STRUCTURE" section: this
// is NOT a pure ellipse in all cases - the elliptical/proportional
// coupling between roll and pitch applies ONLY when both of their own
// maxima are non-zero; a zero maximum on either means "unlimited on that
// axis", not "clamp to zero", and falls back to an independent per-axis
// clamp of whichever of the pair is itself non-zero. Yaw is always
// clamped independently, never coupled into the roll/pitch ellipse.
inline void ang_vel_limit(math::Vector3f& euler_rad, float ang_vel_roll_max_rads, float ang_vel_pitch_max_rads,
                           float ang_vel_yaw_max_rads) {
    if (math::is_zero(ang_vel_roll_max_rads) || math::is_zero(ang_vel_pitch_max_rads)) {
        if (!math::is_zero(ang_vel_roll_max_rads)) {
            euler_rad.x = math::constrain_value(euler_rad.x, -ang_vel_roll_max_rads, ang_vel_roll_max_rads);
        }
        if (!math::is_zero(ang_vel_pitch_max_rads)) {
            euler_rad.y = math::constrain_value(euler_rad.y, -ang_vel_pitch_max_rads, ang_vel_pitch_max_rads);
        }
    } else {
        const math::Vector2<float> thrust_vector_ang_vel(euler_rad.x / ang_vel_roll_max_rads,
                                                           euler_rad.y / ang_vel_pitch_max_rads);
        const float thrust_vector_length = thrust_vector_ang_vel.length();
        if (thrust_vector_length > 1.0f) {
            euler_rad.x = thrust_vector_ang_vel.x * ang_vel_roll_max_rads / thrust_vector_length;
            euler_rad.y = thrust_vector_ang_vel.y * ang_vel_pitch_max_rads / thrust_vector_length;
        }
    }
    if (!math::is_zero(ang_vel_yaw_max_rads)) {
        euler_rad.z = math::constrain_value(euler_rad.z, -ang_vel_yaw_max_rads, ang_vel_yaw_max_rads);
    }
}

// body_to_euler_limit - upstream AC_AttitudeControl::body_to_euler_limit
// (real lines 1229-1248 - a plain, non-static instance method upstream,
// re-verified directly against AC_AttitudeControl.h line 444; ported here
// as a free function regardless, since its own body touches no member
// state). Translates
// real body-frame rate/acceleration limits into the Euler axis.
//
// See this file's own banner's "REAL WHOLE-VECTOR PASSTHROUGH" section:
// if ANY ONE of the three input components is non-positive, the ENTIRE
// input vector is returned unchanged - a single combined guard, not
// three independent per-component passthroughs. Otherwise each trig term
// is magnitude-clamped to [0.1, 1.0] before dividing by it, capping the
// real inflation factor at 10x near a singularity rather than letting it
// run away.
[[nodiscard]] inline math::Vector3f body_to_euler_limit(const math::Quaternion& att, const math::Vector3f& body_limit) {
    if (!math::is_positive(body_limit.x) || !math::is_positive(body_limit.y) || !math::is_positive(body_limit.z)) {
        return math::Vector3f{body_limit};
    }

    const float phi = att.get_euler_roll();
    const float theta = att.get_euler_pitch();

    const float sin_phi = math::constrain_value(std::fabs(std::sin(phi)), 0.1f, 1.0f);
    const float cos_phi = math::constrain_value(std::fabs(std::cos(phi)), 0.1f, 1.0f);
    const float sin_theta = math::constrain_value(std::fabs(std::sin(theta)), 0.1f, 1.0f);
    const float cos_theta = math::constrain_value(std::fabs(std::cos(theta)), 0.1f, 1.0f);

    return math::Vector3f{
        body_limit.x,
        std::min(body_limit.y / cos_phi, body_limit.z / sin_phi),
        std::min(std::min(body_limit.x / sin_theta, body_limit.y / (sin_phi * cos_theta)), body_limit.z / (cos_phi * cos_theta)),
    };
}

// thrust_vector_rotation_angles - upstream AC_AttitudeControl::
// thrust_vector_rotation_angles (real lines 1054-1103). CCP-020 - see
// this file's own banner addendum above for the full design writeup
// (the thrust/heading urgency split, the second-quaternion heading
// decomposition, the mixed-frame dot product, and the degenerate-case
// fallback - all re-verified directly against the real upstream source,
// not trusted from any summary). Ported as a free function for the same
// reason as this module's other three: upstream's own method is `const`
// but its body touches no instance state either - just its own
// parameters.
//
// Out-parameters mirror real upstream's own signature shape exactly
// (matching this module's established bool+out-param/void+out-param
// precedent of following upstream's own parameter shape directly rather
// than switching representations) - upstream returns void here (this
// function always succeeds, unlike body_to_euler_derivative above), so
// this does too.
inline void thrust_vector_rotation_angles(const math::Quaternion& attitude_target, const math::Quaternion& attitude_body,
                                           math::Quaternion& thrust_vector_correction, math::Vector3f& attitude_error_rad,
                                           float& thrust_angle_rad, float& thrust_error_angle_rad) {
    // The direction of thrust is [0,0,-1] in any body-fixed frame
    // (upstream's own comment) - NED "up" is negative Z.
    const math::Vector3f thrust_vector_up{0.0f, 0.0f, -1.0f};

    // Rotating [0,0,-1] by each quaternion expresses (gets a view of)
    // that frame's own thrust vector in the inertial frame.
    const math::Vector3f att_target_thrust_vec = attitude_target * thrust_vector_up;
    const math::Vector3f att_body_thrust_vec = attitude_body * thrust_vector_up;

    // The current lean angle, for callers that limit against it - NOT
    // the error (see this file's own banner addendum). A real,
    // intentional mixed-frame dot product: thrust_vector_up is the
    // BODY-frame constant, att_body_thrust_vec is already in the
    // INERTIAL frame.
    thrust_angle_rad = std::acos(math::constrain_value(thrust_vector_up * att_body_thrust_vec, -1.0f, 1.0f));

    // The cross product of the current and target thrust vectors gives
    // the axis to rotate about; the dot product gives how far.
    math::Vector3f thrust_vec_cross = att_body_thrust_vec % att_target_thrust_vec;
    thrust_error_angle_rad = std::acos(math::constrain_value(att_body_thrust_vec * att_target_thrust_vec, -1.0f, 1.0f));

    // Degenerate when the two thrust vectors are parallel or
    // antiparallel: the cross product has no direction to offer.
    // Upstream substitutes the thrust axis ITSELF here (not a zero
    // vector) - see this file's own banner addendum for why the `||` is
    // not redundant (the antiparallel case trips the length check with
    // a nonzero angle).
    const float thrust_vector_length = thrust_vec_cross.length();
    if (math::is_zero(thrust_vector_length) || math::is_zero(thrust_error_angle_rad)) {
        thrust_vec_cross = thrust_vector_up;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    // thrust_vector_correction is defined relative to the body frame,
    // but thrust_vec_cross was computed in the inertial frame (both its
    // inputs were). Rotate it back by the INVERSE of attitude_body first
    // (upstream's own comment: "First rotate it by the inverse of
    // attitude_body to express it back in the body frame") - not a
    // forward rotation.
    thrust_vec_cross = attitude_body.inverse() * thrust_vec_cross;
    thrust_vector_correction.from_axis_angle(thrust_vec_cross, thrust_error_angle_rad);

    // The roll/pitch error comes straight from the thrust correction. z
    // is deliberately NOT set here - it comes from the second
    // quaternion below.
    math::Vector3f rotation_rad;
    thrust_vector_correction.to_axis_angle(rotation_rad);
    attitude_error_rad.x = rotation_rad.x;
    attitude_error_rad.y = rotation_rad.y;

    // Whatever rotation remains after the thrust correction is a
    // rotation about the body's own thrust axis by construction -
    // heading, not an Euler approximation of it. Re-verify this exact
    // three-way composition and operand order: BOTH thrust_vector_
    // correction and attitude_body are inverted, attitude_target is
    // not.
    const math::Quaternion heading_vec_correction_quat =
        thrust_vector_correction.inverse() * attitude_body.inverse() * attitude_target;

    // x and y should be zero here (upstream's own comment - a real,
    // testable invariant; see this module's own test file for the
    // dedicated assertion, not just a trusted comment).
    heading_vec_correction_quat.to_axis_angle(rotation_rad);
    attitude_error_rad.z = rotation_rad.z;
}

// AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS / _MAX_RADSS / AC_ATTITUDE_
// YAW_MAX_ERROR_ANGLE_RAD (AC_AttitudeControl.h real lines 19-20, 30) -
// see this file's own "CCP-023 ADDENDUM" banner above for why these are
// real, runtime-initialized `inline const float` (not `inline
// constexpr`, unlike ap-motors's own kMaxNumMotors-style pure-literal
// constants): math::radians() is not constexpr-callable.
inline const float kAccelYControllerMinRadss = math::radians(10.0f);
inline const float kAccelYControllerMaxRadss = math::radians(120.0f);
inline const float kYawMaxErrorAngleRad = math::radians(45.0f);

// thrust_heading_rotation_angles - upstream AC_AttitudeControl::
// thrust_heading_rotation_angles (real lines 1033-1050). CCP-023 - see
// this file's own "CCP-023 ADDENDUM" banner above for the full design
// writeup (the corrected CCP-020 deferral, the two-level nested guard,
// the 1.0f/rate_yaw_kp reciprocal, and the real mutation shape - all
// re-verified directly against the real upstream source, not trusted
// from any summary).
//
// Wraps thrust_vector_rotation_angles (CCP-020) with real yaw-error
// limiting: the maximum yaw error is limited based on static output
// saturation (upstream's own doc comment).
//
// rate_yaw_kp/angle_yaw_kp/accel_yaw_max_radss are explicit float
// parameters standing in for upstream's own get_rate_yaw_pid().kP()/
// _p_angle_yaw.kP()/get_accel_yaw_max_radss() member accessors - see
// this file's own banner addendum for why plain gain VALUES are all
// this function actually needs, not stateful gain-owning objects.
inline void thrust_heading_rotation_angles(math::Quaternion& attitude_target, const math::Quaternion& attitude_body,
                                            math::Vector3f& attitude_error_rad, float& thrust_angle_rad,
                                            float& thrust_error_angle_rad, float rate_yaw_kp, float angle_yaw_kp,
                                            float accel_yaw_max_radss) {
    math::Quaternion thrust_vector_correction;
    thrust_vector_rotation_angles(attitude_target, attitude_body, thrust_vector_correction, attitude_error_rad,
                                   thrust_angle_rad, thrust_error_angle_rad);

    // Todo: Limit roll an pitch error based on output saturation and maximum error.

    // Limit Yaw Error based to the maximum that would saturate the output when yaw rate is zero.
    math::Quaternion heading_vec_correction_quat;

    const float heading_accel_max =
        math::constrain_value(accel_yaw_max_radss / 2.0f, kAccelYControllerMinRadss, kAccelYControllerMaxRadss);
    if (!math::is_zero(rate_yaw_kp)) {
        // Real, easy-to-miss reciprocal: 1.0f / rate_yaw_kp is inv_sqrt_
        // controller's own `output` argument, NOT rate_yaw_kp itself -
        // see this file's own banner addendum.
        const float heading_error_max = std::min(
            math::inv_sqrt_controller(1.0f / rate_yaw_kp, angle_yaw_kp, heading_accel_max), kYawMaxErrorAngleRad);
        if (!math::is_zero(angle_yaw_kp) && std::fabs(attitude_error_rad.z) > heading_error_max) {
            attitude_error_rad.z =
                math::constrain_value(math::wrap_PI(attitude_error_rad.z), -heading_error_max, heading_error_max);
            heading_vec_correction_quat.from_axis_angle(math::Vector3f{0.0f, 0.0f, attitude_error_rad.z});
            attitude_target = attitude_body * thrust_vector_correction * heading_vec_correction_quat;
        }
    }
}

// attitude_command_model - upstream AC_AttitudeControl::
// attitude_command_model (real lines 1108-1130). CCP-022 - see this
// file's own "CCP-022 ADDENDUM" banner above for the full design
// writeup (both fallback defaults and their rationale, the jerk-limit
// direction test, the exact argument mapping into
// shape_angle_vel_accel, and the real separate final integration step -
// all re-verified directly against the real upstream source, not
// trusted from any summary).
//
// Turns an angle error into a rate and acceleration target, applying
// acceleration/deceleration limits and jerk limiting (via input_tc) on
// top of CCP-021's shape_angle_vel_accel. Ported as a free function for
// the same reason as this module's other functions: upstream's own
// method is `const` but its body touches no instance state, and no
// AC_AttitudeControl-equivalent class exists in this port for it to be
// a method of regardless.
//
// accel_max and input_tc are plain float (BY VALUE, not float&),
// matching upstream's own AC_AttitudeControl.h line 432 declaration
// exactly - see this file's own banner addendum for why that matters
// (their fallback reassignment below is purely local to this call).
inline void attitude_command_model(float error_angle, float desired_ang_vel, float& target_ang_vel,
                                    float& target_ang_accel, float max_ang_vel, float accel_max, float input_tc,
                                    float dt) {
    if (!math::is_positive(dt)) {
        return;
    }

    // protect against divide by zero: no acceleration set, so default
    // to 1800 degrees/s^2 - computed via radians(), not a hand-typed
    // radian literal (see this file's own banner addendum's "TWO REAL
    // FALLBACK DEFAULTS" section for why that distinction matters).
    if (!math::is_positive(accel_max)) {
        accel_max = math::radians(1800.0f);
    }

    // no time constant set, so default to achieve maximum acceleration
    // in 10 clock cycles (upstream's own comment, reproduced verbatim).
    if (!math::is_positive(input_tc)) {
        input_tc = dt * 10.0f;
    }

    // See this file's own banner addendum's "THE ARGUMENT MAPPING"
    // section - this is NOT a 1:1 forwarding of this function's own
    // parameters in order.
    math::shape_angle_vel_accel(error_angle, desired_ang_vel, 0.0f, 0.0f, target_ang_vel, target_ang_accel,
                                 -max_ang_vel, max_ang_vel, accel_max, accel_max / input_tc, dt, true);

    // The real, easy-to-miss final step - see this file's own banner
    // addendum's "THE REAL, EASY-TO-MISS FINAL STEP" section. Genuinely
    // separate from shape_angle_vel_accel's own internal behavior, not
    // a redundant double-integration.
    target_ang_vel += target_ang_accel * dt;
}

// kAccelRpControllerMinRadss / kAccelRpControllerMaxRadss -
// AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS / _MAX_RADSS
// (AC_AttitudeControl.h real lines 17-18) - see this file's own "CCP-024
// ADDENDUM" banner above for why these are real, runtime-initialized
// `inline const float` (not `inline constexpr`) and why yaw's own
// analogous kAccelYControllerMinRadss/kAccelYControllerMaxRadss (defined
// above, CCP-023) are reused rather than redeclared here.
inline const float kAccelRpControllerMinRadss = math::radians(40.0f);
inline const float kAccelRpControllerMaxRadss = math::radians(720.0f);

// attitude_from_thrust_vector - upstream AC_AttitudeControl::
// attitude_from_thrust_vector (real lines 947-973). CCP-024 - see this
// file's own "CCP-024 ADDENDUM" banner above for the full design writeup
// (the load-bearing composition order, the opposite Z signs, and the
// by-value parameter).
//
// Builds a target attitude from a desired thrust direction plus a
// heading: the thrust rotation is composed first, the heading rotation
// second (`thrust_vec_quat * yaw_quat`) - reversing this order would
// yaw in the earth frame before leaning, putting the lean on the wrong
// axis for any non-zero heading. thrust_vector is taken BY VALUE (real
// upstream's own signature, not a reference) since the body normalizes/
// resets a local copy.
[[nodiscard]] inline math::Quaternion attitude_from_thrust_vector(math::Vector3f thrust_vector,
                                                                    float heading_angle_rad) {
    // The direction of thrust is [0,0,-1] in any body-fixed frame - the
    // same real constant CCP-020's own thrust_vector_rotation_angles
    // already uses.
    const math::Vector3f thrust_vector_up{0.0f, 0.0f, -1.0f};

    if (math::is_zero(thrust_vector.length_squared())) {
        thrust_vector = thrust_vector_up;
    } else {
        thrust_vector.normalize();
    }

    // The cross product of the desired and target thrust vector defines
    // the rotation vector; the dot product gives the angle between them.
    math::Vector3f thrust_vec_cross = thrust_vector_up % thrust_vector;
    const float thrust_vector_angle =
        std::acos(math::constrain_value(thrust_vector_up * thrust_vector, -1.0f, 1.0f));

    // Same real degenerate-case fallback shape as thrust_vector_rotation_
    // angles above: reset to the thrust axis itself, not a zero vector,
    // when the cross product has no direction to offer or the angle is
    // already zero.
    const float thrust_vector_length = thrust_vec_cross.length();
    if (math::is_zero(thrust_vector_length) || math::is_zero(thrust_vector_angle)) {
        thrust_vec_cross = thrust_vector_up;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    math::Quaternion thrust_vec_quat;
    thrust_vec_quat.from_axis_angle(thrust_vec_cross, thrust_vector_angle);

    // Heading is about earth-frame down, (0,0,1) - the OPPOSITE real sign
    // from thrust_vector_up above (0,0,-1). Not a transcription slip: one
    // is the thrust direction, the other is earth-frame down.
    math::Quaternion yaw_quat;
    yaw_quat.from_axis_angle(math::Vector3f{0.0f, 0.0f, 1.0f}, heading_angle_rad);

    // Real, load-bearing composition order: thrust quaternion LEFT, yaw
    // quaternion RIGHT. See this file's own "CCP-024 ADDENDUM" banner.
    return thrust_vec_quat * yaw_quat;
}

// update_ang_vel_target_from_att_error - upstream AC_AttitudeControl::
// update_ang_vel_target_from_att_error (real lines 1345-1371). CCP-024 -
// see this file's own "CCP-024 ADDENDUM" banner above for the full design
// writeup (the real per-axis, not per-vehicle, strategy choice; the
// acceleration-halving fed to sqrt_controller; and the real,
// axis-different clamp bounds).
//
// Turns an attitude error rotation vector into a body-frame angular
// velocity target, one axis at a time. Each axis independently chooses
// between the square-root controller (CCP-021) and a plain proportional
// gain, based on whether use_sqrt_controller is set AND that SAME axis's
// own acceleration limit is non-zero - a vehicle can legitimately run
// sqrt on roll/pitch while running plain proportional on yaw in the same
// call.
//
// angle_p_scale is taken as a single explicit math::Vector3f (matching
// real upstream's own _angle_P_scale Vector3f member shape directly -
// see this file's own "CCP-024 ADDENDUM" banner for why); angle_kp_roll/
// angle_kp_pitch/angle_kp_yaw remain three separate explicit float
// parameters, standing in for upstream's own three separate
// _p_angle_roll/_p_angle_pitch/_p_angle_yaw .kP() accessors, per
// ADR-0012.
[[nodiscard]] inline math::Vector3f update_ang_vel_target_from_att_error(
    const math::Vector3f& attitude_error_rot_vec_rad, float angle_kp_roll, float angle_kp_pitch, float angle_kp_yaw,
    const math::Vector3f& angle_p_scale, float accel_roll_max_radss, float accel_pitch_max_radss,
    float accel_yaw_max_radss, bool use_sqrt_controller, float dt) {
    math::Vector3f rate_target_ang_vel;

    // Compute the roll angular velocity demand from the roll angle error.
    const float angleP_roll = angle_kp_roll * angle_p_scale.x;
    if (use_sqrt_controller && !math::is_zero(accel_roll_max_radss)) {
        rate_target_ang_vel.x = math::sqrt_controller(
            attitude_error_rot_vec_rad.x, angleP_roll,
            math::constrain_value(accel_roll_max_radss / 2.0f, kAccelRpControllerMinRadss,
                                   kAccelRpControllerMaxRadss),
            dt);
    } else {
        rate_target_ang_vel.x = angleP_roll * attitude_error_rot_vec_rad.x;
    }

    // Compute the pitch angular velocity demand from the pitch angle error.
    const float angleP_pitch = angle_kp_pitch * angle_p_scale.y;
    if (use_sqrt_controller && !math::is_zero(accel_pitch_max_radss)) {
        rate_target_ang_vel.y = math::sqrt_controller(
            attitude_error_rot_vec_rad.y, angleP_pitch,
            math::constrain_value(accel_pitch_max_radss / 2.0f, kAccelRpControllerMinRadss,
                                   kAccelRpControllerMaxRadss),
            dt);
    } else {
        rate_target_ang_vel.y = angleP_pitch * attitude_error_rot_vec_rad.y;
    }

    // Compute the yaw angular velocity demand from the yaw angle error -
    // reuses CCP-023's own already-merged kAccelYControllerMinRadss/
    // kAccelYControllerMaxRadss, NOT the roll/pitch bounds above.
    const float angleP_yaw = angle_kp_yaw * angle_p_scale.z;
    if (use_sqrt_controller && !math::is_zero(accel_yaw_max_radss)) {
        rate_target_ang_vel.z = math::sqrt_controller(
            attitude_error_rot_vec_rad.z, angleP_yaw,
            math::constrain_value(accel_yaw_max_radss / 2.0f, kAccelYControllerMinRadss, kAccelYControllerMaxRadss),
            dt);
    } else {
        rate_target_ang_vel.z = angleP_yaw * attitude_error_rot_vec_rad.z;
    }

    return rate_target_ang_vel;
}

} // namespace fwcpp::control
