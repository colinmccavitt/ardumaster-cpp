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

// ---------------------------------------------------------------------
// CCP-025 ADDENDUM: update_attitude_target (real lines 979-986) and
// attitude_controller_run_quat (real lines 989-1027), re-verified
// directly via `grep -n` against the pinned upstream tree - matches
// this ticket's own claimed ranges exactly (`grep -n
// 'update_attitude_target\|attitude_controller_run_quat'` puts the two
// definitions at real lines 979 and 989). This is the real control
// LOOP itself - Phase 8 of the copter-cpp effort's AC_AttitudeControl
// work, tying together almost everything the phase has built so far
// (CCP-018's ang_vel_limit, CCP-019's from_axis_angle, CCP-023's
// thrust_heading_rotation_angles, CCP-024's
// update_ang_vel_target_from_att_error).
//
// AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD - re-confirmed directly this round
// at AC_AttitudeControl.h real line 29: `radians(30.0f)`. Named below
// kAttitudeThrustErrorAngleRad, matching CCP-022/023/024's own
// established `inline const float` (NOT `inline constexpr`, since
// math::radians() is not constexpr-callable) computation discipline.
//
// REUSED INVESTIGATION: copter-rust's own COP-007 ticket already ported
// this exact function pair, under the real, actual merged names
// `update_attitude_target` and `attitude_controller_run` (NOT
// `attitude_controller_run_quat` - the Rust port renamed it, since
// "_quat" only disambiguated it from sibling `_run_*` entry points that
// port never built either), in
// ports/plane-fw-rust/crates/ap-control/src/attitude_error.rs (its own
// merged mainline copy - NOT the stale
// ports/plane-fw-rust/crates/ap-control/src/attitude_controller.rs file
// of the same crate, which only re-exports/consumes these functions,
// and NOT the /srv/ardumaster/worktrees/cop-023-params worktree, which
// turned out to be an unrelated, already-merged COP-023 branch, not
// this function pair's own home). Read in full before writing anything
// here; every finding below independently re-verified against the real
// C++ source rather than trusted on faith.
//
// THE REAL THREE-WAY BRANCH IS "SACRIFICE HEADING TO KEEP THRUST" MADE
// CONCRETE, copter-rust's own words reused directly: "A multirotor
// yaws by unbalancing rotor drag, which costs thrust margin - exactly
// what an aircraft with a large thrust error has none of. Fighting for
// heading there trades the thing that keeps it flying for the thing
// that decides which way it faces." Concretely, re-verified directly
// against real lines 1000-1023:
//   - Under 30 degrees of thrust_error_angle_rad: full feedforward on
//     all three axes.
//   - Between 30 and 60: roll/pitch feedforward fades out linearly:
//     `feedforward_scalar = 1 - (thrust_error_angle_rad -
//     AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD) / AC_ATTITUDE_THRUST_ERROR_
//     ANGLE_RAD` (1.0 at the low/30-degree end, 0.0 at the high/60-
//     degree end - re-verified directly). The yaw COMMAND ITSELF is
//     then blended toward the measured gyro rate.
//   - Over 60: yaw is replaced by the gyro outright; roll/pitch are not
//     touched at all.
//
// THE SINGLE MOST IMPORTANT, EASIEST-TO-GET-WRONG FINDING IN THIS
// TICKET - reused directly from copter-rust's own exact words: "in the
// middle band the yaw feedforward is added in FULL and then the whole
// yaw command is blended toward the gyro. It is NOT scaled by the
// feedforward scalar the way roll and pitch are, so applying the
// scalar to it as well - the obvious reading - would scale it twice."
// Re-verified directly against real lines 1017-1021: roll/pitch get
// `ang_vel_body_rads.{x,y} += ang_vel_body_feedforward.{x,y} *
// feedforward_scalar` (ONE scaled add each), while yaw gets, in this
// exact real order:
//   1. `ang_vel_body_rads.z += ang_vel_body_feedforward.z;`   (UNSCALED)
//   2. `ang_vel_body_rads.z = gyro.z * (1.0f - feedforward_scalar) +
//       ang_vel_body_rads.z * feedforward_scalar;`  (blend the
//       ALREADY-summed value)
// - a structurally different two-step process for yaw, not a parallel
// three-line pattern across all three axes. This module's own test
// file includes a dedicated test constructing a case where the real
// (correct) result and the naive "scale yaw at the point of addition
// too" result are computed side by side from the same underlying
// quantities and differ by a large, non-negligible margin (see "the
// yaw feedforward is not double-scaled" test below) - not merely
// eyeballed against the formula in isolation.
//
// A REAL, DISCLOSED EXTENSION BEYOND THE THREE PERSISTENT-STATE
// VARIABLES THIS TICKET NAMED EXPLICITLY: re-reading the real function
// body directly (lines 992-994) shows `_thrust_angle_rad` and
// `_thrust_error_angle_rad` are ALSO real persistent AC_AttitudeControl
// member state written every call (via the very same
// thrust_heading_rotation_angles call that populates the local
// `attitude_error` in upstream's own body) - not merely transient
// locals. Per the same ADR-0012 explicit-parameter discipline this
// ticket's own architectural note calls for, this port exposes THOSE
// two as explicit `float&` output parameters as well
// (`thrust_angle_rad`, `thrust_error_angle_rad`), on top of the three
// the ticket named directly (`feedforward_scalar`, `attitude_ang_error`,
// `ang_vel_body_rads`) - five real output parameters in total, matching
// upstream's own complete real persistent-state footprint for this
// function exactly. `attitude_error` itself, by contrast, is confirmed
// directly (real line 992) to be a plain LOCAL `Vector3f` in upstream's
// own body, never assigned to a member - this port matches that
// exactly too, keeping it a local inside attitude_controller_run_quat
// below rather than a sixth output parameter that upstream itself does
// not persist.
//
// THE ARCHITECTURAL DECISION, stated here and in this ticket's own
// commit message: no new stateful class is introduced. Every piece of
// real persistent AC_AttitudeControl state this function pair touches
// (`_feedforward_scalar`, `_attitude_ang_error`, `_thrust_angle_rad`,
// `_thrust_error_angle_rad`, `_ang_vel_body_rads`) is threaded through
// as an explicit reference output parameter, matching this whole
// module's own established free-function-with-explicit-state
// convention (CCP-011's check_for_failed_motor, CCP-013's output_logic,
// CCP-023's own mutating `attitude_target&` immediately above).
//
// REAL AHRS DEPENDENCIES, explicit parameters per ADR-0012 (real
// upstream: `_ahrs.get_quat_body_to_ned(attitude_body)` and
// `get_latest_gyro()`): `attitude_body` reuses the exact `const
// Quaternion&` shape CCP-020's own thrust_vector_rotation_angles
// already established above; `gyro_body_rads` is a new explicit `const
// Vector3f&` parameter standing in for the real gyro reading, matching
// this file's own established const-reference convention for
// input-only Vector3f quantities (attitude_error_rot_vec_rad,
// angle_p_scale, etc.).
//
// update_attitude_target's REAL COMPOSITION ORDER AND NORMALIZE, each
// re-verified directly against real lines 981-984:
//   1. `attitude_target_update.from_axis_angle(_ang_vel_target_rads *
//      _dt_s)` - the multiplication happens BEFORE the call, as one
//      real Vector3f, using the CCP-019 self-normalizing single-
//      Vector3-argument overload (a zero-length product resets to the
//      identity rotation, not a divide-by-zero).
//   2. `_attitude_target *= attitude_target_update` - the delta is
//      composed on the RIGHT of the existing target (this port's own
//      `operator*=` is `*this = *this * v`, matching this exactly).
//   3. `_attitude_target.normalize()` - re-verified this call is
//      unconditional, every single call, not merely occasional cleanup.
//      Real reason, reused directly from copter-rust's own words: "at
//      400 Hz the accumulated error from repeatedly composing a small
//      rotation is measurable within seconds." This module's own test
//      file includes a real, multi-iteration (4000, matching copter-
//      rust's own test exactly) test proving the quaternion's own
//      unit-length norm survives that many repeated compositions, which
//      would NOT hold without this explicit normalize() call - see "the
//      quaternion stays normalised over thousands of iterations" test
//      below.
//
// attitude_controller_run_quat's REAL STEP ORDER, each re-verified
// directly against real lines 989-1027:
//   1. Calls this file's own already-merged thrust_heading_rotation_
//      angles (CCP-023) with `attitude_target`/`attitude_body`,
//      producing `attitude_error`/`thrust_angle_rad`/`thrust_error_
//      angle_rad` - `attitude_target` may itself be mutated by this
//      call (CCP-023's own established semantics, reused directly:
//      this port's own signature keeps `attitude_target` a non-const
//      `Quaternion&` for exactly this reason).
//   2. Calls this file's own already-merged update_ang_vel_target_
//      from_att_error (CCP-024) with `attitude_error`, producing the
//      pre-limit `ang_vel_body_rads`.
//   3. Calls this file's own already-merged ang_vel_limit (CCP-018) on
//      `ang_vel_body_rads`, with the real roll/pitch/yaw max-rate
//      parameters each converted via math::radians() - matching this
//      whole file's own established ULP-precision discipline of never
//      hand-converting a degrees-to-radians literal.
//   4. `rotation_target_to_body = attitude_body.inverse() *
//      attitude_target` - re-verified this exact composition order
//      (body inverse LEFT, target RIGHT).
//   5. `ang_vel_body_feedforward = rotation_target_to_body *
//      ang_vel_target_rads`, using this port's own Quaternion::
//      operator*(Vector3) - rotating the TARGET's own angular velocity
//      (expressed in the target frame) into the body frame via the
//      just-computed relative rotation.
//   6. `gyro = gyro_body_rads` (this function's own explicit parameter).
//   7. The real three-way branch above, `feedforward_scalar = 1.0f` set
//      UNCONDITIONALLY before the branch runs (re-verified directly:
//      every real code path, including the under-30-degree `else`
//      branch where it is never subsequently read, still passes through
//      this initial assignment first).
//   8. `attitude_ang_error = attitude_body.inverse() * attitude_target`
//      - real upstream's own comment, reproduced verbatim: recorded "to
//      handle EKF resets". Re-verified this composition is IDENTICAL in
//      form to step 4's `rotation_target_to_body` (both are `attitude_
//      body.inverse() * attitude_target`), but evaluated a SECOND time
//      after `attitude_target` may have been mutated by step 1 above -
//      re-verified upstream really does recompute rather than reuse the
//      step-4 value, so this port's own body does too, not merely
//      aliasing `attitude_ang_error` to `rotation_target_to_body`.
//   9. The final `ang_vel_body_rads` (as computed by whichever branch
//      ran) is the function's own real primary output.
//
// DEFERRED, explicitly, still NOT started here, matching copter-rust's
// own COP-007 identically-still-open deferral of the same functions for
// the same reasons: command_model_rate_predictor (real lines 1134-1152
// - still needs real per-axis STATE this port has nowhere to source
// from yet, per this file's own CCP-022/023 addenda above), the
// input_* entry points (input_euler_angle_or_mag_rate, input_euler_rate_
// roll_pitch_yaw, input_rate_bf_roll_pitch_yaw, input_thrust_vector_
// rate_heading, input_thrust_vector_heading, input_quaternion, etc.),
// and the relax/reset paths (relax, reset_target_and_rate, reset_yaw_
// target_and_rate, inertial_frame_reset) - all separate, deliberately
// deferred future phases, none retroactively unblocked by this ticket's
// own two functions. This is a genuinely shared, still-open frontier
// for both ports, confirmed directly by copter-rust's own COP-007 notes
// naming the identical set as its own next steps.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// CCP-026 ADDENDUM: command_model_rate_predictor (real lines 1134-1152,
// re-verified directly via `grep -n` against the pinned upstream tree -
// matches this ticket's own claimed range exactly: the function opens
// at real line 1134 and its own closing brace is real line 1152).
//
// A SECOND REAL, DISCLOSED CORRECTION TO THIS EFFORT'S OWN EARLIER
// SCOPING - the same category of mistake CCP-023 already caught once
// for thrust_heading_rotation_angles (see this file's own "CCP-023
// ADDENDUM" banner above, "A REAL, DISCLOSED CORRECTION..."). CCP-022's
// own addendum above (and CCP-023's own addendum, repeating the same
// claim) deferred this exact function, reasoning it needed real
// per-axis STATE this port had nowhere to source from
// (`_rate_bf_ff_enabled`, `_angle_P_scale`). Re-reading the real
// function body this round shows that reasoning over-scoped the
// dependency exactly the same way CCP-023 already found for thrust_
// heading_rotation_angles: every real dependency here is either (a) a
// function this port has already built (attitude_command_model,
// CCP-022; ang_vel_limit, CCP-018), or (b) a plain scalar/flag value
// this port's own established ADR-0012 convention already takes as an
// explicit parameter - `_rate_bf_ff_enabled` is a plain bool, `_angle_
// P_scale` is the SAME Vector3f shape CCP-024 already established for
// `angle_p_scale` (reused directly, not reinvented), and every gain/
// max-rate/max-accel value is a plain float this file already threads
// through update_ang_vel_target_from_att_error and attitude_
// controller_run_quat the identical way. There was no real "missing
// state" here either - just parameters nobody had yet had a reason to
// name. This function was genuinely fully unblocked already, no new
// infrastructure needed - corrected here, the second such correction in
// this whole effort.
//
// THE dt-PARAMETER QUIRK, copter-rust's own COP-007-registered `D-025`,
// reused directly and independently RE-VERIFIED against the real source
// here (not trusted on the ticket's own summary): real upstream's own
// `command_model_rate_predictor(const Vector2f&, Vector2f&, Vector2f&,
// float dt) const` takes a `dt` PARAMETER but its own real body never
// reads it anywhere - both internal `attitude_command_model` calls
// (real lines 1138-1139) pass the MEMBER `_dt_s` explicitly, never the
// `dt` parameter; the `ang_vel_limit` call needs no dt at all.
// Confirmed directly against the real function body: `dt` is genuinely
// dead on arrival. copter-rust's own exact words, reused verbatim: "not
// an active defect... but a latent hazard, since any future caller
// asking about a different interval would be answered about the
// controller's own step [interval]."
//
// RESOLUTION CHOSEN: (b), NOT a faithful reproduction of the unused
// parameter. This port's own signature is being written fresh here, not
// literally inheriting a broken base-class parameter list the way real
// upstream's own override mechanics might force - there is no
// polymorphic caller anywhere in this port that could ever pass a
// genuinely different `dt` and have it silently ignored, because there
// is only ever the one real parameter list this function has ever had
// in this port. Faithfully reproducing a real but pointless footgun
// here would mean deliberately building a trap for exactly the "future
// caller asking about a different interval" scenario D-025 itself warns
// about - the safer, still-fully-disclosed choice is to take only the
// ONE real dt-shaped value the function's internal calls actually use
// (matching real upstream's own `_dt_s` member value) as a single
// explicit parameter, named `dt_s` (not `dt`) specifically so nothing
// about its name invites a caller to believe it is "the caller's own
// dt" the way real upstream's misleadingly-named `dt` parameter does.
// This mirrors copter-rust's own chosen resolution for the identical
// quirk in its own Rust port (ports/plane-fw-rust/crates/ap-control/src/
// attitude_controller.rs, `command_model_rate_predictor`, a single `dt:
// f32` parameter, no second unused one) - independently reached here by
// the same reasoning, not copied blind. See this file's own test file's
// dedicated "dt_s is the ONLY dt-shaped parameter, and it is genuinely
// live" test for the runtime proof this parameter is not itself a
// second phantom unused one.
//
// REAL STRUCTURE, each piece re-verified directly against real lines
// 1134-1152:
//   - Vector2f throughout (error_angle_rad in, target_ang_vel_rads/
//     target_ang_accel_rads out) - roll (.x) and pitch (.y) ONLY. Yaw
//     is never read or written anywhere in this function's real body -
//     re-verified directly, and confirmed structurally below (this
//     port's own signature has no yaw-shaped parameter at all to touch,
//     not merely "yaw happens to be zero in every test case").
//   - `if (_rate_bf_ff_enabled)`: TWO real attitude_command_model
//     calls, one per axis. Re-verified EXACT real argument mapping for
//     roll: `wrap_PI(error_angle_rad.x)` -> error_angle, literal `0.0`
//     -> desired_ang_vel, `target_ang_vel_rads.x`/`target_ang_accel_
//     rads.x` -> the two float& parameters, `radians(_ang_vel_roll_max_
//     degs)` -> max_ang_vel, `get_accel_roll_max_radss()` -> accel_max,
//     `_input_tc` -> input_tc, and real upstream's own `_dt_s` (NOT its
//     own `dt` parameter - the quirk above) -> dt. Pitch mirrors this
//     exactly with `.y`/pitch-suffixed values.
//   - IMPORTANT, independently re-derived while writing this ticket's own
//     tests (not stated explicitly by either the ticket or CCP-022's own
//     addendum above): in THIS branch, target_ang_vel_rads.x/.y and
//     target_ang_accel_rads.x/.y are genuine in/out STATE, not fresh
//     outputs computed from scratch. attitude_command_model's own
//     shape_angle_vel_accel call reads the INCOMING target_ang_vel as its
//     own current-velocity input (BY VALUE - see this file's own
//     CCP-022 ADDENDUM banner's THE ARGUMENT MAPPING section), and
//     its own final `target_ang_vel += target_ang_accel * dt` step
//     depends on whatever target_ang_vel already held on entry; target_
//     ang_accel is threaded similarly through shape_accel's own jerk-
//     limiting. This matches real upstream's own non-const `Vector2f&`
//     signature exactly - the caller is expected to hold this Vector2f
//     pair across repeated calls (e.g. one call per control loop tick),
//     not to re-zero it each time. This is precisely the real, ordinary
//     explicit-reference-parameter shape ADR-0012 already sanctions
//     (the caller owns the state, threads it through by reference) - not
//     the real per-axis STATE infrastructure CCP-022's own now-
//     corrected deferral reasoning worried about, which meant something
//     this port would have needed to newly build. Pinned below by a
//     dedicated test starting from a nonzero, axis-distinct prior state
//     and confirming the result genuinely differs from a fresh
//     zero-started call.
//   - `else`: `angleP_roll = angle_kp_roll * angle_p_scale.x`; `target_
//     ang_vel_rads.x = angleP_roll * wrap_PI(error_angle_rad.x)` (pitch
//     mirrors). Re-verified directly: `target_ang_accel_rads` is NEVER
//     assigned anywhere in this branch - real upstream's own Vector2f&
//     output parameter, so this leaves the CALLER's own prior value
//     completely untouched, not zeroed and not re-derived. Pinned below
//     by a dedicated sentinel test.
//   - Unconditionally after the branch: builds a real, temporary
//     `Vector3f ang_vel_rads(target_ang_vel_rads.x, target_ang_vel_
//     rads.y, 0.0f)` (re-verified the explicit `0.0f` yaw component),
//     calls this file's own already-merged ang_vel_limit (CCP-018) with
//     `radians(_ang_vel_roll_max_degs)`, `radians(_ang_vel_pitch_max_
//     degs)`, and a literal `0.0f` for the yaw-limit parameter -
//     re-verified this exact real `0.0f`, meaning ang_vel_limit's own
//     "zero means unlimited" convention leaves yaw completely
//     unconstrained here, consistent with this function never touching
//     yaw at all.
//   - Writes the (possibly re-limited) `ang_vel_rads.x`/`.y` back into
//     `target_ang_vel_rads.x`/`.y`. Re-verified `target_ang_accel_rads`
//     is NEVER touched by this final step - only by the earlier `_rate_
//     bf_ff_enabled`-true branch.
//
// SIGNATURE: `error_angle_rad`/`target_ang_vel_rads`/`target_ang_accel_
// rads` keep real upstream's own Vector2f shape exactly (this
// function's own real, narrow roll/pitch-only scope, unlike the
// Vector3f angle_p_scale below, which keeps its unused z component only
// because CCP-024 already established that shape for the SAME real
// member elsewhere in this file). `rate_bf_ff_enabled` (bool), `angle_
// kp_roll`/`angle_kp_pitch` (float, matching thrust_heading_rotation_
// angles's/update_ang_vel_target_from_att_error's own per-axis-gain
// convention above), `angle_p_scale` (const Vector3f&, CCP-024's own
// established shape, reused directly), `ang_vel_roll_max_degs`/`ang_
// vel_pitch_max_degs` (float, matching attitude_controller_run_quat's
// own degrees-in convention - converted via math::radians() inside,
// never hand-typed), `accel_roll_max_radss`/`accel_pitch_max_radss`
// (float, forwarded straight through to attitude_command_model,
// matching real upstream's own get_accel_roll_max_radss()/get_accel_
// pitch_max_radss() accessors exactly - already-in-radians, no
// conversion needed), `input_tc` (float, forwarded straight through,
// matching real upstream's own `_input_tc`), and `dt_s` (float, the
// chosen single dt-shaped parameter, resolution (b) above).
//
// DEFERRED, explicitly, still NOT started here, and NOT retroactively
// unblocked by this ticket's own correction above - matching copter-
// rust's own COP-007 notes, which confirm these genuinely still need
// real new infrastructure this port does not have yet (a stateful
// AttitudeController-equivalent, or a real AHRS-reading integration
// point), unlike this ticket's own now-corrected situation: the input_*
// entry points (input_euler_angle_or_mag_rate, input_euler_rate_roll_
// pitch_yaw, input_rate_bf_roll_pitch_yaw, input_thrust_vector_rate_
// heading, input_thrust_vector_heading, input_quaternion, etc.) and the
// relax/reset paths (relax, reset_target_and_rate, reset_yaw_target_
// and_rate, inertial_frame_reset).
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

// kAttitudeThrustErrorAngleRad - AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD
// (AC_AttitudeControl.h real line 29) - see this file's own "CCP-025
// ADDENDUM" banner above for why this is a real, runtime-initialized
// `inline const float` (not `inline constexpr`): math::radians() is not
// constexpr-callable.
inline const float kAttitudeThrustErrorAngleRad = math::radians(30.0f);

// update_attitude_target - upstream AC_AttitudeControl::
// update_attitude_target (real lines 979-986). CCP-025 - see this
// file's own "CCP-025 ADDENDUM" banner above for the full design
// writeup (the real composition order and the disclosed reason
// normalize() is unconditional, not optional cleanup).
//
// Advances attitude_target by one step of ang_vel_target_rads: builds a
// delta quaternion via the CCP-019 self-normalizing single-Vector3
// from_axis_angle overload, composes it on the RIGHT of the existing
// target, and normalizes the result - re-verify this last step is not
// dropped; this module's own test file pins the real reason (repeated
// composition drifts off the unit sphere within seconds at 400 Hz) with
// a dedicated multi-iteration test.
inline void update_attitude_target(math::Quaternion& attitude_target, const math::Vector3f& ang_vel_target_rads,
                                    float dt) {
    math::Quaternion attitude_target_update;
    attitude_target_update.from_axis_angle(ang_vel_target_rads * dt);
    attitude_target *= attitude_target_update;
    attitude_target.normalize();
}

// attitude_controller_run_quat - upstream AC_AttitudeControl::
// attitude_controller_run_quat (real lines 989-1027). CCP-025 - see
// this file's own "CCP-025 ADDENDUM" banner above for the full design
// writeup: the real three-way thrust-error branch, the single most
// important yaw-not-double-scaled asymmetry in this whole ticket, the
// architectural decision to expose every real persistent piece of state
// this function touches as an explicit output parameter rather than
// introduce a stateful class, and the explicit AHRS-dependency
// parameters (attitude_body, gyro_body_rads) standing in for real
// upstream's own _ahrs.get_quat_body_to_ned()/get_latest_gyro() calls.
//
// attitude_target is read AND, conditionally, mutated in place (via the
// thrust_heading_rotation_angles call below - CCP-023's own established
// semantics, propagated here exactly). thrust_angle_rad, thrust_error_
// angle_rad, feedforward_scalar, attitude_ang_error, and ang_vel_body_
// rads are this function's five real output parameters, matching real
// upstream's own complete _thrust_angle_rad/_thrust_error_angle_rad/
// _feedforward_scalar/_attitude_ang_error/_ang_vel_body_rads member
// footprint exactly - see this file's own banner addendum for why this
// is two more than the ticket's own three named examples.
inline void attitude_controller_run_quat(
    math::Quaternion& attitude_target, const math::Quaternion& attitude_body,
    const math::Vector3f& ang_vel_target_rads, const math::Vector3f& gyro_body_rads, float rate_yaw_kp,
    float angle_yaw_kp, float angle_kp_roll, float angle_kp_pitch, float angle_kp_yaw,
    const math::Vector3f& angle_p_scale, float accel_roll_max_radss, float accel_pitch_max_radss,
    float accel_yaw_max_radss, bool use_sqrt_controller, float ang_vel_roll_max_degs, float ang_vel_pitch_max_degs,
    float ang_vel_yaw_max_degs, float dt, float& thrust_angle_rad, float& thrust_error_angle_rad,
    float& feedforward_scalar, math::Quaternion& attitude_ang_error, math::Vector3f& ang_vel_body_rads) {
    // Step 1: the thrust/heading error decomposition (CCP-023), which
    // may itself mutate attitude_target - propagated via the same
    // non-const Quaternion& this function itself takes attitude_target
    // by. attitude_error is a plain LOCAL here, matching real upstream's
    // own real line 992 exactly (never assigned to a member there
    // either).
    math::Vector3f attitude_error;
    thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, thrust_angle_rad,
                                    thrust_error_angle_rad, rate_yaw_kp, angle_yaw_kp, accel_yaw_max_radss);

    // Step 2: the angle-error-to-rate-target conversion (CCP-024).
    ang_vel_body_rads = update_ang_vel_target_from_att_error(attitude_error, angle_kp_roll, angle_kp_pitch,
                                                              angle_kp_yaw, angle_p_scale, accel_roll_max_radss,
                                                              accel_pitch_max_radss, accel_yaw_max_radss,
                                                              use_sqrt_controller, dt);

    // Step 3: the configured rate limits (CCP-018), each bound converted
    // via math::radians() - never a hand-typed radian literal.
    ang_vel_limit(ang_vel_body_rads, math::radians(ang_vel_roll_max_degs), math::radians(ang_vel_pitch_max_degs),
                  math::radians(ang_vel_yaw_max_degs));

    // Steps 4-6: the target's own angular velocity, rotated from the
    // target frame into the body frame via the relative rotation
    // between the two attitudes, plus the real gyro reading.
    const math::Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    const math::Vector3f ang_vel_body_feedforward = rotation_target_to_body * ang_vel_target_rads;
    const math::Vector3f gyro = gyro_body_rads;

    // Step 7: the real three-way branch - see this file's own "CCP-025
    // ADDENDUM" banner above for the full writeup of why it exists and
    // the yaw asymmetry within it. feedforward_scalar is set here
    // UNCONDITIONALLY before the branch, re-verified directly against
    // real upstream: every path passes through this assignment first,
    // even the under-30-degree branch below where it is never
    // subsequently read.
    feedforward_scalar = 1.0f;
    if (thrust_error_angle_rad > 2.0f * kAttitudeThrustErrorAngleRad) {
        // Over 60 degrees: yaw is replaced by the gyro OUTRIGHT (a
        // plain overwrite, not a blend). Roll/pitch are NOT touched at
        // all in this branch - they keep exactly whatever steps 2+3
        // already produced, with ZERO feedforward added. See this
        // module's own dedicated test below proving this literally, not
        // just "heavily reduced".
        ang_vel_body_rads.z = gyro.z;
    } else if (thrust_error_angle_rad > kAttitudeThrustErrorAngleRad) {
        // 30-60 degrees, the real fade band. feedforward_scalar
        // evaluates to 1.0 at the low (30-degree) end and 0.0 at the
        // high (60-degree) end - re-verified directly.
        feedforward_scalar =
            1.0f - (thrust_error_angle_rad - kAttitudeThrustErrorAngleRad) / kAttitudeThrustErrorAngleRad;

        // Roll/pitch: ONE scaled add each.
        ang_vel_body_rads.x += ang_vel_body_feedforward.x * feedforward_scalar;
        ang_vel_body_rads.y += ang_vel_body_feedforward.y * feedforward_scalar;

        // Yaw: THE SINGLE MOST IMPORTANT ASYMMETRY IN THIS TICKET - see
        // this file's own banner addendum above. The feedforward is
        // added in FULL and UNSCALED first (NOT `* feedforward_scalar`,
        // which would be the "obvious," symmetric-looking, and WRONG
        // reading - that would scale the yaw feedforward twice, once
        // here and again in the blend immediately below). Only AFTER
        // this unscaled add does the entire, already-summed yaw command
        // get blended toward the gyro as one already-composed value.
        ang_vel_body_rads.z += ang_vel_body_feedforward.z;
        ang_vel_body_rads.z = gyro.z * (1.0f - feedforward_scalar) + ang_vel_body_rads.z * feedforward_scalar;
    } else {
        // Under 30 degrees: full feedforward, all three axes, one plain
        // whole-vector add. feedforward_scalar stays at its initial
        // 1.0f, unused in this branch.
        ang_vel_body_rads += ang_vel_body_feedforward;
    }

    // Step 8: recorded "to handle EKF resets" (real upstream's own
    // comment, reproduced verbatim). Re-verified this is genuinely
    // recomputed here, a second time, using whatever attitude_target
    // step 1 may have left it as - not merely an alias of the step-4
    // rotation_target_to_body value computed before that mutation could
    // have happened to matter.
    attitude_ang_error = attitude_body.inverse() * attitude_target;

    // Step 9: ang_vel_body_rads, as left by whichever branch ran above,
    // is this function's own real primary output.
}

// command_model_rate_predictor - upstream AC_AttitudeControl::
// command_model_rate_predictor (real lines 1134-1152). CCP-026 - see
// this file's own "CCP-026 ADDENDUM" banner above for the full design
// writeup: the corrected CCP-022/023 deferral reasoning, the D-025
// dt-parameter quirk and this port's chosen resolution (a single dt_s
// parameter, deliberately not a second unused one), and the exact real
// argument mapping into attitude_command_model/ang_vel_limit.
//
// Predicts the rate/acceleration targets an angle-error command would
// produce, without touching any real controller state - upstream's own
// use case is letting a caller (e.g. the position controller) ask "what
// rate would this angle request produce" ahead of time, without
// disturbing anything. Roll and pitch ONLY; this port's own signature
// has no yaw-shaped parameter anywhere, matching real upstream's own
// narrow Vector2f scope exactly.
inline void command_model_rate_predictor(const math::Vector2f& error_angle_rad, math::Vector2f& target_ang_vel_rads,
                                          math::Vector2f& target_ang_accel_rads, bool rate_bf_ff_enabled,
                                          float angle_kp_roll, float angle_kp_pitch,
                                          const math::Vector3f& angle_p_scale, float ang_vel_roll_max_degs,
                                          float ang_vel_pitch_max_degs, float accel_roll_max_radss,
                                          float accel_pitch_max_radss, float input_tc, float dt_s) {
    if (rate_bf_ff_enabled) {
        // dt_s (matching real upstream's own _dt_s member) is passed
        // here - see this file's own "CCP-026 ADDENDUM" banner's "THE
        // dt-PARAMETER QUIRK" section for why this port deliberately
        // takes only this one dt-shaped parameter, not a second, unused
        // one matching real upstream's own misleadingly-named `dt`.
        attitude_command_model(math::wrap_PI(error_angle_rad.x), 0.0f, target_ang_vel_rads.x,
                                target_ang_accel_rads.x, math::radians(ang_vel_roll_max_degs), accel_roll_max_radss,
                                input_tc, dt_s);
        attitude_command_model(math::wrap_PI(error_angle_rad.y), 0.0f, target_ang_vel_rads.y,
                                target_ang_accel_rads.y, math::radians(ang_vel_pitch_max_degs), accel_pitch_max_radss,
                                input_tc, dt_s);
    } else {
        // target_ang_accel_rads is deliberately left UNTOUCHED here -
        // real upstream's own Vector2f& output parameter is simply
        // never assigned anywhere in this branch. See this file's own
        // "CCP-026 ADDENDUM" banner and the dedicated sentinel test
        // below.
        const float angleP_roll = angle_kp_roll * angle_p_scale.x;
        const float angleP_pitch = angle_kp_pitch * angle_p_scale.y;
        target_ang_vel_rads.x = angleP_roll * math::wrap_PI(error_angle_rad.x);
        target_ang_vel_rads.y = angleP_pitch * math::wrap_PI(error_angle_rad.y);
    }

    // Re-clamp against the configured rate limits, unconditionally,
    // regardless of which branch ran above. Real upstream's own literal
    // 0.0f yaw-limit argument below leaves yaw completely unconstrained
    // (ang_vel_limit's own "zero means unlimited" convention),
    // consistent with this function never touching yaw anywhere.
    math::Vector3f ang_vel_rads(target_ang_vel_rads.x, target_ang_vel_rads.y, 0.0f);
    ang_vel_limit(ang_vel_rads, math::radians(ang_vel_roll_max_degs), math::radians(ang_vel_pitch_max_degs), 0.0f);

    target_ang_vel_rads.x = ang_vel_rads.x;
    target_ang_vel_rads.y = ang_vel_rads.y;
}

// ---------------------------------------------------------------------
// CCP-029 ADDENDUM: input_euler_angle_roll_pitch_euler_rate_yaw_rad
// (real lines 404-457, re-verified directly via `grep -n` against the
// pinned upstream tree - the function opens at real line 404 and its
// own closing brace is real line 457, essentially matching this
// ticket's own claimed 404-458 range - 458 is a trailing blank line)
// and its own trivial centidegree wrapper,
// input_euler_angle_roll_pitch_euler_rate_yaw_cd (real lines 390-397,
// confirmed exactly). This is the first real `input_*` ENTRY POINT
// this port builds - the pilot/autopilot-facing surface every lower-
// level building block in this whole AC_AttitudeControl phase (CCP-018
// through CCP-026) has been leading toward.
//
// THE REAL CONCEPTUAL REASON THIS FUNCTION (AND EVERY OTHER `input_*`
// ENTRY POINT) EXISTS AT ALL - reused verbatim from copter-rust's own
// COP-007 investigation (ports/plane-fw-rust/crates/ap-control/src/
// attitude_controller.rs's own file banner and its COP-007 ticket
// notes): "a pilot's stick position is NOT the attitude target. It is
// the attitude the target is shaped TOWARD, subject to rate and
// acceleration limits, and the target moves there over many
// iterations. The aircraft chases the target; the target chases the
// stick. Skipping that indirection gives an aircraft that snaps to
// stick inputs and cannot express a rate limit at all." REAL, LOAD-
// BEARING TESTING CONSEQUENCE, reused directly: "the entry point is
// stateful, so one call proves almost nothing: a shaping error
// converges to the same place either way and differs only in how it
// gets there." This module's own test file below drives the shaped
// (`rate_bf_ff_enabled == true`) branch through hundreds of successive
// calls for exactly this reason - see "a scripted stick sequence" test
// below.
//
// THE REAL FRAME-CONVERSION RATIONALE, reused directly: "The limits are
// converted body-frame to Euler-frame before shaping. Not a formality:
// an aircraft leaning hard needs a much larger Euler yaw rate to
// achieve a given body yaw rate, and limiting in the wrong frame would
// either throttle it needlessly or let it exceed the airframe."
// Re-verified directly below: body_to_euler_limit (CCP-018) is called
// TWICE, using the CURRENT `state.attitude_target` - i.e. AFTER step 1
// (update_attitude_target) has already advanced it for this call, but
// BEFORE any of this function's own per-axis shaping runs - matching
// real upstream's own real line 415-418 exactly (both calls read
// `_attitude_target` as it stands at that point in the function, not a
// value captured earlier or later).
//
// THE REAL ROLL/PITCH-VS-YAW ARGUMENT-SHAPE ASYMMETRY IN THE THREE
// attitude_command_model CALLS - re-verified directly against real
// lines 434-440, transcribed exactly, NOT assumed to share one shape:
//   - roll: attitude_command_model(wrap_PI(euler_roll_angle_rad -
//     euler_angle_target_rad.x), 0.0f, ..., input_tc, dt)
//   - pitch: attitude_command_model(wrap_PI(euler_pitch_angle_rad -
//     euler_angle_target_rad.y), 0.0f, ..., input_tc, dt)
//   - yaw: attitude_command_model(0.0f, euler_yaw_rate_rads, ...,
//     rate_y_tc, dt)
// Roll/pitch pass a real, nonzero `error_angle` (the wrapped difference
// between the commanded angle and the current Euler target) and a
// literal `0.0f` `desired_ang_vel` - they are angle commands. Yaw does
// the OPPOSITE: a literal `0.0f` `error_angle` and the real input yaw
// RATE as `desired_ang_vel` - it is a rate command, carried entirely
// through the shaper's velocity argument, never through its angle-error
// argument. AND the yaw call's own time constant is a genuinely
// DIFFERENT real parameter, `rate_y_tc` (upstream `_rate_y_tc`) - NOT
// `input_tc` (upstream `_input_tc`), which only the roll/pitch calls
// use. This port's own signature keeps both as two separate explicit
// float fields on EulerAngleRateShapingGains below, matching real
// upstream's own two genuinely separate member variables. This module's
// own test file below has a dedicated test proving BOTH of these real
// asymmetries with cases that would produce a measurably different
// result under the "obvious" (but wrong) assumption that all three
// calls share one argument shape, or that rate_y_tc and input_tc are
// interchangeable.
//
// get_roll_trim_rad() - A REAL, CONFIRMED MULTIROTOR SIMPLIFICATION,
// re-verified directly against both real headers this round:
// get_roll_trim_cd() is a real, virtual AC_AttitudeControl method whose
// BASE CLASS default is `return 0`; only the real, out-of-scope
// AC_AttitudeControl_Heli subclass overrides it to compensate tail-rotor
// thrust. Since this port's own charter is explicitly multirotor-first/
// non-heli, this term is ALWAYS real, exact 0.0f in this port's own
// actual scope. Per this ticket's own explicit instruction, this
// simplification is stated here rather than silently dropped, and no
// unused `virtual`-trim machinery is built for a heli case this port
// will never exercise - the `euler_roll_angle_rad += get_roll_trim_rad()`
// line upstream has at real line 419 simply has no equivalent below.
//
// THE REAL ARCHITECTURAL QUESTION THIS TICKET ASKS TO BE RESOLVED AND
// DISCLOSED: this function calls the already-merged
// attitude_controller_run_quat (CCP-025), which already has a
// substantial explicit-parameter signature (23 parameters, counted
// directly off its own real declaration above). Combined with this
// function's OWN new persistent state (attitude_target,
// euler_angle_target_rad, euler_rate_target_rads, ang_vel_target_rads -
// shared with CCP-025 - plus NEW state: ang_accel_target_rads) and its
// own new shaping-specific gains (rate_bf_ff_enabled, input_tc,
// rate_y_tc, on top of the rate/accel limits CCP-025 already needed), a
// fully-flattened single parameter list for THIS function would run to
// roughly 30+ individual parameters - confirmed by actually counting
// while drafting a flattened version before writing the version below.
//
// DECISION: two small, plain data-holding structs are introduced,
// scoped to this ticket alone -
//   - AttitudeTargetState bundles exactly the five real persistent
//     Quaternion/Vector3f targets this function and CCP-025 share
//     (attitude_target, euler_angle_target_rad, euler_rate_target_rads,
//     ang_vel_target_rads, ang_accel_target_rads) - the same five
//     fields copter-rust's own COP-007 AttitudeController struct
//     carries (minus attitude_ang_error, which stays CCP-025's own
//     separate output parameter below, matching that Rust struct's own
//     precedent of keeping it out of the bundled target state too).
//   - EulerAngleRateShapingGains bundles the mostly-constant-per-vehicle
//     tuning parameters this function reads: its own shaping config
//     (rate_bf_ff_enabled, input_tc, rate_y_tc, the three ang_vel_*_
//     max_degs, the three accel_*_max_radss) plus every gain
//     attitude_controller_run_quat itself needs downstream (rate_yaw_kp,
//     angle_yaw_kp, angle_kp_roll/pitch/yaw, angle_p_scale,
//     use_sqrt_controller) - mirroring copter-rust's own COP-007 split
//     of ShapingConfig/YawLimitGains/AngleGains into one bundle, since
//     this port has no existing per-purpose gains-struct precedent to
//     match instead.
// This does NOT violate this port's own ADR-0012 (which forbids
// singletons and hidden/implicit global state, not explicit structs a
// caller constructs and passes openly) - both structs are plain,
// caller-owned data with no methods and no ownership of anything beyond
// their own fields, passed by explicit reference exactly like every
// other explicit-parameter convention this whole phase already
// established (CCP-024's Vector3f angle_p_scale bundling three axes for
// the identical "upstream already stores this as one member" reason).
//
// attitude_controller_run_quat's OWN existing CCP-025 signature is
// LEFT AS-IS, NOT retrofitted to accept either struct. Three reasons:
// (1) it is already-merged, already-tested, working code at 23
// parameters - large, but meaningfully under the 25-30-parameter
// threshold this ticket's own architectural question is actually
// concerned with, so the problem this decision is solving does not
// independently apply to it; (2) retrofitting it would touch stable,
// already-verified control-loop code (CCP-025's own 8 existing tests)
// purely for stylistic consistency with a sibling function, not to fix
// a real problem in that function itself - a disproportionate risk for
// "the most complex assembly function yet" per this ticket's own
// framing; (3) this function's own body calls attitude_controller_run_
// quat by simply forwarding individual struct MEMBERS
// (state.attitude_target, gains.angle_kp_roll, etc.) as its already-
// established individual arguments - the struct bundling buys this
// ticket's own new function a real reduction (roughly 30+ down to 13
// parameters) without requiring CCP-025's own signature to change at
// all. If a THIRD entry point later needed the same gains bundle,
// retrofitting CCP-025 at that point would earn its keep on genuine,
// repeated duplication rather than a single caller's own preference.
//
// REAL STRUCTURE, each step re-verified directly against real lines
// 404-457:
//   1. update_attitude_target (CCP-025) - called FIRST, unconditionally,
//      before anything else in this function, exactly matching real
//      line 406.
//   2. state.attitude_target.to_euler(state.euler_angle_target_rad) -
//      this port's own existing Vector3f-overloaded to_euler (already
//      in quaternion.hpp before this ticket).
//   3. get_roll_trim_rad() - see the dedicated paragraph above; no code
//      emitted for this term.
//   4. The real top-level `if (rate_bf_ff_enabled)` branch:
//      - true: body_to_euler_limit (CCP-018) TWICE (accel limits, then
//        rate limits), both against the CURRENT state.attitude_target;
//        body_to_euler_derivative (CCP-018) once, converting the
//        current body-frame acceleration target
//        (state.ang_accel_target_rads) into euler_accel_target_rads -
//        re-verified upstream itself discards this call's own bool
//        return value (real line 424 does not check it), reproduced
//        here via an explicit `(void)`-discarded call rather than
//        silently dropping the [[nodiscard]] result; attitude_command_
//        model (CCP-022) THREE times with the real roll/pitch-vs-yaw
//        asymmetry documented above; euler_derivative_to_body (CCP-018)
//        TWICE, converting the shaped Euler rate and acceleration
//        targets back to body-frame feedforward
//        (state.ang_vel_target_rads, state.ang_accel_target_rads).
//      - false: state.euler_angle_target_rad.x/y set directly from the
//        input angles; state.euler_angle_target_rad.z INTEGRATED (`+=
//        euler_yaw_rate_rads * dt`, plain Euler integration, NOT
//        shaped, re-verified directly against real line 450);
//        state.attitude_target recomputed fresh via from_euler; all
//        three feedforward targets (euler_rate_target_rads,
//        ang_vel_target_rads, ang_accel_target_rads) ZEROED via their
//        own `.zero()` method, matching real upstream's own
//        `.zero()`-style calls at real lines 454-456 exactly.
//   5. attitude_controller_run_quat (CCP-025) - the REAL LAST STEP,
//      called unconditionally regardless of which branch above ran,
//      matching real line 457.
//
// TESTS (tests/attitude_kinematics_test.cpp): a real, multi-step (600
// iterations at 400 Hz / 1.5s, matching copter-rust's own COP-007
// stick-sequence rigor) scripted test for the shaped branch - a step in
// roll,
// a ramp in pitch, and a yaw rate that reverses sign partway through
// (copter-rust's own real script shape, reused directly), asserting
// convergence/tracking at multiple stages rather than only the final
// value; a single-call test of the unshaped (`rate_bf_ff_enabled ==
// false`) branch (legitimate here specifically, unlike the shaped
// branch, since this branch's own output is a direct, one-step function
// of its own inputs - re-verified this distinction directly against the
// real source above); the real roll/pitch-vs-yaw argument-shape
// asymmetry, and the real separate rate_y_tc/input_tc distinction, each
// with a dedicated case that would produce a measurably different
// result under the "obvious" but wrong assumption.
//
// DEFERRED, explicitly, as separate, deliberately deferred future
// tickets, NOT started here (roughly 19 real `input_*` entry points
// total in AC_AttitudeControl.cpp; most are thin unit-conversion or
// argument-reordering wrappers around a smaller number of substantive
// ones): input_euler_angle_roll_pitch_yaw_rad, input_euler_rate_roll_
// pitch_yaw_rads, the several input_rate_bf_roll_pitch_yaw_* variants,
// input_thrust_vector_rate_heading_rads, input_thrust_vector_heading_
// rad, input_thrust_vector_xy, input_quaternion, input_angle_step_bf_
// roll_pitch_yaw_rad, input_rate_step_bf_roll_pitch_yaw_rads, and the
// relax/reset paths (relax, reset_target_and_rate, reset_yaw_target_
// and_rate, inertial_frame_reset) - none retroactively unblocked by
// this ticket's own one entry point plus its trivial _cd wrapper.
// ---------------------------------------------------------------------

// AttitudeTargetState - the controller's own real persistent target
// state: upstream's _attitude_target/_euler_angle_target_rad/_euler_
// rate_target_rads/_ang_vel_target_rads (shared with CCP-025's own
// attitude_controller_run_quat above) plus this ticket's own new
// _ang_accel_target_rads. See this file's own "CCP-029 ADDENDUM" above
// for the full architectural writeup of why this struct exists and why
// attitude_controller_run_quat's own signature is not retrofitted to
// use it.
struct AttitudeTargetState {
    math::Quaternion attitude_target;
    math::Vector3f euler_angle_target_rad;
    math::Vector3f euler_rate_target_rads;
    math::Vector3f ang_vel_target_rads;
    math::Vector3f ang_accel_target_rads;
};

// EulerAngleRateShapingGains - the mostly-constant-per-vehicle tuning
// parameters this entry point reads, plus every gain
// attitude_controller_run_quat (CCP-025) itself needs downstream. See
// this file's own "CCP-029 ADDENDUM" above for the full writeup.
struct EulerAngleRateShapingGains {
    // This entry point's own shaping config (upstream _rate_bf_ff_
    // enabled/_input_tc/_rate_y_tc and the configured rate/accel
    // limits).
    bool rate_bf_ff_enabled = true;
    float input_tc = 0.0f;
    float rate_y_tc = 0.0f;
    // CCP-031: roll/pitch rate-shaping time constant (upstream
    // `_rate_rp_tc`). Distinct from `input_tc` (angle commands) and
    // `rate_y_tc` (yaw rate). Default 0 so the shaper's own
    // `dt * 10` fallback applies unless a caller sets it.
    float rate_rp_tc = 0.0f;
    float ang_vel_roll_max_degs = 0.0f;
    float ang_vel_pitch_max_degs = 0.0f;
    float ang_vel_yaw_max_degs = 0.0f;
    float rate_wp_yaw_max_degs = 0.0f;
    float accel_roll_max_radss = 0.0f;
    float accel_pitch_max_radss = 0.0f;
    float accel_yaw_max_radss = 0.0f;

    // Forwarded straight through to attitude_controller_run_quat
    // (CCP-025) - not read anywhere in this entry point's own body.
    float rate_yaw_kp = 0.0f;
    float angle_yaw_kp = 0.0f;
    float angle_kp_roll = 0.0f;
    float angle_kp_pitch = 0.0f;
    float angle_kp_yaw = 0.0f;
    math::Vector3f angle_p_scale{1.0f, 1.0f, 1.0f};
    bool use_sqrt_controller = false;
};

// input_euler_angle_roll_pitch_euler_rate_yaw_rad - upstream
// AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_rad
// (real lines 404-457). CCP-029 - see this file's own "CCP-029
// ADDENDUM" above for the full design writeup: the real conceptual
// framing this whole `input_*` family exists for, the frame-conversion
// rationale, the real roll/pitch-vs-yaw argument-shape asymmetry, the
// get_roll_trim_rad() multirotor simplification, and the architectural
// decision behind AttitudeTargetState/EulerAngleRateShapingGains above.
//
// state is read AND, unconditionally, mutated in place (attitude_target
// is additionally, conditionally mutated a second time inside the
// attitude_controller_run_quat call at the end, via that function's own
// established CCP-023 semantics). thrust_angle_rad, thrust_error_
// angle_rad, feedforward_scalar, attitude_ang_error, and ang_vel_body_
// rads are this function's own five real output parameters, forwarded
// straight from attitude_controller_run_quat's own identically-named
// five outputs - this entry point does not itself add any NEW output
// beyond what CCP-025 already produces.
inline void input_euler_angle_roll_pitch_euler_rate_yaw_rad(
    float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_rate_rads, AttitudeTargetState& state,
    const math::Quaternion& attitude_body, const math::Vector3f& gyro_body_rads,
    const EulerAngleRateShapingGains& gains, float dt, float& thrust_angle_rad, float& thrust_error_angle_rad,
    float& feedforward_scalar, math::Quaternion& attitude_ang_error, math::Vector3f& ang_vel_body_rads) {
    // Step 1: advance the target (CCP-025), unconditionally, before
    // anything else - real line 406.
    update_attitude_target(state.attitude_target, state.ang_vel_target_rads, dt);

    // Step 2: real line 409.
    state.attitude_target.to_euler(state.euler_angle_target_rad);

    // Step 3: get_roll_trim_rad() - see this file's own "CCP-029
    // ADDENDUM" above. Always exactly 0.0f on this port's own
    // multirotor-only scope, so no term is added here.

    if (gains.rate_bf_ff_enabled) {
        // Body-frame accel/rate limits, converted to the Euler frame
        // using the CURRENT (just-advanced) state.attitude_target -
        // real lines 415-418.
        const math::Vector3f euler_accel_radss = body_to_euler_limit(
            state.attitude_target,
            math::Vector3f{gains.accel_roll_max_radss, gains.accel_pitch_max_radss, gains.accel_yaw_max_radss});
        const math::Vector3f euler_rate_max_rads = body_to_euler_limit(
            state.attitude_target, math::Vector3f{math::radians(gains.ang_vel_roll_max_degs),
                                                    math::radians(gains.ang_vel_pitch_max_degs),
                                                    math::radians(gains.ang_vel_yaw_max_degs)});

        // The current body-frame acceleration target, converted to its
        // own Euler-frame equivalent - real lines 422-424. Upstream
        // itself discards this call's own bool return value; reproduced
        // here as an explicit (void)-discarded call rather than
        // silently dropping the [[nodiscard]] result.
        math::Vector3f euler_accel_target_rads;
        (void)body_to_euler_derivative(state.attitude_target, state.ang_accel_target_rads, euler_accel_target_rads);

        // Shape roll/pitch angle error and the yaw rate command into
        // Euler rate/acceleration targets - real lines 428-440. See
        // this file's own "CCP-029 ADDENDUM" above for the real roll/
        // pitch-vs-yaw argument-shape asymmetry and the real separate
        // rate_y_tc/input_tc distinction, both transcribed exactly
        // here.
        attitude_command_model(math::wrap_PI(euler_roll_angle_rad - state.euler_angle_target_rad.x), 0.0f,
                                state.euler_rate_target_rads.x, euler_accel_target_rads.x,
                                std::fabs(euler_rate_max_rads.x), euler_accel_radss.x, gains.input_tc, dt);
        attitude_command_model(math::wrap_PI(euler_pitch_angle_rad - state.euler_angle_target_rad.y), 0.0f,
                                state.euler_rate_target_rads.y, euler_accel_target_rads.y,
                                std::fabs(euler_rate_max_rads.y), euler_accel_radss.y, gains.input_tc, dt);
        attitude_command_model(0.0f, euler_yaw_rate_rads, state.euler_rate_target_rads.z, euler_accel_target_rads.z,
                                std::fabs(euler_rate_max_rads.z), euler_accel_radss.z, gains.rate_y_tc, dt);

        // Convert the shaped Euler rate/acceleration targets back to
        // body-frame feedforward vectors - real lines 443-446.
        state.ang_vel_target_rads = euler_derivative_to_body(state.attitude_target, state.euler_rate_target_rads);
        state.ang_accel_target_rads = euler_derivative_to_body(state.attitude_target, euler_accel_target_rads);
    } else {
        // No shaping/feedforward - real lines 449-456: roll/pitch
        // targets set directly, yaw target plain-Euler-integrated (NOT
        // shaped), the attitude target rebuilt fresh from the result,
        // and every feedforward target zeroed.
        state.euler_angle_target_rad.x = euler_roll_angle_rad;
        state.euler_angle_target_rad.y = euler_pitch_angle_rad;
        state.euler_angle_target_rad.z += euler_yaw_rate_rads * dt;

        state.attitude_target.from_euler(state.euler_angle_target_rad);

        state.euler_rate_target_rads.zero();
        state.ang_vel_target_rads.zero();
        state.ang_accel_target_rads.zero();
    }

    // Step 5: the real last step, unconditional regardless of which
    // branch ran above - real line 457.
    attitude_controller_run_quat(state.attitude_target, attitude_body, state.ang_vel_target_rads, gyro_body_rads,
                                  gains.rate_yaw_kp, gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch,
                                  gains.angle_kp_yaw, gains.angle_p_scale, gains.accel_roll_max_radss,
                                  gains.accel_pitch_max_radss, gains.accel_yaw_max_radss, gains.use_sqrt_controller,
                                  gains.ang_vel_roll_max_degs, gains.ang_vel_pitch_max_degs,
                                  gains.ang_vel_yaw_max_degs, dt, thrust_angle_rad, thrust_error_angle_rad,
                                  feedforward_scalar, attitude_ang_error, ang_vel_body_rads);
}

// input_euler_angle_roll_pitch_euler_rate_yaw_cd - upstream
// AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_cd
// (real lines 390-397). Trivial centidegree wrapper: converts the two
// angle inputs and the yaw-rate input via this port's own existing
// math::cd_to_rad and forwards everything else unchanged.
inline void input_euler_angle_roll_pitch_euler_rate_yaw_cd(
    float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds, AttitudeTargetState& state,
    const math::Quaternion& attitude_body, const math::Vector3f& gyro_body_rads,
    const EulerAngleRateShapingGains& gains, float dt, float& thrust_angle_rad, float& thrust_error_angle_rad,
    float& feedforward_scalar, math::Quaternion& attitude_ang_error, math::Vector3f& ang_vel_body_rads) {
    input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        math::cd_to_rad(euler_roll_angle_cd), math::cd_to_rad(euler_pitch_angle_cd),
        math::cd_to_rad(euler_yaw_rate_cds), state, attitude_body, gyro_body_rads, gains, dt, thrust_angle_rad,
        thrust_error_angle_rad, feedforward_scalar, attitude_ang_error, ang_vel_body_rads);
}


// ---------------------------------------------------------------------
// CCP-030 ADDENDUM: get_slew_yaw_max_rads (real lines 211-217),
// input_euler_angle_roll_pitch_yaw_rad (real lines 476-540), and its
// trivial centidegree wrapper input_euler_angle_roll_pitch_yaw_cd (real
// lines 461-468). Mirrors copter-rust COP-007's "autonomous entry
// point" investigation verbatim: a mode that knows where it wants to
// point commands a heading, not a rate.
//
// TWO LOAD-BEARING DIFFERENCES FROM CCP-029's rate-yaw entry point,
// reused directly from COP-007: (1) slew_yaw swaps the ordinary yaw rate
// limit for ATC_SLEW_YAW (get_slew_yaw_max_rads) - mission turn rate vs
// pilot spin rate on the same path. (2) yaw is shaped with input_tc
// like roll and pitch here - all three attitude_command_model calls
// share the IDENTICAL argument shape wrap_PI(input - target), 0.0,
// ..., input_tc - the OPPOSITE of CCP-029's asymmetric yaw-as-rate
// shape (0.0, yaw_rate, ..., rate_y_tc).
//
// ACOS PRECISION CLIFF (COP-007, reused verbatim): for small errors
// thrust_error uses acos(dot) with cos(theta)~1-theta^2/2; in f32
// within ~6e-8 of 1.0 rounds to 1.0 so acos returns zero. Combined
// roll-pitch error ~3.2e-4 is the boundary (~5e-8 for 1-cos). Tests
// below deliberately command errors well above that (~0.15 rad).
//
// ADR-0012: get_slew_yaw_max_rads takes ang_vel_yaw_max_degs and
// rate_wp_yaw_max_degs as explicit parameters; rate_wp_yaw_max_degs
// also lives on EulerAngleRateShapingGains for callers that bundle
// vehicle tuning.
// ---------------------------------------------------------------------

// get_slew_yaw_max_rads - upstream AC_AttitudeControl::get_slew_yaw_max_rads.
[[nodiscard]] inline float get_slew_yaw_max_rads(float ang_vel_yaw_max_degs, float rate_wp_yaw_max_degs) {
    if (!math::is_positive(ang_vel_yaw_max_degs)) {
        return math::radians(rate_wp_yaw_max_degs);
    }
    return std::min(math::radians(ang_vel_yaw_max_degs), math::radians(rate_wp_yaw_max_degs));
}

inline void input_euler_angle_roll_pitch_yaw_rad(
    float euler_roll_angle_rad, float euler_pitch_angle_rad, float euler_yaw_angle_rad, bool slew_yaw,
    AttitudeTargetState& state, const math::Quaternion& attitude_body, const math::Vector3f& gyro_body_rads,
    const EulerAngleRateShapingGains& gains, float dt, float& thrust_angle_rad, float& thrust_error_angle_rad,
    float& feedforward_scalar, math::Quaternion& attitude_ang_error, math::Vector3f& ang_vel_body_rads) {
    update_attitude_target(state.attitude_target, state.ang_vel_target_rads, dt);
    state.attitude_target.to_euler(state.euler_angle_target_rad);

    float yaw_rate_max_rads = math::radians(gains.ang_vel_yaw_max_degs);
    if (slew_yaw) {
        yaw_rate_max_rads = get_slew_yaw_max_rads(gains.ang_vel_yaw_max_degs, gains.rate_wp_yaw_max_degs);
    }

    if (gains.rate_bf_ff_enabled) {
        const math::Vector3f euler_accel_radss = body_to_euler_limit(
            state.attitude_target,
            math::Vector3f{gains.accel_roll_max_radss, gains.accel_pitch_max_radss, gains.accel_yaw_max_radss});
        const math::Vector3f euler_rate_max_rads = body_to_euler_limit(
            state.attitude_target,
            math::Vector3f{math::radians(gains.ang_vel_roll_max_degs), math::radians(gains.ang_vel_pitch_max_degs),
                           yaw_rate_max_rads});

        math::Vector3f euler_accel_target_rads;
        (void)body_to_euler_derivative(state.attitude_target, state.ang_accel_target_rads, euler_accel_target_rads);

        attitude_command_model(math::wrap_PI(euler_roll_angle_rad - state.euler_angle_target_rad.x), 0.0f,
                                state.euler_rate_target_rads.x, euler_accel_target_rads.x,
                                std::fabs(euler_rate_max_rads.x), euler_accel_radss.x, gains.input_tc, dt);
        attitude_command_model(math::wrap_PI(euler_pitch_angle_rad - state.euler_angle_target_rad.y), 0.0f,
                                state.euler_rate_target_rads.y, euler_accel_target_rads.y,
                                std::fabs(euler_rate_max_rads.y), euler_accel_radss.y, gains.input_tc, dt);
        attitude_command_model(math::wrap_PI(euler_yaw_angle_rad - state.euler_angle_target_rad.z), 0.0f,
                                state.euler_rate_target_rads.z, euler_accel_target_rads.z,
                                std::fabs(euler_rate_max_rads.z), euler_accel_radss.z, gains.input_tc, dt);

        state.ang_vel_target_rads = euler_derivative_to_body(state.attitude_target, state.euler_rate_target_rads);
        state.ang_accel_target_rads = euler_derivative_to_body(state.attitude_target, euler_accel_target_rads);
    } else {
        state.euler_angle_target_rad.x = euler_roll_angle_rad;
        state.euler_angle_target_rad.y = euler_pitch_angle_rad;

        if (math::is_positive(yaw_rate_max_rads)) {
            const float yaw_error = math::wrap_PI(euler_yaw_angle_rad - state.euler_angle_target_rad.z);
            const float yaw_step =
                math::constrain_value(yaw_error, -yaw_rate_max_rads * dt, yaw_rate_max_rads * dt);
            state.euler_angle_target_rad.z = math::wrap_PI(state.euler_angle_target_rad.z + yaw_step);
        } else {
            state.euler_angle_target_rad.z = euler_yaw_angle_rad;
        }

        state.attitude_target.from_euler(state.euler_angle_target_rad);

        state.euler_rate_target_rads.zero();
        state.ang_vel_target_rads.zero();
        state.ang_accel_target_rads.zero();
    }

    attitude_controller_run_quat(state.attitude_target, attitude_body, state.ang_vel_target_rads, gyro_body_rads,
                                  gains.rate_yaw_kp, gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch,
                                  gains.angle_kp_yaw, gains.angle_p_scale, gains.accel_roll_max_radss,
                                  gains.accel_pitch_max_radss, gains.accel_yaw_max_radss, gains.use_sqrt_controller,
                                  gains.ang_vel_roll_max_degs, gains.ang_vel_pitch_max_degs,
                                  gains.ang_vel_yaw_max_degs, dt, thrust_angle_rad, thrust_error_angle_rad,
                                  feedforward_scalar, attitude_ang_error, ang_vel_body_rads);
}

inline void input_euler_angle_roll_pitch_yaw_cd(
    float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw,
    AttitudeTargetState& state, const math::Quaternion& attitude_body, const math::Vector3f& gyro_body_rads,
    const EulerAngleRateShapingGains& gains, float dt, float& thrust_angle_rad, float& thrust_error_angle_rad,
    float& feedforward_scalar, math::Quaternion& attitude_ang_error, math::Vector3f& ang_vel_body_rads) {
    input_euler_angle_roll_pitch_yaw_rad(math::cd_to_rad(euler_roll_angle_cd), math::cd_to_rad(euler_pitch_angle_cd),
                                          math::cd_to_rad(euler_yaw_angle_cd), slew_yaw, state, attitude_body,
                                          gyro_body_rads, gains, dt, thrust_angle_rad, thrust_error_angle_rad,
                                          feedforward_scalar, attitude_ang_error, ang_vel_body_rads);
}

// ---------------------------------------------------------------------
// CCP-031 ADDENDUM: input_euler_rate_roll_pitch_yaw_rads (real lines
// 543-589). Acro-style entry point: every axis is a RATE command.
// Upstream has no `_cd` wrapper for this function.
//
// LOAD-BEARING DIFFERENCES FROM CCP-029:
//   - All three attitude_command_model calls use error_angle=0 and the
//     input rate as desired_ang_vel. Roll/pitch are NOT angle-error
//     shaped here.
//   - Roll and pitch use rate_rp_tc (upstream `_rate_rp_tc`); yaw uses
//     rate_y_tc. Neither is input_tc (that shapes commanded ANGLES).
//   - The shaper's max_ang_vel argument is the literal 0.0 (unlimited).
//     The command already is a rate; limiting it here would apply the
//     constraint twice. attitude_controller_run_quat still bounds the
//     result. There is no body_to_euler_limit of ang_vel_*_max.
//
// Unshaped branch (rate_bf_ff_enabled false) treats each axis
// differently, and each is correct for that axis:
//   - roll: wrap_PI (signed lean)
//   - pitch: constrain to ±radians(85) — clamp, not wrap, to avoid
//     jumping through the Euler singularity past 90 deg
//   - yaw: wrap_2PI (compass heading), NOT wrap_PI
// Then zero euler_rate / ang_vel / ang_accel targets and from_euler.
//
// Always ends with attitude_controller_run_quat (real line 588).
// ---------------------------------------------------------------------

inline void input_euler_rate_roll_pitch_yaw_rads(
    float euler_roll_rate_rads, float euler_pitch_rate_rads, float euler_yaw_rate_rads, AttitudeTargetState& state,
    const math::Quaternion& attitude_body, const math::Vector3f& gyro_body_rads,
    const EulerAngleRateShapingGains& gains, float dt, float& thrust_angle_rad, float& thrust_error_angle_rad,
    float& feedforward_scalar, math::Quaternion& attitude_ang_error, math::Vector3f& ang_vel_body_rads) {
    update_attitude_target(state.attitude_target, state.ang_vel_target_rads, dt);
    state.attitude_target.to_euler(state.euler_angle_target_rad);

    if (gains.rate_bf_ff_enabled) {
        const math::Vector3f euler_accel_radss = body_to_euler_limit(
            state.attitude_target,
            math::Vector3f{gains.accel_roll_max_radss, gains.accel_pitch_max_radss, gains.accel_yaw_max_radss});

        math::Vector3f euler_accel_target_rads;
        (void)body_to_euler_derivative(state.attitude_target, state.ang_accel_target_rads, euler_accel_target_rads);

        // All three axes are rate commands (error_angle=0). Roll/pitch
        // use rate_rp_tc; yaw uses rate_y_tc. max_ang_vel is 0.0
        // (unlimited) — real lines 562-564.
        attitude_command_model(0.0f, euler_roll_rate_rads, state.euler_rate_target_rads.x, euler_accel_target_rads.x,
                                0.0f, euler_accel_radss.x, gains.rate_rp_tc, dt);
        attitude_command_model(0.0f, euler_pitch_rate_rads, state.euler_rate_target_rads.y, euler_accel_target_rads.y,
                                0.0f, euler_accel_radss.y, gains.rate_rp_tc, dt);
        attitude_command_model(0.0f, euler_yaw_rate_rads, state.euler_rate_target_rads.z, euler_accel_target_rads.z,
                                0.0f, euler_accel_radss.z, gains.rate_y_tc, dt);

        state.ang_vel_target_rads = euler_derivative_to_body(state.attitude_target, state.euler_rate_target_rads);
        state.ang_accel_target_rads = euler_derivative_to_body(state.attitude_target, euler_accel_target_rads);
    } else {
        state.euler_angle_target_rad.x =
            math::wrap_PI(state.euler_angle_target_rad.x + euler_roll_rate_rads * dt);
        state.euler_angle_target_rad.y =
            math::constrain_value(state.euler_angle_target_rad.y + euler_pitch_rate_rads * dt,
                                  math::radians(-85.0f), math::radians(85.0f));
        state.euler_angle_target_rad.z =
            math::wrap_2PI(state.euler_angle_target_rad.z + euler_yaw_rate_rads * dt);

        state.euler_rate_target_rads.zero();
        state.ang_vel_target_rads.zero();
        state.ang_accel_target_rads.zero();

        state.attitude_target.from_euler(state.euler_angle_target_rad);
    }

    attitude_controller_run_quat(state.attitude_target, attitude_body, state.ang_vel_target_rads, gyro_body_rads,
                                  gains.rate_yaw_kp, gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch,
                                  gains.angle_kp_yaw, gains.angle_p_scale, gains.accel_roll_max_radss,
                                  gains.accel_pitch_max_radss, gains.accel_yaw_max_radss, gains.use_sqrt_controller,
                                  gains.ang_vel_roll_max_degs, gains.ang_vel_pitch_max_degs,
                                  gains.ang_vel_yaw_max_degs, dt, thrust_angle_rad, thrust_error_angle_rad,
                                  feedforward_scalar, attitude_ang_error, ang_vel_body_rads);
}

// ---------------------------------------------------------------------
// CCP-032 ADDENDUM: the body-frame rate family used by ACRO
// (input_rate_bf_roll_pitch_yaw_rads / _2_rads / _3_rads /
// no_shaping_rads and each one's trivial _cds wrapper). Upstream
// AC_AttitudeControl.cpp real lines 594-782.
//
// Four real variants, not one function with a mode flag. Each is a
// distinct control law:
//
//   1. Stabilized acro (input_rate_bf_roll_pitch_yaw_rads). Shapes
//      BODY-FRAME ang_vel / ang_accel targets (rate_rp_tc on roll+pitch,
//      rate_y_tc on yaw, max_ang_vel 0.0), then body_to_euler_derivative
//      into euler_rate_target. Unshaped: right-multiply
//      from_axis_angle(rates * dt) onto attitude_target and zero every
//      feedforward vector. Always ends in attitude_controller_run_quat.
//      Opposite of CCP-031, which shapes Euler rates then converts to
//      body, and whose unshaped path is per-axis Euler wrap/clamp.
//
//   2. Rate-only acro (_2_rads). ALWAYS shapes (no rate_bf_ff_enabled
//      branch). Copies the injected AHRS body-to-ned quat into
//      attitude_target (ADR-0012: do not call AHRS). Sets
//      ang_vel_body_rads = ang_vel_target. Does NOT call run_quat.
//
//   3. Plane acro with rate-error integration (_3_rads). Clamps the
//      integrated attitude_ang_error to kAttitudeThrustErrorAngleRad,
//      then left-multiplies from_axis_angle((ang_vel_target - gyro) * dt)
//      using THIS frame's pre-shape ang_vel_target (last command). Then
//      shapes, sets attitude_target = attitude_body * attitude_ang_error,
//      and ang_vel_body = update_ang_vel_target_from_att_error(error)
//      + shaped ang_vel_target. No run_quat. gyro_latest_rads stands in
//      for get_latest_gyro(); attitude_body stands in for
//      _ahrs.get_quat_body_to_ned.
//
//   4. no_shaping_rads. Writes the three rate inputs straight into
//      ang_vel_target (no command_model). Copies AHRS into
//      attitude_target. ang_vel_body = ang_vel_target. No run_quat.
// ---------------------------------------------------------------------

inline void input_rate_bf_roll_pitch_yaw_rads(
    float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads, AttitudeTargetState& state,
    const math::Quaternion& attitude_body, const math::Vector3f& gyro_body_rads,
    const EulerAngleRateShapingGains& gains, float dt, float& thrust_angle_rad, float& thrust_error_angle_rad,
    float& feedforward_scalar, math::Quaternion& attitude_ang_error, math::Vector3f& ang_vel_body_rads) {
    update_attitude_target(state.attitude_target, state.ang_vel_target_rads, dt);
    state.attitude_target.to_euler(state.euler_angle_target_rad);

    if (gains.rate_bf_ff_enabled) {
        // Body-frame rate commands into body-frame ang_vel / ang_accel
        // targets. max_ang_vel is the literal 0.0 (unlimited) — real
        // lines 620-622. Roll/pitch use rate_rp_tc; yaw uses rate_y_tc.
        attitude_command_model(0.0f, roll_rate_bf_rads, state.ang_vel_target_rads.x, state.ang_accel_target_rads.x,
                                0.0f, gains.accel_roll_max_radss, gains.rate_rp_tc, dt);
        attitude_command_model(0.0f, pitch_rate_bf_rads, state.ang_vel_target_rads.y, state.ang_accel_target_rads.y,
                                0.0f, gains.accel_pitch_max_radss, gains.rate_rp_tc, dt);
        attitude_command_model(0.0f, yaw_rate_bf_rads, state.ang_vel_target_rads.z, state.ang_accel_target_rads.z,
                                0.0f, gains.accel_yaw_max_radss, gains.rate_y_tc, dt);

        (void)body_to_euler_derivative(state.attitude_target, state.ang_vel_target_rads, state.euler_rate_target_rads);
    } else {
        math::Quaternion attitude_target_update;
        attitude_target_update.from_axis_angle(
            math::Vector3f{roll_rate_bf_rads, pitch_rate_bf_rads, yaw_rate_bf_rads} * dt);
        state.attitude_target = state.attitude_target * attitude_target_update;
        state.attitude_target.normalize();

        state.euler_rate_target_rads.zero();
        state.ang_vel_target_rads.zero();
        state.ang_accel_target_rads.zero();
    }

    attitude_controller_run_quat(state.attitude_target, attitude_body, state.ang_vel_target_rads, gyro_body_rads,
                                  gains.rate_yaw_kp, gains.angle_yaw_kp, gains.angle_kp_roll, gains.angle_kp_pitch,
                                  gains.angle_kp_yaw, gains.angle_p_scale, gains.accel_roll_max_radss,
                                  gains.accel_pitch_max_radss, gains.accel_yaw_max_radss, gains.use_sqrt_controller,
                                  gains.ang_vel_roll_max_degs, gains.ang_vel_pitch_max_degs,
                                  gains.ang_vel_yaw_max_degs, dt, thrust_angle_rad, thrust_error_angle_rad,
                                  feedforward_scalar, attitude_ang_error, ang_vel_body_rads);
}

inline void input_rate_bf_roll_pitch_yaw_cds(
    float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds, AttitudeTargetState& state,
    const math::Quaternion& attitude_body, const math::Vector3f& gyro_body_rads,
    const EulerAngleRateShapingGains& gains, float dt, float& thrust_angle_rad, float& thrust_error_angle_rad,
    float& feedforward_scalar, math::Quaternion& attitude_ang_error, math::Vector3f& ang_vel_body_rads) {
    input_rate_bf_roll_pitch_yaw_rads(math::cd_to_rad(roll_rate_bf_cds), math::cd_to_rad(pitch_rate_bf_cds),
                                      math::cd_to_rad(yaw_rate_bf_cds), state, attitude_body, gyro_body_rads, gains,
                                      dt, thrust_angle_rad, thrust_error_angle_rad, feedforward_scalar,
                                      attitude_ang_error, ang_vel_body_rads);
}

inline void input_rate_bf_roll_pitch_yaw_2_rads(
    float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads, AttitudeTargetState& state,
    const math::Quaternion& attitude_body, const EulerAngleRateShapingGains& gains, float dt,
    math::Vector3f& ang_vel_body_rads) {
    // Always shapes — no rate_bf_ff_enabled branch. Real lines 664-666.
    attitude_command_model(0.0f, roll_rate_bf_rads, state.ang_vel_target_rads.x, state.ang_accel_target_rads.x, 0.0f,
                            gains.accel_roll_max_radss, gains.rate_rp_tc, dt);
    attitude_command_model(0.0f, pitch_rate_bf_rads, state.ang_vel_target_rads.y, state.ang_accel_target_rads.y, 0.0f,
                            gains.accel_pitch_max_radss, gains.rate_rp_tc, dt);
    attitude_command_model(0.0f, yaw_rate_bf_rads, state.ang_vel_target_rads.z, state.ang_accel_target_rads.z, 0.0f,
                            gains.accel_yaw_max_radss, gains.rate_y_tc, dt);

    // Injected AHRS body-to-ned quat, not an AHRS call. Real lines 669-672.
    state.attitude_target = attitude_body;
    state.attitude_target.to_euler(state.euler_angle_target_rad);
    (void)body_to_euler_derivative(state.attitude_target, state.ang_vel_target_rads, state.euler_rate_target_rads);

    ang_vel_body_rads = state.ang_vel_target_rads;
}

inline void input_rate_bf_roll_pitch_yaw_2_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds,
                                               AttitudeTargetState& state, const math::Quaternion& attitude_body,
                                               const EulerAngleRateShapingGains& gains, float dt,
                                               math::Vector3f& ang_vel_body_rads) {
    input_rate_bf_roll_pitch_yaw_2_rads(math::cd_to_rad(roll_rate_bf_cds), math::cd_to_rad(pitch_rate_bf_cds),
                                        math::cd_to_rad(yaw_rate_bf_cds), state, attitude_body, gains, dt,
                                        ang_vel_body_rads);
}

inline void input_rate_bf_roll_pitch_yaw_3_rads(
    float roll_rate_bf_rads, float pitch_rate_bf_rads, float yaw_rate_bf_rads, AttitudeTargetState& state,
    const math::Quaternion& attitude_body, const math::Vector3f& gyro_latest_rads,
    const EulerAngleRateShapingGains& gains, float dt, math::Quaternion& attitude_ang_error,
    math::Vector3f& ang_vel_body_rads) {
    math::Vector3f attitude_error;
    attitude_ang_error.to_axis_angle(attitude_error);

    const float err_mag = attitude_error.length();
    if (err_mag > kAttitudeThrustErrorAngleRad) {
        attitude_error *= kAttitudeThrustErrorAngleRad / err_mag;
        attitude_ang_error.from_axis_angle(attitude_error);
    }

    // Rate-error integration uses the PRE-shape ang_vel_target (last
    // frame's command) minus the injected latest gyro. Composition is
    // update * error (left multiply). Real lines 710-713.
    math::Quaternion attitude_ang_error_update_quat;
    attitude_ang_error_update_quat.from_axis_angle((state.ang_vel_target_rads - gyro_latest_rads) * dt);
    attitude_ang_error = attitude_ang_error_update_quat * attitude_ang_error;
    attitude_ang_error.normalize();

    attitude_command_model(0.0f, roll_rate_bf_rads, state.ang_vel_target_rads.x, state.ang_accel_target_rads.x, 0.0f,
                            gains.accel_roll_max_radss, gains.rate_rp_tc, dt);
    attitude_command_model(0.0f, pitch_rate_bf_rads, state.ang_vel_target_rads.y, state.ang_accel_target_rads.y, 0.0f,
                            gains.accel_pitch_max_radss, gains.rate_rp_tc, dt);
    attitude_command_model(0.0f, yaw_rate_bf_rads, state.ang_vel_target_rads.z, state.ang_accel_target_rads.z, 0.0f,
                            gains.accel_yaw_max_radss, gains.rate_y_tc, dt);

    state.attitude_target = attitude_body * attitude_ang_error;
    state.attitude_target.normalize();

    state.attitude_target.to_euler(state.euler_angle_target_rad);
    (void)body_to_euler_derivative(state.attitude_target, state.ang_vel_target_rads, state.euler_rate_target_rads);

    attitude_ang_error.to_axis_angle(attitude_error);
    ang_vel_body_rads = update_ang_vel_target_from_att_error(
        attitude_error, gains.angle_kp_roll, gains.angle_kp_pitch, gains.angle_kp_yaw, gains.angle_p_scale,
        gains.accel_roll_max_radss, gains.accel_pitch_max_radss, gains.accel_yaw_max_radss, gains.use_sqrt_controller,
        dt);
    ang_vel_body_rads += state.ang_vel_target_rads;
}

inline void input_rate_bf_roll_pitch_yaw_3_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds,
                                               AttitudeTargetState& state, const math::Quaternion& attitude_body,
                                               const math::Vector3f& gyro_latest_rads,
                                               const EulerAngleRateShapingGains& gains, float dt,
                                               math::Quaternion& attitude_ang_error,
                                               math::Vector3f& ang_vel_body_rads) {
    input_rate_bf_roll_pitch_yaw_3_rads(math::cd_to_rad(roll_rate_bf_cds), math::cd_to_rad(pitch_rate_bf_cds),
                                        math::cd_to_rad(yaw_rate_bf_cds), state, attitude_body, gyro_latest_rads,
                                        gains, dt, attitude_ang_error, ang_vel_body_rads);
}

inline void input_rate_bf_roll_pitch_yaw_no_shaping_rads(float roll_rate_bf_rads, float pitch_rate_bf_rads,
                                                         float yaw_rate_bf_rads, AttitudeTargetState& state,
                                                         const math::Quaternion& attitude_body,
                                                         math::Vector3f& ang_vel_body_rads) {
    state.ang_vel_target_rads.x = roll_rate_bf_rads;
    state.ang_vel_target_rads.y = pitch_rate_bf_rads;
    state.ang_vel_target_rads.z = yaw_rate_bf_rads;

    state.attitude_target = attitude_body;
    state.attitude_target.to_euler(state.euler_angle_target_rad);
    (void)body_to_euler_derivative(state.attitude_target, state.ang_vel_target_rads, state.euler_rate_target_rads);

    ang_vel_body_rads = state.ang_vel_target_rads;
}

inline void input_rate_bf_roll_pitch_yaw_no_shaping_cds(float roll_rate_bf_cds, float pitch_rate_bf_cds,
                                                        float yaw_rate_bf_cds, AttitudeTargetState& state,
                                                        const math::Quaternion& attitude_body,
                                                        math::Vector3f& ang_vel_body_rads) {
    input_rate_bf_roll_pitch_yaw_no_shaping_rads(math::cd_to_rad(roll_rate_bf_cds), math::cd_to_rad(pitch_rate_bf_cds),
                                                 math::cd_to_rad(yaw_rate_bf_cds), state, attitude_body,
                                                 ang_vel_body_rads);
}

} // namespace fwcpp::control
