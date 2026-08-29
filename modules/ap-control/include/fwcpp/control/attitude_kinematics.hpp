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

#include <algorithm>
#include <cmath>

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

} // namespace fwcpp::control
