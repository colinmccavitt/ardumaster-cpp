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
// DEFERRED, explicitly, NOT started here: everything else in
// AC_AttitudeControl.cpp's own ~1550 lines (the attitude error
// decomposition, the command model, the rate target, entry points,
// relax/reset paths, etc.), AC_AttitudeControl_Multi, AC_PosControl,
// AC_WPNav, and the quaternion axis-angle math (from_axis_angle/
// to_axis_angle/from_rotation_vector/rotate_fast) copter-rust's own
// COP-007 needed for its next sub-ticket and which quaternion.hpp's own
// file banner explicitly excludes from this port's current slice.

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

} // namespace fwcpp::control
