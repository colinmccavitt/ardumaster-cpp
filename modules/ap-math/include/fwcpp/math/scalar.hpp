#pragma once

// Port of AP_Math's scalar helpers - is_zero/is_positive/is_negative,
// is_equal, the wrap_* family, and constrain_value. CPP-004.
//
// Upstream sources: AP_Math/ftype.h, AP_Math/AP_Math.h, AP_Math/AP_Math.cpp,
// AP_Math/definitions.h - read directly from the pinned Plane-4.7.0
// worktree (this is a from-scratch C++ port, not a translation of the Rust
// port's ap-math crate - see the fw-cpp effort charter on independence).
//
// The entire wrap_* family is DECLARED here but DEFINED in scalar.cpp, not
// header-inline, even though nothing stops them being header-only. This is
// deliberate, matches upstream's own AP_Math.cpp split, and exists for a
// concrete reason discovered while writing this module, not by design
// up front:
//
// ADR-0012 decision 1's -fsingle-precision-constant flag changes what a
// bare floating literal *means* (float instead of double). That can only be
// applied per translation unit. A first attempt applied it project-wide in
// CMakeLists.txt, including to test files - and CPP-004's own is_equal test
// silently exercised the float overload instead of the double one it was
// written to test, because its `1e-10` literal got coerced along with
// everything else.
//
// The fix moves the flag to a per-target INTERFACE (fwcpp_upstream_flags,
// linked PRIVATE by this module's .cpp only, see CMakeLists.txt) so tests
// get ordinary C++ literal semantics. But that raises a second problem for
// anything left header-only: a template like `wrap_PI<T>` referencing
// upstream's M_PI/M_2PI constants would instantiate differently depending
// on whether the *including* translation unit has the flag - the same
// template symbol meaning different bit patterns in different TUs, which is
// an ODR violation waiting to happen (the linker treats inline/template
// definitions as identical across TUs and keeps whichever one it saw
// first). Moving every wrap_* overload out of the header into scalar.cpp -
// compiled exactly once, always under the flag - removes the ambiguity
// entirely: the shipped behavior matches upstream regardless of what any
// consumer's translation unit does, and there is only one definition to
// begin with.
//
// is_zero/is_positive/is_negative/is_equal/constrain_value stay header-only:
// none of them contain a bare floating literal the flag could touch
// (FLT_EPSILON and numeric_limits<T>::epsilon() are already explicitly
// typed; constrain_value's only literal is the integer 2 in `/ 2`, and
// integer literals are untouched by a flag about *floating* constants).

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <type_traits>

#include <fwcpp/internal_error.hpp>

namespace fwcpp::math {

// D-003 (registered independently here; same conclusion the Rust port
// reached in FW-035, reproduced for the same reason, not merely copied):
// upstream's is_zero(double) compares against FLT_EPSILON, not DBL_EPSILON,
// under AP_MATH_ALLOW_DOUBLE_FUNCTIONS (true for SITL). is_zero gates
// is_positive/is_negative, which gate divide-by-zero guards throughout the
// estimator and controllers in the pattern
// `if (is_positive(len)) { x *= max/len; }`. A tighter (DBL_EPSILON)
// threshold would let smaller values pass the guard and reach the division,
// producing large scale factors upstream's looser threshold prevents.
// Reproduced deliberately: tightening it would move behavior in the
// dangerous direction, and the apparent inconsistency with is_equal (which
// does use the type-correct epsilon) is answered by the two functions asking
// different questions - is_equal asks "are these the same number"
// (precision), is_zero/is_positive ask "is this safe to divide by"
// (physical magnitude safety margin).

[[nodiscard]] inline bool is_zero(float x) {
    return std::fabs(x) < FLT_EPSILON;
}

[[nodiscard]] inline bool is_zero(double x) {
    return std::fabs(x) < static_cast<double>(FLT_EPSILON);
}

[[nodiscard]] inline bool is_positive(float x) {
    return x >= FLT_EPSILON;
}

[[nodiscard]] inline bool is_positive(double x) {
    return x >= static_cast<double>(FLT_EPSILON);
}

[[nodiscard]] inline bool is_negative(float x) {
    return x <= -FLT_EPSILON;
}

[[nodiscard]] inline bool is_negative(double x) {
    return x <= -static_cast<double>(FLT_EPSILON);
}

// is_equal: integral overload is exact comparison (upstream: cast both to
// their common type, compare directly).
template <typename A, typename B>
[[nodiscard]] inline
typename std::enable_if_t<std::is_integral_v<std::common_type_t<A, B>>, bool>
is_equal(A a, B b) {
    using Common = std::common_type_t<A, B>;
    return static_cast<Common>(a) == static_cast<Common>(b);
}

// is_equal: floating overload uses the *type-correct* epsilon (unlike
// is_zero above) - upstream picks the epsilon of the common type.
template <typename A, typename B>
[[nodiscard]] inline
typename std::enable_if_t<std::is_floating_point_v<std::common_type_t<A, B>>, bool>
is_equal(A a, B b) {
    using Common = std::common_type_t<A, B>;
    if constexpr (std::is_same_v<Common, double>) {
        return std::fabs(static_cast<double>(a) - static_cast<double>(b))
               < std::numeric_limits<double>::epsilon();
    } else {
        return std::fabs(static_cast<float>(a) - static_cast<float>(b))
               < std::numeric_limits<float>::epsilon();
    }
}

// Defined in scalar.cpp - see the file banner above. DEG_TO_RAD upstream is
// `M_PI / 180.0f` (definitions.h), a FLOAT-precision constant even where
// it's multiplied against a double `deg` - reproduced at that precision
// deliberately, not "fixed" to full double precision, matching this port's
// standing rule of reproducing upstream's flag-affected literals rather
// than improving them.
[[nodiscard]] double radians(double deg);
[[nodiscard]] float radians(float deg);

// The bare degrees-to-radians constant itself, exposed for the rare caller
// (Location::longitude_scale) that needs to reproduce a specific upstream
// literal-multiplication GROUPING rather than call radians() - floating
// point multiplication isn't associative, so `x * (a * DEG_TO_RAD)` and
// `radians(x * a)` (== `(x * a) * kDegToRad`) aren't guaranteed identical.
// One definition, here, rather than a second hardcoded copy elsewhere.
[[nodiscard]] float deg_to_rad_constant();
[[nodiscard]] float radians(int deg);
[[nodiscard]] float degrees(float rad); // upstream has no degrees(double) overload - not invented here either

[[nodiscard]] float wrap_360(float angle);
[[nodiscard]] double wrap_360(double angle);
[[nodiscard]] int wrap_360(int angle);
[[nodiscard]] float wrap_360_cd(float angle);
[[nodiscard]] double wrap_360_cd(double angle);
[[nodiscard]] long wrap_360_cd(long angle);
[[nodiscard]] int wrap_360_cd(int angle);
[[nodiscard]] float wrap_180(float angle);
[[nodiscard]] double wrap_180(double angle);
[[nodiscard]] int wrap_180(int angle);
[[nodiscard]] short wrap_180(short angle);
[[nodiscard]] float wrap_180_cd(float angle);
[[nodiscard]] double wrap_180_cd(double angle);
[[nodiscard]] int wrap_180_cd(int angle);
[[nodiscard]] long wrap_180_cd(long angle);
[[nodiscard]] short wrap_180_cd(short angle);
[[nodiscard]] float wrap_2PI(float radian);
[[nodiscard]] double wrap_2PI(double radian);
[[nodiscard]] float wrap_PI(float radian);
[[nodiscard]] double wrap_PI(double radian);

// safe_asin / safe_sqrt: both narrow to float BEFORE any comparison or call
// (upstream's own comment on safe_sqrt: "cast before checking so we sqrtf
// the same value we check"), so unlike the wrap_* family there is no
// double-precision code path and no bare literal whose meaning depends on
// the including TU's flags - M_PI_2 below is cast to float immediately
// either way the flag parses it, and the two orderings round to the same
// float. Safe to keep header-only.
template <typename T>
[[nodiscard]] inline float safe_asin(T v) {
    const float f = static_cast<float>(v);
    if (std::isnan(f)) {
        return 0.0f;
    }
    if (f >= 1.0f) {
        return static_cast<float>(M_PI_2);
    }
    if (f <= -1.0f) {
        return static_cast<float>(-M_PI_2);
    }
    return std::asin(f);
}

template <typename T>
[[nodiscard]] inline float safe_sqrt(T v) {
    const float val = static_cast<float>(v);
    if (std::isgreaterequal(val, 0.0f)) {
        return std::sqrt(val);
    }
    return 0.0f;
}

// constrain_value: NaN clamps to the midpoint (upstream's own choice, not
// this port's) and reports through `err` if non-null, matching upstream's
// `INTERNAL_ERROR(AP_InternalError::error_t::constraining_nan)` - but via
// the explicit, non-singleton fwcpp::InternalError (CPP-005) rather than
// AP::internalerror(). A null `err` is the same as a build with
// AP_INTERNALERROR_ENABLED off upstream: reporting is a no-op, but
// constrain_value's own NaN-clamp behavior is unaffected either way.
// Non-floating-point T never NaNs, so the check compiles away entirely via
// `if constexpr`.
template <typename T>
[[nodiscard]] inline T constrain_value(T amt, T low, T high, InternalError* err = nullptr, std::uint16_t line = 0) {
    if constexpr (std::is_floating_point_v<T>) {
        if (std::isnan(amt)) {
            if (err != nullptr) {
                err->record(InternalErrorCode::constraining_nan, line);
            }
            return (low + high) / 2;
        }
    }
    if (amt < low) {
        return low;
    }
    if (amt > high) {
        return high;
    }
    return amt;
}

} // namespace fwcpp::math
