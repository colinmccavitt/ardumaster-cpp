#pragma once

// Port of AP_Math/vector3.h + vector3.cpp's core algebra. CPP-007, slice 1.
//
// SLICE BOUNDARY (matches the vector2.hpp precedent): this lands the
// struct, equality, arithmetic operators, dot/cross, length/length_squared/
// limit_length_xy, normalize/normalized, is_zero/is_nan/is_inf, angle,
// rotate_xy, offset_bearing, reflect/project/projected, distance_squared,
// zero(), scale(), perpendicular, rfu_to_frd, tofloat/todouble.
//
// Deliberately NOT in this slice:
//   - rotate(enum Rotation)/rotate_inverse: need the Rotation enum and its
//     52-entry table (rotations.h) - a substantial separate port, tracked
//     as its own ticket rather than folded in here.
//   - row_times_mat/mul_rowcol: need Matrix3, not yet ported.
//   - xy(): upstream implements this by reinterpret_cast-ing a Vector3's
//     first two members as a Vector2&, relying on standard-layout aliasing
//     that happens to work. ADR-0012's "no unsafe reinterpretation" stance
//     (the same one ap-param's is_sentinel port took on the Rust side)
//     argues against reproducing that with reinterpret_cast here too.
//     Deferred until there's a concrete caller to design the safe
//     equivalent against, rather than guessing at one speculatively.
//   - distance_to_segment, closest_distance_between_line_and_point,
//     point_on_line_closest_to_other_point, segment_to_segment_closest_point,
//     segment_plane_intersect: geometry helpers built on the core algebra,
//     not core algebra themselves - same category deferred in vector2.hpp.
//
// UPSTREAM INCONSISTENCY, preserved rather than fixed: Vector3::angle()
// returns 0 for BOTH cosv>=1 (parallel) and cosv<=-1 (antiparallel).
// Vector2::angle() (vector2.cpp) returns M_PI for the antiparallel case.
// Same shape of function, different upstream files, genuinely different
// behavior - not a transcription slip in this port, a fact about upstream
// worth a reader knowing before "fixing" it to match Vector2's convention.
//
// Nothing in this slice has a bare double literal ambiguous under
// -fsingle-precision-constant (unlike vector2.hpp's angle()), so everything
// here is header-only.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>

namespace fwcpp::math {

template <typename T>
struct Vector3 {
    T x, y, z;

    constexpr Vector3() : x(0), y(0), z(0) {}
    constexpr Vector3(T x0, T y0, T z0) : x(x0), y(y0), z(z0) {}
    constexpr Vector3(const Vector2<T>& v0, T z0) : x(v0.x), y(v0.y), z(z0) {}

    [[nodiscard]] bool operator==(const Vector3<T>& v) const {
        return is_equal(x, v.x) && is_equal(y, v.y) && is_equal(z, v.z);
    }
    [[nodiscard]] bool operator!=(const Vector3<T>& v) const {
        return !is_equal(x, v.x) || !is_equal(y, v.y) || !is_equal(z, v.z);
    }

    [[nodiscard]] Vector3<T> operator-() const { return Vector3<T>(-x, -y, -z); }
    [[nodiscard]] Vector3<T> operator+(const Vector3<T>& v) const { return Vector3<T>(x + v.x, y + v.y, z + v.z); }
    [[nodiscard]] Vector3<T> operator-(const Vector3<T>& v) const { return Vector3<T>(x - v.x, y - v.y, z - v.z); }
    [[nodiscard]] Vector3<T> operator*(T num) const { return Vector3<T>(x * num, y * num, z * num); }
    [[nodiscard]] Vector3<T> operator/(T num) const { return Vector3<T>(x / num, y / num, z / num); }

    Vector3<T>& operator+=(const Vector3<T>& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vector3<T>& operator-=(const Vector3<T>& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vector3<T>& operator*=(T num) { x *= num; y *= num; z *= num; return *this; }
    Vector3<T>& operator/=(T num) { x /= num; y /= num; z /= num; return *this; }
    // Non-uniform (component-wise) scaling - upstream has no non-member
    // form of this, only the compound-assignment one shown here.
    Vector3<T>& operator*=(const Vector3<T>& v) { x *= v.x; y *= v.y; z *= v.z; return *this; }

    T& operator[](std::uint8_t i) { return (&x)[i]; }
    const T& operator[](std::uint8_t i) const { return (&x)[i]; }

    // Dot product.
    [[nodiscard]] T operator*(const Vector3<T>& v) const { return x * v.x + y * v.y + z * v.z; }
    [[nodiscard]] T dot(const Vector3<T>& v) const { return *this * v; }

    // Cross product.
    [[nodiscard]] Vector3<T> operator%(const Vector3<T>& v) const {
        return Vector3<T>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    [[nodiscard]] Vector3<T> cross(const Vector3<T>& v) const { return *this % v; }

    [[nodiscard]] Vector3<T> scale(T v) const { return *this * v; }

    // See the file banner: 0 for both parallel AND antiparallel, unlike
    // Vector2::angle() - upstream's own inconsistency, preserved.
    [[nodiscard]] T angle(const Vector3<T>& v2) const {
        const T len = length() * v2.length();
        if (len <= T(0)) {
            return T(0);
        }
        const T cosv = (*this * v2) / len;
        if (cosv >= T(1) || cosv <= T(-1)) {
            return T(0);
        }
        return std::acos(cosv);
    }

    [[nodiscard]] bool is_nan() const { return std::isnan(x) || std::isnan(y) || std::isnan(z); }
    [[nodiscard]] bool is_inf() const { return std::isinf(x) || std::isinf(y) || std::isinf(z); }

    // Generic (integral T): exact comparison. Overridden below for
    // float/double, matching upstream's template<> split.
    [[nodiscard]] bool is_zero() const { return x == T(0) && y == T(0) && z == T(0); }

    void zero() { x = y = z = T(0); }

    [[nodiscard]] T length_squared() const { return *this * *this; }
    [[nodiscard]] T length() const { return std::sqrt(x * x + y * y + z * z); }

    // Limits only the XY component's length, z untouched - upstream's own
    // scope for this function (used for horizontal-only speed/accel limits).
    bool limit_length_xy(T max_length) {
        const T length_xy = std::sqrt(x * x + y * y);
        if (length_xy > max_length && is_positive(length_xy)) {
            x *= (max_length / length_xy);
            y *= (max_length / length_xy);
            return true;
        }
        return false;
    }

    void normalize() { *this /= length(); }
    [[nodiscard]] Vector3<T> normalized() const { return *this / length(); }

    void project(const Vector3<T>& v) { *this = v * (*this * v) / (v * v); }
    [[nodiscard]] Vector3<T> projected(const Vector3<T>& v) const { return v * (*this * v) / (v * v); }

    void reflect(const Vector3<T>& n) {
        const Vector3<T> orig(*this);
        project(n);
        *this = *this * T(2) - orig;
    }

    [[nodiscard]] T distance_squared(const Vector3<T>& v) const {
        const T dx = x - v.x;
        const T dy = y - v.y;
        const T dz = z - v.z;
        return dx * dx + dy * dy + dz * dz;
    }

    // Rotate in the xy plane only, z untouched.
    void rotate_xy(T angle_rad) {
        const T cs = std::cos(angle_rad);
        const T sn = std::sin(angle_rad);
        const T rx = x * cs - y * sn;
        const T ry = x * sn + y * cs;
        x = rx;
        y = ry;
    }

    // Extrapolate position given bearing and pitch (degrees) and distance,
    // in place. NED-style convention: +x north, +y east, +z down (positive
    // pitch and distance move z toward down).
    void offset_bearing(T bearing, T pitch, T distance) {
        const T cp = std::cos(radians(pitch));
        y += cp * std::sin(radians(bearing)) * distance;
        x += cp * std::cos(radians(bearing)) * distance;
        z += std::sin(radians(pitch)) * distance;
    }

    // ENU (right-front-up) to NED (front-right-down): swap x/y, negate z.
    [[nodiscard]] Vector3<T> rfu_to_frd() const { return Vector3<T>(y, x, -z); }

    [[nodiscard]] Vector3<float> tofloat() const { return Vector3<float>(float(x), float(y), float(z)); }
    [[nodiscard]] Vector3<double> todouble() const { return Vector3<double>(x, y, z); }

    // Component of p1 perpendicular to v1 (p1 projected onto the plane
    // orthogonal to v1). Upstream's doc comment says "returns zero if p1 is
    // zero"; more precisely, the early-return fires whenever the dot
    // product p1*v1 is zero, which includes p1==0 but ALSO the case where
    // p1 is already perpendicular to v1 - in both cases it returns p1
    // itself unchanged, which only equals the zero vector in the first
    // case. Reproduced as upstream actually behaves, not as its comment
    // summarizes it.
    [[nodiscard]] static Vector3<T> perpendicular(const Vector3<T>& p1, const Vector3<T>& v1) {
        const T d = p1 * v1;
        if (math::is_zero(d)) {
            return p1;
        }
        const Vector3<T> parallel = (v1 * d) / v1.length_squared();
        return p1 - parallel;
    }
};

template <>
inline bool Vector3<float>::is_zero() const {
    return math::is_zero(x) && math::is_zero(y) && math::is_zero(z);
}
template <>
inline bool Vector3<double>::is_zero() const {
    return math::is_zero(x) && math::is_zero(y) && math::is_zero(z);
}

using Vector3i = Vector3<std::int16_t>;
using Vector3ui = Vector3<std::uint16_t>;
using Vector3l = Vector3<std::int32_t>;
using Vector3ul = Vector3<std::uint32_t>;
using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;

} // namespace fwcpp::math
