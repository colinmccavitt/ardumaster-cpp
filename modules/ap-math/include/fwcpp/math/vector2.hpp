#pragma once

// Port of AP_Math/vector2.h + vector2.cpp's core algebra. CPP-006, slice 1.
//
// SLICE BOUNDARY: this lands the struct, equality, arithmetic operators,
// dot/cross, length/length_squared/limit_length, normalize/normalized,
// is_zero/is_nan/is_inf, angle (both overloads), rotate, offset_bearing,
// reflect/project/projected. Deliberately NOT in this slice: perpendicular,
// both closest_point overloads, the closest_distance_between_* family,
// segment_intersection, circle_segment_intersection, point_on_segment,
// tofloat/todouble. Those are more specialized geometry helpers (nav/mission
// code, not core vector algebra) - a legitimate follow-on slice, not a
// silent gap; tracked in the CPP-006 ticket notes.
//
// Only angle(const Vector2&) is defined in vector2.cpp rather than here -
// it's the one function in this slice with a bare double literal (M_PI on
// the antiparallel-vectors path). See scalar.hpp's file banner for why that
// matters and vector2.cpp's own banner for how it's compiled.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::math {

template <typename T>
struct Vector2 {
    T x, y;

    constexpr Vector2() : x(0), y(0) {}
    constexpr Vector2(T x0, T y0) : x(x0), y(y0) {}

    [[nodiscard]] bool operator==(const Vector2<T>& v) const {
        return is_equal(x, v.x) && is_equal(y, v.y);
    }
    [[nodiscard]] bool operator!=(const Vector2<T>& v) const {
        return !is_equal(x, v.x) || !is_equal(y, v.y);
    }

    [[nodiscard]] Vector2<T> operator-() const { return Vector2<T>(-x, -y); }
    [[nodiscard]] Vector2<T> operator+(const Vector2<T>& v) const { return Vector2<T>(x + v.x, y + v.y); }
    [[nodiscard]] Vector2<T> operator-(const Vector2<T>& v) const { return Vector2<T>(x - v.x, y - v.y); }
    [[nodiscard]] Vector2<T> operator*(T num) const { return Vector2<T>(x * num, y * num); }
    [[nodiscard]] Vector2<T> operator/(T num) const { return Vector2<T>(x / num, y / num); }

    Vector2<T>& operator+=(const Vector2<T>& v) { x += v.x; y += v.y; return *this; }
    Vector2<T>& operator-=(const Vector2<T>& v) { x -= v.x; y -= v.y; return *this; }
    Vector2<T>& operator*=(T num) { x *= num; y *= num; return *this; }
    Vector2<T>& operator/=(T num) { x /= num; y /= num; return *this; }

    // Dot product.
    [[nodiscard]] T operator*(const Vector2<T>& v) const { return x * v.x + y * v.y; }
    [[nodiscard]] T dot(const Vector2<T>& v) const { return *this * v; }

    // Cross product (scalar in 2D - the z-component of the 3D cross).
    [[nodiscard]] T operator%(const Vector2<T>& v) const { return x * v.y - y * v.x; }

    // Angle between this vector and v2, upstream `angle(const Vector2&)`:
    // 0 if parallel, pi if antiparallel. Defined in vector2.cpp - see the
    // file banner.
    [[nodiscard]] T angle(const Vector2<T>& v2) const;

    // This vector's own angle from the unit vector (1,0), -pi to pi.
    [[nodiscard]] T angle() const { return std::atan2(y, x); }

    [[nodiscard]] bool is_nan() const { return std::isnan(x) || std::isnan(y); }
    [[nodiscard]] bool is_inf() const { return std::isinf(x) || std::isinf(y); }

    // Generic (integral T): exact comparison, matching upstream's in-class
    // body. Overridden below for float/double via the free-function
    // is_zero's epsilon.
    [[nodiscard]] bool is_zero() const { return x == T(0) && y == T(0); }

    T& operator[](std::uint8_t i) { return (&x)[i]; }
    const T& operator[](std::uint8_t i) const { return (&x)[i]; }

    void zero() { x = y = T(0); }

    [[nodiscard]] T length_squared() const { return x * x + y * y; }
    [[nodiscard]] T length() const { return std::sqrt(x * x + y * y); }

    // Limits this vector's length in place. Returns true if it was limited.
    bool limit_length(T max_length) {
        const T len = length();
        if (len > max_length && is_positive(len)) {
            x *= (max_length / len);
            y *= (max_length / len);
            return true;
        }
        return false;
    }

    void normalize() { *this /= length(); }
    [[nodiscard]] Vector2<T> normalized() const { return *this / length(); }

    // Rotate in place by angle_rad radians.
    void rotate(T angle_rad) {
        const T cs = std::cos(angle_rad);
        const T sn = std::sin(angle_rad);
        const T rx = x * cs - y * sn;
        const T ry = x * sn + y * cs;
        x = rx;
        y = ry;
    }

    // Extrapolate position given bearing (degrees) and distance, in place.
    void offset_bearing(T bearing, T distance) {
        x += std::cos(radians(bearing)) * distance;
        y += std::sin(radians(bearing)) * distance;
    }

    void project(const Vector2<T>& v) { *this = v * (*this * v) / (v * v); }
    [[nodiscard]] Vector2<T> projected(const Vector2<T>& v) const { return v * (*this * v) / (v * v); }

    void reflect(const Vector2<T>& n) {
        const Vector2<T> orig(*this);
        project(n);
        *this = *this * T(2) - orig;
    }
};

// Float/double specializations use the epsilon-tolerant free-function
// is_zero rather than exact ==0, matching upstream's explicit
// template<> specializations in vector2.h.
template <>
inline bool Vector2<float>::is_zero() const {
    return math::is_zero(x) && math::is_zero(y);
}
template <>
inline bool Vector2<double>::is_zero() const {
    return math::is_zero(x) && math::is_zero(y);
}

using Vector2i = Vector2<std::int16_t>;
using Vector2ui = Vector2<std::uint16_t>;
using Vector2l = Vector2<std::int32_t>;
using Vector2ul = Vector2<std::uint32_t>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;

} // namespace fwcpp::math
