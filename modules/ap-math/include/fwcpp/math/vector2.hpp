#pragma once

// Port of AP_Math/vector2.h + vector2.cpp. CPP-006 - complete: core algebra
// (struct, arithmetic, dot/cross, length/normalize, is_zero/is_nan/is_inf,
// angle, rotate, offset_bearing, reflect/project) plus the geometry helper
// family added in a follow-on pass (perpendicular, both closest_point
// overloads, the closest_distance_between_* family, segment_intersection,
// circle_segment_intersection, point_on_segment, tofloat/todouble).
//
// Only angle(const Vector2&) is defined in vector2.cpp rather than here -
// it's the one function in this file with a bare double literal (M_PI on
// the antiparallel-vectors path). See scalar.hpp's file banner for why that
// matters and vector2.cpp's own banner for how it's compiled. Everything
// added in the geometry-helper pass has no such literal (FLT_EPSILON is
// already explicitly typed, same as the core-algebra slice) and stays
// header-only.

#include <algorithm>
#include <cfloat>
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

    // Given a position delta and a velocity v1, returns the vector
    // perpendicular to v1 that maximizes distance from pos_delta's origin.
    [[nodiscard]] static Vector2<T> perpendicular(const Vector2<T>& pos_delta, const Vector2<T>& v1) {
        const Vector2<T> perp1(-v1.y, v1.x);
        const Vector2<T> perp2(v1.y, -v1.x);
        const T d1 = perp1 * pos_delta;
        const T d2 = perp2 * pos_delta;
        return (d1 > d2) ? perp1 : perp2;
    }

    // Closest point to p on the segment (v, w).
    [[nodiscard]] static Vector2<T> closest_point(const Vector2<T>& p, const Vector2<T>& v, const Vector2<T>& w) {
        const T l2 = (v - w).length_squared();
        if (l2 < T(FLT_EPSILON)) {
            return v; // v == w
        }
        const T t = ((p - v) * (w - v)) / l2;
        if (t <= T(0)) {
            return v;
        }
        if (t >= T(1)) {
            return w;
        }
        return v + (w - v) * t;
    }

    // Closest point to p on the segment (0, w) - simplification of the
    // three-argument overload with v=(0,0).
    [[nodiscard]] static Vector2<T> closest_point(const Vector2<T>& p, const Vector2<T>& w) {
        const T l2 = w.length_squared();
        if (l2 < T(FLT_EPSILON)) {
            return w;
        }
        const T t = (p * w) / l2;
        if (t <= T(0)) {
            return Vector2<T>(0, 0);
        }
        if (t >= T(1)) {
            return w;
        }
        return w * t;
    }

    // w is a line segment from the origin; p is a point. Squared closest
    // distance between the radial and p.
    [[nodiscard]] static T closest_distance_between_radial_and_point_squared(const Vector2<T>& w, const Vector2<T>& p) {
        const Vector2<T> closest = closest_point(p, w);
        return (closest - p).length_squared();
    }

    [[nodiscard]] static T closest_distance_between_radial_and_point(const Vector2<T>& w, const Vector2<T>& p) {
        return std::sqrt(closest_distance_between_radial_and_point_squared(w, p));
    }

    // w1/w2 define a line segment; p is a point. Squared closest distance
    // between the segment and p.
    [[nodiscard]] static T closest_distance_between_line_and_point_squared(
        const Vector2<T>& w1, const Vector2<T>& w2, const Vector2<T>& p) {
        return closest_distance_between_radial_and_point_squared(w2 - w1, p - w1);
    }

    [[nodiscard]] static T closest_distance_between_line_and_point(
        const Vector2<T>& w1, const Vector2<T>& w2, const Vector2<T>& p) {
        return std::sqrt(closest_distance_between_line_and_point_squared(w1, w2, p));
    }

    // a1->a2 and b1->b2 define two line segments. Squared closest distance
    // between them - approximated by checking each segment's endpoints
    // against the other segment (matches upstream's own approximation,
    // documented there as such, not a full segment-segment closest-point
    // solution).
    [[nodiscard]] static T closest_distance_between_lines_squared(
        const Vector2<T>& a1, const Vector2<T>& a2, const Vector2<T>& b1, const Vector2<T>& b2) {
        const T dist1 = closest_distance_between_line_and_point_squared(b1, b2, a1);
        const T dist2 = closest_distance_between_line_and_point_squared(b1, b2, a2);
        const T dist3 = closest_distance_between_line_and_point_squared(a1, a2, b1);
        const T dist4 = closest_distance_between_line_and_point_squared(a1, a2, b2);
        const T m1 = std::min(dist1, dist2);
        const T m2 = std::min(dist3, dist4);
        return std::min(m1, m2);
    }

    // Find the intersection of two line segments (as rays extended per t,u
    // parameterization, matching upstream: t unclamped-nonnegative since
    // (p, p+r) is treated as a ray, u clamped to [0,1] since (q, q+s) is a
    // genuine segment). Returns false if collinear/parallel/non-intersecting.
    [[nodiscard]] static bool segment_intersection(
        const Vector2<T>& seg1_start, const Vector2<T>& seg1_end,
        const Vector2<T>& seg2_start, const Vector2<T>& seg2_end, Vector2<T>& intersection) {
        const Vector2<T> r1 = seg1_end - seg1_start;
        const Vector2<T> r2 = seg2_end - seg2_start;
        const Vector2<T> ss2_ss1 = seg2_start - seg1_start;
        const T r1xr2 = r1 % r2;
        const T q_pxr = ss2_ss1 % r1;
        if (math::is_zero(r1xr2)) {
            return false;
        }
        const T t = (ss2_ss1 % r2) / r1xr2;
        const T u = q_pxr / r1xr2;
        if (u >= T(0) && u <= T(1) && t >= T(0) && t <= T(1)) {
            intersection = seg1_start + (r1 * t);
            return true;
        }
        return false;
    }

    // Intersection of a line segment with a circle, closest to seg_start.
    // Adapted from stackoverflow.com/questions/1073336.
    [[nodiscard]] static bool circle_segment_intersection(
        const Vector2<T>& seg_start, const Vector2<T>& seg_end,
        const Vector2<T>& circle_center, T radius, Vector2<T>& intersection) {
        const Vector2<T> seg_start_local = seg_start - circle_center;
        const Vector2<T> seg_end_minus_start = seg_end - seg_start;

        const T a = seg_end_minus_start.x * seg_end_minus_start.x + seg_end_minus_start.y * seg_end_minus_start.y;
        const T b = T(2) * ((seg_end_minus_start.x * seg_start_local.x) + (seg_end_minus_start.y * seg_start_local.y));
        const T c = seg_start_local.x * seg_start_local.x + seg_start_local.y * seg_start_local.y - radius * radius;

        if (math::is_zero(a) || std::isnan(a) || std::isnan(b) || std::isnan(c)) {
            return false;
        }

        const T delta = b * b - (T(4) * a * c);
        if (std::isnan(delta)) {
            return false;
        }
        if (delta < T(0)) {
            return false;
        }

        const T delta_sqrt = std::sqrt(delta);
        const T t1 = (-b + delta_sqrt) / (T(2) * a);
        const T t2 = (-b - delta_sqrt) / (T(2) * a);

        if (t1 >= T(0) && t1 <= T(1)) {
            intersection = seg_start + (seg_end_minus_start * t1);
            return true;
        }
        if (t2 >= T(0) && t2 <= T(1)) {
            intersection = seg_start + (seg_end_minus_start * t2);
            return true;
        }
        return false;
    }

    // Whether point falls on the segment seg_start->seg_end (collinear and
    // within the bounding box) - used for endpoint-inclusive checks
    // segment_intersection alone doesn't answer.
    [[nodiscard]] static bool point_on_segment(
        const Vector2<T>& point, const Vector2<T>& seg_start, const Vector2<T>& seg_end) {
        const T expected_run = seg_end.x - seg_start.x;
        const T intersection_run = point.x - seg_start.x;
        if (math::is_zero(expected_run)) {
            if (std::fabs(intersection_run) > FLT_EPSILON) {
                return false;
            }
        } else {
            const T expected_slope = (seg_end.y - seg_start.y) / expected_run;
            const T intersection_slope = (point.y - seg_start.y) / intersection_run;
            if (std::fabs(expected_slope - intersection_slope) > FLT_EPSILON) {
                return false;
            }
        }
        if (seg_start.x < seg_end.x) {
            if (point.x < seg_start.x || point.x > seg_end.x) {
                return false;
            }
        } else {
            if (point.x < seg_end.x || point.x > seg_start.x) {
                return false;
            }
        }
        if (seg_start.y < seg_end.y) {
            if (point.y < seg_start.y || point.y > seg_end.y) {
                return false;
            }
        } else {
            if (point.y < seg_end.y || point.y > seg_start.y) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] Vector2<float> tofloat() const { return Vector2<float>(float(x), float(y)); }
    [[nodiscard]] Vector2<double> todouble() const { return Vector2<double>(x, y); }
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
