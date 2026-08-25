#pragma once

// Port of AP_Math/matrix3.h + matrix3.cpp. CPP-008 - now complete,
// including from_rotation(Rotation), added once CPP-019 landed
// Vector3::rotate(Rotation).
//
// Nothing in matrix3.cpp has a bare ambiguous double literal - from_euler,
// to_euler, from_euler312, to_euler312, from_axis_angle, rotate, normalize,
// det, inverse all use only already-typed values (T-typed locals, FLT_-safe
// constants, or explicitly suffixed literals like 0.5f/1.0f). Entirely
// header-only, unlike ap-math scalar/vector2 which needed compiled .cpp
// sources for their bare-M_PI cases.
//
// IMPLEMENTATION NOTE, not a behavior change: upstream's zero() reinterprets
// the whole object as raw bytes via memset(this, 0, sizeof(*this)), which
// only works because IEEE-754 +0.0 is the all-zero bit pattern and the
// struct has no padding surprises for float/double T. This port calls
// a.zero()/b.zero()/c.zero() instead - same result, doesn't lean on a
// layout assumption the type system can't check. ADR-0012's stance against
// unsafe reinterpretation (already applied to ap-param's is_sentinel and to
// Vector3::xy() being deferred) applies here too, and this one has a
// trivial safe equivalent so there's nothing to defer.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::math {

template <typename T>
struct Matrix3 {
    Vector3<T> a, b, c; // rows

    constexpr Matrix3() = default;
    constexpr Matrix3(const Vector3<T>& a0, const Vector3<T>& b0, const Vector3<T>& c0)
        : a(a0), b(b0), c(c0) {}
    constexpr Matrix3(T ax, T ay, T az, T bx, T by, T bz, T cx, T cy, T cz)
        : a(ax, ay, az), b(bx, by, bz), c(cx, cy, cz) {}

    void operator()(const Vector3<T>& a0, const Vector3<T>& b0, const Vector3<T>& c0) {
        a = a0; b = b0; c = c0;
    }

    [[nodiscard]] bool operator==(const Matrix3<T>& m) const { return a == m.a && b == m.b && c == m.c; }
    [[nodiscard]] bool operator!=(const Matrix3<T>& m) const { return a != m.a || b != m.b || c != m.c; }

    [[nodiscard]] Matrix3<T> operator-() const { return Matrix3<T>(-a, -b, -c); }
    [[nodiscard]] Matrix3<T> operator+(const Matrix3<T>& m) const { return Matrix3<T>(a + m.a, b + m.b, c + m.c); }
    Matrix3<T>& operator+=(const Matrix3<T>& m) { return *this = *this + m; }
    [[nodiscard]] Matrix3<T> operator-(const Matrix3<T>& m) const { return Matrix3<T>(a - m.a, b - m.b, c - m.c); }
    Matrix3<T>& operator-=(const Matrix3<T>& m) { return *this = *this - m; }
    [[nodiscard]] Matrix3<T> operator*(T num) const { return Matrix3<T>(a * num, b * num, c * num); }
    Matrix3<T>& operator*=(T num) { return *this = *this * num; }
    [[nodiscard]] Matrix3<T> operator/(T num) const { return Matrix3<T>(a / num, b / num, c / num); }
    Matrix3<T>& operator/=(T num) { return *this = *this / num; }

    Vector3<T>& operator[](std::uint8_t i) { return (&a)[i]; }
    const Vector3<T>& operator[](std::uint8_t i) const { return (&a)[i]; }

    // Matrix * column vector.
    [[nodiscard]] Vector3<T> operator*(const Vector3<T>& v) const {
        return Vector3<T>(a.x * v.x + a.y * v.y + a.z * v.z,
                           b.x * v.x + b.y * v.y + b.z * v.z,
                           c.x * v.x + c.y * v.y + c.z * v.z);
    }

    // Transpose * column vector, without materializing the transpose.
    [[nodiscard]] Vector3<T> mul_transpose(const Vector3<T>& v) const {
        return Vector3<T>(a.x * v.x + b.x * v.y + c.x * v.z,
                           a.y * v.x + b.y * v.y + c.y * v.z,
                           a.z * v.x + b.z * v.y + c.z * v.z);
    }

    // Matrix * vector, keeping only the xy components of the result.
    [[nodiscard]] Vector2<T> mulXY(const Vector3<T>& v) const {
        return Vector2<T>(a.x * v.x + a.y * v.y + a.z * v.z,
                           b.x * v.x + b.y * v.y + b.z * v.z);
    }

    [[nodiscard]] Vector3<T> colx() const { return Vector3<T>(a.x, b.x, c.x); }
    [[nodiscard]] Vector3<T> coly() const { return Vector3<T>(a.y, b.y, c.y); }
    [[nodiscard]] Vector3<T> colz() const { return Vector3<T>(a.z, b.z, c.z); }

    [[nodiscard]] Matrix3<T> operator*(const Matrix3<T>& m) const {
        return Matrix3<T>(
            Vector3<T>(a.x * m.a.x + a.y * m.b.x + a.z * m.c.x,
                       a.x * m.a.y + a.y * m.b.y + a.z * m.c.y,
                       a.x * m.a.z + a.y * m.b.z + a.z * m.c.z),
            Vector3<T>(b.x * m.a.x + b.y * m.b.x + b.z * m.c.x,
                       b.x * m.a.y + b.y * m.b.y + b.z * m.c.y,
                       b.x * m.a.z + b.y * m.b.z + b.z * m.c.z),
            Vector3<T>(c.x * m.a.x + c.y * m.b.x + c.z * m.c.x,
                       c.x * m.a.y + c.y * m.b.y + c.z * m.c.y,
                       c.x * m.a.z + c.y * m.b.z + c.z * m.c.z));
    }
    Matrix3<T>& operator*=(const Matrix3<T>& m) { return *this = *this * m; }

    [[nodiscard]] Matrix3<T> transposed() const {
        return Matrix3<T>(Vector3<T>(a.x, b.x, c.x), Vector3<T>(a.y, b.y, c.y), Vector3<T>(a.z, b.z, c.z));
    }
    void transpose() { *this = transposed(); }

    [[nodiscard]] T det() const {
        return a.x * (b.y * c.z - b.z * c.y)
             + a.y * (b.z * c.x - b.x * c.z)
             + a.z * (b.x * c.y - b.y * c.x);
    }

    // Leaves inv unmodified and returns false if singular - upstream's own
    // contract, unchanged.
    bool inverse(Matrix3<T>& inv) const {
        const T d = det();
        if (math::is_zero(d)) {
            return false;
        }
        inv.a.x = (b.y * c.z - c.y * b.z) / d;
        inv.a.y = (a.z * c.y - a.y * c.z) / d;
        inv.a.z = (a.y * b.z - a.z * b.y) / d;
        inv.b.x = (b.z * c.x - b.x * c.z) / d;
        inv.b.y = (a.x * c.z - a.z * c.x) / d;
        inv.b.z = (b.x * a.z - a.x * b.z) / d;
        inv.c.x = (b.x * c.y - c.x * b.y) / d;
        inv.c.y = (c.x * a.y - a.x * c.y) / d;
        inv.c.z = (a.x * b.y - b.x * a.y) / d;
        return true;
    }

    bool invert() {
        Matrix3<T> inv;
        if (!inverse(inv)) {
            return false;
        }
        *this = inv;
        return true;
    }

    void zero() { a.zero(); b.zero(); c.zero(); }
    void identity() {
        zero();
        a.x = b.y = c.z = T(1);
    }

    [[nodiscard]] bool is_nan() const { return a.is_nan() || b.is_nan() || c.is_nan(); }

    // Rotation matrix from Euler angles, 321 convention (roll about x, then
    // pitch about y, then yaw about z, applied in that order).
    void from_euler(T roll, T pitch, T yaw) {
        const T cp = std::cos(pitch);
        const T sp = std::sin(pitch);
        const T sr = std::sin(roll);
        const T cr = std::cos(roll);
        const T sy = std::sin(yaw);
        const T cy = std::cos(yaw);

        a.x = cp * cy;
        a.y = (sr * sp * cy) - (cr * sy);
        a.z = (cr * sp * cy) + (sr * sy);
        b.x = cp * sy;
        b.y = (sr * sp * sy) + (cr * cy);
        b.z = (cr * sp * sy) - (sr * cy);
        c.x = -sp;
        c.y = sr * cp;
        c.z = cr * cp;
    }

    // Inverse of from_euler. Any of the three out-params may be null to
    // skip that component - matches upstream's own optional-pointer
    // interface rather than returning a Vector3 unconditionally.
    void to_euler(T* roll, T* pitch, T* yaw) const {
        if (pitch != nullptr) {
            *pitch = -safe_asin(c.x);
        }
        if (roll != nullptr) {
            *roll = std::atan2(c.y, c.z);
        }
        if (yaw != nullptr) {
            *yaw = std::atan2(b.x, a.x);
        }
    }

    // Euler angles, 312 convention, returned as (roll, pitch, yaw).
    [[nodiscard]] Vector3<T> to_euler312() const {
        return Vector3<T>(std::asin(c.y), std::atan2(-c.x, c.z), std::atan2(-a.y, b.y));
    }

    void from_euler312(T roll, T pitch, T yaw) {
        const T c3 = std::cos(pitch);
        const T s3 = std::sin(pitch);
        const T s2 = std::sin(roll);
        const T c2 = std::cos(roll);
        const T s1 = std::sin(yaw);
        const T c1 = std::cos(yaw);

        a.x = c1 * c3 - s1 * s2 * s3;
        b.y = c1 * c2;
        c.z = c3 * c2;
        a.y = -c2 * s1;
        a.z = s3 * c1 + c3 * s2 * s1;
        b.x = c3 * s1 + s3 * s2 * c1;
        b.z = s1 * s3 - s2 * c1 * c3;
        c.x = -s3 * c2;
        c.y = s2;
    }

    // Apply an additional rotation from a body-frame gyro vector (rad, one
    // integration step's worth - caller has already multiplied rate by dt).
    // This is the DCM integration step AP_AHRS_DCM's matrix_update uses.
    void rotate(const Vector3<T>& g) {
        *this += Matrix3<T>(
            a.y * g.z - a.z * g.y, a.z * g.x - a.x * g.z, a.x * g.y - a.y * g.x,
            b.y * g.z - b.z * g.y, b.z * g.x - b.x * g.z, b.x * g.y - b.y * g.x,
            c.y * g.z - c.z * g.y, c.z * g.x - c.x * g.z, c.x * g.y - c.y * g.x);
    }

    // Re-orthonormalize a rotation matrix that's drifted from numerical
    // integration error. This is the DCM renormalization step.
    void normalize() {
        const T error = a * b;
        const Vector3<T> t0 = a - (b * (T(0.5) * error));
        const Vector3<T> t1 = b - (a * (T(0.5) * error));
        const Vector3<T> t2 = t0 % t1;
        a = t0 * (T(1) / t0.length());
        b = t1 * (T(1) / t1.length());
        c = t2 * (T(1) / t2.length());
    }

    // Rotation matrix for a standard (45-degree-increment) rotation.
    // Matches upstream exactly: rotate each basis row, then transpose.
    void from_rotation(Rotation rotation) {
        a = Vector3<T>(T(1), T(0), T(0));
        b = Vector3<T>(T(0), T(1), T(0));
        c = Vector3<T>(T(0), T(0), T(1));
        a.rotate(rotation);
        b.rotate(rotation);
        c.rotate(rotation);
        transpose();
    }

    // Rotation matrix for rotation about axis v by angle theta (Rodrigues'
    // formula). v need not be pre-normalized - this normalizes it.
    void from_axis_angle(const Vector3<T>& v, T theta) {
        const T cs = std::cos(theta);
        const T sn = std::sin(theta);
        const T t = T(1) - cs;
        const Vector3<T> n = v.normalized();
        const T x = n.x;
        const T y = n.y;
        const T z = n.z;

        a.x = t * x * x + cs;
        a.y = t * x * y - z * sn;
        a.z = t * x * z + y * sn;
        b.x = t * x * y + z * sn;
        b.y = t * y * y + cs;
        b.z = t * y * z - x * sn;
        c.x = t * x * z - y * sn;
        c.y = t * y * z + x * sn;
        c.z = t * z * z + cs;
    }

    [[nodiscard]] Matrix3<double> todouble() const { return Matrix3<double>(a.todouble(), b.todouble(), c.todouble()); }
    [[nodiscard]] Matrix3<float> tofloat() const { return Matrix3<float>(a.tofloat(), b.tofloat(), c.tofloat()); }
};

using Matrix3i = Matrix3<std::int16_t>;
using Matrix3ui = Matrix3<std::uint16_t>;
using Matrix3l = Matrix3<std::int32_t>;
using Matrix3ul = Matrix3<std::uint32_t>;
using Matrix3f = Matrix3<float>;
using Matrix3d = Matrix3<double>;

} // namespace fwcpp::math
