#pragma once

// Port of AP_Math/quaternion.h + quaternion.cpp. CPP-009, slice 1.
//
// Upstream is QuaternionT<T> but only ever typedef'd for float/double
// (Quaternion, QuaternionD) - no integer instantiation exists, unlike
// Vector2/Vector3/Matrix3. is_zero() below is therefore NOT split into a
// generic-exact + float/double-epsilon pair the way those types are; it
// always uses the epsilon-based free-function is_zero, matching upstream's
// single (non-specialized) definition.
//
// SLICE BOUNDARY: struct, is_nan/is_zero/zero/initialise, length/
// length_squared, normalize, is_unit_length, inverse/invert, operator[],
// operator*(Quaternion) [composition], operator*(Vector3) [body rotation],
// operator*=, from_euler(roll,pitch,yaw)+from_euler(Vector3), get_euler_
// roll/pitch/yaw, to_euler, from_rotation_matrix, rotation_matrix (both
// Matrix3f and Matrix3d overloads, independent of T - matches upstream),
// todouble/tofloat.
//
// Deliberately NOT in this slice: from_axis_angle (+_fast variants),
// to_axis_angle, from_vector312/to_vector312, from_angular_velocity,
// rotate_fast, angular_difference, roll_pitch_difference, earth_to_body,
// operator/. Tracked in CPP-009's notes.
//
// from_rotation(Rotation)/rotate(Rotation) added by CPP-019 once the
// Rotation enum existed. from_rotation is a precomputed constant table
// (comment upstream: "the constants below can be calculated ... from
// Matrix3f m; m.from_rotation(rotation); Quaternion q; q.from_rotation_matrix(m);"
// - i.e. these are cached results of a computation this port could also
// just run, but upstream hardcodes them and this port reproduces the
// hardcoded values exactly rather than deriving them, so any transcription
// slip in either the constants or the derivation shows up as a genuine
// difference, not a self-consistent alternative). Declared here, DEFINED
// in quaternion.cpp - the constants are bare non-exact literals, same
// compiled-.cpp treatment as scalar.cpp's wrap_* family and vector3.cpp's
// rotate(Rotation).
//
// LITERAL SAFETY: from_euler's `roll*0.5` etc use a bare 0.5 - exactly
// representable in both float and double, so -fsingle-precision-constant's
// float-vs-double parsing makes no difference here (same reasoning already
// used for wrap_360's 360.0/36000.0 literals). is_unit_length's 1E-3
// tolerance IS affected by the flag (0.001 is not exact in either format),
// but the function's own doc comment already calls this "somewhat greater
// than sqrt(FLT_EPSILON)" - a loose heuristic threshold, not a value with
// D-003-style downstream consequences - so the ~1e-10 difference between
// the float- and double-rounded constant is immaterial and this stays
// header-only rather than getting the compiled-.cpp treatment.
//
// normalize()'s zero-quaternion path reports through fwcpp::InternalError
// (CPP-005), matching upstream's
// `INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control)` there but via
// the explicit non-singleton channel rather than AP::internalerror().

#include <cmath>
#include <cstdint>

#include <fwcpp/internal_error.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::math {

template <typename T>
struct QuaternionT {
    T q1, q2, q3, q4; // w, x, y, z

    constexpr QuaternionT() : q1(1), q2(0), q3(0), q4(0) {}
    constexpr QuaternionT(T q1_, T q2_, T q3_, T q4_) : q1(q1_), q2(q2_), q3(q3_), q4(q4_) {}
    explicit QuaternionT(const T q[4]) : q1(q[0]), q2(q[1]), q3(q[2]), q4(q[3]) {}

    [[nodiscard]] bool is_nan() const {
        return std::isnan(q1) || std::isnan(q2) || std::isnan(q3) || std::isnan(q4);
    }

    // Always the epsilon-based free-function is_zero - see file banner.
    [[nodiscard]] bool is_zero() const {
        return math::is_zero(q1) && math::is_zero(q2) && math::is_zero(q3) && math::is_zero(q4);
    }

    // [0,0,0,0] - an INVALID quaternion (zero magnitude, no rotation it can
    // represent). Not the identity rotation; see initialise() for that.
    void zero() { q1 = q2 = q3 = q4 = T(0); }

    // The identity rotation (roll=pitch=yaw=0).
    void initialise() { q1 = T(1); q2 = q3 = q4 = T(0); }

    T& operator[](std::uint8_t i) { return (&q1)[i]; }
    const T& operator[](std::uint8_t i) const { return (&q1)[i]; }

    [[nodiscard]] T length_squared() const { return q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4; }
    [[nodiscard]] T length() const { return std::sqrt(length_squared()); }

    // Leaves [0,0,0,0] unchanged on the zero-magnitude path, reporting
    // through `err` if non-null - see file banner.
    void normalize(InternalError* err = nullptr, std::uint16_t line = 0) {
        const T mag = length();
        if (!math::is_zero(mag)) {
            const T inv = T(1) / mag;
            q1 *= inv;
            q2 *= inv;
            q3 *= inv;
            q4 *= inv;
        } else if (err != nullptr) {
            err->record(InternalErrorCode::flow_of_control, line);
        }
    }

    [[nodiscard]] bool is_unit_length() const {
        return std::fabs(length_squared() - T(1)) < T(1e-3);
    }

    [[nodiscard]] QuaternionT<T> inverse() const { return QuaternionT<T>(q1, -q2, -q3, -q4); }
    void invert() { q2 = -q2; q3 = -q3; q4 = -q4; }

    // Quaternion composition (Hamilton product).
    [[nodiscard]] QuaternionT<T> operator*(const QuaternionT<T>& v) const {
        const T w1 = q1, x1 = q2, y1 = q3, z1 = q4;
        const T w2 = v.q1, x2 = v.q2, y2 = v.q3, z2 = v.q4;
        return QuaternionT<T>(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2);
    }

    QuaternionT<T>& operator*=(const QuaternionT<T>& v) {
        *this = *this * v;
        return *this;
    }

    // Rotate a vector by this quaternion. Equivalent to (but cheaper than)
    // converting to a rotation matrix first - see upstream's own comment on
    // the operation count.
    [[nodiscard]] Vector3<T> operator*(const Vector3<T>& v) const {
        Vector3<T> ret = v;
        T uv[3] = {q3 * v.z - q4 * v.y, q4 * v.x - q2 * v.z, q2 * v.y - q3 * v.x};
        uv[0] += uv[0];
        uv[1] += uv[1];
        uv[2] += uv[2];
        ret.x += q1 * uv[0] + q3 * uv[2] - q4 * uv[1];
        ret.y += q1 * uv[1] + q4 * uv[0] - q2 * uv[2];
        ret.z += q1 * uv[2] + q2 * uv[1] - q3 * uv[0];
        return ret;
    }

    void from_euler(T roll, T pitch, T yaw) {
        const T cr2 = std::cos(roll * T(0.5));
        const T cp2 = std::cos(pitch * T(0.5));
        const T cy2 = std::cos(yaw * T(0.5));
        const T sr2 = std::sin(roll * T(0.5));
        const T sp2 = std::sin(pitch * T(0.5));
        const T sy2 = std::sin(yaw * T(0.5));

        q1 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
        q2 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
        q3 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
        q4 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
    }
    void from_euler(const Vector3<T>& v) { from_euler(v.x, v.y, v.z); }

    [[nodiscard]] T get_euler_roll() const {
        return std::atan2(T(2) * (q1 * q2 + q3 * q4), T(1) - T(2) * (q2 * q2 + q3 * q3));
    }
    [[nodiscard]] T get_euler_pitch() const {
        return safe_asin(T(2) * (q1 * q3 - q4 * q2));
    }
    [[nodiscard]] T get_euler_yaw() const {
        return std::atan2(T(2) * (q1 * q4 + q2 * q3), T(1) - T(2) * (q3 * q3 + q4 * q4));
    }

    // Both overloads exist regardless of T, matching rotation_matrix's
    // pattern above - output precision is a caller choice.
    void to_euler(float& roll, float& pitch, float& yaw) const {
        roll = static_cast<float>(get_euler_roll());
        pitch = static_cast<float>(get_euler_pitch());
        yaw = static_cast<float>(get_euler_yaw());
    }
    void to_euler(Vector3f& rpy) const { to_euler(rpy.x, rpy.y, rpy.z); }
    void to_euler(double& roll, double& pitch, double& yaw) const {
        roll = static_cast<double>(get_euler_roll());
        pitch = static_cast<double>(get_euler_pitch());
        yaw = static_cast<double>(get_euler_yaw());
    }
    void to_euler(Vector3d& rpy) const { to_euler(rpy.x, rpy.y, rpy.z); }

    // Both overloads exist on every QuaternionT<T> regardless of T, exactly
    // as upstream declares them (output precision is a caller choice, not
    // tied to this quaternion's own T).
    void rotation_matrix(Matrix3f& m) const {
        const T q3q3 = q3 * q3, q3q4 = q3 * q4, q2q2 = q2 * q2, q2q3 = q2 * q3,
                q2q4 = q2 * q4, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4, q4q4 = q4 * q4;
        m.a.x = 1.0f - 2.0f * (q3q3 + q4q4);
        m.a.y = 2.0f * (q2q3 - q1q4);
        m.a.z = 2.0f * (q2q4 + q1q3);
        m.b.x = 2.0f * (q2q3 + q1q4);
        m.b.y = 1.0f - 2.0f * (q2q2 + q4q4);
        m.b.z = 2.0f * (q3q4 - q1q2);
        m.c.x = 2.0f * (q2q4 - q1q3);
        m.c.y = 2.0f * (q3q4 + q1q2);
        m.c.z = 1.0f - 2.0f * (q2q2 + q3q3);
    }
    void rotation_matrix(Matrix3d& m) const {
        const T q3q3 = q3 * q3, q3q4 = q3 * q4, q2q2 = q2 * q2, q2q3 = q2 * q3,
                q2q4 = q2 * q4, q1q2 = q1 * q2, q1q3 = q1 * q3, q1q4 = q1 * q4, q4q4 = q4 * q4;
        m.a.x = 1.0 - 2.0 * (q3q3 + q4q4);
        m.a.y = 2.0 * (q2q3 - q1q4);
        m.a.z = 2.0 * (q2q4 + q1q3);
        m.b.x = 2.0 * (q2q3 + q1q4);
        m.b.y = 1.0 - 2.0 * (q2q2 + q4q4);
        m.b.z = 2.0 * (q3q4 - q1q2);
        m.c.x = 2.0 * (q2q4 - q1q3);
        m.c.y = 2.0 * (q3q4 + q1q2);
        m.c.z = 1.0 - 2.0 * (q2q2 + q3q3);
    }

    void from_rotation_matrix(const Matrix3<T>& m) {
        const T& m00 = m.a.x; const T& m11 = m.b.y; const T& m22 = m.c.z;
        const T& m10 = m.b.x; const T& m01 = m.a.y; const T& m20 = m.c.x;
        const T& m02 = m.a.z; const T& m21 = m.c.y; const T& m12 = m.b.z;
        T& qw = q1; T& qx = q2; T& qy = q3; T& qz = q4;

        const T tr = m00 + m11 + m22;

        if (tr > T(0)) {
            const T s = std::sqrt(tr + T(1)) * T(2);
            qw = T(0.25) * s;
            qx = (m21 - m12) / s;
            qy = (m02 - m20) / s;
            qz = (m10 - m01) / s;
        } else if (m00 > m11 && m00 > m22) {
            const T s = std::sqrt(T(1) + m00 - m11 - m22) * T(2);
            qw = (m21 - m12) / s;
            qx = T(0.25) * s;
            qy = (m01 + m10) / s;
            qz = (m02 + m20) / s;
        } else if (m11 > m22) {
            const T s = std::sqrt(T(1) + m11 - m00 - m22) * T(2);
            qw = (m02 - m20) / s;
            qx = (m01 + m10) / s;
            qy = T(0.25) * s;
            qz = (m12 + m21) / s;
        } else {
            const T s = std::sqrt(T(1) + m22 - m00 - m11) * T(2);
            qw = (m10 - m01) / s;
            qx = (m02 + m20) / s;
            qy = (m12 + m21) / s;
            qz = T(0.25) * s;
        }
    }

    [[nodiscard]] QuaternionT<double> todouble() const { return QuaternionT<double>(q1, q2, q3, q4); }
    [[nodiscard]] QuaternionT<float> tofloat() const { return QuaternionT<float>(q1, q2, q3, q4); }

    // Defined in quaternion.cpp - see file banner.
    void from_rotation(Rotation rotation);

    // Compose this quaternion with the quaternion for `rotation`. No
    // literal ambiguity in this body itself (from_rotation is where the
    // constants live), so this stays header-inline.
    void rotate(Rotation rotation) {
        QuaternionT<T> q_from_rot;
        q_from_rot.from_rotation(rotation);
        *this *= q_from_rot;
    }
};

using Quaternion = QuaternionT<float>;
using QuaternionD = QuaternionT<double>;

} // namespace fwcpp::math
