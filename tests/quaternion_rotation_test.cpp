// Tests for QuaternionT<T>::from_rotation(Rotation)/rotate(Rotation)
// (CPP-019 continuation).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/quaternion.hpp>

using namespace fwcpp::math;

namespace {
constexpr Rotation kSomeRotations[] = {
    Rotation::NONE, Rotation::YAW_45, Rotation::YAW_90, Rotation::YAW_180,
    Rotation::ROLL_180, Rotation::ROLL_90, Rotation::PITCH_90,
    Rotation::ROLL_90_PITCH_90, Rotation::ROLL_90_PITCH_68_YAW_293,
    Rotation::PITCH_7, Rotation::ROLL_45, Rotation::ROLL_315,
};
} // namespace

TEST_CASE("from_rotation(NONE) is the identity quaternion", "[quaternion][rotation]") {
    Quaternion q;
    q.from_rotation(Rotation::NONE);
    REQUIRE(q.q1 == 1.0f);
    REQUIRE(q.q2 == 0.0f);
    REQUIRE(q.q3 == 0.0f);
    REQUIRE(q.q4 == 0.0f);
}

TEST_CASE("from_rotation produces a unit quaternion for every sampled rotation", "[quaternion][rotation]") {
    for (Rotation r : kSomeRotations) {
        Quaternion q;
        q.from_rotation(r);
        REQUIRE(q.length() == Catch::Approx(1.0f).margin(1e-4f));
    }
}

TEST_CASE("Quaternion::from_rotation agrees with Matrix3::from_rotation on the same vector", "[quaternion][rotation]") {
    // Cross-check between two independently-transcribed constant tables
    // (quaternion.cpp's from_rotation switch, vector3.cpp's rotate switch
    // via Matrix3::from_rotation) - if either had a transcription error
    // for a given rotation, this would very likely catch it as a
    // disagreement rather than needing per-rotation hand-derived
    // expected values.
    Vector3f v(1.0f, 2.0f, 3.0f);
    for (Rotation r : kSomeRotations) {
        Quaternion q;
        q.from_rotation(r);
        Vector3f via_quat = q * v;

        Matrix3f m;
        m.from_rotation(r);
        Vector3f via_matrix = m * v;

        REQUIRE(via_quat.x == Catch::Approx(via_matrix.x).margin(1e-3f));
        REQUIRE(via_quat.y == Catch::Approx(via_matrix.y).margin(1e-3f));
        REQUIRE(via_quat.z == Catch::Approx(via_matrix.z).margin(1e-3f));
    }
}

TEST_CASE("Quaternion::rotate composes with an existing rotation", "[quaternion][rotation]") {
    Quaternion q; // identity
    q.rotate(Rotation::YAW_90);
    Vector3f v(1.0f, 0.0f, 0.0f);
    Vector3f rotated = q * v;
    // YAW_90 upstream: x,y swap with negation (x=-y_old, y=x_old) applied
    // to the VECTOR path; verify via the matrix cross-check instead of a
    // hand-derived sign to avoid duplicating that assumption here too.
    Matrix3f m;
    m.from_rotation(Rotation::YAW_90);
    Vector3f expected = m * v;
    REQUIRE(rotated.x == Catch::Approx(expected.x).margin(1e-4f));
    REQUIRE(rotated.y == Catch::Approx(expected.y).margin(1e-4f));
    REQUIRE(rotated.z == Catch::Approx(expected.z).margin(1e-4f));
}

TEST_CASE("unsupported rotation values (MAX, CUSTOM_*) leave the quaternion unchanged", "[quaternion][rotation]") {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    q.from_rotation(Rotation::MAX);
    REQUIRE(q.q1 == 0.5f);
    REQUIRE(q.q2 == 0.5f);
}

TEST_CASE("from_rotation works in double precision too", "[quaternion][rotation]") {
    QuaternionD q;
    q.from_rotation(Rotation::ROLL_180);
    REQUIRE(q.length() == Catch::Approx(1.0));
}
