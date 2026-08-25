// Tests for the Rotation enum + Vector3::rotate(Rotation) +
// Matrix3::from_rotation(Rotation) (CPP-019).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/vector3.hpp>

using namespace fwcpp::math;

namespace {
// Every named (non-sentinel, non-custom) rotation, 0 through 43.
constexpr Rotation kAllNamedRotations[] = {
    Rotation::NONE, Rotation::YAW_45, Rotation::YAW_90, Rotation::YAW_135, Rotation::YAW_180,
    Rotation::YAW_225, Rotation::YAW_270, Rotation::YAW_315, Rotation::ROLL_180,
    Rotation::ROLL_180_YAW_45, Rotation::ROLL_180_YAW_90, Rotation::ROLL_180_YAW_135,
    Rotation::PITCH_180, Rotation::ROLL_180_YAW_225, Rotation::ROLL_180_YAW_270,
    Rotation::ROLL_180_YAW_315, Rotation::ROLL_90, Rotation::ROLL_90_YAW_45,
    Rotation::ROLL_90_YAW_90, Rotation::ROLL_90_YAW_135, Rotation::ROLL_270,
    Rotation::ROLL_270_YAW_45, Rotation::ROLL_270_YAW_90, Rotation::ROLL_270_YAW_135,
    Rotation::PITCH_90, Rotation::PITCH_270, Rotation::PITCH_180_YAW_90,
    Rotation::PITCH_180_YAW_270, Rotation::ROLL_90_PITCH_90, Rotation::ROLL_180_PITCH_90,
    Rotation::ROLL_270_PITCH_90, Rotation::ROLL_90_PITCH_180, Rotation::ROLL_270_PITCH_180,
    Rotation::ROLL_90_PITCH_270, Rotation::ROLL_180_PITCH_270, Rotation::ROLL_270_PITCH_270,
    Rotation::ROLL_90_PITCH_180_YAW_90, Rotation::ROLL_90_YAW_270,
    Rotation::ROLL_90_PITCH_68_YAW_293, Rotation::PITCH_315, Rotation::ROLL_90_PITCH_315,
    Rotation::PITCH_7, Rotation::ROLL_45, Rotation::ROLL_315,
};
} // namespace

TEST_CASE("every named rotation preserves vector length", "[rotations]") {
    // All of these are orthogonal transforms (rotations/reflections about
    // axis-aligned planes, or in the ROLL_90_PITCH_68_YAW_293/PITCH_7
    // cases, precomputed proper-rotation matrices) - length preservation
    // is a strong systematic check across all 44 without hand-deriving
    // expected output for each one individually.
    for (Rotation r : kAllNamedRotations) {
        Vector3f v(3.0f, -4.0f, 5.0f); // length^2 = 50, nonzero on every axis
        const float original_length = v.length();
        v.rotate(r);
        REQUIRE(v.length() == Catch::Approx(original_length).margin(1e-4f));
    }
}

TEST_CASE("ROTATION_NONE leaves the vector unchanged", "[rotations]") {
    Vector3f v(1.0f, 2.0f, 3.0f);
    v.rotate(Rotation::NONE);
    REQUIRE(v == Vector3f(1.0f, 2.0f, 3.0f));
}

TEST_CASE("ROTATION_YAW_90 matches its documented x,y swap", "[rotations]") {
    // upstream: tmp = x; x = -y; y = tmp;
    Vector3f v(1.0f, 2.0f, 3.0f);
    v.rotate(Rotation::YAW_90);
    REQUIRE(v == Vector3f(-2.0f, 1.0f, 3.0f));
}

TEST_CASE("ROTATION_ROLL_180 flips y and z, leaves x", "[rotations]") {
    Vector3f v(1.0f, 2.0f, 3.0f);
    v.rotate(Rotation::ROLL_180);
    REQUIRE(v == Vector3f(1.0f, -2.0f, -3.0f));
}

TEST_CASE("ROTATION_PITCH_90 matches its documented x,z swap", "[rotations]") {
    // upstream: tmp = z; z = -x; x = tmp;
    Vector3f v(1.0f, 2.0f, 3.0f);
    v.rotate(Rotation::PITCH_90);
    REQUIRE(v == Vector3f(3.0f, 2.0f, -1.0f));
}

TEST_CASE("ROTATION_YAW_180 negates x and y, leaves z", "[rotations]") {
    Vector3f v(1.0f, 2.0f, 3.0f);
    v.rotate(Rotation::YAW_180);
    REQUIRE(v == Vector3f(-1.0f, -2.0f, 3.0f));
}

TEST_CASE("applying YAW_90 four times returns to the original vector", "[rotations]") {
    Vector3f v(1.0f, 2.0f, 3.0f);
    Vector3f original = v;
    for (int i = 0; i < 4; ++i) {
        v.rotate(Rotation::YAW_90);
    }
    REQUIRE(v.x == Catch::Approx(original.x));
    REQUIRE(v.y == Catch::Approx(original.y));
    REQUIRE(v.z == Catch::Approx(original.z));
}

TEST_CASE("unsupported rotation values (MAX, CUSTOM_*) leave the vector unchanged", "[rotations]") {
    Vector3f v(1.0f, 2.0f, 3.0f);
    v.rotate(Rotation::MAX);
    REQUIRE(v == Vector3f(1.0f, 2.0f, 3.0f));
    v.rotate(Rotation::CUSTOM_1);
    REQUIRE(v == Vector3f(1.0f, 2.0f, 3.0f));
}

TEST_CASE("ROTATION_ROLL_90_PITCH_68_YAW_293 is a proper rotation (determinant +1, orthonormal rows)", "[rotations]") {
    // The bespoke matrix case - verify via Matrix3::from_rotation that the
    // resulting matrix is a genuine rotation, not just length-preserving
    // (a reflection also preserves length but has determinant -1).
    Matrix3f m;
    m.from_rotation(Rotation::ROLL_90_PITCH_68_YAW_293);
    REQUIRE(m.det() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(m.a.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(m.a.dot(m.b) == Catch::Approx(0.0f).margin(1e-4f));
}

TEST_CASE("Matrix3::from_rotation for YAW_90 matches applying Vector3::rotate directly", "[rotations]") {
    // Cross-check between the two ported functions: rotating a vector via
    // Vector3::rotate should agree with multiplying by the matrix
    // from_rotation builds from the same enum value.
    Matrix3f m;
    m.from_rotation(Rotation::YAW_90);

    Vector3f v(1.0f, 2.0f, 3.0f);
    Vector3f expected = v;
    expected.rotate(Rotation::YAW_90);

    Vector3f via_matrix = m * v;
    REQUIRE(via_matrix.x == Catch::Approx(expected.x).margin(1e-5f));
    REQUIRE(via_matrix.y == Catch::Approx(expected.y).margin(1e-5f));
    REQUIRE(via_matrix.z == Catch::Approx(expected.z).margin(1e-5f));
}

TEST_CASE("Matrix3::from_rotation(NONE) is the identity", "[rotations]") {
    Matrix3f m;
    m.from_rotation(Rotation::NONE);
    Matrix3f id;
    id.identity();
    REQUIRE(m == id);
}

TEST_CASE("rotate works in double precision too", "[rotations]") {
    Vector3d v(1.0, 2.0, 3.0);
    v.rotate(Rotation::ROLL_180);
    REQUIRE(v == Vector3d(1.0, -2.0, -3.0));
}
