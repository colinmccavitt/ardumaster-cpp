// Tests for Vector3<T>::rotate_inverse/row_times_mat/mul_rowcol (CPP-007
// closure). Rounds out the Matrix3-dependent slice of vector3.cpp that
// couldn't be ported until CPP-008 (Matrix3) landed.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/vector3.hpp>

using namespace fwcpp::math;

namespace {
constexpr Rotation kSomeRotations[] = {
    Rotation::NONE, Rotation::YAW_45, Rotation::YAW_90, Rotation::YAW_180,
    Rotation::ROLL_180, Rotation::ROLL_90, Rotation::PITCH_90,
    Rotation::ROLL_90_PITCH_90, Rotation::ROLL_90_PITCH_68_YAW_293,
    Rotation::PITCH_7, Rotation::ROLL_45, Rotation::ROLL_315,
};
} // namespace

TEST_CASE("rotate_inverse undoes rotate for every sampled rotation", "[vector3][rotate_inverse]") {
    // The defining property: rotate(r) then rotate_inverse(r) returns to
    // the original vector, for every rotation this port supports - a
    // systematic check instead of hand-deriving each inverse.
    for (Rotation r : kSomeRotations) {
        Vector3f v(1.0f, -2.0f, 3.0f);
        Vector3f original = v;
        v.rotate(r);
        v.rotate_inverse(r);
        REQUIRE(v.x == Catch::Approx(original.x).margin(1e-4f));
        REQUIRE(v.y == Catch::Approx(original.y).margin(1e-4f));
        REQUIRE(v.z == Catch::Approx(original.z).margin(1e-4f));
    }
}

TEST_CASE("rotate_inverse(NONE) leaves the vector unchanged", "[vector3][rotate_inverse]") {
    Vector3f v(1.0f, 2.0f, 3.0f);
    v.rotate_inverse(Rotation::NONE);
    REQUIRE(v == Vector3f(1.0f, 2.0f, 3.0f));
}

TEST_CASE("rotate_inverse(YAW_90) is the opposite x,y swap of rotate(YAW_90)", "[vector3][rotate_inverse]") {
    // rotate(YAW_90): x,y = -y,x (see rotations_test.cpp). The inverse
    // should be x,y = y,-x.
    Vector3f v(1.0f, 2.0f, 3.0f);
    v.rotate_inverse(Rotation::YAW_90);
    REQUIRE(v.x == Catch::Approx(2.0f));
    REQUIRE(v.y == Catch::Approx(-1.0f));
    REQUIRE(v.z == Catch::Approx(3.0f));
}

TEST_CASE("rotate_inverse works in double precision too", "[vector3][rotate_inverse]") {
    Vector3d v(1.0, -2.0, 3.0);
    Vector3d original = v;
    v.rotate(Rotation::ROLL_90_PITCH_68_YAW_293);
    v.rotate_inverse(Rotation::ROLL_90_PITCH_68_YAW_293);
    REQUIRE(v.x == Catch::Approx(original.x).margin(1e-6));
    REQUIRE(v.y == Catch::Approx(original.y).margin(1e-6));
    REQUIRE(v.z == Catch::Approx(original.z).margin(1e-6));
}

TEST_CASE("row_times_mat against the identity matrix is a no-op", "[vector3][row_times_mat]") {
    Vector3f v(1.0f, 2.0f, 3.0f);
    Matrix3f id;
    id.identity();
    Vector3f result = v.row_times_mat(id);
    REQUIRE(result.x == Catch::Approx(v.x));
    REQUIRE(result.y == Catch::Approx(v.y));
    REQUIRE(result.z == Catch::Approx(v.z));
}

TEST_CASE("row_times_mat agrees with the equivalent Matrix3::mul_transpose relationship", "[vector3][row_times_mat]") {
    // row_times_mat(m) dots *this against each COLUMN of m - equivalent to
    // multiplying by m's transpose (m^T * v, read as row-vector-on-the-left
    // convention). Cross-check against mul_transpose, which is already
    // covered by matrix3_test.cpp, instead of hand-deriving a fresh
    // expected vector here.
    Matrix3f m(1.0f, 2.0f, 3.0f,
               4.0f, 5.0f, 6.0f,
               7.0f, 8.0f, 9.0f);
    Vector3f v(1.0f, -1.0f, 2.0f);

    Vector3f via_row_times_mat = v.row_times_mat(m);
    Vector3f via_mul_transpose = m.mul_transpose(v);

    REQUIRE(via_row_times_mat.x == Catch::Approx(via_mul_transpose.x));
    REQUIRE(via_row_times_mat.y == Catch::Approx(via_mul_transpose.y));
    REQUIRE(via_row_times_mat.z == Catch::Approx(via_mul_transpose.z));
}

TEST_CASE("mul_rowcol builds the outer product of two vectors", "[vector3][mul_rowcol]") {
    Vector3f v1(1.0f, 2.0f, 3.0f);
    Vector3f v2(4.0f, 5.0f, 6.0f);
    Matrix3f m = v1.mul_rowcol(v2);

    REQUIRE(m.a.x == Catch::Approx(4.0f));  // v1.x * v2.x
    REQUIRE(m.a.y == Catch::Approx(5.0f));  // v1.x * v2.y
    REQUIRE(m.a.z == Catch::Approx(6.0f));  // v1.x * v2.z
    REQUIRE(m.b.x == Catch::Approx(8.0f));  // v1.y * v2.x
    REQUIRE(m.b.y == Catch::Approx(10.0f)); // v1.y * v2.y
    REQUIRE(m.b.z == Catch::Approx(12.0f)); // v1.y * v2.z
    REQUIRE(m.c.x == Catch::Approx(12.0f)); // v1.z * v2.x
    REQUIRE(m.c.y == Catch::Approx(15.0f)); // v1.z * v2.y
    REQUIRE(m.c.z == Catch::Approx(18.0f)); // v1.z * v2.z
}

TEST_CASE("mul_rowcol against a zero vector produces the zero matrix", "[vector3][mul_rowcol]") {
    Vector3f v1(1.0f, 2.0f, 3.0f);
    Vector3f zero;
    Matrix3f m = v1.mul_rowcol(zero);
    Matrix3f expected_zero(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    REQUIRE(m == expected_zero);
}
