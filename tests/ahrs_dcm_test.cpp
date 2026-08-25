// Tests for fwcpp::ahrs::AhrsDcm (CPP-028 slice 1: pure gyro-integration
// DCM attitude core, no drift correction).

#include <cmath>
#include <limits>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/ahrs/ahrs_dcm.hpp>

using namespace fwcpp::ahrs;
using fwcpp::math::Matrix3f;
using fwcpp::math::Vector3f;

TEST_CASE("reset() from a level accel vector produces roll=0, pitch=0", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.reset(Vector3f(0.0f, 0.0f, -9.8f), false);

    REQUIRE(ahrs.roll == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(ahrs.pitch == Catch::Approx(0.0f).margin(1e-6f));
    REQUIRE(ahrs.dcm_matrix.a == Vector3f(1.0f, 0.0f, 0.0f));
    REQUIRE(ahrs.dcm_matrix.b == Vector3f(0.0f, 1.0f, 0.0f));
    REQUIRE(ahrs.dcm_matrix.c == Vector3f(0.0f, 0.0f, 1.0f));
}

TEST_CASE("reset() from a tilted accel vector matches the independent atan2 formula", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    const Vector3f accel(2.0f, -1.5f, -9.5f);
    ahrs.reset(accel, false);

    // Cross-check 1: the exact upstream formula, computed independently
    // here rather than read back from the class under test.
    const float expected_pitch = std::atan2(accel.x, std::sqrt(accel.y * accel.y + accel.z * accel.z));
    const float expected_roll = std::atan2(-accel.y, -accel.z);
    REQUIRE(ahrs.pitch == Catch::Approx(expected_pitch));
    REQUIRE(ahrs.roll == Catch::Approx(expected_roll));

    // Cross-check 2: round-trip through Matrix3::to_euler, a completely
    // different formula (asin/atan2 on matrix elements) than the from_euler
    // reset() used to build dcm_matrix in the first place.
    float roll_rt = 0.0f;
    float pitch_rt = 0.0f;
    float yaw_rt = 0.0f;
    ahrs.dcm_matrix.to_euler(&roll_rt, &pitch_rt, &yaw_rt);
    REQUIRE(roll_rt == Catch::Approx(expected_roll).margin(1e-5f));
    REQUIRE(pitch_rt == Catch::Approx(expected_pitch).margin(1e-5f));
    REQUIRE(yaw_rt == Catch::Approx(0.0f).margin(1e-5f));
}

TEST_CASE("reset() aligns flat when the accel vector is too small to trust", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.reset(Vector3f(1.0f, 1.0f, 1.0f), false); // length ~1.7, below the 5.0 trust threshold

    REQUIRE(ahrs.roll == 0.0f);
    REQUIRE(ahrs.pitch == 0.0f);
}

TEST_CASE("reset(recover_eulers=true) rebuilds from existing valid eulers, ignoring accel", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.roll = 0.3f;
    ahrs.pitch = -0.2f;
    ahrs.yaw = 1.0f;

    // A wildly different accel vector must be ignored - upstream's own
    // contract for the recover_eulers-and-valid-eulers branch.
    ahrs.reset(Vector3f(100.0f, 100.0f, 100.0f), true);

    Matrix3f expected;
    expected.from_euler(0.3f, -0.2f, 1.0f);
    REQUIRE(ahrs.dcm_matrix.a == expected.a);
    REQUIRE(ahrs.dcm_matrix.b == expected.b);
    REQUIRE(ahrs.dcm_matrix.c == expected.c);
    REQUIRE(ahrs.roll == 0.3f);
    REQUIRE(ahrs.pitch == -0.2f);
    REQUIRE(ahrs.yaw == 1.0f);
}

TEST_CASE("matrix_update rotates dcm_matrix identically to a direct Matrix3::rotate call", "[ahrs_dcm]") {
    // With omega_I/omega_P/omega_yaw_P all zero (this slice has no drift
    // correction), matrix_update()'s rotation step is mathematically
    // equivalent to rotating a fresh identity matrix by delta_angle
    // directly - cross-check against that.
    AhrsDcm ahrs;
    GyroSample sample;
    sample.delta_angle = Vector3f(0.001f, -0.002f, 0.003f);
    sample.dangle_dt = 0.01f;
    sample.gyro = sample.delta_angle / sample.dangle_dt;

    ahrs.matrix_update(sample);

    Matrix3f expected;
    expected.identity();
    expected.rotate(sample.delta_angle);

    REQUIRE(ahrs.dcm_matrix.a.x == Catch::Approx(expected.a.x).margin(1e-6f));
    REQUIRE(ahrs.dcm_matrix.a.y == Catch::Approx(expected.a.y).margin(1e-6f));
    REQUIRE(ahrs.dcm_matrix.a.z == Catch::Approx(expected.a.z).margin(1e-6f));
    REQUIRE(ahrs.dcm_matrix.b.x == Catch::Approx(expected.b.x).margin(1e-6f));
    REQUIRE(ahrs.dcm_matrix.b.y == Catch::Approx(expected.b.y).margin(1e-6f));
    REQUIRE(ahrs.dcm_matrix.b.z == Catch::Approx(expected.b.z).margin(1e-6f));
    REQUIRE(ahrs.dcm_matrix.c.x == Catch::Approx(expected.c.x).margin(1e-6f));
    REQUIRE(ahrs.dcm_matrix.c.y == Catch::Approx(expected.c.y).margin(1e-6f));
    REQUIRE(ahrs.dcm_matrix.c.z == Catch::Approx(expected.c.z).margin(1e-6f));

    // omega is re-derived from the raw gyro rate (no P terms), matching
    // upstream's matrix_update() final line.
    REQUIRE(ahrs.omega.x == Catch::Approx(sample.gyro.x));
    REQUIRE(ahrs.omega.y == Catch::Approx(sample.gyro.y));
    REQUIRE(ahrs.omega.z == Catch::Approx(sample.gyro.z));
}

TEST_CASE("matrix_update with dangle_dt<=0 skips integration but still refreshes omega", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    GyroSample sample;
    sample.delta_angle = Vector3f(1.0f, 1.0f, 1.0f); // would be a huge rotation if applied
    sample.dangle_dt = 0.0f;                         // ...but this marks "no valid sample"
    sample.gyro = Vector3f(0.05f, 0.0f, 0.0f);

    ahrs.matrix_update(sample);

    REQUIRE(ahrs.dcm_matrix.a == Vector3f(1.0f, 0.0f, 0.0f));
    REQUIRE(ahrs.dcm_matrix.b == Vector3f(0.0f, 1.0f, 0.0f));
    REQUIRE(ahrs.dcm_matrix.c == Vector3f(0.0f, 0.0f, 1.0f));
    REQUIRE(ahrs.omega == sample.gyro);
}

TEST_CASE("repeated update() dead-reckons constant angular velocity and stays orthogonal", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    const float omega_z = 0.1f; // rad/s
    const float dt = 0.001f;
    const int steps = 10000; // 10s -> total yaw ~1.0 rad

    GyroSample sample;
    sample.gyro = Vector3f(0.0f, 0.0f, omega_z);
    sample.dangle_dt = dt;

    for (int i = 0; i < steps; ++i) {
        sample.delta_angle = sample.gyro * dt;
        ahrs.update(sample);
    }

    REQUIRE(ahrs.yaw == Catch::Approx(1.0f).margin(0.01f));

    // This is normalize()'s actual job: keep the matrix orthonormal despite
    // 10000 rounds of numerical integration error.
    REQUIRE(ahrs.dcm_matrix.a.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(ahrs.dcm_matrix.b.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE(ahrs.dcm_matrix.c.length() == Catch::Approx(1.0f).margin(1e-4f));
    REQUIRE((ahrs.dcm_matrix.a * ahrs.dcm_matrix.b) == Catch::Approx(0.0f).margin(1e-4f));
    REQUIRE(ahrs.dcm_matrix.det() == Catch::Approx(1.0f).margin(1e-3f));
}

TEST_CASE("without normalize(), repeated matrix_update() measurably drifts off orthogonal", "[ahrs_dcm]") {
    // Same style of integration as the previous test, but calling
    // matrix_update() directly (skipping normalize()) with a rate/step
    // count chosen to make the drift clearly visible - demonstrating what
    // normalize() exists to prevent, not just that it compiles.
    AhrsDcm ahrs;
    const float omega_z = 0.5f;
    const float dt = 0.01f;
    const int steps = 2000;

    GyroSample sample;
    sample.gyro = Vector3f(0.0f, 0.0f, omega_z);
    sample.dangle_dt = dt;

    for (int i = 0; i < steps; ++i) {
        sample.delta_angle = sample.gyro * dt;
        ahrs.matrix_update(sample);
    }

    REQUIRE(std::fabs(ahrs.dcm_matrix.a.length() - 1.0f) > 1e-3f);
}

TEST_CASE("check_matrix() resets to identity when the matrix is NaN", "[ahrs_dcm]") {
    AhrsDcm ahrs; // roll/pitch/yaw default to 0 - a valid recovery target
    ahrs.dcm_matrix.a.x = std::numeric_limits<float>::quiet_NaN();

    ahrs.check_matrix();

    REQUIRE_FALSE(ahrs.dcm_matrix.is_nan());
    REQUIRE(ahrs.dcm_matrix.a == Vector3f(1.0f, 0.0f, 0.0f));
    REQUIRE(ahrs.dcm_matrix.b == Vector3f(0.0f, 1.0f, 0.0f));
    REQUIRE(ahrs.dcm_matrix.c == Vector3f(0.0f, 0.0f, 1.0f));
}

TEST_CASE("check_matrix() normalizes away a wildly out-of-range c.x", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.dcm_matrix.a = Vector3f(1.0f, 0.0f, 0.0f);
    ahrs.dcm_matrix.b = Vector3f(0.0f, 1.0f, 0.0f);
    ahrs.dcm_matrix.c = Vector3f(50.0f, 0.0f, 1.0f); // c.x is nowhere near (-1, 1)

    ahrs.check_matrix();

    REQUIRE_FALSE(ahrs.dcm_matrix.is_nan());
    REQUIRE(ahrs.dcm_matrix.c.x > -1.0f);
    REQUIRE(ahrs.dcm_matrix.c.x < 1.0f);
    // c is fully recomputed as a normalized a x b, not merely clamped.
    REQUIRE(ahrs.dcm_matrix.c.x == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(ahrs.dcm_matrix.c.y == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(ahrs.dcm_matrix.c.z == Catch::Approx(1.0f).margin(1e-5f));
}

TEST_CASE("check_matrix() leaves an already-valid matrix untouched", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.reset(Vector3f(1.0f, 2.0f, -9.5f), false);
    const Matrix3f before = ahrs.dcm_matrix;

    ahrs.check_matrix();

    REQUIRE(ahrs.dcm_matrix.a == before.a);
    REQUIRE(ahrs.dcm_matrix.b == before.b);
    REQUIRE(ahrs.dcm_matrix.c == before.c);
}

TEST_CASE("renorm's catastrophic-failure path resets rather than propagating garbage", "[ahrs_dcm]") {
    AhrsDcm ahrs; // roll/pitch/yaw default to 0
    // 'a' is degenerate (near-zero length): renorm_val = 1/length() blows
    // past the 1e6 sanity bound, so renorm() must return false and
    // normalize() must reset instead of dividing by ~0.
    ahrs.dcm_matrix.a = Vector3f(1.0e-8f, 0.0f, 0.0f);
    ahrs.dcm_matrix.b = Vector3f(0.0f, 1.0f, 0.0f);
    ahrs.dcm_matrix.c = Vector3f(0.0f, 0.0f, 1.0f);

    ahrs.normalize();

    REQUIRE_FALSE(ahrs.dcm_matrix.is_nan());
    REQUIRE(ahrs.dcm_matrix.a == Vector3f(1.0f, 0.0f, 0.0f));
    REQUIRE(ahrs.dcm_matrix.b == Vector3f(0.0f, 1.0f, 0.0f));
    REQUIRE(ahrs.dcm_matrix.c == Vector3f(0.0f, 0.0f, 1.0f));
}

TEST_CASE("renorm() itself reports the catastrophic failure via its return value", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    Vector3f result;
    REQUIRE_FALSE(ahrs.renorm(Vector3f(1.0e-8f, 0.0f, 0.0f), result));
    REQUIRE(ahrs.renorm(Vector3f(1.0f, 0.0f, 0.0f), result));
    REQUIRE(result == Vector3f(1.0f, 0.0f, 0.0f));
}

TEST_CASE("reset_gyro_drift zeroes the (currently always-zero) drift estimate", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.reset_gyro_drift(); // should not throw/crash even though omega_I is already zero in this slice
    SUCCEED();
}
