// Tests for fwcpp::ahrs::AhrsDcm. CPP-028 slice 1 (pure gyro-integration
// DCM attitude core, no drift correction) tests are above the slice-2
// marker below and are unmodified. Slice-2 tests (YAW drift correction -
// yaw_error_compass/use_compass/drift_correction_yaw/p_gain/yaw_gain) are
// appended after it.

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

// ===========================================================================
// CPP-028 slice 2: YAW drift correction (drift_correction_yaw() and
// everything it calls). Everything above this marker is slice 1 and is
// unmodified.
// ===========================================================================

TEST_CASE("yaw_error_compass returns ~0 when the compass field agrees with the current heading",
          "[ahrs_dcm]") {
    AhrsDcm ahrs; // identity dcm_matrix, yaw=0
    CompassSample compass;
    compass.field = Vector3f(1.0f, 0.0f, 0.0f); // "north" field, no declination
    compass.declination_rad = 0.0f;

    REQUIRE(ahrs.yaw_error_compass(compass) == Catch::Approx(0.0f).margin(1e-6f));
}

TEST_CASE("yaw_error_compass returns a real, nonzero sin-proportional value when they disagree",
          "[ahrs_dcm]") {
    AhrsDcm ahrs; // identity dcm_matrix, yaw=0
    CompassSample compass;
    // Field points "east" while attitude/mag_earth_ default assume "north" -
    // a 90-degree earth-frame heading error.
    compass.field = Vector3f(0.0f, -1.0f, 0.0f);
    compass.declination_rad = 0.0f;

    REQUIRE(ahrs.yaw_error_compass(compass) == Catch::Approx(1.0f).margin(1e-6f));
}

TEST_CASE("yaw_error_compass returns 0 for a field with no usable horizontal component", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    CompassSample compass;
    compass.field = Vector3f(0.0f, 0.0f, 5.0f); // purely vertical - mulXY(field) is (0,0)
    REQUIRE(ahrs.yaw_error_compass(compass) == 0.0f);
}

TEST_CASE("yaw_error_compass recomputes mag_earth_ when declination actually changes", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    CompassSample compass;
    compass.field = Vector3f(1.0f, 0.0f, 0.0f);
    compass.declination_rad = 0.0f;
    REQUIRE(ahrs.yaw_error_compass(compass) == Catch::Approx(0.0f).margin(1e-6f));

    // Same field, but now the earth field itself is declared 90 degrees off
    // true north - the cached mag_earth_ must be recomputed, not stale.
    compass.declination_rad = fwcpp::math::radians(90.0f);
    REQUIRE(ahrs.yaw_error_compass(compass) == Catch::Approx(1.0f).margin(1e-5f));
}

TEST_CASE("use_compass returns false immediately when the compass isn't healthy", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    CompassSample compass;
    compass.healthy = false;
    GpsSample gps;
    gps.has_fix = true;
    REQUIRE_FALSE(ahrs.use_compass(compass, gps, true, true, 0.0f, 0));
}

TEST_CASE("use_compass favors the compass when there's no alternative", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    CompassSample compass;
    compass.healthy = true;
    GpsSample gps;

    // Not flying forward - no GPS-course alternative regardless of GPS state.
    gps.has_fix = true;
    REQUIRE(ahrs.use_compass(compass, gps, /*fly_forward=*/false, true, 0.0f, 0));

    // Flying forward but no GPS fix - still no alternative.
    REQUIRE(ahrs.use_compass(compass, gps, true, /*gps_use_enabled=*/false, 0.0f, 0));
    gps.has_fix = false;
    REQUIRE(ahrs.use_compass(compass, gps, true, true, 0.0f, 0));
}

TEST_CASE("use_compass favors the compass when ground speed is below GPS_SPEED_MIN", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    CompassSample compass;
    compass.healthy = true;
    GpsSample gps;
    gps.has_fix = true;
    gps.ground_speed_ms = 1.0f; // below kGpsSpeedMinMs (3 m/s)

    REQUIRE(ahrs.use_compass(compass, gps, true, true, 0.0f, 0));
}

TEST_CASE("use_compass falls back to GPS only after 2s of sustained heading disagreement", "[ahrs_dcm]") {
    AhrsDcm ahrs; // yaw = 0
    CompassSample compass;
    compass.healthy = true;
    GpsSample gps;
    gps.has_fix = true;
    gps.ground_speed_ms = 10.0f; // >= GPS_SPEED_MIN

    // Call 1: compass and GPS agree (small error) - establishes the
    // "last consistent" timestamp at now_ms=1000.
    gps.ground_course_deg = 5.0f;
    REQUIRE(ahrs.use_compass(compass, gps, true, true, /*wind_speed_ms=*/1.0f, 1000));

    // Call 2: large disagreement (>45 deg) and low wind (<80% of ground
    // speed), but only 500ms after the last consistent reading - still
    // within the 2-second latch, so the compass is still trusted.
    gps.ground_course_deg = 170.0f;
    REQUIRE(ahrs.use_compass(compass, gps, true, true, 1.0f, 1500));

    // Call 3: same disagreement, now 2.5s after the last consistent
    // reading - past the latch, so use_compass finally hands off to GPS.
    REQUIRE_FALSE(ahrs.use_compass(compass, gps, true, true, 1.0f, 3500));
}

TEST_CASE("p_gain: below 50 deg/s returns 1.0", "[ahrs_dcm]") {
    REQUIRE(AhrsDcm::p_gain(fwcpp::math::radians(10.0f)) == Catch::Approx(1.0f));
}

TEST_CASE("p_gain: above 500 deg/s returns 10.0", "[ahrs_dcm]") {
    REQUIRE(AhrsDcm::p_gain(fwcpp::math::radians(600.0f)) == Catch::Approx(10.0f));
}

TEST_CASE("p_gain: linear between 50 and 500 deg/s", "[ahrs_dcm]") {
    REQUIRE(AhrsDcm::p_gain(fwcpp::math::radians(250.0f)) == Catch::Approx(5.0f));
}

TEST_CASE("yaw_gain: at or below 4.0 m/s/s of horizontal accel, scales down from 0.9", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.accel_ef = Vector3f(0.0f, 0.0f, 0.0f);
    REQUIRE(ahrs.yaw_gain() == Catch::Approx(0.9f));

    ahrs.accel_ef = Vector3f(3.0f, 0.0f, 0.0f);
    REQUIRE(ahrs.yaw_gain() == Catch::Approx(0.2f * 1.5f));
}

TEST_CASE("yaw_gain: above 4.0 m/s/s of horizontal accel, clamps to 0.1", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.accel_ef = Vector3f(3.0f, 4.0f, 100.0f); // xy() length = 5, z ignored
    REQUIRE(ahrs.yaw_gain() == Catch::Approx(0.1f));
}

TEST_CASE("use_fast_gains: true only while disarmed and within 20s of last_startup_ms_", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.reset(Vector3f(0.0f, 0.0f, -9.8f), false, /*now_ms=*/500);

    REQUIRE(ahrs.use_fast_gains(false, 600));       // disarmed, 100ms later
    REQUIRE_FALSE(ahrs.use_fast_gains(true, 600));  // armed - never fast
    REQUIRE_FALSE(ahrs.use_fast_gains(false, 500 + 20000)); // window expired
}

TEST_CASE("drift_correction_yaw: first-ever compass reading resets DCM to the compass heading",
          "[ahrs_dcm]") {
    AhrsDcm ahrs; // identity dcm_matrix, roll=pitch=yaw=0
    REQUIRE_FALSE(ahrs.yaw_initialised());

    CompassSample compass;
    compass.healthy = true;
    compass.last_update_usec = 1000;
    compass.field = Vector3f(0.0f, -1.0f, 0.0f); // -> heading = atan2(1, 0) = +90 deg
    compass.declination_rad = 0.0f;
    GpsSample gps; // has_fix = false: no GPS alternative, use_compass() trivially true

    // armed=true sidesteps use_fast_gains()'s *8 multiplier so the
    // cross-check formula below stays simple - covered on its own below.
    ahrs.drift_correction_yaw(compass, gps, /*fly_forward=*/true, /*armed=*/true,
                               /*gps_use_enabled=*/false, /*wind_speed_ms=*/0.0f, /*now_ms=*/1000);

    REQUIRE(ahrs.yaw_initialised());

    const float expected_heading = std::atan2(1.0f, 0.0f);
    Matrix3f expected;
    expected.from_euler(0.0f, 0.0f, expected_heading);
    REQUIRE(ahrs.dcm_matrix.a.x == Catch::Approx(expected.a.x).margin(1e-5f));
    REQUIRE(ahrs.dcm_matrix.a.y == Catch::Approx(expected.a.y).margin(1e-5f));
    REQUIRE(ahrs.dcm_matrix.b.x == Catch::Approx(expected.b.x).margin(1e-5f));
    REQUIRE(ahrs.dcm_matrix.b.y == Catch::Approx(expected.b.y).margin(1e-5f));

    // Cross-check omega_yaw_p_.z via the class's own already-independently-
    // tested building blocks, recomposed here rather than a hand-derived
    // second formula - yaw_error_compass is const and side-effect-free
    // against the (now-reset) dcm_matrix, so calling it again is safe.
    const float spin_rate = ahrs.omega.length();
    const float expected_yaw_error = ahrs.yaw_error_compass(compass);
    const float expected_error_z = ahrs.dcm_matrix.c.z * expected_yaw_error;
    const float expected_omega_yaw_p_z =
        expected_error_z * AhrsDcm::p_gain(spin_rate) * 0.2f /* default kp_yaw */ * ahrs.yaw_gain();
    REQUIRE(ahrs.omega_yaw_p().z == Catch::Approx(expected_omega_yaw_p_z));

    // The yaw integral term also picked up this tick's contribution.
    const float yaw_deltat = 0.001f; // (1000 - 0) compass usec -> 1ms
    REQUIRE(ahrs.omega_i().z == Catch::Approx(expected_error_z * kKiYaw * yaw_deltat));
}

TEST_CASE("drift_correction_yaw: use_fast_gains applies its 8x multiplier", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    ahrs.reset(Vector3f(0.0f, 0.0f, -9.8f), false, /*now_ms=*/500);

    CompassSample compass;
    compass.healthy = true;
    compass.last_update_usec = 1000;
    compass.field = Vector3f(0.0f, -1.0f, 0.0f);
    GpsSample gps;

    // Disarmed, 100ms after reset - well within the 20s fast-gains window.
    ahrs.drift_correction_yaw(compass, gps, true, /*armed=*/false, false, 0.0f, /*now_ms=*/600);

    const float spin_rate = ahrs.omega.length();
    const float expected_yaw_error = ahrs.yaw_error_compass(compass);
    const float expected_error_z = ahrs.dcm_matrix.c.z * expected_yaw_error;
    const float expected_omega_yaw_p_z =
        expected_error_z * AhrsDcm::p_gain(spin_rate) * 0.2f * ahrs.yaw_gain() * 8.0f;
    REQUIRE(ahrs.omega_yaw_p().z == Catch::Approx(expected_omega_yaw_p_z));
}

TEST_CASE("drift_correction_yaw: kp_yaw below AP_AHRS_YAW_P_MIN is clamped up before use", "[ahrs_dcm]") {
    AhrsDcm ahrs(0.01f); // below kYawPMin (0.05)

    CompassSample compass;
    compass.healthy = true;
    compass.last_update_usec = 1000;
    compass.field = Vector3f(0.0f, -1.0f, 0.0f);
    GpsSample gps;

    ahrs.drift_correction_yaw(compass, gps, true, /*armed=*/true, false, 0.0f, 1000);

    const float spin_rate = ahrs.omega.length();
    const float expected_yaw_error = ahrs.yaw_error_compass(compass);
    const float expected_error_z = ahrs.dcm_matrix.c.z * expected_yaw_error;
    // Uses kYawPMin, NOT the 0.01f constructor value.
    const float expected_omega_yaw_p_z =
        expected_error_z * AhrsDcm::p_gain(spin_rate) * kYawPMin * ahrs.yaw_gain();
    REQUIRE(ahrs.omega_yaw_p().z == Catch::Approx(expected_omega_yaw_p_z));
}

TEST_CASE("drift_correction_yaw: with no new yaw source, omega_yaw_p_ decays by 0.97", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    CompassSample compass;
    compass.healthy = true;
    compass.last_update_usec = 1000;
    compass.field = Vector3f(0.0f, -1.0f, 0.0f);
    GpsSample gps; // no GPS fix - compass is the only source

    // Call 1: first-ever compass reading resets DCM to match it exactly -
    // the residual error (and so omega_yaw_p_.z) right after that reset is
    // ~0, not a useful nonzero baseline.
    ahrs.drift_correction_yaw(compass, gps, true, true, false, 0.0f, 1000);
    REQUIRE(ahrs.yaw_initialised());

    // Call 2: a genuinely new compass sample (different last_update_usec)
    // whose field no longer matches the now-fixed heading - this is a real
    // residual error, producing a real nonzero omega_yaw_p_.z to decay.
    compass.last_update_usec = 2000;
    compass.field = Vector3f(1.0f, 0.0f, 0.0f);
    ahrs.drift_correction_yaw(compass, gps, true, true, false, 0.0f, 2000);
    const float v1 = ahrs.omega_yaw_p().z;
    REQUIRE(std::fabs(v1) > 1e-4f);

    // Call 3: same CompassSample as call 2 (last_update_usec unchanged)
    // and still no GPS - no new yaw information at all this tick.
    ahrs.drift_correction_yaw(compass, gps, true, true, false, 0.0f, 3000);
    REQUIRE(ahrs.omega_yaw_p().z == Catch::Approx(v1 * 0.97f));
}

TEST_CASE("drift_correction_yaw: GPS reset condition 3 - large error at high speed", "[ahrs_dcm]") {
    AhrsDcm ahrs; // yaw = 0
    CompassSample compass;
    compass.healthy = false; // force the GPS branch every time
    GpsSample gps;
    gps.has_fix = true;

    // Call 1: establish have_initial_yaw via condition 1 (never had yaw
    // before), with a course that agrees with yaw=0 so it doesn't also
    // trip condition 3.
    gps.ground_speed_ms = 10.0f;
    gps.ground_course_deg = 0.0f;
    gps.last_fix_time_ms = 1000;
    ahrs.drift_correction_yaw(compass, gps, true, true, true, 0.0f, 1000);
    REQUIRE(ahrs.yaw_initialised());

    // Call 2: yaw_deltat = 1s (not the 20s-stale condition 2), speed well
    // above 3*GPS_SPEED_MIN, and a course 170 degrees away from yaw=0 (far
    // past the 60-degree/1.047 rad threshold) - only condition 3 applies.
    gps.ground_course_deg = 170.0f;
    gps.last_fix_time_ms = 2000;
    ahrs.drift_correction_yaw(compass, gps, true, true, true, 0.0f, 2000);

    const float expected_course_rad = fwcpp::math::radians(170.0f);
    Matrix3f expected;
    expected.from_euler(0.0f, 0.0f, expected_course_rad);
    REQUIRE(ahrs.dcm_matrix.a.x == Catch::Approx(expected.a.x).margin(1e-5f));
    REQUIRE(ahrs.dcm_matrix.a.y == Catch::Approx(expected.a.y).margin(1e-5f));
    REQUIRE(ahrs.dcm_matrix.b.x == Catch::Approx(expected.b.x).margin(1e-5f));
    REQUIRE(ahrs.dcm_matrix.b.y == Catch::Approx(expected.b.y).margin(1e-5f));

    // Condition-3 resets force yaw_error to 0 for this tick.
    REQUIRE(ahrs.omega_yaw_p().z == Catch::Approx(0.0f).margin(1e-6f));
}

// ===========================================================================
// CPP-028 slice 3: ROLL/PITCH drift correction (drift_correction_accel(),
// accumulate_accel(), ra_delayed(), should_correct_centrifugal()).
// Everything above this marker is slice 1/2 and is unmodified.
// ===========================================================================

TEST_CASE("should_correct_centrifugal: always true for Plane", "[ahrs_dcm]") {
    REQUIRE(AhrsDcm::should_correct_centrifugal());
}

TEST_CASE("ra_delayed: first call passes through, then delays by one sample", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    const Vector3f v1(1.0f, 2.0f, 3.0f);
    // previous buffer starts exactly zero -> passthrough on the first call
    REQUIRE(ahrs.ra_delayed(v1) == v1);

    const Vector3f v2(4.0f, 5.0f, 6.0f);
    REQUIRE(ahrs.ra_delayed(v2) == v1); // now returns the PREVIOUS value

    const Vector3f v3(7.0f, 8.0f, 9.0f);
    REQUIRE(ahrs.ra_delayed(v3) == v2);
}

TEST_CASE("accumulate_accel integrates accel*deltat across ticks (invariant to tick count)", "[ahrs_dcm]") {
    // Two AHRS instances fed the SAME total accel-time history via a
    // DIFFERENT number of accumulate_accel() calls (2 coarse ticks vs 10
    // fine ticks, same total elapsed time and same accel) must reach an
    // identical fused correction - proof that accumulate_accel() really
    // integrates accel*deltat (upstream: `_ra_sum[i] += accel_ef * deltat`)
    // rather than, say, overwriting a snapshot each call.
    AhrsDcm coarse;
    AhrsDcm fine;

    GpsSample gps;
    gps.has_fix = true;
    gps.has_3d_fix = true;
    gps.num_sats = 10;
    gps.velocity_ned = Vector3f(0.0f, 0.0f, 0.0f); // stationary for the bootstrap fix
    CompassSample compass; // unhealthy by default - keeps use_compass() false

    Vector3f accel_reading(0.0f, 0.0f, -9.80665f); // exactly vertical throughout - no accel-tilt error source

    auto feed_coarse = [&](float total_dt) {
        AccelSample a;
        a.accel = accel_reading;
        a.delta_velocity_dt = total_dt / 2.0f;
        a.delta_velocity = a.accel * a.delta_velocity_dt;
        for (int i = 0; i < 2; ++i) {
            coarse.accumulate_accel(a, a.delta_velocity_dt);
        }
    };
    auto feed_fine = [&](float total_dt) {
        AccelSample a;
        a.accel = accel_reading;
        a.delta_velocity_dt = total_dt / 10.0f;
        a.delta_velocity = a.accel * a.delta_velocity_dt;
        for (int i = 0; i < 10; ++i) {
            fine.accumulate_accel(a, a.delta_velocity_dt);
        }
    };

    feed_coarse(0.1f);
    feed_fine(0.1f);
    gps.last_fix_time_ms = 1000;
    coarse.drift_correction_accel(compass, gps, false, true, true, Vector3f(), 0.0f, true, true, 1000);
    fine.drift_correction_accel(compass, gps, false, true, true, Vector3f(), 0.0f, true, true, 1000);
    // Both calls above just bootstrap (first-ever GPS fix) - nothing to
    // compare yet.

    // The real error source: GPS velocity changes between the bootstrap
    // and this cycle (the accel history stays exactly vertical the whole
    // time) - this is what shifts GA_e and produces a real, nonzero error
    // for both instances to (identically) react to.
    gps.velocity_ned = Vector3f(0.0f, 3.0f, 0.0f);
    feed_coarse(0.1f);
    feed_fine(0.1f);
    gps.last_fix_time_ms = 2000;
    coarse.drift_correction_accel(compass, gps, false, true, true, Vector3f(), 0.0f, true, true, 2000);
    fine.drift_correction_accel(compass, gps, false, true, true, Vector3f(), 0.0f, true, true, 2000);

    REQUIRE(coarse.omega_p().x == Catch::Approx(fine.omega_p().x).margin(1e-5f));
    REQUIRE(coarse.omega_p().y == Catch::Approx(fine.omega_p().y).margin(1e-5f));
    REQUIRE(coarse.omega_p().z == Catch::Approx(fine.omega_p().z).margin(1e-5f));
    // Sanity: a real, nonzero correction happened (not a degenerate 0==0
    // pass) - the Y-axis GPS velocity change lands the error on X (see
    // the next test's doc comment for the cross-product rule why).
    REQUIRE(std::fabs(coarse.omega_p().x) > 1e-4f);
}

TEST_CASE("drift_correction_accel: a GPS velocity change produces a correctly-signed nonzero omega_p_",
          "[ahrs_dcm]") {
    AhrsDcm ahrs; // identity dcm_matrix, roll=pitch=yaw=0
    GpsSample gps;
    gps.has_fix = true;
    gps.has_3d_fix = true;
    gps.num_sats = 10;
    CompassSample compass; // unhealthy - keeps use_compass() false, error.z left untouched

    AccelSample accel;
    accel.accel = Vector3f(0.0f, 0.0f, -9.80665f); // perfectly level & stationary accel history
    accel.delta_velocity_dt = 0.01f;
    accel.delta_velocity = accel.accel * accel.delta_velocity_dt;

    // Bootstrap: stationary GPS velocity, sets last_velocity_ = (0,0,0).
    gps.velocity_ned = Vector3f(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 10; ++i) {
        ahrs.accumulate_accel(accel, 0.01f);
    }
    gps.last_fix_time_ms = 1000;
    ahrs.drift_correction_accel(compass, gps, false, true, true, Vector3f(), 0.0f, true, true, 1000);
    REQUIRE(ahrs.omega_p().is_zero()); // no correction produced on the bootstrap pass

    // Real fusion cycle: GPS velocity jumps to (0, 3, 0) while the accel
    // history stayed exactly vertical - this vdelta is what shifts GA_e
    // off (0,0,-1) and produces a real error, independent of any accel tilt.
    gps.velocity_ned = Vector3f(0.0f, 3.0f, 0.0f);
    for (int i = 0; i < 10; ++i) {
        ahrs.accumulate_accel(accel, 0.01f);
    }
    gps.last_fix_time_ms = 2000;
    ahrs.drift_correction_accel(compass, gps, false, true, true, Vector3f(), 0.0f, true, true, 2000);

    // Independent hand-derived cross-check. ra_deltat_ entering THIS call
    // is 0.2s, not 0.1s - the bootstrap call above does NOT reset
    // ra_deltat_/ra_sum_ (only a full, non-bootstrap successful pass does),
    // so the first 0.1s (accumulated before the bootstrap call) is still
    // there, plus the 0.1s accumulated since:
    //   ra_scale = 1/(ra_deltat*g) = 1/(0.2*9.80665)
    //   GA_b (accel exactly vertical throughout - ra_sum_ = accel*ra_deltat_
    //     always scales back to exactly accel/g regardless of ra_deltat_'s
    //     actual value; first-ever ra_delayed call passes through
    //     unchanged) = (0,0,-1) after scale+normalize.
    //   vdelta = (velocity - last_velocity) * (gps_gain * ra_scale)
    //          = (0,3,0) * ra_scale (gps_gain defaults to 1.0)
    //   GA_e = normalize((0,0,-1) + vdelta)
    //   error = GA_b x GA_e - note this lands on the X axis (not Y): for
    //     ga_b=(0,0,-1), cross((0,0,-1), (kx,ky,-1)) = (ky, -kx, 0) - a
    //     vdelta purely in Y (ky!=0, kx=0, as here) produces error on X.
    const float ra_scale = 1.0f / (0.2f * 9.80665f);
    const Vector3f ga_b(0.0f, 0.0f, -1.0f);
    Vector3f ga_e = Vector3f(0.0f, 0.0f, -1.0f) + Vector3f(0.0f, 3.0f, 0.0f) * ra_scale;
    ga_e.normalize();
    const Vector3f expected_error = ga_b % ga_e;
    const float expected_omega_p_x = expected_error.x * AhrsDcm::p_gain(0.0f) * 0.2f; // default kp_, armed=true (no *8)

    REQUIRE(ahrs.omega_p().x == Catch::Approx(expected_omega_p_x).margin(1e-4f));
    REQUIRE(ahrs.omega_p().y == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(ahrs.omega_p().z == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(ahrs.omega_p().x > 0.0f); // a real, correctly-signed nonzero correction
}

TEST_CASE("drift_correction_accel: dead-reckoning fallback with no GPS but a real airspeed change",
          "[ahrs_dcm]") {
    AhrsDcm ahrs;
    GpsSample gps; // has_fix stays false throughout - forces the no-GPS branch
    CompassSample compass; // unhealthy - keeps use_compass() false

    AccelSample accel;
    accel.accel = Vector3f(0.0f, 0.0f, -9.80665f); // perfectly level accel history throughout
    accel.delta_velocity_dt = 0.01f;
    accel.delta_velocity = accel.accel * accel.delta_velocity_dt;

    // Accumulate enough time (>0.2s) BEFORE the very first call - the
    // no-GPS branch's own `ra_deltat_ < 0.2f` guard applies even to the
    // bootstrap call.
    for (int i = 0; i < 25; ++i) {
        ahrs.accumulate_accel(accel, 0.01f);
    }

    // Bootstrap: airspeed_tas=15, wind=0 -> velocity = colx()*15 = (15,0,0).
    ahrs.drift_correction_accel(compass, gps, /*fly_forward=*/true, /*armed=*/true, /*gps_use_enabled=*/true,
                                 Vector3f(), /*airspeed_tas=*/15.0f, true, true, 1000);
    REQUIRE(ahrs.omega_p().is_zero()); // bootstrap pass produces no correction
    REQUIRE(ahrs.last_airspeed_tas() == 0.0f); // never touched by the no-GPS branch

    // Real fusion cycle: airspeed_tas changes to 20 -> velocity = (20,0,0),
    // a real vdelta relative to the bootstrap's (15,0,0) last_velocity_,
    // with no GPS involved at all.
    ahrs.drift_correction_accel(compass, gps, true, true, true, Vector3f(), /*airspeed_tas=*/20.0f, true, true, 2000);

    REQUIRE(ahrs.last_airspeed_tas() == 0.0f); // still untouched - no-GPS branch never writes it
    REQUIRE(ahrs.omega_p().x == Catch::Approx(0.0f).margin(1e-5f));
    REQUIRE(ahrs.omega_p().y < 0.0f); // real, nonzero, correctly-signed correction
    REQUIRE(ahrs.omega_p().z == Catch::Approx(0.0f).margin(1e-5f));
}

TEST_CASE("drift_correction_accel: omega_i_ batches for 5 seconds then folds in, clamped", "[ahrs_dcm]") {
    AhrsDcm ahrs;
    GpsSample gps;
    gps.has_fix = true;
    gps.has_3d_fix = true;
    gps.num_sats = 10;
    gps.velocity_ned = Vector3f(0.0f, 0.0f, 0.0f); // constant -> vdelta always 0
    CompassSample compass; // unhealthy - keeps use_compass() false

    AccelSample accel;
    // Deliberately large, constant tilt: a big roll/pitch error every
    // cycle, chosen so the raw accumulated integrator sum vastly exceeds
    // the tiny default max_gyro_drift_rad_s bound, proving the clamp
    // actually engages rather than just passing the raw sum through.
    accel.accel = Vector3f(2.0f, 0.0f, -9.8f);
    accel.delta_velocity_dt = 0.05f;
    accel.delta_velocity = accel.accel * accel.delta_velocity_dt;

    auto run_cycle = [&](std::uint32_t fix_time_ms) {
        for (int i = 0; i < 10; ++i) {
            ahrs.accumulate_accel(accel, 0.05f); // 10 * 0.05 = 0.5s
        }
        gps.last_fix_time_ms = fix_time_ms;
        ahrs.drift_correction_accel(compass, gps, false, true, true, Vector3f(), 0.0f, true, true, fix_time_ms);
    };

    run_cycle(1000); // call #1 - bootstrap, ra_deltat_ left un-reset (0.5s)

    // Calls #2..#9: call #2 sees 0.5s (pre-bootstrap, unreset) + 0.5s
    // (this cycle) = 1.0s; calls #3-#9 each see a fresh 0.5s. Cumulative
    // omega_i_sum_time_ after call #9: 1.0 + 7*0.5 = 4.5s - still under
    // the 5s threshold, so omega_i_ must not have moved yet.
    for (int cycle = 0; cycle < 8; ++cycle) {
        run_cycle(2000 + static_cast<std::uint32_t>(cycle) * 1000);
        REQUIRE(ahrs.omega_i().x == 0.0f);
        REQUIRE(ahrs.omega_i().y == 0.0f);
    }

    // Call #10: cumulative omega_i_sum_time_ = 4.5 + 0.5 = 5.0s, crossing
    // the >=5s fold-and-clamp threshold.
    run_cycle(10000);

    REQUIRE(ahrs.omega_i().x == 0.0f); // no x-axis error was ever fed in
    REQUIRE(ahrs.omega_i().y > 0.0f);

    // The raw accumulated error*kKi*time vastly exceeds the tiny default
    // gyro-drift-rate bound (by design, see above) - so the folded value
    // must equal the clamp bound exactly, not the larger raw sum.
    const float expected_change_limit = fwcpp::math::radians(0.5f / 60.0f) * 5.0f;
    REQUIRE(ahrs.omega_i().y == Catch::Approx(expected_change_limit).margin(1e-6f));
}

TEST_CASE("drift_correction_accel: stationary level GPS-locked scenario settles near zero, no divergence",
          "[ahrs_dcm]") {
    AhrsDcm ahrs;
    GpsSample gps;
    gps.has_fix = true;
    gps.has_3d_fix = true;
    gps.num_sats = 10;
    gps.velocity_ned = Vector3f(0.0f, 0.0f, 0.0f); // stationary, never changes
    CompassSample compass;                          // unhealthy - keeps use_compass() false

    AccelSample accel;
    accel.accel = Vector3f(0.0f, 0.0f, -9.80665f); // exactly level & stationary
    accel.delta_velocity_dt = 0.01f;
    accel.delta_velocity = accel.accel * accel.delta_velocity_dt;

    for (int cycle = 0; cycle < 30; ++cycle) {
        for (int i = 0; i < 10; ++i) {
            ahrs.accumulate_accel(accel, 0.01f);
        }
        gps.last_fix_time_ms = 1000 + static_cast<std::uint32_t>(cycle) * 1000;
        ahrs.drift_correction_accel(compass, gps, false, true, true, Vector3f(), 0.0f, true, true,
                                     gps.last_fix_time_ms);
    }

    REQUIRE(ahrs.omega_p().length() < 1e-5f);
    REQUIRE(ahrs.omega_i().x == 0.0f);
    REQUIRE(ahrs.omega_i().y == 0.0f);
    REQUIRE(ahrs.omega_i().z == 0.0f);
}
