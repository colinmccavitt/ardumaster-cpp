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
