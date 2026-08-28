// CPP-028 leftover closer: AhrsDcm production accessors + leftover-complete
// catalog. Does not redo DCM slices 1-3 or the NavEKF3/AhrsBackend arc.

#include <string_view>

#include <fwcpp/ahrs/ahrs_backend.hpp>
#include <fwcpp/ahrs/ahrs_dcm.hpp>
#include <fwcpp/ahrs/ahrs_leftover.hpp>
#include <fwcpp/airspeed/airspeed_sensor.hpp>
#include <fwcpp/ekf/ekf_backend.hpp>
#include <fwcpp/location.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using fwcpp::Location;
using fwcpp::ahrs::AccelSample;
using fwcpp::ahrs::AhrsBackend;
using fwcpp::ahrs::AhrsDcm;
using fwcpp::ahrs::AhrsPortItem;
using fwcpp::ahrs::CompassSample;
using fwcpp::ahrs::GpsSample;
using fwcpp::ahrs::GyroSample;
using fwcpp::ahrs::PortStatus;
using fwcpp::ahrs::ahrs_completeness_count;
using fwcpp::ahrs::kAhrsCompleteness;
using fwcpp::math::Vector2f;
using fwcpp::math::Vector3f;

namespace {

void drive_full_cycle(AhrsBackend& backend, const GpsSample& gps, std::uint32_t now_ms,
                      float airspeed_tas = 0.0f, bool accel_healthy = true, bool ins_healthy = true) {
    GyroSample gyro;
    gyro.dangle_dt = 0.01f;
    AccelSample accel;
    accel.delta_velocity_dt = 0.01f;
    accel.accel = Vector3f(0.0f, 0.0f, -9.80665f);
    CompassSample compass;
    backend.update_full_cycle(gyro, accel, 0.01f, compass, gps, /*fly_forward=*/true,
                              /*armed_and_safety_off=*/false, /*gps_use_enabled=*/true, /*wind_speed_ms=*/0.0f,
                              Vector3f{}, airspeed_tas, accel_healthy, ins_healthy, now_ms);
}

} // namespace

TEST_CASE("CPP-028 leftover catalog: Remaining is empty of in-scope work", "[ahrs_leftover]") {
    REQUIRE(ahrs_completeness_count() == 13);
    REQUIRE(ahrs_completeness_count(PortStatus::OnMain) == 5);
    REQUIRE(ahrs_completeness_count(PortStatus::ThisSlice) == 6);
    REQUIRE(ahrs_completeness_count(PortStatus::Remaining) == 0);
    REQUIRE(ahrs_completeness_count(PortStatus::OutOfScope) == 2);

    bool saw_wind = false;
    bool saw_gcs = false;
    for (const AhrsPortItem& item : kAhrsCompleteness) {
        REQUIRE(item.name != nullptr);
        REQUIRE(item.note != nullptr);
        if (item.status == PortStatus::OutOfScope) {
            const bool wind = std::string_view(item.name).find("wind") != std::string_view::npos;
            const bool gcs = std::string_view(item.name).find("GCS") != std::string_view::npos;
            REQUIRE((wind || gcs));
            if (wind) {
                saw_wind = true;
            }
            if (gcs) {
                saw_gcs = true;
            }
        }
        REQUIRE(item.status != PortStatus::Remaining);
    }
    REQUIRE(saw_wind);
    REQUIRE(saw_gcs);
}

TEST_CASE("healthy defaults true and pre_arm_check follows it", "[ahrs_leftover]") {
    AhrsDcm ahrs;
    REQUIRE(ahrs.healthy(0));
    REQUIRE(ahrs.pre_arm_check(/*requires_position=*/true, 0));
    REQUIRE(ahrs.pre_arm_check(/*requires_position=*/false, 0));
}

TEST_CASE("unhealthy accel stamps last_failure_ms; healthy recovers after 5s", "[ahrs_leftover]") {
    AhrsDcm ahrs;
    ahrs.reset(Vector3f(0.0f, 0.0f, -9.80665f), false, 1000);

    GpsSample gps;
    gps.has_fix = true;
    gps.has_3d_fix = true;
    gps.ground_speed_ms = 15.0f;
    gps.velocity_ned = Vector3f(15.0f, 0.0f, 0.0f);
    gps.last_fix_time_ms = 1000;
    gps.num_sats = 8;

    // First GPS-triggered call only arms ra_sum_start_ (not a failure).
    // Second call with a new fix reaches the no-healthy-accelerometers return
    // (upstream stamps _last_failure_ms there).
    ahrs.accumulate_accel(AccelSample{}, 0.25f);
    ahrs.drift_correction_accel(CompassSample{}, gps, true, false, true, Vector3f{}, 0.0f, false, true, 1000);
    REQUIRE(ahrs.healthy(1000));

    gps.last_fix_time_ms = 1200;
    ahrs.accumulate_accel(AccelSample{}, 0.25f);
    ahrs.drift_correction_accel(CompassSample{}, gps, true, false, true, Vector3f{}, 0.0f, false, true, 1200);

    REQUIRE_FALSE(ahrs.healthy(1200));
    REQUIRE_FALSE(ahrs.pre_arm_check(true, 1200));
    REQUIRE_FALSE(ahrs.healthy(6199));
    REQUIRE(ahrs.healthy(6201));
    REQUIRE(ahrs.pre_arm_check(true, 6201));
}

TEST_CASE("groundspeed accessors read the last GPS sample from update_full_cycle", "[ahrs_leftover]") {
    AhrsDcm ahrs;
    ahrs.reset(Vector3f(0.0f, 0.0f, -9.80665f), false, 200);

    REQUIRE(ahrs.groundspeed() == 0.0f);
    REQUIRE(ahrs.groundspeed_vector() == Vector2f{});

    GpsSample gps;
    gps.has_fix = true;
    gps.has_3d_fix = true;
    gps.ground_speed_ms = 12.5f;
    gps.ground_course_deg = 90.0f;
    gps.velocity_ned = Vector3f(0.0f, 12.5f, 0.2f);
    gps.last_fix_time_ms = 200;
    gps.num_sats = 10;

    drive_full_cycle(ahrs, gps, 200);

    REQUIRE(ahrs.groundspeed() == Catch::Approx(12.5f));
    REQUIRE(ahrs.groundspeed_vector().x == Catch::Approx(0.0f));
    REQUIRE(ahrs.groundspeed_vector().y == Catch::Approx(12.5f));

    AhrsBackend& backend = ahrs;
    REQUIRE(backend.groundspeed() == Catch::Approx(12.5f));
    REQUIRE(backend.groundspeed_vector().y == Catch::Approx(12.5f));
}

TEST_CASE("no-fix GPS yields zero groundspeed", "[ahrs_leftover]") {
    AhrsDcm ahrs;
    GpsSample gps;
    gps.ground_speed_ms = 99.0f;
    gps.velocity_ned = Vector3f(99.0f, 0.0f, 0.0f);
    drive_full_cycle(ahrs, gps, 200);
    REQUIRE(ahrs.groundspeed() == 0.0f);
    REQUIRE(ahrs.groundspeed_vector() == Vector2f{});
}

TEST_CASE("airspeed_EAS uses a real AirspeedSensor when healthy", "[ahrs_leftover]") {
    AhrsDcm ahrs;
    fwcpp::airspeed::AirspeedSensor sensor;
    REQUIRE_FALSE(ahrs.using_airspeed_sensor());

    float eas = -1.0f;
    REQUIRE_FALSE(ahrs.airspeed_EAS(eas));
    REQUIRE(eas == Catch::Approx(0.0f));

    // q = 0.5 * 1.225 * V^2, V=20 m/s => 245 Pa. ratio=2 => sqrt(245*2) = 22.135...
    sensor.update(245.0f);
    REQUIRE(sensor.healthy());
    ahrs.observe_airspeed(sensor.airspeed(), sensor.healthy());

    REQUIRE(ahrs.using_airspeed_sensor());
    REQUIRE(ahrs.airspeed_EAS(eas));
    REQUIRE(eas == Catch::Approx(sensor.airspeed()));

    AhrsBackend& backend = ahrs;
    float via_backend = 0.0f;
    REQUIRE(backend.using_airspeed_sensor());
    REQUIRE(backend.airspeed_EAS(via_backend));
    REQUIRE(via_backend == Catch::Approx(sensor.airspeed()));
}

TEST_CASE("airspeed_EAS falls back to last GPS-derived TAS cache when the sensor is unused",
          "[ahrs_leftover]") {
    AhrsDcm ahrs;
    ahrs.reset(Vector3f(0.0f, 0.0f, -9.80665f), false, 200);

    GpsSample gps;
    gps.has_fix = true;
    gps.has_3d_fix = true;
    gps.ground_speed_ms = 18.0f;
    gps.velocity_ned = Vector3f(18.0f, 0.0f, 0.0f);
    gps.last_fix_time_ms = 200;
    gps.num_sats = 8;

    // airspeed_tas=0: drift_correction_accel still writes last_airspeed_tas_ from GPS velocity.
    drive_full_cycle(ahrs, gps, 200, /*airspeed_tas=*/0.0f);
    REQUIRE(ahrs.last_airspeed_tas() > 0.0f);

    ahrs.observe_airspeed(0.0f, false);
    float eas = -1.0f;
    REQUIRE_FALSE(ahrs.airspeed_EAS(eas));
    REQUIRE(eas == Catch::Approx(ahrs.last_airspeed_tas()));
    REQUIRE_FALSE(ahrs.using_airspeed_sensor());
}

TEST_CASE("relative position NE/D home uses Location helpers", "[ahrs_leftover]") {
    AhrsDcm ahrs;
    Vector2f ne;
    float pos_d = 0.0f;
    REQUIRE_FALSE(ahrs.home_is_set());
    REQUIRE_FALSE(ahrs.get_relative_position_NE_home(ne));
    REQUIRE_FALSE(ahrs.get_relative_position_D_home(pos_d));

    // ~111.3 m/deg at equator; 1e7 deg units. 100 m north ~= 898 deg*1e7 units.
    const Location home(0, 0, 10000, Location::AltFrame::ABSOLUTE); // 100.00 m AMSL
    ahrs.set_home(home);
    REQUIRE(ahrs.home_is_set());
    REQUIRE_FALSE(ahrs.get_relative_position_NE_home(ne)); // no current position yet

    Location current = home;
    current.offset(50.0f, -20.0f);
    current.alt = 11200; // 12 m above home
    ahrs.observe_position(current);

    REQUIRE(ahrs.get_relative_position_NE_home(ne));
    REQUIRE(ne.x == Catch::Approx(50.0f).margin(0.5f));
    REQUIRE(ne.y == Catch::Approx(-20.0f).margin(0.5f));

    REQUIRE(ahrs.get_relative_position_D_home(pos_d));
    REQUIRE(pos_d == Catch::Approx(-12.0f));

    AhrsBackend& backend = ahrs;
    Vector2f ne2;
    float d2 = 0.0f;
    REQUIRE(backend.home_is_set());
    REQUIRE(backend.get_relative_position_NE_home(ne2));
    REQUIRE(backend.get_relative_position_D_home(d2));
    REQUIRE(ne2.x == Catch::Approx(ne.x));
    REQUIRE(d2 == Catch::Approx(pos_d));
}

TEST_CASE("EkfCoreBackend implements the leftover AhrsBackend production surface", "[ahrs_leftover]") {
    fwcpp::ekf::EkfCoreBackend ekf;
    AhrsBackend& backend = ekf;

    REQUIRE(backend.healthy(0));
    REQUIRE(backend.pre_arm_check(false, 0));
    REQUIRE(backend.groundspeed() == 0.0f);

    GpsSample gps;
    gps.has_fix = true;
    gps.has_3d_fix = true;
    gps.ground_speed_ms = 8.0f;
    gps.velocity_ned = Vector3f(8.0f, 0.0f, 0.0f);
    gps.last_fix_time_ms = 200;
    gps.num_sats = 9;
    drive_full_cycle(backend, gps, 200);
    REQUIRE(backend.groundspeed() == Catch::Approx(8.0f));
    REQUIRE(backend.groundspeed_vector().x == Catch::Approx(8.0f));

    fwcpp::airspeed::AirspeedSensor sensor;
    sensor.update(245.0f);
    backend.observe_airspeed(sensor.airspeed(), sensor.healthy());
    float eas = 0.0f;
    REQUIRE(backend.using_airspeed_sensor());
    REQUIRE(backend.airspeed_EAS(eas));
    REQUIRE(eas == Catch::Approx(sensor.airspeed()));

    const Location home(10000000, 0, 0, Location::AltFrame::ABSOLUTE);
    backend.set_home(home);
    Location current = home;
    current.offset(10.0f, 0.0f);
    backend.observe_position(current);
    Vector2f ne;
    REQUIRE(backend.get_relative_position_NE_home(ne));
    REQUIRE(ne.x == Catch::Approx(10.0f).margin(0.5f));
}

TEST_CASE("shared leftover helpers match the AhrsDcm accessors", "[ahrs_leftover]") {
    using fwcpp::ahrs::airspeed_eas_from_sensor;
    using fwcpp::ahrs::groundspeed_from_gps;
    using fwcpp::ahrs::groundspeed_vector_from_gps;
    using fwcpp::ahrs::healthy_from_last_failure;
    using fwcpp::ahrs::pre_arm_check_from_healthy;
    using fwcpp::ahrs::relative_position_d_home;
    using fwcpp::ahrs::relative_position_ne_home;
    using fwcpp::ahrs::using_airspeed_from_sensor;

    REQUIRE(healthy_from_last_failure(0, 0));
    REQUIRE_FALSE(healthy_from_last_failure(1000, 1000));
    REQUIRE(healthy_from_last_failure(1000, 6001));
    REQUIRE(pre_arm_check_from_healthy(true, true));
    REQUIRE_FALSE(pre_arm_check_from_healthy(false, false));

    REQUIRE(groundspeed_from_gps(true, 7.0f) == Catch::Approx(7.0f));
    REQUIRE(groundspeed_from_gps(false, 7.0f) == Catch::Approx(0.0f));
    const Vector2f v = groundspeed_vector_from_gps(true, Vector3f(3.0f, 4.0f, 9.0f));
    REQUIRE(v.x == Catch::Approx(3.0f));
    REQUIRE(v.y == Catch::Approx(4.0f));

    float eas = 0.0f;
    REQUIRE(airspeed_eas_from_sensor(19.0f, true, 11.0f, eas));
    REQUIRE(eas == Catch::Approx(19.0f));
    REQUIRE_FALSE(airspeed_eas_from_sensor(19.0f, false, 11.0f, eas));
    REQUIRE(eas == Catch::Approx(11.0f));
    REQUIRE(using_airspeed_from_sensor(true));
    REQUIRE_FALSE(using_airspeed_from_sensor(false));

    const Location home(0, 0, 5000, Location::AltFrame::ABSOLUTE);
    Location current = home;
    current.offset(0.0f, 30.0f);
    current.alt = 4700;
    Vector2f ne;
    float pos_d = 0.0f;
    REQUIRE(relative_position_ne_home(home, true, current, true, ne));
    REQUIRE(ne.y == Catch::Approx(30.0f).margin(0.5f));
    REQUIRE(relative_position_d_home(home, true, current, true, pos_d));
    REQUIRE(pos_d == Catch::Approx(3.0f));
    REQUIRE_FALSE(relative_position_ne_home(home, false, current, true, ne));
}
