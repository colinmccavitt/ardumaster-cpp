// CCP-043/045: SitlCopterHarness sensor synth + motor PWM → SimMulticopter.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/compass/compass.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/mode.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

using fwcpp::Location;
using fwcpp::compass::Compass;
using fwcpp::copter::LeftoverCopter;
using fwcpp::copter::ModeAltHold;
using fwcpp::copter::ModeStabilize;
using fwcpp::copter::SpoolState;
using fwcpp::copter::leftover_copter_tick;
using fwcpp::hal_sitl::SitlCopterHarness;
using fwcpp::hal_sitl::sitl_copter::PortStatus;
using fwcpp::hal_sitl::sitl_copter::completeness_has;
using fwcpp::hal_sitl::sitl_copter::completeness_size;
using fwcpp::hal_sitl::sitl_copter::on_main_count;
using fwcpp::hal_sitl::sitl_copter::out_of_scope_count;
using fwcpp::hal_sitl::sitl_copter::remaining_count;
using fwcpp::hal_sitl::sitl_copter::this_slice_count;
using fwcpp::math::Vector3f;
using fwcpp::sim::SimMulticopter;

TEST_CASE("SitlCopterHarness step synthesizes gyro accel baro GPS compass",
          "[copter][sitl][ccp-043]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    sim.gyro = Vector3f{0.1f, -0.2f, 0.3f};
    sim.accel_body = Vector3f{0.0f, 0.0f, -9.81f};
    sim.position = Vector3f{10.0f, -20.0f, -50.0f};

    const auto dcm_before = sim.dcm;
    SitlCopterHarness harness(copter, sim);
    REQUIRE(harness.tick_count() == 0);
    REQUIRE(copter.tick_count == 0);
    REQUIRE_FALSE(copter.gyro_injected);
    REQUIRE_FALSE(copter.accel_injected);
    REQUIRE_FALSE(copter.baro_injected);
    REQUIRE_FALSE(copter.gps_injected);
    REQUIRE_FALSE(copter.compass_injected);

    harness.step(0.0025f);
    REQUIRE(harness.tick_count() == 1);
    REQUIRE(copter.tick_count == 1);
    REQUIRE(copter.gyro_injected);
    REQUIRE(copter.accel_injected);
    REQUIRE(copter.gyro_buffer.x == Catch::Approx(0.1f));
    REQUIRE(copter.gyro_buffer.y == Catch::Approx(-0.2f));
    REQUIRE(copter.gyro_buffer.z == Catch::Approx(0.3f));
    REQUIRE(copter.accel_buffer.z == Catch::Approx(-9.81f));

    REQUIRE(copter.baro_injected);
    REQUIRE(copter.baro_altitude_m == Catch::Approx(50.0f));

    REQUIRE(copter.gps_injected);
    Location expected(copter.home_lat, copter.home_lng, 0, Location::AltFrame::ABSOLUTE);
    expected.offset(10.0f, -20.0f);
    REQUIRE(copter.gps_lat == expected.lat);
    REQUIRE(copter.gps_lng == expected.lng);

    REQUIRE(copter.compass_injected);
    REQUIRE(copter.compass_field_bf.length() > 0.1f);
    (void)dcm_before;

    harness.step(0.0025f);
    REQUIRE(copter.tick_count == 2);
}

TEST_CASE("SitlCopterHarness arm spool hold smoke", "[copter][sitl][ccp-043]") {
    LeftoverCopter copter{};
    ModeAltHold althold{};
    copter.current = &althold;
    copter.motors_armed = true;

    SimMulticopter sim{};
    sim.gyro = Vector3f{0.0f, 0.0f, 0.0f};
    sim.accel_body = Vector3f{0.0f, 0.0f, -9.81f};
    sim.position = Vector3f{0.0f, 0.0f, -10.0f};

    SitlCopterHarness harness(copter, sim);
    REQUIRE_FALSE(copter.motors_armed_injected);
    REQUIRE_FALSE(copter.spool_injected);
    REQUIRE_FALSE(copter.attitude_hold_injected);
    REQUIRE(copter.spool_state == SpoolState::SHUT_DOWN);
    REQUIRE_FALSE(copter.attitude_hold);

    harness.step(0.0025f);

    REQUIRE(copter.motors_armed_injected);
    REQUIRE(copter.motors_armed);
    REQUIRE(copter.spool_injected);
    REQUIRE(copter.spool_state == SpoolState::THROTTLE_UNLIMITED);
    REQUIRE(copter.attitude_hold_injected);
    REQUIRE(copter.attitude_hold);
    REQUIRE(copter.tick_count == 1);
    REQUIRE(copter.gyro_injected);
    REQUIRE(copter.baro_injected);

    copter.motors_armed = false;
    harness.step(0.0025f);
    REQUIRE(copter.motors_armed_injected);
    REQUIRE(copter.spool_state == SpoolState::SHUT_DOWN);
    REQUIRE_FALSE(copter.attitude_hold);
    REQUIRE(copter.tick_count == 2);
}

TEST_CASE("SitlCopterHarness motor PWM drives Frame mixing", "[copter][sitl][ccp-045]") {
    LeftoverCopter copter{};
    SimMulticopter sim{};
    sim.position.z = -15.0f;
    copter.motors_armed = true;
    const std::uint16_t high = sim.command_to_pwm(0.70f);
    const std::uint16_t low = sim.command_to_pwm(0.20f);
    copter.motor_pwm[0] = low;
    copter.motor_pwm[1] = high;
    copter.motor_pwm[2] = high;
    copter.motor_pwm[3] = low;

    SitlCopterHarness harness(copter, sim);
    constexpr float kDt = 0.0025f;
    for (int i = 0; i < 20; ++i) {
        harness.step(kDt);
    }
    REQUIRE(sim.gyro.x > 0.05f);  // +roll rate from left-high differential
}

TEST_CASE("leftover_copter_tick wires update_flight_mode when Mode* set",
          "[copter][sitl][ccp-043]") {
    LeftoverCopter copter{};
    ModeStabilize stabilize{};
    copter.current = &stabilize;
    REQUIRE(copter.tick_count == 0);
    leftover_copter_tick(copter);
    REQUIRE(copter.tick_count == 1);
}

TEST_CASE("SitlCopterHarness leftover catalog remaining_count",
          "[copter][sitl][ccp-043][leftover]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(this_slice_count() == 9);
    REQUIRE(on_main_count() == 3);
    REQUIRE(out_of_scope_count() == 2);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("SitlCopterHarness scaffold", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_copter_tick", PortStatus::kThisSlice));
    REQUIRE(completeness_has("gyro/accel synthesis", PortStatus::kThisSlice));
    REQUIRE(completeness_has("baro synthesis", PortStatus::kThisSlice));
    REQUIRE(completeness_has("GPS synthesis", PortStatus::kThisSlice));
    REQUIRE(completeness_has("compass synthesis", PortStatus::kThisSlice));
    REQUIRE(completeness_has("closed-loop arm/spool/hold", PortStatus::kThisSlice));
    REQUIRE(completeness_has("motor PWM to SimMulticopter", PortStatus::kThisSlice));
    REQUIRE(completeness_has("SIM_Multicopter Frame/Motor plant", PortStatus::kOnMain));
    REQUIRE(completeness_has("SitlHarness Plane path (CPP-084)", PortStatus::kOnMain));
}
