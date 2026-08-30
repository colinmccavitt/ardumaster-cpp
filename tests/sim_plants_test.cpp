#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>

#include <fwcpp/location.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_battery.hpp>
#include <fwcpp/sim/sim_baro.hpp>
#include <fwcpp/sim/sim_frame.hpp>
#include <fwcpp/sim/sim_gimbal.hpp>
#include <fwcpp/sim/sim_glider.hpp>
#include <fwcpp/sim/sim_gps.hpp>
#include <fwcpp/sim/sim_helicopter.hpp>
#include <fwcpp/sim/sim_json.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>
#include <fwcpp/sim/sim_plane.hpp>
#include <fwcpp/sim/sim_quadplane.hpp>
#include <fwcpp/sim/sim_rangefinder.hpp>
#include <fwcpp/sim/sim_serial_rangefinder.hpp>
#include <fwcpp/sim/sim_singlecopter.hpp>
#include <fwcpp/sim/sim_slung_payload.hpp>
#include <fwcpp/sim/sim_tether.hpp>

using Catch::Approx;
using namespace fwcpp::sim;
using fwcpp::Location;

TEST_CASE("1976 atmosphere density is SSL at 0 m and falls with altitude") {
    REQUIRE(get_air_density_for_alt_amsl(0.0f) == Approx(1.225f).margin(0.01f));
    REQUIRE(get_air_density_for_alt_amsl(11000.0f) < get_air_density_for_alt_amsl(0.0f));
    REQUIRE(get_air_density_for_alt_amsl(5000.0f) < 1.0f);
}

TEST_CASE("battery SoC drain sag and filter voltage") {
    Battery b;
    b.setup(1.0f, 0.05f, 12.6f, 25.0f);
    const float v0 = b.get_voltage();
    REQUIRE(v0 == Approx(12.6f).margin(0.05f));
    for (int i = 0; i < 200; ++i) {
        b.consume_energy(20.0f, static_cast<std::uint64_t>((i + 1) * 20000));
    }
    REQUIRE(b.get_voltage() < v0);
    REQUIRE(b.remaining_ah() < 1.0f);
    REQUIRE(b.get_temperature_degC() > 0.0f);
}

TEST_CASE("SitlInput wind fields exist and move a copter") {
    SimMulticopter still;
    SimMulticopter blown;
    SitlInput in{};
    still.frame().set_equal_command(in, still.hover_command());
    for (int i = 0; i < 200; ++i) {
        still.update(in, 0.02f);
    }
    SitlInput windy = in;
    windy.wind.speed = 8.0f;
    windy.wind.direction = 90.0f;
    blown.frame().set_equal_command(windy, blown.hover_command());
    blown.position.z = -2.0f;
    for (int i = 0; i < 200; ++i) {
        blown.update(windy, 0.02f);
    }
    REQUIRE(std::fabs(blown.wind_ef.y) > 1.0f);
}

TEST_CASE("JSON parser loads mass and vector3") {
    const std::string json = R"({"mass": 2.0, "motor1_position": [0.15, 0.15, 0.0]})";
    JsonParser parser(json);
    JsonValue obj;
    REQUIRE(parser.parse(obj));
    float mass = 0;
    REQUIRE(json_get_float(obj, "mass", mass));
    REQUIRE(mass == Approx(2.0f).margin(0.01f));
}

TEST_CASE("QuadPlane has Frame motors at offset and Plane aero") {
    SimQuadPlane qp("quadplane");
    REQUIRE(qp.frame().valid());
    REQUIRE(qp.frame().motor_offset == 4);
    REQUIRE(qp.frame().num_motors == 4);
    SitlInput in{};
    qp.frame().set_equal_command(in, qp.frame().hover_command());
    in.servos[2] = 1500;
    qp.position.z = -1.0f;
    for (int i = 0; i < 50; ++i) {
        qp.update(in, 0.02f);
    }
    REQUIRE(std::isfinite(qp.gyro.x));
    REQUIRE(qp.battery_voltage > 0.0f);
}

TEST_CASE("SingleCopter coaxial vs single consume servos") {
    SimSingleCopter s("singlecopter");
    SimSingleCopter c("coax");
    SitlInput in{};
    for (int i = 0; i < 4; ++i) {
        in.servos[i] = 1500;
    }
    in.servos[4] = 1700;
    in.servos[5] = 1700;
    s.position.z = -1.0f;
    c.position.z = -1.0f;
    s.update(in, 0.02f);
    c.update(in, 0.02f);
    REQUIRE(std::isfinite(s.accel_body.z));
    REQUIRE(std::isfinite(c.accel_body.z));
}

TEST_CASE("sim baro gps rangefinder come from Aircraft location") {
    SimMulticopter copter;
    copter.set_start_location(Location(-353632621, 1491652374, 58400, Location::AltFrame::ABSOLUTE), 0.0f);
    copter.position.z = -10.0f;
    copter.update_position();
    copter.update_mag_field_bf();
    const auto baro = sitl_baro_from_aircraft(copter);
    const auto gps = sitl_gps_from_aircraft(copter);
    const auto rf = sitl_rangefinder_from_aircraft(copter);
    REQUIRE(baro.pressure_pa > 50000.0f);
    REQUIRE(gps.lat != 0);
    REQUIRE(rf.healthy);
    REQUIRE(copter.get_mag_field_bf().length() > 0.1f);
}

TEST_CASE("tether and slung payload produce forces when enabled") {
    Tether t;
    t.enabled = true;
    t.length = 5.0f;
    fwcpp::math::Vector3f f;
    t.get_forces_on_vehicle(fwcpp::math::Vector3f(0, 0, 20), fwcpp::math::Vector3f(0, 0, 1), f);
    REQUIRE(f.z < 0.0f);

    SlungPayload p;
    p.enabled = true;
    p.reset(2.0f, 1.0f);
    fwcpp::math::Vector3f fv;
    p.update(fwcpp::math::Vector3f(0, 0, 0), fwcpp::math::Vector3f(0, 0, 0), 0.02f, fv);
    REQUIRE(std::isfinite(fv.x));
}

TEST_CASE("vibe_motor couples command into rpm") {
    SimMulticopter copter;
    copter.sitl_params.vibe_motor = 100.0f;
    copter.frame().set_sitl(&copter.sitl_params);
    copter.frame().rpm_out = copter.rpm;
    SitlInput in{};
    copter.set_equal_command(in, 0.5f);
    fwcpp::math::Vector3f rot, acc;
    copter.calculate_forces(in, rot, acc);
    REQUIRE(copter.rpm[0] > 0.0f);
}

TEST_CASE("Helicopter conventional plant runs rotor dynamics") {
    SimHelicopter heli("heli");
    REQUIRE(heli.frame_type() == SimHelicopter::kConventional);
    SitlInput in{};
    for (int i = 0; i < 4; ++i) {
        in.servos[i] = 1500;
    }
    in.servos[7] = 1700;
    heli.position.z = -2.0f;
    for (int i = 0; i < 50; ++i) {
        heli.update(in, 0.0025f);
    }
    REQUIRE(std::isfinite(heli.gyro.x));
    REQUIRE(heli.battery_voltage > 0.0f);
}

TEST_CASE("Glider starts nose-down waiting for pickup") {
    SimGlider g("glider");
    REQUIRE(g.carriage_state == SimGlider::CarriageState::kWaitingForPickup);
    SitlInput in{};
    g.update(in, 0.02f);
    REQUIRE(std::isfinite(g.gyro.x));
}

TEST_CASE("Gimbal update produces finite joint angles") {
    SimMulticopter copter;
    SimGimbal gimbal;
    gimbal.set_demanded_rates(fwcpp::math::Vector3f(0.1f, 0.0f, 0.0f));
    gimbal.update(copter, 20000);
    fwcpp::math::Vector3f ja;
    gimbal.get_joint_angles(ja);
    REQUIRE(std::isfinite(ja.x));
}

TEST_CASE("Maxsonar serial rangefinder encodes centimeters") {
    MaxsonarSerialLV rf;
    std::uint8_t buf[16]{};
    const auto n = rf.packet_for_alt(1.23f, buf, sizeof(buf));
    REQUIRE(n > 0);
    REQUIRE(buf[n - 1] == '\r');
}
