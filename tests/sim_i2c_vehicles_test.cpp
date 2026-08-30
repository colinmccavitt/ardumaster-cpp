#include <cstdint>
#include <cstring>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/sim/sim_balancebot.hpp>
#include <fwcpp/sim/sim_balloon.hpp>
#include <fwcpp/sim/sim_i2c.hpp>
#include <fwcpp/sim/sim_rover.hpp>
#include <fwcpp/sim/sim_sailboat.hpp>
#include <fwcpp/sim/sim_tracker.hpp>
#include <fwcpp/sim/sim_submarine.hpp>
#include <fwcpp/sim/sim_payloads.hpp>

using namespace fwcpp::sim;
using Catch::Matchers::WithinAbs;

TEST_CASE("I2C 16-bit register read is big-endian") {
    I2CRegisters_16Bit regs;
    regs.add_register("x", 0x03, I2CRegisters::RegMode::RDWR);
    regs.set_register(0x03, 0x1234);
    std::uint8_t addr = 0x03;
    std::uint8_t out[2]{};
    I2cMsg msgs[2]{{0, 0x0D, 0, &addr, 1}, {0, 0x0D, I2C_M_RD, out, 2}};
    I2cRdwr data{msgs, 2};
    REQUIRE(regs.rdwr(data) == 0);
    REQUIRE(out[0] == 0x34);
    REQUIRE(out[1] == 0x12);
}

TEST_CASE("QMC5883L continuous mode fills XYZ from body mag") {
    QMC5883L mag;
    mag.registers[0x09] = 0x01;
    Aircraft ac;
    ac.mag_bf = fwcpp::math::Vector3f(10, 20, 30);
    mag.update(ac);
    REQUIRE((mag.registers[0x06] & 0x04) != 0);
}

TEST_CASE("MaxSonar I2C command then read after 20 ms") {
    MaxSonarI2CXL rf;
    rf.rangefinder_range = 1.25f;
    std::uint8_t cmd = 0x51;
    I2cMsg w{0, 0x70, 0, &cmd, 1};
    I2cRdwr wr{&w, 1};
    rf.set_now_ms(0);
    REQUIRE(rf.rdwr(wr) == 0);
    std::uint8_t buf[2]{};
    I2cMsg r{0, 0x70, I2C_M_RD, buf, 2};
    I2cRdwr rd{&r, 1};
    rf.set_now_ms(10);
    REQUIRE(rf.rdwr(rd) == -1);
    rf.set_now_ms(25);
    REQUIRE(rf.rdwr(rd) == 0);
    const std::uint16_t cm = static_cast<std::uint16_t>((buf[0] << 8) | buf[1]);
    REQUIRE(cm == 125);
}

TEST_CASE("Rover throttle produces forward speed") {
    SimRover rover("rover");
    SitlInput in{};
    in.servos[0] = 1500;
    in.servos[2] = 1800;
    for (int i = 0; i < 200; i++) {
        rover.update(in, 0.01f);
    }
    REQUIRE(rover.get_velocity_ef().length() > 0.5f);
}

TEST_CASE("Balloon release climbs then burst") {
    Balloon b;
    SitlInput in{};
    in.servos[6] = 1900;
    for (int i = 0; i < 50; i++) {
        b.update(in, 0.05f);
    }
    REQUIRE(b.released);
    REQUIRE(b.position.z < 0);
}

TEST_CASE("Sailboat motor-connected throttle moves hull") {
    MotorBoat boat;
    REQUIRE(boat.motor_connected);
    REQUIRE(boat.sail_area == 0.0f);
    SitlInput in{};
    in.servos[0] = 1500;
    in.servos[2] = 1800;
    for (int i = 0; i < 200; i++) {
        boat.update(in, 0.01f);
    }
    REQUIRE(boat.get_velocity_ef().length() > 0.1f);
}

TEST_CASE("Tracker attitude PWM slews pitch") {
    Tracker tr;
    SitlInput in{};
    in.servos[0] = 1500;
    in.servos[1] = 2000;
    for (int i = 0; i < 200; i++) {
        tr.update(in, 0.02f);
    }
    float r, p, y;
    tr.get_dcm().to_euler(&r, &p, &y);
    REQUIRE(std::fabs(p) > 0.01f);
}

TEST_CASE("NoVehicle update is a no-op") {
    NoVehicle nv;
    const auto z = nv.position.z;
    SitlInput in{};
    nv.update(in, 0.01f);
    REQUIRE(nv.position.z == z);
}

TEST_CASE("BalanceBot armed pitch dynamics stay finite") {
    BalanceBot bot;
    SitlInput in{};
    in.servos[0] = 1600;
    in.servos[2] = 1600;
    for (int i = 0; i < 50; i++) {
        bot.update(in, 0.0025f);
    }
    REQUIRE(std::isfinite(bot.position.x));
    REQUIRE(std::isfinite(bot.gyro.y));
}

TEST_CASE("Submarine vertical thrusters produce finite accel") {
    Submarine sub("vectored");
    SitlInput in{};
    in.servos[4] = 1800;
    in.servos[5] = 1800;
    for (int i = 0; i < 50; i++) {
        sub.update(in, 0.01f);
    }
    REQUIRE(std::isfinite(sub.position.z));
}

TEST_CASE("Gripper jaw opens on release PWM") {
    Gripper_Servo g;
    g.gripper_servo_pin = 1;
    g.altitude = 0;
    SitlInput in{};
    in.servos[0] = 1000;
    for (int i = 0; i < 200; i++) {
        g.update(in, 0.05f);
    }
    REQUIRE(g.is_jaw_open());
}

TEST_CASE("Sprayer pump depletes capacity") {
    Sprayer s;
    s.sprayer_pump_pin = 1;
    SitlInput in{};
    in.servos[0] = 2000;
    const double c0 = s.capacity;
    for (int i = 0; i < 50; i++) {
        s.update(in, 0.1f);
    }
    REQUIRE(s.capacity < c0);
}

TEST_CASE("Parachute deploys above 1250 PWM") {
    Parachute p;
    p.parachute_pin = 1;
    SitlInput in{};
    in.servos[0] = 1600;
    p.update(in, 1234);
    REQUIRE(p.deployed_ms == 1234);
}
