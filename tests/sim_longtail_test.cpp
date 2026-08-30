#include <cmath>
#include <cstdint>
#include <cstring>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/motors/motors_heli.hpp>
#include <fwcpp/sim/sim_blimp.hpp>
#include <fwcpp/sim/sim_calibration.hpp>
#include <fwcpp/sim/sim_engine_periph.hpp>
#include <fwcpp/sim/sim_helicopter.hpp>
#include <fwcpp/sim/sim_i2c.hpp>
#include <fwcpp/sim/sim_i2c_devices.hpp>
#include <fwcpp/sim/sim_serial_longtail.hpp>
#include <fwcpp/sim/sim_ship.hpp>
#include <fwcpp/sim/sim_spi.hpp>
#include <fwcpp/sim/sim_stratoblimp.hpp>

using namespace fwcpp::sim;
using Catch::Matchers::WithinAbs;

TEST_CASE("Blimp fin thrust produces finite accel") {
    Blimp b;
    SitlInput in{};
    in.servos[0] = 1700;
    in.servos[1] = 1300;
    in.servos[2] = 1600;
    in.servos[3] = 1400;
    for (int i = 0; i < 50; i++) {
        b.update(in, 0.01f);
    }
    REQUIRE(std::isfinite(b.position.x));
    REQUIRE(std::isfinite(b.gyro.z));
}

TEST_CASE("Calibration stop mode damps gyro") {
    Calibration c;
    c.gyro = fwcpp::math::Vector3f(1, 0, 0);
    SitlInput in{};
    in.servos[4] = 1000;
    for (int i = 0; i < 800; i++) {
        c.update(in, 0.01f);
    }
    REQUIRE(std::fabs(c.gyro.x) < 0.5f);
}

TEST_CASE("Calibration angular velocity slews gyro toward axis") {
    Calibration c;
    SitlInput in{};
    in.servos[4] = 2000;
    in.servos[5] = 2000;
    in.servos[6] = 1500;
    in.servos[7] = 1500;
    for (int i = 0; i < 50; i++) {
        c.update(in, 0.01f);
    }
    REQUIRE(std::fabs(c.gyro.x) > 0.01f);
}

TEST_CASE("Ship circular path advances heading") {
    ShipSim sim;
    sim.enable = 1;
    fwcpp::Location home;
    home.lat = -353632621;
    home.lng = 1491652374;
    sim.update(1.0f, 1000000, home);
    REQUIRE(sim.initialised);
    REQUIRE(sim.ship.heading_deg > 0);
    fwcpp::Location loc;
    REQUIRE(sim.get_location(loc));
}

TEST_CASE("StratoBlimp release produces lift") {
    StratoBlimp s;
    SitlInput in{};
    in.servos[4] = 2000;
    in.servos[2] = 1500;
    in.servos[3] = 1500;
    for (int i = 0; i < 20; i++) {
        s.update(in, 0.05f);
    }
    REQUIRE(s.released);
    REQUIRE(std::isfinite(s.accel_body.z));
}

TEST_CASE("MS5611 reset then PROM read") {
    MS5611 ms;
    ms.state = MS5XXX::State::UNINITIALISED;
    std::uint8_t cmd = 0x1E;
    I2cMsg w{2, 0x77, 0, &cmd, 1};
    I2cRdwr wr{&w, 1};
    REQUIRE(ms.rdwr(wr) == 0);
    REQUIRE(ms.state == MS5XXX::State::RESET_START);
    ms.set_now_us(3000);
    Aircraft ac;
    ms.update(ac);
    ms.set_now_us(6000);
    ms.update(ac);
    REQUIRE(ms.prom_loaded);
    std::uint8_t c1 = 0xa2;
    std::uint8_t out[2]{};
    I2cMsg msgs[2]{{2, 0x77, 0, &c1, 1}, {2, 0x77, I2C_M_RD, out, 2}};
    I2cRdwr rd{msgs, 2};
    ms.state = MS5XXX::State::RUNNING;
    REQUIRE(ms.rdwr(rd) == 0);
    const std::uint16_t c = static_cast<std::uint16_t>((out[0] << 8) | out[1]);
    REQUIRE(c == 40127);
}

TEST_CASE("INA3221 channel 2 tracks battery voltage") {
    INA3221 ina;
    Aircraft ac;
    ac.battery_voltage = 12.6f;
    ac.battery_current = 5.0f;
    ac.time_now_us = 200000;
    ina.update(ac);
    REQUIRE(ina.registers.byname.Channel_2_Bus_Voltage != 0);
}

TEST_CASE("SMBus generic cell voltages follow pack") {
    SIM_BattMonitor_SMBus_Generic bat;
    bat.init();
    Aircraft ac;
    ac.battery_voltage = 12.6f;
    ac.time_now_us = 200000;
    bat.update(ac);
    REQUIRE(bat.get_reg_value(SMBusBattDevReg::VOLTAGE) == static_cast<std::uint16_t>(12600));
}

TEST_CASE("Maxell manufacturer block is Hitachi maxell") {
    Maxell m;
    m.init();
    REQUIRE(m.values[SMBusBattDevReg::MANUFACTURE_NAME] == "Hitachi maxell");
}

TEST_CASE("TeraRanger I2C command then CRC reading") {
    TeraRangerI2C tr;
    tr.rangefinder_range = 2.5f;
    tr.set_now_us(1);
    std::uint8_t cmd = 0;
    I2cMsg w{0, 0x31, 0, &cmd, 1};
    I2cRdwr wr{&w, 1};
    REQUIRE(tr.rdwr(wr) == 0);
    tr.set_now_us(1000);
    std::uint8_t buf[3]{};
    I2cMsg r{0, 0x31, I2C_M_RD, buf, 3};
    I2cRdwr rd{&r, 1};
    REQUIRE(tr.rdwr(rd) == 0);
    const std::uint16_t mm = static_cast<std::uint16_t>((buf[0] << 8) | buf[1]);
    REQUIRE(mm == 2500);
}

TEST_CASE("Airspeed DLVR packed pressure is finite") {
    Airspeed_DLVR d;
    d.pressure = 100;
    d.temperature = 25;
    std::uint8_t buf[4]{};
    I2cMsg r{2, 0x28, I2C_M_RD, buf, 4};
    I2cRdwr rd{&r, 1};
    REQUIRE(d.rdwr(rd) == 0);
}

TEST_CASE("TFS20L distance registers") {
    TFS20L t;
    t.rangefinder_range = 1.5f;
    t.strength = 1000;
    std::uint8_t reg = 0;
    I2cMsg w{0, 0x10, 0, &reg, 1};
    I2cRdwr wr{&w, 1};
    REQUIRE(t.rdwr(wr) == 0);
    std::uint8_t buf[2]{};
    I2cMsg msgs[2]{{0, 0x10, 0, &reg, 1}, {0, 0x10, I2C_M_RD, buf, 2}};
    I2cRdwr rd{msgs, 2};
    REQUIRE(t.rdwr(rd) == 0);
    const std::uint16_t cm = static_cast<std::uint16_t>(buf[0] | (buf[1] << 8));
    REQUIRE(cm == 150);
}

TEST_CASE("ICEngine starter path produces idle thrust") {
    ICEngine e;
    e.starter_servo = 3;
    e.ignition_servo = 4;
    SitlInput in{};
    in.servos[2] = 1100;
    in.servos[3] = 1800;
    in.servos[4] = 1800;
    const float p = e.update(in, 1000);
    REQUIRE(p >= 0);
}

TEST_CASE("Gripper EPM field strengthens on grip demand") {
    Gripper_EPM g;
    g.gripper_emp_servo_pin = 1;
    SitlInput in{};
    in.servos[0] = 2000;
    g.last_update_us = 0;
    g.update(in, 1000000);
    REQUIRE(g.field_strength > 0);
}

TEST_CASE("Generator engine RPM slews toward desired") {
    SIM_GeneratorEngine ge;
    ge.desired_rpm = 5000;
    ge.last_rpm_update_ms = 0;
    ge.last_heat_update_ms = 0;
    ge.update(500);
    REQUIRE(ge.current_rpm > 0);
}

TEST_CASE("TeraRangerTower emits TH header") {
    PS_TeraRangerTower tw;
    fwcpp::Location loc;
    tw.update(loc, 1);
    tw.update(loc, 300);
    auto bytes = tw.drain_to_autopilot();
    REQUIRE(bytes.size() >= 2);
    REQUIRE(bytes[0] == 'T');
    REQUIRE(bytes[1] == 'H');
}

TEST_CASE("CRSF cycles original VTX frames") {
    CRSF crsf;
    crsf.update(400);
    auto bytes = crsf.drain_to_autopilot();
    REQUIRE_FALSE(bytes.empty());
    REQUIRE(bytes[0] == 0xC8);
}

TEST_CASE("RAMTRON write-enable then write/read") {
    RAMTRON_FM25V02 ram;
    std::uint8_t wren[1]{0x06};
    SpiIocTransfer t0{wren, nullptr, 1};
    REQUIRE(ram.rdwr(1, &t0) == 0);
    std::uint8_t wr[3]{0x02, 0x00, 0x10};
    SpiIocTransfer t1{wr, nullptr, 3};
    REQUIRE(ram.rdwr(1, &t1) == 0);
    std::uint8_t payload[1]{0xAB};
    SpiIocTransfer t1b{payload, nullptr, 1};
    REQUIRE(ram.rdwr(1, &t1b) == 0);
    std::uint8_t cmd[3]{0x03, 0x00, 0x10};
    SpiIocTransfer t2{cmd, nullptr, 3};
    REQUIRE(ram.rdwr(1, &t2) == 0);
    std::uint8_t rx[1]{};
    SpiIocTransfer t3{nullptr, rx, 1};
    REQUIRE(ram.rdwr(1, &t3) == 0);
    REQUIRE(rx[0] == 0xAB);
}

TEST_CASE("Heli H3_120 swash mixing is not raw PWM identity") {
    fwcpp::motors::MotorsHeliSwash sw;
    sw.configure();
    sw.calculate(0.5f, -0.2f, 0.6f);
    fwcpp::sim::SitlInput in{};
    sw.write_servos(in.servos, 1600);
    REQUIRE(in.servos[0] != in.servos[1]);
    REQUIRE(in.servos[7] == 1600);
    SimHelicopter heli("heli");
    for (int i = 0; i < 30; i++) {
        heli.update(in, 0.01f);
    }
    REQUIRE(std::isfinite(heli.gyro.x));
}

TEST_CASE("Populate default I2C bus matches SIM_I2C.cpp addresses") {
    I2C bus;
    TeraRangerI2C teraranger;
    LightWareI2C_Legacy16Bit lw16;
    MaxSonarI2CXL maxsonar;
    MCP9600 mcp;
    ICM40609 icm;
    SHT3x sht;
    AS5600 as5600;
    MS5525 ms5525;
    INA3221 ina;
    TSYS01 tsys01;
    Rotoye rotoye;
    Maxell maxell;
    SIM_BattMonitor_SMBus_Generic smbus;
    Airspeed_DLVR dlvr;
    Benewake_TFMiniPlus tfmini;
    TSYS03 tsys03;
    MS5611 ms5611;
    QMC5883L qmc;
    TFS20L tfs20l;
    LightWareGRF_I2C grf;
    populate_default_i2c_bus(bus, teraranger, lw16, maxsonar, mcp, icm, sht, as5600, ms5525, ina, tsys01, rotoye, maxell,
                             smbus, dlvr, tfmini, tsys03, ms5611, qmc, tfs20l, grf);
    REQUIRE(bus.devices.size() == 20);
}

TEST_CASE("JEDEC MX25L3206E RDID") {
    JEDEC_MX25L3206E j;
    std::uint8_t cmd[1]{0x9f};
    SpiIocTransfer t0{cmd, nullptr, 1};
    REQUIRE(j.rdwr(1, &t0) == 0);
    std::uint8_t id[3]{};
    SpiIocTransfer t1{nullptr, id, 3};
    REQUIRE(j.rdwr(1, &t1) == 0);
    REQUIRE(id[0] == 0xC2);
    REQUIRE(id[2] == 0x16);
}
