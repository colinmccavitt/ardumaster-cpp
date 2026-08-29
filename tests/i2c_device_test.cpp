// CPP-088 slice 2: SITL I2C Device register access against an in-memory bank.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/i2c_device.hpp>

using fwcpp::hal::BusType;
using fwcpp::hal::DeviceRegisterBank;
using fwcpp::hal::I2cBus;
using fwcpp::hal::I2cDevice;
using fwcpp::hal::Speed;

TEST_CASE("i2c write_register/read_registers round-trip", "[hal][i2c][device]") {
    DeviceRegisterBank bank;
    I2cBus bus{1};
    I2cDevice dev{bus, bank, 0x1A};

    REQUIRE(dev.bus_type() == BusType::kI2c);
    REQUIRE(dev.bus_num() == 1);
    REQUIRE(dev.get_bus_address() == 0x1A);

    REQUIRE(dev.write_register(0x10, 0xAB));
    std::uint8_t value = 0;
    REQUIRE(dev.read_registers(0x10, &value, 1));
    REQUIRE(value == 0xAB);

    REQUIRE(dev.write_register(0x11, 0xC1));
    REQUIRE(dev.write_register(0x12, 0xC2));
    std::uint8_t seq[2] = {0, 0};
    REQUIRE(dev.read_registers(0x11, seq, 2));
    REQUIRE(seq[0] == 0xC1);
    REQUIRE(seq[1] == 0xC2);
}

TEST_CASE("i2c read_registers ORs read_flag onto first_reg", "[hal][i2c][device]") {
    DeviceRegisterBank bank;
    I2cBus bus{0};
    I2cDevice dev{bus, bank, 0x42};

    REQUIRE(dev.write_register(0x10, 0xAA));
    REQUIRE(dev.write_register(0x90, 0x55));

    std::uint8_t unflagged = 0;
    REQUIRE(dev.read_registers(0x10, &unflagged, 1));
    REQUIRE(unflagged == 0xAA);

    dev.set_read_flag(0x80);
    REQUIRE(dev.read_flag() == 0x80);

    // first_reg 0x10 | 0x80 == 0x90, so the bank read starts at 0x90.
    std::uint8_t flagged = 0;
    REQUIRE(dev.read_registers(0x10, &flagged, 1));
    REQUIRE(flagged == 0x55);
}

TEST_CASE("i2c empty transfer returns false like SITL nmsgs==0", "[hal][i2c][device]") {
    DeviceRegisterBank bank;
    I2cBus bus{0};
    I2cDevice dev{bus, bank, 0x20};

    REQUIRE_FALSE(dev.transfer(nullptr, 0, nullptr, 0));

    const std::uint8_t dummy = 0;
    REQUIRE_FALSE(dev.transfer(&dummy, 0, nullptr, 0));
}

TEST_CASE("i2c set_speed is stored; transfer still uses the bank", "[hal][i2c][device]") {
    DeviceRegisterBank bank;
    I2cBus bus{0};
    I2cDevice dev{bus, bank, 0x30};

    REQUIRE(dev.speed() == Speed::kHigh);
    REQUIRE(dev.set_speed(Speed::kLow));
    REQUIRE(dev.speed() == Speed::kLow);
    REQUIRE(dev.set_speed(Speed::kHigh));
    REQUIRE(dev.speed() == Speed::kHigh);

    REQUIRE(dev.write_register(0x01, 0xFE));
    std::uint8_t value = 0;
    REQUIRE(dev.read_registers(0x01, &value, 1));
    REQUIRE(value == 0xFE);
}

TEST_CASE("i2c get_semaphore is the bus semaphore; periodic is recorded not fired",
          "[hal][i2c][device]") {
    DeviceRegisterBank bank;
    I2cBus bus{2};
    I2cDevice dev{bus, bank, 0x50};

    REQUIRE(&dev.get_semaphore() == &bus.semaphore());
    REQUIRE(dev.get_semaphore().take_nonblocking());
    REQUIRE(dev.get_semaphore().give());

    const auto handle = dev.register_periodic_callback(1000);
    REQUIRE(handle != 0);
    REQUIRE(bus.has_periodic_callback());
    REQUIRE(bus.periodic_period_usec() == 1000);
    REQUIRE(bus.periodic_token() == handle);
    REQUIRE_FALSE(dev.adjust_periodic_callback(handle, 2000));
    REQUIRE_FALSE(dev.read_registers_multiple(0x00, nullptr, 0, 1));
    REQUIRE_FALSE(dev.setup_checked_registers(4));
}
