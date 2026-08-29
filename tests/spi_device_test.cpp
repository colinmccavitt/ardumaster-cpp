// CPP-088 slice 2: SITL SPI Device transfer against an in-memory bank.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/spi_device.hpp>

using fwcpp::hal::BusType;
using fwcpp::hal::DeviceRegisterBank;
using fwcpp::hal::Speed;
using fwcpp::hal::SpiBus;
using fwcpp::hal::SpiDevice;

TEST_CASE("spi transfer send-then-recv round-trips the register bank", "[hal][spi][device]") {
    DeviceRegisterBank bank;
    SpiBus bus{0};
    SpiDevice dev{bus, bank, /*cs_pin=*/3};

    REQUIRE(dev.bus_type() == BusType::kSpi);
    REQUIRE(dev.bus_num() == 0);
    REQUIRE(dev.cs_pin() == 3);
    REQUIRE(dev.get_bus_address() == 3);

    const std::uint8_t write_cmd[2] = {0x20, 0xDE};
    REQUIRE(dev.transfer(write_cmd, 2, nullptr, 0));

    const std::uint8_t start = 0x20;
    std::uint8_t recv = 0;
    REQUIRE(dev.transfer(&start, 1, &recv, 1));
    REQUIRE(recv == 0xDE);

    const std::uint8_t multi_write[3] = {0x30, 0x11, 0x22};
    REQUIRE(dev.transfer(multi_write, 3, nullptr, 0));
    const std::uint8_t multi_start = 0x30;
    std::uint8_t multi_recv[2] = {0, 0};
    REQUIRE(dev.transfer(&multi_start, 1, multi_recv, 2));
    REQUIRE(multi_recv[0] == 0x11);
    REQUIRE(multi_recv[1] == 0x22);
}

TEST_CASE("spi empty transfer returns false like SITL nmsgs==0", "[hal][spi][device]") {
    DeviceRegisterBank bank;
    SpiBus bus{1};
    SpiDevice dev{bus, bank, 0};

    REQUIRE_FALSE(dev.transfer(nullptr, 0, nullptr, 0));
}

TEST_CASE("spi set_speed is stored; transfer still uses the bank", "[hal][spi][device]") {
    DeviceRegisterBank bank;
    SpiBus bus{0};
    SpiDevice dev{bus, bank, 1};

    REQUIRE(dev.speed() == Speed::kHigh);
    REQUIRE(dev.set_speed(Speed::kLow));
    REQUIRE(dev.speed() == Speed::kLow);

    const std::uint8_t write_cmd[2] = {0x05, 0x99};
    REQUIRE(dev.transfer(write_cmd, 2, nullptr, 0));
    const std::uint8_t start = 0x05;
    std::uint8_t recv = 0;
    REQUIRE(dev.transfer(&start, 1, &recv, 1));
    REQUIRE(recv == 0x99);

    REQUIRE(dev.set_speed(Speed::kHigh));
    REQUIRE(dev.speed() == Speed::kHigh);
}

TEST_CASE("spi get_semaphore is the bus semaphore; fullduplex is refused",
          "[hal][spi][device]") {
    DeviceRegisterBank bank;
    SpiBus bus{0};
    SpiDevice dev{bus, bank, 2};

    REQUIRE(&dev.get_semaphore() == &bus.semaphore());
    REQUIRE_FALSE(dev.transfer_fullduplex(nullptr, nullptr, 0));
    REQUIRE_FALSE(dev.clock_pulse(8));
}
