// CPP-088 slice 5: in-memory WSPIDevice command header and transfer.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/leftover.hpp>
#include <fwcpp/hal/wspi_device.hpp>

using fwcpp::hal::BusType;
using fwcpp::hal::CommandHeader;
using fwcpp::hal::DeviceRegisterBank;
using fwcpp::hal::PortStatus;
using fwcpp::hal::WspiBus;
using fwcpp::hal::WspiDevice;
using fwcpp::hal::completeness_has;
using fwcpp::hal::remaining_count;

TEST_CASE("wspi set_cmd_header round-trips cmd cfg addr alt dummy",
          "[hal][wspi][header]") {
    DeviceRegisterBank bank;
    WspiBus bus{0};
    WspiDevice dev{bus, bank};

    REQUIRE(dev.bus_type() == BusType::kWspi);
    REQUIRE(dev.cmd_header().cmd == 0);
    REQUIRE(dev.cmd_header().cfg == 0);
    REQUIRE(dev.cmd_header().addr == 0);
    REQUIRE(dev.cmd_header().alt == 0);
    REQUIRE(dev.cmd_header().dummy == 0);

    const CommandHeader hdr{0x06, 0x0100, 0x00ABCDEF, 0x11, 8};
    dev.set_cmd_header(hdr);
    REQUIRE(dev.cmd_header().cmd == 0x06);
    REQUIRE(dev.cmd_header().cfg == 0x0100);
    REQUIRE(dev.cmd_header().addr == 0x00ABCDEF);
    REQUIRE(dev.cmd_header().alt == 0x11);
    REQUIRE(dev.cmd_header().dummy == 8);
}

TEST_CASE("wspi is_busy makes transfer and command-only fail", "[hal][wspi][busy]") {
    DeviceRegisterBank bank;
    WspiBus bus{0};
    WspiDevice dev{bus, bank};

    REQUIRE_FALSE(dev.is_busy());
    REQUIRE(dev.transfer(nullptr, 0, nullptr, 0));

    dev.set_busy(true);
    REQUIRE(dev.is_busy());
    REQUIRE_FALSE(dev.transfer(nullptr, 0, nullptr, 0));

    const std::uint8_t write_cmd[2] = {0x00, 0xAA};
    REQUIRE_FALSE(dev.transfer(write_cmd, 2, nullptr, 0));

    dev.set_busy(false);
    REQUIRE_FALSE(dev.is_busy());
    REQUIRE(dev.transfer(nullptr, 0, nullptr, 0));
}

TEST_CASE("wspi command-only empty send+recv succeeds when not busy",
          "[hal][wspi][command]") {
    DeviceRegisterBank bank;
    WspiBus bus{1};
    WspiDevice dev{bus, bank, 0x10};

    REQUIRE(dev.transfer(nullptr, 0, nullptr, 0));
    REQUIRE(bank.slave_count() == 0);
}

TEST_CASE("wspi send-only writes bank and recv-only reads bank",
          "[hal][wspi][bank]") {
    DeviceRegisterBank bank;
    WspiBus bus{0};
    WspiDevice dev{bus, bank, 0x20};

    const std::uint8_t write_cmd[2] = {0x10, 0xDE};
    REQUIRE(dev.transfer(write_cmd, 2, nullptr, 0));
    REQUIRE(bank.peek(0, 0x20, 0x10) == 0xDE);

    const std::uint8_t at_zero[2] = {0x00, 0xAB};
    REQUIRE(dev.transfer(at_zero, 2, nullptr, 0));

    std::uint8_t recv = 0;
    REQUIRE(dev.transfer(nullptr, 0, &recv, 1));
    REQUIRE(recv == 0xAB);
}

TEST_CASE("wspi combined send+recv is refused like ChibiOS",
          "[hal][wspi][combined]") {
    DeviceRegisterBank bank;
    WspiBus bus{0};
    WspiDevice dev{bus, bank, 0x30};

    const std::uint8_t write_cmd[2] = {0x05, 0x99};
    REQUIRE(dev.transfer(write_cmd, 2, nullptr, 0));

    const std::uint8_t start = 0x05;
    std::uint8_t recv = 0xFF;
    REQUIRE_FALSE(dev.transfer(&start, 1, &recv, 1));
    REQUIRE(recv == 0xFF);
    REQUIRE(bank.peek(0, 0x30, 0x05) == 0x99);
}

TEST_CASE("wspi get_semaphore is the bus semaphore and leftover remaining is 0",
          "[hal][wspi][leftover]") {
    DeviceRegisterBank bank;
    WspiBus bus{2};
    WspiDevice dev{bus, bank};

    REQUIRE(&dev.get_semaphore() == &bus.semaphore());
    REQUIRE(bus.semaphore().depth() == 0);
    REQUIRE(dev.get_semaphore().take(0));
    REQUIRE(bus.semaphore().depth() == 1);
    REQUIRE(dev.get_semaphore().give());

    REQUIRE(remaining_count() == 0);
    REQUIRE(completeness_has("WSPI", PortStatus::kThisSlice));
    REQUIRE(completeness_has("CAN", PortStatus::kOnMain));
}
