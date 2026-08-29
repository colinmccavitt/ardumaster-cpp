// CPP-088 slice 4: in-memory CANIface send/receive.

#include <cstdint>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/can_iface.hpp>
#include <fwcpp/hal/leftover.hpp>
#include <fwcpp/hal/semaphore.hpp>

using fwcpp::hal::BinarySemaphore;
using fwcpp::hal::CANFrame;
using fwcpp::hal::CanIface;
using fwcpp::hal::PortStatus;
using fwcpp::hal::completeness_has;
using fwcpp::hal::kCanQueueCapacity;
using fwcpp::hal::remaining_count;

TEST_CASE("can init stores bitrate and marks initialized", "[hal][can][init]") {
    CanIface can;
    REQUIRE_FALSE(can.is_initialized());
    REQUIRE(can.bitrate() == 0);

    REQUIRE(can.init(1'000'000));
    REQUIRE(can.is_initialized());
    REQUIRE(can.bitrate() == 1'000'000);

    CanIface fd;
    REQUIRE(fd.init(500'000, 2'000'000));
    REQUIRE(fd.is_initialized());
    REQUIRE(fd.bitrate() == 500'000);
}

TEST_CASE("can send/receive loopback round-trip", "[hal][can][roundtrip]") {
    CanIface can;
    REQUIRE(can.init(1'000'000));
    can.set_now_us(12345);

    const std::uint8_t payload[3] = {0x11, 0x22, 0x33};
    const CANFrame sent{0x123, payload, 3};

    REQUIRE(sent.id == 0x123);
    REQUIRE(sent.dlc == 3);
    REQUIRE_FALSE(sent.isExtended());
    REQUIRE((sent.id & CANFrame::MaskStdID) == 0x123);

    const auto ext = CANFrame{CANFrame::FlagEFF | 0x1ABCDEF, payload, 3};
    REQUIRE(ext.isExtended());
    REQUIRE((ext.id & CANFrame::MaskExtID) == 0x1ABCDEF);

    BinarySemaphore sem;
    REQUIRE(can.set_event_handle(&sem));
    REQUIRE_FALSE(sem.is_pending());

    REQUIRE(can.send(sent, 999, 0) == 1);
    REQUIRE(can.tx_queued() == 1);
    REQUIRE(can.rx_queued() == 1);
    REQUIRE(sem.is_pending());

    CANFrame got;
    std::uint64_t ts = 0;
    CanIface::CanIOFlags flags = 0;
    REQUIRE(can.receive(got, ts, flags) == 1);
    REQUIRE(got == sent);
    REQUIRE(got.data[0] == 0x11);
    REQUIRE(got.data[1] == 0x22);
    REQUIRE(got.data[2] == 0x33);
    REQUIRE(ts == 12345);
    REQUIRE((flags & CanIface::Loopback) != 0);
    REQUIRE(can.rx_queued() == 0);
    REQUIRE(can.receive(got, ts, flags) == 0);
}

TEST_CASE("can full TX queue returns 0", "[hal][can][full]") {
    CanIface can;
    REQUIRE(can.init(250'000));

    const std::uint8_t payload[1] = {0xAA};
    const CANFrame frame{0x7FF, payload, 1};

    for (std::size_t i = 0; i < kCanQueueCapacity; ++i) {
        REQUIRE(can.send(frame, 0, 0) == 1);
    }
    REQUIRE(can.tx_queued() == kCanQueueCapacity);
    REQUIRE(can.send(frame, 0, 0) == 0);
    REQUIRE(can.tx_queued() == kCanQueueCapacity);
}

TEST_CASE("can uninitialized send returns -1", "[hal][can][uninit]") {
    CanIface can;
    const std::uint8_t payload[1] = {0x01};
    const CANFrame frame{0x001, payload, 1};
    REQUIRE(can.send(frame, 0, 0) == -1);

    CANFrame got;
    std::uint64_t ts = 99;
    CanIface::CanIOFlags flags = 7;
    REQUIRE(can.receive(got, ts, flags) == 0);
}

TEST_CASE("leftover remaining_count is 1 after CAN slice", "[hal][can][leftover]") {
    REQUIRE(remaining_count() == 1);
    REQUIRE(completeness_has("CAN", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Util", PortStatus::kOnMain));
    REQUIRE(completeness_has("WSPI", PortStatus::kRemaining));
}
