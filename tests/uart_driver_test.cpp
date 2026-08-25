// Tests for fwcpp::hal::UartDriver (minimal AP_HAL/UARTDriver.h port,
// matched against AP_HAL_SITL::UARTDriver's real ring-buffer backend -
// see uart_driver.hpp's file banner for scope and buffer-full policy).

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <fwcpp/hal/uart_driver.hpp>

using namespace fwcpp::hal;

// Small capacity so full-buffer tests don't need to push thousands of
// bytes; the ring-buffer logic is identical at any capacity.
using SmallUart = UartDriver<4>;

TEST_CASE("UartDriver starts empty with full space in both directions", "[uart]") {
    SmallUart uart;
    REQUIRE(uart.available() == 0);
    REQUIRE(uart.txspace() == 4);
    REQUIRE(uart.rx_available() == 0);
    REQUIRE(uart.tx_available() == 0);
}

TEST_CASE("single-byte write then read round-trips the value", "[uart]") {
    SmallUart uart;
    REQUIRE(uart.write(std::uint8_t{0x42}));
    REQUIRE(uart.tx_available() == 1);

    // write() lands in TX, not RX - a transport (or test) must move it
    // across via drain_tx()/inject_rx(), mirroring how a real UART's TX
    // and RX are independent buffers.
    std::uint8_t out = 0;
    REQUIRE(uart.drain_tx(&out, 1) == 1);
    REQUIRE(out == 0x42);
    REQUIRE(uart.tx_available() == 0);
}

TEST_CASE("inject_rx simulates bytes arriving, read() consumes them FIFO", "[uart]") {
    SmallUart uart;
    const std::uint8_t bytes[3] = {1, 2, 3};
    REQUIRE(uart.inject_rx(bytes, 3) == 3);
    REQUIRE(uart.available() == 3);

    std::uint8_t b = 0;
    REQUIRE(uart.read(b));
    REQUIRE(b == 1);
    REQUIRE(uart.read(b));
    REQUIRE(b == 2);
    REQUIRE(uart.available() == 1);
    REQUIRE(uart.read(b));
    REQUIRE(b == 3);
    REQUIRE(uart.available() == 0);
}

TEST_CASE("bulk read/write round trip via inject_rx/drain_tx", "[uart]") {
    SmallUart uart;
    const std::uint8_t in[4] = {10, 20, 30, 40};
    REQUIRE(uart.inject_rx(in, 4) == 4);

    std::uint8_t out[4] = {};
    REQUIRE(uart.read(out, 4) == 4);
    REQUIRE(out[0] == 10);
    REQUIRE(out[3] == 40);
    REQUIRE(uart.available() == 0);

    REQUIRE(uart.write(in, 4) == 4);
    std::uint8_t drained[4] = {};
    REQUIRE(uart.drain_tx(drained, 4) == 4);
    REQUIRE(drained[2] == 30);
}

TEST_CASE("read on empty RX returns false/0 and leaves state untouched", "[uart]") {
    SmallUart uart;
    std::uint8_t b = 0xAA;
    REQUIRE_FALSE(uart.read(b));
    REQUIRE(b == 0xAA); // untouched on failure

    std::uint8_t buf[4] = {};
    REQUIRE(uart.read(buf, 4) == 0);
}

TEST_CASE("single-byte write rejects (returns false) when TX is full, no overwrite", "[uart]") {
    SmallUart uart; // capacity 4
    REQUIRE(uart.write(std::uint8_t{1}));
    REQUIRE(uart.write(std::uint8_t{2}));
    REQUIRE(uart.write(std::uint8_t{3}));
    REQUIRE(uart.write(std::uint8_t{4}));
    REQUIRE(uart.txspace() == 0);

    // Buffer is full: the 5th byte must be rejected, not overwrite byte 1
    // (this port's documented buffer-full policy - see file banner).
    REQUIRE_FALSE(uart.write(std::uint8_t{5}));
    REQUIRE(uart.tx_available() == 4);

    std::uint8_t out = 0;
    REQUIRE(uart.drain_tx(&out, 1) == 1);
    REQUIRE(out == 1); // oldest byte survived untouched
}

TEST_CASE("bulk write on overflow partially accepts and returns the actual count", "[uart]") {
    SmallUart uart; // capacity 4
    const std::uint8_t data[6] = {1, 2, 3, 4, 5, 6};

    // Only 4 bytes fit; write() must accept exactly that many and report
    // it via the return value (matching upstream ByteBuffer::write()'s
    // clamp-to-space() behavior - see file banner), not silently drop the
    // whole call and not overwrite anything already queued.
    const std::size_t written = uart.write(data, 6);
    REQUIRE(written == 4);
    REQUIRE(uart.txspace() == 0);

    std::uint8_t out[4] = {};
    REQUIRE(uart.drain_tx(out, 4) == 4);
    REQUIRE(out[0] == 1);
    REQUIRE(out[1] == 2);
    REQUIRE(out[2] == 3);
    REQUIRE(out[3] == 4);
}

TEST_CASE("bulk write partially fills remaining space when already partially full", "[uart]") {
    SmallUart uart; // capacity 4
    REQUIRE(uart.write(std::uint8_t{0xFF}));
    REQUIRE(uart.txspace() == 3);

    const std::uint8_t data[5] = {1, 2, 3, 4, 5};
    REQUIRE(uart.write(data, 5) == 3); // only 3 slots remained
    REQUIRE(uart.txspace() == 0);

    std::uint8_t out[4] = {};
    REQUIRE(uart.drain_tx(out, 4) == 4);
    REQUIRE(out[0] == 0xFF);
    REQUIRE(out[1] == 1);
    REQUIRE(out[2] == 2);
    REQUIRE(out[3] == 3);
}

TEST_CASE("inject_rx on a full RX buffer partially accepts, matching write()'s policy", "[uart]") {
    SmallUart uart; // capacity 4
    const std::uint8_t data[6] = {9, 8, 7, 6, 5, 4};
    REQUIRE(uart.inject_rx(data, 6) == 4);
    REQUIRE(uart.available() == 4);

    std::uint8_t out[4] = {};
    REQUIRE(uart.read(out, 4) == 4);
    REQUIRE(out[0] == 9);
    REQUIRE(out[3] == 6);
}

TEST_CASE("available()/txspace() accounting stays correct across a mixed sequence", "[uart]") {
    SmallUart uart; // capacity 4
    REQUIRE(uart.available() == 0);
    REQUIRE(uart.txspace() == 4);

    uart.inject_rx(reinterpret_cast<const std::uint8_t*>("ab"), 2);
    REQUIRE(uart.available() == 2);

    std::uint8_t b = 0;
    REQUIRE(uart.read(b));
    REQUIRE(uart.available() == 1);

    REQUIRE(uart.write(std::uint8_t{1}));
    REQUIRE(uart.write(std::uint8_t{2}));
    REQUIRE(uart.txspace() == 2);

    std::uint8_t out = 0;
    REQUIRE(uart.drain_tx(&out, 1) == 1);
    REQUIRE(out == 1);
    REQUIRE(uart.txspace() == 3);

    // Drain the one remaining queued byte (value 2) so TX is empty before
    // the wrap-around loop below, which assumes each write() is drained
    // by the very next drain_tx().
    REQUIRE(uart.drain_tx(&out, 1) == 1);
    REQUIRE(out == 2);
    REQUIRE(uart.txspace() == 4);

    // Ring buffer must wrap correctly: push more than capacity's worth of
    // total lifetime bytes through a small buffer so head_/tail_ wrap
    // around the backing array at least once.
    for (int i = 0; i < 10; ++i) {
        uart.write(std::uint8_t{static_cast<std::uint8_t>(i)});
        std::uint8_t drained = 0;
        REQUIRE(uart.drain_tx(&drained, 1) == 1);
        REQUIRE(drained == static_cast<std::uint8_t>(i));
    }
    REQUIRE(uart.txspace() == 4); // fully drained again after wrap-around
}

TEST_CASE("default-capacity UartDriver matches SITL's own 16384-byte buffers", "[uart]") {
    UartDriver<> uart; // default template argument
    REQUIRE(uart.txspace() == kDefaultUartBufferBytes);
    REQUIRE(uart.available() == 0);
}
