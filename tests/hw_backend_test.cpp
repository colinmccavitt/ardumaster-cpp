// CPP-089: BoardKind + Watchdog stub (slice 1); LinuxHalContext (slice 2).

#include <array>
#include <cstdint>
#include <span>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/board.hpp>
#include <fwcpp/hal/hw_leftover.hpp>
#include <fwcpp/hal/leftover.hpp>
#include <fwcpp/hal/linux_hal.hpp>
#include <fwcpp/hal/watchdog.hpp>

using fwcpp::hal::BoardKind;
using fwcpp::hal::LinuxHalContext;
using fwcpp::hal::PinMode;
using fwcpp::hal::Watchdog;
using fwcpp::hal::kDefaultBoardKind;
using fwcpp::hal::kWatchdogPersistentWords;
using fwcpp::hal::hw::PortStatus;
using fwcpp::hal::hw::completeness_has;
using fwcpp::hal::hw::hw_completeness_size;
using fwcpp::hal::hw::on_main_count;
using fwcpp::hal::hw::out_of_scope_count;
using fwcpp::hal::hw::remaining_count;
using fwcpp::hal::hw::this_slice_count;

TEST_CASE("BoardKind default is SITL", "[hal][hw][board]") {
    REQUIRE(kDefaultBoardKind == BoardKind::kSitl);
    const BoardKind kind{};
    REQUIRE(kind == BoardKind::kSitl);
}

TEST_CASE("watchdog init then pat records both seams", "[hal][hw][watchdog]") {
    Watchdog wdg;
    REQUIRE_FALSE(wdg.is_initialized());
    REQUIRE_FALSE(wdg.is_enabled());
    REQUIRE(wdg.pat_count() == 0);

    wdg.pat();
    REQUIRE(wdg.pat_count() == 0);

    wdg.init();
    REQUIRE(wdg.is_initialized());
    REQUIRE(wdg.is_enabled());

    wdg.pat();
    wdg.pat();
    REQUIRE(wdg.pat_count() == 2);
}

TEST_CASE("watchdog was_reset is an injected stored bool", "[hal][hw][watchdog]") {
    Watchdog wdg;
    REQUIRE_FALSE(wdg.was_reset());
    wdg.set_was_reset(true);
    REQUIRE(wdg.was_reset());
    wdg.set_was_reset(false);
    REQUIRE_FALSE(wdg.was_reset());
}

TEST_CASE("watchdog save/load round-trips a uint32 word span", "[hal][hw][watchdog]") {
    Watchdog wdg;
    const std::array<std::uint32_t, 4> src{0x11111111u, 0x22222222u, 0x33333333u, 0x44444444u};
    wdg.save(std::span<const std::uint32_t>(src));
    REQUIRE(wdg.saved_word_count() == 4);

    std::array<std::uint32_t, 4> dst{};
    wdg.load(std::span<std::uint32_t>(dst));
    REQUIRE(dst == src);

    std::array<std::uint32_t, kWatchdogPersistentWords + 1> overflow{};
    overflow.fill(0xA5A5A5A5u);
    wdg.save(std::span<const std::uint32_t>(overflow));
    REQUIRE(wdg.saved_word_count() == kWatchdogPersistentWords);

    std::array<std::uint32_t, kWatchdogPersistentWords> clipped{};
    wdg.load(std::span<std::uint32_t>(clipped));
    REQUIRE(clipped[0] == 0xA5A5A5A5u);
    REQUIRE(clipped[kWatchdogPersistentWords - 1] == 0xA5A5A5A5u);
}

TEST_CASE("LinuxHalContext board_kind is kLinux", "[hal][hw][linux]") {
    REQUIRE(LinuxHalContext::board_kind() == BoardKind::kLinux);
    LinuxHalContext linux;
    REQUIRE(linux.board_kind() == BoardKind::kLinux);
    REQUIRE(kDefaultBoardKind == BoardKind::kSitl);
}

TEST_CASE("LinuxHalContext UART write/read round-trips via in-memory UartDriver",
          "[hal][hw][linux][uart]") {
    LinuxHalContext linux;
    const std::uint8_t payload[4] = {0x10, 0x20, 0x30, 0x40};
    REQUIRE(linux.uart.write(payload, 4) == 4);

    std::uint8_t drained[4] = {};
    REQUIRE(linux.uart.drain_tx(drained, 4) == 4);
    REQUIRE(linux.uart.inject_rx(drained, 4) == 4);

    std::uint8_t out[4] = {};
    REQUIRE(linux.uart.read(out, 4) == 4);
    REQUIRE(out[0] == 0x10);
    REQUIRE(out[1] == 0x20);
    REQUIRE(out[2] == 0x30);
    REQUIRE(out[3] == 0x40);
}

TEST_CASE("LinuxHalContext GPIO/RC/storage members exist and compile", "[hal][hw][linux]") {
    LinuxHalContext linux(250);
    REQUIRE(linux.now_ms() == 250);
    linux.set_now_ms(1000);
    REQUIRE(linux.now_ms() == 1000);

    linux.gpio.set_pin_mode(3, PinMode::kOutput);
    linux.gpio.write(3, 1);
    REQUIRE(linux.gpio.read(3) == 1);

    linux.rc_input.set_channel(0, 1500);
    REQUIRE(linux.rc_input.read(0) == 1500);

    linux.rc_output.force_safety_off();
    linux.rc_output.write(0, 1600);
    REQUIRE(linux.rc_output.read(0) == 1600);

    const std::uint8_t bytes[3] = {1, 2, 3};
    REQUIRE(linux.storage.write_block(0, bytes, 3));
    std::uint8_t readback[3] = {};
    REQUIRE(linux.storage.read_block(readback, 0, 3));
    REQUIRE(readback[0] == 1);
    REQUIRE(readback[2] == 3);

    linux.watchdog.init();
    REQUIRE(linux.watchdog.is_initialized());
}

TEST_CASE("hw leftover remaining_count matches catalog", "[hal][hw][leftover]") {
    REQUIRE(this_slice_count() == 2);
    REQUIRE(on_main_count() == 8);
    REQUIRE(remaining_count() == 2);
    REQUIRE(out_of_scope_count() == 4);
    REQUIRE(hw_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("BoardKind", PortStatus::kOnMain));
    REQUIRE(completeness_has("Watchdog stub", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL time", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL UART", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL RC", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL GPIO", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL storage", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL Util", PortStatus::kOnMain));
    REQUIRE(completeness_has("Linux backend", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ChibiOS peripherals", PortStatus::kRemaining));
    REQUIRE(completeness_has("ESP32 backend", PortStatus::kRemaining));
    REQUIRE(completeness_has("ChibiOS peripheral drivers", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("hwdef.dat", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("DMA", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("FATFS", PortStatus::kOutOfScope));
}

TEST_CASE("CPP-088 leftover.hpp remaining_count stays 0", "[hal][leftover]") {
    REQUIRE(fwcpp::hal::remaining_count() == 0);
}
