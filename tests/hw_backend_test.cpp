// CPP-089 slice 1: BoardKind default SITL + compile-only Watchdog stub.

#include <array>
#include <cstdint>
#include <span>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/board.hpp>
#include <fwcpp/hal/hw_leftover.hpp>
#include <fwcpp/hal/watchdog.hpp>

using fwcpp::hal::BoardKind;
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

TEST_CASE("hw leftover remaining_count matches catalog", "[hal][hw][leftover]") {
    REQUIRE(this_slice_count() == 3);
    REQUIRE(on_main_count() == 6);
    REQUIRE(remaining_count() == 3);
    REQUIRE(remaining_count() >= 3);
    REQUIRE(out_of_scope_count() == 4);
    REQUIRE(hw_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("BoardKind", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Watchdog stub", PortStatus::kThisSlice));
    REQUIRE(completeness_has("SITL time", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL UART", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL RC", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL GPIO", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL storage", PortStatus::kOnMain));
    REQUIRE(completeness_has("SITL Util", PortStatus::kOnMain));
    REQUIRE(completeness_has("ChibiOS peripherals", PortStatus::kRemaining));
    REQUIRE(completeness_has("Linux backend", PortStatus::kRemaining));
    REQUIRE(completeness_has("ESP32 backend", PortStatus::kRemaining));
    REQUIRE(completeness_has("ChibiOS peripheral drivers", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("hwdef.dat", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("DMA", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("FATFS", PortStatus::kOutOfScope));
}
