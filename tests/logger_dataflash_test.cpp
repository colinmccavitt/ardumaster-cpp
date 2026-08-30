#include <catch2/catch_test_macros.hpp>
#include <fwcpp/logger/logger.hpp>

#include <array>
#include <cstdint>

using fwcpp::logger::DataFlashPageMap;
using fwcpp::logger::PortStatus;
using fwcpp::logger::completeness_has;
using fwcpp::logger::finish_write_page;
using fwcpp::logger::kDfPageSize;
using fwcpp::logger::kDfTestPageCount;
using fwcpp::logger::remaining_count;

TEST_CASE("DataFlash page map BufferToPage / PageToBuffer roundtrip", "[logger][dataflash]") {
    DataFlashPageMap<> map;
    REQUIRE(map.page_size() == kDfPageSize);
    REQUIRE(map.page_count() == kDfTestPageCount);
    REQUIRE(kDfPageSize == 256);
    REQUIRE(kDfTestPageCount == 16);

    std::array<std::uint8_t, kDfPageSize> pattern{};
    for (std::size_t i = 0; i < pattern.size(); ++i) {
        pattern[i] = static_cast<std::uint8_t>(i & 0xFF);
    }
    map.fill_buffer(pattern);
    REQUIRE(map.BufferToPage(1));
    REQUIRE(map.BufferToPage(7));

    // Overwrite buffer, then reload page 1.
    map.fill_buffer(std::array<std::uint8_t, 4>{0xAA, 0xBB, 0xCC, 0xDD});
    REQUIRE(map.PageToBuffer(1));
    auto buf = map.buffer();
    REQUIRE(buf.size() == kDfPageSize);
    for (std::size_t i = 0; i < buf.size(); ++i) {
        REQUIRE(buf[i] == static_cast<std::uint8_t>(i & 0xFF));
    }

    // Page 7 still holds the same pattern.
    REQUIRE(map.PageToBuffer(7));
    for (std::size_t i = 0; i < map.buffer().size(); ++i) {
        REQUIRE(map.buffer()[i] == static_cast<std::uint8_t>(i & 0xFF));
    }
}

TEST_CASE("DataFlash page map ErasePage zero-fills", "[logger][dataflash]") {
    DataFlashPageMap<8, 4> map;
    std::array<std::uint8_t, 8> ones{};
    ones.fill(0x11);
    map.fill_buffer(ones);
    REQUIRE(map.BufferToPage(2));

    auto stored = map.page_bytes(2);
    REQUIRE(stored.size() == 8);
    REQUIRE(stored[0] == 0x11);

    map.ErasePage(2);
    stored = map.page_bytes(2);
    for (auto b : stored) {
        REQUIRE(b == 0);
    }

    REQUIRE(map.PageToBuffer(2));
    for (auto b : map.buffer()) {
        REQUIRE(b == 0);
    }
}

TEST_CASE("DataFlash page map rejects invalid pages", "[logger][dataflash]") {
    DataFlashPageMap<16, 4> map;
    std::array<std::uint8_t, 16> data{};
    data.fill(0x5A);
    map.fill_buffer(data);

    REQUIRE_FALSE(map.BufferToPage(0));
    REQUIRE_FALSE(map.BufferToPage(5));
    REQUIRE(map.BufferToPage(4));

    REQUIRE_FALSE(map.PageToBuffer(0));
    for (auto b : map.buffer()) {
        REQUIRE(b == 0);
    }
    REQUIRE_FALSE(map.PageToBuffer(99));

    map.ErasePage(0);  // no-op
    map.ErasePage(99);
}

TEST_CASE("finish_write_page flushes and wraps", "[logger][dataflash]") {
    DataFlashPageMap<4, 3> map;
    map.fill_buffer(std::array<std::uint8_t, 4>{1, 2, 3, 4});

    REQUIRE(finish_write_page(map, 1) == 2);
    REQUIRE(finish_write_page(map, 2) == 3);
    REQUIRE(finish_write_page(map, 3) == 1);  // wrap past PageCount
    REQUIRE(finish_write_page(map, 0) == 0);  // rejected

    REQUIRE(map.PageToBuffer(3));
    REQUIRE(map.buffer()[0] == 1);
    REQUIRE(map.buffer()[3] == 4);
}

TEST_CASE("leftover remaining_count is 0 after transfer close", "[logger][dataflash][completeness]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(completeness_has("DataFlash page map", PortStatus::kOnMain));
    REQUIRE(completeness_has("transfer", PortStatus::kThisSlice));
    REQUIRE(completeness_has("max-files rotation", PortStatus::kOnMain));
    REQUIRE_FALSE(completeness_has("transfer", PortStatus::kRemaining));
}
