#include <catch2/catch_test_macros.hpp>
#include <fwcpp/logger/logger.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

using fwcpp::logger::MemoryBackend;

TEST_CASE("MemoryBackend WriteBlock records blocks", "[logger][backend]") {
    MemoryBackend<16> log;
    const std::uint8_t head[] = {'D', 'F'};
    const std::uint8_t rest[] = {0x80, 0x01};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(head, sizeof(head))));
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(rest, sizeof(rest))));
    REQUIRE(log.size() == 4);
    REQUIRE(log.num_dropped() == 0);
}

TEST_CASE("MemoryBackend WriteBlock round-trips bytes", "[logger][backend]") {
    MemoryBackend<32> log;
    log.StartWrite(1);
    const std::uint8_t head[] = {'H', 'E', 'A', 'D'};
    const std::uint8_t payload[] = {0x10, 0x20, 0x30};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(head, sizeof(head))));
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(payload, sizeof(payload))));
    log.EndWrite();

    const std::array<std::uint8_t, 7> expected = {
        'H', 'E', 'A', 'D', 0x10, 0x20, 0x30,
    };
    const auto rec = log.recorded();
    REQUIRE(rec.size() == expected.size());
    REQUIRE(std::equal(rec.begin(), rec.end(), expected.begin()));
    REQUIRE(log.page_adr() == 1);
    REQUIRE(log.ended_writes() == 1);
    REQUIRE_FALSE(log.is_writing());
}

TEST_CASE("MemoryBackend drops and counts when the cap is full", "[logger][backend][drop]") {
    MemoryBackend<2> log;
    const std::uint8_t first[] = {1, 2};
    const std::uint8_t extra[] = {3};
    const std::uint8_t too_big[] = {4, 5};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(first, sizeof(first))));
    REQUIRE(log.num_dropped() == 0);
    REQUIRE_FALSE(log.WriteBlock(std::span<const std::uint8_t>(extra, sizeof(extra))));
    REQUIRE(log.num_dropped() == 1);
    REQUIRE_FALSE(log.WriteBlock(std::span<const std::uint8_t>(too_big, sizeof(too_big))));
    REQUIRE(log.num_dropped() == 2);

    const auto rec = log.recorded();
    REQUIRE(rec.size() == 2);
    REQUIRE(rec[0] == 1);
    REQUIRE(rec[1] == 2);
    REQUIRE(log.capacity() == 2);
}

TEST_CASE("StartWrite and EndWrite bookend a page write", "[logger][backend]") {
    MemoryBackend<8> log;
    REQUIRE_FALSE(log.is_writing());
    log.StartWrite(7);
    REQUIRE(log.page_adr() == 7);
    REQUIRE(log.is_writing());
    const std::uint8_t ab[] = {'A', 'B'};
    REQUIRE(log.WriteBlock(std::span<const std::uint8_t>(ab, sizeof(ab))));
    log.EndWrite();
    REQUIRE_FALSE(log.is_writing());
    REQUIRE(log.ended_writes() == 1);
    REQUIRE(log.page_adr() == 7);
    const auto rec = log.recorded();
    REQUIRE(rec.size() == 2);
    REQUIRE(rec[0] == 'A');
    REQUIRE(rec[1] == 'B');
}
