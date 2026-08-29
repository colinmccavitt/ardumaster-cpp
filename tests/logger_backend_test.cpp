#include <catch2/catch_test_macros.hpp>
#include <fwcpp/logger/logger.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
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

TEST_CASE("Fill_Format copies LogStructure into a packed FMT packet", "[logger][fmt]") {
    using fwcpp::logger::Fill_Format;
    using fwcpp::logger::HEAD_BYTE1;
    using fwcpp::logger::HEAD_BYTE2;
    using fwcpp::logger::LOG_FORMAT_MSG;
    using fwcpp::logger::LogStructure;
    using fwcpp::logger::log_Format;

    REQUIRE(offsetof(log_Format, head1) == 0);
    REQUIRE(offsetof(log_Format, head2) == 1);
    REQUIRE(offsetof(log_Format, msgid) == 2);
    REQUIRE(offsetof(log_Format, type) == 3);
    REQUIRE(offsetof(log_Format, length) == 4);
    REQUIRE(offsetof(log_Format, name) == 5);
    REQUIRE(offsetof(log_Format, format) == 9);
    REQUIRE(offsetof(log_Format, labels) == 25);
    REQUIRE(sizeof(log_Format) == 89);

    const LogStructure s{1, 12, "DUMY", "Qf", "TimeUS,Val"};
    log_Format pkt{};
    Fill_Format(s, pkt);

    REQUIRE(pkt.head1 == HEAD_BYTE1);
    REQUIRE(pkt.head2 == HEAD_BYTE2);
    REQUIRE(pkt.msgid == LOG_FORMAT_MSG);
    REQUIRE(pkt.type == 1);
    REQUIRE(pkt.length == 12);
    REQUIRE(std::memcmp(pkt.name, "DUMY", 4) == 0);
    REQUIRE(std::memcmp(pkt.format, "Qf", 3) == 0);
    REQUIRE(std::memcmp(pkt.labels, "TimeUS,Val", 11) == 0);
}

TEST_CASE("Write_Format round-trips a FMT packet through MemoryBackend", "[logger][fmt]") {
    using fwcpp::logger::Fill_Format;
    using fwcpp::logger::LOG_FORMAT_MSG;
    using fwcpp::logger::Write_Format;
    using fwcpp::logger::log_Format;
    using fwcpp::logger::structure_for_name;

    MemoryBackend<128> log;
    const auto* fmt = structure_for_name("FMT");
    REQUIRE(fmt != nullptr);
    REQUIRE(fmt->msg_type == LOG_FORMAT_MSG);
    REQUIRE(Write_Format(log, *fmt));

    log_Format expected{};
    Fill_Format(*fmt, expected);
    const auto rec = log.recorded();
    REQUIRE(rec.size() == sizeof(log_Format));
    REQUIRE(std::memcmp(rec.data(), &expected, sizeof(expected)) == 0);
    REQUIRE(rec[0] == fwcpp::logger::HEAD_BYTE1);
    REQUIRE(rec[1] == fwcpp::logger::HEAD_BYTE2);
    REQUIRE(rec[2] == LOG_FORMAT_MSG);
    REQUIRE(log.num_dropped() == 0);
}

TEST_CASE("FMT registry lookup by name and msg type", "[logger][fmt]") {
    using fwcpp::logger::LOG_DUMMY_MSG;
    using fwcpp::logger::LOG_FORMAT_MSG;
    using fwcpp::logger::log_structure_count;
    using fwcpp::logger::structure_for_msg_type;
    using fwcpp::logger::structure_for_name;

    REQUIRE(log_structure_count() >= 2);

    const auto* fmt = structure_for_name("FMT");
    REQUIRE(fmt != nullptr);
    REQUIRE(fmt->msg_type == LOG_FORMAT_MSG);
    REQUIRE(structure_for_msg_type(LOG_FORMAT_MSG) == fmt);

    const auto* dumy = structure_for_name("DUMY");
    REQUIRE(dumy != nullptr);
    REQUIRE(dumy->msg_type == LOG_DUMMY_MSG);
    REQUIRE(structure_for_msg_type(LOG_DUMMY_MSG) == dumy);

    REQUIRE(structure_for_name("NOPE") == nullptr);
    REQUIRE(structure_for_msg_type(255) == nullptr);
}
