#include <catch2/catch_test_macros.hpp>
#include <fwcpp/logger/logger.hpp>

#include <cstdint>
#include <cstdio>

using fwcpp::logger::FileBackend;
using fwcpp::logger::LogDirectory;
using fwcpp::logger::PortStatus;
using fwcpp::logger::completeness_has;
using fwcpp::logger::get_max_num_logs;
using fwcpp::logger::kMaxLogFiles;
using fwcpp::logger::kMinLogFiles;
using fwcpp::logger::remaining_count;

TEST_CASE("get_max_num_logs constrains to [2, 500]", "[logger][rotation]") {
    REQUIRE(kMinLogFiles == 2);
    REQUIRE(kMaxLogFiles == 500);
    REQUIRE(get_max_num_logs(1) == 2);
    REQUIRE(get_max_num_logs(501) == 500);
    REQUIRE(get_max_num_logs(2) == 2);
    REQUIRE(get_max_num_logs(500) == 500);
    REQUIRE(get_max_num_logs(50) == 50);
    REQUIRE(get_max_num_logs(0) == 2);

    LogDirectory clamped_low(1);
    REQUIRE(clamped_low.max_num_logs() == 2);
    LogDirectory clamped_high(501);
    REQUIRE(clamped_high.max_num_logs() == 500);
}

TEST_CASE("find_last_log is 0 when LASTLOG is empty or missing", "[logger][rotation]") {
    LogDirectory dir;
    REQUIRE(dir.find_last_log() == 0);
    REQUIRE_FALSE(dir.last_log_is_marked_discard());

    dir.load_lastlog_text("");
    REQUIRE(dir.find_last_log() == 0);
    REQUIRE_FALSE(dir.last_log_is_marked_discard());

    dir.load_lastlog_text(nullptr);
    REQUIRE(dir.find_last_log() == 0);
}

TEST_CASE("find_last_log stores LASTLOG number and optional D flag", "[logger][rotation]") {
    LogDirectory dir;
    dir.write_lastlog(7);
    REQUIRE(dir.find_last_log() == 7);
    REQUIRE_FALSE(dir.last_log_is_marked_discard());

    dir.load_lastlog_text("12D\r\n");
    REQUIRE(dir.find_last_log() == 12);
    REQUIRE(dir.last_log_is_marked_discard());

    dir.load_lastlog_text("3\r\n");
    REQUIRE(dir.find_last_log() == 3);
    REQUIRE_FALSE(dir.last_log_is_marked_discard());
}

TEST_CASE("next_log_number increments when the last slot has size", "[logger][rotation]") {
    LogDirectory dir(4);
    REQUIRE(dir.next_log_number() == 1);

    dir.write_lastlog(1);
    REQUIRE(dir.log_size(1) == 0);
    REQUIRE(dir.next_log_number() == 1);

    dir.occupy(1, 128);
    REQUIRE(dir.log_size(1) == 128);
    REQUIRE(dir.next_log_number() == 2);

    dir.write_lastlog(2);
    dir.occupy(2, 64);
    REQUIRE(dir.next_log_number() == 3);
}

TEST_CASE("next_log_number wraps at max", "[logger][rotation]") {
    LogDirectory dir(2);
    dir.write_lastlog(2);
    dir.occupy(2, 16);
    REQUIRE(dir.next_log_number() == 1);

    LogDirectory full(500);
    full.write_lastlog(500);
    full.occupy(500, 1);
    REQUIRE(full.next_log_number() == 1);
}

TEST_CASE("erase_next walk clears slots 1..max then LASTLOG", "[logger][rotation]") {
    LogDirectory dir(2);
    dir.occupy(1, 10);
    dir.occupy(2, 20);
    dir.write_lastlog(2, true);
    REQUIRE(dir.log_size(1) == 10);
    REQUIRE(dir.log_size(2) == 20);
    REQUIRE(dir.find_last_log() == 2);
    REQUIRE(dir.last_log_is_marked_discard());

    dir.EraseAll();
    REQUIRE(dir.erase_log_num() == 1);

    std::uint16_t steps = 0;
    while (dir.erase_log_num() != 0) {
        dir.erase_next();
        ++steps;
        REQUIRE(steps <= 3);
    }
    REQUIRE(steps == 2);
    REQUIRE(dir.erase_log_num() == 0);
    REQUIRE(dir.log_size(1) == 0);
    REQUIRE(dir.log_size(2) == 0);
    REQUIRE(dir.find_last_log() == 0);
    REQUIRE_FALSE(dir.last_log_is_marked_discard());
}

TEST_CASE("FileBackend EraseAll pokes injected LogDirectory", "[logger][rotation][erase]") {
    LogDirectory dir(2);
    dir.occupy(1, 5);
    dir.write_lastlog(1);

    std::FILE* f = std::tmpfile();
    REQUIRE(f != nullptr);
    FileBackend log(f);
    log.attach_directory(&dir);

    log.EraseAll(true);
    REQUIRE(dir.erase_log_num() == 0);
    REQUIRE(dir.log_size(1) == 5);

    log.EraseAll(false);
    REQUIRE(dir.erase_log_num() == 1);
    while (dir.erase_log_num() != 0) {
        dir.erase_next();
    }
    REQUIRE(dir.log_size(1) == 0);
    REQUIRE(dir.find_last_log() == 0);
    std::fclose(f);
}

TEST_CASE("leftover remaining_count is 0 after transfer close", "[logger][rotation][completeness]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(completeness_has("DataFlash page map", PortStatus::kOnMain));
    REQUIRE(completeness_has("transfer", PortStatus::kThisSlice));
    REQUIRE(completeness_has("POSIX/SD file backend", PortStatus::kOnMain));
    REQUIRE(completeness_has("EraseAll", PortStatus::kOnMain));
    REQUIRE(completeness_has("max-files rotation", PortStatus::kOnMain));
}
