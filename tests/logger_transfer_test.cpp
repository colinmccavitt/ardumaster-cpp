#include <catch2/catch_test_macros.hpp>
#include <fwcpp/logger/logger.hpp>

#include <array>
#include <cstdint>

using fwcpp::logger::LogDirectory;
using fwcpp::logger::LogEntry;
using fwcpp::logger::LogRequestList;
using fwcpp::logger::PortStatus;
using fwcpp::logger::completeness_has;
using fwcpp::logger::get_num_logs;
using fwcpp::logger::handle_log_request_data;
using fwcpp::logger::handle_log_request_end;
using fwcpp::logger::handle_log_request_erase;
using fwcpp::logger::handle_log_request_list;
using fwcpp::logger::kLogEntryCrcExtra;
using fwcpp::logger::kLogEntryLen;
using fwcpp::logger::kLogRequestListCrcExtra;
using fwcpp::logger::kLogRequestListLen;
using fwcpp::logger::kMsgIdLogEntry;
using fwcpp::logger::kMsgIdLogErase;
using fwcpp::logger::kMsgIdLogRequestData;
using fwcpp::logger::kMsgIdLogRequestEnd;
using fwcpp::logger::kMsgIdLogRequestList;
using fwcpp::logger::log_num_from_list_entry;
using fwcpp::logger::pack_log_entry;
using fwcpp::logger::pack_log_request_list;
using fwcpp::logger::remaining_count;
using fwcpp::logger::unpack_log_entry;
using fwcpp::logger::unpack_log_request_list;

TEST_CASE("LOG_REQUEST_LIST / LOG_ENTRY msgid and CRC match mavlink headers",
          "[logger][transfer]") {
    REQUIRE(kMsgIdLogRequestList == 117);
    REQUIRE(kMsgIdLogEntry == 118);
    REQUIRE(kLogRequestListCrcExtra == 128);
    REQUIRE(kLogEntryCrcExtra == 56);
    REQUIRE(kLogRequestListLen == 6);
    REQUIRE(kLogEntryLen == 14);
    REQUIRE(kMsgIdLogRequestData == 119);
    REQUIRE(kMsgIdLogErase == 121);
    REQUIRE(kMsgIdLogRequestEnd == 122);
}

TEST_CASE("LOG_REQUEST_LIST pack/unpack roundtrip", "[logger][transfer]") {
    LogRequestList req{};
    req.start = 1;
    req.end = 0xFFFF;
    req.target_system = 1;
    req.target_component = 1;

    std::array<std::uint8_t, kLogRequestListLen> buf{};
    REQUIRE(pack_log_request_list(req, buf) == kLogRequestListLen);

    LogRequestList decoded{};
    REQUIRE(unpack_log_request_list(buf, decoded));
    REQUIRE(decoded.start == 1);
    REQUIRE(decoded.end == 0xFFFF);
    REQUIRE(decoded.target_system == 1);
    REQUIRE(decoded.target_component == 1);
}

TEST_CASE("LOG_ENTRY pack/unpack roundtrip", "[logger][transfer]") {
    LogEntry entry{};
    entry.time_utc = 1700000000;
    entry.size = 4096;
    entry.id = 2;
    entry.num_logs = 3;
    entry.last_log_num = 3;

    std::array<std::uint8_t, kLogEntryLen> buf{};
    REQUIRE(pack_log_entry(entry, buf) == kLogEntryLen);

    LogEntry decoded{};
    REQUIRE(unpack_log_entry(buf, decoded));
    REQUIRE(decoded.time_utc == 1700000000);
    REQUIRE(decoded.size == 4096);
    REQUIRE(decoded.id == 2);
    REQUIRE(decoded.num_logs == 3);
    REQUIRE(decoded.last_log_num == 3);
}

TEST_CASE("handle_log_request_list empty directory emits zero entry", "[logger][transfer]") {
    LogDirectory dir(4);
    LogRequestList req{};
    req.start = 0;
    req.end = 0xFFFF;

    std::array<LogEntry, 4> out{};
    std::array<std::uint32_t, 5> times{};
    REQUIRE(handle_log_request_list(dir, req, out, times) == 1);
    REQUIRE(out[0].id == 0);
    REQUIRE(out[0].num_logs == 0);
    REQUIRE(out[0].last_log_num == 0);
    REQUIRE(out[0].size == 0);
    REQUIRE(out[0].time_utc == 0);
}

TEST_CASE("handle_log_request_list emits LOG_ENTRY for range with time_utc inject",
          "[logger][transfer]") {
    LogDirectory dir(8);
    dir.occupy(1, 100);
    dir.occupy(3, 300);  // gap at 2 — list entries skip empty slots
    dir.occupy(5, 500);
    dir.write_lastlog(5);

    REQUIRE(get_num_logs(dir) == 3);
    REQUIRE(log_num_from_list_entry(dir, 1) == 1);
    REQUIRE(log_num_from_list_entry(dir, 2) == 3);
    REQUIRE(log_num_from_list_entry(dir, 3) == 5);

    // Indexed by physical log_num (slot 0 unused).
    std::array<std::uint32_t, 6> times{};
    times[1] = 111;
    times[3] = 333;
    times[5] = 555;

    LogRequestList req{};
    req.start = 0;  // clamp to 1
    req.end = 0xFFFF;

    std::array<LogEntry, 8> out{};
    REQUIRE(handle_log_request_list(dir, req, out, times) == 3);

    REQUIRE(out[0].id == 1);
    REQUIRE(out[0].num_logs == 3);
    REQUIRE(out[0].last_log_num == 3);
    REQUIRE(out[0].size == 100);
    REQUIRE(out[0].time_utc == 111);

    REQUIRE(out[1].id == 2);
    REQUIRE(out[1].size == 300);
    REQUIRE(out[1].time_utc == 333);
    REQUIRE(out[1].num_logs == 3);
    REQUIRE(out[1].last_log_num == 3);

    REQUIRE(out[2].id == 3);
    REQUIRE(out[2].size == 500);
    REQUIRE(out[2].time_utc == 555);
    REQUIRE(out[2].last_log_num == 3);
}

TEST_CASE("handle_log_request_list clamps end and start", "[logger][transfer]") {
    LogDirectory dir(4);
    dir.occupy(1, 10);
    dir.occupy(2, 20);

    std::array<std::uint32_t, 3> times{};
    times[1] = 1;
    times[2] = 2;

    LogRequestList req{};
    req.start = 2;
    req.end = 99;

    std::array<LogEntry, 4> out{};
    REQUIRE(handle_log_request_list(dir, req, out, times) == 1);
    REQUIRE(out[0].id == 2);
    REQUIRE(out[0].num_logs == 2);
    REQUIRE(out[0].last_log_num == 2);
    REQUIRE(out[0].size == 20);
    REQUIRE(out[0].time_utc == 2);
}

TEST_CASE("LOG_REQUEST_DATA/ERASE/END stubs are OOS", "[logger][transfer]") {
    REQUIRE_FALSE(handle_log_request_data());
    REQUIRE_FALSE(handle_log_request_erase());
    REQUIRE_FALSE(handle_log_request_end());
}

TEST_CASE("leftover remaining_count is 0 after transfer", "[logger][transfer][completeness]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(completeness_has("transfer", PortStatus::kThisSlice));
    REQUIRE(completeness_has("DataFlash page map", PortStatus::kOnMain));
    REQUIRE(completeness_has("LOG_REQUEST_DATA/ERASE/END", PortStatus::kOutOfScope));
}
