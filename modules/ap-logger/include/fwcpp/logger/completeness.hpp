#pragma once

// CPP-090 completeness: this slice (WriteStreaming rate-limit gate) vs
// remaining AP_Logger / DataFlash surfaces. remaining_count() > 0 until
// later slices land the DataFlash page map, POSIX/SD, transfer, and
// erase/rotate.

#include <cstddef>
#include <cstdint>

namespace fwcpp::logger {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct LoggerPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr LoggerPortItem kLoggerCompleteness[] = {
    {"backend", PortStatus::kOnMain,
     "LogBackend WriteBlock/StartWrite/EndWrite + MemoryBackend"},
    {"drop", PortStatus::kOnMain,
     "buffer-full drop counter (upstream _dropped / num_dropped)"},
    {"completeness catalog", PortStatus::kOnMain, "this table"},
    {"DataFlash page map", PortStatus::kRemaining,
     "page-based DataFlash layout; BufferToPage / PageToBuffer (NAND)"},
    {"POSIX/SD file backend", PortStatus::kRemaining,
     "AP_Logger_File on a real filesystem; no filesystem in this slice"},
    {"FMT registry", PortStatus::kOnMain,
     "LogStructure / log_Format, Fill_Format, Write_Format, lookup by name/type"},
    {"streaming", PortStatus::kThisSlice,
     "WriteStreaming rate-limit gate (should_log_streaming, 1000/rate_hz ms)"},
    {"transfer", PortStatus::kRemaining,
     "MAVLink LOG_REQUEST_LIST / LOG_ENTRY listing"},
    {"erase/rotate", PortStatus::kRemaining,
     "LOG_ERASE / EraseAll and max-files rotation"},
};

[[nodiscard]] inline constexpr std::size_t logger_completeness_size() {
    return sizeof(kLoggerCompleteness) / sizeof(kLoggerCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kLoggerCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kLoggerCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}
[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}

}  // namespace fwcpp::logger
