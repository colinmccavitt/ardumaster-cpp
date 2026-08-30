#pragma once

// CPP-090 completeness: closing slice — MAVLink LOG_REQUEST_LIST /
// LOG_ENTRY transfer. remaining_count()==0. DataFlash page map and
// POSIX/SD FileBackend / max-files are on main. LOG_REQUEST_DATA /
// ERASE / END stay out of scope.

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
    {"DataFlash page map", PortStatus::kOnMain,
     "RAM page map BufferToPage/PageToBuffer/ErasePage; separate from MemoryBackend; no NAND/SPI"},
    {"POSIX/SD file backend", PortStatus::kOnMain,
     "FileBackend fwrite/write seam; caller-owned FILE*/fd; no ringbuffer/io_timer"},
    {"FMT registry", PortStatus::kOnMain,
     "LogStructure / log_Format, Fill_Format, Write_Format, lookup by name/type"},
    {"streaming", PortStatus::kOnMain,
     "WriteStreaming rate-limit gate (should_log_streaming, 1000/rate_hz ms)"},
    {"transfer", PortStatus::kThisSlice,
     "MAVLink LOG_REQUEST_LIST / LOG_ENTRY listing via LogDirectory"},
    {"EraseAll", PortStatus::kOnMain,
     "armed-gate + rewind/truncate of the open stream; no directory walk"},
    {"max-files rotation", PortStatus::kOnMain,
     "in-memory log slot table, LASTLOG, next_log_number wrap, erase_next"},
    {"LOG_REQUEST_DATA/ERASE/END", PortStatus::kOutOfScope,
     "msgid 119/121/122 stubs only; no LOG_DATA stream or erase-via-MAVLink"},
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
