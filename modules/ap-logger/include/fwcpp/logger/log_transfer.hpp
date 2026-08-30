#pragma once

// Port of AP_Logger_MAVLinkLogTransfer handle_log_request_list / LOG_ENTRY.
// CPP-090, slice 8 (closing).
//
// Upstream: AP_Logger_MAVLinkLogTransfer.cpp handle_log_request_list (~68+)
// and handle_log_send_listing (~259-291). common.xml / generated headers:
// LOG_REQUEST_LIST msgid 117 CRC 128; LOG_ENTRY msgid 118 CRC 56.
//
// Injects LogDirectory (slot table on main) for sizes. time_utc is injected
// by the caller (no POSIX mtime). Emits all LOG_ENTRY payloads for the
// clamped range in one call (no GCS link / payload-space / heartbeat gate).
// LOG_REQUEST_DATA / LOG_ERASE / LOG_REQUEST_END are out of scope stubs.

#include <cstddef>
#include <cstdint>
#include <span>

#include <fwcpp/logger/log_directory.hpp>

namespace fwcpp::logger {

inline constexpr std::uint32_t kMsgIdLogRequestList = 117;
inline constexpr std::uint32_t kMsgIdLogEntry = 118;
inline constexpr std::uint32_t kMsgIdLogRequestData = 119;
inline constexpr std::uint32_t kMsgIdLogErase = 121;
inline constexpr std::uint32_t kMsgIdLogRequestEnd = 122;

// mavlink_msg_log_request_list.h / mavlink_msg_log_entry.h CRC extras.
inline constexpr std::uint8_t kLogRequestListCrcExtra = 128;
inline constexpr std::uint8_t kLogEntryCrcExtra = 56;

inline constexpr std::size_t kLogRequestListLen = 6;
inline constexpr std::size_t kLogEntryLen = 14;

// Size-sorted v2 wire: start/end uint16, target_system, target_component.
struct LogRequestList {
    std::uint16_t start{};
    std::uint16_t end{};
    std::uint8_t target_system{};
    std::uint8_t target_component{};
};

// Size-sorted v2 wire: time_utc/size uint32, id/num_logs/last_log_num uint16.
struct LogEntry {
    std::uint32_t time_utc{};
    std::uint32_t size{};
    std::uint16_t id{};
    std::uint16_t num_logs{};
    std::uint16_t last_log_num{};
};

namespace detail {

inline void write_u16_le(std::uint8_t* p, std::uint16_t v) {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}

[[nodiscard]] inline std::uint16_t read_u16_le(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}

inline void write_u32_le(std::uint8_t* p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
    p[2] = static_cast<std::uint8_t>(v >> 16);
    p[3] = static_cast<std::uint8_t>(v >> 24);
}

[[nodiscard]] inline std::uint32_t read_u32_le(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

}  // namespace detail

[[nodiscard]] inline std::size_t pack_log_request_list(const LogRequestList& req,
                                                       std::span<std::uint8_t> buf) {
    if (buf.size() < kLogRequestListLen) {
        return 0;
    }
    detail::write_u16_le(buf.data() + 0, req.start);
    detail::write_u16_le(buf.data() + 2, req.end);
    buf[4] = req.target_system;
    buf[5] = req.target_component;
    return kLogRequestListLen;
}

[[nodiscard]] inline bool unpack_log_request_list(std::span<const std::uint8_t> buf,
                                                  LogRequestList& out) {
    if (buf.size() < kLogRequestListLen) {
        return false;
    }
    out.start = detail::read_u16_le(buf.data() + 0);
    out.end = detail::read_u16_le(buf.data() + 2);
    out.target_system = buf[4];
    out.target_component = buf[5];
    return true;
}

[[nodiscard]] inline std::size_t pack_log_entry(const LogEntry& entry, std::span<std::uint8_t> buf) {
    if (buf.size() < kLogEntryLen) {
        return 0;
    }
    detail::write_u32_le(buf.data() + 0, entry.time_utc);
    detail::write_u32_le(buf.data() + 4, entry.size);
    detail::write_u16_le(buf.data() + 8, entry.id);
    detail::write_u16_le(buf.data() + 10, entry.num_logs);
    detail::write_u16_le(buf.data() + 12, entry.last_log_num);
    return kLogEntryLen;
}

[[nodiscard]] inline bool unpack_log_entry(std::span<const std::uint8_t> buf, LogEntry& out) {
    if (buf.size() < kLogEntryLen) {
        return false;
    }
    out.time_utc = detail::read_u32_le(buf.data() + 0);
    out.size = detail::read_u32_le(buf.data() + 4);
    out.id = detail::read_u16_le(buf.data() + 8);
    out.num_logs = detail::read_u16_le(buf.data() + 10);
    out.last_log_num = detail::read_u16_le(buf.data() + 12);
    return true;
}

// Count occupied slots (size > 0). Stands in for backends[0]->get_num_logs().
[[nodiscard]] inline std::uint16_t get_num_logs(const LogDirectory& dir) {
    std::uint16_t n = 0;
    const std::uint16_t max_n = dir.max_num_logs();
    for (std::uint16_t log_num = 1; log_num <= max_n; ++log_num) {
        if (dir.log_size(log_num) > 0) {
            ++n;
        }
    }
    return n;
}

// 1-based list entry → physical log_num among occupied slots (ascending).
// 0 if list_entry is out of range / empty directory.
[[nodiscard]] inline std::uint16_t log_num_from_list_entry(const LogDirectory& dir,
                                                           std::uint16_t list_entry) {
    if (list_entry == 0) {
        return 0;
    }
    std::uint16_t seen = 0;
    const std::uint16_t max_n = dir.max_num_logs();
    for (std::uint16_t log_num = 1; log_num <= max_n; ++log_num) {
        if (dir.log_size(log_num) == 0) {
            continue;
        }
        ++seen;
        if (seen == list_entry) {
            return log_num;
        }
    }
    return 0;
}

// size from LogDirectory; time_utc from inject table indexed by log_num
// (index 0 unused). Missing inject → 0.
[[nodiscard]] inline bool get_log_info(const LogDirectory& dir, std::uint16_t list_entry,
                                       std::span<const std::uint32_t> time_utc_by_log_num,
                                       std::uint32_t& size, std::uint32_t& time_utc) {
    const std::uint16_t log_num = log_num_from_list_entry(dir, list_entry);
    if (log_num == 0) {
        size = 0;
        time_utc = 0;
        return false;
    }
    size = dir.log_size(log_num);
    if (log_num < time_utc_by_log_num.size()) {
        time_utc = time_utc_by_log_num[log_num];
    } else {
        time_utc = 0;
    }
    return true;
}

// handle_log_request_list + listing: fill out_entries with LOG_ENTRY fields
// for each list id in the clamped [start,end] range. Returns count written
// (0 if out_entries too small for the needed entries). When num_logs==0,
// writes one zeroed entry (id=0) like upstream's empty listing.
[[nodiscard]] inline std::size_t handle_log_request_list(
    const LogDirectory& dir, const LogRequestList& req, std::span<LogEntry> out_entries,
    std::span<const std::uint32_t> time_utc_by_log_num) {
    const std::uint16_t num_logs = get_num_logs(dir);
    std::uint16_t next = 0;
    std::uint16_t last = 0;
    if (num_logs == 0) {
        next = 0;
        last = 0;
    } else {
        next = req.start;
        last = req.end;
        if (last > num_logs) {
            last = num_logs;
        }
        if (next < 1) {
            next = 1;
        }
        if (next > last) {
            return 0;
        }
    }

    const std::size_t needed =
        (num_logs == 0) ? 1 : static_cast<std::size_t>(last - next) + 1;
    if (out_entries.size() < needed) {
        return 0;
    }

    for (std::size_t i = 0; i < needed; ++i) {
        const std::uint16_t id =
            (num_logs == 0) ? 0 : static_cast<std::uint16_t>(next + static_cast<std::uint16_t>(i));
        std::uint32_t size = 0;
        std::uint32_t time_utc = 0;
        if (id != 0) {
            (void)get_log_info(dir, id, time_utc_by_log_num, size, time_utc);
        }
        out_entries[i] = LogEntry{};
        out_entries[i].time_utc = time_utc;
        out_entries[i].size = size;
        out_entries[i].id = id;
        out_entries[i].num_logs = num_logs;
        out_entries[i].last_log_num = last;
    }
    return needed;
}

// OOS stubs — LOG_REQUEST_DATA / ERASE / END not ported this ticket.
[[nodiscard]] inline constexpr bool handle_log_request_data() { return false; }
[[nodiscard]] inline constexpr bool handle_log_request_erase() { return false; }
[[nodiscard]] inline constexpr bool handle_log_request_end() { return false; }

}  // namespace fwcpp::logger
