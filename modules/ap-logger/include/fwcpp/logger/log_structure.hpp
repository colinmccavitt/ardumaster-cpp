#pragma once

// Port of AP_Logger LogStructure / log_Format and
// AP_Logger_Backend::Fill_Format / Write_Format. CPP-090, slice 2.
//
// LogStructure holds the fields Fill_Format reads (upstream LogStructure.h
// msg_type, msg_len, name, format, labels). log_Format is the on-wire FMT
// packet (LogStructure.h:187). Fill_Format lives in LogFile.cpp; Write_Format
// then WriteCriticalBlock's the packet. Here Write_Format calls
// MemoryBackend::WriteBlock with the packed bytes. No vtable (ADR-0012).

#include <fwcpp/logger/memory_backend.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

namespace fwcpp::logger {

inline constexpr std::uint8_t HEAD_BYTE1 = 0xA3;
inline constexpr std::uint8_t HEAD_BYTE2 = 0x95;
inline constexpr std::uint8_t LOG_FORMAT_MSG = 128;

// Maximum lengths of LogStructure string fields, including trailing nulls
// (upstream LS_NAME_SIZE / LS_FORMAT_SIZE / LS_LABELS_SIZE).
inline constexpr std::uint8_t LS_NAME_SIZE = 5;
inline constexpr std::uint8_t LS_FORMAT_SIZE = 17;
inline constexpr std::uint8_t LS_LABELS_SIZE = 65;

struct LogStructure {
    std::uint8_t msg_type;
    std::uint8_t msg_len;
    const char* name;
    const char* format;
    const char* labels;
};

#pragma pack(push, 1)
struct log_Format {
    std::uint8_t head1;
    std::uint8_t head2;
    std::uint8_t msgid;
    std::uint8_t type;
    std::uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};
#pragma pack(pop)

static_assert(sizeof(log_Format) == 89, "upstream packed log_Format is 89 bytes");
static_assert(offsetof(log_Format, head1) == 0);
static_assert(offsetof(log_Format, head2) == 1);
static_assert(offsetof(log_Format, msgid) == 2);
static_assert(offsetof(log_Format, type) == 3);
static_assert(offsetof(log_Format, length) == 4);
static_assert(offsetof(log_Format, name) == 5);
static_assert(offsetof(log_Format, format) == 9);
static_assert(offsetof(log_Format, labels) == 25);

// Dummy registry type (not an upstream msgid). Packet is header + one uint8.
inline constexpr std::uint8_t LOG_DUMMY_MSG = 1;
inline constexpr std::uint8_t kDummyMsgLen = 4;

// FMT describes itself; DUMY is a second static entry so lookup-by-name and
// lookup-by-type have more than one row (upstream LOG_COMMON_STRUCTURES).
inline constexpr LogStructure kLogStructures[] = {
    {LOG_FORMAT_MSG, static_cast<std::uint8_t>(sizeof(log_Format)), "FMT", "BBnNZ",
     "Type,Length,Name,Format,Columns"},
    {LOG_DUMMY_MSG, kDummyMsgLen, "DUMY", "B", "Val"},
};

[[nodiscard]] inline constexpr std::size_t log_structure_count() {
    return sizeof(kLogStructures) / sizeof(kLogStructures[0]);
}

namespace detail {

inline void strncpy_noterm(char* dest, const char* src, std::size_t n) {
    if (src == nullptr || n == 0) {
        return;
    }
    std::size_t len = 0;
    while (len < n && src[len] != '\0') {
        ++len;
    }
    if (len < n) {
        ++len;
    }
    std::memcpy(dest, src, len);
}

[[nodiscard]] inline bool names_equal(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
        return a == b;
    }
    while (*a != '\0' && *b != '\0') {
        if (*a != *b) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

}  // namespace detail

// AP_Logger_Backend::Fill_Format (LogFile.cpp): zero the packet, stamp
// HEAD_BYTE1/HEAD_BYTE2 and msgid LOG_FORMAT_MSG, then copy type/length/name/
// format/labels from the LogStructure (strncpy_noterm, no terminator if full).
inline void Fill_Format(const LogStructure& s, log_Format& pkt) {
    pkt = log_Format{};
    pkt.head1 = HEAD_BYTE1;
    pkt.head2 = HEAD_BYTE2;
    pkt.msgid = LOG_FORMAT_MSG;
    pkt.type = s.msg_type;
    pkt.length = s.msg_len;
    detail::strncpy_noterm(pkt.name, s.name, sizeof(pkt.name));
    detail::strncpy_noterm(pkt.format, s.format, sizeof(pkt.format));
    detail::strncpy_noterm(pkt.labels, s.labels, sizeof(pkt.labels));
}

[[nodiscard]] inline const LogStructure* structure_for_name(const char* name) {
    for (const auto& s : kLogStructures) {
        if (detail::names_equal(s.name, name)) {
            return &s;
        }
    }
    return nullptr;
}

[[nodiscard]] inline const LogStructure* structure_for_msg_type(std::uint8_t msg_type) {
    for (const auto& s : kLogStructures) {
        if (s.msg_type == msg_type) {
            return &s;
        }
    }
    return nullptr;
}

// AP_Logger_Backend::Write_Format: Fill_Format then WriteBlock of the packet.
template <std::size_t Capacity>
[[nodiscard]] bool Write_Format(MemoryBackend<Capacity>& backend, const LogStructure& s) {
    log_Format pkt{};
    Fill_Format(s, pkt);
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&pkt);
    return backend.WriteBlock(std::span<const std::uint8_t>(bytes, sizeof(pkt)));
}

}  // namespace fwcpp::logger
