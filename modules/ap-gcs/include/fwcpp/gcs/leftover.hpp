#pragma once

// CPP-087 slice 5 completeness catalog.
//
// On main: MAVLink 2 framing, HEARTBEAT pack/unpack, msgid dispatch stub,
// leftover catalog, COMMAND_LONG ARM/DISARM + DO_SET_MODE + COMMAND_ACK,
// PARAM_REQUEST_LIST / PARAM_SET / PARAM_VALUE, MISSION_ITEM_INT /
// MISSION_REQUEST_INT.
// This slice: Plane vehicle handler leftover_send_attitude (ATTITUDE 30).
// Remaining: XML dialect generation. Copter vehicle handlers noted on the
// vehicle row (not a separate remaining row this slice).
// remaining_count() is 1.

#include <cstddef>
#include <cstdint>

namespace fwcpp::gcs {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
};

struct GcsPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr GcsPortItem kGcsCompleteness[] = {
    {"MAVLink 2 framing", PortStatus::kOnMain,
     "stx 0xFD, flags, seq, sysid, compid, msgid 24-bit LE, payload, CRC-16/MCRF4XX"},
    {"HEARTBEAT pack/unpack", PortStatus::kOnMain,
     "msgid 0: type, autopilot, base_mode, custom_mode, system_status, mavlink_version=3"},
    {"msgid dispatch stub", PortStatus::kOnMain,
     "known HEARTBEAT vs unknown; no GCS singleton"},
    {"leftover catalog", PortStatus::kOnMain, "this table"},
    {"COMMAND_LONG", PortStatus::kOnMain,
     "msgid 76 ARM/DISARM DO_SET_MODE + COMMAND_ACK msgid 77"},
    {"PARAM protocol", PortStatus::kOnMain,
     "PARAM_REQUEST_LIST / PARAM_SET / PARAM_VALUE"},
    {"MISSION", PortStatus::kOnMain, "MISSION_ITEM_INT / MISSION_REQUEST_INT"},
    {"Plane/Copter vehicle handlers", PortStatus::kThisSlice,
     "Plane leftover_send_attitude ATTITUDE msgid 30; Copter handlers remaining"},
    {"XML dialect generation", PortStatus::kRemaining,
     "generate from modules/mavlink; do not hand-roll the dialect here"},
};

[[nodiscard]] inline constexpr std::size_t leftover_completeness_size() {
    return sizeof(kGcsCompleteness) / sizeof(kGcsCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kGcsCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kGcsCompleteness) {
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

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}

}  // namespace fwcpp::gcs
