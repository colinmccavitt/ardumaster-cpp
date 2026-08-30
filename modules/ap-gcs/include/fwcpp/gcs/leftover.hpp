#pragma once

// CPP-087 slice 6 completeness catalog close.
//
// On main: MAVLink 2 framing, HEARTBEAT pack/unpack, msgid dispatch stub,
// COMMAND_LONG ARM/DISARM + DO_SET_MODE + COMMAND_ACK, PARAM_REQUEST_LIST /
// PARAM_SET / PARAM_VALUE, MISSION_ITEM_INT / MISSION_REQUEST_INT, Plane
// leftover_send_attitude (ATTITUDE 30).
// This slice: leftover catalog close — remaining_count()==0.
// Out of scope: XML dialect generation (pinned modules/mavlink; no in-tree
// codegen). Copter vehicle handlers share the same ATTITUDE wire; vehicle-
// specific Copter send path deferred (noted on the vehicle handlers row).

#include <cstddef>
#include <cstdint>

namespace fwcpp::gcs {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
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
    {"leftover catalog", PortStatus::kThisSlice, "CPP-087 close; remaining_count 0"},
    {"COMMAND_LONG", PortStatus::kOnMain,
     "msgid 76 ARM/DISARM DO_SET_MODE + COMMAND_ACK msgid 77"},
    {"PARAM protocol", PortStatus::kOnMain,
     "PARAM_REQUEST_LIST / PARAM_SET / PARAM_VALUE"},
    {"MISSION", PortStatus::kOnMain, "MISSION_ITEM_INT / MISSION_REQUEST_INT"},
    {"Plane/Copter vehicle handlers", PortStatus::kOnMain,
     "Plane leftover_send_attitude ATTITUDE msgid 30; Copter same ATTITUDE wire; "
     "vehicle-specific later"},
    {"XML dialect generation", PortStatus::kOutOfScope,
     "pinned modules/mavlink generated headers; no in-tree codegen this port"},
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

[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}

}  // namespace fwcpp::gcs
