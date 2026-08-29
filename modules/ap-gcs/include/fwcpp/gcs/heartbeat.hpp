#pragma once

// HEARTBEAT payload (msgid 0). Wire order is size-sorted: custom_mode
// (uint32) first, then type, autopilot, base_mode, system_status,
// mavlink_version. Pack always writes mavlink_version = 3
// (uint8_t_mavlink_version is not caller-writable).

#include <cstddef>
#include <cstdint>
#include <span>

#include <fwcpp/gcs/framing.hpp>

namespace fwcpp::gcs {

inline constexpr std::size_t kHeartbeatLen = 9;
inline constexpr std::uint8_t kMavAutopilotArdupilotmega = 3;
inline constexpr std::uint8_t kMavTypeFixedWing = 1;
inline constexpr std::uint8_t kMavlinkVersion = 3;

struct Heartbeat {
    std::uint32_t custom_mode{};
    std::uint8_t type{};
    std::uint8_t autopilot{};
    std::uint8_t base_mode{};
    std::uint8_t system_status{};
    std::uint8_t mavlink_version{};
};

[[nodiscard]] inline Heartbeat plane_heartbeat(std::uint8_t type, std::uint8_t base_mode,
                                               std::uint32_t custom_mode,
                                               std::uint8_t system_status) {
    Heartbeat hb{};
    hb.custom_mode = custom_mode;
    hb.type = type;
    hb.autopilot = kMavAutopilotArdupilotmega;
    hb.base_mode = base_mode;
    hb.system_status = system_status;
    hb.mavlink_version = kMavlinkVersion;
    return hb;
}

// Pack 9 little-endian bytes. Returns kHeartbeatLen, or 0 if buf is short.
[[nodiscard]] inline std::size_t pack_heartbeat(const Heartbeat& hb, std::span<std::uint8_t> buf) {
    if (buf.size() < kHeartbeatLen) {
        return 0;
    }
    buf[0] = static_cast<std::uint8_t>(hb.custom_mode);
    buf[1] = static_cast<std::uint8_t>(hb.custom_mode >> 8);
    buf[2] = static_cast<std::uint8_t>(hb.custom_mode >> 16);
    buf[3] = static_cast<std::uint8_t>(hb.custom_mode >> 24);
    buf[4] = hb.type;
    buf[5] = hb.autopilot;
    buf[6] = hb.base_mode;
    buf[7] = hb.system_status;
    buf[8] = kMavlinkVersion;
    return kHeartbeatLen;
}

[[nodiscard]] inline bool unpack_heartbeat(std::span<const std::uint8_t> buf, Heartbeat& out) {
    if (buf.size() < kHeartbeatLen) {
        return false;
    }
    out.custom_mode = static_cast<std::uint32_t>(buf[0]) | (static_cast<std::uint32_t>(buf[1]) << 8) |
                      (static_cast<std::uint32_t>(buf[2]) << 16) |
                      (static_cast<std::uint32_t>(buf[3]) << 24);
    out.type = buf[4];
    out.autopilot = buf[5];
    out.base_mode = buf[6];
    out.system_status = buf[7];
    out.mavlink_version = buf[8];
    return true;
}

[[nodiscard]] inline bool heartbeat_from_frame(const Frame& frame, Heartbeat& out) {
    if (frame.msgid != kMsgIdHeartbeat) {
        return false;
    }
    return unpack_heartbeat(frame.payload_bytes(), out);
}

}  // namespace fwcpp::gcs
