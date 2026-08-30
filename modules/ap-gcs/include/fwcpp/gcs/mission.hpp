#pragma once

// MISSION_REQUEST_INT (msgid 51) / MISSION_ITEM_INT (73) / optional
// MISSION_COUNT stub (44) for CPP-087 slice 4. Upstream: GCS_Common /
// MissionItemProtocol handle_mission_request_int / send_mission_item /
// MISSION_COUNT on request_list. Injected MissionStore stands in for
// AP_Mission (ADR-0012: no singleton). Enough to list one waypoint.
// No MISSION_ACK write path, CLEAR_ALL, or vehicle handlers.

#include <cstddef>
#include <cstdint>
#include <span>

#include <fwcpp/gcs/framing.hpp>

namespace fwcpp::gcs {

inline constexpr std::size_t kMissionRequestIntLen = 5;
inline constexpr std::size_t kMissionItemIntLen = 38;
inline constexpr std::size_t kMissionCountLen = 5;
inline constexpr std::size_t kMaxMissionItems = 8;

// common.xml MAV_MISSION_TYPE_MISSION / MAV_CMD_NAV_WAYPOINT.
inline constexpr std::uint8_t kMavMissionTypeMission = 0;
inline constexpr std::uint16_t kMavCmdNavWaypoint = 16;
inline constexpr std::uint8_t kMavFrameGlobalRelAlt = 3;

// Size-sorted v2 wire: seq uint16, target_system, target_component,
// mission_type. Matches modules/mavlink MISSION_REQUEST_INT pack.
struct MissionRequestInt {
    std::uint16_t seq{};
    std::uint8_t target_system{};
    std::uint8_t target_component{};
    std::uint8_t mission_type{};
};

// Size-sorted v2 wire: param1-4 float, x/y int32, z float, seq, command,
// target_system, target_component, frame, current, autocontinue,
// mission_type. Matches mavlink_msg_mission_item_int pack offsets.
struct MissionItemInt {
    float param1{};
    float param2{};
    float param3{};
    float param4{};
    std::int32_t x{};
    std::int32_t y{};
    float z{};
    std::uint16_t seq{};
    std::uint16_t command{};
    std::uint8_t target_system{};
    std::uint8_t target_component{};
    std::uint8_t frame{};
    std::uint8_t current{};
    std::uint8_t autocontinue{};
    std::uint8_t mission_type{};
};

// Size-sorted v2 wire (optional stub): count uint16, target_system,
// target_component, mission_type.
struct MissionCount {
    std::uint16_t count{};
    std::uint8_t target_system{};
    std::uint8_t target_component{};
    std::uint8_t mission_type{};
};

// Fixed table injected by the caller. Not AP_Mission.
struct MissionStore {
    MissionItemInt items[kMaxMissionItems]{};
    std::uint16_t count{0};
};

[[nodiscard]] inline std::size_t pack_mission_request_int(const MissionRequestInt& req,
                                                          std::span<std::uint8_t> buf) {
    if (buf.size() < kMissionRequestIntLen) {
        return 0;
    }
    write_u16_le(buf.data() + 0, req.seq);
    buf[2] = req.target_system;
    buf[3] = req.target_component;
    buf[4] = req.mission_type;
    return kMissionRequestIntLen;
}

[[nodiscard]] inline bool unpack_mission_request_int(std::span<const std::uint8_t> buf,
                                                     MissionRequestInt& out) {
    if (buf.size() < kMissionRequestIntLen) {
        return false;
    }
    out.seq = read_u16_le(buf.data() + 0);
    out.target_system = buf[2];
    out.target_component = buf[3];
    out.mission_type = buf[4];
    return true;
}

[[nodiscard]] inline std::size_t pack_mission_item_int(const MissionItemInt& item,
                                                       std::span<std::uint8_t> buf) {
    if (buf.size() < kMissionItemIntLen) {
        return 0;
    }
    write_f32_le(buf.data() + 0, item.param1);
    write_f32_le(buf.data() + 4, item.param2);
    write_f32_le(buf.data() + 8, item.param3);
    write_f32_le(buf.data() + 12, item.param4);
    write_i32_le(buf.data() + 16, item.x);
    write_i32_le(buf.data() + 20, item.y);
    write_f32_le(buf.data() + 24, item.z);
    write_u16_le(buf.data() + 28, item.seq);
    write_u16_le(buf.data() + 30, item.command);
    buf[32] = item.target_system;
    buf[33] = item.target_component;
    buf[34] = item.frame;
    buf[35] = item.current;
    buf[36] = item.autocontinue;
    buf[37] = item.mission_type;
    return kMissionItemIntLen;
}

[[nodiscard]] inline bool unpack_mission_item_int(std::span<const std::uint8_t> buf,
                                                  MissionItemInt& out) {
    if (buf.size() < kMissionItemIntLen) {
        return false;
    }
    out.param1 = read_f32_le(buf.data() + 0);
    out.param2 = read_f32_le(buf.data() + 4);
    out.param3 = read_f32_le(buf.data() + 8);
    out.param4 = read_f32_le(buf.data() + 12);
    out.x = read_i32_le(buf.data() + 16);
    out.y = read_i32_le(buf.data() + 20);
    out.z = read_f32_le(buf.data() + 24);
    out.seq = read_u16_le(buf.data() + 28);
    out.command = read_u16_le(buf.data() + 30);
    out.target_system = buf[32];
    out.target_component = buf[33];
    out.frame = buf[34];
    out.current = buf[35];
    out.autocontinue = buf[36];
    out.mission_type = buf[37];
    return true;
}

[[nodiscard]] inline std::size_t pack_mission_count(const MissionCount& count,
                                                    std::span<std::uint8_t> buf) {
    if (buf.size() < kMissionCountLen) {
        return 0;
    }
    write_u16_le(buf.data() + 0, count.count);
    buf[2] = count.target_system;
    buf[3] = count.target_component;
    buf[4] = count.mission_type;
    return kMissionCountLen;
}

[[nodiscard]] inline bool unpack_mission_count(std::span<const std::uint8_t> buf, MissionCount& out) {
    if (buf.size() < kMissionCountLen) {
        return false;
    }
    out.count = read_u16_le(buf.data() + 0);
    out.target_system = buf[2];
    out.target_component = buf[3];
    out.mission_type = buf[4];
    return true;
}

[[nodiscard]] inline bool mission_request_int_from_frame(const Frame& frame,
                                                         MissionRequestInt& out) {
    if (frame.msgid != kMsgIdMissionRequestInt) {
        return false;
    }
    return unpack_mission_request_int(frame.payload_bytes(), out);
}

[[nodiscard]] inline bool mission_item_int_from_frame(const Frame& frame, MissionItemInt& out) {
    if (frame.msgid != kMsgIdMissionItemInt) {
        return false;
    }
    return unpack_mission_item_int(frame.payload_bytes(), out);
}

[[nodiscard]] inline bool mission_count_from_frame(const Frame& frame, MissionCount& out) {
    if (frame.msgid != kMsgIdMissionCount) {
        return false;
    }
    return unpack_mission_count(frame.payload_bytes(), out);
}

[[nodiscard]] inline bool mission_store_insert(MissionStore& store, const MissionItemInt& item) {
    if (store.count >= kMaxMissionItems) {
        return false;
    }
    store.items[store.count] = item;
    ++store.count;
    return true;
}

[[nodiscard]] inline const MissionItemInt* mission_store_find(const MissionStore& store,
                                                             std::uint16_t seq) {
    for (std::uint16_t i = 0; i < store.count; ++i) {
        if (store.items[i].seq == seq) {
            return &store.items[i];
        }
    }
    return nullptr;
}

[[nodiscard]] inline MissionItemInt* mission_store_find(MissionStore& store, std::uint16_t seq) {
    return const_cast<MissionItemInt*>(
        mission_store_find(static_cast<const MissionStore&>(store), seq));
}

// Reply to MISSION_REQUEST_INT: copy matching item (if any) into out.
[[nodiscard]] inline bool reply_mission_request_int(const MissionStore& store,
                                                    const MissionRequestInt& req,
                                                    MissionItemInt& out) {
    const MissionItemInt* item = mission_store_find(store, req.seq);
    if (item == nullptr) {
        return false;
    }
    out = *item;
    return true;
}

[[nodiscard]] inline MissionCount make_mission_count(const MissionStore& store,
                                                     std::uint8_t target_system,
                                                     std::uint8_t target_component,
                                                     std::uint8_t mission_type) {
    MissionCount c{};
    c.count = store.count;
    c.target_system = target_system;
    c.target_component = target_component;
    c.mission_type = mission_type;
    return c;
}

}  // namespace fwcpp::gcs
