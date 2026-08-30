#pragma once

// MAVLink 2 framing for CPP-087 slice 1. Upstream helpers live in
// libraries/GCS_MAVLink / modules/mavlink (mavlink_helpers.h). This is
// the wire seam only: one payload, no signing, no generated dialect.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <fwcpp/result.hpp>

namespace fwcpp::gcs {

inline constexpr std::uint8_t kStxV2 = 0xFD;
inline constexpr std::size_t kHeaderLenV2 = 10;
inline constexpr std::size_t kCrcLen = 2;
inline constexpr std::size_t kMaxPayloadLen = 255;
inline constexpr std::uint32_t kMsgIdHeartbeat = 0;
inline constexpr std::uint32_t kMsgIdParamRequestList = 21;
inline constexpr std::uint32_t kMsgIdParamValue = 22;
inline constexpr std::uint32_t kMsgIdParamSet = 23;
inline constexpr std::uint32_t kMsgIdMissionCount = 44;
inline constexpr std::uint32_t kMsgIdMissionRequestInt = 51;
inline constexpr std::uint32_t kMsgIdMissionItemInt = 73;
inline constexpr std::uint32_t kMsgIdCommandLong = 76;
inline constexpr std::uint32_t kMsgIdCommandAck = 77;

// CRC extras from same-tree lua (libraries/AP_Scripting/modules/MAVLink)
// where present, else pymavlink message_checksum / generated headers of
// pinned modules/mavlink/message_definitions/v1.0/common.xml:
// HEARTBEAT.crc_extra = 50 msgid 0; PARAM_REQUEST_LIST = 159 msgid 21;
// PARAM_VALUE = 220 msgid 22; PARAM_SET = 168 msgid 23;
// MISSION_COUNT = 221 msgid 44; MISSION_REQUEST_INT = 196 msgid 51;
// MISSION_ITEM_INT = 38 msgid 73;
// COMMAND_LONG.crc_extra = 152 msgid 76; COMMAND_ACK.crc_extra = 143 msgid 77.
inline constexpr std::uint8_t kHeartbeatCrcExtra = 50;
inline constexpr std::uint8_t kParamRequestListCrcExtra = 159;
inline constexpr std::uint8_t kParamValueCrcExtra = 220;
inline constexpr std::uint8_t kParamSetCrcExtra = 168;
inline constexpr std::uint8_t kMissionCountCrcExtra = 221;
inline constexpr std::uint8_t kMissionRequestIntCrcExtra = 196;
inline constexpr std::uint8_t kMissionItemIntCrcExtra = 38;
inline constexpr std::uint8_t kCommandLongCrcExtra = 152;
inline constexpr std::uint8_t kCommandAckCrcExtra = 143;

struct Frame {
    std::uint8_t seq{};
    std::uint8_t sysid{};
    std::uint8_t compid{};
    std::uint32_t msgid{};
    std::uint8_t payload_len{};
    std::array<std::uint8_t, kMaxPayloadLen> payload{};

    [[nodiscard]] std::span<const std::uint8_t> payload_bytes() const {
        return std::span<const std::uint8_t>(payload.data(), payload_len);
    }
};

enum class DecodeError : std::uint8_t {
    kTruncated = 0,
    kBadMagic = 1,
    kBadCrc = 2,
    kUnsupportedFlags = 3,
};

[[nodiscard]] inline bool make_frame(std::uint8_t seq, std::uint8_t sysid, std::uint8_t compid,
                                     std::uint32_t msgid, std::span<const std::uint8_t> payload,
                                     Frame& out) {
    if (payload.size() > kMaxPayloadLen) {
        return false;
    }
    out = Frame{};
    out.seq = seq;
    out.sysid = sysid;
    out.compid = compid;
    out.msgid = msgid;
    out.payload_len = static_cast<std::uint8_t>(payload.size());
    for (std::size_t i = 0; i < payload.size(); ++i) {
        out.payload[i] = payload[i];
    }
    return true;
}

// CRC-16/MCRF4XX step, upstream crc_accumulate. Init value is 0xFFFF.
[[nodiscard]] inline constexpr std::uint16_t crc_accumulate(std::uint16_t crc, std::uint8_t data) {
    std::uint8_t tmp = static_cast<std::uint8_t>(data ^ static_cast<std::uint8_t>(crc));
    tmp = static_cast<std::uint8_t>(tmp ^ static_cast<std::uint8_t>(tmp << 4));
    return static_cast<std::uint16_t>((crc >> 8) ^ (static_cast<std::uint16_t>(tmp) << 8) ^
                                      (static_cast<std::uint16_t>(tmp) << 3) ^ (tmp >> 4));
}

[[nodiscard]] inline std::uint16_t crc16(std::span<const std::uint8_t> bytes, std::uint8_t extra,
                                         bool use_extra) {
    std::uint16_t crc = 0xFFFF;
    for (std::uint8_t b : bytes) {
        crc = crc_accumulate(crc, b);
    }
    if (use_extra) {
        crc = crc_accumulate(crc, extra);
    }
    return crc;
}

// Known msgid extras. encode_v2 refuses unknown msgid (returns 0).
[[nodiscard]] inline constexpr bool crc_extra(std::uint32_t msgid, std::uint8_t& extra) {
    if (msgid == kMsgIdHeartbeat) {
        extra = kHeartbeatCrcExtra;
        return true;
    }
    if (msgid == kMsgIdCommandLong) {
        extra = kCommandLongCrcExtra;
        return true;
    }
    if (msgid == kMsgIdCommandAck) {
        extra = kCommandAckCrcExtra;
        return true;
    }
    if (msgid == kMsgIdParamRequestList) {
        extra = kParamRequestListCrcExtra;
        return true;
    }
    if (msgid == kMsgIdParamValue) {
        extra = kParamValueCrcExtra;
        return true;
    }
    if (msgid == kMsgIdParamSet) {
        extra = kParamSetCrcExtra;
        return true;
    }
    if (msgid == kMsgIdMissionCount) {
        extra = kMissionCountCrcExtra;
        return true;
    }
    if (msgid == kMsgIdMissionRequestInt) {
        extra = kMissionRequestIntCrcExtra;
        return true;
    }
    if (msgid == kMsgIdMissionItemInt) {
        extra = kMissionItemIntCrcExtra;
        return true;
    }
    return false;
}

inline void write_u16_le(std::uint8_t* p, std::uint16_t v) {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}

[[nodiscard]] inline std::uint16_t read_u16_le(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}

inline void write_i32_le(std::uint8_t* p, std::int32_t v) {
    const auto u = static_cast<std::uint32_t>(v);
    p[0] = static_cast<std::uint8_t>(u);
    p[1] = static_cast<std::uint8_t>(u >> 8);
    p[2] = static_cast<std::uint8_t>(u >> 16);
    p[3] = static_cast<std::uint8_t>(u >> 24);
}

[[nodiscard]] inline std::int32_t read_i32_le(const std::uint8_t* p) {
    const std::uint32_t u = static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
                            (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
    return static_cast<std::int32_t>(u);
}

inline void write_f32_le(std::uint8_t* p, float v) {
    std::uint32_t bits = 0;
    static_assert(sizeof(float) == 4);
    std::memcpy(&bits, &v, sizeof(bits));
    p[0] = static_cast<std::uint8_t>(bits);
    p[1] = static_cast<std::uint8_t>(bits >> 8);
    p[2] = static_cast<std::uint8_t>(bits >> 16);
    p[3] = static_cast<std::uint8_t>(bits >> 24);
}

[[nodiscard]] inline float read_f32_le(const std::uint8_t* p) {
    const std::uint32_t bits = static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
                               (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
    float v = 0;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

// Encode STX, incompat/compat flags (0/0), seq, sysid, compid, 24-bit
// little-endian msgid, payload, CRC. Returns framed length, or 0.
[[nodiscard]] inline std::size_t encode_v2(const Frame& frame, std::span<std::uint8_t> out) {
    std::uint8_t extra = 0;
    if (!crc_extra(frame.msgid, extra)) {
        return 0;
    }
    const std::size_t payload_len = frame.payload_len;
    const std::size_t total = kHeaderLenV2 + payload_len + kCrcLen;
    if (out.size() < total) {
        return 0;
    }
    out[0] = kStxV2;
    out[1] = frame.payload_len;
    out[2] = 0;  // incompat_flags (signing unsupported this slice)
    out[3] = 0;  // compat_flags
    out[4] = frame.seq;
    out[5] = frame.sysid;
    out[6] = frame.compid;
    out[7] = static_cast<std::uint8_t>(frame.msgid);
    out[8] = static_cast<std::uint8_t>(frame.msgid >> 8);
    out[9] = static_cast<std::uint8_t>(frame.msgid >> 16);
    for (std::size_t i = 0; i < payload_len; ++i) {
        out[kHeaderLenV2 + i] = frame.payload[i];
    }
    const auto body = out.subspan(1, kHeaderLenV2 - 1 + payload_len);
    const std::uint16_t crc = crc16(body, extra, true);
    out[kHeaderLenV2 + payload_len] = static_cast<std::uint8_t>(crc);
    out[kHeaderLenV2 + payload_len + 1] = static_cast<std::uint8_t>(crc >> 8);
    return total;
}

[[nodiscard]] inline Result<Frame, DecodeError> decode_v2(std::span<const std::uint8_t> buf) {
    if (buf.empty()) {
        return Err(DecodeError::kTruncated);
    }
    if (buf[0] != kStxV2) {
        return Err(DecodeError::kBadMagic);
    }
    if (buf.size() < kHeaderLenV2) {
        return Err(DecodeError::kTruncated);
    }
    const std::uint8_t payload_len = buf[1];
    const std::uint8_t incompat = buf[2];
    if (incompat != 0) {
        return Err(DecodeError::kUnsupportedFlags);
    }
    const std::size_t payload_end = kHeaderLenV2 + static_cast<std::size_t>(payload_len);
    const std::size_t total = payload_end + kCrcLen;
    if (buf.size() < total) {
        return Err(DecodeError::kTruncated);
    }
    const std::uint32_t msgid = static_cast<std::uint32_t>(buf[7]) |
                                (static_cast<std::uint32_t>(buf[8]) << 8) |
                                (static_cast<std::uint32_t>(buf[9]) << 16);
    std::uint8_t extra = 0;
    if (!crc_extra(msgid, extra)) {
        return Err(DecodeError::kBadCrc);
    }
    const auto body = buf.subspan(1, payload_end - 1);
    const std::uint16_t want =
        static_cast<std::uint16_t>(buf[payload_end] | (static_cast<std::uint16_t>(buf[payload_end + 1]) << 8));
    if (crc16(body, extra, true) != want) {
        return Err(DecodeError::kBadCrc);
    }
    Frame frame{};
    if (!make_frame(buf[4], buf[5], buf[6], msgid, buf.subspan(kHeaderLenV2, payload_len), frame)) {
        return Err(DecodeError::kTruncated);
    }
    return frame;
}

}  // namespace fwcpp::gcs
