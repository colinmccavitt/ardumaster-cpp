#pragma once

// Msgid dispatch stub: known HEARTBEAT vs unknown. One caller-owned
// channel (ADR-0012: no GCS singleton). Later slices add COMMAND_LONG,
// PARAM, MISSION, and vehicle handlers.

#include <cstddef>
#include <cstdint>
#include <span>

#include <fwcpp/gcs/framing.hpp>
#include <fwcpp/gcs/heartbeat.hpp>
#include <fwcpp/result.hpp>

namespace fwcpp::gcs {

inline constexpr std::uint8_t kDefaultSysid = 1;
inline constexpr std::uint8_t kMavCompIdAutopilot1 = 1;
inline constexpr std::uint8_t kDefaultGcsSysid = 255;

enum class DispatchKind : std::uint8_t {
    kHeartbeat = 0,
    kUnknown = 1,
};

struct Dispatch {
    DispatchKind kind{DispatchKind::kUnknown};
    Heartbeat heartbeat{};
    bool from_gcs{false};
    std::uint32_t msgid{};
};

class GcsMavlink {
public:
    GcsMavlink() = default;

    void set_gcs_sysid(std::uint8_t gcs_sysid) { gcs_sysid_ = gcs_sysid; }

    [[nodiscard]] std::uint32_t last_gcs_heartbeat_ms() const { return last_gcs_heartbeat_ms_; }

    // Encode one outgoing HEARTBEAT. Returns framed length, or 0.
    [[nodiscard]] std::size_t send_heartbeat(std::span<std::uint8_t> out, std::uint8_t type,
                                             std::uint8_t base_mode, std::uint32_t custom_mode,
                                             std::uint8_t system_status) {
        const Heartbeat hb = plane_heartbeat(type, base_mode, custom_mode, system_status);
        std::uint8_t payload[kHeartbeatLen]{};
        if (pack_heartbeat(hb, payload) == 0) {
            return 0;
        }
        Frame frame{};
        if (!make_frame(seq_, sysid_, compid_, kMsgIdHeartbeat, payload, frame)) {
            return 0;
        }
        seq_ = static_cast<std::uint8_t>(seq_ + 1);
        return encode_v2(frame, out);
    }

    [[nodiscard]] Dispatch handle_message(const Frame& frame, std::uint32_t now_ms) {
        if (frame.msgid != kMsgIdHeartbeat) {
            Dispatch d{};
            d.kind = DispatchKind::kUnknown;
            d.msgid = frame.msgid;
            return d;
        }
        Heartbeat hb{};
        if (!heartbeat_from_frame(frame, hb)) {
            Dispatch d{};
            d.kind = DispatchKind::kUnknown;
            d.msgid = kMsgIdHeartbeat;
            return d;
        }
        const bool from_gcs = frame.sysid == gcs_sysid_;
        if (from_gcs) {
            last_gcs_heartbeat_ms_ = now_ms;
        }
        Dispatch d{};
        d.kind = DispatchKind::kHeartbeat;
        d.heartbeat = hb;
        d.from_gcs = from_gcs;
        d.msgid = kMsgIdHeartbeat;
        return d;
    }

    [[nodiscard]] Result<Dispatch, DecodeError> handle_bytes(std::span<const std::uint8_t> buf,
                                                            std::uint32_t now_ms) {
        auto decoded = decode_v2(buf);
        if (!decoded.has_value()) {
            return Err(decoded.error());
        }
        return handle_message(decoded.value(), now_ms);
    }

private:
    std::uint8_t sysid_{kDefaultSysid};
    std::uint8_t compid_{kMavCompIdAutopilot1};
    std::uint8_t seq_{0};
    std::uint8_t gcs_sysid_{kDefaultGcsSysid};
    std::uint32_t last_gcs_heartbeat_ms_{0};
};

}  // namespace fwcpp::gcs
