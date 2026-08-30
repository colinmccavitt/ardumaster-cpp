#pragma once

// Msgid dispatch: HEARTBEAT, COMMAND_LONG (ARM/DISARM, DO_SET_MODE),
// PARAM_REQUEST_LIST / PARAM_SET. One caller-owned channel (ADR-0012: no
// GCS singleton). Hooks and ParamStore are injected. Later slices add
// MISSION and vehicle handlers.

#include <cstddef>
#include <cstdint>
#include <span>

#include <fwcpp/gcs/command.hpp>
#include <fwcpp/gcs/framing.hpp>
#include <fwcpp/gcs/heartbeat.hpp>
#include <fwcpp/gcs/param.hpp>
#include <fwcpp/result.hpp>

namespace fwcpp::gcs {

inline constexpr std::uint8_t kDefaultSysid = 1;
inline constexpr std::uint8_t kMavCompIdAutopilot1 = 1;
inline constexpr std::uint8_t kDefaultGcsSysid = 255;

enum class DispatchKind : std::uint8_t {
    kHeartbeat = 0,
    kUnknown = 1,
    kCommandLong = 2,
    kParamRequestList = 3,
    kParamSet = 4,
};

struct Dispatch {
    DispatchKind kind{DispatchKind::kUnknown};
    Heartbeat heartbeat{};
    CommandLong command_long{};
    CommandAck command_ack{};
    MavResult command_result{MavResult::kUnsupported};
    ParamRequestList param_request_list{};
    ParamSet param_set{};
    ParamValue param_value{};
    std::uint16_t param_count{};
    bool param_applied{false};
    bool from_gcs{false};
    std::uint32_t msgid{};
};

class GcsMavlink {
public:
    GcsMavlink() = default;

    void set_gcs_sysid(std::uint8_t gcs_sysid) { gcs_sysid_ = gcs_sysid; }

    void set_hooks(const CommandHooks& hooks) { hooks_ = hooks; }

    void set_param_store(ParamStore& store) { params_ = &store; }

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

    // Encode COMMAND_ACK (progress=0, result_param2=0 set by caller).
    [[nodiscard]] std::size_t send_command_ack(std::span<std::uint8_t> out, const CommandAck& ack) {
        std::uint8_t payload[kCommandAckLen]{};
        if (pack_command_ack(ack, payload) == 0) {
            return 0;
        }
        Frame frame{};
        if (!make_frame(seq_, sysid_, compid_, kMsgIdCommandAck, payload, frame)) {
            return 0;
        }
        seq_ = static_cast<std::uint8_t>(seq_ + 1);
        return encode_v2(frame, out);
    }

    [[nodiscard]] std::size_t send_param_value(std::span<std::uint8_t> out, const ParamValue& value) {
        std::uint8_t payload[kParamValueLen]{};
        if (pack_param_value(value, payload) == 0) {
            return 0;
        }
        Frame frame{};
        if (!make_frame(seq_, sysid_, compid_, kMsgIdParamValue, payload, frame)) {
            return 0;
        }
        seq_ = static_cast<std::uint8_t>(seq_ + 1);
        return encode_v2(frame, out);
    }

    [[nodiscard]] Dispatch handle_message(const Frame& frame, std::uint32_t now_ms) {
        if (frame.msgid == kMsgIdCommandLong) {
            return handle_command_long_frame(frame);
        }
        if (frame.msgid == kMsgIdParamRequestList) {
            return handle_param_request_list_frame(frame);
        }
        if (frame.msgid == kMsgIdParamSet) {
            return handle_param_set_frame(frame);
        }
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

    // handle_command_long: convert, handle int packet, fill ACK (progress=0,
    // result_param2=0, target = incoming sysid/compid). Does not encode.
    [[nodiscard]] Dispatch handle_command_long_frame(const Frame& frame) {
        CommandLong cmd{};
        if (!command_from_frame(frame, cmd)) {
            Dispatch d{};
            d.kind = DispatchKind::kUnknown;
            d.msgid = kMsgIdCommandLong;
            return d;
        }
        const MavResult result = try_command_long_as_command_int(cmd, hooks_);
        Dispatch d{};
        d.kind = DispatchKind::kCommandLong;
        d.command_long = cmd;
        d.command_result = result;
        d.command_ack = make_command_ack(cmd.command, result, frame.sysid, frame.compid);
        d.msgid = kMsgIdCommandLong;
        return d;
    }

    // Handle COMMAND_LONG and encode COMMAND_ACK into out. Returns framed
    // ACK length, or 0.
    [[nodiscard]] std::size_t handle_command_long(const Frame& frame, std::span<std::uint8_t> out) {
        const Dispatch d = handle_command_long_frame(frame);
        if (d.kind != DispatchKind::kCommandLong) {
            return 0;
        }
        return send_command_ack(out, d.command_ack);
    }

    [[nodiscard]] Dispatch handle_param_request_list_frame(const Frame& frame) {
        ParamRequestList req{};
        if (!param_request_list_from_frame(frame, req)) {
            Dispatch d{};
            d.kind = DispatchKind::kUnknown;
            d.msgid = kMsgIdParamRequestList;
            return d;
        }
        Dispatch d{};
        d.kind = DispatchKind::kParamRequestList;
        d.param_request_list = req;
        d.param_count = params_ != nullptr ? params_->count : 0;
        d.msgid = kMsgIdParamRequestList;
        return d;
    }

    // handle_param_request_list: emit one PARAM_VALUE per injected entry
    // into caller-owned out. Returns how many were written.
    [[nodiscard]] std::size_t handle_param_request_list(const Frame& frame,
                                                        std::span<ParamValue> out) {
        if (handle_param_request_list_frame(frame).kind != DispatchKind::kParamRequestList) {
            return 0;
        }
        if (params_ == nullptr) {
            return 0;
        }
        return emit_param_list(*params_, out);
    }

    [[nodiscard]] Dispatch handle_param_set_frame(const Frame& frame) {
        ParamSet set{};
        if (!param_set_from_frame(frame, set)) {
            Dispatch d{};
            d.kind = DispatchKind::kUnknown;
            d.msgid = kMsgIdParamSet;
            return d;
        }
        Dispatch d{};
        d.kind = DispatchKind::kParamSet;
        d.param_set = set;
        d.msgid = kMsgIdParamSet;
        if (params_ == nullptr) {
            return d;
        }
        ParamValue ack{};
        const ParamSetStatus status = apply_param_set(*params_, set, ack);
        d.param_applied = status == ParamSetStatus::kApplied;
        if (d.param_applied) {
            d.param_value = ack;
        }
        return d;
    }

    // Handle PARAM_SET and encode PARAM_VALUE ack into out. Returns framed
    // length, or 0 if unknown / rejected (upstream ignores; no crash).
    [[nodiscard]] std::size_t handle_param_set(const Frame& frame, std::span<std::uint8_t> out) {
        const Dispatch d = handle_param_set_frame(frame);
        if (!d.param_applied) {
            return 0;
        }
        return send_param_value(out, d.param_value);
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
    CommandHooks hooks_{};
    ParamStore* params_{nullptr};
};

}  // namespace fwcpp::gcs
