#pragma once

// COMMAND_LONG (msgid 76) + COMMAND_ACK (msgid 77) for CPP-087 slice 2.
// Upstream: GCS_Common.cpp handle_command_long / try_command_long_as_command_int
// / convert_COMMAND_LONG_to_COMMAND_INT / handle_command_int_packet
// / handle_command_do_set_mode / _set_mode_common
// / handle_command_component_arm_disarm.
// ARM/SET_MODE do not store location; convert still copies params.
// No PARAM, MISSION, vehicle-specific handlers, scripting skip, or
// logger Write_Command. Hooks are injected (ADR-0012: no GCS / AP::
// arming / AP::vehicle / rcout singletons).

#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>

#include <fwcpp/gcs/framing.hpp>

namespace fwcpp::gcs {

inline constexpr std::size_t kCommandLongLen = 33;
inline constexpr std::size_t kCommandAckLen = 10;

// Pinned common.xml / minimal.xml (do not invent).
inline constexpr std::uint16_t kMavCmdDoSetMode = 176;
inline constexpr std::uint16_t kMavCmdComponentArmDisarm = 400;
inline constexpr std::uint8_t kMavModeFlagCustomModeEnabled = 1;
inline constexpr std::uint8_t kMavModeFlagDecodePositionSafety = 128;

// GCS.h magic_force_arm_value / magic_force_arm_disarm_value.
inline constexpr float kMagicForceArmValue = 2989.0f;
inline constexpr float kMagicForceArmDisarmValue = 21196.0f;

enum class MavResult : std::uint8_t {
    kAccepted = 0,
    kDenied = 2,
    kUnsupported = 3,
    kFailed = 4,
};

// Zero-allocation hooks (function pointer + ctx, not std::function).
struct CommandHooks {
    bool (*is_armed)(void* ctx) = nullptr;
    bool (*arm)(void* ctx, bool do_arming_checks) = nullptr;
    bool (*disarm)(void* ctx, bool do_disarm_checks) = nullptr;
    bool (*set_mode)(void* ctx, std::uint32_t custom_mode) = nullptr;
    bool (*force_safety_on)(void* ctx) = nullptr;
    void (*force_safety_off)(void* ctx) = nullptr;
    void* ctx = nullptr;
};

struct CommandLong {
    float param1{};
    float param2{};
    float param3{};
    float param4{};
    float param5{};
    float param6{};
    float param7{};
    std::uint16_t command{};
    std::uint8_t target_system{};
    std::uint8_t target_component{};
    std::uint8_t confirmation{};
};

// COMMAND_INT fields this slice needs after convert (params copied;
// x/y are non-location integer casts of param5/param6).
struct CommandInt {
    float param1{};
    float param2{};
    float param3{};
    float param4{};
    std::int32_t x{};
    std::int32_t y{};
    float z{};
    std::uint16_t command{};
    std::uint8_t target_system{};
    std::uint8_t target_component{};
    std::uint8_t frame{};
    std::uint8_t current{};
    std::uint8_t autocontinue{};
};

// Size-sorted v2 wire: result_param2 (i4), command (I2), result, progress,
// target_system, target_component. Lua lists command first; MAVLink2
// size-sorts like HEARTBEAT (custom_mode uint32 first).
struct CommandAck {
    std::uint16_t command{};
    std::uint8_t result{};
    std::uint8_t progress{};
    std::int32_t result_param2{};
    std::uint8_t target_system{};
    std::uint8_t target_component{};
};

[[nodiscard]] inline bool mav_is_zero(float x) {
    return std::fabs(x) < FLT_EPSILON;
}

[[nodiscard]] inline bool mav_is_equal(float a, float b) {
    return std::fabs(a - b) < std::numeric_limits<float>::epsilon();
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

// Lua / size-sorted: param1-7 float, command uint16, target_system,
// target_component, confirmation.
[[nodiscard]] inline std::size_t pack_command_long(const CommandLong& cmd, std::span<std::uint8_t> buf) {
    if (buf.size() < kCommandLongLen) {
        return 0;
    }
    write_f32_le(buf.data() + 0, cmd.param1);
    write_f32_le(buf.data() + 4, cmd.param2);
    write_f32_le(buf.data() + 8, cmd.param3);
    write_f32_le(buf.data() + 12, cmd.param4);
    write_f32_le(buf.data() + 16, cmd.param5);
    write_f32_le(buf.data() + 20, cmd.param6);
    write_f32_le(buf.data() + 24, cmd.param7);
    write_u16_le(buf.data() + 28, cmd.command);
    buf[30] = cmd.target_system;
    buf[31] = cmd.target_component;
    buf[32] = cmd.confirmation;
    return kCommandLongLen;
}

[[nodiscard]] inline bool unpack_command_long(std::span<const std::uint8_t> buf, CommandLong& out) {
    if (buf.size() < kCommandLongLen) {
        return false;
    }
    out.param1 = read_f32_le(buf.data() + 0);
    out.param2 = read_f32_le(buf.data() + 4);
    out.param3 = read_f32_le(buf.data() + 8);
    out.param4 = read_f32_le(buf.data() + 12);
    out.param5 = read_f32_le(buf.data() + 16);
    out.param6 = read_f32_le(buf.data() + 20);
    out.param7 = read_f32_le(buf.data() + 24);
    out.command = read_u16_le(buf.data() + 28);
    out.target_system = buf[30];
    out.target_component = buf[31];
    out.confirmation = buf[32];
    return true;
}

[[nodiscard]] inline std::size_t pack_command_ack(const CommandAck& ack, std::span<std::uint8_t> buf) {
    if (buf.size() < kCommandAckLen) {
        return 0;
    }
    write_i32_le(buf.data() + 0, ack.result_param2);
    write_u16_le(buf.data() + 4, ack.command);
    buf[6] = ack.result;
    buf[7] = ack.progress;
    buf[8] = ack.target_system;
    buf[9] = ack.target_component;
    return kCommandAckLen;
}

[[nodiscard]] inline bool unpack_command_ack(std::span<const std::uint8_t> buf, CommandAck& out) {
    if (buf.size() < kCommandAckLen) {
        return false;
    }
    out.result_param2 = read_i32_le(buf.data() + 0);
    out.command = read_u16_le(buf.data() + 4);
    out.result = buf[6];
    out.progress = buf[7];
    out.target_system = buf[8];
    out.target_component = buf[9];
    return true;
}

[[nodiscard]] inline bool command_from_frame(const Frame& frame, CommandLong& out) {
    if (frame.msgid != kMsgIdCommandLong) {
        return false;
    }
    return unpack_command_long(frame.payload_bytes(), out);
}

[[nodiscard]] inline bool command_ack_from_frame(const Frame& frame, CommandAck& out) {
    if (frame.msgid != kMsgIdCommandAck) {
        return false;
    }
    return unpack_command_ack(frame.payload_bytes(), out);
}

// ARM/SET_MODE are not in command_long_stores_location; x/y are the
// non-location branch of convert_COMMAND_LONG_loc_param (cast, not *1e7).
inline void convert_command_long_to_command_int(const CommandLong& in, CommandInt& out) {
    out = CommandInt{};
    out.target_system = in.target_system;
    out.target_component = in.target_component;
    out.frame = 0;
    out.command = in.command;
    out.current = 0;
    out.autocontinue = 0;
    out.param1 = in.param1;
    out.param2 = in.param2;
    out.param3 = in.param3;
    out.param4 = in.param4;
    if (std::isnan(in.param5)) {
        out.x = 0;
    } else {
        out.x = static_cast<std::int32_t>(in.param5);
    }
    if (std::isnan(in.param6)) {
        out.y = 0;
    } else {
        out.y = static_cast<std::int32_t>(in.param6);
    }
    out.z = in.param7;
}

[[nodiscard]] inline MavResult handle_command_component_arm_disarm(const CommandInt& packet,
                                                                  const CommandHooks& hooks) {
    if (mav_is_equal(packet.param1, 1.0f)) {
        if (hooks.is_armed != nullptr && hooks.is_armed(hooks.ctx)) {
            return MavResult::kAccepted;
        }
        const bool do_arming_checks = !mav_is_equal(packet.param2, kMagicForceArmValue) &&
                                      !mav_is_equal(packet.param2, kMagicForceArmDisarmValue);
        if (hooks.arm != nullptr && hooks.arm(hooks.ctx, do_arming_checks)) {
            return MavResult::kAccepted;
        }
        return MavResult::kFailed;
    }
    if (mav_is_zero(packet.param1)) {
        if (hooks.is_armed == nullptr || !hooks.is_armed(hooks.ctx)) {
            return MavResult::kAccepted;
        }
        const bool forced = mav_is_equal(packet.param2, kMagicForceArmDisarmValue);
        if (hooks.disarm != nullptr && hooks.disarm(hooks.ctx, !forced)) {
            return MavResult::kAccepted;
        }
        return MavResult::kFailed;
    }
    return MavResult::kUnsupported;
}

[[nodiscard]] inline MavResult set_mode_common(std::uint8_t base_mode, std::uint32_t custom_mode,
                                               const CommandHooks& hooks) {
    if ((base_mode & kMavModeFlagCustomModeEnabled) != 0) {
        if (hooks.set_mode != nullptr && hooks.set_mode(hooks.ctx, custom_mode)) {
            return MavResult::kAccepted;
        }
        return MavResult::kFailed;
    }
    if (base_mode == kMavModeFlagDecodePositionSafety) {
        if (custom_mode == 0) {
            if (hooks.force_safety_off != nullptr) {
                hooks.force_safety_off(hooks.ctx);
            }
            return MavResult::kAccepted;
        }
        if (custom_mode == 1) {
            if (hooks.force_safety_on != nullptr && hooks.force_safety_on(hooks.ctx)) {
                return MavResult::kAccepted;
            }
            return MavResult::kFailed;
        }
        return MavResult::kDenied;
    }
    return MavResult::kDenied;
}

[[nodiscard]] inline MavResult handle_command_do_set_mode(const CommandInt& packet,
                                                          const CommandHooks& hooks) {
    const auto base_mode = static_cast<std::uint8_t>(packet.param1);
    const auto custom_mode = static_cast<std::uint32_t>(packet.param2);
    return set_mode_common(base_mode, custom_mode, hooks);
}

[[nodiscard]] inline MavResult handle_command_int_packet(const CommandInt& packet,
                                                        const CommandHooks& hooks) {
    switch (packet.command) {
        case kMavCmdDoSetMode:
            return handle_command_do_set_mode(packet, hooks);
        case kMavCmdComponentArmDisarm:
            return handle_command_component_arm_disarm(packet, hooks);
        default:
            return MavResult::kUnsupported;
    }
}

[[nodiscard]] inline MavResult try_command_long_as_command_int(const CommandLong& packet,
                                                              const CommandHooks& hooks) {
    CommandInt command_int{};
    convert_command_long_to_command_int(packet, command_int);
    return handle_command_int_packet(command_int, hooks);
}

[[nodiscard]] inline CommandAck make_command_ack(std::uint16_t command, MavResult result,
                                                 std::uint8_t target_system,
                                                 std::uint8_t target_component) {
    CommandAck ack{};
    ack.command = command;
    ack.result = static_cast<std::uint8_t>(result);
    ack.progress = 0;
    ack.result_param2 = 0;
    ack.target_system = target_system;
    ack.target_component = target_component;
    return ack;
}

}  // namespace fwcpp::gcs
