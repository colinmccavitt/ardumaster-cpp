// CPP-087 slice 2: COMMAND_LONG pack/unpack, ARM/DISARM, DO_SET_MODE, ACK.

#include <array>
#include <cstdint>
#include <span>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/gcs/gcs.hpp>

using fwcpp::gcs::CommandAck;
using fwcpp::gcs::CommandHooks;
using fwcpp::gcs::CommandLong;
using fwcpp::gcs::DecodeError;
using fwcpp::gcs::DispatchKind;
using fwcpp::gcs::Frame;
using fwcpp::gcs::GcsMavlink;
using fwcpp::gcs::Heartbeat;
using fwcpp::gcs::MavResult;
using fwcpp::gcs::PortStatus;
using fwcpp::gcs::command_ack_from_frame;
using fwcpp::gcs::completeness_has;
using fwcpp::gcs::crc_extra;
using fwcpp::gcs::decode_v2;
using fwcpp::gcs::encode_v2;
using fwcpp::gcs::heartbeat_from_frame;
using fwcpp::gcs::kCommandAckCrcExtra;
using fwcpp::gcs::kCommandAckLen;
using fwcpp::gcs::kCommandLongCrcExtra;
using fwcpp::gcs::kCommandLongLen;
using fwcpp::gcs::kHeartbeatLen;
using fwcpp::gcs::kMagicForceArmDisarmValue;
using fwcpp::gcs::kMagicForceArmValue;
using fwcpp::gcs::kMavAutopilotArdupilotmega;
using fwcpp::gcs::kMavCmdComponentArmDisarm;
using fwcpp::gcs::kMavCmdDoSetMode;
using fwcpp::gcs::kMavModeFlagCustomModeEnabled;
using fwcpp::gcs::kMavModeFlagDecodePositionSafety;
using fwcpp::gcs::kMavTypeFixedWing;
using fwcpp::gcs::kMavlinkVersion;
using fwcpp::gcs::kMsgIdCommandAck;
using fwcpp::gcs::kMsgIdCommandLong;
using fwcpp::gcs::kMsgIdHeartbeat;
using fwcpp::gcs::kStxV2;
using fwcpp::gcs::leftover_completeness_size;
using fwcpp::gcs::make_frame;
using fwcpp::gcs::on_main_count;
using fwcpp::gcs::pack_command_ack;
using fwcpp::gcs::pack_command_long;
using fwcpp::gcs::pack_heartbeat;
using fwcpp::gcs::plane_heartbeat;
using fwcpp::gcs::remaining_count;
using fwcpp::gcs::this_slice_count;
using fwcpp::gcs::unpack_command_ack;
using fwcpp::gcs::unpack_command_long;
using fwcpp::gcs::unpack_heartbeat;

namespace {

struct FakeVehicle {
    bool armed = false;
    bool arm_ok = true;
    bool disarm_ok = true;
    bool set_mode_ok = true;
    bool safety_on_ok = true;
    bool last_arm_checks = true;
    bool last_disarm_checks = true;
    std::uint32_t last_custom_mode = 0;
    int safety_on_calls = 0;
    int safety_off_calls = 0;
};

bool fake_is_armed(void* ctx) { return static_cast<FakeVehicle*>(ctx)->armed; }

bool fake_arm(void* ctx, bool do_arming_checks) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    v->last_arm_checks = do_arming_checks;
    if (!v->arm_ok) {
        return false;
    }
    v->armed = true;
    return true;
}

bool fake_disarm(void* ctx, bool do_disarm_checks) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    v->last_disarm_checks = do_disarm_checks;
    if (!v->disarm_ok) {
        return false;
    }
    v->armed = false;
    return true;
}

bool fake_set_mode(void* ctx, std::uint32_t custom_mode) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    v->last_custom_mode = custom_mode;
    return v->set_mode_ok;
}

bool fake_safety_on(void* ctx) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    ++v->safety_on_calls;
    return v->safety_on_ok;
}

void fake_safety_off(void* ctx) {
    auto* v = static_cast<FakeVehicle*>(ctx);
    ++v->safety_off_calls;
}

CommandHooks make_hooks(FakeVehicle& v) {
    CommandHooks h{};
    h.is_armed = fake_is_armed;
    h.arm = fake_arm;
    h.disarm = fake_disarm;
    h.set_mode = fake_set_mode;
    h.force_safety_on = fake_safety_on;
    h.force_safety_off = fake_safety_off;
    h.ctx = &v;
    return h;
}

CommandLong make_cmd(std::uint16_t command, float p1, float p2) {
    CommandLong cmd{};
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.command = command;
    cmd.target_system = 1;
    cmd.target_component = 1;
    return cmd;
}

Frame frame_command_long(const CommandLong& cmd, std::uint8_t sysid = 255, std::uint8_t compid = 190) {
    std::array<std::uint8_t, kCommandLongLen> payload{};
    REQUIRE(pack_command_long(cmd, payload) == kCommandLongLen);
    Frame frame{};
    REQUIRE(make_frame(0, sysid, compid, kMsgIdCommandLong, payload, frame));
    return frame;
}

}  // namespace

TEST_CASE("HEARTBEAT payload and MAVLink2 frame round-trip", "[gcs][heartbeat]") {
    const Heartbeat hb = plane_heartbeat(kMavTypeFixedWing, 0x81, 12, 4);
    std::array<std::uint8_t, kHeartbeatLen> payload{};
    REQUIRE(pack_heartbeat(hb, payload) == kHeartbeatLen);

    Heartbeat unpacked{};
    REQUIRE(unpack_heartbeat(payload, unpacked));
    REQUIRE(unpacked.custom_mode == 12);
    REQUIRE(unpacked.type == kMavTypeFixedWing);
    REQUIRE(unpacked.autopilot == kMavAutopilotArdupilotmega);
    REQUIRE(unpacked.base_mode == 0x81);
    REQUIRE(unpacked.system_status == 4);
    REQUIRE(unpacked.mavlink_version == kMavlinkVersion);

    Frame frame{};
    REQUIRE(make_frame(7, 1, 1, kMsgIdHeartbeat, payload, frame));
    std::array<std::uint8_t, 32> wire{};
    const std::size_t n = encode_v2(frame, wire);
    REQUIRE(n == 21);
    REQUIRE(wire[0] == kStxV2);

    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().seq == 7);
    REQUIRE(decoded.value().sysid == 1);
    REQUIRE(decoded.value().compid == 1);
    REQUIRE(decoded.value().msgid == kMsgIdHeartbeat);

    Heartbeat from_frame{};
    REQUIRE(heartbeat_from_frame(decoded.value(), from_frame));
    REQUIRE(from_frame.custom_mode == 12);
    REQUIRE(from_frame.type == kMavTypeFixedWing);
    REQUIRE(from_frame.autopilot == kMavAutopilotArdupilotmega);
    REQUIRE(from_frame.mavlink_version == kMavlinkVersion);
}

TEST_CASE("MAVLink2 rejects bad CRC", "[gcs][framing]") {
    const Heartbeat hb = plane_heartbeat(kMavTypeFixedWing, 0x81, 12, 4);
    std::array<std::uint8_t, kHeartbeatLen> payload{};
    REQUIRE(pack_heartbeat(hb, payload) == kHeartbeatLen);
    Frame frame{};
    REQUIRE(make_frame(0, 1, 1, kMsgIdHeartbeat, payload, frame));
    std::array<std::uint8_t, 32> wire{};
    const std::size_t n = encode_v2(frame, wire);
    REQUIRE(n >= 2);
    wire[n - 2] = static_cast<std::uint8_t>(wire[n - 2] ^ 0x01);

    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE_FALSE(decoded.has_value());
    REQUIRE(decoded.error() == DecodeError::kBadCrc);
}

TEST_CASE("unknown msgid is not dispatched", "[gcs][dispatch]") {
    Frame frame{};
    REQUIRE(make_frame(0, 255, 190, 253, {}, frame));
    GcsMavlink gcs;
    const auto d = gcs.handle_message(frame, 0);
    REQUIRE(d.kind == DispatchKind::kUnknown);
    REQUIRE(d.msgid == 253);
    REQUIRE(gcs.last_gcs_heartbeat_ms() == 0);
}

TEST_CASE("send_heartbeat dispatches as msgid 0", "[gcs][dispatch]") {
    GcsMavlink sender;
    std::array<std::uint8_t, 32> wire{};
    const std::size_t n = sender.send_heartbeat(wire, kMavTypeFixedWing, 0x81, 12, 4);
    REQUIRE(n > 0);

    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdHeartbeat);

    GcsMavlink peer;
    const auto d = peer.handle_message(decoded.value(), 1000);
    REQUIRE(d.kind == DispatchKind::kHeartbeat);
    REQUIRE_FALSE(d.from_gcs);
    REQUIRE(d.heartbeat.custom_mode == 12);
    REQUIRE(d.heartbeat.autopilot == kMavAutopilotArdupilotmega);
    REQUIRE(peer.last_gcs_heartbeat_ms() == 0);
}

TEST_CASE("leftover catalog this slice vs remaining", "[gcs][leftover]") {
    REQUIRE(remaining_count() == 4);
    REQUIRE(this_slice_count() == 1);
    REQUIRE(on_main_count() == 4);
    REQUIRE(leftover_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count());
    REQUIRE(completeness_has("MAVLink 2 framing", PortStatus::kOnMain));
    REQUIRE(completeness_has("HEARTBEAT pack/unpack", PortStatus::kOnMain));
    REQUIRE(completeness_has("msgid dispatch stub", PortStatus::kOnMain));
    REQUIRE(completeness_has("leftover catalog", PortStatus::kOnMain));
    REQUIRE(completeness_has("COMMAND_LONG", PortStatus::kThisSlice));
    REQUIRE(completeness_has("PARAM protocol", PortStatus::kRemaining));
    REQUIRE(completeness_has("MISSION", PortStatus::kRemaining));
    REQUIRE(completeness_has("Plane/Copter vehicle handlers", PortStatus::kRemaining));
    REQUIRE(completeness_has("XML dialect generation", PortStatus::kRemaining));
}

TEST_CASE("COMMAND_LONG pack/unpack round-trip", "[gcs][command_long]") {
    const CommandLong cmd = make_cmd(kMavCmdComponentArmDisarm, 1.0f, kMagicForceArmValue);
    std::array<std::uint8_t, kCommandLongLen> payload{};
    REQUIRE(pack_command_long(cmd, payload) == kCommandLongLen);

    CommandLong unpacked{};
    REQUIRE(unpack_command_long(payload, unpacked));
    REQUIRE(unpacked.param1 == 1.0f);
    REQUIRE(unpacked.param2 == kMagicForceArmValue);
    REQUIRE(unpacked.command == kMavCmdComponentArmDisarm);
    REQUIRE(unpacked.target_system == 1);
    REQUIRE(unpacked.target_component == 1);
    REQUIRE(unpacked.confirmation == 0);
}

TEST_CASE("COMMAND_ACK pack/unpack size-sorted v2", "[gcs][command_ack]") {
    CommandAck ack{};
    ack.command = kMavCmdDoSetMode;
    ack.result = static_cast<std::uint8_t>(MavResult::kAccepted);
    ack.progress = 0;
    ack.result_param2 = 0;
    ack.target_system = 255;
    ack.target_component = 190;
    std::array<std::uint8_t, kCommandAckLen> payload{};
    REQUIRE(pack_command_ack(ack, payload) == kCommandAckLen);
    REQUIRE(payload[4] == static_cast<std::uint8_t>(kMavCmdDoSetMode));
    REQUIRE(payload[5] == static_cast<std::uint8_t>(kMavCmdDoSetMode >> 8));

    CommandAck unpacked{};
    REQUIRE(unpack_command_ack(payload, unpacked));
    REQUIRE(unpacked.command == kMavCmdDoSetMode);
    REQUIRE(unpacked.result == static_cast<std::uint8_t>(MavResult::kAccepted));
    REQUIRE(unpacked.progress == 0);
    REQUIRE(unpacked.result_param2 == 0);
    REQUIRE(unpacked.target_system == 255);
    REQUIRE(unpacked.target_component == 190);
}

TEST_CASE("crc extra for COMMAND_LONG and COMMAND_ACK", "[gcs][framing]") {
    std::uint8_t extra = 0;
    REQUIRE(crc_extra(kMsgIdCommandLong, extra));
    REQUIRE(extra == kCommandLongCrcExtra);
    REQUIRE(extra == 152);
    REQUIRE(crc_extra(kMsgIdCommandAck, extra));
    REQUIRE(extra == kCommandAckCrcExtra);
    REQUIRE(extra == 143);
    REQUIRE_FALSE(crc_extra(253, extra));
}

TEST_CASE("ARM/DISARM and force-magic", "[gcs][arm]") {
    FakeVehicle veh;
    GcsMavlink gcs;
    gcs.set_hooks(make_hooks(veh));

    SECTION("arm when disarmed ACCEPTED and runs checks") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, 0.0f)));
        REQUIRE(d.kind == DispatchKind::kCommandLong);
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.armed);
        REQUIRE(veh.last_arm_checks);
        REQUIRE(d.command_ack.result == static_cast<std::uint8_t>(MavResult::kAccepted));
    }

    SECTION("already armed ACCEPTED without calling arm") {
        veh.armed = true;
        veh.arm_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.armed);
    }

    SECTION("force arm 2989 skips checks") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, kMagicForceArmValue)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE_FALSE(veh.last_arm_checks);
        REQUIRE(veh.armed);
    }

    SECTION("force arm 21196 skips checks") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, kMagicForceArmDisarmValue)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE_FALSE(veh.last_arm_checks);
    }

    SECTION("arm fail FAILED") {
        veh.arm_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kFailed);
        REQUIRE_FALSE(veh.armed);
    }

    SECTION("disarm when armed ACCEPTED with checks") {
        veh.armed = true;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 0.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE_FALSE(veh.armed);
        REQUIRE(veh.last_disarm_checks);
    }

    SECTION("already disarmed ACCEPTED") {
        veh.armed = false;
        veh.disarm_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 0.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
    }

    SECTION("force disarm 21196 skips checks") {
        veh.armed = true;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 0.0f, kMagicForceArmDisarmValue)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE_FALSE(veh.last_disarm_checks);
        REQUIRE_FALSE(veh.armed);
    }

    SECTION("disarm fail FAILED") {
        veh.armed = true;
        veh.disarm_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 0.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kFailed);
        REQUIRE(veh.armed);
    }

    SECTION("other param1 UNSUPPORTED") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 2.0f, 0.0f)));
        REQUIRE(d.command_result == MavResult::kUnsupported);
    }
}

TEST_CASE("DO_SET_MODE custom vs safety switch", "[gcs][set_mode]") {
    FakeVehicle veh;
    GcsMavlink gcs;
    gcs.set_hooks(make_hooks(veh));

    SECTION("custom mode enabled calls set_mode ACCEPTED") {
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagCustomModeEnabled), 12.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.last_custom_mode == 12);
    }

    SECTION("custom mode fail FAILED") {
        veh.set_mode_ok = false;
        const auto d = gcs.handle_command_long_frame(
            frame_command_long(make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagCustomModeEnabled), 5.0f)));
        REQUIRE(d.command_result == MavResult::kFailed);
    }

    SECTION("safety off custom 0 ACCEPTED") {
        const auto d = gcs.handle_command_long_frame(frame_command_long(
            make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagDecodePositionSafety), 0.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.safety_off_calls == 1);
        REQUIRE(veh.safety_on_calls == 0);
    }

    SECTION("safety on custom 1 ACCEPTED") {
        const auto d = gcs.handle_command_long_frame(frame_command_long(
            make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagDecodePositionSafety), 1.0f)));
        REQUIRE(d.command_result == MavResult::kAccepted);
        REQUIRE(veh.safety_on_calls == 1);
    }

    SECTION("safety on custom 1 fail FAILED") {
        veh.safety_on_ok = false;
        const auto d = gcs.handle_command_long_frame(frame_command_long(
            make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagDecodePositionSafety), 1.0f)));
        REQUIRE(d.command_result == MavResult::kFailed);
    }

    SECTION("safety other custom DENIED") {
        const auto d = gcs.handle_command_long_frame(frame_command_long(
            make_cmd(kMavCmdDoSetMode, static_cast<float>(kMavModeFlagDecodePositionSafety), 2.0f)));
        REQUIRE(d.command_result == MavResult::kDenied);
    }

    SECTION("no custom bit DENIED") {
        const auto d =
            gcs.handle_command_long_frame(frame_command_long(make_cmd(kMavCmdDoSetMode, 0.0f, 12.0f)));
        REQUIRE(d.command_result == MavResult::kDenied);
        REQUIRE(veh.last_custom_mode == 0);
    }
}

TEST_CASE("unknown command UNSUPPORTED", "[gcs][command_long]") {
    FakeVehicle veh;
    GcsMavlink gcs;
    gcs.set_hooks(make_hooks(veh));
    const auto d = gcs.handle_command_long_frame(frame_command_long(make_cmd(22, 0.0f, 0.0f)));
    REQUIRE(d.kind == DispatchKind::kCommandLong);
    REQUIRE(d.command_result == MavResult::kUnsupported);
    REQUIRE(d.command_ack.result == static_cast<std::uint8_t>(MavResult::kUnsupported));
    REQUIRE(d.command_ack.command == 22);
}

TEST_CASE("COMMAND_ACK is framed after COMMAND_LONG", "[gcs][command_ack]") {
    FakeVehicle veh;
    GcsMavlink gcs;
    gcs.set_hooks(make_hooks(veh));
    const Frame in = frame_command_long(make_cmd(kMavCmdComponentArmDisarm, 1.0f, 0.0f), 255, 190);
    std::array<std::uint8_t, 48> wire{};
    const std::size_t n = gcs.handle_command_long(in, wire);
    REQUIRE(n == 10 + 10 + 2);
    REQUIRE(wire[0] == kStxV2);

    auto decoded = decode_v2(std::span<const std::uint8_t>(wire.data(), n));
    REQUIRE(decoded.has_value());
    REQUIRE(decoded.value().msgid == kMsgIdCommandAck);
    REQUIRE(decoded.value().sysid == 1);
    REQUIRE(decoded.value().compid == 1);

    CommandAck ack{};
    REQUIRE(command_ack_from_frame(decoded.value(), ack));
    REQUIRE(ack.command == kMavCmdComponentArmDisarm);
    REQUIRE(ack.result == static_cast<std::uint8_t>(MavResult::kAccepted));
    REQUIRE(ack.progress == 0);
    REQUIRE(ack.result_param2 == 0);
    REQUIRE(ack.target_system == 255);
    REQUIRE(ack.target_component == 190);
}
