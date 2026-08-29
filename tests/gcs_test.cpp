// CPP-087 slice 1: MAVLink 2 framing, HEARTBEAT pack/unpack, msgid stub.

#include <array>
#include <cstdint>
#include <span>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/gcs/gcs.hpp>

using fwcpp::gcs::DecodeError;
using fwcpp::gcs::DispatchKind;
using fwcpp::gcs::Frame;
using fwcpp::gcs::GcsMavlink;
using fwcpp::gcs::Heartbeat;
using fwcpp::gcs::PortStatus;
using fwcpp::gcs::completeness_has;
using fwcpp::gcs::decode_v2;
using fwcpp::gcs::encode_v2;
using fwcpp::gcs::heartbeat_from_frame;
using fwcpp::gcs::kHeartbeatLen;
using fwcpp::gcs::kMavAutopilotArdupilotmega;
using fwcpp::gcs::kMavTypeFixedWing;
using fwcpp::gcs::kMavlinkVersion;
using fwcpp::gcs::kMsgIdHeartbeat;
using fwcpp::gcs::kStxV2;
using fwcpp::gcs::leftover_completeness_size;
using fwcpp::gcs::make_frame;
using fwcpp::gcs::on_main_count;
using fwcpp::gcs::pack_heartbeat;
using fwcpp::gcs::plane_heartbeat;
using fwcpp::gcs::remaining_count;
using fwcpp::gcs::this_slice_count;
using fwcpp::gcs::unpack_heartbeat;

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
    REQUIRE(remaining_count() > 0);
    REQUIRE(this_slice_count() > 0);
    REQUIRE(on_main_count() == 0);
    REQUIRE(leftover_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count());
    REQUIRE(completeness_has("MAVLink 2 framing", PortStatus::kThisSlice));
    REQUIRE(completeness_has("HEARTBEAT pack/unpack", PortStatus::kThisSlice));
    REQUIRE(completeness_has("msgid dispatch stub", PortStatus::kThisSlice));
    REQUIRE(completeness_has("COMMAND_LONG", PortStatus::kRemaining));
    REQUIRE(completeness_has("PARAM protocol", PortStatus::kRemaining));
    REQUIRE(completeness_has("MISSION", PortStatus::kRemaining));
    REQUIRE(completeness_has("Plane/Copter vehicle handlers", PortStatus::kRemaining));
    REQUIRE(completeness_has("XML dialect generation", PortStatus::kRemaining));
}
