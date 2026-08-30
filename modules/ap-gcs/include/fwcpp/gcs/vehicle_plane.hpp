#pragma once

// CPP-087 slice 5: Plane vehicle-handler leftover scaffold.
// Upstream: ArduPlane/GCS_MAVLink_Plane.cpp send_attitude() packs
// ATTITUDE (msgid 30) from AHRS roll/pitch/yaw + gyro rates.
// Injected PlaneAttitudeInputs stand in for AP_AHRS (ADR-0012).
// Copter vehicle handlers are not implemented this slice (catalog note).

#include <cstddef>
#include <cstdint>
#include <span>

#include <fwcpp/gcs/framing.hpp>

namespace fwcpp::gcs {

inline constexpr std::size_t kAttitudeLen = 28;

// Injected attitude/rates for leftover_send_attitude (no AHRS singleton).
struct PlaneAttitudeInputs {
    std::uint32_t time_boot_ms{};
    float roll{};
    float pitch{};
    float yaw{};
    float rollspeed{};
    float pitchspeed{};
    float yawspeed{};
};

// Size-sorted v2 wire matches mavlink_msg_attitude: time_boot_ms uint32,
// then roll/pitch/yaw/rollspeed/pitchspeed/yawspeed float. Len 28.
struct Attitude {
    std::uint32_t time_boot_ms{};
    float roll{};
    float pitch{};
    float yaw{};
    float rollspeed{};
    float pitchspeed{};
    float yawspeed{};
};

[[nodiscard]] inline Attitude attitude_from_inputs(const PlaneAttitudeInputs& in) {
    Attitude a{};
    a.time_boot_ms = in.time_boot_ms;
    a.roll = in.roll;
    a.pitch = in.pitch;
    a.yaw = in.yaw;
    a.rollspeed = in.rollspeed;
    a.pitchspeed = in.pitchspeed;
    a.yawspeed = in.yawspeed;
    return a;
}

// Pack 28 little-endian bytes. Returns kAttitudeLen, or 0 if buf is short.
[[nodiscard]] inline std::size_t pack_attitude(const Attitude& a, std::span<std::uint8_t> buf) {
    if (buf.size() < kAttitudeLen) {
        return 0;
    }
    write_u32_le(buf.data() + 0, a.time_boot_ms);
    write_f32_le(buf.data() + 4, a.roll);
    write_f32_le(buf.data() + 8, a.pitch);
    write_f32_le(buf.data() + 12, a.yaw);
    write_f32_le(buf.data() + 16, a.rollspeed);
    write_f32_le(buf.data() + 20, a.pitchspeed);
    write_f32_le(buf.data() + 24, a.yawspeed);
    return kAttitudeLen;
}

[[nodiscard]] inline bool unpack_attitude(std::span<const std::uint8_t> buf, Attitude& out) {
    if (buf.size() < kAttitudeLen) {
        return false;
    }
    out.time_boot_ms = read_u32_le(buf.data() + 0);
    out.roll = read_f32_le(buf.data() + 4);
    out.pitch = read_f32_le(buf.data() + 8);
    out.yaw = read_f32_le(buf.data() + 12);
    out.rollspeed = read_f32_le(buf.data() + 16);
    out.pitchspeed = read_f32_le(buf.data() + 20);
    out.yawspeed = read_f32_le(buf.data() + 24);
    return true;
}

[[nodiscard]] inline bool attitude_from_frame(const Frame& frame, Attitude& out) {
    if (frame.msgid != kMsgIdAttitude) {
        return false;
    }
    return unpack_attitude(frame.payload_bytes(), out);
}

// Plane leftover send_attitude: pack ATTITUDE msgid 30 and MAVLink2-frame
// it (CRC extra 39). Returns framed length, or 0 on short buffer / encode fail.
[[nodiscard]] inline std::size_t leftover_send_attitude(const PlaneAttitudeInputs& in,
                                                        std::span<std::uint8_t> out,
                                                        std::uint8_t seq = 0,
                                                        std::uint8_t sysid = 1,
                                                        std::uint8_t compid = 1) {
    const Attitude a = attitude_from_inputs(in);
    std::uint8_t payload[kAttitudeLen]{};
    if (pack_attitude(a, payload) == 0) {
        return 0;
    }
    Frame frame{};
    if (!make_frame(seq, sysid, compid, kMsgIdAttitude, payload, frame)) {
        return 0;
    }
    return encode_v2(frame, out);
}

// Copter vehicle handlers (GCS_MAVLINK_Copter) remain for a later slice.

}  // namespace fwcpp::gcs
