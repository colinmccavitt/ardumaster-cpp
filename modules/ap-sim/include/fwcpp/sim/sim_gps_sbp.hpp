#pragma once

// Port of libraries/SITL/SIM_GPS_SBP_Common.h/.cpp, SIM_GPS_SBP.h/.cpp,
// SIM_GPS_SBP2.h/.cpp. Framing is 0x55 + type + sender + len + payload +
// crc16_ccitt over type/sender/len/payload.

#include <cstdint>
#include <cstring>

#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_gps.hpp>

namespace fwcpp::sim {

#if defined(__GNUC__)
#define FWCPP_SIM_PACKED __attribute__((packed))
#else
#define FWCPP_SIM_PACKED
#endif

class GPS_SBP_Common : public GPS_Backend {
public:
    using GPS_Backend::GPS_Backend;
    GPS_SBP_Common(const GPS_SBP_Common&) = delete;
    GPS_SBP_Common& operator=(const GPS_SBP_Common&) = delete;

protected:
    void sbp_send_message(std::uint16_t msg_type, std::uint16_t sender_id, std::uint8_t len, std::uint8_t* payload);
};

inline void GPS_SBP_Common::sbp_send_message(std::uint16_t msg_type, std::uint16_t sender_id, std::uint8_t len,
                                             std::uint8_t* payload) {
    if (len != 0 && payload == nullptr) {
        return;
    }
    std::uint8_t preamble = 0x55;
    write_to_autopilot(reinterpret_cast<char*>(&preamble), 1);
    write_to_autopilot(reinterpret_cast<char*>(&msg_type), 2);
    write_to_autopilot(reinterpret_cast<char*>(&sender_id), 2);
    write_to_autopilot(reinterpret_cast<char*>(&len), 1);
    if (len > 0) {
        write_to_autopilot(reinterpret_cast<char*>(payload), len);
    }
    std::uint16_t crc = crc16_ccitt(reinterpret_cast<std::uint8_t*>(&msg_type), 2, 0);
    crc = crc16_ccitt(reinterpret_cast<std::uint8_t*>(&sender_id), 2, crc);
    crc = crc16_ccitt(&len, 1, crc);
    crc = crc16_ccitt(payload, len, crc);
    write_to_autopilot(reinterpret_cast<char*>(&crc), 2);
}

struct FWCPP_SIM_PACKED SbpGpsTime {
    std::uint16_t wn;
    std::uint32_t tow;
    std::int32_t ns;
    std::uint8_t flags;
};
struct FWCPP_SIM_PACKED SbpPosLlh {
    std::uint32_t tow;
    double lat;
    double lon;
    double height;
    std::uint16_t h_accuracy;
    std::uint16_t v_accuracy;
    std::uint8_t n_sats;
    std::uint8_t flags;
};
struct FWCPP_SIM_PACKED SbpVelNed {
    std::uint32_t tow;
    std::int32_t n;
    std::int32_t e;
    std::int32_t d;
    std::uint16_t h_accuracy;
    std::uint16_t v_accuracy;
    std::uint8_t n_sats;
    std::uint8_t flags;
};
struct FWCPP_SIM_PACKED SbpDops {
    std::uint32_t tow;
    std::uint16_t gdop;
    std::uint16_t pdop;
    std::uint16_t tdop;
    std::uint16_t hdop;
    std::uint16_t vdop;
    std::uint8_t flags;
};
struct SbpHeartbeat {
    bool sys_error : 1;
    bool io_error : 1;
    bool nap_error : 1;
    std::uint8_t res : 5;
    std::uint8_t protocol_minor : 8;
    std::uint8_t protocol_major : 8;
    std::uint8_t res2 : 7;
    bool ext_antenna : 1;
};

class GPS_SBP : public GPS_SBP_Common {
public:
    using GPS_SBP_Common::GPS_SBP_Common;
    GPS_SBP(const GPS_SBP&) = delete;
    GPS_SBP& operator=(const GPS_SBP&) = delete;
    void publish(const GPS_Data* d) override;

private:
    std::uint32_t do_every_count_{0};
};

inline void GPS_SBP::publish(const GPS_Data* d) {
    SbpHeartbeat hb {};
    SbpGpsTime t {};
    SbpPosLlh pos {};
    SbpVelNed velned {};
    SbpDops dops {};
    const auto gps_tow = gps_time();
    t.wn = gps_tow.week;
    t.tow = gps_tow.ms;
    sbp_send_message(0x0100, 0x2222, sizeof(t), reinterpret_cast<std::uint8_t*>(&t));
    if (!d->have_lock) {
        return;
    }
    pos.tow = gps_tow.ms;
    pos.lon = d->longitude;
    pos.lat = d->latitude;
    pos.height = d->altitude;
    pos.h_accuracy = static_cast<std::uint16_t>(d->horizontal_acc * 1000);
    pos.v_accuracy = static_cast<std::uint16_t>(d->vertical_acc * 1000);
    pos.n_sats = d->have_lock ? d->num_sats : 3;
    pos.flags = 0;
    sbp_send_message(0x0201, 0x2222, sizeof(pos), reinterpret_cast<std::uint8_t*>(&pos));
    pos.flags = 1;
    sbp_send_message(0x0201, 0x2222, sizeof(pos), reinterpret_cast<std::uint8_t*>(&pos));
    velned.tow = gps_tow.ms;
    velned.n = static_cast<std::int32_t>(1e3 * d->speedN);
    velned.e = static_cast<std::int32_t>(1e3 * d->speedE);
    velned.d = static_cast<std::int32_t>(1e3 * d->speedD);
    velned.h_accuracy = 5e3;
    velned.v_accuracy = 5e3;
    velned.n_sats = d->have_lock ? d->num_sats : 3;
    sbp_send_message(0x0205, 0x2222, sizeof(velned), reinterpret_cast<std::uint8_t*>(&velned));
    do_every_count_++;
    if (do_every_count_ % 5 == 0) {
        dops.tow = gps_tow.ms;
        dops.gdop = 1;
        dops.pdop = 1;
        dops.tdop = 1;
        dops.hdop = 100;
        dops.vdop = 1;
        dops.flags = 1;
        sbp_send_message(0x0206, 0x2222, sizeof(dops), reinterpret_cast<std::uint8_t*>(&dops));
        hb = {};
        hb.protocol_major = 0;
        sbp_send_message(0xFFFF, 0x2222, sizeof(hb), reinterpret_cast<std::uint8_t*>(&hb));
    }
}

class GPS_SBP2 : public GPS_SBP_Common {
public:
    using GPS_SBP_Common::GPS_SBP_Common;
    GPS_SBP2(const GPS_SBP2&) = delete;
    GPS_SBP2& operator=(const GPS_SBP2&) = delete;
    void publish(const GPS_Data* d) override;

private:
    std::uint32_t do_every_count_{0};
};

inline void GPS_SBP2::publish(const GPS_Data* d) {
    SbpHeartbeat hb {};
    SbpGpsTime t {};
    SbpPosLlh pos {};
    SbpVelNed velned {};
    SbpDops dops {};
    const auto gps_tow = gps_time();
    t.wn = gps_tow.week;
    t.tow = gps_tow.ms;
    t.flags = 1;
    sbp_send_message(0x0102, 0x2222, sizeof(t), reinterpret_cast<std::uint8_t*>(&t));
    if (!d->have_lock) {
        return;
    }
    pos.tow = gps_tow.ms;
    pos.lon = d->longitude;
    pos.lat = d->latitude;
    pos.height = d->altitude;
    pos.h_accuracy = static_cast<std::uint16_t>(d->horizontal_acc * 1000);
    pos.v_accuracy = static_cast<std::uint16_t>(d->vertical_acc * 1000);
    pos.n_sats = d->have_lock ? d->num_sats : 3;
    pos.flags = 1;
    sbp_send_message(0x020A, 0x2222, sizeof(pos), reinterpret_cast<std::uint8_t*>(&pos));
    pos.flags = 4;
    sbp_send_message(0x020A, 0x2222, sizeof(pos), reinterpret_cast<std::uint8_t*>(&pos));
    velned.tow = gps_tow.ms;
    velned.n = static_cast<std::int32_t>(1e3 * d->speedN);
    velned.e = static_cast<std::int32_t>(1e3 * d->speedE);
    velned.d = static_cast<std::int32_t>(1e3 * d->speedD);
    velned.h_accuracy = static_cast<std::uint16_t>(1e3 * 0.5);
    velned.v_accuracy = static_cast<std::uint16_t>(1e3 * 0.5);
    velned.n_sats = d->have_lock ? d->num_sats : 3;
    velned.flags = 1;
    sbp_send_message(0x020E, 0x2222, sizeof(velned), reinterpret_cast<std::uint8_t*>(&velned));
    do_every_count_++;
    if (do_every_count_ % 5 == 0) {
        dops.tow = gps_tow.ms;
        dops.gdop = 1;
        dops.pdop = 1;
        dops.tdop = 1;
        dops.hdop = 100;
        dops.vdop = 1;
        dops.flags = 1;
        sbp_send_message(0x0208, 0x2222, sizeof(dops), reinterpret_cast<std::uint8_t*>(&dops));
        hb = {};
        hb.protocol_major = 2;
        sbp_send_message(0xFFFF, 0x2222, sizeof(hb), reinterpret_cast<std::uint8_t*>(&hb));
    }
}

#undef FWCPP_SIM_PACKED

}  // namespace fwcpp::sim
