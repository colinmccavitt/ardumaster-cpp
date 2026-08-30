#pragma once

// Port of libraries/SITL/SIM_GPS_NOVA.h/.cpp. NovAtel BESTPOS/BESTVEL/PSRDOP
// with crc_crc32 trailer. device_baud is 19200.

#include <cmath>
#include <cstdint>
#include <cstring>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_gps.hpp>

namespace fwcpp::sim {

#if defined(__GNUC__)
#define FWCPP_SIM_PACKED __attribute__((packed))
#else
#define FWCPP_SIM_PACKED
#endif

class GPS_NOVA : public GPS_Backend {
public:
    using GPS_Backend::GPS_Backend;
    GPS_NOVA(const GPS_NOVA&) = delete;
    GPS_NOVA& operator=(const GPS_NOVA&) = delete;
    void publish(const GPS_Data* d) override;
    std::uint32_t device_baud() const override { return 19200; }

private:
    void nova_send_message(std::uint8_t* header, std::uint8_t headerlength, std::uint8_t* payload,
                           std::uint8_t payloadlen);
    std::uint16_t sequence_{0};
};

inline void GPS_NOVA::nova_send_message(std::uint8_t* header, std::uint8_t headerlength, std::uint8_t* payload,
                                        std::uint8_t payloadlen) {
    write_to_autopilot(reinterpret_cast<char*>(header), headerlength);
    write_to_autopilot(reinterpret_cast<char*>(payload), payloadlen);
    std::uint32_t crc = crc_crc32(0, header, headerlength);
    crc = crc_crc32(crc, payload, payloadlen);
    write_to_autopilot(reinterpret_cast<char*>(&crc), 4);
}

inline void GPS_NOVA::publish(const GPS_Data* d) {
    struct FWCPP_SIM_PACKED nova_header {
        std::uint8_t preamble[3];
        std::uint8_t headerlength;
        std::uint16_t messageid;
        std::uint8_t messagetype;
        std::uint8_t portaddr;
        std::uint16_t messagelength;
        std::uint16_t sequence;
        std::uint8_t idletime;
        std::uint8_t timestatus;
        std::uint16_t week;
        std::uint32_t tow;
        std::uint32_t recvstatus;
        std::uint16_t resv;
        std::uint16_t recvswver;
    } header {};
    struct FWCPP_SIM_PACKED psrdop {
        float gdop;
        float pdop;
        float hdop;
        float htdop;
        float tdop;
        float cutoff;
        std::uint32_t svcount;
    } psrdop {};
    struct FWCPP_SIM_PACKED bestpos {
        std::uint32_t solstat;
        std::uint32_t postype;
        double lat;
        double lng;
        double hgt;
        float undulation;
        std::uint32_t datumid;
        float latsdev;
        float lngsdev;
        float hgtsdev;
        std::uint8_t stnid[4];
        float diffage;
        float sol_age;
        std::uint8_t svstracked;
        std::uint8_t svsused;
        std::uint8_t svsl1;
        std::uint8_t svsmultfreq;
        std::uint8_t resv;
        std::uint8_t extsolstat;
        std::uint8_t galbeisigmask;
        std::uint8_t gpsglosigmask;
    } bestpos {};
    struct FWCPP_SIM_PACKED bestvel {
        std::uint32_t solstat;
        std::uint32_t veltype;
        float latency;
        float age;
        double horspd;
        double trkgnd;
        double vertspd;
        float resv;
    } bestvel {};

    const auto gps_tow = gps_time();
    header.preamble[0] = 0xaa;
    header.preamble[1] = 0x44;
    header.preamble[2] = 0x12;
    header.headerlength = sizeof(header);
    header.week = gps_tow.week;
    header.tow = gps_tow.ms;

    header.messageid = 174;
    header.messagelength = sizeof(psrdop);
    header.sequence = ++sequence_;
    psrdop.hdop = 1.20f;
    psrdop.htdop = 1.20f;
    nova_send_message(reinterpret_cast<std::uint8_t*>(&header), sizeof(header), reinterpret_cast<std::uint8_t*>(&psrdop),
                      sizeof(psrdop));

    header.messageid = 99;
    header.messagelength = sizeof(bestvel);
    header.sequence = ++sequence_;
    bestvel.horspd = std::hypot(d->speedN, d->speedE);
    bestvel.trkgnd = math::degrees(static_cast<float>(std::atan2(d->speedE, d->speedN)));
    bestvel.vertspd = -d->speedD;
    nova_send_message(reinterpret_cast<std::uint8_t*>(&header), sizeof(header), reinterpret_cast<std::uint8_t*>(&bestvel),
                      sizeof(bestvel));

    header.messageid = 42;
    header.messagelength = sizeof(bestpos);
    header.sequence = ++sequence_;
    bestpos.lat = d->latitude;
    bestpos.lng = d->longitude;
    bestpos.hgt = d->altitude;
    bestpos.svsused = d->have_lock ? d->num_sats : 3;
    bestpos.latsdev = 0.2f;
    bestpos.lngsdev = 0.2f;
    bestpos.hgtsdev = 0.2f;
    bestpos.solstat = 0;
    bestpos.postype = 32;
    nova_send_message(reinterpret_cast<std::uint8_t*>(&header), sizeof(header), reinterpret_cast<std::uint8_t*>(&bestpos),
                      sizeof(bestpos));
}

#undef FWCPP_SIM_PACKED

}  // namespace fwcpp::sim
