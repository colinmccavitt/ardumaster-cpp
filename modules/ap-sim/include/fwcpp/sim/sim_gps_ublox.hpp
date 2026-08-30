#pragma once

// Port of libraries/SITL/SIM_GPS_UBLOX.h/.cpp. UBX NAV POSLLH/STATUS/VELNED
// /SOL/DOP/PVT/TIMEGPS/SVINFO/RELPOSNED encoding matches the original.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_gps.hpp>

namespace fwcpp::sim {

#if defined(__GNUC__)
#define FWCPP_SIM_PACKED __attribute__((packed))
#else
#define FWCPP_SIM_PACKED
#endif

class GPS_UBlox : public GPS_Backend {
public:
    using GPS_Backend::GPS_Backend;
    GPS_UBlox(const GPS_UBlox&) = delete;
    GPS_UBlox& operator=(const GPS_UBlox&) = delete;

    void publish(const GPS_Data* d) override;

private:
    enum RELPOSNED : std::uint32_t {
        gnssFixOK = 1U << 0,
        diffSoln = 1U << 1,
        relPosValid = 1U << 2,
        carrSolnFloat = 1U << 3,
        carrSolnFixed = 1U << 4,
        isMoving = 1U << 5,
        refPosMiss = 1U << 6,
        refObsMiss = 1U << 7,
        relPosHeadingValid = 1U << 8,
        relPosNormalized = 1U << 9
    };
    struct FWCPP_SIM_PACKED ubx_nav_relposned {
        std::uint8_t version;
        std::uint8_t reserved1;
        std::uint16_t refStationId;
        std::uint32_t iTOW;
        std::int32_t relPosN;
        std::int32_t relPosE;
        std::int32_t relPosD;
        std::int32_t relPosLength;
        std::int32_t relPosHeading;
        std::uint8_t reserved2[4];
        std::int8_t relPosHPN;
        std::int8_t relPosHPE;
        std::int8_t relPosHPD;
        std::int8_t relPosHPLength;
        std::uint32_t accN;
        std::uint32_t accE;
        std::uint32_t accD;
        std::uint32_t accLength;
        std::uint32_t accHeading;
        std::uint8_t reserved3[4];
        std::uint32_t flags;
    };

    std::uint32_t next_timegps_send_ms_{0};

    void update_relposned(ubx_nav_relposned& relposned, std::uint32_t tow_ms, float yaw_deg);
    void send_ubx(std::uint8_t msgid, std::uint8_t* buf, std::uint16_t size);
};

inline void GPS_UBlox::send_ubx(std::uint8_t msgid, std::uint8_t* buf, std::uint16_t size) {
    const std::uint8_t PREAMBLE1 = 0xb5;
    const std::uint8_t PREAMBLE2 = 0x62;
    const std::uint8_t CLASS_NAV = 0x1;
    std::uint8_t hdr[6], chk[2];
    hdr[0] = PREAMBLE1;
    hdr[1] = PREAMBLE2;
    hdr[2] = CLASS_NAV;
    hdr[3] = msgid;
    hdr[4] = static_cast<std::uint8_t>(size & 0xFF);
    hdr[5] = static_cast<std::uint8_t>(size >> 8);
    chk[0] = chk[1] = hdr[2];
    chk[1] = static_cast<std::uint8_t>(chk[1] + (chk[0] = static_cast<std::uint8_t>(chk[0] + hdr[3])));
    chk[1] = static_cast<std::uint8_t>(chk[1] + (chk[0] = static_cast<std::uint8_t>(chk[0] + hdr[4])));
    chk[1] = static_cast<std::uint8_t>(chk[1] + (chk[0] = static_cast<std::uint8_t>(chk[0] + hdr[5])));
    for (std::uint16_t i = 0; i < size; i++) {
        chk[1] = static_cast<std::uint8_t>(chk[1] + (chk[0] = static_cast<std::uint8_t>(chk[0] + buf[i])));
    }
    write_to_autopilot(reinterpret_cast<char*>(hdr), sizeof(hdr));
    write_to_autopilot(reinterpret_cast<char*>(buf), size);
    write_to_autopilot(reinterpret_cast<char*>(chk), sizeof(chk));
}

inline void GPS_UBlox::update_relposned(ubx_nav_relposned& relposned, std::uint32_t tow_ms, float yaw_deg) {
    math::Vector3f ant1_pos{NAN, NAN, NAN};
    GpsParms* all = front.all_parms();
    for (std::uint8_t i = 0; i < kSimMaxGpsSensors; i++) {
        if (i == instance) {
            continue;
        }
        if (all[i].hdg_enabled != GpsHeading::BASE) {
            continue;
        }
        ant1_pos = all[i].pos_offset;
        break;
    }
    if (ant1_pos.is_nan()) {
        return;
    }
    const math::Vector3f ant2_pos = all[instance].pos_offset;
    math::Vector3f rel_antenna_pos = ant2_pos - ant1_pos;
    math::Matrix3f rot;
    const auto& st = front.world();
    math::Vector3f gyro(math::radians(st.roll_rate), math::radians(st.pitch_rate), math::radians(st.yaw_rate));
    rot.from_euler(math::radians(st.roll_deg), math::radians(st.pitch_deg), math::radians(yaw_deg));
    const float lag = all[instance].delay_ms * 0.001f;
    rot.rotate(gyro * (-lag));
    rel_antenna_pos = rot * rel_antenna_pos;
    relposned.version = 1;
    relposned.iTOW = tow_ms;
    relposned.relPosN = static_cast<std::int32_t>(rel_antenna_pos.x * 100);
    relposned.relPosE = static_cast<std::int32_t>(rel_antenna_pos.y * 100);
    relposned.relPosD = static_cast<std::int32_t>(rel_antenna_pos.z * 100);
    relposned.relPosLength = static_cast<std::int32_t>(rel_antenna_pos.length() * 100);
    relposned.relPosHeading =
        static_cast<std::int32_t>(math::degrees(math::Vector2f(rel_antenna_pos.x, rel_antenna_pos.y).angle()) * 1.0e5);
    relposned.flags = gnssFixOK | diffSoln | carrSolnFixed | isMoving | relPosValid | relPosHeadingValid;
}

inline void GPS_UBlox::publish(const GPS_Data* d) {
    struct FWCPP_SIM_PACKED ubx_nav_posllh {
        std::uint32_t time;
        std::int32_t longitude;
        std::int32_t latitude;
        std::int32_t altitude_ellipsoid;
        std::int32_t altitude_msl;
        std::uint32_t horizontal_accuracy;
        std::uint32_t vertical_accuracy;
    } pos {};
    struct FWCPP_SIM_PACKED ubx_nav_status {
        std::uint32_t time;
        std::uint8_t fix_type;
        std::uint8_t fix_status;
        std::uint8_t differential_status;
        std::uint8_t res;
        std::uint32_t time_to_first_fix;
        std::uint32_t uptime;
    } status {};
    struct FWCPP_SIM_PACKED ubx_nav_velned {
        std::uint32_t time;
        std::int32_t ned_north;
        std::int32_t ned_east;
        std::int32_t ned_down;
        std::uint32_t speed_3d;
        std::uint32_t speed_2d;
        std::int32_t heading_2d;
        std::uint32_t speed_accuracy;
        std::uint32_t heading_accuracy;
    } velned {};
    struct FWCPP_SIM_PACKED ubx_nav_solution {
        std::uint32_t time;
        std::int32_t time_nsec;
        std::int16_t week;
        std::uint8_t fix_type;
        std::uint8_t fix_status;
        std::int32_t ecef_x;
        std::int32_t ecef_y;
        std::int32_t ecef_z;
        std::uint32_t position_accuracy_3d;
        std::int32_t ecef_x_velocity;
        std::int32_t ecef_y_velocity;
        std::int32_t ecef_z_velocity;
        std::uint32_t speed_accuracy;
        std::uint16_t position_DOP;
        std::uint8_t res;
        std::uint8_t satellites;
        std::uint32_t res2;
    } sol {};
    struct FWCPP_SIM_PACKED ubx_nav_dop {
        std::uint32_t time;
        std::uint16_t gDOP;
        std::uint16_t pDOP;
        std::uint16_t tDOP;
        std::uint16_t vDOP;
        std::uint16_t hDOP;
        std::uint16_t nDOP;
        std::uint16_t eDOP;
    } dop {};
    struct FWCPP_SIM_PACKED ubx_nav_pvt {
        std::uint32_t itow;
        std::uint16_t year;
        std::uint8_t month, day, hour, min, sec;
        std::uint8_t valid;
        std::uint32_t t_acc;
        std::int32_t nano;
        std::uint8_t fix_type;
        std::uint8_t flags;
        std::uint8_t flags2;
        std::uint8_t num_sv;
        std::int32_t lon, lat;
        std::int32_t height, h_msl;
        std::uint32_t h_acc, v_acc;
        std::int32_t velN, velE, velD, gspeed;
        std::int32_t head_mot;
        std::uint32_t s_acc;
        std::uint32_t head_acc;
        std::uint16_t p_dop;
        std::uint8_t reserved1[6];
        std::uint32_t headVeh;
        std::uint8_t reserved2[4];
    } pvt {};
    struct FWCPP_SIM_PACKED ubx_nav_timegps {
        std::uint32_t itow;
        std::int32_t ftow;
        std::uint16_t week;
        std::int8_t leapS;
        std::uint8_t valid;
        std::uint32_t tAcc;
    } timegps {};
    const std::uint8_t SV_COUNT = 10;
    struct FWCPP_SIM_PACKED ubx_nav_svinfo {
        std::uint32_t itow;
        std::uint8_t numCh;
        std::uint8_t globalFlags;
        std::uint8_t reserved1[2];
        struct FWCPP_SIM_PACKED svinfo_sv {
            std::uint8_t chn;
            std::uint8_t svid;
            std::uint8_t flags;
            std::uint8_t quality;
            std::uint8_t cno;
            std::int8_t elev;
            std::int16_t azim;
            std::int32_t prRes;
        } sv[SV_COUNT];
    } svinfo {};
    ubx_nav_relposned relposned {};
    const std::uint8_t MSG_POSLLH = 0x2;
    const std::uint8_t MSG_STATUS = 0x3;
    const std::uint8_t MSG_DOP = 0x4;
    const std::uint8_t MSG_VELNED = 0x12;
    const std::uint8_t MSG_SOL = 0x6;
    const std::uint8_t MSG_PVT = 0x7;
    const std::uint8_t MSG_TIMEGPS = 0x20;
    const std::uint8_t MSG_SVINFO = 0x30;
    const std::uint8_t MSG_RELPOSNED = 0x3c;

    std::uint32_t next_nav_sv_info_time = 0;
    const auto gps_tow = gps_time();

    pos.time = gps_tow.ms;
    pos.longitude = static_cast<std::int32_t>(d->longitude * 1.0e7);
    pos.latitude = static_cast<std::int32_t>(d->latitude * 1.0e7);
    pos.altitude_ellipsoid = static_cast<std::int32_t>(d->altitude * 1000.0f);
    pos.altitude_msl = static_cast<std::int32_t>(d->altitude * 1000.0f);
    pos.horizontal_accuracy = static_cast<std::uint32_t>(d->horizontal_acc * 1000);
    pos.vertical_accuracy = static_cast<std::uint32_t>(d->vertical_acc * 1000);

    status.time = gps_tow.ms;
    status.fix_type = d->have_lock ? 3 : 0;
    status.fix_status = d->have_lock ? 1 : 0;
    status.uptime = front.now_ms();

    velned.time = gps_tow.ms;
    velned.ned_north = static_cast<std::int32_t>(100.0f * d->speedN);
    velned.ned_east = static_cast<std::int32_t>(100.0f * d->speedE);
    velned.ned_down = static_cast<std::int32_t>(100.0f * d->speedD);
    velned.speed_2d = static_cast<std::uint32_t>(std::hypot(d->speedN, d->speedE) * 100);
    velned.speed_3d = static_cast<std::uint32_t>(std::sqrt(d->speedN * d->speedN + d->speedE * d->speedE + d->speedD * d->speedD) * 100);
    velned.heading_2d = static_cast<std::int32_t>(math::degrees(static_cast<float>(std::atan2(d->speedE, d->speedN))) * 100000.0f);
    if (velned.heading_2d < 0) {
        velned.heading_2d += static_cast<std::int32_t>(360.0f * 100000.0f);
    }
    velned.speed_accuracy = static_cast<std::uint32_t>(d->speed_acc * 100);
    velned.heading_accuracy = 4;

    std::memset(&sol, 0, sizeof(sol));
    sol.fix_type = d->have_lock ? 3 : 0;
    sol.fix_status = 221;
    sol.satellites = d->have_lock ? d->num_sats : 3;
    sol.time = gps_tow.ms;
    sol.week = static_cast<std::int16_t>(gps_tow.week);

    dop.time = gps_tow.ms;
    dop.gDOP = 65535;
    dop.pDOP = 65535;
    dop.tDOP = 65535;
    dop.vDOP = 200;
    dop.hDOP = 121;
    dop.nDOP = 65535;
    dop.eDOP = 65535;

    pvt.itow = gps_tow.ms;
    pvt.fix_type = d->have_lock ? 0x3 : 0;
    pvt.flags = 0b10000011;
    pvt.num_sv = d->have_lock ? d->num_sats : 3;
    pvt.lon = static_cast<std::int32_t>(d->longitude * 1.0e7);
    pvt.lat = static_cast<std::int32_t>(d->latitude * 1.0e7);
    pvt.height = static_cast<std::int32_t>(d->altitude * 1000.0f);
    pvt.h_msl = static_cast<std::int32_t>(d->altitude * 1000.0f);
    pvt.h_acc = static_cast<std::uint32_t>(d->horizontal_acc * 1000);
    pvt.v_acc = static_cast<std::uint32_t>(d->vertical_acc * 1000);
    pvt.velN = static_cast<std::int32_t>(1000.0f * d->speedN);
    pvt.velE = static_cast<std::int32_t>(1000.0f * d->speedE);
    pvt.velD = static_cast<std::int32_t>(1000.0f * d->speedD);
    pvt.gspeed = static_cast<std::int32_t>(std::hypot(d->speedN, d->speedE) * 1000);
    pvt.head_mot = static_cast<std::int32_t>(math::degrees(static_cast<float>(std::atan2(d->speedE, d->speedN))) * 1.0e5);
    pvt.s_acc = velned.speed_accuracy;
    pvt.head_acc = static_cast<std::uint32_t>(38 * 1.0e5);
    pvt.p_dop = 65535;

    timegps.itow = gps_tow.ms;
    timegps.week = gps_tow.week;
    timegps.valid = d->have_lock ? 0x03 : 0x00;

    switch (front.parms().hdg_enabled) {
    case GpsHeading::NONE:
    case GpsHeading::BASE:
        break;
    case GpsHeading::THS:
    case GpsHeading::KSXT:
    case GpsHeading::HDT:
        update_relposned(relposned, gps_tow.ms, static_cast<float>(d->yaw_deg));
        break;
    }

    send_ubx(MSG_POSLLH, reinterpret_cast<std::uint8_t*>(&pos), sizeof(pos));
    send_ubx(MSG_STATUS, reinterpret_cast<std::uint8_t*>(&status), sizeof(status));
    send_ubx(MSG_VELNED, reinterpret_cast<std::uint8_t*>(&velned), sizeof(velned));
    const bool is_f9p = (front.parms().options & static_cast<std::int32_t>(GpsOptions::UBX_IS_F9P)) != 0;
    if (is_f9p) {
        const std::uint32_t now_ms = front.now_ms();
        if (static_cast<std::int32_t>(now_ms - next_timegps_send_ms_) >= 0) {
            next_timegps_send_ms_ = now_ms + 1000;
            send_ubx(MSG_TIMEGPS, reinterpret_cast<std::uint8_t*>(&timegps), sizeof(timegps));
        }
    } else {
        send_ubx(MSG_SOL, reinterpret_cast<std::uint8_t*>(&sol), sizeof(sol));
    }
    send_ubx(MSG_DOP, reinterpret_cast<std::uint8_t*>(&dop), sizeof(dop));
    send_ubx(MSG_PVT, reinterpret_cast<std::uint8_t*>(&pvt), sizeof(pvt));
    if (front.parms().hdg_enabled > GpsHeading::NONE) {
        send_ubx(MSG_RELPOSNED, reinterpret_cast<std::uint8_t*>(&relposned), sizeof(relposned));
    }

    if (gps_tow.ms > next_nav_sv_info_time) {
        svinfo.itow = gps_tow.ms;
        svinfo.numCh = 32;
        svinfo.globalFlags = 4;
        for (std::uint8_t i = 0; i < SV_COUNT; i++) {
            svinfo.sv[i].chn = i;
            svinfo.sv[i].svid = i;
            svinfo.sv[i].flags = (i < d->num_sats) ? 0x7 : 0x6;
            svinfo.sv[i].quality = 7;
            svinfo.sv[i].cno = static_cast<std::uint8_t>(std::max(20, 30 - i));
            svinfo.sv[i].elev = static_cast<std::int8_t>(std::max(30, 90 - i));
            svinfo.sv[i].azim = i;
        }
        send_ubx(MSG_SVINFO, reinterpret_cast<std::uint8_t*>(&svinfo), sizeof(svinfo));
        next_nav_sv_info_time = gps_tow.ms + 10000;
    }
}

#undef FWCPP_SIM_PACKED

}  // namespace fwcpp::sim
