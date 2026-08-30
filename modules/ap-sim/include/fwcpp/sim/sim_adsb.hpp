#pragma once

// Port of libraries/SITL/SIM_ADSB.h/.cpp, SIM_ADSB_Device.h, and the
// Sagetech MXS packers from SIM_ADSB_Sagetech_MXS.h/.cpp.
// MAVLink ADSB_VEHICLE encode (GCS_MAVLink) is omitted; vehicle kinematics
// and Sagetech serial packing are original-source. rand_normal matches
// SIM_Aircraft.cpp Box-Muller (static n2 cache).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

#ifndef M_PER_SEC_TO_KNOTS
#define M_PER_SEC_TO_KNOTS 1.94384449f
#endif
#ifndef METRES_TO_FEET
#define METRES_TO_FEET 3.280839895013123
#endif
#ifndef AP_MSEC_PER_HOUR
#define AP_MSEC_PER_HOUR (3600U * 1000U)
#endif

namespace fwcpp::sim {

inline double aircraft_rand_normal(double mean, double stddev) {
    static double n2 = 0.0;
    static int n2_cached = 0;
    if (!n2_cached) {
        double x, y, r;
        do {
            x = 2.0 * std::rand() / RAND_MAX - 1;
            y = 2.0 * std::rand() / RAND_MAX - 1;
            r = x * x + y * y;
        } while (math::is_zero(static_cast<float>(r)) || r > 1.0);
        const double d = std::sqrt(-2.0 * std::log(r) / r);
        const double n1 = x * d;
        n2 = y * d;
        const double result = n1 * stddev + mean;
        n2_cached = 1;
        return result;
    }
    n2_cached = 0;
    return n2 * stddev + mean;
}

enum class AdsbEmitterType : std::uint8_t {
    NO_INFO = 0,
    LIGHT = 1,
    SMALL = 2,
    LARGE = 3,
    HIGH_VORTEX_LARGE = 4,
    HEAVY = 5,
    HIGHLY_MANUV = 6,
    ROTOCRAFT = 7,
    UNASSIGNED = 8,
    GLIDER = 9,
    LIGHTER_AIR = 10,
    PARACHUTE = 11,
    ULTRA_LIGHT = 12,
    UNASSIGNED2 = 13,
    UAV = 14,
    SPACE = 15,
    UNASSGINED3 = 16,
    EMERGENCY_SURFACE = 17,
    SERVICE_SURFACE = 18,
    POINT_OBSTACLE = 19,
};

struct AdsbParms {
    int plane_count = -1;
    float radius_m = 10000;
    float altitude_m = 1000;
};

class AdsbVehicle {
public:
    bool initialised = false;
    std::uint32_t ICAO_address = 0;
    math::Vector3f velocity_ef{};
    char callsign[9]{};
    math::Vector3d position{};
    Location location{};
    AdsbEmitterType type = AdsbEmitterType::NO_INFO;
    std::uint64_t stationary_object_created_ms = 0;

    [[nodiscard]] const Location& get_location() const { return location; }
    bool velocity(math::Vector3f& ret) const {
        ret = velocity_ef;
        return true;
    }

    void update(const Location& origin, const Location& aircraft_location, float delta_t, const AdsbParms& parms,
                std::uint64_t now_ms) {
        if (!initialised) {
            initialised = true;
            ICAO_address = static_cast<std::uint32_t>(std::rand() % 10000);
            std::snprintf(callsign, sizeof(callsign), "SIM%u", ICAO_address);
            const math::Vector2f aircraft_offset_ne = aircraft_location.get_distance_NE(origin);
            position.x = aircraft_offset_ne[1];
            position.y = aircraft_offset_ne[0];
            position.x += aircraft_rand_normal(0, parms.radius_m);
            position.y += aircraft_rand_normal(0, parms.radius_m);
            position.z = -std::fabs(parms.altitude_m);

            double vel_min = 5, vel_max = 20;
            if (position.length() > 500) {
                vel_min *= 3;
                vel_max *= 3;
            } else if (position.length() > 10000) {
                vel_min *= 10;
                vel_max *= 10;
            }
            type = static_cast<AdsbEmitterType>(std::rand() % (static_cast<int>(AdsbEmitterType::POINT_OBSTACLE) + 1));
            if (type == AdsbEmitterType::POINT_OBSTACLE) {
                stationary_object_created_ms = now_ms;
                velocity_ef.zero();
            } else {
                stationary_object_created_ms = 0;
                velocity_ef.x = static_cast<float>(aircraft_rand_normal(vel_min, vel_max));
                velocity_ef.y = static_cast<float>(aircraft_rand_normal(vel_min, vel_max));
                if (type < AdsbEmitterType::EMERGENCY_SURFACE) {
                    velocity_ef.z = static_cast<float>(aircraft_rand_normal(-3, 3));
                }
            }
        } else if (stationary_object_created_ms > 0 && now_ms - stationary_object_created_ms > AP_MSEC_PER_HOUR) {
            initialised = false;
        }

        position += velocity_ef.todouble() * static_cast<double>(delta_t);
        if (position.z > 0) {
            initialised = false;
        }

        Location ret = origin;
        ret.offset(static_cast<float>(position.x), static_cast<float>(position.y));
        location = ret;
    }
};

class Adsb : public SerialDevice {
public:
    static constexpr std::uint8_t num_vehicles_MAX = 200;
    std::uint8_t num_vehicles = 0;
    AdsbVehicle vehicles[num_vehicles_MAX]{};
    AdsbParms parms{};

    void update_simulated_vehicles(const Location& origin, const Location& aircraft_loc, std::uint32_t now_us) {
        if (parms.plane_count <= 0) {
            return;
        }
        if (parms.plane_count >= num_vehicles_MAX) {
            parms.plane_count = 0;
            num_vehicles = 0;
            return;
        }
        if (num_vehicles != static_cast<std::uint8_t>(parms.plane_count)) {
            num_vehicles = static_cast<std::uint8_t>(parms.plane_count);
            for (std::uint8_t i = 0; i < num_vehicles_MAX; i++) {
                vehicles[i].initialised = false;
            }
        }

        float delta_t = (now_us - last_update_us_) * 1.0e-6f;
        last_update_us_ = now_us;

        for (std::uint8_t i = 0; i < num_vehicles; i++) {
            auto& vehicle = vehicles[i];
            vehicle.update(origin, aircraft_loc, delta_t, parms, now_us / 1000U);
            if (aircraft_loc.get_distance(vehicle.get_location()) > parms.radius_m) {
                vehicle.initialised = false;
            }
        }
    }

private:
    std::uint32_t last_update_us_ = 0;
};

inline void pack_int32_into_uint8_ts(std::int32_t source, std::uint8_t dest[3]) {
    dest[0] = static_cast<std::uint8_t>(source >> 16);
    dest[1] = static_cast<std::uint8_t>(source >> 8);
    dest[2] = static_cast<std::uint8_t>(source >> 0);
}

inline void pack_scaled_geocoord(std::uint8_t buf[3], float coord) {
    const std::int32_t scaled = static_cast<std::int32_t>(coord * (1U << 23) / 180.0);
    pack_int32_into_uint8_ts(scaled, buf);
}

inline void pack_scaled_alt(std::uint8_t buf[3], float alt_m) {
    const std::int32_t scaled = static_cast<std::int32_t>(METRES_TO_FEET * alt_m * (1 / 0.015625));
    pack_int32_into_uint8_ts(scaled, buf);
}

inline std::uint8_t scaled_groundspeed(float speed_m_s) {
    if (math::is_zero(speed_m_s)) {
        return 0x01;
    }
    const float knots = M_PER_SEC_TO_KNOTS * speed_m_s;
    if (knots < 0.125f) {
        return 0x02;
    }
    static const struct Threshold {
        float min;
        std::uint8_t code;
        float increment;
    } thresholds[]{
        {0.125f, 0x03, 0.146f},
        {1, 0x09, 0.25f},
        {2, 0x0D, 0.50f},
        {15, 0x27, 1},
        {70, 0x5e, 2},
        {100, 0x6d, 5},
        {175, 0x7c, 0},
    };
    auto* entry = &thresholds[0];
    for (std::uint8_t i = 1; i < sizeof(thresholds) / sizeof(thresholds[0]); i++) {
        auto* next_entry = &thresholds[i];
        if (knots > entry->min && knots < next_entry->min) {
            const std::uint8_t code_delta = next_entry->code - entry->code;
            return entry->code + static_cast<std::uint8_t>((knots - entry->min) / code_delta);
        }
        entry = next_entry;
    }
    return 0x7c;
}

inline void pack_scaled_airspeed(std::uint8_t dest[2], float speed_m_s) {
    const std::int16_t scaled = static_cast<std::int16_t>(M_PER_SEC_TO_KNOTS * speed_m_s * 8);
    dest[0] = static_cast<std::uint8_t>(scaled >> 8);
    dest[1] = static_cast<std::uint8_t>(scaled >> 0);
}

inline void pack_scaled_vertical_rate(std::uint8_t dest[2], float speed_m_s) {
    const std::int16_t scaled = static_cast<std::int16_t>(METRES_TO_FEET * speed_m_s);
    dest[0] = static_cast<std::uint8_t>(scaled >> 8);
    dest[1] = static_cast<std::uint8_t>(scaled >> 0);
}

enum class SagetechMsgType : std::uint8_t {
    INSTALLATION = 0x01,
    FLIGHTID = 0x02,
    OPMSG = 0x03,
    GPS = 0x04,
    DATAREQ = 0x05,
    TARGETREQUEST = 0x0B,
    ACK = 0x80,
    STATEVECTORREPORT = 0x91,
    MODESTATUSREPORT = 0x92,
};

class AdsbSagetechMxs : public SerialDevice {
public:
    using SerialDevice::SerialDevice;

    std::uint8_t msgid = 0;

    void send_vehicle_message_state_vector(const AdsbVehicle& vehicle) {
        enum class SVR_ValidityFlag : std::uint8_t {
            LatitudeAndLongitude = (1U << 7),
            Altitude_Geometric = (1U << 6),
            NS_and_EW_Velocity = (1U << 5),
            VerticalRate_Geometric = (1U << 1),
        };
#pragma pack(push, 1)
        struct Report {
            std::uint8_t rs0;
            std::uint8_t rs1;
            std::uint8_t rs2;
            std::uint8_t validity_flags;
            std::uint8_t estimated_validity_flags;
            std::uint8_t participant_address[3];
            std::uint8_t address_qualifier;
            std::uint16_t epos_toa;
            std::uint16_t pos_toa;
            std::uint16_t vel_toa;
            std::uint8_t latitude[3];
            std::uint8_t longitude[3];
            std::uint8_t alt_geometric[3];
            std::uint8_t ns_velocity[2];
            std::uint8_t ew_velocity[2];
            std::uint8_t up_velocity[2];
        } my_report{};
#pragma pack(pop)

        my_report.rs0 = static_cast<std::uint8_t>(1 | (1 << 1) | (1 << 2) | (1 << 3));

        pack_int32_into_uint8_ts(static_cast<std::int32_t>(vehicle.ICAO_address), my_report.participant_address);
        my_report.address_qualifier = 0x02;

        const Location loc = vehicle.get_location();
        pack_scaled_geocoord(my_report.latitude, loc.lat * 1e-7f);
        pack_scaled_geocoord(my_report.longitude, loc.lng * 1e-7f);
        my_report.validity_flags |= static_cast<std::uint8_t>(SVR_ValidityFlag::LatitudeAndLongitude);

        pack_scaled_alt(my_report.alt_geometric, loc.alt * 0.01f);
        my_report.validity_flags |= static_cast<std::uint8_t>(SVR_ValidityFlag::Altitude_Geometric);

        pack_scaled_airspeed(my_report.ns_velocity, vehicle.velocity_ef.x);
        pack_scaled_airspeed(my_report.ew_velocity, vehicle.velocity_ef.y);
        my_report.validity_flags |= static_cast<std::uint8_t>(SVR_ValidityFlag::NS_and_EW_Velocity);

        pack_scaled_vertical_rate(my_report.up_velocity, -vehicle.velocity_ef.z);
        my_report.validity_flags |= static_cast<std::uint8_t>(SVR_ValidityFlag::VerticalRate_Geometric);

        const std::uint8_t payloadlen = static_cast<std::uint8_t>(sizeof(my_report));
        std::uint8_t frame[4 + sizeof(my_report) + 1]{};
        frame[0] = 0xAA;
        frame[1] = static_cast<std::uint8_t>(SagetechMsgType::STATEVECTORREPORT);
        frame[2] = msgid++;
        frame[3] = payloadlen;
        std::memcpy(frame + 4, &my_report, sizeof(my_report));
        std::uint8_t cs = 0;
        for (std::uint8_t i = 0; i < 4 + payloadlen; i++) {
            cs = static_cast<std::uint8_t>(cs + frame[i]);
        }
        frame[4 + payloadlen] = cs;
        write_to_autopilot(reinterpret_cast<const char*>(frame), sizeof(frame));
    }
};

}  // namespace fwcpp::sim
