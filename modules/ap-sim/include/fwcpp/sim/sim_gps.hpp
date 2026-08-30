#pragma once

// Port of libraries/SITL/SIM_GPS.h sensor synthesis (aircraft location /
// velocity -> GPS sample). Protocol backends (UBLOX/NMEA/SBP/...) are the
// original SIM_GPS_* files; this is the plant-facing sample the harness
// consumes instead of leftover NED-to-sensor shortcuts.

#include <cmath>
#include <cstdint>

#include <fwcpp/location.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>

namespace fwcpp::sim {

struct GPS_Data {
    std::uint32_t timestamp_ms{0};
    double latitude{0};
    double longitude{0};
    float altitude{0};
    double speedN{0};
    double speedE{0};
    double speedD{0};
    double yaw_deg{0};
    double roll_deg{0};
    double pitch_deg{0};
    bool have_lock{true};
    float horizontal_acc{0.3f};
    float vertical_acc{0.5f};
    float speed_acc{0.3f};
    std::uint8_t num_sats{10};
    std::uint8_t fix_type{3};

    [[nodiscard]] float ground_track_rad() const {
        return std::atan2(static_cast<float>(speedE), static_cast<float>(speedN));
    }
    [[nodiscard]] float speed_2d() const {
        return std::sqrt(static_cast<float>(speedN * speedN + speedE * speedE));
    }
};

inline GPS_Data gps_data_from_aircraft(const Aircraft& aircraft) {
    GPS_Data d;
    d.latitude = aircraft.location.lat * 1.0e-7;
    d.longitude = aircraft.location.lng * 1.0e-7;
    d.altitude = aircraft.location.alt * 0.01f;
    d.speedN = aircraft.velocity_ef.x;
    d.speedE = aircraft.velocity_ef.y;
    d.speedD = aircraft.velocity_ef.z;
    d.have_lock = true;
    d.num_sats = 10;
    d.fix_type = 3;
    return d;
}


struct SitlGpsSample {
    std::int32_t lat{0};
    std::int32_t lng{0};
    std::int32_t alt_cm{0};
    math::Vector3f velocity_ef{};
    bool have_lock{true};
    std::uint8_t num_sats{10};
};

inline SitlGpsSample sitl_gps_from_aircraft(const Aircraft& aircraft) {
    SitlGpsSample s;
    s.lat = aircraft.location.lat;
    s.lng = aircraft.location.lng;
    s.alt_cm = aircraft.location.alt;
    s.velocity_ef = aircraft.velocity_ef;
    s.have_lock = true;
    s.num_sats = 10;
    return s;
}

}  // namespace fwcpp::sim
