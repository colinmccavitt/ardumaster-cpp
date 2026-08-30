#pragma once

// Port of SIM_GPS.cpp check_backend_allocation() switch on Type.

#include <memory>

#include <fwcpp/sim/sim_gps.hpp>
#include <fwcpp/sim/sim_gps_file.hpp>
#include <fwcpp/sim/sim_gps_msp.hpp>
#include <fwcpp/sim/sim_gps_nmea.hpp>
#include <fwcpp/sim/sim_gps_nova.hpp>
#include <fwcpp/sim/sim_gps_sbf.hpp>
#include <fwcpp/sim/sim_gps_sbp.hpp>
#include <fwcpp/sim/sim_gps_trimble.hpp>
#include <fwcpp/sim/sim_gps_ublox.hpp>

namespace fwcpp::sim {

inline std::unique_ptr<GPS_Backend> make_gps_backend(GPS& gps, std::uint8_t instance, GpsType type) {
    switch (type) {
    case GpsType::NONE:
        return nullptr;
    case GpsType::UBLOX:
        return std::make_unique<GPS_UBlox>(gps, instance);
    case GpsType::NMEA:
        return std::make_unique<GPS_NMEA>(gps, instance);
    case GpsType::SBP:
        return std::make_unique<GPS_SBP>(gps, instance);
    case GpsType::SBP2:
        return std::make_unique<GPS_SBP2>(gps, instance);
    case GpsType::NOVA:
        return std::make_unique<GPS_NOVA>(gps, instance);
    case GpsType::MSP:
        return std::make_unique<GPS_MSP>(gps, instance);
    case GpsType::SBF:
        return std::make_unique<GPS_SBF>(gps, instance);
    case GpsType::TRIMBLE:
        return std::make_unique<GPS_Trimble>(gps, instance);
    case GpsType::FILE:
        return std::make_unique<GPS_FILE>(gps, instance);
    }
    return nullptr;
}

inline void register_gps_backends() {
    GPS::set_backend_factory(&make_gps_backend);
}

struct GpsBackendRegistration {
    GpsBackendRegistration() { register_gps_backends(); }
};

inline GpsBackendRegistration& gps_backend_registration() {
    static GpsBackendRegistration once;
    return once;
}

}  // namespace fwcpp::sim
