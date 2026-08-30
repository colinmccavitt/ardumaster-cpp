#pragma once

// Port of libraries/SITL/SITL_Baro.cpp: pressure/temperature from
// Aircraft altitude via AP_Baro 1976 atmosphere.

#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>

namespace fwcpp::sim {

struct SitlBaroSample {
    float pressure_pa{101325.0f};
    float temperature_k{288.15f};
    float altitude_amsl_m{0.0f};
};

inline SitlBaroSample sitl_baro_from_aircraft(const Aircraft& aircraft) {
    SitlBaroSample s;
    s.altitude_amsl_m = aircraft.location.alt * 0.01f;
    get_pressure_temperature_for_alt_amsl(s.altitude_amsl_m, s.pressure_pa, s.temperature_k);
    return s;
}

}  // namespace fwcpp::sim
