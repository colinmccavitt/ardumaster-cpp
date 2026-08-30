#pragma once

// Port of libraries/SITL/SIM_Balloon.h/.cpp.

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class Balloon : public Aircraft {
public:
    float terminal_rotation_rate = math::radians(100.0f);
    float climb_rate = 20;
    float terminal_velocity = 40;
    float burst_altitude = 20000;
    bool burst = false;
    bool released = false;

    explicit Balloon(const char* = "balloon") { mass = 5.0f; }

    void update(const SitlInput& input, float delta_time) {
        update_wind(input);
        if (!released && input.servos[6] > 1800) {
            released = true;
        }
        if (!burst && input.servos[7] > 1800) {
            burst = true;
        }
        math::Vector3f rot_accel = -gyro * math::radians(400.0f) / terminal_rotation_rate;
        math::Vector3f air_resistance = -velocity_air_ef * (kGravityMss / terminal_velocity) / eas2tas;
        float lift_accel = 0;
        if (!burst && released) {
            float air_resistance_at_climb_rate = climb_rate * (kGravityMss / terminal_velocity);
            lift_accel = air_resistance_at_climb_rate + kGravityMss * dcm.c.z;
        }
        accel_body = math::Vector3f(0, 0, -lift_accel);
        accel_body += dcm.transposed() * air_resistance;
        update_dynamics(rot_accel, delta_time);
        if (position.z < -burst_altitude) {
            burst = true;
        }
        update_position();
        time_advance(delta_time);
        update_mag_field_bf();
    }
};

}  // namespace fwcpp::sim
