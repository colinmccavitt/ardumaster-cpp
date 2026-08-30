#pragma once

// Port of libraries/SITL/SIM_Tracker.h/.cpp.

#include <cmath>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class Tracker : public Aircraft {
public:
    using Aircraft::Aircraft;
    const bool onoff = false;
    const float yawrate = 9.0f;
    const float pitchrate = 1.0f;
    const float pitch_range = 45;
    const float yaw_range = 170;
    const float zero_yaw = 270;
    const float zero_pitch = 10;
    std::uint64_t last_debug_us = 0;
    float pitch_input = 0;
    float yaw_input = 0;
    float yaw_current_relative = 0;
    float pitch_current_relative = 0;

    void update_position_servos(float /*delta_time*/, float& yaw_rate, float& pitch_rate) const {
        float pitch_target = pitch_input * pitch_range;
        float yaw_target = yaw_input * yaw_range;
        pitch_rate = math::constrain_value(pitch_target - pitch_current_relative, -pitchrate, pitchrate);
        yaw_rate = math::constrain_value(yaw_target - yaw_current_relative, -yawrate, yawrate);
    }

    void update_onoff_servos(float& yaw_rate, float& pitch_rate) const {
        if (std::fabs(yaw_input) < 0.1f) {
            yaw_rate = 0;
        } else if (yaw_input >= 0.1f) {
            yaw_rate = yawrate;
        } else {
            yaw_rate = -yawrate;
        }
        if (std::fabs(pitch_input) < 0.1f) {
            pitch_rate = 0;
        } else if (pitch_input >= 0.1f) {
            pitch_rate = pitchrate;
        } else {
            pitch_rate = -pitchrate;
        }
    }

    void update(const SitlInput& input, float delta_time) {
        float yaw_rate = 0.0f, pitch_rate = 0.0f;
        yaw_input = (input.servos[0] - 1500) / 500.0f;
        pitch_input = (input.servos[1] - 1500) / 500.0f;
        float r, p, y;
        dcm.to_euler(&r, &p, &y);
        pitch_current_relative = math::degrees(p) - zero_pitch;
        yaw_current_relative = math::degrees(y) - zero_yaw;
        float roll_current = math::degrees(r);
        if (yaw_current_relative > 180) {
            yaw_current_relative -= 360;
        }
        if (yaw_current_relative < -180) {
            yaw_current_relative += 360;
        }
        if (yaw_rate > 0 && yaw_current_relative >= yaw_range) {
            yaw_rate = 0;
        }
        if (yaw_rate < 0 && yaw_current_relative <= -yaw_range) {
            yaw_rate = 0;
        }
        if (pitch_rate > 0 && pitch_current_relative >= pitch_range) {
            pitch_rate = 0;
        }
        if (pitch_rate < 0 && pitch_current_relative <= -pitch_range) {
            pitch_rate = 0;
        }
        if (onoff) {
            update_onoff_servos(yaw_rate, pitch_rate);
        } else {
            update_position_servos(delta_time, yaw_rate, pitch_rate);
        }
        float roll_rate = 0 - roll_current;
        (void)roll_rate;
        gyro = math::Vector3f(math::radians(roll_rate), math::radians(pitch_rate), math::radians(yaw_rate));
        dcm.rotate(gyro * delta_time);
        dcm.normalize();
        math::Vector3f accel_earth = math::Vector3f(0, 0, -kGravityMss);
        accel_body = dcm.transposed() * accel_earth;
        velocity_ef.zero();
        update_position();
        time_advance(delta_time);
        update_mag_field_bf();
    }
};

class NoVehicle : public Aircraft {
public:
    explicit NoVehicle(const char* = "novehicle") {}
    void update(const SitlInput&, float) {}
};

}  // namespace fwcpp::sim
