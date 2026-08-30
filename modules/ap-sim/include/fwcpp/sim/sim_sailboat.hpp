#pragma once

// Port of libraries/SITL/SIM_Sailboat.h/.cpp and SIM_MotorBoat.h.
// Wave/tide params are plain fields (AP_Param dropped).

#include <cmath>
#include <cstring>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_rover.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

struct SailWave {
    std::int8_t enable = 0;
    float direction = 0;
    float speed = 0;
    float length = 10;
    float amp = 0;
};
struct SailTide {
    float speed = 0;
    float direction = 0;
};

class Sailboat : public Aircraft {
public:
    bool motor_connected = false;
    bool skid_steering = false;
    float sail_area = 1.0f;
    std::int8_t sail_type = 0;
    bool armed = true;
    SailWave wave{};
    SailTide tide{};

    float steering_angle_max = 35;
    float turning_circle = 1.8f;
    const float lift_curve[18] = {0.00f, 0.50f, 1.00f, 1.10f, 0.95f, 0.75f, 0.60f, 0.40f, 0.20f, 0.00f,
                                  -0.20f, -0.40f, -0.60f, -0.75f, -0.95f, -1.10f, -1.00f, -0.50f};
    const float drag_curve[18] = {0.10f, 0.10f, 0.20f, 0.40f, 0.80f, 1.20f, 1.50f, 1.70f, 1.90f, 1.95f,
                                  1.90f, 1.70f, 1.50f, 1.20f, 0.80f, 0.40f, 0.20f, 0.10f};
    const float hull_mass = 2.0f;
    math::Vector3f velocity_ef_water{};
    math::Vector3f wave_gyro{};
    float wave_heave = 0;
    float wave_phase = 0;

    explicit Sailboat(const char* frame_str = "sailboat") {
        motor_connected = (std::strcmp(frame_str, "sailboat-motor") == 0);
        skid_steering = std::strstr(frame_str, "skid") != nullptr;
        mass = hull_mass;
        ground_behavior = GroundBehavior::kNoMovement;
    }

    static float sq(float x) { return x * x; }

    void calc_lift_and_drag(float wind_speed, float angle_of_attack_deg, float& lift, float& drag) const {
        const std::uint16_t index_width_deg = 10;
        const std::uint8_t index_max = 17;
        angle_of_attack_deg = math::wrap_180(angle_of_attack_deg);
        const float aoa = std::fabs(angle_of_attack_deg);
        if (aoa <= 0.0f) {
            lift = lift_curve[0];
            drag = drag_curve[0];
        } else if (aoa >= index_max * index_width_deg) {
            lift = lift_curve[index_max];
            drag = drag_curve[index_max];
        } else {
            std::uint8_t index = static_cast<std::uint8_t>(math::constrain_value(aoa / index_width_deg, 0.0f, float(index_max)));
            float remainder = aoa - (index * index_width_deg);
            lift = math::linear_interpolate(lift_curve[index], lift_curve[index + 1], remainder, 0.0f, float(index_width_deg));
            drag = math::linear_interpolate(drag_curve[index], drag_curve[index + 1], remainder, 0.0f, float(index_width_deg));
        }
        lift *= wind_speed * sail_area;
        drag *= wind_speed * sail_area;
        if (math::is_negative(angle_of_attack_deg)) {
            lift *= -1;
        }
    }

    float get_turn_circle(float steering) const {
        if (math::is_zero(steering)) {
            return 0;
        }
        return turning_circle * std::sin(math::radians(steering_angle_max)) / std::sin(math::radians(steering * steering_angle_max));
    }

    float get_yaw_rate(float steering, float speed) const {
        float rate = 0.0f;
        if (math::is_zero(steering) || (!skid_steering && math::is_zero(speed))) {
            return rate;
        }
        if (math::is_zero(speed) && skid_steering) {
            rate = steering * static_cast<float>(M_PI) * 5;
        } else {
            float d = get_turn_circle(steering);
            float c = static_cast<float>(M_PI) * d;
            float t = c / speed;
            rate = 360.0f / t;
        }
        return rate;
    }

    void update_wave(float delta_time) {
        const float wave_heading = wave.direction;
        const float wave_speed = wave.speed;
        const float wave_lenght = wave.length;
        const float wave_amp = wave.amp;
        float r, p, y;
        dcm.to_euler(&r, &p, &y);
        if (wave.enable == 0 || !armed || math::is_zero(wave_amp)) {
            wave_gyro = math::Vector3f(-r, -p, 0.0f) * 1.0f;
            wave_heave = -velocity_ef.z * 1.0f;
            wave_phase = 0.0f;
            return;
        }
        const float boat_speed = velocity_ef.x * std::sin(math::radians(wave_heading)) + velocity_ef.y * std::cos(math::radians(wave_heading));
        const float aprarent_wave_distance = (wave_speed - boat_speed) * delta_time;
        const float apparent_wave_phase_change = (aprarent_wave_distance / wave_lenght) * (2.0f * static_cast<float>(M_PI));
        wave_phase += apparent_wave_phase_change;
        wave_phase = math::wrap_2PI(wave_phase);
        const float wave_slope = (wave_amp * 0.5f) * ((2.0f * static_cast<float>(M_PI)) / wave_lenght) * std::cos(wave_phase);
        const float wave_angle = std::atan(wave_slope);
        const float heading_dif = wave_heading - y;
        float angle_error_x = (std::sin(heading_dif) * wave_angle) - r;
        float angle_error_y = (std::cos(heading_dif) * wave_angle) - p;
        wave_gyro.x = angle_error_x * 1.0f;
        wave_gyro.y = angle_error_y * 1.0f;
        wave_gyro.z = 0.0f;
        if (wave.enable == 2) {
            wave_heave = (wave_slope - velocity_ef.z) * 1.0f;
        } else {
            wave_heave = 0.0f;
        }
    }

    void update(const SitlInput& input, float delta_time) {
        update_wind(input);
        float steering = 0.0f;
        if (skid_steering) {
            const float steering_left = input.servos[0] ? SimRover::normalise_servo_input(input.servos[0]) : 0;
            const float steering_right = input.servos[2] ? SimRover::normalise_servo_input(input.servos[2]) : 0;
            steering = steering_left - steering_right;
        } else {
            steering = input.servos[0] ? SimRover::normalise_servo_input(input.servos[0]) : 0;
        }
        math::Vector3f wind_apparent_ef = velocity_ef - wind_ef;
        const float wind_apparent_dir_ef = math::degrees(std::atan2(wind_apparent_ef.y, wind_apparent_ef.x));
        const float wind_apparent_speed = math::safe_sqrt(sq(wind_apparent_ef.x) + sq(wind_apparent_ef.y));
        float roll, pitch, yaw;
        dcm.to_euler(&roll, &pitch, &yaw);
        const float wind_apparent_dir_bf = math::wrap_180(wind_apparent_dir_ef - math::degrees(yaw));
        rpm[0] = wind_apparent_speed;
        airspeed_pitot = wind_apparent_speed;
        float aoa_deg = 0.0f;
        if (sail_type == 1) {
            float wing_angle_bf = math::constrain_value((input.servos[4] - 1500) / 500.0f * 90.0f, -90.0f, 90.0f);
            aoa_deg = wind_apparent_dir_bf - wing_angle_bf;
        } else {
            float mainsail_angle_bf = math::constrain_value((input.servos[3] - 1000) / 1000.0f * 90.0f, 0.0f, 90.0f);
            aoa_deg = std::max(std::fabs(wind_apparent_dir_bf) - mainsail_angle_bf, 0.0f);
            if (math::is_negative(wind_apparent_dir_bf)) {
                aoa_deg *= -1;
            }
        }
        float lift_wf, drag_wf;
        calc_lift_and_drag(wind_apparent_speed, aoa_deg, lift_wf, drag_wf);
        const float sin_rot_rad = std::sin(math::radians(wind_apparent_dir_bf));
        const float cos_rot_rad = std::cos(math::radians(wind_apparent_dir_bf));
        const float force_fwd = (lift_wf * sin_rot_rad) - (drag_wf * cos_rot_rad);
        math::Vector3f velocity_body = dcm.transposed() * velocity_ef_water;
        float speed = velocity_body.x;
        float yaw_rate = get_yaw_rate(steering, speed);
        gyro = math::Vector3f(0, 0, math::radians(yaw_rate)) + wave_gyro;
        dcm.rotate(gyro * delta_time);
        dcm.normalize();
        float hull_drag = sq(speed) * 0.5f;
        if (!math::is_positive(speed)) {
            hull_drag *= -1.0f;
        }
        float throttle_force = 0.0f;
        if (motor_connected) {
            if (skid_steering) {
                const float throttle_left = input.servos[0] ? SimRover::normalise_servo_input(input.servos[0]) : 0;
                const float throttle_right = input.servos[2] ? SimRover::normalise_servo_input(input.servos[2]) : 0;
                throttle_force = (0.5f * (throttle_left + throttle_right)) * 50.0f;
            } else {
                const float throttle_out = input.servos[2] ? SimRover::normalise_servo_input(input.servos[2]) : 0;
                throttle_force = throttle_out * 50.0f;
            }
        }
        accel_body = math::Vector3f((throttle_force + force_fwd) - hull_drag, 0, 0);
        accel_body /= hull_mass;
        accel_body.y += math::radians(yaw_rate) * speed;
        float r, p, y;
        dcm.to_euler(&r, &p, &y);
        math::Matrix3f temp_dcm;
        temp_dcm.from_euler(0.0f, 0.0f, y);
        math::Vector3f accel_earth = temp_dcm * accel_body;
        accel_earth.z = 0 + wave_heave;
        accel_body = dcm.transposed() * (accel_earth + math::Vector3f(0, 0, -kGravityMss));
        math::Vector3f tide_velocity_ef;
        if (armed && !math::is_zero(tide.speed)) {
            tide_velocity_ef.x = -std::cos(math::radians(tide.direction)) * tide.speed;
            tide_velocity_ef.y = -std::sin(math::radians(tide.direction)) * tide.speed;
            tide_velocity_ef.z = 0.0f;
        }
        velocity_ef_water += accel_earth * delta_time;
        velocity_ef = velocity_ef_water + tide_velocity_ef;
        position += velocity_ef * delta_time;
        update_position();
        time_advance(delta_time);
        update_mag_field_bf();
        update_wave(delta_time);
    }
};

class MotorBoat : public Sailboat {
public:
    explicit MotorBoat(const char* frame_str = "motorboat") : Sailboat(frame_str) {
        motor_connected = true;
        sail_area = 0.0;
    }
};

}  // namespace fwcpp::sim
