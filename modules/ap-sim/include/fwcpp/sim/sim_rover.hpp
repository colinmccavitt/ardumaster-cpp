#pragma once

// Port of libraries/SITL/SIM_Rover.h/.cpp. Servo PWM uses original
// normalise_servo_input. dt is passed in (no frame_time_us).

#include <cmath>
#include <cstring>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class SimRover : public Aircraft {
public:
    static constexpr float MAX_YAW_RATE = 360.0f;
    float max_speed = 20.0f;
    float max_accel = 10.0f;
    float max_wheel_turn = 35.0f;
    float turning_circle = 1.8f;
    float skid_turn_rate = 140.0f;
    bool skid_steering = false;
    bool vectored_thrust = false;
    float vectored_angle_max = 90.0f;
    float vectored_turn_rate_max = 90.0f;
    bool omni3 = false;
    float omni3_max_speed = 2.3625f;
    float omni3_max_accel = 1.0f;
    float omni3_wheel_max_ang_vel = 50.0f;

    explicit SimRover(const char* frame_str = "rover") {
        skid_steering = std::strstr(frame_str, "skid") != nullptr;
        if (skid_steering) {
            max_accel = 14;
            max_speed = 4;
            return;
        }
        vectored_thrust = std::strstr(frame_str, "vector") != nullptr;
        omni3 = std::strstr(frame_str, "omni3mecanum") != nullptr;
    }

    static float normalise_servo_input(std::uint16_t input) { return 2 * ((input - 1000) / 1000.0f - 0.5f); }

    float turn_circle(float steering) const {
        if (std::fabs(steering) < 1.0e-6f) {
            return 0;
        }
        return turning_circle * std::sin(math::radians(max_wheel_turn)) / std::sin(math::radians(steering * max_wheel_turn));
    }

    float calc_yaw_rate(float steering, float speed) {
        if (skid_steering) {
            return math::constrain_value(steering * skid_turn_rate, -MAX_YAW_RATE, MAX_YAW_RATE);
        }
        if (vectored_thrust) {
            return math::constrain_value(steering * vectored_turn_rate_max, -MAX_YAW_RATE, MAX_YAW_RATE);
        }
        if (std::fabs(steering) < 1.0e-6f || std::fabs(speed) < 1.0e-6f) {
            return 0;
        }
        float d = turn_circle(steering);
        float c = static_cast<float>(M_PI) * d;
        float t = c / speed;
        float rate = math::constrain_value(360.0f / t, -MAX_YAW_RATE, MAX_YAW_RATE);
        return rate;
    }

    float calc_lat_accel(float steering_angle, float speed) {
        float yaw_rate = calc_yaw_rate(steering_angle, speed);
        return math::radians(yaw_rate) * speed;
    }

    void update_ackermann_or_skid(const SitlInput& input, float delta_time) {
        float steering, throttle;
        if (skid_steering) {
            const float motor1 = input.servos[0] ? normalise_servo_input(input.servos[0]) : 0;
            const float motor2 = input.servos[2] ? normalise_servo_input(input.servos[2]) : 0;
            steering = motor1 - motor2;
            throttle = 0.5f * (motor1 + motor2);
        } else {
            steering = input.servos[0] ? normalise_servo_input(input.servos[0]) : 0;
            throttle = input.servos[2] ? normalise_servo_input(input.servos[2]) : 0;
            if (vectored_thrust) {
                const float steering_angle_rad = math::radians(steering * vectored_angle_max);
                steering = std::sin(steering_angle_rad) * throttle;
                throttle *= std::cos(steering_angle_rad);
            }
        }
        math::Vector3f velocity_body = dcm.transposed() * velocity_ef;
        float speed = velocity_body.x;
        float yaw_rate = calc_yaw_rate(steering, speed);
        float target_speed = throttle * max_speed;
        float accel = max_accel * (target_speed - speed) / max_speed;
        gyro = math::Vector3f(0, 0, math::radians(yaw_rate));
        dcm.rotate(gyro * delta_time);
        dcm.normalize();
        accel_body = math::Vector3f(accel, 0, 0);
        accel_body.y += math::radians(yaw_rate) * speed;
    }

    void update_omni3(const SitlInput& input, float delta_time) {
        math::Vector3f wheel_ang_vel;
        for (std::uint8_t i = 0; i < 3; i++) {
            wheel_ang_vel[i] = input.servos[i] ? normalise_servo_input(input.servos[i]) : 0;
        }
        wheel_ang_vel *= omni3_wheel_max_ang_vel;
        constexpr math::Matrix3f Minv(-0.0215149f, 0.01575f, 0.0057649f, -0.0057649f, -0.01575f, 0.0215149f, 0.0875f, 0.0875f,
                                      0.0875f);
        math::Vector3f twist = Minv * wheel_ang_vel;
        math::Vector3f velocity_body = dcm.transposed() * velocity_ef;
        float accel_x = omni3_max_accel * (twist.x - velocity_body.x) / omni3_max_speed;
        float accel_y = omni3_max_accel * (twist.y - velocity_body.y) / omni3_max_speed;
        gyro = math::Vector3f(0, 0, twist.z);
        dcm.rotate(gyro * delta_time);
        dcm.normalize();
        accel_body = math::Vector3f(accel_x, accel_y, 0);
    }

    void update(const SitlInput& input, float delta_time) {
        if (omni3) {
            update_omni3(input, delta_time);
        } else {
            update_ackermann_or_skid(input, delta_time);
        }
        math::Vector3f accel_earth = dcm * accel_body;
        accel_earth += math::Vector3f(0, 0, kGravityMss);
        accel_earth.z = 0;
        accel_body = dcm.transposed() * (accel_earth + math::Vector3f(0, 0, -kGravityMss));
        velocity_ef += accel_earth * delta_time;
        position += velocity_ef * delta_time;
        update_position();
        time_advance(delta_time);
        update_mag_field_bf();
    }
};

}  // namespace fwcpp::sim
