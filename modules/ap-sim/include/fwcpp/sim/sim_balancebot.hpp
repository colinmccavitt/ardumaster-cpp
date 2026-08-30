#pragma once

// Port of libraries/SITL/SIM_BalanceBot.h/.cpp inverted pendulum on wheels.

#include <cmath>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_rover.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class BalanceBot : public Aircraft {
public:
    float velocity_vf_x = 0;
    float skid_turn_rate = 0.15708f;
    bool armed = true;

    explicit BalanceBot(const char* = "balancebot") { dcm.from_euler(0, 0, 0); }

    float calc_yaw_rate(float steering) const {
        float wheel_base_length = 0.15f;
        return steering * math::degrees(skid_turn_rate / wheel_base_length);
    }

    void update(const SitlInput& input, float delta_time) {
        const float m_p = 3.0f;
        const float l = 0.10f;
        const float i_p = 0.01250f;
        const float r_w = 0.05f;
        const float m_w = 0.1130f;
        const float i_w = 0.00015480f;
        const float R = 3.0f;
        const float k_e = 0.240f;
        const float k_t = 0.240f;
        const float v_max = 12.0f;
        const float gear_ratio = 50.0f;
        float motor1 = input.servos[0] ? SimRover::normalise_servo_input(input.servos[0]) : 0;
        float motor2 = input.servos[2] ? SimRover::normalise_servo_input(input.servos[2]) : 0;
        const float steering = motor1 - motor2;
        const float throttle = 0.5f * (motor1 + motor2);
        const float v = throttle * v_max;
        float yaw_rate = calc_yaw_rate(steering);
        float r, p, y;
        dcm.to_euler(&r, &p, &y);
        float theta = p;
        float ang_vel = gyro.y;
        if (!armed) {
            const float p_gain = 200;
            const float pitch_response = -std::sin(p) * p_gain * delta_time;
            ang_vel += pitch_response;
            const float y_gain = 100000;
            const float yaw_response = -std::sin(math::wrap_180(math::degrees(y))) * y_gain * delta_time;
            yaw_rate += yaw_response;
        }
        const float t1 = ((2.0f * gear_ratio * k_t * v / (R * r_w)) -
                          (2.0f * gear_ratio * k_t * k_e * velocity_vf_x / (R * r_w * r_w)) -
                          (m_p * l * ang_vel * ang_vel * std::sin(theta))) *
                         (i_p + m_p * l * l);
        const float t2 = -m_p * l * std::cos(theta) *
                         ((2.0f * gear_ratio * k_t * k_e * velocity_vf_x / (R * r_w)) - (2.0f * gear_ratio * k_t * v / (R)) +
                          (m_p * kGravityMss * l * std::sin(theta)));
        const float t3 = (((2.0f * m_w + 2.0f * i_w / (r_w * r_w) + m_p) * (i_p + m_p * l * l)) -
                          (m_p * m_p * l * l * std::cos(theta) * std::cos(theta)));
        const float accel_vf_x = (t1 - t2) / t3;
        const float angular_accel_bf_y =
            ((2.0f * gear_ratio * k_t * k_e * velocity_vf_x / (R * r_w)) - (2.0f * gear_ratio * k_t * v / (R)) +
             m_p * l * accel_vf_x * std::cos(theta) + m_p * kGravityMss * l * std::sin(theta)) /
            (i_p + m_p * l * l);
        accel_body = math::Vector3f(accel_vf_x * std::cos(theta), 0, -accel_vf_x * std::sin(theta));
        ang_vel += angular_accel_bf_y * delta_time;
        theta += ang_vel * delta_time;
        theta = std::fmod(theta, math::radians(360.0f));
        gyro = math::Vector3f(0, ang_vel, math::radians(yaw_rate));
        dcm.rotate(gyro * delta_time);
        dcm.normalize();
        accel_body.y += math::radians(yaw_rate) * velocity_vf_x;
        velocity_vf_x += accel_vf_x * delta_time;
        math::Vector3f accel_earth = dcm * accel_body;
        accel_earth += math::Vector3f(0, 0, kGravityMss);
        accel_earth.z = 0;
        if (!armed && p < math::radians(2.0f)) {
            accel_earth.zero();
            velocity_ef.zero();
            velocity_vf_x = 0;
            gyro[1] = 0;
            if (y < math::radians(2.0f)) {
                dcm.identity();
                gyro.zero();
            }
        }
        accel_body += dcm.transposed() * (math::Vector3f(0, 0, -kGravityMss));
        velocity_ef += accel_earth * delta_time;
        position += velocity_ef * delta_time;
        dcm.to_euler(&r, &p, &y);
        dcm.from_euler(0.0f, p, y);
        update_position();
        time_advance(delta_time);
        update_mag_field_bf();
    }
};

}  // namespace fwcpp::sim
