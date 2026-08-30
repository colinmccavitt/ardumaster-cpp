#pragma once

// CCP-045: port of libraries/SITL/SIM_Motor.h + SIM_Motor.cpp (Copter-4.7.0,
// same commit as Plane-4.7.0). Header-only, namespace fwcpp::sim.
//
// Upstream Motor::calculate_forces takes sitl_input PWM, converts to a
// 0..1 command (pwm_to_command), slews, computes disc-actuator thrust
// (calc_thrust), rotor yaw torque, optional tilt-servo rotation, optional
// momentum drag, then torque = position % thrust + rotor_torque.
//
// ADR-0012 adaptations (not behavior changes to the aero math):
//   - AP_HAL::micros64() is an explicit time_us argument (no HAL singleton).
//   - Battery voltage is an argument (no Battery / AP::sitl() singleton).
//   - struct sitl_input is SitlInput below (servos[32] PWM microseconds).

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::sim {

inline constexpr std::uint8_t kSitlServoChannels = 32;

// Upstream: libraries/SITL/SITL_Input.h `struct sitl_input`. Wind fields
// are omitted here (optional; SimMulticopter does not drive update_wind).
struct SitlInput {
    std::uint16_t servos[kSitlServoChannels]{};
};

// Upstream: AP_MotorsMatrix.h
inline constexpr float kMotorsYawFactorCw = -1.0f;
inline constexpr float kMotorsYawFactorCcw = 1.0f;

// Upstream: AP_Motors_Class.h AP_MOTORS_MOT_1.. (0-based servo index).
inline constexpr std::uint8_t kMot1 = 0;
inline constexpr std::uint8_t kMot2 = 1;
inline constexpr std::uint8_t kMot3 = 2;
inline constexpr std::uint8_t kMot4 = 3;
inline constexpr std::uint8_t kMot5 = 4;
inline constexpr std::uint8_t kMot6 = 5;
inline constexpr std::uint8_t kMot7 = 6;
inline constexpr std::uint8_t kMot8 = 7;
inline constexpr std::uint8_t kMot9 = 8;
inline constexpr std::uint8_t kMot10 = 9;
inline constexpr std::uint8_t kMot11 = 10;
inline constexpr std::uint8_t kMot12 = 11;
inline constexpr std::uint8_t kMot13 = 12;
inline constexpr std::uint8_t kMot14 = 13;
inline constexpr std::uint8_t kMot15 = 14;
inline constexpr std::uint8_t kMot16 = 15;
inline constexpr std::uint8_t kMot17 = 16;
inline constexpr std::uint8_t kMot18 = 17;
inline constexpr std::uint8_t kMot19 = 18;
inline constexpr std::uint8_t kMot20 = 19;
inline constexpr std::uint8_t kMot21 = 20;
inline constexpr std::uint8_t kMot22 = 21;
inline constexpr std::uint8_t kMot23 = 22;
inline constexpr std::uint8_t kMot24 = 23;
inline constexpr std::uint8_t kMot25 = 24;
inline constexpr std::uint8_t kMot26 = 25;
inline constexpr std::uint8_t kMot27 = 26;
inline constexpr std::uint8_t kMot28 = 27;
inline constexpr std::uint8_t kMot29 = 28;
inline constexpr std::uint8_t kMot30 = 29;
inline constexpr std::uint8_t kMot31 = 30;
inline constexpr std::uint8_t kMot32 = 31;

class Motor {
public:
    float angle{0.0f};
    float yaw_factor{0.0f};
    std::uint8_t servo{0};
    std::uint8_t display_order{0};

    std::int8_t roll_servo{-1};
    float roll_min{0.0f};
    float roll_max{0.0f};
    std::int8_t pitch_servo{-1};
    float pitch_min{0.0f};
    float pitch_max{0.0f};

    enum ServoType : std::uint8_t { kServoNormal = 0, kServoRetract = 1 };
    ServoType servo_type{kServoNormal};
    float servo_rate{0.24f};
    std::uint64_t last_change_usec{0};
    float last_roll_value{0.0f};
    float last_pitch_value{0.0f};

    Motor() = default;

    Motor(std::uint8_t servo_idx, float angle_deg, float yaw, std::uint8_t order)
        : angle(angle_deg), yaw_factor(yaw), servo(servo_idx), display_order(order) {
        position.x = std::cos(math::radians(angle));
        position.y = std::sin(math::radians(angle));
        position.z = 0.0f;
        thrust_vector = math::Vector3f{0.0f, 0.0f, -1.0f};
    }

    Motor(std::uint8_t servo_idx, float angle_deg, float yaw, std::uint8_t order, std::int8_t roll_srv, float rmin,
          float rmax, std::int8_t pitch_srv, float pmin, float pmax)
        : angle(angle_deg), yaw_factor(yaw), servo(servo_idx), display_order(order), roll_servo(roll_srv),
          roll_min(rmin), roll_max(rmax), pitch_servo(pitch_srv), pitch_min(pmin), pitch_max(pmax) {
        position.x = std::cos(math::radians(angle));
        position.y = std::sin(math::radians(angle));
        position.z = 0.0f;
        thrust_vector = math::Vector3f{0.0f, 0.0f, -1.0f};
    }

    void calculate_forces(const SitlInput& input, std::uint8_t motor_offset, math::Vector3f& torque,
                          math::Vector3f& thrust, const math::Vector3f& velocity_air_bf, const math::Vector3f& gyro,
                          float air_density, float voltage, bool use_drag, std::uint64_t time_us) {
        const float pwm = static_cast<float>(input.servos[motor_offset + servo]);
        float command = pwm_to_command(pwm);
        const float voltage_scale = voltage / voltage_max;

        if (voltage_scale < 0.1f) {
            torque.zero();
            thrust.zero();
            current = 0.0f;
            return;
        }

        if (last_calc_us != 0 && slew_max > 0.0f) {
            const float dt = static_cast<float>(time_us - last_calc_us) * 1.0e-6f;
            const float slew_max_change = slew_max * dt;
            command = math::constrain_value(command, last_command - slew_max_change, last_command + slew_max_change);
        }
        last_calc_us = time_us;
        last_command = command;

        math::Vector3f motor_vel = velocity_air_bf;
        motor_vel += -(position % gyro);

        const float velocity_in = std::max(0.0f, -motor_vel.projected(thrust_vector).z);
        const float motor_thrust = calc_thrust(command, air_density, velocity_in, voltage_scale);

        const float yaw_scale = 0.05f * diagonal_size * motor_thrust;
        math::Vector3f rotor_torque = thrust_vector * yaw_factor * command * yaw_scale * -1.0f;

        thrust = thrust_vector * motor_thrust;

        float roll = 0.0f;
        float pitch = 0.0f;

        if (roll_servo >= 0) {
            const std::uint16_t servoval = update_servo(
                input.servos[static_cast<std::uint8_t>(roll_servo) + motor_offset], time_us, last_roll_value);
            if (roll_min < roll_max) {
                roll = math::constrain_value(roll_min + (servoval - 1000) * 0.001f * (roll_max - roll_min), roll_min,
                                             roll_max);
            } else {
                roll = math::constrain_value(roll_max + (2000 - servoval) * 0.001f * (roll_min - roll_max), roll_max,
                                             roll_min);
            }
        }
        if (pitch_servo >= 0) {
            const std::uint16_t servoval = update_servo(
                input.servos[static_cast<std::uint8_t>(pitch_servo) + motor_offset], time_us, last_pitch_value);
            if (pitch_min < pitch_max) {
                pitch = math::constrain_value(pitch_min + (servoval - 1000) * 0.001f * (pitch_max - pitch_min),
                                              pitch_min, pitch_max);
            } else {
                pitch = math::constrain_value(pitch_max + (2000 - servoval) * 0.001f * (pitch_min - pitch_max),
                                              pitch_max, pitch_min);
            }
        }
        last_change_usec = time_us;

        if (!math::is_zero(roll) || !math::is_zero(pitch)) {
            math::Matrix3f rotation;
            rotation.from_euler(math::radians(roll), math::radians(pitch), 0.0f);
            thrust = rotation * thrust;
            rotor_torque = rotation * rotor_torque;
        }

        if (use_drag) {
            const float momentum_drag_factor = momentum_drag_coefficient * std::sqrt(air_density * true_prop_area);
            math::Vector3f momentum_drag;
            momentum_drag.x =
                momentum_drag_factor * motor_vel.x * (std::sqrt(std::fabs(thrust.y)) + std::sqrt(std::fabs(thrust.z)));
            momentum_drag.y =
                momentum_drag_factor * motor_vel.y * (std::sqrt(std::fabs(thrust.x)) + std::sqrt(std::fabs(thrust.z)));
            momentum_drag.z = momentum_drag_factor * motor_vel.z *
                              (std::sqrt(std::fabs(thrust.x)) + std::sqrt(std::fabs(thrust.y)) +
                               std::sqrt(std::fabs(thrust.z)));
            thrust -= momentum_drag;
        }

        torque = (position % thrust) + rotor_torque;

        const float power = power_factor * std::fabs(motor_thrust);
        current = power / std::max(voltage, 0.1f);
    }

    std::uint16_t update_servo(std::uint16_t demand, std::uint64_t time_usec, float& last_value) const {
        if (servo_rate <= 0.0f) {
            return demand;
        }
        if (servo_type == kServoRetract) {
            if (demand > 1700) {
                demand = 2000;
            } else if (demand < 1300) {
                demand = 1000;
            } else {
                demand = static_cast<std::uint16_t>(last_value);
            }
        }
        demand = static_cast<std::uint16_t>(math::constrain_value(static_cast<int>(demand), 1000, 2000));
        const float dt = static_cast<float>(time_usec - last_change_usec) * 1.0e-6f;
        const float max_change = 1000.0f * (dt / servo_rate) * 60.0f / 90.0f;
        last_value = math::constrain_value(static_cast<float>(demand), last_value - max_change, last_value + max_change);
        return static_cast<std::uint16_t>(last_value + 0.5f);
    }

    [[nodiscard]] float get_current() const { return current; }
    [[nodiscard]] float get_command() const { return last_command; }

    void setup_params(std::uint16_t pwm_min, std::uint16_t pwm_max, float spin_min, float spin_max, float expo,
                      float slew, float diag_size, float pwr_factor, float volt_max, float eff_prop_area,
                      float velocity_max, const math::Vector3f& pos, const math::Vector3f& thrust_vec, float yaw,
                      float true_area, float mdrag) {
        mot_pwm_min = static_cast<float>(pwm_min);
        mot_pwm_max = static_cast<float>(pwm_max);
        mot_spin_min = spin_min;
        mot_spin_max = spin_max;
        mot_expo = expo;
        slew_max = slew;
        power_factor = pwr_factor;
        voltage_max = volt_max;
        effective_prop_area = eff_prop_area;
        max_outflow_velocity = velocity_max;
        true_prop_area = true_area;
        momentum_drag_coefficient = mdrag;
        diagonal_size = diag_size;

        if (!pos.is_zero()) {
            position = pos;
        } else {
            position.x = std::cos(math::radians(angle)) * diag_size;
            position.y = std::sin(math::radians(angle)) * diag_size;
            position.z = 0.0f;
        }

        if (!thrust_vec.is_zero()) {
            thrust_vector = thrust_vec;
        }
        if (!math::is_zero(yaw)) {
            yaw_factor = yaw;
        }
    }

    [[nodiscard]] float pwm_to_command(float pwm) const {
        const float pwm_thrust_max = mot_pwm_min + mot_spin_max * (mot_pwm_max - mot_pwm_min);
        const float pwm_thrust_min = mot_pwm_min + mot_spin_min * (mot_pwm_max - mot_pwm_min);
        const float pwm_thrust_range = pwm_thrust_max - pwm_thrust_min;
        return math::constrain_value((pwm - pwm_thrust_min) / pwm_thrust_range, 0.0f, 1.0f);
    }

    [[nodiscard]] std::uint16_t command_to_pwm(float command) const {
        const float pwm_thrust_max = mot_pwm_min + mot_spin_max * (mot_pwm_max - mot_pwm_min);
        const float pwm_thrust_min = mot_pwm_min + mot_spin_min * (mot_pwm_max - mot_pwm_min);
        const float pwm_thrust_range = pwm_thrust_max - pwm_thrust_min;
        const float cmd = math::constrain_value(command, 0.0f, 1.0f);
        return static_cast<std::uint16_t>(pwm_thrust_min + cmd * pwm_thrust_range + 0.5f);
    }

    [[nodiscard]] float calc_thrust(float command, float air_density, float velocity_in, float voltage_scale) const {
        const float velocity_out = voltage_scale * max_outflow_velocity *
                                   std::sqrt((1.0f - mot_expo) * command + mot_expo * command * command);
        return 0.5f * air_density * effective_prop_area * (velocity_out * velocity_out - velocity_in * velocity_in);
    }

    void set_slew_max(float slew) { slew_max = slew; }

    [[nodiscard]] const math::Vector3f& get_position() const { return position; }
    [[nodiscard]] const math::Vector3f& get_thrust_vector() const { return thrust_vector; }

private:
    float mot_pwm_min{1000.0f};
    float mot_pwm_max{2000.0f};
    float mot_spin_min{0.15f};
    float mot_spin_max{0.95f};
    float mot_expo{0.65f};
    float slew_max{150.0f};
    float current{0.0f};
    float power_factor{1.0f};
    float voltage_max{12.6f};
    float effective_prop_area{0.0f};
    float max_outflow_velocity{0.0f};
    float true_prop_area{0.0f};
    float momentum_drag_coefficient{0.0f};
    float diagonal_size{0.35f};

    float last_command{0.0f};
    std::uint64_t last_calc_us{0};

    math::Vector3f position{};
    math::Vector3f thrust_vector{0.0f, 0.0f, -1.0f};
};

}  // namespace fwcpp::sim
