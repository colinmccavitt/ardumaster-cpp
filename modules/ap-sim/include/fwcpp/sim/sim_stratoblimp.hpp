#pragma once

// Port of libraries/SITL/SIM_StratoBlimp.h/.cpp (ungated in Plane-4.7.0
// SIM_config.h). AP_Param members are plain floats with original defaults.

#include <cmath>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class StratoBlimp : public Aircraft {
public:
    float helium_mass = 13.54f;
    float arm_length = 3.6f;
    float motor_thrust = 145;
    float drag_fwd = 0.27f;
    float drag_side = 0.5f;
    float drag_up = 0.4f;
    float altitude_target = 20000;
    float target_climb_rate = 5;
    float motor_angle = 20;
    float yaw_rate_max = 60;
    float moi_roll = 1400;
    float moi_yaw = 2800;
    float moi_pitch = 3050;
    float center_of_lift = 2.54f;
    float center_of_drag = 0.3f;
    float free_lift_rate = 0.12f;
    float drag_yaw = 1.0f;
    bool released = false;
    bool helper_balloon_attached = true;
    float blimp_mass = 80;

    explicit StratoBlimp(const char* /*frame_str*/ = "stratoblimp") { mass = 80; }

    static float servo_angle(const SitlInput& input, std::uint8_t idx) {
        if (input.servos[idx] == 0) {
            return 0;
        }
        return (static_cast<float>(input.servos[idx]) - 1500.0f) / 500.0f;
    }
    static float servo_range(const SitlInput& input, std::uint8_t idx) {
        return (static_cast<float>(input.servos[idx]) - 1000.0f) / 1000.0f;
    }

    void handle_motor(float throttle, float tilt, math::Vector3f& body_acc, math::Vector3f& rot_accel,
                      float lateral_position) {
        const float angle_rad = math::radians(motor_angle) * tilt;
        const float thrust_x = motor_thrust * throttle;
        const float total_mass = blimp_mass + helium_mass;
        const math::Vector3f thrust{std::cos(angle_rad) * thrust_x, 0, -std::sin(angle_rad) * thrust_x};
        body_acc += thrust / total_mass;
        math::Vector3f pos{0, lateral_position, 0};
        math::Vector3f torque = (pos % thrust);
        rot_accel.z += torque.z / moi_yaw;
    }

    void get_drag(const math::Vector3f& velocity_linear, const math::Vector3f& velocity_rot,
                  math::Vector3f& drag_linear, math::Vector3f& drag_rotaccel) {
        const float drag_x_sign = velocity_linear.x > 0 ? -1.0f : 1.0f;
        const float drag_y_sign = velocity_linear.y > 0 ? -1.0f : 1.0f;
        const float drag_z_sign = velocity_linear.z > 0 ? -1.0f : 1.0f;
        drag_linear.x = 0.5f * drag_x_sign * air_density * velocity_linear.x * velocity_linear.x * drag_fwd;
        drag_linear.y = 0.5f * drag_y_sign * air_density * velocity_linear.y * velocity_linear.y * drag_fwd;
        drag_linear.z = 0.5f * drag_z_sign * air_density * velocity_linear.z * velocity_linear.z * drag_up;
        drag_rotaccel = -velocity_rot * drag_yaw;
        math::Vector3f drag_force = drag_linear * blimp_mass;
        math::Vector3f drag_pos{-center_of_drag, 0, -center_of_lift};
        drag_rotaccel += (drag_pos % drag_force) / moi_pitch;
    }

    float get_lift(float altitude) {
        float lift_accel = kGravityMss;
        if (helper_balloon_attached) {
            lift_accel += kGravityMss * free_lift_rate;
            if (altitude >= altitude_target) {
                helper_balloon_attached = false;
            }
        }
        return blimp_mass * lift_accel;
    }

    void calculate_coefficients() {
        drag_yaw = 1.0f;
        math::Vector3f body_acc, rot_accel;
        handle_motor(1, 0, body_acc, rot_accel, -arm_length);
        math::Vector3f vel_bf, g, drag_linear, drag_rotaccel;
        g.z = math::radians(yaw_rate_max);
        get_drag(vel_bf, g, drag_linear, drag_rotaccel);
        if (drag_rotaccel.z != 0) {
            drag_yaw = rot_accel.z / -drag_rotaccel.z;
        }
    }

    void calculate_forces(const SitlInput& input, math::Vector3f& body_acc, math::Vector3f& rot_accel) {
        body_acc.zero();
        rot_accel.zero();
        handle_motor(servo_range(input, 2), servo_angle(input, 0), body_acc, rot_accel, -arm_length);
        handle_motor(servo_range(input, 3), servo_angle(input, 1), body_acc, rot_accel, arm_length);
        math::Vector3f drag_linear, drag_rotaccel;
        get_drag(velocity_air_bf, gyro, drag_linear, drag_rotaccel);
        body_acc += drag_linear;
        rot_accel += drag_rotaccel;
        if (servo_range(input, 4) > 0.9f) {
            released = true;
        }
        if (released) {
            math::Vector3f lift_thrust_ef{0, 0, -get_lift(location.alt * 0.01f)};
            math::Vector3f lift_thrust_bf = dcm.transposed() * lift_thrust_ef;
            body_acc += lift_thrust_bf / blimp_mass;
            math::Vector3f lift_pos{0, 0, -center_of_lift};
            rot_accel += (lift_pos % lift_thrust_bf) / moi_roll;
        }
    }

    void update(const SitlInput& input, float delta_time) {
        mass = blimp_mass;
        air_density = get_air_density(location.alt * 0.01f);
        eas2tas = std::sqrt(kSslAirDensity / air_density);
        calculate_coefficients();
        math::Vector3f rot_accel{0, 0, 0};
        calculate_forces(input, accel_body, rot_accel);
        gyro += rot_accel * delta_time;
        gyro.x = math::constrain_value(gyro.x, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.y = math::constrain_value(gyro.y, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.z = math::constrain_value(gyro.z, -math::radians(2000.0f), math::radians(2000.0f));
        dcm.rotate(gyro * delta_time);
        dcm.normalize();
        update_dynamics(rot_accel, delta_time);
        update_position();
        update_wind(input);
        time_advance(delta_time);
        update_mag_field_bf();
    }
};

}  // namespace fwcpp::sim
