#pragma once

// Port of libraries/SITL/SIM_Calibration.h/.cpp. Four PWM modes.
// Quaternion error uses desired * current.inverse() (original operator/).

#include <cmath>
#include <cstdint>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class Calibration : public Aircraft {
public:
    static constexpr float MAX_ANGULAR_SPEED = static_cast<float>(2 * M_PI);
    bool use_smoothing = false;

    explicit Calibration(const char* /*frame_str*/ = "calibration") { mass = 1.5f; }

    void _stop_control(math::Vector3f& rot_accel, float dt) {
        math::Vector3f error = math::Vector3f{0, 0, 0} - gyro;
        rot_accel = error * (1.0f / dt);
        rot_accel *= 0.002f;
    }

    void _attitude_set(float desired_roll, float desired_pitch, float desired_yaw, math::Vector3f& rot_accel,
                       float dt) {
        math::Quaternion desired_q;
        desired_q.from_euler(desired_roll, desired_pitch, desired_yaw);
        desired_q.normalize();
        math::Quaternion current_q;
        current_q.from_rotation_matrix(dcm);
        current_q.normalize();
        math::Quaternion error_q = desired_q * current_q.inverse();
        math::Vector3f angle_differential;
        error_q.normalize();
        error_q.to_axis_angle(angle_differential);
        math::Vector3f desired_angvel = angle_differential * (1 / dt);
        desired_angvel *= 0.005f;
        math::Vector3f error = desired_angvel - gyro;
        rot_accel = error * (1.0f / dt);
    }

    void _attitude_control(const SitlInput& input, math::Vector3f& rot_accel, float dt) {
        float desired_roll = -static_cast<float>(M_PI) + 2 * static_cast<float>(M_PI) * (input.servos[5] - 1000) / 1000.f;
        float desired_pitch = -static_cast<float>(M_PI) + 2 * static_cast<float>(M_PI) * (input.servos[6] - 1000) / 1000.f;
        float desired_yaw = -static_cast<float>(M_PI) + 2 * static_cast<float>(M_PI) * (input.servos[7] - 1000) / 1000.f;
        _attitude_set(desired_roll, desired_pitch, desired_yaw, rot_accel, dt);
    }

    void _angular_velocity_control(const SitlInput& in, math::Vector3f& rot_accel, float dt) {
        math::Vector3f axis{static_cast<float>(in.servos[5] - 1500), static_cast<float>(in.servos[6] - 1500),
                            static_cast<float>(in.servos[7] - 1500)};
        float theta = MAX_ANGULAR_SPEED * (in.servos[4] - 1300) / 700.f;
        if (axis.length() > 0) {
            axis.normalize();
        }
        math::Vector3f desired_angvel = axis * theta;
        math::Vector3f error = desired_angvel - gyro;
        rot_accel = error * (1.0f / dt);
        rot_accel *= 0.05f;
    }

    void _calibration_poses(math::Vector3f& rot_accel, float tnow) {
        const struct Pose {
            std::int16_t roll, pitch, yaw;
            std::uint8_t axis;
        } poses[] = {
            {0, 0, 0, 0},  {0, 0, 0, 1},  {0, 0, 0, 2},   {90, 0, 0, 1}, {0, 90, 0, 1},  {0, 180, 0, 2},
            {45, 0, 0, 1}, {0, 45, 0, 2}, {0, 0, 45, 0},  {30, 0, 0, 1}, {0, 30, 0, 0},  {30, 0, 0, 1},
            {0, 0, 30, 0}, {0, 0, 30, 1}, {60, 20, 0, 1}, {0, 50, 10, 0},{0, 30, 50, 1}, {0, 30, 50, 2},
        };
        const float secs_per_pose = 6;
        const float rate = math::radians(360.0f / secs_per_pose);
        float t_in_pose = std::fmod(tnow, secs_per_pose);
        std::uint8_t pose_num = static_cast<std::uint8_t>(static_cast<unsigned>(tnow / secs_per_pose) % 18);
        const Pose& pose = poses[pose_num];
        use_smoothing = true;
        dcm.identity();
        dcm.from_euler(math::radians(pose.roll), math::radians(pose.pitch), math::radians(pose.yaw));
        math::Vector3f axis{0, 0, 0};
        axis[pose.axis] = 1;
        math::Matrix3f r2;
        r2.from_axis_angle(axis, rate * t_in_pose);
        dcm = r2 * dcm;
        accel_body = {0, 0, -kGravityMss};
        accel_body = dcm.transposed() * accel_body;
        (void)rot_accel;
    }

    void update(const SitlInput& input, float dt) {
        math::Vector3f rot_accel{0, 0, 0};
        float switcher_pwm = input.servos[4];
        if (switcher_pwm < 1100) {
            _stop_control(rot_accel, dt);
        } else if (switcher_pwm < 1200) {
            _attitude_control(input, rot_accel, dt);
        } else if (switcher_pwm < 1300) {
            _calibration_poses(rot_accel, static_cast<float>(time_now_us) * 1.0e-6f);
        } else {
            _angular_velocity_control(input, rot_accel, dt);
        }
        if (switcher_pwm < 1200 || switcher_pwm >= 1300) {
            accel_body.zero();
            update_dynamics(rot_accel, dt);
        }
        update_position();
        time_advance(dt);
        update_mag_field_bf();
    }
};

}  // namespace fwcpp::sim
