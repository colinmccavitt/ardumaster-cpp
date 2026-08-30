#pragma once

// Port of libraries/SITL/SIM_Submarine.h/.cpp. on_ground override is
// approximated by a deep ground_level so hagl stays positive (Aircraft
// on_ground is not virtual). buoyancy is a plain field.

#include <cmath>
#include <cstring>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class Thruster {
public:
    Thruster(std::int8_t servo_, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac,
             float lat_fac)
        : servo(servo_), linear(forward_fac, lat_fac, -throttle_fac), rotational(roll_fac, pitch_fac, yaw_fac) {}
    std::int8_t servo;
    math::Vector3f linear;
    math::Vector3f rotational;
};

class Submarine : public Aircraft {
public:
    const float water_density = 1023.6f;
    float buoyancy = 1.0f;
    struct {
        float length = 0.457f;
        float width = 0.338f;
        float height = 0.254f;
        float weight = 10.5f;
        float thrust = 51.48f;
        float thruster_mount_radius = 0.25f;
        float equivalent_sphere_radius = 0.2f;
        float volume = 4 * static_cast<float>(M_PI) * std::pow(0.2f, 3) / 3;
        float density = 500;
        float mass = (4 * static_cast<float>(M_PI) * std::pow(0.2f, 3) / 3) * 500;
        float moment_of_inertia = 2 * (((4 * static_cast<float>(M_PI) * std::pow(0.2f, 3) / 3) * 500) * std::pow(0.2f, 2) / 5);
        math::Vector3f linear_drag_coefficient{1.4f, 1.8f, 2.0f};
        math::Vector3f angular_drag_coefficient{1.05f, 1.05f, 1.05f};
        float equivalent_sphere_area = static_cast<float>(M_PI) * 0.25f *
                                       std::pow((4 * static_cast<float>(M_PI) * std::pow(0.2f, 3) / 3) * 3.0f / 4.0f, 2.0f / 3.0f);
    } frame_property;

    Thruster vectored_thrusters[6]{
        {0, 0, 0, 1.0f, 0, -1.0f, 1.0f},  {1, 0, 0, -1.0f, 0, -1.0f, -1.0f}, {2, 0, 0, -1.0f, 0, 1.0f, 1.0f},
        {3, 0, 0, 1.0f, 0, 1.0f, -1.0f},  {4, 1.0f, 0, 0, -1.0f, 0, 0},      {5, -1.0f, 0, 0, -1.0f, 0, 0},
    };
    Thruster vectored_6dof_thrusters[8]{
        {0, 0, 0, 1.0f, 0, -1.0f, 1.0f},     {1, 0, 0, -1.0f, 0, -1.0f, -1.0f}, {2, 0, 0, -1.0f, 0, 1.0f, 1.0f},
        {3, 0, 0, 1.0f, 0, 1.0f, -1.0f},     {4, 1.0f, -1.0f, 0, -1.0f, 0, 0},  {5, -1.0f, -1.0f, 0, -1.0f, 0, 0},
        {6, 1.0f, 1.0f, 0, -1.0f, 0, 0},     {7, -1.0f, 1.0f, 0, -1.0f, 0, 0},
    };
    Thruster* thrusters = vectored_thrusters;
    std::uint8_t n_thrusters = 6;

    explicit Submarine(const char* frame_str = "vectored") {
        frame_height = 0.0;
        ground_behavior = GroundBehavior::kNone;
        ground_level = -1.0e6f;
        if (std::strstr(frame_str, "vectored_6dof") != nullptr) {
            thrusters = vectored_6dof_thrusters;
            n_thrusters = 8;
        }
        mass = frame_property.mass;
    }

    float calculate_sea_floor_depth(const math::Vector3f&) const { return 50; }

    void calculate_drag_force(const math::Vector3f& velocity, const math::Vector3f& drag_coefficient,
                              math::Vector3f& force) const {
        const math::Vector3f velocity_2(std::fabs(velocity.x) * velocity.x, std::fabs(velocity.y) * velocity.y,
                                        std::fabs(velocity.z) * velocity.z);
        force = (velocity_2 * water_density) * frame_property.equivalent_sphere_area / 2.0f;
        force *= drag_coefficient;
    }

    void calculate_angular_drag_torque(const math::Vector3f& angular_velocity, const math::Vector3f& drag_coefficient,
                                       math::Vector3f& torque) const {
        math::Vector3f v_2(std::fabs(angular_velocity.x) * angular_velocity.x, std::fabs(angular_velocity.y) * angular_velocity.y,
                           std::fabs(angular_velocity.z) * angular_velocity.z);
        math::Vector3f f_d = v_2;
        f_d *= drag_coefficient * frame_property.equivalent_sphere_area * 1000 / 2;
        torque = f_d * frame_property.equivalent_sphere_radius;
    }

    void calculate_buoyancy_torque(math::Vector3f& torque) {
        const math::Vector3f force_up(0, 0, -40);
        const math::Vector3f force_position = dcm.transposed() * math::Vector3f(0, 0, 0.15f);
        torque = force_position % force_up;
    }

    float calculate_buoyancy_acceleration() {
        float below_water_level = position.z - frame_property.height / 2;
        if (below_water_level < 0) {
            return 0.0f;
        }
        if (below_water_level > frame_property.height / 2) {
            return kGravityMss + buoyancy / frame_property.mass;
        }
        return kGravityMss + (buoyancy * below_water_level / frame_property.height) / frame_property.mass;
    }

    void calculate_forces(const SitlInput& input, math::Vector3f& rot_accel, math::Vector3f& body_accel) {
        rot_accel = math::Vector3f(0, 0, 0);
        body_accel = dcm.transposed() * math::Vector3f(0, 0, -calculate_buoyancy_acceleration());
        for (int i = 0; i < n_thrusters; i++) {
            Thruster t = thrusters[i];
            std::int16_t pwm = static_cast<std::int16_t>(input.servos[t.servo]);
            float output = 0;
            if (pwm < 2000 && pwm > 1000 && (pwm < 1475 || pwm > 1525)) {
                output = (pwm - 1500) / 400.0f;
            }
            float thrust = output * std::fabs(output) * frame_property.thrust;
            body_accel += t.linear * thrust / frame_property.weight;
            rot_accel += t.rotational * thrust * frame_property.thruster_mount_radius / frame_property.moment_of_inertia;
        }
        const float floor_depth = calculate_sea_floor_depth(position);
        if (position.z > floor_depth && body_accel.z > -kGravityMss) {
            body_accel.z = -kGravityMss;
        }
        math::Vector3f linear_drag_forces;
        calculate_drag_force(velocity_air_bf, frame_property.linear_drag_coefficient, linear_drag_forces);
        body_accel -= linear_drag_forces / frame_property.weight;
        math::Vector3f angular_drag_torque;
        calculate_angular_drag_torque(gyro, frame_property.angular_drag_coefficient, angular_drag_torque);
        math::Vector3f buoyancy_torque;
        calculate_buoyancy_torque(buoyancy_torque);
        rot_accel -= angular_drag_torque / frame_property.moment_of_inertia;
        rot_accel += buoyancy_torque / frame_property.moment_of_inertia;
    }

    void update(const SitlInput& input, float delta_time) {
        update_wind(input);
        math::Vector3f rot_accel;
        calculate_forces(input, rot_accel, accel_body);
        update_dynamics(rot_accel, delta_time);
        update_position();
        time_advance(delta_time);
        update_mag_field_bf();
    }
};

}  // namespace fwcpp::sim
