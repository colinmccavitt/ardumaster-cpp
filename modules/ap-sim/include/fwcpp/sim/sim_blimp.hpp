#pragma once

// Port of libraries/SITL/SIM_Blimp.h/.cpp. Fin kinematics, thrust, drag and
// buoyancy match original (logging and HAL init-gate omitted; dt is explicit).

#include <cmath>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_atmosphere.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

struct BlimpFins {
    float angle = 0;
    float last_angle = 0;
    float servo_angle = 0;
    bool dir = false;
    float vel = 0;
    float T = 0;
    float N = 0;
    float Fx = 0;
    float Fy = 0;
    float Fz = 0;
};

class Blimp : public Aircraft {
public:
    math::Vector3f moment_of_inertia{0.004375f, 0.004375f, 0.004375f};
    math::Vector3f cog{0, 0, 0.1f};
    BlimpFins fin[4]{};
    float k_tan = 0.6e-7f;
    float k_nor = 0;
    float drag_constant = 0.05f;
    float drag_gyr_constant = 0.15f;
    float radius = 0.25f;

    explicit Blimp(const char* /*frame_str*/ = "blimp") { mass = 0.07f; }

    static float servo_angle(const SitlInput& input, std::uint8_t idx) {
        if (input.servos[idx] == 0) {
            return 0;
        }
        return (static_cast<float>(input.servos[idx]) - 1500.0f) / 500.0f;
    }

    void calculate_forces(const SitlInput& input, float delta_time, math::Vector3f& body_acc,
                          math::Vector3f& rot_accel) {
        for (std::uint8_t i = 0; i < 4; i++) {
            fin[i].last_angle = fin[i].angle;
            if (input.servos[i] == 0) {
                fin[i].angle = 0;
                fin[1].servo_angle = 0;
            } else {
                fin[i].angle = servo_angle(input, i) * math::radians(45.0f) + math::radians(13.5f);
                fin[i].servo_angle = servo_angle(input, i);
            }
            fin[i].dir = !(fin[i].angle < fin[i].last_angle);
            fin[i].vel = math::degrees(fin[i].angle - fin[i].last_angle) / delta_time;
            fin[i].vel = math::constrain_value(fin[i].vel, -450.0f, 450.0f);
            const float vsq = fin[i].vel * fin[i].vel;
            fin[i].T = vsq * k_tan;
            fin[i].N = vsq * k_nor;
            if (!fin[i].dir) {
                fin[i].N = -fin[1].N;
            }
            fin[i].Fx = 0;
            fin[i].Fy = 0;
            fin[i].Fz = 0;
        }
        fin[0].Fx = fin[0].T * std::cos(fin[0].angle);
        fin[0].Fz = fin[0].T * std::sin(fin[0].angle);
        fin[1].Fx = -fin[1].T * std::cos(fin[1].angle);
        fin[1].Fz = fin[1].T * std::sin(fin[1].angle);
        fin[2].Fy = -fin[2].T * std::cos(fin[2].angle);
        fin[2].Fx = fin[2].T * std::sin(fin[2].angle);
        fin[3].Fy = fin[3].T * std::cos(fin[3].angle);
        fin[3].Fx = fin[3].T * std::sin(fin[3].angle);
        math::Vector3f F_BF{0, 0, 0};
        for (std::uint8_t i = 0; i < 4; i++) {
            F_BF.x += fin[i].Fx;
            F_BF.y += fin[i].Fy;
            F_BF.z += fin[i].Fz;
        }
        body_acc.x = F_BF.x / mass;
        body_acc.y = F_BF.y / mass;
        body_acc.z = F_BF.z / mass;
        math::Vector3f rot_T{0, 0, 0};
        rot_T.z = fin[2].Fx * radius - fin[3].Fx * radius;
        rot_accel.x = rot_T.x / moment_of_inertia.x;
        rot_accel.y = rot_T.y / moment_of_inertia.y;
        rot_accel.z = rot_T.z / moment_of_inertia.z;
    }

    void update(const SitlInput& input, float delta_time) {
        math::Vector3f rot_accel{0, 0, 0};
        calculate_forces(input, delta_time, accel_body, rot_accel);
        const float gyr_sq = gyro.length_squared();
        if (math::is_positive(gyr_sq)) {
            math::Vector3f force_gyr = (gyro.normalized() * drag_gyr_constant * gyr_sq);
            math::Vector3f ef_drag_accel_gyr = -force_gyr / mass;
            rot_accel += dcm.transposed() * ef_drag_accel_gyr;
        }
        gyro += rot_accel * delta_time;
        gyro.x = math::constrain_value(gyro.x, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.y = math::constrain_value(gyro.y, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.z = math::constrain_value(gyro.z, -math::radians(2000.0f), math::radians(2000.0f));
        dcm.rotate(gyro * delta_time);
        dcm.normalize();
        const float speed_sq = velocity_ef.length_squared();
        if (math::is_positive(speed_sq)) {
            math::Vector3f force = (velocity_ef.normalized() * drag_constant * speed_sq);
            accel_body += dcm.transposed() * (-force / mass);
        }
        accel_body += dcm.transposed() * math::Vector3f(0, 0, -kGravityMss);
        math::Vector3f accel_earth = dcm * accel_body;
        accel_earth += math::Vector3f(0.0f, 0.0f, kGravityMss);
        velocity_ef += accel_earth * delta_time;
        position += velocity_ef * delta_time;
        update_position();
        time_advance(delta_time);
        update_mag_field_bf();
    }
};

}  // namespace fwcpp::sim
