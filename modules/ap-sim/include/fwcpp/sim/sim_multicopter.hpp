#pragma once

// CCP-045: port of libraries/SITL/SIM_Multicopter.h/.cpp plus the
// Aircraft::update_dynamics rigid-body integrator MultiCopter::update()
// actually calls (SIM_Aircraft.cpp). This is the copter SITL plant.
// Plane keeps SimPlane; copter must not use SimPlane::update() /
// leftover body-z thrust as its aero model.
//
// Upstream MultiCopter::update:
//   update_wind, calculate_forces (Frame::calculate_forces + shove/twist/
//   external), clamp, current_and_voltage, update_dynamics,
//   update_external_payload, update_position, time_advance,
//   update_mag_field_bf.
//
// Ported here: Frame/Motor calculate_forces, Aircraft::update_dynamics
// (standard config, GROUND_BEHAVIOR_NO_MOVEMENT as MultiCopter sets),
// time_advance. Sensor lat/lon/mag synthesis stays in SitlCopterHarness
// (CCP-043), matching how SitlHarness owns GPS/compass for Plane.
//
// Disclosed leftovers vs original MultiCopter (same class of optional
// SITL extras this port already stubs on SimPlane):
//   - update_wind / sitl_input.wind (optional; default still air).
//   - add_shove_forces / add_twist_forces / add_external_forces / clamp
//     (SIM param extras, off unless configured).
//   - slung payload / tether.
//   - JSON custom frame models (Frame leftover).
//   - Battery drain (Frame leftover: constant maxVoltage).

#include <cstdint>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_frame.hpp>
#include <fwcpp/sim/sim_motor.hpp>

namespace fwcpp::sim {

class SimMulticopter {
public:
    explicit SimMulticopter(const char* frame_str = "x") {
        dcm.identity();
        frame_ = Frame::create_frame(frame_str);
        if (!frame_.valid()) {
            frame_ = Frame::create_frame("x");
        }
        frame_.init(frame_str);
        mass = frame_.get_mass();
    }

    [[nodiscard]] Frame& frame() { return frame_; }
    [[nodiscard]] const Frame& frame() const { return frame_; }
    [[nodiscard]] std::uint8_t num_motors() const { return frame_.num_motors; }
    [[nodiscard]] float hover_thr_out() const { return frame_.hover_thr_out(); }
    [[nodiscard]] float hover_command() const { return frame_.hover_command(); }
    [[nodiscard]] std::uint16_t command_to_pwm(float command) const { return frame_.command_to_pwm(command); }

    void set_equal_command(SitlInput& input, float command) const { frame_.set_equal_command(input, command); }

    [[nodiscard]] float gross_mass() const { return mass; }

    [[nodiscard]] bool on_ground() const { return position.z >= 0.0f; }

    // Upstream: MultiCopter::calculate_forces.
    void calculate_forces(const SitlInput& input, math::Vector3f& rot_accel, math::Vector3f& body_acc) {
        const float alt_amsl = home_alt_amsl_m - position.z;
        frame_.calculate_forces(dcm, velocity_ef, gyro, alt_amsl, input, rot_accel, body_acc, gross_mass(), true,
                                time_us_);
    }

    // Upstream: MultiCopter::update without wind/clamp/payload/mag/latlon.
    void update(const SitlInput& input, float dt) {
        math::Vector3f rot_accel;
        calculate_forces(input, rot_accel, accel_body);
        frame_.current_and_voltage(battery_voltage, battery_current);
        update_dynamics(rot_accel, dt);
        time_us_ += static_cast<std::uint64_t>(dt * 1.0e6f + 0.5f);
    }

    // Upstream MultiCopter sets GROUND_BEHAVIOR_NO_MOVEMENT.
    void apply_ground_behavior() {
        if (!on_ground()) {
            return;
        }
        float r = 0.0f;
        float p = 0.0f;
        float y = 0.0f;
        dcm.to_euler(&r, &p, &y);
        dcm.from_euler(0.0f, 0.0f, y);
        velocity_ef.x = 0.0f;
        velocity_ef.y = 0.0f;
        if (velocity_ef.z > 0.0f) {
            velocity_ef.z = 0.0f;
        }
        gyro.zero();
    }

    // Upstream: Aircraft::update_dynamics (SIM_Aircraft.cpp) — same
    // integrator SimPlane ports for Plane. Copter uses this copy on the
    // multicopter plant, not SimPlane::update_dynamics via leftover
    // body-z thrust.
    void update_dynamics(const math::Vector3f& rot_accel, float dt) {
        gyro += rot_accel * dt;

        gyro.x = math::constrain_value(gyro.x, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.y = math::constrain_value(gyro.y, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.z = math::constrain_value(gyro.z, -math::radians(2000.0f), math::radians(2000.0f));

        const float accel_limit = 64.0f * kGravityMss;
        accel_body.x = math::constrain_value(accel_body.x, -accel_limit, accel_limit);
        accel_body.y = math::constrain_value(accel_body.y, -accel_limit, accel_limit);
        accel_body.z = math::constrain_value(accel_body.z, -accel_limit, accel_limit);

        dcm.rotate(gyro * dt);
        dcm.normalize();

        math::Vector3f accel_earth = dcm * accel_body;
        accel_earth += math::Vector3f(0.0f, 0.0f, kGravityMss);

        if (on_ground() && accel_earth.z > 0.0f) {
            accel_earth.z = 0.0f;
        }

        accel_body = dcm.transposed() * (accel_earth + math::Vector3f(0.0f, 0.0f, -kGravityMss));

        velocity_ef += accel_earth * dt;
        position += velocity_ef * dt;

        velocity_air_ef = velocity_ef - wind_ef;
        velocity_air_bf = dcm.transposed() * velocity_air_ef;
        airspeed = velocity_air_bf.length();

        if (on_ground() && velocity_ef.z > 0.0f) {
            velocity_ef.z = 0.0f;
        }

        apply_ground_behavior();
    }

    math::Matrix3f dcm{};
    math::Vector3f gyro{};
    math::Vector3f accel_body{};
    math::Vector3f velocity_ef{};
    math::Vector3f velocity_air_ef{};
    math::Vector3f velocity_air_bf{};
    math::Vector3f position{};
    math::Vector3f wind_ef{};
    float airspeed{0.0f};
    float mass{3.0f};
    float battery_voltage{12.6f};
    float battery_current{0.0f};
    float home_alt_amsl_m{0.0f};

    [[nodiscard]] std::uint64_t time_us() const { return time_us_; }

private:
    Frame frame_{};
    std::uint64_t time_us_{0};
};

}  // namespace fwcpp::sim
