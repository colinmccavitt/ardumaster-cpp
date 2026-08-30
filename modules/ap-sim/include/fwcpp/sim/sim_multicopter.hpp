#pragma once

// Port of libraries/SITL/SIM_Multicopter.h/.cpp. Inherits Aircraft so
// update_wind / update_dynamics / update_position / update_mag_field_bf /
// shove / twist / external / clamp / battery drain live in one place.

#include <cstdint>
#include <cstring>

#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_frame.hpp>
#include <fwcpp/sim/sim_motor.hpp>

namespace fwcpp::sim {

class SimMulticopter : public Aircraft {
public:
    explicit SimMulticopter(const char* frame_str = "x") {
        dcm.identity();
        frame_ = Frame::create_frame(frame_str);
        if (!frame_.valid()) {
            frame_ = Frame::create_frame("x");
        }
        frame_.set_sitl(&sitl_params);
        frame_.rpm_out = rpm;
        frame_.init(frame_str);
        mass = frame_.get_mass();
        frame_height = 0.0f;
        ground_behavior = GroundBehavior::kNoMovement;
        battery.setup(frame_.get_model_batt_capacity_ah(), frame_.get_model_batt_resistance_ohm(),
                      frame_.get_model_batt_max_voltage(), 25.0f);
        battery_voltage = battery.get_voltage();
        frame_.set_battery_voltage(battery_voltage);
    }

    [[nodiscard]] Frame& frame() { return frame_; }
    [[nodiscard]] const Frame& frame() const { return frame_; }
    [[nodiscard]] std::uint8_t num_motors() const { return frame_.num_motors; }
    [[nodiscard]] float hover_thr_out() const { return frame_.hover_thr_out(); }
    [[nodiscard]] float hover_command() const { return frame_.hover_command(); }
    // CCP-045 test surface: density altitude used by Frame::calculate_forces.
    float home_alt_amsl_m{0.0f};

    [[nodiscard]] std::uint16_t command_to_pwm(float command) const { return frame_.command_to_pwm(command); }
    void set_equal_command(SitlInput& input, float command) const { frame_.set_equal_command(input, command); }

    void calculate_forces(const SitlInput& input, math::Vector3f& rot_accel, math::Vector3f& body_acc) {
        mass = frame_.get_mass();
        const float alt_amsl = (home_alt_amsl_m != 0.0f) ? home_alt_amsl_m : location.alt * 0.01f;
        frame_.set_battery_voltage(battery_voltage);
        frame_.calculate_forces(dcm, velocity_air_ef, gyro, alt_amsl, input, rot_accel, body_acc, gross_mass(), true,
                                time_now_us);
        const std::uint32_t now_ms = static_cast<std::uint32_t>(time_now_us / 1000U);
        add_shove_forces(rot_accel, body_acc, now_ms);
        add_twist_forces(rot_accel, now_ms);
        add_external_forces(body_acc);
    }

    void update_battery_from_frame() {
        if (frame_.battery_changed()) {
            battery.setup(frame_.get_model_batt_capacity_ah(), frame_.get_model_batt_resistance_ohm(),
                          frame_.get_model_batt_max_voltage(), 25.0f);
        }
        battery.maybe_reset(sitl_params.batt_voltage, sitl_params.batt_capacity_ah, sitl_params.batt_resistance);
        battery_voltage = battery.get_voltage();
        battery_current = frame_.get_current_amp();
        battery_temperature_degC = battery.get_temperature_degC();
        battery.consume_energy(battery_current, time_now_us);
        frame_.set_battery_voltage(battery_voltage);
    }

    // Upstream MultiCopter::update
    void update(const SitlInput& input, float dt) {
        mass = frame_.get_mass();
        update_wind(input);
        math::Vector3f rot_accel;
        calculate_forces(input, rot_accel, accel_body);
        if (clamp_active(input)) {
            rot_accel.zero();
            accel_body.zero();
        }
        update_battery_from_frame();
        update_dynamics(rot_accel, dt);
        time_advance(dt);
        update_position();
        update_mag_field_bf();
    }

private:
    Frame frame_{};
};

}  // namespace fwcpp::sim
