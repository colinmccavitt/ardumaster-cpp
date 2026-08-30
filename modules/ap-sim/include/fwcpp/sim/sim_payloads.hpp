#pragma once

// Port of libraries/SITL/SIM_Gripper_Servo, SIM_Sprayer, SIM_Parachute.
// AP_Param dropped; pin/enable are plain fields. dt from caller.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class Gripper_Servo {
public:
    static constexpr std::int16_t SIM_GRIPPER_GRAB_PWM_DEFAULT = 2000;
    static constexpr std::int16_t SIM_GRIPPER_RELEASE_PWM_DEFAULT = 1000;
    std::int8_t gripper_enable = 0;
    std::int8_t gripper_servo_pin = -1;
    std::int16_t grab_pwm = SIM_GRIPPER_GRAB_PWM_DEFAULT;
    std::int16_t release_pwm = SIM_GRIPPER_RELEASE_PWM_DEFAULT;
    std::int8_t reverse = 0;
    bool jaw_open = false;
    const float gap = 30;
    float altitude = 0;
    float position = 0;
    float position_slew_rate = 35;
    const float string_length = 2.0f;
    float load_mass = 0.0f;

    void set_alt(float alt) { altitude = alt; }
    [[nodiscard]] bool is_enabled() const { return static_cast<bool>(gripper_enable); }
    [[nodiscard]] bool is_jaw_open() const { return jaw_open; }

    void update(const SitlInput& input, float dt) {
        const std::int16_t gripper_pwm = gripper_servo_pin >= 1 ? static_cast<std::int16_t>(input.servos[gripper_servo_pin - 1]) : -1;
        if (gripper_pwm < 0) {
            return;
        }
        const std::int16_t diff_pwm = static_cast<std::int16_t>(std::abs(grab_pwm - release_pwm));
        float position_demand = (gripper_pwm - diff_pwm) * 0.001f;
        if (gripper_pwm < std::min(grab_pwm, release_pwm) || position_demand > 1.0f) {
            position_demand = position;
        }
        const float position_max_change = position_slew_rate * 0.01f * dt;
        position = math::constrain_value(position_demand, position - position_max_change, position + position_max_change);
        float jaw_gap;
        if ((release_pwm < grab_pwm && reverse) || (release_pwm > grab_pwm && !reverse)) {
            jaw_gap = gap * position;
        } else {
            jaw_gap = gap * (1.0f - position);
        }
        if (jaw_gap < 5) {
            if (altitude <= 0.0f) {
                load_mass = 1.0f;
                jaw_open = false;
            }
        } else if (jaw_gap > 10) {
            load_mass = 0.0f;
            jaw_open = true;
        }
    }

    [[nodiscard]] float payload_mass() const {
        if (altitude < string_length) {
            return 0.0f;
        }
        return load_mass;
    }
};

class Sprayer {
public:
    std::int8_t sprayer_enable = 0;
    std::int8_t sprayer_pump_pin = -1;
    std::int8_t sprayer_spin_pin = -1;
    const float pump_max_rate = 0.01f;
    const float pump_slew_rate = 20.0f;
    float last_pump_output = 0;
    const float spinner_max_rate = 3600.0f;
    const float spinner_slew_rate = 20.0f;
    float last_spinner_output = 0;
    double capacity = 0.25;
    [[nodiscard]] bool is_enabled() const { return static_cast<bool>(sprayer_enable); }
    [[nodiscard]] float payload_mass() const { return static_cast<float>(capacity); }

    void update(const SitlInput& input, float dt) {
        const std::int16_t pump_pwm = sprayer_pump_pin >= 1 ? static_cast<std::int16_t>(input.servos[sprayer_pump_pin - 1]) : -1;
        const std::int16_t spinner_pwm = sprayer_spin_pin >= 1 ? static_cast<std::int16_t>(input.servos[sprayer_spin_pin - 1]) : -1;
        if (pump_pwm >= 0) {
            if (capacity > 0) {
                const double delta = last_pump_output * pump_max_rate * dt;
                capacity -= delta;
                if (capacity < 0) {
                    capacity = 0.0f;
                }
            }
            float pump_demand = (pump_pwm - 1000) * 0.001f;
            if (pump_demand < 0) {
                pump_demand = 0;
            }
            const float pump_max_change = pump_slew_rate * 0.01f * dt;
            last_pump_output = math::constrain_value(pump_demand, last_pump_output - pump_max_change, last_pump_output + pump_max_change);
            last_pump_output = math::constrain_value(last_pump_output, 0.0f, 1.0f);
        } else {
            last_pump_output = 0.0f;
        }
        if (spinner_pwm >= 0) {
            const float spinner_demand = (spinner_pwm - 1000) * 0.001f;
            const float spinner_max_change = spinner_slew_rate * 0.01f * dt;
            last_spinner_output =
                math::constrain_value(spinner_demand, last_spinner_output - spinner_max_change, last_spinner_output + spinner_max_change);
            last_spinner_output = math::constrain_value(last_spinner_output, 0.0f, 1.0f);
        }
    }
};

class Parachute {
public:
    std::int8_t parachute_enable = 0;
    std::int8_t parachute_pin = -1;
    std::uint32_t deployed_ms = 0;
    [[nodiscard]] bool is_enabled() const { return static_cast<bool>(parachute_enable); }

    void update(const SitlInput& input, std::uint32_t now_ms) {
        const std::int16_t pwm = parachute_pin >= 1 ? static_cast<std::int16_t>(input.servos[parachute_pin - 1]) : -1;
        if (pwm >= 1250) {
            if (!deployed_ms) {
                deployed_ms = now_ms;
            }
        }
    }
};

}  // namespace fwcpp::sim
