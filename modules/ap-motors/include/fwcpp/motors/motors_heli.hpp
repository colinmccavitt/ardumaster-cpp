#pragma once

// Port of libraries/AP_Motors/AP_MotorsHeli_Swash.cpp calculate_roll_pitch_collective_factors
// and calculate()/rc_write so SIM_Helicopter can be fed real CCPM mixing PWM
// rather than raw cyclic. RSC/Dual/Quad mixing helpers included for Single.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::motors {

enum class SwashPlateType : std::uint8_t {
    H3 = 0,
    H1 = 1,
    H3_140 = 2,
    H3_120 = 3,
    H4_90 = 4,
    H4_45 = 5,
};

enum class CollectiveDirection : std::uint8_t { NORMAL = 0, REVERSED = 1 };

class MotorsHeliSwash {
public:
    static constexpr std::uint8_t kMaxServos = 4;
    SwashPlateType _swash_type = SwashPlateType::H3_120;
    CollectiveDirection _collective_direction = CollectiveDirection::NORMAL;
    bool _make_servo_linear = false;
    float _servo1_pos = -60;
    float _servo2_pos = 60;
    float _servo3_pos = 180;
    float _phase_angle = 0;
    bool _enabled[kMaxServos]{};
    float _rollFactor[kMaxServos]{};
    float _pitchFactor[kMaxServos]{};
    float _collectiveFactor[kMaxServos]{};
    float _output[kMaxServos]{};
    float _roll_input = 0;
    float _pitch_input = 0;
    float _collective_input_scaled = 0;

    void add_servo_raw(std::uint8_t num, float roll, float pitch, float collective) {
        if (num >= kMaxServos) {
            return;
        }
        _enabled[num] = true;
        _rollFactor[num] = roll * 0.45f;
        _pitchFactor[num] = pitch * 0.45f;
        _collectiveFactor[num] = collective;
    }
    void add_servo_angle(std::uint8_t num, float angle, float collective) {
        add_servo_raw(num, std::cos(math::radians(angle + 90)), std::cos(math::radians(angle)), collective);
    }
    void calculate_roll_pitch_collective_factors() {
        for (std::uint8_t i = 0; i < kMaxServos; i++) {
            _enabled[i] = false;
            _rollFactor[i] = _pitchFactor[i] = _collectiveFactor[i] = 0;
        }
        switch (_swash_type) {
        case SwashPlateType::H3:
            add_servo_angle(0, _servo1_pos - _phase_angle, 1.0f);
            add_servo_angle(1, _servo2_pos - _phase_angle, 1.0f);
            add_servo_angle(2, _servo3_pos - _phase_angle, 1.0f);
            break;
        case SwashPlateType::H1:
            add_servo_raw(0, 1.0f, 0.0f, 0.0f);
            add_servo_raw(1, 0.0f, 1.0f, 0.0f);
            add_servo_raw(2, 0.0f, 0.0f, 1.0f);
            break;
        case SwashPlateType::H3_140:
            add_servo_raw(0, 1.0f, 1.0f, 1.0f);
            add_servo_raw(1, -1.0f, 1.0f, 1.0f);
            add_servo_raw(2, 0.0f, -1.0f, 1.0f);
            break;
        case SwashPlateType::H3_120:
            add_servo_angle(0, -60.0f, 1.0f);
            add_servo_angle(1, 60.0f, 1.0f);
            add_servo_angle(2, 180.0f, 1.0f);
            break;
        case SwashPlateType::H4_90:
            add_servo_angle(0, -90.0f, 1.0f);
            add_servo_angle(1, 90.0f, 1.0f);
            add_servo_angle(2, 180.0f, 1.0f);
            add_servo_angle(3, 0.0f, 1.0f);
            break;
        case SwashPlateType::H4_45:
            add_servo_angle(0, -45.0f, 1.0f);
            add_servo_angle(1, 45.0f, 1.0f);
            add_servo_angle(2, -135.0f, 1.0f);
            add_servo_angle(3, 135.0f, 1.0f);
            break;
        }
    }
    void configure() { calculate_roll_pitch_collective_factors(); }
    static float get_linear_servo_output(float input) {
        input = math::constrain_value(input, -1.0f, 1.0f);
        return math::safe_asin(0.766044f * input) * 1.145916f;
    }
    void calculate(float roll, float pitch, float collective) {
        _roll_input = roll;
        _pitch_input = pitch;
        _collective_input_scaled = collective;
        if (_collective_direction == CollectiveDirection::REVERSED) {
            collective = 1 - collective;
        }
        for (std::uint8_t i = 0; i < kMaxServos; i++) {
            if (!_enabled[i]) {
                continue;
            }
            _output[i] = (_rollFactor[i] * roll) + (_pitchFactor[i] * pitch) + _collectiveFactor[i] * collective;
            if (_swash_type == SwashPlateType::H1 && (i == 0 || i == 1)) {
                _output[i] += 0.5f;
            }
            _output[i] = 2.0f * _output[i] - 1.0f;
            if (_make_servo_linear) {
                _output[i] = get_linear_servo_output(_output[i]);
            }
        }
    }
    static std::uint16_t rc_write_pwm(float swash_in) {
        return static_cast<std::uint16_t>(1500 + 500 * swash_in);
    }
    void write_servos(std::uint16_t* servos, float rsc_pwm = 1600) const {
        for (std::uint8_t i = 0; i < kMaxServos; i++) {
            if (_enabled[i]) {
                servos[i] = rc_write_pwm(_output[i]);
            }
        }
        servos[7] = static_cast<std::uint16_t>(rsc_pwm);
    }
};

class MotorsHeliRSC {
public:
    float desired_rotor_speed = 0;
    float rotor_ramp = 0;
    float ramp_time = 1.0f;
    float runup_time = 10.0f;
    float rotor_runup = 0;
    void set_desired_rotor_speed(float s) { desired_rotor_speed = s; }
    void update_rotor_ramp(float dt) {
        const float step = (ramp_time > 0) ? dt / ramp_time : 1.0f;
        if (rotor_ramp < desired_rotor_speed) {
            rotor_ramp = std::min(desired_rotor_speed, rotor_ramp + step);
        } else {
            rotor_ramp = std::max(desired_rotor_speed, rotor_ramp - step);
        }
        const float run_step = (runup_time > 0) ? dt / runup_time : 1.0f;
        if (rotor_runup < rotor_ramp) {
            rotor_runup = std::min(rotor_ramp, rotor_runup + run_step);
        } else {
            rotor_runup = std::max(rotor_ramp, rotor_runup - run_step);
        }
    }
    float get_rotor_speed() const { return rotor_runup; }
    std::uint16_t output_pwm() const { return static_cast<std::uint16_t>(1000 + 1000 * rotor_ramp); }
};

}  // namespace fwcpp::motors
