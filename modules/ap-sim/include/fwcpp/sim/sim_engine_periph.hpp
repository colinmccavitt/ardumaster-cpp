#pragma once

// Port of SIM_EFI_MegaSquirt, SIM_EFI_Hirth, SIM_ICEngine, SIM_GeneratorEngine,
// SIM_Gripper_EPM, SIM_Buzzer, SIM_ToneAlarm. SFML audio in Buzzer is omitted
// (WITH_SITL_TONEALARM off for Copter/Plane). Buzzer tracks pin_mask on/off.

#include <cmath>
#include <cstdint>
#include <cstring>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class ICEngine {
public:
    std::int8_t throttle_servo = 2;
    std::int8_t ignition_servo = -1;
    std::int8_t choke_servo = -1;
    std::int8_t starter_servo = -1;
    bool throttle_reversed = false;
    float slew_rate = 100;
    float last_output = 0;
    std::uint64_t last_update_us = 0;
    std::uint64_t start_time_us = 0;
    bool overheat = false;
    struct State {
        bool ignition = true;
        bool choke = false;
        bool starter = false;
        std::uint8_t value = 0;
    } state, last_state;

    float update(const SitlInput& input, std::uint64_t now) {
        const bool have_ignition = ignition_servo >= 0;
        const bool have_choke = choke_servo >= 0;
        const bool have_starter = starter_servo >= 0;
        float throttle_demand = (input.servos[throttle_servo] - 1000) * 0.001f;
        if (throttle_reversed) {
            throttle_demand = 1 - throttle_demand;
        }
        state.ignition = have_ignition ? input.servos[ignition_servo] > 1700 : true;
        state.choke = have_choke ? input.servos[choke_servo] > 1700 : false;
        state.starter = have_starter ? input.servos[starter_servo] > 1700 : false;
        const float dt = (now - last_update_us) * 1.0e-6f;
        const float max_change = slew_rate * 0.01f * dt;
        if (!have_starter) {
            last_output = throttle_demand;
            return last_output;
        }
        if (have_ignition && !state.ignition) {
            if (!state.starter) {
                goto engine_off;
            }
            last_update_us = now;
            throttle_demand = 0.1f;
            goto output;
        }
        if (have_choke && state.choke && now - start_time_us > 1000 * 1000UL) {
            goto engine_off;
        }
        if (last_output <= 0 && !state.starter) {
            goto engine_off;
        }
        if (start_time_us == 0 && state.starter) {
            if (throttle_demand <= 0.2f) {
                start_time_us = now;
            }
        }
        if (start_time_us != 0 && state.starter) {
            if ((now - start_time_us) > 3000 * 1000UL && !overheat) {
                overheat = true;
            }
        } else {
            overheat = false;
        }
    output:
        if (start_time_us != 0 && throttle_demand < 0.01f) {
            throttle_demand = 0.01f;
        }
        last_output = math::constrain_value(throttle_demand, last_output - max_change, last_output + max_change);
        last_output = math::constrain_value(last_output, 0.0f, 1.0f);
        last_update_us = now;
        last_state = state;
        return last_output;
    engine_off:
        last_update_us = now;
        start_time_us = 0;
        last_output = 0;
        last_state = state;
        return 0;
    }
};

class SIM_GeneratorEngine {
public:
    float current_current = 0;
    float desired_rpm = 0;
    float current_rpm = 0;
    float max_current = 50;
    float max_slew_rpm_per_second = 2000;
    float temperature = 20;
    std::uint32_t last_rpm_update_ms = 0;
    std::uint32_t last_heat_update_ms = 0;
    void update(std::uint32_t now) {
        if (!math::is_zero(desired_rpm)) {
            desired_rpm -= 1500 * (current_current / max_current);
        }
        const float max_slew_rpm = max_slew_rpm_per_second * ((now - last_rpm_update_ms) / 1000.0f);
        last_rpm_update_ms = now;
        const float rpm_delta = current_rpm - desired_rpm;
        if (rpm_delta > 0) {
            current_rpm -= std::min(max_slew_rpm, rpm_delta);
        } else {
            current_rpm += std::min(max_slew_rpm, std::fabs(rpm_delta));
        }
        const std::uint32_t time_delta_ms = now - last_heat_update_ms;
        last_heat_update_ms = now;
        constexpr float heat_environment_loss_factor = 0.15f;
        temperature += (current_rpm * time_delta_ms * (1 / 1000.0f) * 0.0035f);
        temperature = std::min(temperature, 150.0f);
        temperature -= temperature * heat_environment_loss_factor * (time_delta_ms * (1 / 1000.0f));
    }
};

class Gripper_EPM {
public:
    std::int8_t gripper_emp_enable = 0;
    std::int8_t gripper_emp_servo_pin = -1;
    bool servo_based = true;
    float demand = 0;
    float field_strength = 0;
    float field_decay_rate = 2.0f;
    float field_strength_slew_rate = 400.0f;
    float field_degauss_rate = 200.0f;
    std::uint64_t last_update_us = 0;
    void update_servobased(std::int16_t gripper_pwm) {
        if (!servo_based) {
            return;
        }
        if (gripper_pwm >= 0) {
            demand = (gripper_pwm - 1000) * 0.001f;
            if (math::is_negative(demand)) {
                demand = 0.0f;
            }
        }
    }
    void update_from_demand(std::uint64_t now) {
        const float dt = (now - last_update_us) * 1.0e-6f;
        field_strength = field_strength * (100.0f - field_decay_rate * dt) * 0.01f;
        if (demand > 0.6f) {
            field_strength = field_strength + (100.0f - field_strength) * field_strength_slew_rate * 0.01f * dt;
        } else if (demand < 0.4f) {
            field_strength = field_strength * (100.0f - field_degauss_rate * dt) * 0.01f;
        }
        last_update_us = now;
    }
    void update(const SitlInput& input, std::uint64_t now) {
        const std::int16_t gripper_pwm =
            gripper_emp_servo_pin >= 1 ? static_cast<std::int16_t>(input.servos[gripper_emp_servo_pin - 1]) : -1;
        update_servobased(gripper_pwm);
        update_from_demand(now);
    }
    float tesla() const { return 0.25f * field_strength * 0.01f; }
};

class Buzzer {
public:
    std::int8_t _enable = 0;
    std::int8_t _pin = -1;
    bool was_on = false;
    std::uint32_t on_time = 0;
    void update(std::uint32_t pin_mask, std::uint32_t now) {
        const bool on = _pin >= 1 && (pin_mask & (1u << _pin));
        if (on) {
            if (!was_on) {
                on_time = now;
                was_on = true;
            }
        } else if (was_on) {
            was_on = false;
        }
    }
};

class ToneAlarm {
public:
    std::int8_t _enable = 1;
    void update(const SitlInput& /*input*/) {}
};

class EFI_MegaSquirt : public SerialDevice {
public:
    float tps = 0;
    struct {
        std::uint16_t rpm = 0;
        std::uint16_t fuelload = 20;
        float dwell = 2.0f;
        std::uint16_t baro_hPa = 1000;
        std::uint16_t map_hPa = 895;
        std::uint16_t throttle_pos = 0;
        std::uint16_t afr_target1 = 148;
    } table7;
    std::uint8_t ofs = 0;
    std::uint8_t buf[16]{};
    void update(float rpm) {
        tps = 0.9f * tps + 0.1f * (rpm / 7000) * 100;
        table7.rpm = static_cast<std::uint16_t>(rpm);
        table7.throttle_pos = static_cast<std::uint16_t>(tps * 10);
        while (ofs < 7) {
            char c = 0;
            if (read_from_autopilot(&c, 1) != 1) {
                break;
            }
            buf[ofs] = static_cast<std::uint8_t>(c);
            if (ofs == 0 && buf[0] == 0) {
                ofs++;
            } else if (ofs == 1 && buf[1] == 7) {
                ofs++;
            } else if (ofs == 2 && buf[2] == 0x72) {
                ofs++;
            } else if (ofs > 2) {
                ofs++;
            } else {
                ofs = 0;
            }
        }
        if (ofs >= 7) {
            std::uint8_t pkt[16]{};
            pkt[0] = static_cast<std::uint8_t>(table7.rpm >> 8);
            pkt[1] = static_cast<std::uint8_t>(table7.rpm & 0xff);
            pkt[2] = static_cast<std::uint8_t>(table7.throttle_pos >> 8);
            pkt[3] = static_cast<std::uint8_t>(table7.throttle_pos & 0xff);
            const std::uint32_t crc = crc_crc32(0, pkt, 4);
            pkt[4] = static_cast<std::uint8_t>(crc);
            pkt[5] = static_cast<std::uint8_t>(crc >> 8);
            pkt[6] = static_cast<std::uint8_t>(crc >> 16);
            pkt[7] = static_cast<std::uint8_t>(crc >> 24);
            write_to_autopilot(reinterpret_cast<const char*>(pkt), 8);
            ofs = 0;
        }
    }
};

class EFI_Hirth : public SerialDevice {
public:
    float rpm = 0;
    std::uint8_t throttle = 0;
    void update(float rpm_in, float throttle_norm) {
        rpm = rpm_in;
        throttle = static_cast<std::uint8_t>(math::constrain_value(throttle_norm * 100.0f, 0.0f, 100.0f));
        std::uint8_t pkt[8]{0xA5, 0x5A, static_cast<std::uint8_t>(static_cast<std::uint16_t>(rpm) >> 8),
                            static_cast<std::uint8_t>(static_cast<std::uint16_t>(rpm) & 0xff), throttle, 0, 0, 0};
        pkt[7] = crc_sum_of_bytes(pkt, 7);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 8);
    }
};

}  // namespace fwcpp::sim
