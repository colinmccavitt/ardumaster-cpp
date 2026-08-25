#pragma once

// Port of RC_Channel/RC_Channel.h + RC_Channel.cpp's PWM<->normalized-
// value conversion core: pwm_to_angle*/pwm_to_range*/norm_input*. CPP-030
// slice 1.
//
// SLICE BOUNDARY: this is the pure conversion math - given a channel's
// own radio_in/min/max/trim/dead_zone/reversed state, produce an angle
// (centidegrees, e.g. -4500..4500 for a control surface), a range value
// (0..high_in), or a normalized -1..1 value. Deliberately NOT in this
// slice: RC_Channels (the vehicle-wide array of channels, aux function
// mapping, has_valid_input()/failsafe detection - a real subsystem of
// its own), and norm_input_ignore_trim's bool-returning overload (needs
// rc().has_valid_input(), i.e. that same not-yet-ported subsystem).
//
// AP_Int16 REPLACED WITH PLAIN std::int16_t for radio_min/max/trim -
// same AP_Param-not-wired-in-yet precedent used throughout this port
// (AcPid::Gains, L1Control::Gains) until a caller actually wires AP_Param
// in.
//
// LITERAL SAFETY: no bare ambiguous double literals - every constant is
// an explicit float-suffixed literal, matching upstream's own.

#include <cstdint>

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::rc {

enum class ControlType : std::uint8_t {
    kAngle = 0,
    kRange = 1,
};

class RcChannel {
public:
    std::int16_t radio_in = 0;
    std::int16_t radio_min = 1100;
    std::int16_t radio_max = 1900;
    std::int16_t radio_trim = 1500;
    std::uint16_t dead_zone = 0;
    bool reversed = false;
    std::int16_t high_in = 4500; // e.g. ANGLE_MAX for a control surface channel
    ControlType type_in = ControlType::kAngle;

    // Angle (centidegrees) from radio_in, using an explicit dead_zone and
    // trim rather than this channel's own configured ones - the shared
    // core pwm_to_angle_dz/pwm_to_angle build on.
    [[nodiscard]] float pwm_to_angle_dz_trim(std::uint16_t dz, std::int16_t trim) const {
        const std::int16_t trim_high = static_cast<std::int16_t>(trim + dz);
        const std::int16_t trim_low = static_cast<std::int16_t>(trim - dz);
        const float reverse_mul = reversed ? -1.0f : 1.0f;

        const std::int16_t r_in = math::constrain_value(radio_in, radio_min, radio_max);

        if (r_in > trim_high && radio_max != trim_high) {
            return reverse_mul * (static_cast<float>(high_in) * static_cast<float>(r_in - trim_high)) / static_cast<float>(radio_max - trim_high);
        }
        if (r_in < trim_low && trim_low != radio_min) {
            return reverse_mul * (static_cast<float>(high_in) * static_cast<float>(r_in - trim_low)) / static_cast<float>(trim_low - radio_min);
        }
        return 0.0f;
    }

    [[nodiscard]] float pwm_to_angle_dz(std::uint16_t dz) const { return pwm_to_angle_dz_trim(dz, radio_trim); }
    [[nodiscard]] float pwm_to_angle() const { return pwm_to_angle_dz(dead_zone); }

    // Range value (0..high_in) from radio_in, using an explicit dead_zone.
    [[nodiscard]] float pwm_to_range_dz(std::uint16_t dz) const {
        std::int16_t r_in = math::constrain_value(radio_in, radio_min, radio_max);
        if (reversed) {
            r_in = static_cast<std::int16_t>(radio_max - (r_in - radio_min));
        }
        const std::int16_t trim_low = static_cast<std::int16_t>(radio_min + dz);
        if (r_in > trim_low) {
            return (static_cast<float>(high_in) * static_cast<float>(r_in - trim_low)) / static_cast<float>(radio_max - trim_low);
        }
        return 0.0f;
    }

    [[nodiscard]] float pwm_to_range() const { return pwm_to_range_dz(dead_zone); }

    // Dispatches to pwm_to_range_dz(0) or pwm_to_angle_dz(0) depending on
    // this channel's configured ControlType.
    [[nodiscard]] float get_control_in_zero_dz() const {
        return type_in == ControlType::kRange ? pwm_to_range_dz(0) : pwm_to_angle_dz(0);
    }

    // Normalized (-1..1) input using this channel's own trim as center
    // and dead_zone.
    [[nodiscard]] float norm_input() const {
        const float reverse_mul = reversed ? -1.0f : 1.0f;
        float ret;
        if (radio_in < radio_trim) {
            if (radio_min >= radio_trim) {
                return 0.0f;
            }
            ret = reverse_mul * static_cast<float>(radio_in - radio_trim) / static_cast<float>(radio_trim - radio_min);
        } else {
            if (radio_max <= radio_trim) {
                return 0.0f;
            }
            ret = reverse_mul * static_cast<float>(radio_in - radio_trim) / static_cast<float>(radio_max - radio_trim);
        }
        return math::constrain_value(ret, -1.0f, 1.0f);
    }

    [[nodiscard]] float norm_input_dz() const {
        const std::int16_t dz_min = static_cast<std::int16_t>(radio_trim - dead_zone);
        const std::int16_t dz_max = static_cast<std::int16_t>(radio_trim + dead_zone);
        const float reverse_mul = reversed ? -1.0f : 1.0f;
        float ret;
        if (radio_in < dz_min && dz_min > radio_min) {
            ret = reverse_mul * static_cast<float>(radio_in - dz_min) / static_cast<float>(dz_min - radio_min);
        } else if (radio_in > dz_max && radio_max > dz_max) {
            ret = reverse_mul * static_cast<float>(radio_in - dz_max) / static_cast<float>(radio_max - dz_max);
        } else {
            ret = 0.0f;
        }
        return math::constrain_value(ret, -1.0f, 1.0f);
    }

    // Normalized (-1..1) input using radio_min/radio_max only - trim and
    // dead_zone are not involved at all (matches upstream's own name).
    [[nodiscard]] float norm_input_ignore_trim() const {
        if (radio_max <= radio_min) {
            return 0.0f;
        }
        const float ret = (reversed ? -2.0f : 2.0f)
            * ((static_cast<float>(radio_in - radio_min) / static_cast<float>(radio_max - radio_min)) - 0.5f);
        return math::constrain_value(ret, -1.0f, 1.0f);
    }
};

} // namespace fwcpp::rc
