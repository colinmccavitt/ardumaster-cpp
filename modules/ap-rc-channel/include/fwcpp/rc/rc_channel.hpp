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
//
// CPP-027 slice: added ch_in/control_in (RC_Channel.h:521,529) and
// update() (RC_Channel.cpp:~303) so fwcpp::rc::RcChannels (rc_channels.hpp,
// same module) can drive a channel from an fwcpp::hal::RcInput. update()
// here deliberately takes the already-read PWM value as a parameter
// rather than pulling from hal.rcin itself: this keeps RcChannel free of
// any HAL dependency (it stays pure per-channel math, as it was before
// this slice) and matches this port's explicit-context convention - the
// registry (which does depend on ap-hal) owns "where does the PWM value
// come from" (RcInput::read(ch_in), or in upstream's case also GCS
// overrides - see rc_channels.hpp's banner for why overrides aren't
// reproduced here). Functionally this is upstream's update() with the
// has_override()/has_had_rc_receiver() gate hoisted to the caller: by
// the time a caller has a PWM value in hand to pass in, upstream would
// already have decided to update, so update() always returns true here.

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

    // Index into the RC input source (fwcpp::hal::RcInput::read(ch_in)),
    // matching RC_Channel.h:521's ch_in. RcChannels::RcChannels() below
    // defaults every channel's ch_in to its own array index (matching
    // RC_Channels::init()'s `channel(i)->ch_in = i` loop) - overridable
    // afterwards for a non-1:1 mapping, same as upstream allows.
    std::uint8_t ch_in = 0;

    // Cached scaled value from the last update(), matching RC_Channel.h:
    // 529's control_in. int16_t (not float) is upstream's own choice,
    // even though pwm_to_range()/pwm_to_angle() are float-returning -
    // reproduced here with an explicit narrowing cast rather than
    // upstream's implicit one (get_control_in_zero_dz() above stays
    // float-returning since it's a fresh per-call computation, not this
    // cache).
    std::int16_t control_in = 0;

    // Pulls a freshly-read PWM value in as radio_in and recomputes
    // control_in via pwm_to_range() or pwm_to_angle() depending on
    // type_in - matches RC_Channel::update()'s dispatch (RC_Channel.cpp:
    // ~303) minus the override/has_had_rc_receiver gating (see file
    // banner - that decision is the caller's, i.e. RcChannels::
    // read_input() in rc_channels.hpp). Always returns true: reaching
    // this call already means the caller decided a real update happens.
    bool update(std::uint16_t new_radio_in) {
        radio_in = static_cast<std::int16_t>(new_radio_in);
        control_in = static_cast<std::int16_t>(
            type_in == ControlType::kRange ? pwm_to_range() : pwm_to_angle());
        return true;
    }

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
