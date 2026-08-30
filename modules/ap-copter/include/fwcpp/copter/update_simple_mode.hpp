#pragma once

// Copter::update_simple_mode leftover. Upstream Copter.cpp ~857-887.
// Inject simple_mode, new_radio_frame, has_valid_input, roll/pitch
// control_in, simple/super_simple yaw trig, and ahrs yaw trig.
// No RC channel / ahrs objects — control_in is float.
//
// NONE or !new_radio_frame: return without consuming the frame.
// Otherwise consume new_radio_frame, then if !has_valid_input return
// (bind-time RC; roll/pitch unchanged).
// SIMPLE rotates by simple_cos/sin; else SUPERSIMPLE (NONE already
// returned) by super_simple_cos/sin. Then vehicle-frame rotate by
// ahrs cos/sin.
//
// Do not port update_super_simple_bearing (radius / 5deg) — own
// remaining row.

namespace fwcpp::copter {

enum class SimpleMode {
    NONE = 0,
    SIMPLE = 1,
    SUPERSIMPLE = 2,
};

struct UpdateSimpleModeInputs {
    SimpleMode simple_mode{SimpleMode::NONE};
    bool new_radio_frame{false};
    bool has_valid_input{false};
    float roll_control_in{0};
    float pitch_control_in{0};
    float simple_cos_yaw{0};
    float simple_sin_yaw{0};
    float super_simple_cos_yaw{0};
    float super_simple_sin_yaw{0};
    float ahrs_cos_yaw{0};
    float ahrs_sin_yaw{0};
};

struct UpdateSimpleModeEffects {
    bool new_radio_frame{false};
    float roll_control_in{0};
    float pitch_control_in{0};
    bool skipped_none_or_no_frame{false};
    bool skipped_invalid_input{false};
    bool rotated{false};
};

[[nodiscard]] inline UpdateSimpleModeEffects update_simple_mode(
    const UpdateSimpleModeInputs& in = {}) {
    UpdateSimpleModeEffects fx{};
    fx.new_radio_frame = in.new_radio_frame;
    fx.roll_control_in = in.roll_control_in;
    fx.pitch_control_in = in.pitch_control_in;

    if (in.simple_mode == SimpleMode::NONE || !in.new_radio_frame) {
        fx.skipped_none_or_no_frame = true;
        return fx;
    }

    fx.new_radio_frame = false;

    if (!in.has_valid_input) {
        fx.skipped_invalid_input = true;
        return fx;
    }

    float rollx = 0;
    float pitchx = 0;
    if (in.simple_mode == SimpleMode::SIMPLE) {
        rollx = in.roll_control_in * in.simple_cos_yaw -
                in.pitch_control_in * in.simple_sin_yaw;
        pitchx = in.roll_control_in * in.simple_sin_yaw +
                 in.pitch_control_in * in.simple_cos_yaw;
    } else {
        rollx = in.roll_control_in * in.super_simple_cos_yaw -
                in.pitch_control_in * in.super_simple_sin_yaw;
        pitchx = in.roll_control_in * in.super_simple_sin_yaw +
                 in.pitch_control_in * in.super_simple_cos_yaw;
    }

    fx.roll_control_in = rollx * in.ahrs_cos_yaw + pitchx * in.ahrs_sin_yaw;
    fx.pitch_control_in = -rollx * in.ahrs_sin_yaw + pitchx * in.ahrs_cos_yaw;
    fx.rotated = true;
    return fx;
}

}  // namespace fwcpp::copter
