#pragma once

// Copter pilot lean / yaw / throttle helpers. Free functions with injected
// stick values — no rc() / AP:: / RC_Channel singletons (ADR-0012).
//
// Upstream (Plane-4.7.0, read directly):
//   ArduCopter/mode.cpp get_pilot_desired_lean_angles_rad ~558-569
//   ArduCopter/mode.cpp get_pilot_desired_yaw_rate_rads ~1065-1076
//   ArduCopter/mode.cpp get_pilot_desired_throttle ~968-992
//   AP_Math/control.cpp input_expo ~760-770
//   AP_Math/control.cpp rc_input_to_roll_pitch_rad ~806-826
//   ArduCopter/Attitude.cpp set_accel_throttle_I_from_pilot_throttle ~120-127
//   ArduCopter/Attitude.cpp get_pilot_desired_climb_rate_ms ~69-112
//   ArduCopter/Attitude.cpp get_pilot_speed_dn_ms ~129-137
//   ArduCopter/mode.cpp get_pilot_desired_velocity ~572-596
//   AP_AHRS/AP_AHRS_Backend.cpp body_to_earth2D ~246-249
//
// Not copied from quadplane_pilot_input.hpp (different vehicle, cd-based
// lean path). Mode::run() bodies stay CCP-039. AutoYaw get_heading
// PILOT_RATE vs HOLD is autoyaw.hpp (on main). weathervane stays leftover
// (see leftover catalog below).

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>

namespace fwcpp::copter {

struct LeanAnglesRad {
    float roll_rad{0.0f};
    float pitch_rad{0.0f};
};

// AP_Math/control.cpp:760. expo < 0.95 shapes; otherwise pass-through.
[[nodiscard]] inline float input_expo(float input, float expo) {
    input = math::constrain_value(input, -1.0f, 1.0f);
    if (expo < 0.95f) {
        return (1.0f - expo) * input / (1.0f - expo * std::fabs(input));
    }
    return input;
}

// AP_Math/control.cpp:806. 85deg cap, angle_limit in [10deg, angle_max],
// tan thrust, Vector2::limit_length, atan back to Euler.
inline void rc_input_to_roll_pitch_rad(float roll_in_norm, float pitch_in_norm, float angle_max_rad,
                                       float angle_limit_rad, float& roll_out_rad,
                                       float& pitch_out_rad) {
    angle_max_rad = std::min(angle_max_rad, math::radians(85.0f));

    math::Vector2f thrust;
    thrust.x = -std::tan(angle_max_rad * pitch_in_norm);
    thrust.y = std::tan(angle_max_rad * roll_in_norm);

    angle_limit_rad = math::constrain_value(angle_limit_rad, math::radians(10.0f), angle_max_rad);
    const float thrust_limit = std::tan(angle_limit_rad);
    thrust.limit_length(thrust_limit);

    pitch_out_rad = -std::atan(thrust.x);
    roll_out_rad = std::atan(std::cos(pitch_out_rad) * thrust.y);
}

// mode.cpp:558. Invalid RC zeros both angles.
[[nodiscard]] inline LeanAnglesRad get_pilot_desired_lean_angles_rad(bool has_valid_input,
                                                                     float roll_norm_dz,
                                                                     float pitch_norm_dz,
                                                                     float angle_max_rad,
                                                                     float angle_limit_rad) {
    LeanAnglesRad out;
    if (!has_valid_input) {
        return out;
    }
    rc_input_to_roll_pitch_rad(roll_norm_dz, pitch_norm_dz, angle_max_rad, angle_limit_rad,
                               out.roll_rad, out.pitch_rad);
    return out;
}

// mode.cpp:1065. Invalid RC zeros yaw rate.
// command_model_pilot_y_rate is the command-model rate in deg/s (upstream
// g2.command_model_pilot_y.get_rate()).
[[nodiscard]] inline float get_pilot_desired_yaw_rate_rads(bool has_valid_input, float yaw_norm_dz,
                                                           float command_model_pilot_y_rate,
                                                           float expo) {
    if (!has_valid_input) {
        return 0.0f;
    }
    return math::radians(command_model_pilot_y_rate) * input_expo(yaw_norm_dz, expo);
}

// mode.cpp:968. mid_stick <= 0 becomes 500; throttle_control 0..1000;
// piecewise mid mapping; expo = constrain(-(thr_mid-0.5)/0.375, -0.5, 1).
[[nodiscard]] inline float get_pilot_desired_throttle(std::int16_t mid_stick,
                                                      std::int16_t throttle_control,
                                                      float throttle_hover) {
    if (mid_stick <= 0) {
        mid_stick = 500;
    }
    throttle_control = math::constrain_value(throttle_control, std::int16_t{0}, std::int16_t{1000});

    float throttle_in;
    if (throttle_control < mid_stick) {
        throttle_in = (static_cast<float>(throttle_control) * 0.5f) / static_cast<float>(mid_stick);
    } else {
        throttle_in = 0.5f + (static_cast<float>(throttle_control - mid_stick) * 0.5f) /
                                 static_cast<float>(1000 - mid_stick);
    }

    const float expo =
        math::constrain_value(-(throttle_hover - 0.5f) / 0.375f, -0.5f, 1.0f);
    return throttle_in * (1.0f - expo) + expo * throttle_in * throttle_in * throttle_in;
}

// Attitude.cpp:120. Returns the integrator value to set on accel PID I.
// Does not call a pos_control object.
[[nodiscard]] inline float set_accel_throttle_I_from_pilot_throttle(float throttle_in,
                                                                    float throttle_hover) {
    const float pilot_throttle = math::constrain_value(throttle_in, 0.0f, 1.0f);
    return -(pilot_throttle - throttle_hover);
}

// Attitude.cpp:129. PILOT_SPD_DN if non-zero, else |PILOT_SPD_UP|.
[[nodiscard]] inline float get_pilot_speed_dn_ms(float speed_dn_ms, float speed_up_ms) {
    if (math::is_zero(speed_dn_ms)) {
        return std::fabs(speed_up_ms);
    }
    return std::fabs(speed_dn_ms);
}

// Attitude.cpp:69. Invalid RC zeros. TOY_MODE skipped (compile-time
// out of scope). throttle_control 0..1000; deadzone 0..400.
// mid_stick is get_throttle_mid() injected. speed_dn_ms is the raw
// PILOT_SPD_DN (resolved via get_pilot_speed_dn_ms).
[[nodiscard]] inline float get_pilot_desired_climb_rate_ms(bool has_valid_input,
                                                           float throttle_control,
                                                           float mid_stick,
                                                           std::int16_t throttle_deadzone,
                                                           float speed_dn_ms,
                                                           float speed_up_ms) {
    if (!has_valid_input) {
        return 0.0f;
    }

    throttle_control = math::constrain_value(throttle_control, 0.0f, 1000.0f);
    throttle_deadzone = math::constrain_value(throttle_deadzone, std::int16_t{0}, std::int16_t{400});

    const float deadband_top = mid_stick + static_cast<float>(throttle_deadzone);
    const float deadband_bottom = mid_stick - static_cast<float>(throttle_deadzone);

    if (throttle_control < deadband_bottom) {
        return get_pilot_speed_dn_ms(speed_dn_ms, speed_up_ms) *
               (throttle_control - deadband_bottom) / deadband_bottom;
    }
    if (throttle_control > deadband_top) {
        return speed_up_ms * (throttle_control - deadband_top) / (1000.0f - deadband_top);
    }
    return 0.0f;
}

// mode.cpp:572. Invalid RC zeros. Stick vector is (-pitch, roll) in
// body NE; is_zero early-returns. body_to_earth2D is the 2D yaw rotate
// (x*cos-y*sin, x*sin+y*cos). Square-to-circle: vel_scalar = vel_ne /
// MAX(|x|,|y|); vel_ne *= vel_max / vel_scalar.length().
[[nodiscard]] inline math::Vector2f get_pilot_desired_velocity(bool has_valid_input,
                                                               float roll_norm,
                                                               float pitch_norm,
                                                               float vel_max,
                                                               float yaw_rad) {
    math::Vector2f vel_ne_ms;
    if (!has_valid_input) {
        return vel_ne_ms;
    }

    vel_ne_ms = math::Vector2f(-pitch_norm, roll_norm);
    if (vel_ne_ms.is_zero()) {
        return vel_ne_ms;
    }

    const float cs = std::cos(yaw_rad);
    const float sn = std::sin(yaw_rad);
    vel_ne_ms = math::Vector2f(vel_ne_ms.x * cs - vel_ne_ms.y * sn,
                               vel_ne_ms.x * sn + vel_ne_ms.y * cs);

    const math::Vector2f vel_scalar =
        vel_ne_ms / std::max(std::fabs(vel_ne_ms.x), std::fabs(vel_ne_ms.y));
    vel_ne_ms *= vel_max / vel_scalar.length();
    return vel_ne_ms;
}

// Nested so leftover remaining_count() does not collide with
// copter_leftover.hpp / mode_leftover.hpp in fwcpp::copter.
namespace pilot {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct PortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr PortItem kCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"get_pilot_desired_lean_angles", PortStatus::kOnMain,
     "mode.cpp ~558-569; zeros if !has_valid_input"},
    {"get_pilot_desired_yaw_rate", PortStatus::kOnMain,
     "mode.cpp ~1065-1076; radians(rate)*input_expo"},
    {"get_pilot_desired_throttle", PortStatus::kOnMain,
     "mode.cpp ~968-992; mid_stick 500; expo cubic"},
    {"rc_input_to_roll_pitch_rad", PortStatus::kOnMain, "AP_Math/control.cpp ~806-826"},
    {"input_expo", PortStatus::kOnMain, "AP_Math/control.cpp ~760-770"},
    {"set_accel_throttle_I", PortStatus::kOnMain,
     "Attitude.cpp ~120-127; returns integrator value"},
    {"get_pilot_desired_climb_rate", PortStatus::kOnMain,
     "Attitude.cpp ~69-112; deadband mid+/-dz; skip TOY_MODE"},
    {"get_pilot_speed_dn", PortStatus::kOnMain,
     "Attitude.cpp ~129-137; zero dn uses |speed_up|"},
    {"AutoYaw state machine", PortStatus::kOnMain,
     "autoyaw.cpp get_heading ~330-347 PILOT_RATE vs HOLD"},
    {"weathervane", PortStatus::kRemaining, "update_weathervane; WEATHERVANE_ENABLED"},
    {"get_pilot_desired_velocity", PortStatus::kThisSlice,
     "mode.cpp ~572-596; body_to_earth2D + square-to-circle"},
};

[[nodiscard]] inline constexpr std::size_t completeness_size() {
    return sizeof(kCompleteness) / sizeof(kCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

}  // namespace pilot

}  // namespace fwcpp::copter
