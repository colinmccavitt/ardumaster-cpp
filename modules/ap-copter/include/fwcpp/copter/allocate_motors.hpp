#pragma once

// Copter::allocate_motors leftover — motors class selection plus
// ahrs_view / attitude / pos / wp / loiter / circle leftover flags.
// Upstream ArduCopter/system.cpp ~363-488. Stop before
// reload_defaults_file / Y6 PID defaults (~490).
// This port is not heli (FRAME_CONFIG != HELI_FRAME path).
//
// No objects: do not NEW_NOTHROW / heap-allocate motors,
// attitude_control, pos_control, wp_nav, loiter_nav, or circle_nav.
// Record leftover MotorsKind + controller flags only. Do not invent
// AP_BoardConfig or AP_Param.
//
// allocation_failed is true only when motors would be nullptr
// (scripting-off 6DoF / dynamic). scripting is kOutOfScope; inject
// scripting_enabled (default false). Upstream
// AP_BoardConfig::allocation_error does not return — skip leftover
// controllers when allocation_failed.
//
// Inject ahrs_view_ok (default true) and attitude_ok (default true)
// for the create_view / NEW_NOTHROW nullptr checks. Inject
// oapathplanner_enabled (default false) and circle_enabled (default
// true / MODE_CIRCLE_ENABLED).
//
// Do not port Y6/TRI PID set_default, brushed rc_speed,
// convert_pid_parameters, heli_motors_param_conversions, or
// Copter::init_ardupilot.

#include <cstdint>

namespace fwcpp::copter {

// Local leftover enum matching AP_Motors_Class.h ~54-73
// (motor_frame_class). 6DOF_SCRIPTING is SIXDOF_SCRIPTING here
// because a C++ enumerator cannot start with a digit.
enum class MotorFrameClass : std::uint8_t {
    UNDEFINED = 0,
    QUAD = 1,
    HEXA = 2,
    OCTA = 3,
    OCTAQUAD = 4,
    Y6 = 5,
    HELI = 6,
    TRI = 7,
    SINGLE = 8,
    COAX = 9,
    TAILSITTER = 10,
    HELI_DUAL = 11,
    DODECAHEXA = 12,
    HELI_QUAD = 13,
    DECA = 14,
    SCRIPTING_MATRIX = 15,
    SIXDOF_SCRIPTING = 16,
    DYNAMIC_SCRIPTING_MATRIX = 17,
};

enum class MotorsKind : std::uint8_t {
    None = 0,
    Matrix = 1,
    Tri = 2,
    Single = 3,
    Coax = 4,
    Tailsitter = 5,
    Matrix6DoF = 6,
    MatrixDynamic = 7,
};

// Local leftover enum for AC_AttitudeControl class selection.
// Heli is not this port (FRAME_CONFIG != HELI_FRAME).
enum class AttitudeKind : std::uint8_t {
    None = 0,
    Multi = 1,
    Multi6DoF = 2,
};

struct AllocateMotorsInputs {
    MotorFrameClass frame_class{MotorFrameClass::UNDEFINED};
    std::uint16_t loop_rate_hz{400};
    bool scripting_enabled{false};
    bool ahrs_view_ok{true};
    bool attitude_ok{true};
    bool oapathplanner_enabled{false};
    bool circle_enabled{true};
};

struct AllocateMotorsEffects {
    MotorsKind motors_kind{MotorsKind::None};
    std::uint16_t loop_rate_hz{0};
    bool frame_type_tricopter{false};
    bool allocation_failed{false};
    bool load_motors_eeprom{false};
    bool ahrs_view{false};
    bool ahrs_view_failed{false};
    AttitudeKind attitude_kind{AttitudeKind::None};
    bool load_attitude_eeprom{false};
    bool attitude_failed{false};
    bool pos_control{false};
    bool load_pos_eeprom{false};
    bool wp_nav{false};
    bool wp_nav_oa{false};
    bool load_wp_eeprom{false};
    bool loiter_nav{false};
    bool load_loiter_eeprom{false};
    bool circle_nav{false};
    bool load_circle_eeprom{false};
};

[[nodiscard]] inline AllocateMotorsEffects allocate_motors(
    const AllocateMotorsInputs& in = {}) {
    AllocateMotorsEffects fx{};
    fx.loop_rate_hz = in.loop_rate_hz;

    switch (in.frame_class) {
        case MotorFrameClass::QUAD:
        case MotorFrameClass::HEXA:
        case MotorFrameClass::Y6:
        case MotorFrameClass::OCTA:
        case MotorFrameClass::OCTAQUAD:
        case MotorFrameClass::DODECAHEXA:
        case MotorFrameClass::DECA:
        case MotorFrameClass::SCRIPTING_MATRIX:
        case MotorFrameClass::UNDEFINED:
        case MotorFrameClass::HELI:
        case MotorFrameClass::HELI_DUAL:
        case MotorFrameClass::HELI_QUAD:
        default:
            // Non-heli path: HELI / HELI_DUAL / HELI_QUAD and
            // UNDEFINED fall into default → AP_MotorsMatrix.
            fx.motors_kind = MotorsKind::Matrix;
            break;
        case MotorFrameClass::TRI:
            fx.motors_kind = MotorsKind::Tri;
            fx.frame_type_tricopter = true;
            break;
        case MotorFrameClass::SINGLE:
            fx.motors_kind = MotorsKind::Single;
            break;
        case MotorFrameClass::COAX:
            fx.motors_kind = MotorsKind::Coax;
            break;
        case MotorFrameClass::TAILSITTER:
            fx.motors_kind = MotorsKind::Tailsitter;
            break;
        case MotorFrameClass::SIXDOF_SCRIPTING:
            if (in.scripting_enabled) {
                fx.motors_kind = MotorsKind::Matrix6DoF;
            } else {
                fx.motors_kind = MotorsKind::None;
                fx.allocation_failed = true;
            }
            break;
        case MotorFrameClass::DYNAMIC_SCRIPTING_MATRIX:
            if (in.scripting_enabled) {
                fx.motors_kind = MotorsKind::MatrixDynamic;
            } else {
                fx.motors_kind = MotorsKind::None;
                fx.allocation_failed = true;
            }
            break;
    }

    // allocation_error does not return — skip leftover controllers.
    if (fx.allocation_failed) {
        return fx;
    }

    fx.load_motors_eeprom = true;

    fx.ahrs_view = true;
    if (!in.ahrs_view_ok) {
        fx.ahrs_view_failed = true;
        return fx;
    }

    if (in.frame_class == MotorFrameClass::SIXDOF_SCRIPTING && in.scripting_enabled) {
        fx.attitude_kind = AttitudeKind::Multi6DoF;
    } else {
        fx.attitude_kind = AttitudeKind::Multi;
    }
    if (!in.attitude_ok) {
        fx.attitude_failed = true;
        return fx;
    }
    fx.load_attitude_eeprom = true;

    fx.pos_control = true;
    fx.load_pos_eeprom = true;

    if (in.oapathplanner_enabled) {
        fx.wp_nav_oa = true;
    } else {
        fx.wp_nav = true;
    }
    fx.load_wp_eeprom = true;

    fx.loiter_nav = true;
    fx.load_loiter_eeprom = true;

    if (in.circle_enabled) {
        fx.circle_nav = true;
        fx.load_circle_eeprom = true;
    }

    return fx;
}

}  // namespace fwcpp::copter
