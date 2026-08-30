#pragma once

// Copter::allocate_motors leftover — motors class selection only.
// Upstream ArduCopter/system.cpp ~363-430 (the frame_class switch
// that constructs motors). Stop before ahrs.create_view (~436).
// This port is not heli (FRAME_CONFIG != HELI_FRAME path).
//
// No objects: do not NEW_NOTHROW / heap-allocate motors,
// attitude_control, pos_control, wp_nav, loiter_nav, or circle_nav.
// Record leftover MotorsKind only. Do not record attitude/pos/wp
// leftovers this slice. Do not invent AP_BoardConfig.
//
// allocation_failed is true only when motors would be nullptr
// (scripting-off 6DoF / dynamic). scripting is kOutOfScope; inject
// scripting_enabled (default false).
//
// Do not port ahrs.create_view, AP_Param::load_object_from_eeprom,
// Y6/TRI PID defaults, convert_pid_parameters, or
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

struct AllocateMotorsInputs {
    MotorFrameClass frame_class{MotorFrameClass::UNDEFINED};
    std::uint16_t loop_rate_hz{400};
    bool scripting_enabled{false};
};

struct AllocateMotorsEffects {
    MotorsKind motors_kind{MotorsKind::None};
    std::uint16_t loop_rate_hz{0};
    bool frame_type_tricopter{false};
    bool allocation_failed{false};
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
    return fx;
}

}  // namespace fwcpp::copter
