#pragma once

// Frame-class / airframe selection — upstream QuadPlane::setup() switch.
// Rust spec: ap-quadplane/src/lib.rs (MotorFrameClass, classify_frame).

#include <cstdint>
#include <optional>

namespace fwcpp::quadplane {

enum class MotorFrameClass : std::uint8_t {
    kUndefined = 0,
    kQuad = 1,
    kHexa = 2,
    kOcta = 3,
    kOctaQuad = 4,
    kY6 = 5,
    kHeli = 6,
    kTri = 7,
    kSingle = 8,
    kCoax = 9,
    kTailsitter = 10,
    kHeliDual = 11,
    kDodecaHexa = 12,
    kHeliQuad = 13,
    kDeca = 14,
    kScriptingMatrix = 15,
    kSixDofScripting = 16,
    kDynamicScriptingMatrix = 17,
};

[[nodiscard]] inline constexpr std::optional<MotorFrameClass> motor_frame_class_from_u8(std::uint8_t value) {
    switch (value) {
        case 0: return MotorFrameClass::kUndefined;
        case 1: return MotorFrameClass::kQuad;
        case 2: return MotorFrameClass::kHexa;
        case 3: return MotorFrameClass::kOcta;
        case 4: return MotorFrameClass::kOctaQuad;
        case 5: return MotorFrameClass::kY6;
        case 6: return MotorFrameClass::kHeli;
        case 7: return MotorFrameClass::kTri;
        case 8: return MotorFrameClass::kSingle;
        case 9: return MotorFrameClass::kCoax;
        case 10: return MotorFrameClass::kTailsitter;
        case 11: return MotorFrameClass::kHeliDual;
        case 12: return MotorFrameClass::kDodecaHexa;
        case 13: return MotorFrameClass::kHeliQuad;
        case 14: return MotorFrameClass::kDeca;
        case 15: return MotorFrameClass::kScriptingMatrix;
        case 16: return MotorFrameClass::kSixDofScripting;
        case 17: return MotorFrameClass::kDynamicScriptingMatrix;
        default: return std::nullopt;
    }
}

[[nodiscard]] inline constexpr std::uint8_t motor_frame_class_as_u8(MotorFrameClass value) {
    return static_cast<std::uint8_t>(value);
}

enum class MotorsKind {
    kMatrix,
    kTri,
    kTailsitter,
};

enum class VtolAirframe {
    kMulticopter,
    kTailsitter,
    kTiltrotor,
};

struct FrameSetup {
    VtolAirframe airframe{VtolAirframe::kMulticopter};
    MotorsKind motors_kind{MotorsKind::kMatrix};
};

[[nodiscard]] inline constexpr bool frame_class_supported(std::uint8_t frame_class) {
    switch (frame_class) {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 7:
        case 10:
        case 14:
        case 15:
        case 17:
            return true;
        default:
            return false;
    }
}

[[nodiscard]] inline constexpr MotorsKind motors_kind_for(std::uint8_t frame_class) {
    if (frame_class == motor_frame_class_as_u8(MotorFrameClass::kTri)) {
        return MotorsKind::kTri;
    }
    if (frame_class == motor_frame_class_as_u8(MotorFrameClass::kTailsitter)) {
        return MotorsKind::kTailsitter;
    }
    return MotorsKind::kMatrix;
}

[[nodiscard]] inline std::optional<FrameSetup> classify_frame(std::uint8_t frame_class, std::int8_t tailsit_enable,
                                                              std::int8_t tilt_enable) {
    if (!frame_class_supported(frame_class)) {
        return std::nullopt;
    }
    const bool tailsitter =
        frame_class == motor_frame_class_as_u8(MotorFrameClass::kTailsitter) || tailsit_enable > 0;
    if (tailsitter && tilt_enable > 0) {
        return std::nullopt;
    }
    FrameSetup sel{};
    if (tailsitter) {
        sel.airframe = VtolAirframe::kTailsitter;
    } else if (tilt_enable > 0) {
        sel.airframe = VtolAirframe::kTiltrotor;
    } else {
        sel.airframe = VtolAirframe::kMulticopter;
    }
    sel.motors_kind = motors_kind_for(frame_class);
    return sel;
}

}  // namespace fwcpp::quadplane
