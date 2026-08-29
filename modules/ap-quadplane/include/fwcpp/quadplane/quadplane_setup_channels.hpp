#pragma once

// Setup-time motor channel defaults and AHRS view creation — upstream
// QuadPlane::setup_default_channels / ahrs.create_view (Plane-4.7.0 quadplane.cpp).
// ADR-0012: caller-owned SRV wiring is injected via SetupChannelsSink; AHRS
// view parameters are explicit structs, not singleton AP::ahrs().

#include <array>
#include <cstdint>

#include <fwcpp/quadplane/quadplane_frame.hpp>

namespace fwcpp::quadplane {

inline constexpr std::uint8_t kSetupMotorChannelBase = 5;

struct MotorChannelDefault {
    std::uint8_t motor_function_index{0};
    std::uint8_t channel{0};
};

inline constexpr std::size_t kMaxRecordedMotorChannelDefaults = 16;

struct SetupChannelsSink {
    std::array<MotorChannelDefault, kMaxRecordedMotorChannelDefaults> motor_defaults{};
    std::size_t motor_default_count{0};
    bool tri_frame_param_flags{false};
};

enum class AhrsViewRotation : std::uint8_t {
    kNone = 0,
    kPitch90 = 1,
};

struct AhrsViewCreateInputs {
    std::int8_t tailsit_enable{0};
    float trim_pitch_rad{0.f};
};

struct AhrsViewSetup {
    AhrsViewRotation rotation{AhrsViewRotation::kNone};
    float trim_pitch_rad{0.f};
    bool created{false};
};

[[nodiscard]] inline AhrsViewSetup make_ahrs_view_setup(const AhrsViewCreateInputs& inputs) {
    AhrsViewSetup out{};
    out.rotation =
        inputs.tailsit_enable > 0 ? AhrsViewRotation::kPitch90 : AhrsViewRotation::kNone;
    out.trim_pitch_rad = inputs.trim_pitch_rad;
    out.created = true;
    return out;
}

inline void record_motor_default(SetupChannelsSink& sink, std::uint8_t motor_fn, std::uint8_t channel) {
    if (sink.motor_default_count >= kMaxRecordedMotorChannelDefaults) {
        return;
    }
    sink.motor_defaults[sink.motor_default_count++] = MotorChannelDefault{motor_fn, channel};
}

inline void setup_default_channels(std::uint8_t num_motors, SetupChannelsSink& sink) {
    for (std::uint8_t i = 0; i < num_motors; ++i) {
        record_motor_default(sink, i, static_cast<std::uint8_t>(kSetupMotorChannelBase + i));
    }
}

inline void wire_setup_channels(std::uint8_t frame_class, SetupChannelsSink& sink) {
    sink = SetupChannelsSink{};
    const auto mfc = motor_frame_class_from_u8(frame_class);
    if (!mfc) {
        return;
    }
    switch (*mfc) {
        case MotorFrameClass::kQuad:
            setup_default_channels(4, sink);
            break;
        case MotorFrameClass::kHexa:
            setup_default_channels(6, sink);
            break;
        case MotorFrameClass::kOcta:
        case MotorFrameClass::kOctaQuad:
            setup_default_channels(8, sink);
            break;
        case MotorFrameClass::kY6:
            setup_default_channels(7, sink);
            break;
        case MotorFrameClass::kDeca:
            setup_default_channels(10, sink);
            break;
        case MotorFrameClass::kTri:
            record_motor_default(sink, 0, 5);
            record_motor_default(sink, 1, 6);
            record_motor_default(sink, 3, 8);
            record_motor_default(sink, 6, 11);
            sink.tri_frame_param_flags = true;
            break;
        case MotorFrameClass::kTailsitter:
        case MotorFrameClass::kScriptingMatrix:
        case MotorFrameClass::kDynamicScriptingMatrix:
            break;
        default:
            break;
    }
}

}  // namespace fwcpp::quadplane
