#pragma once

#include <cstdint>

namespace fwcpp::quadplane {

/// Q_ENABLE default — upstream AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, ...).
inline constexpr std::int8_t kQEnableDefault = 0;

/// Q_FRAME_CLASS default — upstream AP_GROUPINFO("FRAME_CLASS", 46, QuadPlane, frame_class, 1).
inline constexpr std::uint8_t kQFrameClassDefault = 1;

/// Q_FRAME_TYPE default — upstream AP_GROUPINFO("FRAME_TYPE", 31, QuadPlane, frame_type, 1).
inline constexpr std::uint8_t kQFrameTypeDefault = 1;

/// Q_TILT_ENABLE default — upstream Tiltrotor enable parameter default 0.
inline constexpr std::int8_t kQTiltEnableDefault = 0;

/// Q_TAILSIT_ENABLE default — tailsitter enable parameter default 0.
inline constexpr std::int8_t kQTailsitEnableDefault = 0;

/// Conservative setup memory floor (ADR-0012: caller supplies available bytes;
/// full upstream sizeof(*) sum is a later slice).
inline constexpr std::uint32_t kSetupMinMemoryBytes = 4096u;

}  // namespace fwcpp::quadplane
