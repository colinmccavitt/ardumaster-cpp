#pragma once

#include <cstdint>

namespace fwcpp::quadplane {

/// Default Q_OPTIONS, upstream AP_GROUPINFO(OPTIONS, ..., 0).
inline constexpr std::int32_t kQOptionsDefault = 0;

/// Upstream QuadPlane::Option — Q_OPTIONS bit flags (Plane-4.7.0 quadplane.h).
enum class QOption : std::int32_t {
    kLevelTransition = 1 << 0,
    kAllowFwTakeoff = 1 << 1,
    kAllowFwLand = 1 << 2,
    kRespectTakeoffFrame = 1 << 3,
    kMissionLandFwApproach = 1 << 4,
    kFsQrtl = 1 << 5,
    kIdleGovManual = 1 << 6,
    kQAssistForceEnable = 1 << 7,
    kTailsitQAssistMotorsOnly = 1 << 8,
    kAirmodeUnused = 1 << 9,
    kDisarmedTilt = 1 << 10,
    kDelayArming = 1 << 11,
    kDisableSyntheticAirspeedAssist = 1 << 12,
    kDisableGroundEffectComp = 1 << 13,
    kIgnoreFwAngleLimitsInQModes = 1 << 14,
    kThrLandingControl = 1 << 15,
    kDisableApproach = 1 << 16,
    kRepositionLanding = 1 << 17,
    kOnlyArmInQmodeOrAuto = 1 << 18,
    kTransFailToFw = 1 << 19,
    kFsRtl = 1 << 20,
    kDisarmedTiltUp = 1 << 21,
    kScaleFfAngleP = 1 << 22,
};

[[nodiscard]] inline constexpr bool option_is_set(std::int32_t options, QOption option) {
    return (options & static_cast<std::int32_t>(option)) != 0;
}

}  // namespace fwcpp::quadplane
