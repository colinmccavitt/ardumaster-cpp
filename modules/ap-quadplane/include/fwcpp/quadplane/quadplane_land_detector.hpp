#pragma once

// QuadPlane land detector — Plane-4.7.0 ArduPlane/quadplane.cpp:
// should_relax (1213-1231), land_detector (3532-3561),
// check_land_complete (3567-3593), check_land_final (3600-3617).
//
// ADR-0012: motors / attitude / inertial_nav / mission / GCS are injected.
// Mutates PosControlLandStub timers and last_land_final_agl_m.

#include <cmath>
#include <cstdint>

#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

namespace fwcpp::quadplane {

inline constexpr float kLandFinalAltDefaultM = 6.0f;  // Q_LAND_FINAL_ALT
inline constexpr float kLandFinalMaxChangeM = 5.0f;
inline constexpr float kLandThrottleRest = 0.01f;
inline constexpr std::uint32_t kShouldRelaxLowerLimitMs = 1000;
inline constexpr std::uint32_t kCheckLandCompleteTimeoutMs = 4000;
inline constexpr std::uint32_t kCheckLandFinalTimeoutMs = 6000;

struct ShouldRelaxInputs {
    std::uint32_t now_ms{0};
    bool throttle_lower{false};
    bool throttle_mix_min{false};
    float throttle{0.f};
};

struct LandDetectorInputs {
    std::uint32_t now_ms{0};
    bool throttle_lower{false};
    bool throttle_mix_min{false};
    float throttle{0.f};
    float height_m{0.f};
    bool pilot_correction_active{false};
};

struct CheckLandCompleteInputs {
    LandDetectorInputs detector{};
    bool in_payload_place{false};
    bool mode_auto{false};
    bool continue_after_land{false};
};

struct CheckLandCompleteResult {
    bool complete{false};
    bool landed_text{false};
    bool spool_shut_down{false};
    bool disarm{false};
    PosControlSetStateSink set_state_sink{};
};

struct CheckLandFinalInputs {
    LandDetectorInputs detector{};
    float height_above_ground_m{0.f};
    float land_final_alt_m{kLandFinalAltDefaultM};
};

[[nodiscard]] inline ShouldRelaxInputs relax_from(const LandDetectorInputs& in) {
    return ShouldRelaxInputs{
        .now_ms = in.now_ms,
        .throttle_lower = in.throttle_lower,
        .throttle_mix_min = in.throttle_mix_min,
        .throttle = in.throttle,
    };
}

/// motors->limit.throttle_lower && attitude_control->is_throttle_mix_min(),
/// or throttle < 0.01. Not at limit zeros both timers. Else latch
/// lower_limit_start_ms and return (now - latch) > 1000.
[[nodiscard]] inline bool should_relax(PosControlLandStub& land, const ShouldRelaxInputs& in) {
    bool motor_at_lower_limit = in.throttle_lower && in.throttle_mix_min;
    if (in.throttle < kLandThrottleRest) {
        motor_at_lower_limit = true;
    }

    if (!motor_at_lower_limit) {
        land.lower_limit_start_ms = 0;
        land.land_start_ms = 0;
        return false;
    }
    if (land.lower_limit_start_ms == 0) {
        land.lower_limit_start_ms = in.now_ms;
    }
    return (in.now_ms - land.lower_limit_start_ms) > kShouldRelaxLowerLimitMs;
}

/// might_be_landed = should_relax() && !pilot_correction_active.
/// Height change > detect_alt_change_m (Q_LAND_ALTCHG 0.2) aborts.
/// Landed only when both timeout_ms (from land_start) and timeout_ms+1000
/// (from lower_limit_start) have elapsed.
[[nodiscard]] inline bool land_detector(std::uint32_t timeout_ms, PosControlLandStub& land,
                                         const LandDetectorInputs& in) {
    const bool might_be_landed = should_relax(land, relax_from(in)) && !in.pilot_correction_active;
    if (!might_be_landed) {
        land.land_start_ms = 0;
        return false;
    }
    if (land.land_start_ms == 0) {
        land.land_start_ms = in.now_ms;
        land.vpos_start_m = in.height_m;
    }
    if (std::fabs(in.height_m - land.vpos_start_m) > land.detect_alt_change_m) {
        land.land_start_ms = 0;
        return false;
    }
    if ((in.now_ms - land.land_start_ms) < timeout_ms ||
        (in.now_ms - land.lower_limit_start_ms) < (timeout_ms + kShouldRelaxLowerLimitMs)) {
        return false;
    }
    return true;
}

/// Only in QPOS_LAND_FINAL. Detector 4000ms → set_state(QPOS_LAND_COMPLETE).
/// PAYLOAD_PLACE: spool SHUT_DOWN, return false (do not disarm).
/// Else disarm unless mode_auto && continue_after_land.
[[nodiscard]] inline CheckLandCompleteResult check_land_complete(PosControlState& pc,
                                                                 PosControlLandStub& land,
                                                                 const CheckLandCompleteInputs& in) {
    CheckLandCompleteResult out{};
    if (pc.state != PositionControlState::kLandFinal) {
        return out;
    }
    if (!land_detector(kCheckLandCompleteTimeoutMs, land, in.detector)) {
        return out;
    }
    const PosControlSetStateInputs st{.now_ms = in.detector.now_ms};
    poscontrol_apply_set_state(pc, PositionControlState::kLandComplete, st, out.set_state_sink, land);
    out.landed_text = true;
    if (in.in_payload_place) {
        out.spool_shut_down = true;
        out.complete = false;
        return out;
    }
    if (!in.mode_auto || !in.continue_after_land) {
        out.disarm = true;
    }
    out.complete = true;
    return out;
}

/// Two AGL readings within 5m and below land_final_alt_m → final.
/// Else latch last_land_final_agl_m and fall through to land_detector(6000).
[[nodiscard]] inline bool check_land_final(PosControlLandStub& land, const CheckLandFinalInputs& in) {
    if (in.height_above_ground_m < in.land_final_alt_m &&
        std::fabs(in.height_above_ground_m - land.last_land_final_agl_m) < kLandFinalMaxChangeM) {
        return true;
    }
    land.last_land_final_agl_m = in.height_above_ground_m;
    return land_detector(kCheckLandFinalTimeoutMs, land, in.detector);
}

}  // namespace fwcpp::quadplane
