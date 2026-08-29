#pragma once

// Copter::update_land_and_crash_detectors leftover. Upstream
// ArduCopter/land_detector.cpp ~16-158. No AHRS / motors / flightmode /
// LPF / parachute objects — inject accel_ef, armed, land_complete,
// taking-off, throttle_out, non_takeoff_throttle, spool, standby, count.
//
// Dispatcher: z += GRAVITY_MSS, record filter_apply (land_accel_ef_filter
// apply is leftover; ap-filter LowPassFilter exists but is not wired here),
// always run update_land_detector. HAL_PARACHUTE_ENABLED parachute_check
// is skipped. crash_check / thrust_loss_check / yaw_imbalance_check stay
// remaining on this leftover (flags false).
//
// update_land_detector this slice (multirotor, not heli):
//   !armed -> land_complete
//   land_complete + high throttle + THROTTLE_UNLIMITED + !taking_off
//     -> land_complete false + AP_InternalError::flow_of_control
//   standby_active -> land_detector_count = 0
//   else: optional injected `moving` resets count; the stationary AND-gate
//     (motor_at_lower_limit && mix_min && angle && accel && vel &&
//     rangefinder && WoW) is the next leftover.
// set_land_complete_maybe is remaining: this slice sets maybe=land_complete.

#include <cstdint>

#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::copter {

// AP_Math/definitions.h GRAVITY_MSS — local copy, same 9.80665f as
// ahrs_dcm.hpp / l1_control.hpp / sim_plane.hpp.
inline constexpr float kGravityMss = 9.80665f;

// ArduCopter/config.h LAND_DETECTOR_TRIGGER_SEC. Unused until the
// stationary AND-gate leftover increments land_detector_count.
inline constexpr float kLandDetectorTriggerSec = 1.0f;

// SpoolState is declared in mode_stabilize.hpp (AP_Motors_Class.h
// ~184-190). Reused here so copter.hpp can include both headers.

struct UpdateLandDetectorInputs {
    bool armed{false};
    bool land_complete{false};
    bool is_taking_off{false};
    float throttle_out{0};
    float non_takeoff_throttle{0};
    SpoolState spool{SpoolState::SHUT_DOWN};
    bool standby_active{false};
    std::uint32_t land_detector_count{0};
    bool moving{false};  // AND-gate leftover: reset count if true
};

struct UpdateLandDetectorEffects {
    bool land_complete{false};
    bool land_complete_maybe{false};  // maybe leftover: == land_complete
    bool internal_error_flow_of_control{false};
    std::uint32_t land_detector_count{0};
};

struct UpdateLandAndCrashDetectorsInputs {
    math::Vector3f accel_ef_mss{};
    UpdateLandDetectorInputs land{};
};

struct UpdateLandAndCrashDetectorsEffects {
    math::Vector3f accel_ef_mss{};
    bool filter_apply{false};
    bool update_land_detector{false};
    bool parachute_check{false};
    bool crash_check{false};
    bool thrust_loss_check{false};
    bool yaw_imbalance_check{false};
    UpdateLandDetectorEffects land{};
};

// Copter::set_land_complete leftover (land_detector.cpp ~207-214 change
// detect + count reset only). Disarm-on-land / stats / flying-state
// remain later leftovers.
inline void set_land_complete(bool b, UpdateLandDetectorEffects& fx) {
    if (fx.land_complete == b) {
        return;
    }
    fx.land_detector_count = 0;
    fx.land_complete = b;
}

[[nodiscard]] inline UpdateLandDetectorEffects update_land_detector(
    const UpdateLandDetectorInputs& in) {
    UpdateLandDetectorEffects fx{};
    fx.land_complete = in.land_complete;
    fx.land_detector_count = in.land_detector_count;

    if (!in.armed) {
        set_land_complete(true, fx);
    } else if (in.land_complete) {
        if (!in.is_taking_off && in.throttle_out > in.non_takeoff_throttle &&
            in.spool == SpoolState::THROTTLE_UNLIMITED) {
            fx.internal_error_flow_of_control = true;
            set_land_complete(false, fx);
        }
    } else if (in.standby_active) {
        fx.land_detector_count = 0;
    } else if (in.moving) {
        fx.land_detector_count = 0;
    }

    // set_land_complete_maybe remaining: maybe = land_complete this slice
    // (upstream ORs count >= LAND_DETECTOR_MAYBE_TRIGGER_SEC * loop_rate).
    fx.land_complete_maybe = fx.land_complete;
    return fx;
}

[[nodiscard]] inline UpdateLandAndCrashDetectorsEffects update_land_and_crash_detectors(
    const UpdateLandAndCrashDetectorsInputs& in) {
    UpdateLandAndCrashDetectorsEffects fx{};
    fx.accel_ef_mss = in.accel_ef_mss;
    fx.accel_ef_mss.z += kGravityMss;
    fx.filter_apply = true;
    fx.update_land_detector = true;
    fx.parachute_check = false;
    fx.crash_check = false;
    fx.thrust_loss_check = false;
    fx.yaw_imbalance_check = false;
    fx.land = update_land_detector(in.land);
    return fx;
}

}  // namespace fwcpp::copter
