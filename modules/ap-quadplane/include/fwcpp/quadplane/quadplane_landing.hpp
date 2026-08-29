#pragma once

// QuadPlane landing descent / abort / reposition — Plane-4.7.0
// ArduPlane/quadplane.cpp: landing_descent_rate_ms (1269-1316),
// update_land_positioning (2179-2205), abort_landing (4835-4853).
//
// ADR-0012: header-only ticks. Persist on PosControlState /
// PosControlLandStub. No Location / AHRS objects — yaw and sticks are
// injected. get_pilot_land_throttle and compute_in_vtol_land_descent are
// called, not inlined. land_detector / guided / xy are not re-ported.

#include <algorithm>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/quadplane/quadplane_land_detector.hpp>
#include <fwcpp/quadplane/quadplane_options.hpp>
#include <fwcpp/quadplane/quadplane_pilot_input.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>
#include <fwcpp/quadplane/quadplane_tecs_mixing.hpp>
#include <fwcpp/quadplane/quadplane_xy_controller.hpp>

namespace fwcpp::quadplane {

inline constexpr float kLandFinalSpeedMsDefault = 0.5f;  // Q_LAND_FINAL_SPD
inline constexpr float kDefaultSpeedDownMs = 1.5f;
inline constexpr float kDefaultSpeedUpMs = 2.5f;
inline constexpr float kThrCtrlLandThresh = 0.7f;
inline constexpr float kLandStickScale = 1.0f / 4500.0f;
inline constexpr float kLandFinalInterpSpanM = 6.0f;
inline constexpr float kThrLandDz = 0.1f;
inline constexpr float kThrLandMid = 0.5f;
inline constexpr std::uint32_t kOverrideDescentWindowMs = 1000;

struct LandingDescentRateInputs {
    std::uint32_t now_ms{0};
    std::int32_t options{0};
    float land_final_alt_m{kLandFinalAltDefaultM};
    float land_final_speed_ms{kLandFinalSpeedMsDefault};
    float default_speed_down_ms{kDefaultSpeedDownMs};
    float default_speed_up_ms{kDefaultSpeedUpMs};
    PilotLandThrottleInputs land_throttle{};
};

struct LandingDescentRateTick {
    float rate_ms{0.f};
    bool used_override{false};
    bool clamped_final_height{false};
    bool thr_ctrl_land{false};
    bool stopped_for_correction{false};
};

struct LandPositioningInputs {
    std::int32_t options{0};
    float roll_control_in{0.f};
    float pitch_control_in{0.f};
    float wp_accel_mss{kWpAccelMssDefault};
    float loop_period_s{0.f};
    float yaw_rad{0.f};
};

struct LandPositioningTick {
    bool enabled{false};
    bool pilot_correction_active{false};
    bool pilot_correction_done{false};
    float target_vel_north_ms{0.f};
    float target_vel_east_ms{0.f};
    float target_vel_down_ms{0.f};
    float correction_north_m{0.f};
    float correction_east_m{0.f};
};

struct AbortLandingInputs {
    bool mode_auto{false};
    bool in_auto_payload_place{false};
    InVtolLandDescentInputs land_descent{};
    PosControlSetStateInputs set_state{};
};

struct AbortLandingTick {
    bool aborted{false};
    bool payload_place_landed{false};
};

[[nodiscard]] inline LandingDescentRateTick landing_descent_rate_ms(PosControlState& pc,
                                                                   PosControlLandStub& land,
                                                                   float height_above_ground_m,
                                                                   const LandingDescentRateInputs& in) {
    LandingDescentRateTick tick{};
    if (pc.last_override_descent_ms != 0) {
        if (in.now_ms - pc.last_override_descent_ms < kOverrideDescentWindowMs) {
            tick.used_override = true;
            tick.rate_ms = pc.override_descent_rate_ms;
            tick.thr_ctrl_land = land.thr_ctrl_land;
            return tick;
        }
    }

    if (pc.state == PositionControlState::kLandFinal) {
        tick.clamped_final_height = true;
        height_above_ground_m = std::min(height_above_ground_m, in.land_final_alt_m);
    }

    float ret_ms = fwcpp::math::linear_interpolate(in.land_final_speed_ms, in.default_speed_down_ms,
                                                   height_above_ground_m, in.land_final_alt_m,
                                                   in.land_final_alt_m + kLandFinalInterpSpanM);

    if (option_is_set(in.options, QOption::kThrLandingControl)) {
        const float thr_in = get_pilot_land_throttle(in.land_throttle);
        if (thr_in > kThrCtrlLandThresh) {
            land.thr_ctrl_land = true;
        }
        if (land.thr_ctrl_land) {
            const float thresh1 = kThrLandMid + kThrLandDz;
            const float thresh2 = kThrLandMid - kThrLandDz;
            const float scaling = 1.0f / (kThrLandMid - kThrLandDz);
            if (thr_in > thresh1) {
                ret_ms = -(thr_in - thresh1) * scaling * in.default_speed_up_ms;
            } else if (thr_in > thresh2) {
                ret_ms = 0.f;
            } else {
                ret_ms *= (thresh2 - thr_in) * scaling;
            }
        }
    }

    if (pc.pilot_correction_active) {
        tick.stopped_for_correction = true;
        ret_ms = std::min(0.f, ret_ms);
    }

    tick.rate_ms = ret_ms;
    tick.thr_ctrl_land = land.thr_ctrl_land;
    return tick;
}

inline LandPositioningTick update_land_positioning(PosControlState& pc, const LandPositioningInputs& in) {
    LandPositioningTick tick{};
    if (!option_is_set(in.options, QOption::kRepositionLanding)) {
        pc.pilot_correction_active = false;
        pc.target_vel_north_ms = 0.f;
        pc.target_vel_east_ms = 0.f;
        pc.target_vel_down_ms = 0.f;
        tick.pilot_correction_active = false;
        tick.pilot_correction_done = pc.pilot_correction_done;
        tick.correction_north_m = pc.correction_north_m;
        tick.correction_east_m = pc.correction_east_m;
        return tick;
    }

    tick.enabled = true;
    const float roll_in = in.roll_control_in * kLandStickScale;
    const float pitch_in = in.pitch_control_in * kLandStickScale;
    const float speed_max_ms = in.wp_accel_mss * 0.5f;

    fwcpp::math::Vector3f target_vel_ms(-pitch_in, roll_in, 0.f);
    target_vel_ms *= speed_max_ms;
    target_vel_ms.rotate_xy(in.yaw_rad);

    pc.target_vel_north_ms = target_vel_ms.x;
    pc.target_vel_east_ms = target_vel_ms.y;
    pc.target_vel_down_ms = target_vel_ms.z;

    const auto dxy = target_vel_ms.xy() * in.loop_period_s;
    pc.correction_north_m += dxy.x;
    pc.correction_east_m += dxy.y;

    pc.pilot_correction_active = !fwcpp::math::is_zero(roll_in) || !fwcpp::math::is_zero(pitch_in);
    if (pc.pilot_correction_active) {
        pc.pilot_correction_done = true;
    }

    tick.pilot_correction_active = pc.pilot_correction_active;
    tick.pilot_correction_done = pc.pilot_correction_done;
    tick.target_vel_north_ms = pc.target_vel_north_ms;
    tick.target_vel_east_ms = pc.target_vel_east_ms;
    tick.target_vel_down_ms = pc.target_vel_down_ms;
    tick.correction_north_m = pc.correction_north_m;
    tick.correction_east_m = pc.correction_east_m;
    return tick;
}

inline AbortLandingTick abort_landing(PosControlState& pc, PosControlLandStub& land,
                                      PosControlSetStateSink& sink, const AbortLandingInputs& in) {
    AbortLandingTick tick{};
    if (pc.state == PositionControlState::kLandAbort || !in.mode_auto) {
        return tick;
    }

    tick.payload_place_landed =
        in.in_auto_payload_place && pc.state == PositionControlState::kLandComplete;

    InVtolLandDescentInputs descent = in.land_descent;
    descent.pos_state = pc.state;
    if (!tick.payload_place_landed && !compute_in_vtol_land_descent(descent)) {
        return tick;
    }

    poscontrol_apply_set_state(pc, PositionControlState::kLandAbort, in.set_state, sink, land);
    tick.aborted = true;
    return tick;
}

}  // namespace fwcpp::quadplane
