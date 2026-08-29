#pragma once

// QuadPlane::verify_vtol_land — Plane-4.7.0 ArduPlane/quadplane.cpp 3623-3705.
// Header-only ticks/effects (ADR-0012). Reuses check_land_final /
// check_land_complete. No GCS / landing-gear / ICE objects — flags.
// AP_LANDINGGEAR_ENABLED and AP_ICENGINE_ENABLED are injected bools.

#include <cstdint>

#include <fwcpp/math/vector2.hpp>
#include <fwcpp/quadplane/quadplane_land_detector.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

namespace fwcpp::quadplane {

inline constexpr float kVerifyLandDescendDistM = 2.0f;
inline constexpr float kVerifyLandDescendSpeedMs = 3.0f;
inline constexpr std::uint32_t kVerifyLandVelocityMatchFreshMs = 1000;

enum class VerifyVtolLandText : std::uint8_t {
    kNone = 0,
    kLandDescendStarted,
    kLandFinalStarted,
    kPayloadPlaceAborted,
    kMissionContinue,
};

struct VerifyVtolLandInputs {
    bool available{false};
    std::uint32_t now_ms{0};
    float pos_n_m{0.f};
    float pos_e_m{0.f};
    float vel_north_ms{0.f};
    float vel_east_ms{0.f};
    float current_alt_m{0.f};
    float height_agl_m{0.f};
    bool mode_auto{false};
    bool payload_place{false};
    std::uint16_t cmd_p1{0};
    bool continue_after_land{false};
    bool landing_gear_enabled{false};
    bool icengine_enabled{false};
    bool land_icengine_cut{false};
    float land_final_alt_m{kLandFinalAltDefaultM};
    LandDetectorInputs detector{};
    PosControlSetStateInputs set_state{};
};

struct VerifyVtolLandTick {
    bool done{false};
    VerifyVtolLandText send_text{VerifyVtolLandText::kNone};
    bool set_lean_angle_max{false};
    std::int32_t lean_angle_max_cd{0};
    bool deploy_landing_gear{false};
    bool ice_cut{false};
    bool set_next_wp_from_mission{false};
    bool set_next_wp_from_current{false};
    bool copy_alt_from_home{false};
    bool complete{false};
    bool landed_text{false};
    bool spool_shut_down{false};
    bool disarm{false};
    PosControlSetStateSink set_state_sink{};
};

[[nodiscard]] inline VerifyVtolLandTick verify_vtol_land(PosControlState& pc, PosControlLandStub& land,
                                                        const VerifyVtolLandInputs& in) {
    VerifyVtolLandTick tick{};
    if (!in.available) {
        tick.done = true;
        return tick;
    }

    PosControlSetStateInputs st = in.set_state;
    if (st.now_ms == 0) {
        st.now_ms = in.now_ms;
    }
    st.current_alt_m = in.current_alt_m;

    if (pc.state == PositionControlState::kPosition2) {
        bool reached_position = false;
        if (pc.pilot_correction_done) {
            reached_position = !pc.pilot_correction_active;
        } else {
            const fwcpp::math::Vector2<float> delta{in.pos_n_m - pc.target_ned_n_m,
                                                    in.pos_e_m - pc.target_ned_e_m};
            reached_position = delta.length() < kVerifyLandDescendDistM;
        }
        fwcpp::math::Vector2<float> approach_vel{};
        if (in.now_ms - pc.last_velocity_match_ms < kVerifyLandVelocityMatchFreshMs) {
            approach_vel = {pc.velocity_match_north_ms, pc.velocity_match_east_ms};
        }
        const fwcpp::math::Vector2<float> vel_xy{in.vel_north_ms, in.vel_east_ms};
        if (reached_position && (vel_xy - approach_vel).length() < kVerifyLandDescendSpeedMs) {
            poscontrol_apply_set_state(pc, PositionControlState::kLandDescend, st, tick.set_state_sink, land);
            pc.pilot_correction_done = false;
            tick.set_lean_angle_max = true;
            tick.lean_angle_max_cd = 0;
            pc.correction_north_m = 0.f;
            pc.correction_east_m = 0.f;
            if (in.landing_gear_enabled) {
                tick.deploy_landing_gear = true;
            }
            land.last_land_final_agl_m = in.height_agl_m;
            tick.send_text = VerifyVtolLandText::kLandDescendStarted;
            if (in.mode_auto) {
                tick.set_next_wp_from_mission = true;
            } else {
                tick.set_next_wp_from_current = true;
                tick.copy_alt_from_home = true;
            }
        }
    }

    if (pc.state == PositionControlState::kLandDescend &&
        check_land_final(land, CheckLandFinalInputs{
                                   .detector = in.detector,
                                   .height_above_ground_m = in.height_agl_m,
                                   .land_final_alt_m = in.land_final_alt_m,
                               })) {
        poscontrol_apply_set_state(pc, PositionControlState::kLandFinal, st, tick.set_state_sink, land);
        if (in.icengine_enabled && in.land_icengine_cut) {
            tick.ice_cut = true;
        }
        tick.send_text = VerifyVtolLandText::kLandFinalStarted;
    }

    if (pc.state == PositionControlState::kLandAbort &&
        in.current_alt_m >= land.land_descend_start_alt_m) {
        tick.done = true;
        return tick;
    }

    if (in.payload_place &&
        (pc.state == PositionControlState::kLandDescend || pc.state == PositionControlState::kLandFinal) &&
        in.cmd_p1 > 0 &&
        in.current_alt_m < land.land_descend_start_alt_m - static_cast<float>(in.cmd_p1) * 0.01f) {
        tick.send_text = VerifyVtolLandText::kPayloadPlaceAborted;
        poscontrol_apply_set_state(pc, PositionControlState::kLandAbort, st, tick.set_state_sink, land);
    }

    const auto complete = check_land_complete(
        pc, land,
        CheckLandCompleteInputs{
            .detector = in.detector,
            .in_payload_place = in.payload_place,
            .mode_auto = in.mode_auto,
            .continue_after_land = in.continue_after_land,
        });
    tick.complete = complete.complete;
    tick.landed_text = complete.landed_text;
    tick.spool_shut_down = complete.spool_shut_down;
    tick.disarm = complete.disarm;
    if (complete.landed_text || complete.complete || complete.spool_shut_down || complete.disarm) {
        tick.set_state_sink = complete.set_state_sink;
    }
    if (complete.complete && in.continue_after_land) {
        tick.send_text = VerifyVtolLandText::kMissionContinue;
        tick.done = true;
    }
    return tick;
}

}  // namespace fwcpp::quadplane
