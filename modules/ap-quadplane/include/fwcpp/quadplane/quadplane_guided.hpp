#pragma once

// QuadPlane guided start / update / user takeoff — Plane-4.7.0
// ArduPlane/quadplane.cpp: guided_start (3946-3959), guided_update
// (3964-3978), do_user_takeoff (4017-4043). guided_mode_enabled
// (3991-4006) is a small helper, not a catalog remaining item.
//
// ADR-0012: header-only ticks/effects. No GCS objects — send_text is
// an enum. setup_target_position and poscontrol_init_approach_prep are
// called, not inlined. takeoff_controller / vtol_position_controller
// are flags. is_flying is injected.

#include <cstdint>
#include <optional>

#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/quadplane/quadplane_options.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_approach.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>
#include <fwcpp/quadplane/quadplane_takeoff_controller.hpp>

namespace fwcpp::quadplane {

enum class UserTakeoffText : std::uint8_t {
    kNone = 0,
    kOnlyInGuided = 1,
    kMustBeArmed = 2,
    kAlreadyFlying = 3,
};

[[nodiscard]] inline constexpr const char* user_takeoff_text(UserTakeoffText text) {
    switch (text) {
        case UserTakeoffText::kOnlyInGuided:
            return "User Takeoff only in GUIDED mode";
        case UserTakeoffText::kMustBeArmed:
            return "Must be armed for takeoff";
        case UserTakeoffText::kAlreadyFlying:
            return "Already flying - no takeoff";
        case UserTakeoffText::kNone:
            return "";
    }
    return "";
}

struct GuidedStartInputs {
    SetupTargetPositionInputs target{};
    PoscontrolApproachInitInputs approach{};
    std::optional<std::int32_t> from_alt_abs_cm{};
    std::optional<std::int32_t> to_alt_abs_cm{};
    std::int32_t current_loc_alt_cm{0};
    std::int32_t next_wp_alt_cm{0};
};

struct GuidedStartTick {
    bool guided_takeoff{false};
    bool setup_target_position{false};
    SetupTargetPositionTick setup{};
    bool poscontrol_init_approach{false};
    PoscontrolApproachInitResult approach{};
    bool slow_descent{false};
};

struct GuidedUpdateInputs {
    bool mode_guided{false};
    bool guided_takeoff{false};
    std::int32_t current_alt_cm{0};
    std::int32_t next_wp_alt_cm{0};
    PosControlSetStateInputs set_state{};
};

struct GuidedUpdateTick {
    bool throttle_wait{false};
    bool clear_throttle_wait{false};
    bool set_desired_spool{false};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool run_takeoff{false};
    bool set_state_position2{false};
    bool guided_takeoff{false};
    bool run_vtol_position_controller{false};
};

struct UserTakeoffInputs {
    bool mode_guided{false};
    bool armed_and_safety_off{false};
    bool is_flying{false};
    std::int32_t options{0};
    GuidedStartInputs start{};
};

struct UserTakeoffTick {
    bool ok{false};
    UserTakeoffText send_text{UserTakeoffText::kNone};
    bool vtol_loiter{false};
    bool prev_wp_from_current{false};
    bool next_wp_from_current{false};
    bool offset_up_m{false};
    float takeoff_altitude_m{0.f};
    bool set_desired_spool{false};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool call_guided_start{false};
    GuidedStartTick start{};
    bool guided_takeoff{false};
    bool guided_wait_takeoff{false};
    bool set_takeoff_expected{false};
};

struct GuidedModeEnabledInputs {
    bool available{false};
    bool mode_guided{false};
    bool mode_auto{false};
    bool auto_nav_loiter_turns{false};
    std::int8_t guided_mode{0};
};

[[nodiscard]] inline bool guided_mode_enabled(const GuidedModeEnabledInputs& in) {
    if (!in.available) {
        return false;
    }
    if (!in.mode_guided && !in.mode_auto) {
        return false;
    }
    if (in.mode_auto && in.auto_nav_loiter_turns) {
        return false;
    }
    return in.guided_mode != 0;
}

inline GuidedStartTick guided_start(PosControlState& pc, PosControlLandStub& land,
                                    const GuidedStartInputs& in) {
    GuidedStartTick tick{};
    tick.guided_takeoff = false;
    tick.setup = setup_target_position(pc, in.target);
    tick.setup_target_position = true;
    tick.approach = poscontrol_init_approach_prep(pc, land, in.approach);
    tick.poscontrol_init_approach = true;
    // approach_prep zeros slow_descent — guided_start wins after prep.
    if (in.from_alt_abs_cm.has_value() && in.to_alt_abs_cm.has_value()) {
        pc.slow_descent = *in.from_alt_abs_cm > *in.to_alt_abs_cm;
    } else {
        pc.slow_descent = in.current_loc_alt_cm > in.next_wp_alt_cm;
    }
    tick.slow_descent = pc.slow_descent;
    return tick;
}

inline GuidedUpdateTick guided_update(PosControlState& pc, PosControlLandStub& land,
                                      PosControlSetStateSink& sink, const GuidedUpdateInputs& in) {
    GuidedUpdateTick tick{};
    if (in.mode_guided && in.guided_takeoff && in.current_alt_cm < in.next_wp_alt_cm) {
        tick.throttle_wait = false;
        tick.clear_throttle_wait = true;
        tick.set_desired_spool = true;
        tick.desired_spool = DesiredSpoolState::kThrottleUnlimited;
        tick.run_takeoff = true;
        tick.guided_takeoff = true;
    } else {
        if (in.guided_takeoff) {
            poscontrol_apply_set_state(pc, PositionControlState::kPosition2, in.set_state, sink, land);
            tick.set_state_position2 = true;
        }
        tick.guided_takeoff = false;
        tick.run_vtol_position_controller = true;
    }
    return tick;
}

inline UserTakeoffTick do_user_takeoff(PosControlState& pc, PosControlLandStub& land,
                                       float takeoff_altitude, const UserTakeoffInputs& in) {
    UserTakeoffTick tick{};
    if (!in.mode_guided) {
        tick.send_text = UserTakeoffText::kOnlyInGuided;
        return tick;
    }
    if (!in.armed_and_safety_off) {
        tick.send_text = UserTakeoffText::kMustBeArmed;
        return tick;
    }
    if (in.is_flying) {
        tick.send_text = UserTakeoffText::kAlreadyFlying;
        return tick;
    }
    tick.vtol_loiter = true;
    tick.prev_wp_from_current = true;
    tick.next_wp_from_current = true;
    tick.offset_up_m = true;
    tick.takeoff_altitude_m = takeoff_altitude;
    tick.set_desired_spool = true;
    tick.desired_spool = DesiredSpoolState::kThrottleUnlimited;
    tick.start = guided_start(pc, land, in.start);
    tick.call_guided_start = true;
    tick.guided_takeoff = true;
    tick.guided_wait_takeoff = false;
    if (!option_is_set(in.options, QOption::kDisableGroundEffectComp)) {
        tick.set_takeoff_expected = true;
    }
    tick.ok = true;
    return tick;
}

}  // namespace fwcpp::quadplane
