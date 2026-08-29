#pragma once

// QuadPlane::control_auto — Plane-4.7.0 ArduPlane/quadplane.cpp 3309-3366.
// Header-only tick/effects dispatcher. Does not inline takeoff / waypoint /
// vtol_position bodies (those stay in their own headers / stubs).
//
// should_run_motors is initialized false; the two ifs only assign false
// again. The THROTTLE_UNLIMITED branch is dead — reproduced, not "fixed".

#include <cstdint>

#include <fwcpp/quadplane/quadplane_auto_vtol_mission.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

namespace fwcpp::quadplane {

inline constexpr std::uint32_t kControlAutoLoiterResetMs = 100;

struct ControlAutoInputs {
    bool available{false};
    std::uint16_t nav_cmd_id{0};
    std::int32_t options{0};
    bool spiral_vtol_landing{true};
    bool delay_arming{false};
    DesiredSpoolState desired_spool{DesiredSpoolState::kShutDown};
    bool in_auto_mission_payload_place{false};
    std::uint32_t now_ms{0};
    PosControlSetStateInputs set_state{};
};

struct ControlAutoTick {
    bool early_return{false};
    bool should_run_motors{false};
    bool set_spool_unlimited{false};
    bool delay_arming_checked{false};
    bool payload_place_shutdown_checked{false};
    bool run_takeoff{false};
    bool run_vtol_position_controller{false};
    bool run_waypoint{false};
    bool reset_poscontrol_position1{false};
};

[[nodiscard]] inline ControlAutoTick control_auto(PosControlState& pc,
                                                 PosControlLandStub& land,
                                                 PosControlSetStateSink& sink,
                                                 const ControlAutoInputs& in) {
    ControlAutoTick tick{};
    if (!in.available) {
        tick.early_return = true;
        return tick;
    }

    if (static_cast<std::uint8_t>(pc.state) >
        static_cast<std::uint8_t>(PositionControlState::kApproach)) {
        tick.should_run_motors = false;

        if (in.delay_arming) {
            tick.delay_arming_checked = true;
            tick.should_run_motors = false;
        }

        if (in.desired_spool == DesiredSpoolState::kShutDown && in.in_auto_mission_payload_place &&
            pc.state == PositionControlState::kLandComplete) {
            tick.payload_place_shutdown_checked = true;
            tick.should_run_motors = false;
        }

        if (tick.should_run_motors) {
            tick.set_spool_unlimited = true;
        }
    }

    switch (in.nav_cmd_id) {
        case kMavCmdNavVtolTakeoff:
        case kMavCmdNavTakeoff:
            if (is_vtol_takeoff(in.nav_cmd_id, in.available, in.options)) {
                tick.run_takeoff = true;
            }
            break;
        case kMavCmdNavVtolLand:
        case kMavCmdNavPayloadPlace:
        case kMavCmdNavLand:
            if (is_vtol_land(in.nav_cmd_id, in.available, in.options, in.spiral_vtol_landing)) {
                tick.run_vtol_position_controller = true;
            }
            break;
        case kMavCmdNavLoiterUnlim:
        case kMavCmdNavLoiterTime:
        case kMavCmdNavLoiterTurns:
        case kMavCmdNavLoiterToAlt:
            if (in.now_ms - pc.last_run_ms > kControlAutoLoiterResetMs) {
                PosControlSetStateInputs set_in = in.set_state;
                set_in.now_ms = in.now_ms;
                poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
                tick.reset_poscontrol_position1 = true;
            }
            tick.run_vtol_position_controller = true;
            break;
        default:
            tick.run_waypoint = true;
            break;
    }
    return tick;
}

}  // namespace fwcpp::quadplane
