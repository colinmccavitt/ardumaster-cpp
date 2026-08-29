#pragma once

#include <fwcpp/quadplane/quadplane_mode_predicates.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>

#include <cstdint>

namespace fwcpp::quadplane {

struct VtolPositionControllerInputs {
    std::uint32_t now_ms{0};
    bool armed_and_safety_off{false};
    InVtolModeInputs in_vtol{};
};

struct VtolPositionControllerTick {
    bool ran{false};
    bool none_to_position1_failsafe{false};
    bool approach_nvtol_failsafe{false};
};

/// Stub for upstream `QuadPlane::vtol_position_controller()` landing/RTL path.
inline VtolPositionControllerTick run_vtol_position_controller(PosControlState& pc,
                                                               PosControlLandStub& land,
                                                               PosControlSetStateSink& sink,
                                                               const VtolPositionControllerInputs& in) {
    VtolPositionControllerTick tick{};
    if (!in.in_vtol.available) {
        return tick;
    }
    tick.ran = true;
    if (in.armed_and_safety_off) {
        pc.last_run_ms = in.now_ms;
    }

    const PosControlSetStateInputs set_in{.now_ms = in.now_ms};

    switch (pc.state) {
        case PositionControlState::kNone:
            poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
            tick.none_to_position1_failsafe = true;
            break;
        case PositionControlState::kApproach:
            if (compute_in_vtol_mode(in.in_vtol)) {
                poscontrol_apply_set_state(pc, PositionControlState::kPosition1, set_in, sink, land);
                tick.approach_nvtol_failsafe = true;
            }
            break;
        default:
            break;
    }
    return tick;
}

}  // namespace fwcpp::quadplane
