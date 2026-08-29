#pragma once

#include <fwcpp/quadplane/quadplane_options.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

#include <cstdint>

namespace fwcpp::quadplane {

inline constexpr float kApproachDistanceDefaultM = 0.f;

/// Upstream geometry for `poscontrol_init_approach` (transition threshold is vcp-003 prep input).
struct ApproachInitView {
    float dist_m{0.f};
    float transition_threshold_m{50.f};
    bool tailsitter_enabled{false};
    bool spool_unlimited{false};
};

/// QPOS transition prep from `poscontrol_init_approach` (not SLT `transition->update`).
struct PosControlTransitionPrep {
    bool set_last_fw_pitch{false};
};

struct PoscontrolApproachInitInputs {
    std::int32_t options{kQOptionsDefault};
    float approach_distance_m{kApproachDistanceDefaultM};
    ApproachInitView view{};
    PosControlSetStateInputs set_state{};
};

struct PoscontrolApproachInitResult {
    PositionControlState chosen{PositionControlState::kNone};
    PosControlTransitionPrep transition_prep{};
    PosControlSetStateSink set_state_sink{};
};

[[nodiscard]] inline bool is_positive(float v) { return v > 0.f; }

inline PoscontrolApproachInitResult poscontrol_init_approach_prep(PosControlState& pc,
                                                                    PosControlLandStub& land,
                                                                    const PoscontrolApproachInitInputs& in) {
    PoscontrolApproachInitResult out{};
    PosControlSetStateSink sink{};

    const bool disable = option_is_set(in.options, QOption::kDisableApproach);
    const bool short_approach =
        is_positive(in.approach_distance_m) && in.view.dist_m < in.approach_distance_m;

    if (disable || short_approach) {
        poscontrol_apply_set_state(pc, PositionControlState::kPosition1, in.set_state, sink, land);
        out.chosen = PositionControlState::kPosition1;
    } else if (pc.state != PositionControlState::kApproach) {
        if (in.view.dist_m < in.view.transition_threshold_m) {
            if (in.view.tailsitter_enabled || in.view.spool_unlimited) {
                poscontrol_apply_set_state(pc, PositionControlState::kPosition1, in.set_state, sink, land);
                out.chosen = PositionControlState::kPosition1;
                out.transition_prep.set_last_fw_pitch = true;
            } else {
                poscontrol_apply_set_state(pc, PositionControlState::kAirbrake, in.set_state, sink, land);
                out.chosen = PositionControlState::kAirbrake;
            }
        } else {
            poscontrol_apply_set_state(pc, PositionControlState::kApproach, in.set_state, sink, land);
            out.chosen = PositionControlState::kApproach;
        }
        pc.thrust_loss_start_ms = 0;
    } else {
        out.chosen = PositionControlState::kApproach;
    }

    pc.pilot_correction_done = false;
    pc.correction_north_m = 0.f;
    pc.correction_east_m = 0.f;
    pc.slow_descent = false;

    out.set_state_sink = sink;
    return out;
}

}  // namespace fwcpp::quadplane
