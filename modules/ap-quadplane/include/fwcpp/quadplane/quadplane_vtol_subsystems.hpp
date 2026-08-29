#pragma once

// Tailsitter / tiltrotor call-site wiring — upstream QuadPlane::setup()
// thrust_type = SLT; tailsitter.setup(); tiltrotor.setup(); and
// `if (!transition) transition = SLT_Transition` (Plane-4.7.0
// quadplane.cpp ~840-850). Tailsitter.cpp / tiltrotor.cpp bodies are
// VCP-007 / VCP-008; this header only resolves their setup heuristics
// and records the injected thrust_type / transition assignment.

#include <cstdint>

#include <fwcpp/tailsitter/tailsitter_enable.hpp>
#include <fwcpp/tailsitter/tailsitter_setup.hpp>
#include <fwcpp/tailsitter/tailsitter_types.hpp>
#include <fwcpp/tiltrotor/tiltrotor_enable.hpp>
#include <fwcpp/tiltrotor/tiltrotor_setup.hpp>
#include <fwcpp/tiltrotor/tiltrotor_types.hpp>

namespace fwcpp::quadplane {

enum class ThrustType : std::uint8_t {
    kSlt = 0,
    kTailsitter = 1,
    kTiltrotor = 2,
};

enum class TransitionKind : std::uint8_t {
    kSlt = 0,
    kTailsitter = 1,
    kTiltrotor = 2,
};

struct VtolSubsystemWireInputs {
    std::int8_t tailsit_enable{0};
    std::int8_t tilt_enable{0};
    std::uint8_t frame_class{0};
    std::uint16_t motor_mask{0};
    std::uint16_t tilt_mask{0};
    fwcpp::tiltrotor::TiltType tilt_type{fwcpp::tiltrotor::TiltType::kContinuous};
};

struct VtolSubsystemWireResult {
    fwcpp::tailsitter::TailsitterGate tailsitter{};
    fwcpp::tiltrotor::TiltrotorGate tiltrotor{};
    std::int8_t resolved_tailsit_enable{0};
    std::int8_t resolved_tilt_enable{0};
    ThrustType thrust_type{ThrustType::kSlt};
    TransitionKind transition_kind{TransitionKind::kSlt};
    bool ok{false};
};

[[nodiscard]] inline VtolSubsystemWireResult wire_vtol_subsystems(const VtolSubsystemWireInputs& in) {
    fwcpp::tailsitter::TailsitterSetupInputs ts_in{};
    if (in.tailsit_enable != 0) {
        ts_in.enable = in.tailsit_enable;
    }
    ts_in.frame_class = in.frame_class;
    ts_in.motor_mask = in.motor_mask;
    ts_in.tiltrotor_type = (in.tilt_type == fwcpp::tiltrotor::TiltType::kBicopter)
                               ? fwcpp::tailsitter::TiltrotorType::kBicopter
                               : fwcpp::tailsitter::TiltrotorType::kNone;

    fwcpp::tiltrotor::TiltrotorSetupInputs tr_in{};
    if (in.tilt_enable != 0) {
        tr_in.enable = in.tilt_enable;
    }
    tr_in.tilt_mask = in.tilt_mask;
    tr_in.type = in.tilt_type;

    const auto ts = fwcpp::tailsitter::resolve_setup(ts_in);
    const auto tr = fwcpp::tiltrotor::resolve_setup(tr_in);

    VtolSubsystemWireResult out{};
    out.resolved_tailsit_enable = ts.enable;
    out.resolved_tilt_enable = tr.enable;
    if (ts.enable > 0 && tr.enable > 0) {
        return out;
    }

    // Provisionally SLT; tailsitter.setup() / tiltrotor.setup() overwrite
    // thrust_type and assign their Transition. If neither assigned one,
    // setup() allocates SLT_Transition (quadplane.cpp ~848).
    out.thrust_type = ThrustType::kSlt;
    out.transition_kind = TransitionKind::kSlt;
    if (ts.enable > 0) {
        out.thrust_type = ThrustType::kTailsitter;
        out.transition_kind = TransitionKind::kTailsitter;
    } else if (tr.enable > 0) {
        out.thrust_type = ThrustType::kTiltrotor;
        out.transition_kind = TransitionKind::kTiltrotor;
    }

    out.tailsitter = fwcpp::tailsitter::TailsitterGate::from_setup(ts);
    out.tiltrotor = fwcpp::tiltrotor::TiltrotorGate::from_setup(tr);
    out.ok = true;
    return out;
}

}  // namespace fwcpp::quadplane
