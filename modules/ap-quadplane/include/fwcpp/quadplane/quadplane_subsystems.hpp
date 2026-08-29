#pragma once

// Held tailsitter / tiltrotor wiring state for QuadPlane::setup() and
// QuadPlane::update() tiltrotor.update() (Plane-4.7.0 quadplane.cpp ~840
// and ~1799). Does not reimplement tailsitter.cpp / tiltrotor.cpp.

#include <cstdint>

#include <fwcpp/quadplane/quadplane_vtol_subsystems.hpp>
#include <fwcpp/tailsitter/tailsitter_enable.hpp>
#include <fwcpp/tiltrotor/tiltrotor_enable.hpp>
#include <fwcpp/tiltrotor/tiltrotor_types.hpp>

namespace fwcpp::quadplane {

struct VtolSubsystemsState {
    ThrustType thrust_type{ThrustType::kSlt};
    TransitionKind transition_kind{TransitionKind::kSlt};
    fwcpp::tailsitter::TailsitterGate tailsitter{};
    fwcpp::tiltrotor::TiltrotorGate tiltrotor{};
    bool wired{false};
    std::uint32_t tiltrotor_update_ticks{0};
};

struct VtolSubsystemsSetupInputs {
    std::uint8_t frame_class{0};
    std::int8_t tailsit_enable{0};
    std::int8_t tilt_enable{0};
    std::uint16_t motor_mask{0};
    std::uint16_t tilt_mask{0};
    fwcpp::tiltrotor::TiltType tilt_type{fwcpp::tiltrotor::TiltType::kContinuous};
};

struct VtolSubsystemsUpdateTick {
    bool ran_tiltrotor_update{false};
};

inline void apply_vtol_subsystem_wire(VtolSubsystemsState& state, const VtolSubsystemWireResult& wire) {
    if (!wire.ok) {
        return;
    }
    state.thrust_type = wire.thrust_type;
    state.transition_kind = wire.transition_kind;
    state.tailsitter = wire.tailsitter;
    state.tiltrotor = wire.tiltrotor;
    state.wired = true;
}

inline void wire_vtol_subsystems_setup(VtolSubsystemsState& state, const VtolSubsystemsSetupInputs& in) {
    const VtolSubsystemWireInputs wire_in{
        .tailsit_enable = in.tailsit_enable,
        .tilt_enable = in.tilt_enable,
        .frame_class = in.frame_class,
        .motor_mask = in.motor_mask,
        .tilt_mask = in.tilt_mask,
        .tilt_type = in.tilt_type,
    };
    apply_vtol_subsystem_wire(state, wire_vtol_subsystems(wire_in));
}

[[nodiscard]] inline VtolSubsystemsUpdateTick wire_vtol_subsystems_update(VtolSubsystemsState& state) {
    VtolSubsystemsUpdateTick tick{};
    if (!state.wired) {
        return tick;
    }
    if (state.tiltrotor.enabled()) {
        tick.ran_tiltrotor_update = true;
        ++state.tiltrotor_update_ticks;
    }
    return tick;
}

}  // namespace fwcpp::quadplane
