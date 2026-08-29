#pragma once

#include <cstddef>
#include <cstdint>

namespace fwcpp::quadplane_transition {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct QuadplaneTransitionPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr QuadplaneTransitionPortItem kQuadplaneTransitionCompleteness[] = {
    {"SLT_Transition::State enum", PortStatus::kThisSlice,
     "transition_state.hpp discriminants 0/1/2"},
    {"can_transition gate", PortStatus::kThisSlice,
     "transition_fsm.hpp explicit state moves"},
    {"set_state / restart / force complete", PortStatus::kThisSlice,
     "mark_transition_done + force effects"},
    {"completeness catalog", PortStatus::kThisSlice, "this table"},
    {"update_airspeed_wait", PortStatus::kThisSlice,
     "AIRSPEED_WAIT timing + assist gate"},
    {"update_timer / Q_TRANSITION_MS dwell", PortStatus::kThisSlice,
     "TIMER completion vs tilt_fwd_complete"},
    {"update_forward_timing / apply_assist_back", PortStatus::kThisSlice,
     "assist-back then stage switch order"},
    {"apply_transition_fail / Q_TRANS_FAIL", PortStatus::kThisSlice,
     "fail timer + Q* fallback tokens"},
    {"Q_TRANSITION_MS constrain + decel helpers", PortStatus::kThisSlice,
     "transition_timing.hpp parity constants"},
    {"set_last_fw_pitch / assist reset on force", PortStatus::kThisSlice,
     "force_transition_complete side effects"},
    {"tiltrotor_fwd early complete path", PortStatus::kThisSlice,
     "try_complete_tiltrotor_fwd in update_forward_timing"},
    {"SLT_Transition::VTOL_update state prep", PortStatus::kThisSlice,
     "vtol_update throttle_wait / airspeed wait"},
    {"active_frwd / show_vtol_view", PortStatus::kThisSlice,
     "transition_paths.hpp injected inputs"},
    {"get_mav_vtol_state / TransitionPhase", PortStatus::kThisSlice,
     "transition_state.hpp mav_vtol_state_slt"},
    {"Transition base-class virtuals", PortStatus::kRemaining,
     "roll/pitch limits, yaw, throttle mix"},
    {"SLT_Transition::update loop", PortStatus::kRemaining,
     "motor output + TECS synthetic airspeed wiring"},
    {"QuadPlane& / motors refs", PortStatus::kOutOfScope,
     "ADR-0012 inject inputs; no Plane singleton"},
};

[[nodiscard]] inline constexpr std::size_t quadplane_transition_completeness_size() {
    return sizeof(kQuadplaneTransitionCompleteness) /
           sizeof(kQuadplaneTransitionCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kQuadplaneTransitionCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kQuadplaneTransitionCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

}  // namespace fwcpp::quadplane_transition
