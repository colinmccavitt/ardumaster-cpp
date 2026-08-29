#pragma once

#include <cstddef>
#include <cstdint>

namespace fwcpp::q_modes {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct QModesPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr QModesPortItem kQModesCompleteness[] = {
    {"Q mode Number enum 17/18/23", PortStatus::kThisSlice, "q_mode_meta.hpp"},
    {"mode metadata flags", PortStatus::kThisSlice, "is_vtol_man_* constexpr"},
    {"QStabilize _enter stub", PortStatus::kThisSlice, "throttle_wait=false"},
    {"QStabilize run phase gate", PortStatus::kThisSlice, "mode_qstabilize.hpp run stub"},
    {"QHover _enter stub", PortStatus::kThisSlice, "climb_rate 0 + throttle_wait init"},
    {"QHover run phase gate", PortStatus::kThisSlice, "mode_qhover.hpp run stub"},
    {"QAcro _enter stub", PortStatus::kThisSlice, "force_transition_complete flags"},
    {"QAcro run phase gate", PortStatus::kThisSlice, "mode_qacro.hpp run stub"},
    {"QAcro tailsitter body-rate swap", PortStatus::kThisSlice, "qacro_body_rates_from_sticks"},
    {"shared FW-transition run branch", PortStatus::kThisSlice, "q_run_common.hpp"},
    {"completeness catalog", PortStatus::kThisSlice, "this table"},
    {"QStabilize update / stick scaling", PortStatus::kRemaining, "mode_qstabilize.cpp update()"},
    {"set_limited_roll_pitch helpers", PortStatus::kRemaining, "PTCH_LIM + roll_limit"},
    {"set_tailsitter_roll_pitch", PortStatus::kRemaining, "transition roll/pitch limit"},
    {"hold_stabilize + pilot throttle", PortStatus::kThisSlice, "QuadPlane hold_stabilize"},
    {"ESC calibration run body", PortStatus::kThisSlice, "run_esc_calibration path"},
    {"QHover update delegate", PortStatus::kRemaining, "mode_qhover.cpp update()"},
    {"pos_control D limits on enter", PortStatus::kRemaining, "AC_PosControl wiring"},
    {"hold_hover + climb rate", PortStatus::kThisSlice, "get_pilot_desired_climb_rate_cms"},
    {"assist recovery + spin output", PortStatus::kThisSlice, "VTOL_Assist in QHOVER run"},
    {"QAcro update att_target euler", PortStatus::kRemaining, "mode_qacro.cpp update()"},
    {"acro_locking rate input variant", PortStatus::kThisSlice, "input_rate_bf_*_3_cds vs _2_cds"},
    {"FW stabilize / rudder output", PortStatus::kThisSlice, "stabilize_roll pitch rudder"},
    {"assign_tilt_to_fwd_thr", PortStatus::kThisSlice, "tilt rotor mix hook"},
    {"Plane / QuadPlane refs", PortStatus::kOutOfScope, "ADR-0012 inject inputs; no singleton"},
};

[[nodiscard]] inline constexpr std::size_t q_modes_completeness_size() {
    return sizeof(kQModesCompleteness) / sizeof(kQModesCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kQModesCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kQModesCompleteness) {
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

}  // namespace fwcpp::q_modes
