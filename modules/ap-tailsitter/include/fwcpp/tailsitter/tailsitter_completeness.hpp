#pragma once

#include <cstddef>
#include <cstdint>

namespace fwcpp::tailsitter {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct TailsitterPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr TailsitterPortItem kTailsitterCompleteness[] = {
    {"enable/check gate", PortStatus::kThisSlice, "tailsitter_enable.hpp enabled()"},
    {"setup enable heuristic", PortStatus::kThisSlice, "tailsitter_setup.hpp resolve_setup"},
    {"bicopter exclusion gate", PortStatus::kThisSlice, "setup_heuristic_applies tiltrotor"},
    {"input bitmask enums", PortStatus::kThisSlice, "tailsitter_defaults.hpp PlaneMode/BF_ROLL"},
    {"gscl_mask enums", PortStatus::kThisSlice, "tailsitter_defaults.hpp GSCMSK bits"},
    {"is_vectored predicate", PortStatus::kThisSlice, "tailsitter_input_type.hpp"},
    {"is_control_surface_tailsitter", PortStatus::kThisSlice, "tailsitter_input_type.hpp left tilt rule"},
    {"input_type resolver", PortStatus::kThisSlice, "resolve_input_type"},
    {"defaults constants", PortStatus::kThisSlice, "tailsitter_defaults.hpp"},
    {"completeness catalog", PortStatus::kThisSlice, "this table"},
    {"setup SRV surface flags", PortStatus::kThisSlice, "tailsitter_setup_flags.hpp"},
    {"enable==2 assist/airmode/arm", PortStatus::kThisSlice, "tailsitter_setup_flags.hpp resolve_enable2"},
    {"transition_rate_fw auto-set", PortStatus::kThisSlice, "tailsitter_setup_flags.hpp"},
    {"Tailsitter::active", PortStatus::kThisSlice, "tailsitter.cpp active()"},
    {"output skip / disarm min", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output VTOL transition throttle", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output FW motor_mask", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output vectored forward tilt", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output Q assist hover tilt", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output Q assist motors-only I-relax", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output motors_output / hold_stabilize", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output VTOL copter surfaces", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output hover vectored pitch", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output elevon vtail mix", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"output surface saturation limits", PortStatus::kThisSlice, "tailsitter_output.hpp"},
    {"Tailsitter::check_input", PortStatus::kThisSlice, "tailsitter.cpp check_input()"},
    {"transition_fw_complete", PortStatus::kThisSlice, "tailsitter_transition_complete.hpp"},
    {"transition_vtol_complete", PortStatus::kThisSlice, "tailsitter_transition_complete.hpp"},
    {"in_vtol_transition", PortStatus::kThisSlice, "tailsitter.cpp"},
    {"is_in_fw_flight", PortStatus::kThisSlice, "tailsitter.cpp"},
    {"get_transition_angle_vtol", PortStatus::kThisSlice, "tailsitter.cpp"},
    {"speed_scaling", PortStatus::kThisSlice, "tailsitter_speed_scaling.hpp"},
    {"write_log TSIT", PortStatus::kOutOfScope, "no logger; HAL_LOGGING_ENABLED"},
    {"relax_pitch", PortStatus::kThisSlice, "tailsitter_speed_scaling.hpp relax_pitch()"},
    {"Tailsitter_Transition FSM update", PortStatus::kThisSlice, "tailsitter.cpp transition"},
    {"Tailsitter_Transition VTOL_update", PortStatus::kThisSlice, "tailsitter.cpp transition"},
    {"show_vtol_view / mav_vtol_state", PortStatus::kThisSlice, "tailsitter.cpp transition"},
    {"set_FW_roll_pitch / pitch limits", PortStatus::kThisSlice, "tailsitter.cpp transition"},
    {"allow_stick_mixing / weathervane", PortStatus::kThisSlice, "tailsitter.cpp transition"},
    {"restart / force_transition_complete", PortStatus::kThisSlice, "tailsitter.cpp transition"},
    {"AP_Param var_info", PortStatus::kOutOfScope, "ADR-0012 inject via setters"},
    {"defaults_table_tailsitter", PortStatus::kOutOfScope, "parameter defaults on QuadPlane"},
    {"QuadPlane& wiring", PortStatus::kOutOfScope, "ADR-0012 caller applies"},
    {"transition object allocation", PortStatus::kOutOfScope, "NEW_NOTHROW on Plane"},
};

[[nodiscard]] inline constexpr std::size_t tailsitter_completeness_size() {
    return sizeof(kTailsitterCompleteness) / sizeof(kTailsitterCompleteness[0]);
}



[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kTailsitterCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kTailsitterCompleteness) {
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

}  // namespace fwcpp::tailsitter
