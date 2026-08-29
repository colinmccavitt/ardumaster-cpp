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
    {"setup SRV surface flags", PortStatus::kRemaining, "tailsitter.cpp setup()"},
    {"enable==2 assist/airmode/arm", PortStatus::kRemaining, "tailsitter.cpp setup()"},
    {"transition_rate_fw auto-set", PortStatus::kRemaining, "tailsitter.cpp setup()"},
    {"Tailsitter::active", PortStatus::kRemaining, "tailsitter.cpp active()"},
    {"Tailsitter::output", PortStatus::kRemaining, "tailsitter.cpp output()"},
    {"Tailsitter::check_input", PortStatus::kRemaining, "tailsitter.cpp check_input()"},
    {"transition_fw_complete", PortStatus::kRemaining, "tailsitter.cpp"},
    {"transition_vtol_complete", PortStatus::kRemaining, "tailsitter.cpp"},
    {"in_vtol_transition", PortStatus::kRemaining, "tailsitter.cpp"},
    {"is_in_fw_flight", PortStatus::kRemaining, "tailsitter.cpp"},
    {"get_transition_angle_vtol", PortStatus::kRemaining, "tailsitter.cpp"},
    {"speed_scaling", PortStatus::kRemaining, "tailsitter.cpp"},
    {"write_log TSIT", PortStatus::kRemaining, "tailsitter.cpp write_log()"},
    {"relax_pitch", PortStatus::kRemaining, "tailsitter.cpp relax_pitch()"},
    {"Tailsitter_Transition FSM update", PortStatus::kRemaining, "tailsitter.cpp transition"},
    {"Tailsitter_Transition VTOL_update", PortStatus::kRemaining, "tailsitter.cpp transition"},
    {"show_vtol_view / mav_vtol_state", PortStatus::kRemaining, "tailsitter.cpp transition"},
    {"set_FW_roll_pitch / pitch limits", PortStatus::kRemaining, "tailsitter.cpp transition"},
    {"allow_stick_mixing / weathervane", PortStatus::kRemaining, "tailsitter.cpp transition"},
    {"restart / force_transition_complete", PortStatus::kRemaining, "tailsitter.cpp transition"},
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
