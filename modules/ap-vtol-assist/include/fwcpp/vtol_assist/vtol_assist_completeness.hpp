#pragma once

#include <cstddef>
#include <cstdint>

namespace fwcpp::vtol_assist {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct VtolAssistPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr VtolAssistPortItem kVtolAssistCompleteness[] = {
    {"enable/check gate", PortStatus::kThisSlice,
     "VtolAssist STATE / should_check / is_enabled"},
    {"Assist_Hysteresis", PortStatus::kThisSlice,
     "assist_hysteresis.hpp trigger/clear delays"},
    {"should_assist gate inputs", PortStatus::kThisSlice,
     "should_assist.hpp pre-speed early exits"},
    {"speed assist trigger", PortStatus::kThisSlice,
     "assist_triggers.hpp evaluate_speed_assist"},
    {"completeness catalog", PortStatus::kThisSlice, "this table"},
    {"altitude assist trigger", PortStatus::kThisSlice,
     "assist_triggers.hpp evaluate_alt_assist_trigger + delay"},
    {"angle-error trigger", PortStatus::kThisSlice,
     "assist_triggers.hpp envelope + angle error"},
    {"should_assist full OR latch", PortStatus::kThisSlice,
     "should_assist.hpp force||speed||alt||angle + hysteresis reset"},
    {"check_VTOL_recovery", PortStatus::kThisSlice,
     "assist_recovery.hpp check_vtol_recovery FSM"},
    {"output_spin_recovery", PortStatus::kThisSlice,
     "assist_recovery.hpp output_spin_recovery surfaces"},
    {"logging/GCS getters", PortStatus::kThisSlice,
     "assist_gcs_getters.hpp in_*_assist + STATUSTEXT stubs"},
    {"Q_ASSIST_OPTIONS recovery paths", PortStatus::kThisSlice,
     "assist_recovery_options.hpp FW_FORCE / SPIN paths"},
    {"reset() and fly_inverted", PortStatus::kThisSlice,
     "reset_vtol_assist + angle hysteresis skip when inverted"},
    {"AP_Param var_info", PortStatus::kOutOfScope,
     "parameter tree on QuadPlane; inject via VtolAssist setters"},
    {"QuadPlane& wiring", PortStatus::kOutOfScope,
     "ADR-0012 inject inputs; no Plane singleton"},
};

[[nodiscard]] inline constexpr std::size_t vtol_assist_completeness_size() {
    return sizeof(kVtolAssistCompleteness) / sizeof(kVtolAssistCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kVtolAssistCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kVtolAssistCompleteness) {
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

inline constexpr const char* kGcsAltAssistPrefix = "Alt assist";
inline constexpr const char* kGcsAngleAssistPrefix = "Angle assist";

}  // namespace fwcpp::vtol_assist
