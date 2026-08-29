#pragma once

#include <cstddef>
#include <cstdint>

namespace fwcpp::qrtl {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct QrtlPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr QrtlPortItem kQrtlCompleteness[] = {
    {"QRTL mode Number 21", PortStatus::kThisSlice, "mode_qrtl_meta.hpp"},
    {"mode metadata flags", PortStatus::kThisSlice, "is_vtol / auto throttle / pre_arm"},
    {"get_VTOL_return_radius", PortStatus::kThisSlice, "qrtl_vtol_return_radius_m"},
    {"Q_RTL_ALT_MIN constrain", PortStatus::kThisSlice, "qrtl_min_climb_m"},
    {"climb cone target AGL", PortStatus::kThisSlice, "qrtl_climb_cone_target_alt_m"},
    {"calc_best_rally_or_home", PortStatus::kThisSlice, "home vs rally pick"},
    {"_enter QLAND_INSTEAD path", PortStatus::kThisSlice, "guided_wait_takeoff gate"},
    {"_enter climb submode", PortStatus::kThisSlice, "mode_qrtl_enter.hpp"},
    {"_enter RTL close-in POSITION1", PortStatus::kThisSlice, "mode_qrtl_enter.hpp"},
    {"completeness catalog", PortStatus::kThisSlice, "this table"},
    {"update delegate QStabilize", PortStatus::kThisSlice, "mode_qrtl_update.hpp"},
    {"run tailsitter FW branch", PortStatus::kThisSlice, "mode_qrtl_run.hpp"},
    {"run climb tick", PortStatus::kThisSlice, "mode_qrtl_run.hpp"},
    {"run RTL vtol_position_controller", PortStatus::kThisSlice, "mode_qrtl_run.hpp"},
    {"run land handoff verify_vtol_land", PortStatus::kThisSlice, "mode_qrtl_land_handoff.hpp"},
    {"update_target_altitude approach", PortStatus::kThisSlice, "mode_qrtl_target_altitude.hpp"},
    {"allows_throttle_nudging", PortStatus::kThisSlice, "mode_qrtl_target_altitude.hpp"},
    {"QuadPlane::mode_enter wiring", PortStatus::kOutOfScope, "ADR-0012 caller applies"},
};

[[nodiscard]] inline constexpr std::size_t qrtl_completeness_size() {
    return sizeof(kQrtlCompleteness) / sizeof(kQrtlCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kQrtlCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kQrtlCompleteness) {
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

}  // namespace fwcpp::qrtl
