#pragma once

#include <cstddef>
#include <cstdint>

namespace fwcpp::quadplane {

enum class PortStatus : std::uint8_t { kOnMain = 0, kThisSlice = 1, kRemaining = 2, kOutOfScope = 3 };

struct QuadPlanePortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr QuadPlanePortItem kQuadPlaneCompleteness[] = {
    {"Q_ENABLE / enabled", PortStatus::kThisSlice, "enable != 0"},
    {"setup / available / initialised", PortStatus::kThisSlice, "QuadPlaneSetupInputs ADR-0012"},
    {"Q_FRAME_CLASS defaults", PortStatus::kThisSlice, "quadplane_defaults.hpp"},
    {"classify_frame", PortStatus::kThisSlice, "quadplane_frame.hpp"},
    {"motors_kind stub flags", PortStatus::kThisSlice, "post-setup queries"},
    {"motors_init frame_class/type", PortStatus::kThisSlice, "motors->init stub"},
    {"Q_OPTIONS option_is_set", PortStatus::kThisSlice, "quadplane_options.hpp"},
    {"mode_enter lean poscontrol", PortStatus::kThisSlice, "poscontrol stub reset"},
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"get_singleton", PortStatus::kRemaining, "ADR-0012 no singleton"},
    {"AP_Param var_info", PortStatus::kRemaining, "parameter tree"},
    {"setup channels ahrs_view", PortStatus::kThisSlice, "setup_channels.hpp ADR-0012"},
    {"wp_nav loiter_nav", PortStatus::kThisSlice, "setup_navigators.hpp ADR-0012"},
    {"mode_enter poscontrol FSM", PortStatus::kThisSlice, "set_state + init_approach prep"},
    {"update transition FSM", PortStatus::kThisSlice, "quadplane_update.hpp SLT wiring"},
    {"tailsitter tiltrotor", PortStatus::kRemaining, "subsystems"},
    {"vtol controllers landing", PortStatus::kThisSlice, "vtol_position_controller stub"},
    {"AUTO VTOL mission", PortStatus::kThisSlice, "quadplane_auto_vtol_mission.hpp"},
    {"motors_output motor_test", PortStatus::kThisSlice, "motors_output.hpp gating stub"},
    {"guided in_vtol_mode", PortStatus::kThisSlice, "compute_in_vtol_mode"},
    {"air_mode active latch", PortStatus::kThisSlice, "air_mode aux latch"},
    {"TECS stick mixing", PortStatus::kThisSlice, "quadplane_tecs_mixing.hpp"},
    {"logging QControl", PortStatus::kOutOfScope, "no logger"},
    {"scripting dynamic motors", PortStatus::kOutOfScope, "no scripting"},
};

[[nodiscard]] inline constexpr std::size_t quadplane_completeness_size() {
    return sizeof(kQuadPlaneCompleteness) / sizeof(kQuadPlaneCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kQuadPlaneCompleteness) {
        if (item.status == status) ++n;
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kQuadPlaneCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a && *b && *a == *b) { ++a; ++b; }
        if (*a == *b && item.status == status) return true;
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t remaining_count() { return count_status(PortStatus::kRemaining); }
[[nodiscard]] inline constexpr std::size_t on_main_count() { return count_status(PortStatus::kOnMain); }
[[nodiscard]] inline constexpr std::size_t this_slice_count() { return count_status(PortStatus::kThisSlice); }
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() { return count_status(PortStatus::kOutOfScope); }

}
