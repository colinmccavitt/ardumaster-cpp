#pragma once

#include <cstddef>
#include <cstdint>

namespace fwcpp::qautotune {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct QAutotunePortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr QAutotunePortItem kQAutotuneCompleteness[] = {
    {"QAUTOTUNE mode Number 22", PortStatus::kThisSlice, "mode_qautotune_meta.hpp"},
    {"QAUTOTUNE compile enable gate", PortStatus::kThisSlice, "qautotune_enable.hpp SITL+quadplane"},
    {"QAutoTune::init availability gate", PortStatus::kThisSlice, "qautotune_init.hpp"},
    {"QLOITER position_hold on init", PortStatus::kThisSlice, "qautotune_init.hpp"},
    {"QAutoTune::init_internals wiring", PortStatus::kThisSlice, "qautotune_init_internals.hpp"},
    {"get_desired_climb_rate_ms", PortStatus::kThisSlice, "qautotune_climb_rate.hpp"},
    {"get_pilot_desired_rp_yrate_rad", PortStatus::kThisSlice, "qautotune_pilot_desired.hpp"},
    {"init_z_limits speed/accel bundle", PortStatus::kThisSlice, "qautotune_z_limits.hpp"},
    {"QAutoTune::log_pids", PortStatus::kThisSlice, "qautotune_log_pids.hpp PIQR/PIQP/PIQY"},
    {"ModeQAutotune::_enter", PortStatus::kThisSlice, "mode_qautotune_enter.hpp"},
    {"ModeQAutotune::update delegate QStabilize", PortStatus::kThisSlice, "mode_qautotune_update.hpp"},
    {"ModeQAutotune::run tailsitter FW branch", PortStatus::kThisSlice, "mode_qautotune_run.hpp"},
    {"ModeQAutotune::run qautotune.run hook", PortStatus::kThisSlice, "mode_qautotune_run.hpp"},
    {"ModeQAutotune::run FW stabilize tail", PortStatus::kThisSlice, "mode_qautotune_run.hpp"},
    {"ModeQAutotune::_exit stop", PortStatus::kThisSlice, "mode_qautotune_exit.hpp"},
    {"completeness catalog", PortStatus::kThisSlice, "this table"},
    {"QAutoTune::init_internals body", PortStatus::kOutOfScope, "AC_AutoTune_Multi"},
    {"QAutoTune::run autotune FSM", PortStatus::kOutOfScope, "AC_AutoTune_Multi"},
    {"QAutoTune::stop cleanup", PortStatus::kOutOfScope, "AC_AutoTune_Multi"},
    {"QuadPlane qautotune member", PortStatus::kOutOfScope, "ADR-0012 caller applies"},
    {"Plane previous_mode pointer compare", PortStatus::kOutOfScope, "ADR-0012 caller applies"},
    {"pos_control D_set_max_speed_accel", PortStatus::kOutOfScope, "ADR-0012 poscontrol inject"},
};

[[nodiscard]] inline constexpr std::size_t qautotune_completeness_size() {
    return sizeof(kQAutotuneCompleteness) / sizeof(kQAutotuneCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kQAutotuneCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kQAutotuneCompleteness) {
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

[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}
[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}

}  // namespace fwcpp::qautotune
