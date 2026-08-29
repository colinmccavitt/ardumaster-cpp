#pragma once

#include <cstddef>
#include <cstdint>

namespace fwcpp::tiltrotor {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct TiltrotorPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr TiltrotorPortItem kTiltrotorCompleteness[] = {
    {"enable/check gate", PortStatus::kThisSlice, "tiltrotor_enable.hpp enabled()"},
    {"setup enable heuristic", PortStatus::kThisSlice, "tiltrotor_setup.hpp resolve_setup"},
    {"tilt type enums", PortStatus::kThisSlice, "tiltrotor_types.hpp TiltType"},
    {"defaults constants", PortStatus::kThisSlice, "tiltrotor_defaults.hpp Q_TILT_* defaults"},
    {"is_vectored predicate", PortStatus::kThisSlice, "tiltrotor_predicates.hpp setup + type"},
    {"is_motor_tilting", PortStatus::kThisSlice, "tiltrotor_predicates.hpp tilt_mask bit"},
    {"tilt_angle_achieved", PortStatus::kThisSlice, "tiltrotor_predicates.hpp continuous guard"},
    {"get_fully_forward_tilt", PortStatus::kThisSlice, "tiltrotor_predicates.hpp flap scaling"},
    {"is_continuous_type", PortStatus::kThisSlice, "tiltrotor_predicates.hpp"},
    {"completeness catalog", PortStatus::kThisSlice, "this table"},
    {"setup thrust_type/motor scan", PortStatus::kRemaining, "tiltrotor.cpp setup()"},
    {"setup SRV tilt servo ranges", PortStatus::kRemaining, "tiltrotor.cpp setup() vectored yaw"},
    {"thrust compensation callback", PortStatus::kRemaining, "tiltrotor.cpp setup()"},
    {"Tiltrotor_Transition alloc", PortStatus::kOutOfScope, "NEW_NOTHROW on Plane"},
    {"Tiltrotor::slew", PortStatus::kRemaining, "tiltrotor.cpp slew()"},
    {"Tiltrotor::continuous_update", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"Tiltrotor::binary_update", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"Tiltrotor::binary_slew", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"Tiltrotor::update", PortStatus::kRemaining, "tiltrotor.cpp update()"},
    {"Tiltrotor::vectoring", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"Tiltrotor::bicopter_output", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"tilt_compensate_angle", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"tilt_compensate", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"tilt_max_change", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"fully_fwd / fully_up", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"get_forward_flight_tilt", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"update_yaw_target", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"write_log TRTL", PortStatus::kRemaining, "tiltrotor.cpp write_log()"},
    {"tilt_over_max_angle", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"get_forward_throttle", PortStatus::kRemaining, "tiltrotor.cpp"},
    {"Tiltrotor_Transition yaw/view/vfwd", PortStatus::kRemaining, "tiltrotor.cpp transition class"},
    {"AP_Param var_info", PortStatus::kOutOfScope, "ADR-0012 inject via setters"},
    {"QuadPlane& wiring", PortStatus::kOutOfScope, "ADR-0012 caller applies"},
};

[[nodiscard]] inline constexpr std::size_t tiltrotor_completeness_size() {
    return sizeof(kTiltrotorCompleteness) / sizeof(kTiltrotorCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kTiltrotorCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kTiltrotorCompleteness) {
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

[[nodiscard]] inline constexpr std::size_t on_main_count() { return count_status(PortStatus::kOnMain); }
[[nodiscard]] inline constexpr std::size_t this_slice_count() { return count_status(PortStatus::kThisSlice); }
[[nodiscard]] inline constexpr std::size_t remaining_count() { return count_status(PortStatus::kRemaining); }
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() { return count_status(PortStatus::kOutOfScope); }

}  // namespace fwcpp::tiltrotor
