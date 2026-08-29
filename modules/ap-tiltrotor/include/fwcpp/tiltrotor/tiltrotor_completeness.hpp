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
    {"setup thrust_type/motor scan", PortStatus::kThisSlice, "tiltrotor_setup_flags.hpp resolve_setup_flags"},
    {"setup SRV tilt servo ranges", PortStatus::kThisSlice, "tiltrotor_setup_flags.hpp tilt_servo_range"},
    {"thrust compensation callback", PortStatus::kThisSlice, "tiltrotor_setup_flags.hpp bind_thrust_compensation"},
    {"Tiltrotor_Transition alloc", PortStatus::kOutOfScope, "NEW_NOTHROW on Plane"},
    {"Tiltrotor::slew", PortStatus::kThisSlice, "tiltrotor_control.hpp slew"},
    {"Tiltrotor::continuous_update", PortStatus::kThisSlice, "tiltrotor_control.hpp continuous_update"},
    {"Tiltrotor::binary_update", PortStatus::kThisSlice, "tiltrotor_control.hpp binary_update"},
    {"Tiltrotor::binary_slew", PortStatus::kThisSlice, "tiltrotor_control.hpp binary_slew"},
    {"Tiltrotor::update", PortStatus::kThisSlice, "tiltrotor_control.hpp update dispatch"},
    {"Tiltrotor::vectoring", PortStatus::kThisSlice, "tiltrotor_control.hpp vectoring"},
    {"Tiltrotor::bicopter_output", PortStatus::kThisSlice, "tiltrotor_output.hpp bicopter_output"},
    {"tilt_compensate_angle", PortStatus::kThisSlice, "tiltrotor_control.hpp tilt_compensate_angle"},
    {"tilt_compensate", PortStatus::kThisSlice, "tiltrotor_control.hpp tilt_compensate"},
    {"tilt_max_change", PortStatus::kThisSlice, "tiltrotor_control.hpp"},
    {"fully_fwd / fully_up", PortStatus::kThisSlice, "tiltrotor_control.hpp"},
    {"get_forward_flight_tilt", PortStatus::kThisSlice, "tiltrotor_control.hpp"},
    {"update_yaw_target", PortStatus::kThisSlice, "tiltrotor_transition.hpp update_yaw_target"},
    {"write_log TRTL", PortStatus::kOutOfScope, "no logger; HAL_LOGGING_ENABLED WriteBlock"},
    {"tilt_over_max_angle", PortStatus::kThisSlice, "tiltrotor_control.hpp"},
    {"get_forward_throttle", PortStatus::kThisSlice, "tiltrotor_output.hpp"},
    {"Tiltrotor_Transition yaw/view/vfwd", PortStatus::kThisSlice, "tiltrotor_transition.hpp"},
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
