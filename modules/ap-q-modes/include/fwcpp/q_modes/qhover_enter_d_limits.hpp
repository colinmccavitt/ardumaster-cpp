#pragma once

#include <fwcpp/pid/ac_pid.hpp>
#include <fwcpp/poscontrol/pos_control_d.hpp>
#include <fwcpp/q_modes/mode_qhover.hpp>

namespace fwcpp::q_modes {

struct QHoverEnterDLimitsResult {
    poscontrol::DLimits limits{};
    bool applied_max_speed_accel{false};
    bool applied_correction_speed_accel{false};
};

[[nodiscard]] inline QHoverEnterDLimitsResult qhover_enter_apply_d_limits(
    poscontrol::DLimits limits, pid::AcP1d& pos_p, pid::AcPid& accel_pid,
    const QHoverEnterEffects& effects) {
    QHoverEnterDLimitsResult out{};
    out.limits = limits;
    if (effects.set_d_max_speed_accel) {
        out.limits = poscontrol::d_set_max_speed_accel_m(
            out.limits, effects.pilot_velocity_z_max_dn_ms, effects.pilot_speed_z_max_up_ms,
            effects.pilot_accel_z_mss, out.limits.jerk_max_d_msss, accel_pid);
        out.applied_max_speed_accel = true;
    }
    if (effects.set_d_correction_speed_accel) {
        poscontrol::d_set_correction_speed_accel_m(pos_p, effects.pilot_velocity_z_max_dn_ms,
                                                   effects.pilot_speed_z_max_up_ms,
                                                   effects.pilot_accel_z_mss);
        out.applied_correction_speed_accel = true;
    }
    return out;
}

}  // namespace fwcpp::q_modes
