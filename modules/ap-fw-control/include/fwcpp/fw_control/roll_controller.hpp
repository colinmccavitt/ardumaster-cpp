#pragma once

// Port of APM_Control/AP_RollController.h + AP_RollController.cpp.
// CPP-032. Written by Jon Challinger, modified by Paul Riseborough.
// Upstream: libraries/APM_Control/AP_RollController.{h,cpp}
// (Plane-4.7.0, 33 + 268 lines) - read directly from the pinned upstream
// worktree in full before writing a line of this file, not from
// training-data memory.
//
// See fw_controller.hpp's file banner for the module-wide judgment calls
// this file relies on: composition over inheritance, RateLoopInputs
// replacing the is_underspeed()/get_airspeed()/get_measured_rate()
// virtual hooks, and the full AP_AutoTune exclusion.
//
// var_info[] DEFAULTS (AP_RollController::var_info, AP_RollController.cpp)
// transcribed into Gains below:
//   ROLL2SRV_TCONST  -> tau       (0.5f)
//   ROLL2SRV_RMAX    -> rmax_pos  (0, meaning "disabled" - upstream's own
//                        `if (gains.rmax_pos && ...)` treats 0 as "no
//                        limit", reproduced verbatim below)
//   _RATE_P/_I/_D/_FF/_IMAX/_FLTT/_FLTE/_FLTD/_SMAX (AP_SUBGROUPINFO into
//   the embedded AC_PID, index 9) -> Gains::rate_pid, defaults taken from
//   AP_RollController's constructor's AC_PID::Defaults initializer, NOT
//   var_info (var_info only documents ranges/units for these - the real
//   defaults are the constructor's aggregate-init literals):
//     p=0.08, i=0.15, d=0.0, ff=0.345, imax=0.666, filt_T_hz=3.0,
//     filt_E_hz=0.0, filt_D_hz=12.0, srmax=150.0, srtau=1.0
//   _RATE_NTF/_RATE_NEF (notch filter indices) and _RATE_PDMX/_RATE_D_FF
//   are AC_PID sub-fields with no non-zero/non-default value set by
//   AP_RollController's constructor - left at AcPid::Gains's own defaults
//   (matches upstream: an AC_PID::Defaults aggregate-init that doesn't
//   mention a field leaves that field at AC_PID's own class-default).
//
// in_recovery / set_in_recovery() NOT PORTED: upstream's own comment
// states its exact purpose - "set the in_recovery flag, which is used
// during a VTOL upset recovery" - and its only effect is to skip the
// rmax_pos rate-limit clamp for one loop. No VTOL/quadplane vehicle
// exists in this port (task-mandated exclusion), so in_recovery's value
// is permanently false, its only reachable state here - the rmax_pos
// clamp below is applied unconditionally, exactly matching that
// permanent-false behavior with no flag needed to express it.
//
// convert_pid() NOT PORTED: EEPROM/AP_Param old-to-new-gain migration
// helper, meaningless without AP_Param backing (see fw_controller.hpp's
// banner).
//
// LITERAL SAFETY: no bare ambiguous double literals - 0.01f, 0.05f, 160.0f,
// 180.0f, 2.0f all explicitly float-suffixed, matching upstream's own
// (upstream writes some of these, e.g. `angle_err * 0.01`, as bare double
// literals against a float LHS - this port makes every one explicitly
// float per the port's own literal-safety convention, with no change in
// value).

#include <cmath>
#include <cstdint>

#include <fwcpp/fw_control/fw_controller.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::fw_control {

class RollController {
public:
    struct Gains {
        float tau = 0.5f;      // ROLL2SRV_TCONST, seconds
        float rmax_pos = 0.0f; // ROLL2SRV_RMAX, deg/s. 0 = disabled
        pid::AcPid::Gains rate_pid{
            .p = 0.08f, .i = 0.15f, .d = 0.0f, .ff = 0.345f, .imax = 0.666f, .filt_t_hz = 3.0f,
            .filt_e_hz = 0.0f,      .filt_d_hz = 12.0f, .srmax = 150.0f, .srtau = 1.0f,
        };
    };

    RollController(const Gains& gains, const FwAparm& aparm) : base_(gains.rate_pid), gains_(gains), aparm_(aparm) {}

    RollController(const RollController&) = delete;
    RollController& operator=(const RollController&) = delete;

    // upstream: AP_RollController::get_servo_out(angle_err, scaler,
    // disable_integrator, ground_mode).
    //
    // Function returns an equivalent aileron deflection in centi-degrees
    // in the range from -4500 to 4500. A positive demand is up.
    // Inputs are:
    // 1) demanded bank angle in centi-degrees
    // 2) control gain scaler = scaling_speed / aspeed
    // 3) boolean which is true when the integrator should be disabled
    //    (e.g. stabilise mode)
    // 4) boolean which is true when the aircraft is on the ground
    float get_servo_out(std::int32_t angle_err_cd, float scaler, bool disable_integrator, bool ground_mode,
                         const RateLoopInputs& in) {
        if (gains_.tau < 0.05f) {
            gains_.tau = 0.05f;
        }

        // Calculate the desired roll rate (deg/sec) from the angle error
        const float angle_err_deg = static_cast<float>(angle_err_cd) * 0.01f;
        float desired_rate = angle_err_deg / gains_.tau;

        // prevent indecision in the roll controller when target roll is
        // close to 180 degrees from the current roll
        constexpr float indecision_threshold_deg = 160.0f;
        const float last_desired_rate = base_.get_pid_info().target;
        const float abs_angle_err_deg = std::fabs(angle_err_deg);
        if (abs_angle_err_deg > indecision_threshold_deg && angle_err_deg <= 180.0f) {
            if (desired_rate * last_desired_rate < 0.0f) {
                desired_rate = -desired_rate;
                // increase the desired rate in proportion to the extra
                // angle we are requesting
                const float new_angle_err_deg = abs_angle_err_deg + (180.0f - abs_angle_err_deg) * 2.0f;
                desired_rate *= new_angle_err_deg / abs_angle_err_deg;
            }
        }

        // Limit the demanded roll rate. See file banner: in_recovery's
        // "skip this limit for one loop" is not ported (no VTOL/quadplane
        // in this port), so the limit is applied unconditionally.
        if (gains_.rmax_pos != 0.0f && desired_rate < -gains_.rmax_pos) {
            desired_rate = -gains_.rmax_pos;
        } else if (gains_.rmax_pos != 0.0f && desired_rate > gains_.rmax_pos) {
            desired_rate = gains_.rmax_pos;
        }

        const bool underspeed = in.airspeed <= aparm_.airspeed_min;
        return base_.get_rate_out_full(desired_rate, scaler, disable_integrator, underspeed, ground_mode, in);
    }

    // Forwarded from the composed FwController base - see
    // fw_controller.hpp's file banner for why these exist as forwards
    // rather than through inheritance.
    float get_rate_out(float desired_rate, float scaler, const RateLoopInputs& in) {
        const bool underspeed = in.airspeed <= aparm_.airspeed_min;
        return base_.get_rate_out(desired_rate, scaler, underspeed, in);
    }
    void reset_i() { base_.reset_i(); }
    void decay_i() { base_.decay_i(); }
    void set_ff_scale(float ff_scale) { base_.set_ff_scale(ff_scale); }
    [[nodiscard]] const pid::PidInfo& get_pid_info() const { return base_.get_pid_info(); }
    [[nodiscard]] pid::AcPid& rate_pid() { return base_.rate_pid(); }

private:
    FwController base_;
    Gains gains_;
    FwAparm aparm_;
};

} // namespace fwcpp::fw_control
