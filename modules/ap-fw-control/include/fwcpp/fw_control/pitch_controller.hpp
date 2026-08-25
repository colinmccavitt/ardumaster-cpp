#pragma once

// Port of APM_Control/AP_PitchController.h + AP_PitchController.cpp.
// CPP-032. Initial code by Jon Challinger, modified by Paul Riseborough.
// Upstream: libraries/APM_Control/AP_PitchController.{h,cpp}
// (Plane-4.7.0, 28 + 336 lines) - read directly from the pinned upstream
// worktree in full before writing a line of this file, not from
// training-data memory.
//
// See fw_controller.hpp's file banner for the module-wide judgment calls
// this file relies on (composition over inheritance, RateLoopInputs, the
// AP_AutoTune exclusion).
//
// PITCH HAS REAL ADDITIONAL COMPLEXITY ROLL DOESN'T (per the task's own
// warning - confirmed by reading both files in full): get_servo_out()
// adds two whole mechanisms roll's equivalent has no counterpart for:
//   1) _get_coordination_rate_offset() (ported below as
//      get_coordination_rate_offset(), private): a gravity/kinematic
//      correction term - the pitch rate needed, purely from bank angle
//      and airspeed, to hold height in a coordinated turn (a banked
//      aircraft's nose must rise at a rate related to
//      g*tan(bank)*sin(bank)/V to avoid descending). Added to the angle-
//      error-derived desired_rate before the rmax clamp, unless inverted
//      (see get_servo_out() below - inverted flips this from an addend
//      rate offset to preserved sign while desired_rate itself flips).
//   2) The roll-limit pitch-authority blend at the bottom of
//      get_servo_out(): once bank angle exceeds the configured roll
//      limit (plus an 8500cd-capped margin), pitch authority is linearly
//      reduced to zero at 90 degrees of bank - upstream's own comment:
//      "Using elevator for pitch control at large roll angles is
//      ineffective, and can be counter productive as it induces earth-
//      frame yaw which can reduce the ability to roll."
//
// var_info[] DEFAULTS (AP_PitchController::var_info,
// AP_PitchController.cpp) transcribed into Gains below:
//   PTCH2SRV_TCONST   -> tau        (0.5f)
//   PTCH2SRV_RMAX_UP  -> rmax_pos   (0.0f, disabled)
//   PTCH2SRV_RMAX_DN  -> rmax_neg   (0.0f, disabled)
//   PTCH2SRV_RLL      -> roll_ff    (1.0f) - upstream's `_roll_ff`, the
//                         turn-coordination gain multiplier
//   _RATE_P/_I/_D/_FF/_IMAX/_FLTT/_FLTE/_FLTD/_SMAX (AP_SUBGROUPINFO,
//   index 11) -> Gains::rate_pid, defaults from AP_PitchController's
//   constructor's AC_PID::Defaults initializer (NOT var_info, which only
//   documents ranges/units - identical values to roll's):
//     p=0.04, i=0.15, d=0.0, ff=0.345, imax=0.666, filt_T_hz=3.0,
//     filt_E_hz=0.0, filt_D_hz=12.0, srmax=150.0, srtau=1.0
//   (note: p differs from roll's 0.04 vs 0.08 - transcribed exactly as
//   each controller's own constructor initializes it, not assumed equal)
//
// convert_pid() NOT PORTED - same EEPROM/AP_Param-migration reasoning as
// roll_controller.hpp.
//
// LITERAL SAFETY: no bare ambiguous double literals - every upstream bare
// double literal (radians(90), 0.01, 7000/70deg-equivalent, etc.) is
// explicitly float-suffixed here with no value change.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/fw_control/fw_controller.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::fw_control {

// RateLoopInputs plus the two additional attitude readings
// _get_coordination_rate_offset()/get_servo_out()'s roll-limit blend need
// - both are upstream AP::ahrs() reads (get_roll_rad(), get_pitch_rad(),
// and the pitch_sensor/roll_sensor integer-centidegree twins of the same
// two angles - this port works in float radians/degrees throughout
// rather than also carrying redundant centidegree integer copies, since
// float precision loss at the 0.01-degree centidegree quantum is not
// meaningful to any comparison performed against these two values).
struct PitchInputs : RateLoopInputs {
    float bank_angle_rad = 0.0f; // upstream: AP::ahrs().get_roll_rad(), read twice (coordination + roll-limit blend)
    float pitch_rad = 0.0f;      // upstream: AP::ahrs().get_pitch_rad() / .pitch_sensor
};

class PitchController {
public:
    struct Gains {
        float tau = 0.5f;      // PTCH2SRV_TCONST, seconds
        float rmax_pos = 0.0f; // PTCH2SRV_RMAX_UP, deg/s. 0 = disabled
        float rmax_neg = 0.0f; // PTCH2SRV_RMAX_DN, deg/s. 0 = disabled
        float roll_ff = 1.0f;  // PTCH2SRV_RLL, turn-coordination gain
        pid::AcPid::Gains rate_pid{
            .p = 0.04f, .i = 0.15f, .d = 0.0f, .ff = 0.345f, .imax = 0.666f, .filt_t_hz = 3.0f,
            .filt_e_hz = 0.0f,      .filt_d_hz = 12.0f, .srmax = 150.0f, .srtau = 1.0f,
        };
    };

    PitchController(const Gains& gains, const FwAparm& aparm) : base_(gains.rate_pid), gains_(gains), aparm_(aparm) {}

    PitchController(const PitchController&) = delete;
    PitchController& operator=(const PitchController&) = delete;

    // upstream: AP_PitchController::get_servo_out(angle_err, scaler,
    // disable_integrator, ground_mode).
    //
    // Function returns an equivalent elevator deflection in centi-degrees
    // in the range from -4500 to 4500. A positive demand is up.
    float get_servo_out(std::int32_t angle_err_cd, float scaler, bool disable_integrator, bool ground_mode,
                         const PitchInputs& in) {
        if (gains_.tau < 0.05f) {
            gains_.tau = 0.05f;
        }

        const float aspeed = in.airspeed;

        bool inverted = false;
        const float rate_offset = get_coordination_rate_offset(aspeed, in.eas2tas, in.bank_angle_rad, in.pitch_rad, inverted);

        // Calculate the desired pitch rate (deg/sec) from the angle error
        const float angle_err_deg = static_cast<float>(angle_err_cd) * 0.01f;
        float desired_rate = angle_err_deg / gains_.tau;

        // limit the maximum pitch rate demand. Don't apply when inverted
        // as the rates will be tuned when upright, and it is common that
        // much higher rates are needed inverted
        if (!inverted) {
            desired_rate += rate_offset;
            if (gains_.rmax_neg != 0.0f && desired_rate < -gains_.rmax_neg) {
                desired_rate = -gains_.rmax_neg;
            } else if (gains_.rmax_pos != 0.0f && desired_rate > gains_.rmax_pos) {
                desired_rate = gains_.rmax_pos;
            }
        } else {
            // Make sure not to invert the turn coordination offset
            desired_rate = -desired_rate + rate_offset;
        }

        // when we are past the configured roll limit for the aircraft our
        // priority should be to bring the aircraft back within the roll
        // limit - see file banner. Linearly reduce demanded pitch rate
        // when beyond the configured roll limit, reducing to zero at 90
        // degrees.
        float roll_deg = std::fabs(math::degrees(in.bank_angle_rad));
        if (roll_deg > 90.0f) {
            roll_deg = 180.0f - roll_deg;
        }
        const float roll_limit_margin_deg = std::min(aparm_.roll_limit_deg + 5.0f, 85.0f);
        if (roll_deg > roll_limit_margin_deg && std::fabs(math::degrees(in.pitch_rad)) < 70.0f) {
            const float roll_prop = (roll_deg - roll_limit_margin_deg) / (90.0f - roll_limit_margin_deg);
            desired_rate *= (1.0f - roll_prop);
        }

        const bool underspeed = aspeed <= 0.5f * aparm_.airspeed_min;
        return base_.get_rate_out_full(desired_rate, scaler, disable_integrator, underspeed, ground_mode, in);
    }

    // Forwarded from the composed FwController base - see
    // fw_controller.hpp's file banner.
    float get_rate_out(float desired_rate, float scaler, const RateLoopInputs& in) {
        const bool underspeed = in.airspeed <= 0.5f * aparm_.airspeed_min;
        return base_.get_rate_out(desired_rate, scaler, underspeed, in);
    }
    void reset_i() { base_.reset_i(); }
    void decay_i() { base_.decay_i(); }
    void set_ff_scale(float ff_scale) { base_.set_ff_scale(ff_scale); }
    [[nodiscard]] const pid::PidInfo& get_pid_info() const { return base_.get_pid_info(); }
    [[nodiscard]] pid::AcPid& rate_pid() { return base_.rate_pid(); }

private:
    // upstream: AP_PitchController::_get_coordination_rate_offset().
    //
    // Get the rate offset in degrees/second needed for pitch in body
    // frame to maintain height in a coordinated turn. Also returns the
    // inverted flag via the output parameter, matching upstream's
    // `bool &inverted` out-parameter shape.
    float get_coordination_rate_offset(float aspeed, float eas2tas, float bank_angle_in, float pitch_rad,
                                        bool& inverted) const {
        float bank_angle = bank_angle_in;

        // limit bank angle between +- 80 deg if right way up
        if (std::fabs(bank_angle) < math::radians(90.0f)) {
            bank_angle = math::constrain_value(bank_angle, -math::radians(80.0f), math::radians(80.0f));
            inverted = false;
        } else {
            inverted = true;
            if (bank_angle > 0.0f) {
                bank_angle = math::constrain_value(bank_angle, math::radians(100.0f), math::radians(180.0f));
            } else {
                bank_angle = math::constrain_value(bank_angle, -math::radians(180.0f), -math::radians(100.0f));
            }
        }

        float rate_offset;
        if (std::fabs(pitch_rad) > math::radians(70.0f)) {
            // don't do turn coordination handling when at very high pitch
            // angles (upstream: abs(_ahrs.pitch_sensor) > 7000)
            rate_offset = 0.0f;
        } else {
            rate_offset = std::cos(pitch_rad) *
                          std::fabs(math::degrees((kGravityMss / std::max(aspeed * eas2tas, std::max(aparm_.airspeed_min, 1.0f))) *
                                                   std::tan(bank_angle) * std::sin(bank_angle))) *
                          gains_.roll_ff;
        }
        if (inverted) {
            rate_offset = -rate_offset;
        }
        return rate_offset;
    }

    FwController base_;
    Gains gains_;
    FwAparm aparm_;
};

} // namespace fwcpp::fw_control
