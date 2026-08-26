#pragma once

// Port of APM_Control/AP_SteerController.h + AP_SteerController.cpp -
// ground/taxi steering (nosewheel/tailwheel rudder control while on the
// ground), distinct from in-air coordinated-turn rudder (YawController,
// ap-fw-control, CPP-032). Written by Andrew Tridgell, based upon the roll
// controller by Paul Riseborough and Jon Challinger (upstream's own file
// banner). Upstream: libraries/APM_Control/AP_SteerController.{h,cpp}
// (Plane-4.7.0, 72 + 258 lines) - read directly from the pinned upstream
// worktree in full before writing a line of this file, not from
// training-data memory.
//
// NEW MODULE, NOT FOLDED INTO ap-fw-control - A DELIBERATE CHOICE, made
// after reading upstream's real implementation (the porting ticket left
// this open deliberately, pending that read): unlike RollController/
// PitchController/YawController (ap-fw-control, CPP-032), all three of
// which COMPOSE FwController, which itself wraps this port's AC_PID
// (pid::AcPid) - see fw_controller.hpp's own banner - AP_SteerController
// has NO AC_PID dependency at all. Its rate loop
// (get_steering_out_rate(), below) is hand-rolled arithmetic private to
// this one class: its own ki_rate/kp_ff/k_ff gain-equivalence conversion
// (upstream's own comment: "Calculate equivalent gains so that values for
// K_P and K_I can be taken across from the old PID law"), its own manual
// integrator with a sign-aware anti-windup clamp against the PREVIOUS
// output (not AC_PID's own internal limiter), and its own D-term computed
// directly from the raw rate error (not a filtered derivative of the
// process variable, unlike AC_PID's D handling) - there is no shared base
// or PID object to reuse here. It is also conceptually a different
// subsystem from ap-fw-control: taxi/ground steering, not in-air attitude
// stabilization - the two only resemble each other at the "gains struct +
// rate/angle-error entry points" shape level, the SAME resemblance
// L1Control/Tecs/the fw-control controllers already share with EACH OTHER
// despite living in three separate modules (ap-nav/ap-tecs/ap-fw-control).
// Matching that existing one-module-per-control-law-family convention,
// and given zero shared code with ap-pid/ap-fw-control, this lands in its
// own module (ap-steer-control) instead of growing ap-fw-control with an
// outlier internal mechanism.
//
// pid::PidInfo REUSED PURELY AS A CONVENIENT AGGREGATE, NOT pid::AcPid
// ITSELF: get_pid_info() below returns the same already-tested
// target/actual/p/i/d/ff struct (ap-pid, ac_pid.hpp) every other
// controller in this port already exposes, for the same debug/test-
// introspection purpose - but nothing here calls into pid::AcPid's own
// update logic. Only target/actual/p/i/d/ff are ever written (matching
// upstream's own _pid_info usage exactly); error/dff/dmod/slew_rate/
// limit/pd_limit/reset/i_term_set are never touched, staying at their
// own defaults, exactly as upstream's AP_SteerController never touches
// the AP_PIDInfo fields it has no equivalent computation for either.
//
// NO SINGLETONS, EXPLICIT INPUTS INSTEAD (ADR-0012), matching this port's
// RateLoopInputs (fw_controller.hpp)/L1Inputs/TecsInputs precedent: the
// porting ticket's own suggested signature - get_steering_out_rate(float
// desired_rate, float dt, std::uint32_t now_ms) - is adapted here to
// bundle dt/now_ms TOGETHER WITH the two upstream singleton reads this
// controller also needs (AP::ahrs().groundspeed(), degrees(AP::ahrs().
// get_yaw_rate_earth())) into one SteerInputs struct, passed as the final
// argument - exactly RateLoopInputs's own established shape (dt/now_ms
// bundled alongside measured_rate/airspeed/eas2tas in ONE struct, not
// three-plus loose scalar parameters). This is a deliberate consistency
// choice, not a shortcut: every other per-tick controller entry point in
// this port (RollController::get_servo_out, L1Control::update_waypoint,
// Tecs::update_50hz) takes its scalar demand(s) as named parameters and
// everything else AP::ahrs()/AP::gps()-shaped as one bundled input
// struct.
//
// _REVERSE / set_reverse() NOT PORTED: grepping ArduPlane's own source
// (ArduPlane/*.cpp) for every `steerController.` call site shows
// set_reverse() has NO caller anywhere in ArduPlane - it exists purely
// for Rover's reverse-driving use of this same library, a vehicle this
// port does not build. Dropping it also sidesteps a genuine upstream
// quirk: AP_SteerController's private `bool _reverse;` has no in-class
// initializer and is not AP_Param-backed, so it is only ever
// deterministically false because AP_SteerController lives inside a
// global, zero-initialized `Plane` object upstream - a reliance on static
// storage duration this port does not want to reproduce. With no
// ArduPlane caller ever setting it away from that default, this port
// simply never carries the field at all - not an invented behavior
// change, since ArduPlane's own _reverse is permanently false in
// practice either way.
//
// get_steering_out_lat_accel() / get_turn_radius() / active() NOT PORTED:
// grepping ArduPlane's own source shows get_steering_out_lat_accel() and
// get_turn_radius() have NO caller anywhere in ArduPlane (Rover-only
// entry points again), and active() is called only from Log.cpp/
// GCS_MAVLink_Plane.cpp - a logging/telemetry subsystem this port has
// never built (matching every other "no GCS/logger" exclusion already
// established elsewhere in this port). calc_nav_yaw_ground()/
// calc_nav_yaw_course() (plane.hpp, ap-vehicle) - this controller's only
// two REAL ArduPlane call sites - use exclusively get_steering_out_rate()
// and get_steering_out_angle_error(), both ported below in full.
//
// var_info[] DEFAULTS (AP_SteerController::var_info, AP_SteerController.cpp)
// transcribed into Gains below, by AP_GROUPINFO parameter name (NOT
// index - index 2 is absent from upstream's own table, a genuine gap in
// the real var_info[], not an omission here):
//   STEER2SRV_TCONST  -> tau          (0.75f)
//   STEER2SRV_P       -> k_p          (1.8f)
//   STEER2SRV_I       -> k_i          (0.2f)
//   STEER2SRV_D       -> k_d          (0.005f)
//   STEER2SRV_IMAX    -> imax         (1500, centidegrees)
//   STEER2SRV_MINSPD  -> minspeed     (1.0f, m/s)
//   STEER2SRV_FF      -> k_ff         (0.0f)
//   STEER2SRV_DRTSPD  -> deratespeed  (0.0f, m/s - 0 means "no derating")
//   STEER2SRV_DRTFCT  -> deratefactor (10.0f, deg per m/s over deratespeed)
//   STEER2SRV_DRTMIN  -> mindegree    (4500.0f, centidegrees)
// No AP_Param backing (same "plain field, upstream's real default"
// precedent as pid::AcPid::Gains/nav::L1Control::Gains/tecs::Tecs::Gains)
// until a caller wires AP_Param in.
//
// convert_pid() NOT PORTED: same EEPROM/AP_Param old-to-new-gain
// migration helper already excluded from fw_controller.hpp - meaningless
// without AP_Param backing the gains.
//
// LITERAL SAFETY: no bare ambiguous double literals - every constant
// below (45.0f, 0.001f, 0.01f, 4.0f, 4500.0f, etc.) is an explicit
// float-suffixed literal, matching upstream's own values exactly (verified
// against AP_SteerController.cpp directly, not re-derived).

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/pid/ac_pid.hpp>

namespace fwcpp::steer_control {

// Everything get_steering_out_rate()/get_steering_out_angle_error() need
// per tick that upstream read from AP::ahrs()/AP_HAL::millis() - see file
// banner's "NO SINGLETONS" note.
struct SteerInputs {
    float ground_speed_ms = 0.0f;    // upstream: AP::ahrs().groundspeed()
    float yaw_rate_earth_dps = 0.0f; // upstream: degrees(AP::ahrs().get_yaw_rate_earth())
    float dt = 0.0f;                 // seconds. Upstream derives dt from AP_HAL::millis() itself (see below) -
                                      // kept here anyway for shape-consistency with every other *Inputs struct in
                                      // this port; NOT read by get_steering_out_rate() below (which recomputes its
                                      // own dt_ms from now_ms - last_t_ms_, matching upstream's real tnow-_last_t
                                      // arithmetic exactly, including its own >1000ms/first-call reset rule - a
                                      // caller-supplied dt could not reproduce that reset rule faithfully).
    std::uint32_t now_ms = 0;        // upstream: AP_HAL::millis()
};

// upstream: AP_SteerController (libraries/APM_Control). Ground/taxi
// steering rate and angle-error controller - see file banner for the
// full "why a new module, why not AC_PID" rationale.
class SteerController {
public:
    struct Gains {
        float tau = 0.75f;          // STEER2SRV_TCONST, seconds
        float k_p = 1.8f;           // STEER2SRV_P
        float k_i = 0.2f;           // STEER2SRV_I
        float k_d = 0.005f;         // STEER2SRV_D
        std::int16_t imax = 1500;   // STEER2SRV_IMAX, centidegrees
        float minspeed = 1.0f;      // STEER2SRV_MINSPD, m/s
        float k_ff = 0.0f;          // STEER2SRV_FF
        float deratespeed = 0.0f;   // STEER2SRV_DRTSPD, m/s (0 = no derating)
        float deratefactor = 10.0f; // STEER2SRV_DRTFCT, deg per m/s over deratespeed
        float mindegree = 4500.0f;  // STEER2SRV_DRTMIN, centidegrees
    };

    explicit SteerController(const Gains& gains) : gains_(gains) {}

    SteerController(const SteerController&) = delete;
    SteerController& operator=(const SteerController&) = delete;

    // upstream: AP_SteerController::get_steering_out_rate(desired_rate).
    // Returns a steering servo output from -4500 to 4500 given a desired
    // yaw rate in degrees/sec. Positive yaw rate means clockwise
    // (rightward) yaw - ported field-for-field, condition-for-condition.
    [[nodiscard]] std::int32_t get_steering_out_rate(float desired_rate, const SteerInputs& in) {
        // upstream: `uint32_t dt = tnow - _last_t; if (_last_t == 0 || dt >
        // 1000) { dt = 0; } _last_t = tnow;` - a stale-or-first-call reset,
        // not a caller-suppliable dt (see SteerInputs::dt's own doc comment).
        std::uint32_t dt_ms = 0;
        if (last_t_ms_ != 0 && (in.now_ms - last_t_ms_) <= 1000U) {
            dt_ms = in.now_ms - last_t_ms_;
        }
        last_t_ms_ = in.now_ms;

        float speed = in.ground_speed_ms;
        if (speed < gains_.minspeed) {
            // assume a minimum speed. This stops oscillations when first
            // starting to move.
            speed = gains_.minspeed;
        }

        // this is a linear approximation of the inverse steering equation
        // for a ground vehicle. It returns steering as an angle from -45
        // to 45.
        const float scaler = 1.0f / speed;

        pid_info_.target = desired_rate;

        // Calculate the steering rate error (deg/sec) and apply gain
        // scaler. Upstream does this in earth frame to allow for the
        // vehicle leaning over in hard corners - see SteerInputs::
        // yaw_rate_earth_dps's own doc comment for its provenance.
        const float yaw_rate_earth = in.yaw_rate_earth_dps;
        pid_info_.actual = yaw_rate_earth;

        const float rate_error = (desired_rate - yaw_rate_earth) * scaler;

        // Calculate equivalent gains so that values for K_P and K_I can be
        // taken across from the old PID law. No conversion is required
        // for K_D.
        const float ki_rate = gains_.k_i * gains_.tau * 45.0f;
        const float kp_ff = std::max((gains_.k_p - gains_.k_i * gains_.tau) * gains_.tau - gains_.k_d, 0.0f) * 45.0f;
        const float k_ff = gains_.k_ff * 45.0f;
        const float delta_time = static_cast<float>(dt_ms) * 0.001f;

        // Multiply yaw rate error by ki_rate and integrate. Don't
        // integrate if in stabilize mode (upstream's own comment) as the
        // integrator will wind up against the pilot's inputs - modeled
        // here exactly as upstream models it, via the ki_rate>0/speed
        // gate below, not a separate disable_integrator flag (this
        // controller has none - see calc_nav_yaw_ground()/
        // calc_nav_yaw_course(), ap-vehicle, for its only two real
        // callers, neither of which ever needs one).
        if (ki_rate > 0.0f && speed >= gains_.minspeed) {
            // only integrate if gain and time step are positive.
            if (dt_ms > 0U) {
                float integrator_delta = rate_error * ki_rate * delta_time * scaler;
                if (last_out_ < -45.0f) {
                    // prevent the integrator from increasing if steering
                    // deflection demand is above the upper limit
                    integrator_delta = std::max(integrator_delta, 0.0f);
                } else if (last_out_ > 45.0f) {
                    // prevent the integrator from decreasing if steering
                    // deflection demand is below the lower limit
                    integrator_delta = std::min(integrator_delta, 0.0f);
                }
                pid_info_.i += integrator_delta;
            }
        } else {
            pid_info_.i = 0.0f;
        }

        // Scale the integration limit and constrain the integrator state.
        const float int_lim_scaled = static_cast<float>(gains_.imax) * 0.01f;
        pid_info_.i = math::constrain_value(pid_info_.i, -int_lim_scaled, int_lim_scaled);

        pid_info_.d = rate_error * gains_.k_d * 4.0f;
        pid_info_.p = (math::radians(desired_rate) * kp_ff) * scaler;
        pid_info_.ff = (math::radians(desired_rate) * k_ff) * scaler;

        // Calculate the demanded control surface deflection.
        last_out_ = pid_info_.d + pid_info_.ff + pid_info_.p + pid_info_.i;

        float derate_constraint = 4500.0f;
        // Calculate required constraint based on speed.
        if (gains_.deratespeed != 0.0f && speed > gains_.deratespeed) {
            derate_constraint = 4500.0f - (speed - gains_.deratespeed) * gains_.deratefactor * 100.0f;
            if (derate_constraint < gains_.mindegree) {
                derate_constraint = gains_.mindegree;
            }
        }

        // Convert to centi-degrees and constrain. Upstream's own
        // `constrain_float(...)` return value implicitly truncates to
        // int32_t on return - reproduced here with an explicit
        // static_cast rather than an implicit narrowing conversion, same
        // "explicit, not implicit" precedent this port's literal-safety
        // convention already applies elsewhere.
        return static_cast<std::int32_t>(math::constrain_value(last_out_ * 100.0f, -derate_constraint, derate_constraint));
    }

    // upstream: AP_SteerController::get_steering_out_angle_error(angle_err).
    // Returns a steering servo output from -4500 to 4500 given an angular
    // steering error in centidegrees.
    [[nodiscard]] std::int32_t get_steering_out_angle_error(std::int32_t angle_err_cd, const SteerInputs& in) {
        // upstream mutates the underlying tau parameter in place if it's
        // below the floor - reproduced here exactly, matching
        // RollController::get_servo_out()'s own `if (gains_.tau < 0.05f) {
        // gains_.tau = 0.05f; }` mutate-in-place precedent (roll_
        // controller.hpp).
        if (gains_.tau < 0.1f) {
            gains_.tau = 0.1f;
        }

        // Calculate the desired steering rate (deg/sec) from the angle
        // error.
        const float desired_rate = static_cast<float>(angle_err_cd) * 0.01f / gains_.tau;

        return get_steering_out_rate(desired_rate, in);
    }

    // upstream: AP_SteerController::reset_I() - `_pid_info.I = 0;`.
    void reset_I() { pid_info_.i = 0.0f; }

    [[nodiscard]] const pid::PidInfo& get_pid_info() const { return pid_info_; }

private:
    Gains gains_;
    std::uint32_t last_t_ms_ = 0;
    float last_out_ = 0.0f;
    pid::PidInfo pid_info_{};
};

} // namespace fwcpp::steer_control
