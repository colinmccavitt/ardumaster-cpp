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
//
// CPP-047 ADDENDUM (see below, after this class's closing brace, for the
// full implementation): the "No AP_Param backing... until a caller wires
// AP_Param in" note directly above is now half-obsolete - this ticket
// wires real AP_Param support for every one of Gains' ten fields, all of
// which turn out to be genuinely upstream-backed (see the addendum for
// the full upstream var_info[] citation). The note is left as-is above
// since it accurately describes SteerController's OWN internals (the
// controller class itself still has no AP_Param dependency baked in -
// Gains stays a plain struct of native C++ fields, per the addendum's own
// "why native_value.hpp, not ParamValue<T>" reasoning).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/param/defaults.hpp>
#include <fwcpp/param/group_info.hpp>
#include <fwcpp/param/native_value.hpp>
#include <fwcpp/param/param.hpp>
#include <fwcpp/param/persistence.hpp>
#include <fwcpp/param/storage.hpp>
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

// === CPP-047 ADDENDUM: real top-level AP_Param GroupInfo/Info table for
// SteerController::Gains (STEER2SRV_ prefix) ===
//
// Phase 2d of the AP_Param vehicle-integration effort CPP-043 started
// (phase 1: Plane::aparm, a FLAT table - see plane.hpp's own "CPP-043
// ADDENDUM"). This ticket follows that same pattern (real Info[]-building
// free function, native_value.hpp's memcpy-based read/write, a real
// save/load round-trip via storage::RawStorage/StorageAccess) but departs
// from aparm's own FLAT shape - see FINDING #1 below for why, verified by
// reading upstream directly rather than assumed.
//
// REAL UPSTREAM SOURCE, read in full for this ticket:
// libraries/APM_Control/AP_SteerController.cpp's var_info[] table and
// AP_SteerController.h's field declarations, plus ArduPlane/Parameters.cpp
// line 838's real registration:
//   GOBJECT(steerController,        "STEER2SRV_",   AP_SteerController),
// (character-by-character: prefix IS "STEER2SRV_" with a trailing
// underscore, exactly as this port's own pre-existing field comments
// above already guessed - confirmed, not assumed).
//
// FINDING #1 - why this ticket builds a GROUP-shaped table, NOT aparm's
// flat per-scalar shape, despite this ticket's own scope text initially
// suggesting "steer_param_info(...) -> std::array<param::Info, N+1>...
// matching CPP-043's aparm_param_info() shape exactly": reading
// AP_SteerController.cpp's var_info[] shows every one of its ten entries
// is an AP_GROUPINFO macro (a GroupInfo-shaped table with per-field name/
// idx/offset, e.g. `AP_GROUPINFO("TCONST", 0, AP_SteerController, _tau,
// 0.75f)`), and Parameters.cpp genuinely registers the WHOLE object under
// ONE GOBJECT/GROUP entry (quoted above) - a real, qualitatively
// different shape from aparm's own real ASCALAR-per-field flat
// registration (CPP-043's own FINDING #1: aparm has NO GROUP entry at
// all). This ticket's own scope text explicitly flagged this exact
// possibility ("or GROUP entry usage of find_group/top_level::find if
// this object turns out to be reachable as a real GROUP... matching
// upstream's real GOBJECT registration") and told the porting agent to
// decide based on what's actually found in var_info[], not by assuming
// either shape - reading upstream directly settles it: a GROUP is the
// upstream-faithful shape here. Concretely: `steer_gains_group_info()`
// below is the real, upstream-shaped GroupInfo[] (ten field entries + a
// sentinel, matching var_info[] name-for-name and idx-for-idx), and
// `steer_param_info()` wraps that ONE GroupInfo table as a SINGLE
// top-level GROUP-type param::Info entry named "STEER2SRV_" plus a
// sentinel - i.e. N=1 real top-level Info entry (the group itself), not
// N=10. This is a smaller top-level array than aparm's 13-entry one, but
// it is the more faithful reproduction of what upstream itself actually
// does for this specific object, and it is what exercises top_level::
// find()'s GROUP-dispatch branch (CPP-043's top_level.hpp) for a REAL
// (not synthetic) table for the first time in this port - CPP-043's own
// top_level_test.cpp had to use a synthetic table for that branch
// precisely because aparm itself didn't need it.
//
// FINDING #2 - every one of Gains' ten fields IS genuinely upstream
// AP_Param-backed, unlike aparm's own partial (13-of-~50) coverage:
// reading AP_SteerController.h's private section directly shows its
// entire field list is `AP_Float _tau, _K_FF, _K_P, _K_I, _K_D,
// _minspeed, _deratespeed, _deratefactor, _mindegree; AP_Int16 _imax;`
// (plus `_last_t`/`_last_out`/`_pid_info`/`_reverse`, none of which are
// AP_Param types and none of which this port's Gains struct carries at
// all - see this file's own "_REVERSE... NOT PORTED" and "pid::PidInfo
// REUSED..." banner notes above). Every AP_Param field has its own
// AP_GROUPINFO entry in var_info[] - full 1:1 coverage between this
// port's Gains and upstream's real AP_Param-backed fields. There is no
// "internal-only tuning constant with no real upstream backing" category
// to name here (unlike aparm's ~37 g/g2/AP_Landing-backed fields, or
// NotchFilterParams' none) - all ten of Gains' fields get a real Info
// entry below.
//
// var_info[] TRANSCRIBED DIRECTLY (name, idx, type, real default -
// AP_SteerController.cpp, read in full):
//   TCONST  idx 0,  AP_Float, 0.75f   -> tau
//   P       idx 1,  AP_Float, 1.8f    -> k_p
//   (idx 2 is ABSENT from the real var_info[] table - a genuine gap in
//    upstream's own numbering, reproduced as found, not filled in; same
//    "index skipped upstream too" precedent as NotchFilterParams' own
//    idx-0 gap, notch_filter_params.hpp)
//   I       idx 3,  AP_Float, 0.2f    -> k_i
//   D       idx 4,  AP_Float, 0.005f  -> k_d
//   IMAX    idx 5,  AP_Int16, 1500    -> imax (centidegrees)
//   MINSPD  idx 6,  AP_Float, 1.0f    -> minspeed (m/s)
//   FF      idx 7,  AP_Float, 0.0f    -> k_ff
//   DRTSPD  idx 8,  AP_Float, 0.0f    -> deratespeed (m/s)
//   DRTFCT  idx 9,  AP_Float, 10.0f   -> deratefactor (deg per m/s)
//   DRTMIN  idx 10, AP_Float, 4500.0f -> mindegree (centidegrees)
// group_element for each of these (verified against AP_Param.cpp's real
// find_var_info()/find_var_info_group() call chain, ~line 634:
// `find_var_info_group(group_info, i, 0, 0, 0, ...)` - group_base and
// group_shift both start at 0 for a top-level GROUP's own direct
// children) is `group_id(table, 0, i, 0)`, which for shift=0 reduces to
// exactly the field's own idx (group_info.hpp's group_id: the idx-0-
// aliasing substitution only triggers at a NONZERO shift) - so
// group_element == idx for every field here, including TCONST's idx 0.
//
// FINDING #3 - why native_value.hpp (CPP-043), not CPP-022 slice 6/7's
// setup_object_defaults/set_value/cast_to_float, same reasoning as
// aparm's own FINDING #4: Gains' fields are plain `float`/`std::int16_t`
// (see this file's own pre-existing "No AP_Param backing" banner note
// and the Gains struct itself, above) - NOT this port's ParamValue<T>/
// ParamFloat wrapper classes (unlike NotchFilterParams, CPP-024, whose
// fields genuinely ARE declared param::ParamFloat). Reinterpreting a
// `float`/`int16_t` object as a `ParamFloat`/`ParamInt16` object to call
// its member functions would be exactly the unsafe reinterpretation
// ADR-0012 forbids, even though the classes share layout on every
// compiler this port targets - the object at that address is not
// actually of that type. get_default_value's GroupInfo overload (defaults
// .hpp) IS reused completely unchanged below - it only ever reads
// info.def_value (a plain float in the GroupInfo struct itself), never
// touching the pointee's static type. Likewise find_group/get_base/
// adjust_group_offset/group_id (group_info.hpp), load_raw/save_raw/scan/
// should_skip_save/type_size (persistence.hpp) are reused completely
// unchanged - none of them touch the pointee's static type either.
//
// KEY ALLOCATION: this port has no vehicle-wide k_param_* key space yet
// (ADR-0013/CPP-043's own note - Plane's real g/g2/AP_Landing-backed
// fields haven't been ported into one either). `SteerParamKey::
// kSteerController` below is this port's OWN single top-level key for
// the whole steerController GROUP object - informed by, but independent
// of, upstream's real Parameters.h `k_param_steerController` enumerator
// value (an EEPROM-migration/ordering detail specific to upstream's own
// numbering, not part of this ticket's scope - CPP-023's conversion
// machinery is the separately-tracked ticket for that). Every one of
// Gains' ten fields shares this ONE key (upstream's own design: a single
// key identifies the containing GROUP object; group_element - the field's
// own idx, per FINDING #2 above - distinguishes which field within it,
// exactly how ParamHeader's key+group_element pair is defined,
// param.hpp).
//
// EXPLICIT, NOT IMPLICIT: apply_steer_defaults/load_steer_parameters/
// save_steer_parameters below are ordinary free functions a caller
// invokes explicitly - NOT called from SteerController's constructor.
// This changes nothing for steer_controller_test.cpp's existing ~20
// tests: none of them call these new functions, and Gains keeps getting
// its real defaults exactly as before, via its own C++ in-class default
// member initializers (independently re-verified against
// AP_SteerController.cpp's real var_info[] defaults for this ticket, per
// FINDING #2's transcription above).
//
// DEFERRED, EXPLICITLY NAMED, NOT SILENTLY SKIPPED (matching this
// ticket's own "explicitly out of scope" list): Plane-level wiring/
// convenience methods binding these free functions to a real
// SteerController instance living inside Plane and to hal.storage (a
// separate future integration ticket, once CPP-044 through CPP-049 all
// land - this ticket must NOT touch plane.hpp/mode.hpp, keeping every
// phase-2 ticket independently mergeable); find_var_info by-pointer-
// identity self-discovery (no real caller - same exclusion CPP-043
// already established for aparm); CPP-023's conversion/upgrade machinery
// for older-format storage.

// This port's own top-level key allocation for the steerController GROUP
// object - see this addendum's "KEY ALLOCATION" note above for why this
// is a single shared key, not one key per field.
enum class SteerParamKey : std::uint16_t {
    kSteerController = 1,
};

namespace detail {

[[nodiscard]] inline constexpr param::GroupInfo make_steer_group_entry(const char* name, std::ptrdiff_t offset, std::uint8_t idx, param::VarType type, float def_value) {
    param::GroupInfo g{};
    g.name = name;
    g.offset = offset;
    g.idx = idx;
    g.type = static_cast<std::uint8_t>(type);
    g.def_value = def_value;
    return g;
}

[[nodiscard]] inline constexpr param::GroupInfo make_steer_group_sentinel() {
    param::GroupInfo g{};
    g.type = static_cast<std::uint8_t>(param::VarType::None);
    return g;
}

} // namespace detail

// The real, upstream-shaped GroupInfo[] for SteerController::Gains: ten
// field entries (name/idx/type/default transcribed directly from
// AP_SteerController.cpp's real var_info[], FINDING #2's citation above)
// plus a VarType::None sentinel, matching every other GroupInfo table in
// this port's AP_Param module (NotchFilterParams::var_info(), CPP-024).
// A `static constexpr` function-local table, matching NotchFilterParams'
// own pattern - offsetof(SteerController::Gains, ...) needs Gains to be a
// COMPLETE type, true here since this function is defined after Gains'
// own closing brace.
[[nodiscard]] inline const param::GroupInfo* steer_gains_group_info() {
    using detail::make_steer_group_entry;
    using detail::make_steer_group_sentinel;
    using param::VarType;
    static constexpr param::GroupInfo table[] = {
        make_steer_group_entry("TCONST", offsetof(SteerController::Gains, tau), 0, VarType::Float, 0.75f),
        make_steer_group_entry("P", offsetof(SteerController::Gains, k_p), 1, VarType::Float, 1.8f),
        // idx 2 intentionally absent - see this addendum's var_info[]
        // transcription above (a genuine gap in upstream's own table).
        make_steer_group_entry("I", offsetof(SteerController::Gains, k_i), 3, VarType::Float, 0.2f),
        make_steer_group_entry("D", offsetof(SteerController::Gains, k_d), 4, VarType::Float, 0.005f),
        make_steer_group_entry("IMAX", offsetof(SteerController::Gains, imax), 5, VarType::Int16, 1500.0f),
        make_steer_group_entry("MINSPD", offsetof(SteerController::Gains, minspeed), 6, VarType::Float, 1.0f),
        make_steer_group_entry("FF", offsetof(SteerController::Gains, k_ff), 7, VarType::Float, 0.0f),
        make_steer_group_entry("DRTSPD", offsetof(SteerController::Gains, deratespeed), 8, VarType::Float, 0.0f),
        make_steer_group_entry("DRTFCT", offsetof(SteerController::Gains, deratefactor), 9, VarType::Float, 10.0f),
        make_steer_group_entry("DRTMIN", offsetof(SteerController::Gains, mindegree), 10, VarType::Float, 4500.0f),
        make_steer_group_sentinel(),
    };
    return table;
}

// Builds a fresh top-level param::Info[] table addressing `gains`
// directly: ONE real GROUP-type entry (name "STEER2SRV_", wrapping
// steer_gains_group_info() above) plus a VarType::None sentinel - see
// this addendum's FINDING #1 for why this is the upstream-faithful shape
// (a real GOBJECT/GROUP registration), not aparm_param_info()'s own flat
// per-scalar shape. Built per-call, not a shared `static` table, for the
// same reason aparm_param_info() is (plane.hpp, CPP-043): this port
// allows more than one live Gains object (the round-trip test below
// constructs two), so there is no single fixed address to bake in at
// compile time.
[[nodiscard]] inline std::array<param::Info, 2> steer_param_info(SteerController::Gains& gains) {
    using param::Info;
    using param::VarType;
    Info group{};
    group.name = "STEER2SRV_";
    group.ptr = &gains;
    group.group_info = steer_gains_group_info();
    group.flags = 0;
    group.key = static_cast<std::uint16_t>(SteerParamKey::kSteerController);
    group.type = static_cast<std::uint8_t>(VarType::Group);
    return {{group, Info{}}}; // Info{} sentinel: type == VarType::None (0) via zero-init
}

// Applies every field's own AP_Param-table default directly into
// `gains`' live fields - see this addendum's "EXPLICIT, NOT IMPLICIT"
// note above for why this is a separate, opt-in function rather than
// something SteerController's constructor calls. Mirrors CPP-022 slice
// 6's setup_object_defaults exactly, except using native_value.hpp's
// set_native_value (FINDING #3) since Gains' fields are plain native C++
// types, not ParamValue<T>/ParamFloat wrappers.
inline void apply_steer_defaults(SteerController::Gains& gains) {
    const auto base = reinterpret_cast<std::ptrdiff_t>(&gains);
    const param::GroupInfo* table = steer_gains_group_info();
    for (std::size_t i = 0; table[i].type != static_cast<std::uint8_t>(param::VarType::None); ++i) {
        void* ptr = reinterpret_cast<void*>(base + table[i].offset);
        param::set_native_value(static_cast<param::VarType>(table[i].type), ptr, param::get_default_value(ptr, table[i]));
    }
}

// Port of AP_Param::load() (AP_Param.cpp ~line 1310, read in full),
// specialized to Gains' own single-level GroupInfo table (FINDING #2's
// group_element == idx finding above means no further nesting-offset
// computation is needed) and to NOT use find_var_info's by-pointer-
// identity self-discovery (out of scope - the caller already knows which
// object/table it's loading, exactly as upstream's own
// load_object_from_eeprom does via a supplied key rather than load()'s
// own self-discovery). Per real upstream load(): if a stored value is
// found, its bytes are read straight into the live object; if not found,
// the default value is applied - both reproduced exactly, using load_raw
// (CPP-022, unchanged: a plain memcpy, safe to target Gains' own live
// field address) and set_native_value (CPP-043) for the not-found/default
// case.
inline void load_steer_parameters(const storage::StorageAccess& storage_access, SteerController::Gains& gains) {
    const auto base = reinterpret_cast<std::ptrdiff_t>(&gains);
    const param::GroupInfo* table = steer_gains_group_info();
    for (std::uint8_t i = 0; table[i].type != static_cast<std::uint8_t>(param::VarType::None); ++i) {
        const auto type = static_cast<param::VarType>(table[i].type);
        void* ptr = reinterpret_cast<void*>(base + table[i].offset);
        param::ParamHeader phdr{};
        phdr.type = table[i].type;
        param::set_key(phdr, static_cast<std::uint16_t>(SteerParamKey::kSteerController));
        phdr.group_element = param::group_id(table, 0, i, 0);
        if (!param::load_raw(storage_access, phdr, ptr, param::type_size(type))) {
            param::set_native_value(type, ptr, param::get_default_value(ptr, table[i]));
        }
    }
}

// Port of AP_Param::save_sync's default-skip-then-write path (AP_Param
// .cpp ~line 1138, read in full), specialized the same way
// load_steer_parameters is above. Reuses should_skip_save (CPP-022,
// persistence.hpp) COMPLETELY UNCHANGED - pure float arithmetic, no
// pointer casting. `force_save` matches upstream's own save_sync
// (force_save, ...) parameter, wired through for a future caller (e.g. a
// GCS PARAM_SET) that needs it, though this ticket's own test relies on
// the default-skip path.
inline void save_steer_parameters(storage::StorageAccess& storage_access, SteerController::Gains& gains, bool force_save = false) {
    const auto base = reinterpret_cast<std::ptrdiff_t>(&gains);
    const param::GroupInfo* table = steer_gains_group_info();
    for (std::uint8_t i = 0; table[i].type != static_cast<std::uint8_t>(param::VarType::None); ++i) {
        const auto type = static_cast<param::VarType>(table[i].type);
        void* ptr = reinterpret_cast<void*>(base + table[i].offset);
        const float current = param::native_cast_to_float(type, ptr);
        const float default_value = param::get_default_value(ptr, table[i]);
        if (param::should_skip_save(type, current, default_value, force_save)) {
            continue;
        }
        param::ParamHeader phdr{};
        phdr.type = table[i].type;
        param::set_key(phdr, static_cast<std::uint16_t>(SteerParamKey::kSteerController));
        phdr.group_element = param::group_id(table, 0, i, 0);
        (void)param::save_raw(storage_access, phdr, ptr, param::type_size(type));
    }
}

} // namespace fwcpp::steer_control
