#pragma once

// Port of APM_Control/AP_FW_Controller.h + AP_FW_Controller.cpp. CPP-032,
// slice 1 (shared base). Written by Jon Challinger, modified by Paul
// Riseborough. Upstream: libraries/APM_Control/AP_FW_Controller.{h,cpp}
// (Plane-4.7.0, 67 + 158 lines) - read directly from the pinned upstream
// worktree in full before writing a line of this file, not from
// training-data memory.
//
// DISCOVERED WHILE SCOPING CPP-031: Attitude.cpp's stabilize_roll()/
// stabilize_pitch()/stabilize_yaw() call AP_RollController::get_servo_out()
// / AP_PitchController::get_servo_out() / AP_YawController::get_servo_out()
// - thin wrappers AROUND this port's already-ported AcPid (CPP-016), not
// AcPid directly. This module ports those wrappers: the airspeed/scaler
// input-scaling and rate-shaping logic, not the PID machinery itself
// (already done).
//
// COMPOSITION INSTEAD OF PUBLIC INHERITANCE: upstream's AP_RollController/
// AP_PitchController publicly inherit AP_FW_Controller and override its
// three pure-virtual hooks (is_underspeed/get_airspeed/get_measured_rate).
// That inheritance exists to let AP_AutoTune - which holds a
// `rate_pid&` and calls back into the shared _get_rate_out() machinery
// through the concrete subclass - operate uniformly across axes via a
// common AP_FW_Controller* /virtual dispatch. With AP_AutoTune entirely
// out of scope (see below), nothing in this port ever needs an
// AP_FW_Controller* to dispatch through - RollController/PitchController/
// YawController are always called by their concrete type. This module
// therefore uses FwController as a plain COMPOSED member (no virtual
// functions, no vtable - ADR-0012 already disfavors indirection that has
// no live caller), not a base class. RollController/PitchController each
// hold a `FwController base_` and forward the handful of upstream public
// members (get_rate_out/reset_i/decay_i/set_ff_scale/get_pid_info/
// rate_pid()) that make sense for a caller outside get_servo_out() itself
// (e.g. ACRO-mode direct rate control, expected to matter to CPP-031).
//
// NO SINGLETONS, EXPLICIT INPUTS INSTEAD (ADR-0012), matching this port's
// L1Control/Tecs/AhrsDcm precedent:
//   - is_underspeed()/get_airspeed()/get_measured_rate() were upstream
//     PURE VIRTUAL hooks the base class called on itself (`get_measured_
//     rate()`, `is_underspeed(aspeed)`) to reach AP::ahrs(). Removing the
//     virtual dispatch removes the hooks too: RateLoopInputs (below)
//     bundles what each hook actually read (AP::ahrs().get_gyro().{x,y,z},
//     AP::ahrs().airspeed_EAS(), AP::ahrs().get_EAS2TAS()) as plain fields
//     a caller fills in per tick - same shape as L1Inputs/TecsInputs.
//     `is_underspeed()`'s own comparison (aspeed <= threshold) differs per
//     axis (roll/yaw: <= airspeed_min; pitch: <= 0.5*airspeed_min) so it
//     is computed by the concrete controller (which owns the threshold),
//     not by this shared base - get_rate_out_full() below takes the
//     already-decided `underspeed` bool, exactly mirroring how upstream's
//     base class received one bool result from a per-axis virtual call.
//   - AP::scheduler().get_loop_period_s() -> RateLoopInputs::dt, an
//     explicit parameter (AcPid/L1Control/Tecs precedent: all take dt
//     explicitly rather than reaching for a scheduler singleton).
//   - AP_HAL::millis() (threaded implicitly into AC_PID's slew limiter
//     upstream) -> RateLoopInputs::now_ms, matching AcPid::update_all's
//     own explicit now_ms parameter (see ac_pid.hpp's file banner).
//
// AP_AUTOTUNE: ENTIRELY OUT OF SCOPE. No AP_AutoTune port exists or is
// planned (task-mandated exclusion). Concretely dropped from this base:
//   - The `AP_AutoTune::ATGains gains`, `AP_AutoTune *autotune`,
//     `bool failed_autotune_alloc`, and `const AP_AutoTune::ATType
//     autotune_type` members - all autotune-only bookkeeping.
//   - `autotune_start()`/`autotune_restore()` - both call sites into
//     AP_AutoTune, no equivalent needed with no autotune subsystem.
//   - The `if (autotune != nullptr && autotune->running ...)
//     autotune->update(pinfo, scaler, angle_err_deg)` block inside
//     _get_rate_out() (~line 96-99 of AP_FW_Controller.cpp) - dropped
//     along with `angle_err_deg` itself, an AP_FW_Controller member whose
//     ONLY reader anywhere in the base class is that autotune call.
//     RollController/PitchController compute their own local
//     angle_err_deg (needed for the rate-from-angle-error math) without
//     ever storing it on the shared base, since nothing else reads it.
//   - AP_RollController::convert_pid()/AP_PitchController::convert_pid()
//     (old-PID-to-new-PID one-time migration helpers) - EEPROM/AP_Param-
//     specific, meaningless without AP_Param backing the gains (same
//     "NOT PORTED" precedent ac_pid.hpp's own banner already established
//     for AC_PID::load_gains/save_gains/var_info).
//   - Every `AP_Param::GroupInfo var_info[]` table - no AP_Param backing
//     these gains (this port's AcPid::Gains/L1Control::Gains/Tecs::Gains
//     precedent: plain float fields with upstream's real default value).
//     Every default below is transcribed from the corresponding
//     var_info[] AP_GROUPINFO/AP_SUBGROUPINFO entry, cited by parameter
//     name in each controller's own header.
//
// decay_I() IS PORTED (get_rate_out_full()'s caller-visible decay_i()
// below), despite existing upstream purely "for when we have a low scale
// factor in a quadplane hover" (its own doc comment) and this port having
// no quadplane/VTOL vehicle at all: it is a two-line, self-contained
// method with no VTOL-specific types or dependencies, so leaving it out
// would only remove a harmless, independently testable piece of parity
// for zero simplification benefit - unlike in_recovery (see
// roll_controller.hpp), which is genuinely `#if`-shaped around real VTOL
// behavior. A future quadplane port slice can call it exactly as upstream
// would, with no changes needed here.
//
// LITERAL SAFETY: no bare ambiguous double literals - every constant
// below (45.0f threshold, 0.995f decay, 4500.0f/-4500.0f clamp, etc.) is
// explicitly float-suffixed, matching upstream's own literals exactly.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/pid/ac_pid.hpp>

namespace fwcpp::fw_control {

// upstream: AP_Math/definitions.h's GRAVITY_MSS (9.80665f) - same constant
// this port's l1_control.hpp/tecs.hpp/ahrs_dcm.hpp/sim_plane.hpp already
// each independently reproduce (established per-module-copy precedent -
// see e.g. tecs.hpp's own note pointing back to l1_control.hpp's copy).
inline constexpr float kGravityMss = 9.80665f;

// Everything the shared rate-loop (get_rate_out_full()) needs per tick
// that upstream read from AP::ahrs()/AP::scheduler() singletons - see
// file banner. Shared by RollController, PitchController, and
// YawController::get_rate_out() (the direct-rate-control entry point;
// see yaw_controller.hpp for why yaw's OTHER entry point,
// get_servo_out(), does not use this struct at all).
struct RateLoopInputs {
    float measured_rate = 0.0f; // rad/s, body-axis rate for this axis. Upstream: AP::ahrs().get_gyro().{x,y,z}
    float airspeed = 0.0f;      // m/s EAS. Upstream: get_airspeed()'s AP::ahrs().airspeed_EAS() call - each
                                 // controller's own get_airspeed() override had a different no-airspeed-sensor
                                 // fallback (roll/yaw: 0; pitch: average of airspeed_min/airspeed_max); the
                                 // caller building this struct is expected to apply the same fallback the
                                 // relevant controller's own file banner documents.
    float eas2tas = 1.0f;       // upstream: AP::ahrs().get_EAS2TAS()
    float dt = 0.0f;            // seconds. Upstream: AP::scheduler().get_loop_period_s()
    std::uint32_t now_ms = 0;   // upstream: implicit AP_HAL::millis() inside AC_PID's slew limiter
};

// The subset of AP_FixedWing (libraries/AP_Vehicle/AP_FixedWing.h) that
// APM_Control's rate controllers actually read - three fields out of
// upstream's ~30. No AP_FixedWing-equivalent aggregate exists in this
// port (task-mandated: "take just the specific field(s) each controller
// actually reads"). Shared across Roll/Pitch/Yaw the same way Tecs's own
// nested FixedWingParams bundles fields only some of its callers need.
struct FwAparm {
    float airspeed_min = 9.0f;    // AP_FixedWing::airspeed_min (AIRSPEED_MIN), m/s. Upstream AP_Int16 default AIRSPEED_FBW_MIN
    float airspeed_max = 22.0f;   // AP_FixedWing::airspeed_max (AIRSPEED_MAX), m/s. Upstream AP_Int16 default AIRSPEED_FBW_MAX
    float roll_limit_deg = 45.0f; // AP_FixedWing::roll_limit (ROLL_LIMIT_DEG), deg. Upstream AP_Float default ROLL_LIMIT_DEG (ArduPlane/config.h). PitchController-only.
};

// Port of AP_FW_Controller's shared AC_PID-based rate loop. See file
// banner for why this is a composed member, not a base class.
class FwController {
public:
    explicit FwController(const pid::AcPid::Gains& pid_gains) : rate_pid_(pid_gains) {
        // upstream: AP_FW_Controller's constructor body,
        // `rate_pid.set_slew_limit_scale(45);` - unconditional, same for
        // every axis (roll/pitch inherit it from the shared base; yaw
        // duplicates the identical call in its own constructor - see
        // yaw_controller.hpp).
        rate_pid_.set_slew_limit_scale(45);
    }

    FwController(const FwController&) = delete;
    FwController& operator=(const FwController&) = delete;

    // upstream: AP_FW_Controller::get_rate_out(desired_rate, scaler) - the
    // simple public entry point (disable_integrator=false, ground_mode=
    // false). `underspeed` replaces upstream's internal
    // `is_underspeed(get_airspeed())` call - see file banner.
    float get_rate_out(float desired_rate, float scaler, bool underspeed, const RateLoopInputs& in) {
        return get_rate_out_full(desired_rate, scaler, /*disable_integrator=*/false, underspeed,
                                  /*ground_mode=*/false, in);
    }

    // upstream: AP_FW_Controller::_get_rate_out(desired_rate, scaler,
    // disable_integrator, aspeed, ground_mode). `aspeed` itself is not
    // used inside this function upstream except to feed is_underspeed()
    // (called by the PUBLIC get_rate_out()) and the dropped autotune gate
    // (`aspeed > aparm.airspeed_min`) - so it is not part of this port's
    // signature at all; `underspeed` (already resolved) and `in` (which
    // still carries `in.airspeed` for completeness/logging parity, unused
    // in the arithmetic below, exactly as upstream's own `aspeed`
    // parameter was) take its place.
    float get_rate_out_full(float desired_rate, float scaler, bool disable_integrator, bool underspeed,
                             bool ground_mode, const RateLoopInputs& in) {
        bool limit_i = std::fabs(last_out_) >= 45.0f;
        const float rate = in.measured_rate;
        const float old_i = rate_pid_.get_i();

        if (underspeed) {
            // when underspeed we lock the integrator
            limit_i = true;
        }

        // the PID elements are scaled by sq(scaler). To use an unmodified
        // AcPid object we scale the inputs (target and measurement).
        //
        // note that we run AcPid in radians so that the normal scaling
        // range for imax in AcPid applies (usually an imax value less
        // than 1.0)
        rate_pid_.update_all(math::radians(desired_rate) * scaler * scaler, rate * scaler * scaler, in.dt, in.now_ms,
                              limit_i);

        if (underspeed) {
            // when underspeed we lock the integrator
            rate_pid_.set_integrator(old_i);
        }

        // FF and DFF should be scaled by scaler/eas2tas, but since we have
        // scaled the AcPid target above by scaler*scaler we need to
        // instead divide by scaler*eas2tas to get the right scaling
        const float ff = math::degrees(ff_scale_ * rate_pid_.get_ff_component() / (scaler * in.eas2tas));
        const float dff = math::degrees(ff_scale_ * rate_pid_.get_dff_component() / (scaler * in.eas2tas));
        ff_scale_ = 1.0f;

        if (disable_integrator) {
            rate_pid_.reset_i();
        }

        // convert AcPid info object to same scale as old controller
        pid_info_ = rate_pid_.get_pid_info();

        const float deg_scale = math::degrees(1.0f);
        pid_info_.ff = ff;
        pid_info_.p *= deg_scale;
        pid_info_.i *= deg_scale;
        pid_info_.d *= deg_scale;
        pid_info_.dff = dff;

        // fix the logged target and actual values to not have the
        // scalers applied
        pid_info_.target = desired_rate;
        pid_info_.actual = math::degrees(rate);

        // sum components
        float out = pid_info_.ff + pid_info_.p + pid_info_.i + pid_info_.d + pid_info_.dff;
        if (ground_mode) {
            // when on ground suppress D and half P term to prevent
            // oscillations
            out -= pid_info_.d + 0.5f * pid_info_.p;
        }

        // remember the last output to trigger the I limit
        last_out_ = out;

        // AP_AutoTune's `autotune->update(...)` call lived here upstream -
        // dropped, see file banner (AP_AUTOTUNE section).

        // output is scaled to notional centidegrees of deflection
        return math::constrain_value(out * 100.0f, -4500.0f, 4500.0f);
    }

    // upstream: AP_FW_Controller::reset_I() (renamed reset_i to match
    // AcPid's own lowercase naming, which this port's reset_i() call
    // below forwards into).
    void reset_i() { rate_pid_.reset_i(); }

    // upstream: AP_FW_Controller::decay_I() - see file banner for why
    // this is ported despite being VTOL-hover-only upstream.
    void decay_i() {
        // this reduces integrator by 95% over 2s
        pid_info_.i *= 0.995f;
        rate_pid_.set_integrator(rate_pid_.get_i() * 0.995f);
    }

    // upstream: AP_FW_Controller::set_ff_scale(). "setup a one loop FF
    // scale multiplier. This replaces any previous scale applied so
    // should only be used when only one source of scaling is needed"
    void set_ff_scale(float ff_scale) { ff_scale_ = ff_scale; }

    // upstream: AP_FW_Controller::get_pid_info()
    [[nodiscard]] const pid::PidInfo& get_pid_info() const { return pid_info_; }

    // upstream: AP_FW_Controller's kP()/kI()/kD()/kFF()/tau() accessors
    // returned AP_Float&; there is no AP_Float here (see ac_pid.hpp's own
    // kP()/kI()/... accessors), so this exposes the underlying AcPid
    // directly instead - a strict superset of what the AP_Float&
    // accessors let a caller do.
    [[nodiscard]] pid::AcPid& rate_pid() { return rate_pid_; }
    [[nodiscard]] const pid::AcPid& rate_pid() const { return rate_pid_; }

    // upstream's `_last_out` member (pre-centidegree-scale, unclamped
    // degrees) - exposed so YawController can reproduce the fact that
    // upstream's AP_YawController shares this EXACT field between its two
    // entry points (get_servo_out()'s own saturation-detection reads
    // whatever get_rate_out() last wrote here) - see yaw_controller.hpp.
    // Roll/PitchController have no second entry point that would ever
    // observe this, so they never call it.
    [[nodiscard]] float get_last_out() const { return last_out_; }

private:
    pid::AcPid rate_pid_;
    float ff_scale_ = 1.0f;
    float last_out_ = 0.0f;
    pid::PidInfo pid_info_;
};

} // namespace fwcpp::fw_control
