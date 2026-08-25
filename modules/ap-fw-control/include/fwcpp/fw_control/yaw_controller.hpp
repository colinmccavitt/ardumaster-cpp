#pragma once

// Port of APM_Control/AP_YawController.h + AP_YawController.cpp.
// CPP-032. Code by Jon Challinger, modified by Paul Riseborough to
// implement a three-loop autopilot topology. Upstream:
// libraries/APM_Control/AP_YawController.{h,cpp} (Plane-4.7.0,
// 79 + 417 lines) - read directly from the pinned upstream worktree in
// full before writing a line of this file, not from training-data memory.
//
// YAW'S REAL CONTRACT (verified by reading, not assumed - the task
// explicitly flagged this as something to check rather than guess): yaw
// does NOT mirror roll/pitch's "angle-error-in, rate-shaping, servo-out"
// shape at all. AP_YawController does not even inherit AP_FW_Controller
// upstream (it is a plain, unrelated class - confirmed: no
// `: public AP_FW_Controller` on its class line). It exposes TWO
// unrelated public entry points, both ported below:
//
//   1) get_servo_out(scaler, disable_integrator) -> int32_t: the REAL
//      normal-flight yaw controller. It is a sideslip-damping / turn-
//      coordination controller, not an angle-to-rate controller - it
//      takes NO angle error at all. It computes rudder deflection from
//      (a) lateral acceleration (_K_A * accel_y), (b) a high-pass-
//      filtered yaw-rate-relative-to-coordinated-turn-rate term
//      (_K_D * rate_hp_out, "yaw damping"), and (c) an integrator of (a)
//      folded into (b) (_K_I, "sideslip integrator", trims out steady-
//      state sideslip) - it does NOT use AC_PID/rate_pid at all; that
//      member exists on AP_YawController purely for entry point #2.
//      Ported below as get_servo_out(), taking a YawCoordinationInputs.
//
//   2) get_rate_out(desired_rate, scaler, disable_integrator) -> float:
//      a SEPARATE direct-rate-control entry point for aerobatic modes
//      (gated by the _RATE_ENABLE parameter / rate_control_enabled()) -
//      THIS one does use rate_pid, and its body is upstream's own
//      near-verbatim COPY of AP_FW_Controller::_get_rate_out() (same
//      radians()*scaler*scaler scaling, same underspeed-locks-integrator
//      logic, same ff/dff-by-scaler/eas2tas division) - duplicated
//      in-place upstream rather than shared, presumably because
//      AP_YawController predates or was never refactored to share
//      AP_FW_Controller's base. This port does NOT duplicate that logic a
//      second time: YawController composes the same FwController used by
//      Roll/PitchController (see fw_controller.hpp) for entry point #2,
//      since the two are byte-for-byte the same arithmetic modulo two
//      features yaw's own copy never used (ff_scale multiplier - yaw's
//      formula has no such factor, exactly reproduced by never calling
//      set_ff_scale() so FwController's ff_scale_ stays fixed at its
//      default 1.0; and ground_mode D/half-P suppression - yaw's own
//      get_rate_out has no ground_mode parameter at all, exactly
//      reproduced by always passing ground_mode=false).
//
// enabled()/rate_control_enabled(): ported as plain accessors over Gains
// fields (see below) - upstream's own definitions
// (`rate_control_enabled() || (_K_D > 0.0)` / `_rate_enable != 0`).
//
// AP_AUTOTUNE: entirely out of scope, same as fw_controller.hpp's banner.
// Dropped: `AP_AutoTune::ATGains gains`, `AP_AutoTune *autotune`,
// `bool failed_autotune_alloc`, autotune_start()/autotune_restore(), and
// get_rate_out()'s `autotune->update(...)` block (which also computed a
// synthetic `angle_err_deg = desired_rate * gains.tau` purely to feed
// autotune - dropped along with it, nothing else reads it).
//
// decay_I() PORTED, WITH A FAITHFULLY-REPRODUCED UPSTREAM ODDITY, NOT A
// BUG WE GET TO FIX (per this port's "port fixes bugs in the port, not
// upstream - register every divergence" policy: this is the reverse call,
// keeping the odd behavior because it is genuinely upstream's shipped
// behavior, not a discovered defect worth diverging over): unlike
// FwController::decay_i() (used by roll/pitch), upstream
// AP_YawController::decay_I() ONLY decays the LOGGED `_pid_info.I` value -
// it never touches the real `rate_pid` integrator at all (compare its
// two-line body, inline in AP_YawController.h, against
// AP_FW_Controller::decay_I()'s two DIFFERENT lines, one of which calls
// `rate_pid.set_integrator(...)`). Since decay_I() exists purely for
// "quadplane hover" scaling (upstream's own comment) and no quadplane
// exists in this port anyway, the divergence is functionally moot here -
// reproduced exactly as upstream defines it (decay_i() below touches only
// pid_info_.i) rather than "fixed" to match FwController's version.
//
// var_info[] DEFAULTS (AP_YawController::var_info, AP_YawController.cpp)
// transcribed into Gains below:
//   YAW2SRV_SLIP  -> k_a        (0.0f)
//   YAW2SRV_INT   -> k_i        (0.0f)
//   YAW2SRV_DAMP  -> k_d        (0.0f)
//   YAW2SRV_RLL   -> k_ff       (1.0f)
//   YAW2SRV_IMAX  -> imax_cd    (1500.0f, centidegrees of servo travel)
//   _RATE_ENABLE  -> rate_enable (false / AP_PARAM_FLAG_ENABLE default 0)
//   _RATE_P/_I/_D/_FF/_IMAX/_FLTT/_FLTE/_FLTD/_SMAX (AP_SUBGROUPINFO,
//   index 9) -> Gains::rate_pid - unlike roll/pitch, yaw's AC_PID is
//   constructed with an IN-CLASS DEFAULT MEMBER INITIALIZER, not an
//   AC_PID::Defaults constructor argument: `AC_PID rate_pid{0.04, 0.15,
//   0, 0.15, 0.666, 3, 0, 12, 150, 1};` (AP_YawController.h). Positional
//   order (checked against AC_PID's constructor / this port's
//   AcPid::Gains field order): p, i, d, ff, imax, filt_T_hz, filt_E_hz,
//   filt_D_hz, srmax, srtau -> p=0.04, i=0.15, d=0.0, ff=0.15 (NOT 0.345 -
//   yaw's default FF genuinely differs from roll/pitch's, transcribed
//   exactly, not assumed equal), imax=0.666, filt_T_hz=3.0, filt_E_hz=0.0,
//   filt_D_hz=12.0, srmax=150.0, srtau=1.0.
//
// TWO FIELDS UPSTREAM SHARES BETWEEN THE TWO ENTRY POINTS, both
// reproduced as genuinely shared state here rather than accidentally
// forked into two independent copies:
//   - `_pid_info` (this port: pid_info_) - see get_pid_info()'s own note.
//   - `_last_out` (this port: last_out_) - get_servo_out()'s own
//     saturation-detection (`if (_last_out < -45) ... else if (_last_out
//     > 45)`) reads whatever get_rate_out() most recently wrote, since
//     both entry points upstream write the SAME member. get_rate_out()
//     below syncs this controller's last_out_ from the composed
//     FwController base's own copy (FwController::get_last_out(), added
//     specifically for this) after every call, so a caller alternating
//     between get_servo_out() and get_rate_out() sees the identical
//     cross-talk upstream has.
//
// accel_y (get_servo_out's lateral-acceleration input): upstream reads
// `AP::ins().get_accel().y` then immediately subtracts
// `_ahrs.get_accel_bias().y`. No AP_InertialSensor/AP_AHRS singleton
// exists in this port's public interface here (ADR-0012) - the caller is
// expected to supply the ALREADY BIAS-CORRECTED scalar via
// YawCoordinationInputs::accel_y, exactly the value upstream computes
// right before using it, not a step earlier.
//
// reset_rate_PID(): ported (`rate_pid.reset_I(); rate_pid.reset_filter();`
// via the composed FwController's rate_pid() accessor) - a real, simple,
// independently-useful public member with no VTOL/autotune coupling.
//
// LITERAL SAFETY: no bare ambiguous double literals - 1.5707964f/
// 1.3962634f (upstream's own hand-rounded +-90deg/+-80deg radian
// constants, reproduced digit-for-digit rather than substituted with
// math::radians(90.0f)/radians(80.0f) - upstream itself does not call
// radians() at these two call sites, using the literal directly, so this
// port matches the literal exactly for bit-for-bit parity), 0.9960080f,
// 0.0001f, 45.0f, 1.0f, all explicitly float-suffixed.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/fw_control/fw_controller.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::fw_control {

// Everything AP_YawController::get_servo_out() (the sideslip/turn-
// coordination controller - entry point #1, see file banner) needs per
// tick that upstream read from AP::ahrs()/AP::ins() singletons.
struct YawCoordinationInputs {
    float bank_angle_rad = 0.0f;  // upstream: AP::ahrs().get_roll_rad()
    float gyro_z = 0.0f;          // rad/s. Upstream: AP::ahrs().get_gyro().z
    float accel_y = 0.0f;         // m/s^2, bias-corrected - see file banner
    bool airspeed_valid = false;  // upstream: AP::ahrs().airspeed_EAS() return value
    float airspeed_eas = 0.0f;    // m/s EAS, meaningful only if airspeed_valid
    std::uint32_t now_ms = 0;     // upstream: AP_HAL::millis(), for the dt/reset-detection logic below
};

class YawController {
public:
    struct Gains {
        float k_a = 0.0f;          // YAW2SRV_SLIP, sideslip control gain
        float k_i = 0.0f;          // YAW2SRV_INT, sideslip integrator gain
        float k_d = 0.0f;          // YAW2SRV_DAMP, yaw damping gain
        float k_ff = 1.0f;         // YAW2SRV_RLL, turn-coordination gain
        float imax_cd = 1500.0f;   // YAW2SRV_IMAX, centidegrees of servo travel
        bool rate_enable = false;  // _RATE_ENABLE, enables get_rate_out() for aerobatics
        pid::AcPid::Gains rate_pid{
            .p = 0.04f, .i = 0.15f, .d = 0.0f, .ff = 0.15f, .imax = 0.666f, .filt_t_hz = 3.0f,
            .filt_e_hz = 0.0f,      .filt_d_hz = 12.0f, .srmax = 150.0f, .srtau = 1.0f,
        };
    };

    YawController(const Gains& gains, const FwAparm& aparm) : base_(gains.rate_pid), gains_(gains), aparm_(aparm) {
        // upstream: AP_YawController's constructor,
        // `rate_pid.set_slew_limit_scale(45);` - identical call to
        // AP_FW_Controller's own constructor (see fw_controller.hpp);
        // yaw duplicates it because it does not inherit that base.
        base_.rate_pid().set_slew_limit_scale(45);
        // upstream also initializes `_pid_info.target = 0; _pid_info.FF =
        // 0; _pid_info.P = 0;` in the constructor body - a no-op here,
        // since pid::PidInfo's own default member initializers already
        // zero every field (see ac_pid.hpp).
    }

    YawController(const YawController&) = delete;
    YawController& operator=(const YawController&) = delete;

    // upstream: return true if rate control OR damping is enabled
    [[nodiscard]] bool enabled() const { return rate_control_enabled() || gains_.k_d > 0.0f; }
    // upstream: return true if rate control is enabled
    [[nodiscard]] bool rate_control_enabled() const { return gains_.rate_enable; }

    // upstream: AP_YawController::get_servo_out(scaler,
    // disable_integrator) - entry point #1, see file banner. Returns a
    // rudder deflection in centi-degrees, +ve deflection yaws nose right.
    std::int32_t get_servo_out(float scaler, bool disable_integrator, const YawCoordinationInputs& in) {
        std::uint32_t dt_ms;
        if (last_t_ms_ == 0 || (in.now_ms - last_t_ms_) > 1000) {
            dt_ms = 0;
            pid_info_.i = 0.0f;
        } else {
            dt_ms = in.now_ms - last_t_ms_;
        }
        last_t_ms_ = in.now_ms;

        float aspd_min = aparm_.airspeed_min;
        if (aspd_min < 1.0f) {
            aspd_min = 1.0f;
        }

        const float delta_time = static_cast<float>(dt_ms) * 0.001f;

        // Calculate yaw rate required to keep up with a constant height
        // coordinated turn
        float bank_angle = in.bank_angle_rad;
        // limit bank angle between +- 80 deg if right way up (see file
        // banner: these two constants are upstream's own hand-written
        // literals, not radians(90)/radians(80))
        if (std::fabs(bank_angle) < 1.5707964f) {
            bank_angle = math::constrain_value(bank_angle, -1.3962634f, 1.3962634f);
        }

        const float aspeed = in.airspeed_valid ? in.airspeed_eas : 0.5f * (aspd_min + aparm_.airspeed_max);

        const float rate_offset = (kGravityMss / std::max(aspeed, aspd_min)) * std::sin(bank_angle) * gains_.k_ff;

        // Get body rate vector (radians/sec) and lateral acceleration
        // (m/s^2, already bias-corrected - see file banner)
        const float omega_z = in.gyro_z;
        const float accel_y = in.accel_y;

        // Subtract the steady turn component of rate from the measured
        // rate to calculate the rate relative to the turn requirement in
        // degrees/sec
        const float rate_hp_in = math::degrees(omega_z - rate_offset);

        // Apply a high-pass filter to the rate to washout any steady
        // state error due to bias errors in rate_offset. Cut-off
        // frequency omega = 0.2 rad/sec.
        const float rate_hp_out = 0.9960080f * last_rate_hp_out_ + rate_hp_in - last_rate_hp_in_;
        last_rate_hp_out_ = rate_hp_out;
        last_rate_hp_in_ = rate_hp_in;

        // Calculate input to integrator
        const float integ_in = -gains_.k_i * (gains_.k_a * accel_y + rate_hp_out);

        // Apply integrator, but clamp input to prevent control saturation
        // and freeze integrator below min FBW speed. Don't integrate if
        // disable_integrator is set (e.g. stabilise mode, integrator
        // would wind up against pilot input) or if k_d is zero (would
        // keep winding up with nothing damping it back down).
        if (!disable_integrator && gains_.k_d > 0.0f) {
            if (aspeed > aspd_min) {
                if (last_out_ < -45.0f) {
                    // prevent the integrator from increasing if surface
                    // defln demand is above the upper limit
                    integrator_ += std::max(integ_in * delta_time, 0.0f);
                } else if (last_out_ > 45.0f) {
                    // prevent the integrator from decreasing if surface
                    // defln demand is below the lower limit
                    integrator_ += std::min(integ_in * delta_time, 0.0f);
                } else {
                    integrator_ += integ_in * delta_time;
                }
            }
        } else {
            integrator_ = 0.0f;
        }

        if (gains_.k_d < 0.0001f) {
            // yaw damping is disabled, and the integrator is scaled by
            // damping, so return 0
            return 0;
        }

        // Scale the integration limit
        const float int_lim_scaled = gains_.imax_cd * 0.01f / (gains_.k_d * scaler * scaler);

        // Constrain the integrator state
        integrator_ = math::constrain_value(integrator_, -int_lim_scaled, int_lim_scaled);

        // Protect against increases to k_d during in-flight tuning from
        // creating large control transients due to stored integrator
        // values
        if (gains_.k_d > k_d_last_ && gains_.k_d > 0.0f) {
            integrator_ = k_d_last_ / gains_.k_d * integrator_;
        }
        k_d_last_ = gains_.k_d;

        // Calculate demanded rudder deflection, +ve deflection yaws nose
        // right. Save to last value before application of limiter so
        // that integrator limiting can detect exceedance next frame.
        // Scale using inverse dynamic pressure (1/V^2).
        pid_info_.i = gains_.k_d * integrator_ * scaler * scaler;
        pid_info_.d = gains_.k_d * (-rate_hp_out) * scaler * scaler;
        last_out_ = pid_info_.i + pid_info_.d;

        return static_cast<std::int32_t>(math::constrain_value(last_out_ * 100.0f, -4500.0f, 4500.0f));
    }

    // upstream: AP_YawController::get_rate_out(desired_rate, scaler,
    // disable_integrator) - entry point #2, see file banner (direct-
    // rate-control for aerobatic modes; shares FwController's rate loop
    // rather than duplicating it a second time). Syncs this controller's
    // own pid_info_ from the composed base afterwards - see get_pid_info()'s
    // note on why a single shared struct (matching upstream's one
    // `_pid_info` member) rather than reading straight from base_ here.
    float get_rate_out(float desired_rate, float scaler, bool disable_integrator, const RateLoopInputs& in) {
        const bool underspeed = in.airspeed <= aparm_.airspeed_min;
        const float out = base_.get_rate_out_full(desired_rate, scaler, disable_integrator, underspeed,
                                                    /*ground_mode=*/false, in);
        pid_info_ = base_.get_pid_info();
        // upstream shares ONE `_last_out` field between both entry points
        // (get_servo_out()'s own `if (_last_out < -45)`/`> 45` saturation
        // check reads whatever this call last wrote) - reproduced via
        // FwController::get_last_out() rather than letting this
        // controller's own last_out_ (written only by get_servo_out())
        // silently diverge from base_'s.
        last_out_ = base_.get_last_out();
        return out;
    }

    // upstream: AP_YawController::reset_I()
    void reset_I() {
        pid_info_.i = 0.0f;
        base_.reset_i();
        integrator_ = 0.0f;
    }

    // upstream: AP_YawController::reset_rate_PID()
    void reset_rate_PID() {
        base_.rate_pid().reset_i();
        base_.rate_pid().reset_filter();
    }

    // Exposes the AC_PID that backs get_rate_out() (entry point #2) - the
    // real rate integrator that decay_I()'s documented quirk (below)
    // deliberately does NOT touch, distinct from pid_info_.i (the logged
    // value get_servo_out(), entry point #1, actually mutates). Same
    // rationale as RollController::rate_pid()/PitchController::rate_pid().
    [[nodiscard]] pid::AcPid& rate_pid() { return base_.rate_pid(); }

    // upstream: AP_YawController::decay_I() - see file banner for the
    // faithfully-reproduced divergence from FwController::decay_i(): only
    // the logged pid_info_.i is decayed, the real rate_pid integrator
    // (only ever touched by get_rate_out(), entry point #2) is untouched.
    void decay_I() { pid_info_.i *= 0.995f; }

    // upstream: AP_YawController::get_pid_info(). Backed by a SINGLE
    // pid::PidInfo member, matching upstream's single shared `_pid_info`:
    // get_rate_out() (entry point #2) overwrites the whole struct each
    // call (via base_'s identical computation, synced in above);
    // get_servo_out() (entry point #1) mutates only its `.i`/`.d` fields
    // directly, leaving `.target`/`.ff`/`.p`/`.dff` at whatever
    // get_rate_out() last left them (or their zero defaults if
    // get_rate_out() was never called) - exactly upstream's own
    // last-write-wins-per-field behavior on one shared struct.
    [[nodiscard]] const pid::PidInfo& get_pid_info() const { return pid_info_; }

private:
    FwController base_;
    Gains gains_;
    FwAparm aparm_;

    std::uint32_t last_t_ms_ = 0;
    float last_out_ = 0.0f;
    float last_rate_hp_out_ = 0.0f;
    float last_rate_hp_in_ = 0.0f;
    float k_d_last_ = 0.0f;
    float integrator_ = 0.0f;

    // upstream's single shared `_pid_info` member - see get_pid_info()'s
    // own note.
    pid::PidInfo pid_info_;
};

} // namespace fwcpp::fw_control
