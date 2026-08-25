#pragma once

// Port of AC_PID/AC_PID.h + AC_PID.cpp. CPP-016, slice 1.
//
// AP_Float REPLACED WITH PLAIN float: every gain upstream stores as
// AP_Float (EEPROM-backed, live-editable via GCS) is a plain float member
// here. AP_Param does not exist in this port yet. This is not a
// speculative choice - it is the exact precedent the Rust port's own
// controllers (PidGains, RateGains, YawGains, SteerGains) already
// established for the identical problem, before AP_Param existed there
// either: take gains as plain values owned by the caller. When AP_Param
// eventually lands in this port, wiring it in is a caller-side concern
// (whoever owns the AC_PID instance decides whether its floats are backed
// by parameters), not a change to AC_PID's own arithmetic.
//
// AP_HAL::millis() REPLACED WITH AN EXPLICIT now_ms PARAMETER on
// update_all(), threading through to the embedded SlewLimiter's own
// modifier() call - same reasoning as SlewLimiter's own file banner.
//
// NOTCH FILTERS STILL NOT INCLUDED, and this is now the one deliberately
// remaining gap (as of CPP-016 slice 2): unlike Filter/NotchFilter.h itself
// (ported as fwcpp::filter::NotchFilter, CPP-018, done), AC_PID's notch
// integration doesn't just construct a NotchFilter - it looks one up at
// runtime by index through AP_Filter's global registry (`AP::filters()`,
// a singleton returning a filter definition the user configured via GCS
// parameters). Wiring that in means either porting AP_Filter's registry
// (a real subsystem, its own ticket) or inventing a different lookup
// mechanism upstream doesn't have - neither is a small addition to bolt
// onto this file. set_notch_sample_rate and the AP_FILTER_ENABLED members
// remain absent; every other function in this slice reproduces the
// "notch filters not configured" behavior exactly, since upstream's own
// notch application is a no-op whenever the pointer is null.
//
// load_gains/save_gains/var_info NOT PORTED: EEPROM/AP_Param-specific,
// meaningless without AP_Param.
//
// LITERAL SAFETY: no bare ambiguous double literals in the ported
// functions - constrain_float's contribution to AC_PID_RESET_TC (0.16f)
// and similar constants are all explicitly float-suffixed upstream.

#include <cmath>
#include <cstdint>

#include <fwcpp/filter/slew_limiter.hpp>
#include <fwcpp/internal_error.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::pid {

struct PidInfo {
    float target = 0.0f;
    float actual = 0.0f;
    float error = 0.0f;
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;
    float ff = 0.0f;
    float dff = 0.0f;
    float dmod = 0.0f;
    float slew_rate = 0.0f;
    bool limit = false;
    bool pd_limit = false;
    bool reset = false;
    bool i_term_set = false;
};

class AcPid {
public:
    struct Gains {
        float p = 0.0f;
        float i = 0.0f;
        float d = 0.0f;
        float ff = 0.0f;
        float imax = 0.0f;
        float filt_t_hz = 0.0f; // AC_PID_TFILT_HZ_DEFAULT upstream: 0
        float filt_e_hz = 0.0f;
        float filt_d_hz = 20.0f; // AC_PID_DFILT_HZ_DEFAULT upstream
        float srmax = 0.0f;
        float srtau = 1.0f;
        float dff = 0.0f;
    };

    explicit AcPid(const Gains& g)
        : kp_(g.p), ki_(g.i), kd_(g.d), kff_(g.ff), kimax_(g.imax),
          filt_t_hz_(g.filt_t_hz), filt_e_hz_(g.filt_e_hz), filt_d_hz_(g.filt_d_hz),
          slew_rate_max_(g.srmax), slew_rate_tau_(g.srtau), kdff_(g.dff),
          slew_limiter_(slew_rate_max_, slew_rate_tau_) {
        flags_reset_filter_ = true;
    }

    AcPid(const AcPid&) = delete;
    AcPid& operator=(const AcPid&) = delete;

    // now_ms: see file banner (replaces upstream's internal AP_HAL::millis()
    // call inside the slew limiter).
    float update_all(float target, float measurement, float dt, std::uint32_t now_ms,
                      bool limit = false, float pd_scale = 1.0f, float i_scale = 1.0f) {
        if (!std::isfinite(target) || !std::isfinite(measurement)) {
            return 0.0f;
        }

        pid_info_.reset = flags_reset_filter_;
        if (flags_reset_filter_) {
            flags_reset_filter_ = false;
            target_ = target;
            error_ = target_ - measurement;
            derivative_ = 0.0f;
            target_derivative_ = 0.0f;
        } else {
            const float target_last = target_;
            target_ += math::calc_lowpass_alpha_dt(dt, filt_t_hz_) * (target - target_);

            const float error_last = error_;
            const float error = target_ - measurement;
            error_ += math::calc_lowpass_alpha_dt(dt, filt_e_hz_) * (error - error_);

            if (math::is_positive(dt)) {
                const float derivative = (error_ - error_last) / dt;
                derivative_ += math::calc_lowpass_alpha_dt(dt, filt_d_hz_) * (derivative - derivative_);
                target_derivative_ = (target_ - target_last) / dt;
            }
        }

        update_i(dt, limit, i_scale);

        float p_out = error_ * kp_;
        float d_out = derivative_ * kd_;
        const float i_out = integrator_;

        pid_info_.dmod = slew_limiter_.modifier((pid_info_.p + pid_info_.d) * slew_limit_scale_, dt, now_ms);
        pid_info_.slew_rate = slew_limiter_.get_slew_rate();

        p_out *= pid_info_.dmod;
        d_out *= pid_info_.dmod;

        p_out *= pd_scale;
        d_out *= pd_scale;

        pid_info_.pd_limit = false;
        if (math::is_positive(kpdmax_)) {
            const float pd_sum_abs = std::fabs(p_out + d_out);
            if (pd_sum_abs > kpdmax_) {
                const float pd_limit_scale = kpdmax_ / pd_sum_abs;
                p_out *= pd_limit_scale;
                d_out *= pd_limit_scale;
                pid_info_.pd_limit = true;
            }
        }

        pid_info_.target = target_;
        pid_info_.actual = measurement;
        pid_info_.error = error_;
        pid_info_.p = p_out;
        pid_info_.d = d_out;
        pid_info_.i = i_out;
        pid_info_.limit = limit;
        pid_info_.i_term_set = flags_i_set_;
        flags_i_set_ = false;
        pid_info_.ff = target_ * kff_;
        pid_info_.dff = target_derivative_ * kdff_;

        return p_out + d_out + i_out;
    }

    // Computes the PID output from an error input only (target assumed
    // zero). Reuses update_all's code path exactly as upstream does:
    // target is force-set to 0 (bypassing the target filter, matching
    // upstream's direct `_target = 0.0` before the call), then update_all
    // runs with measurement = -error so that its own `target - measurement`
    // reproduces `error`. now_ms: see update_all's own note.
    float update_error(float error, float dt, std::uint32_t now_ms, bool limit = false) {
        if (!std::isfinite(error)) {
            return 0.0f;
        }
        target_ = 0.0f;
        const float output = update_all(0.0f, -error, dt, now_ms, limit);
        pid_info_.target = 0.0f;
        pid_info_.actual = 0.0f;
        return output;
    }

    void reset_filter() { flags_reset_filter_ = true; }

    void reset_i() {
        flags_i_set_ = true;
        integrator_ = 0.0f;
    }

    void set_integrator(float integrator, InternalError* err = nullptr, std::uint16_t line = 0) {
        flags_i_set_ = true;
        integrator_ = math::constrain_value(integrator, -kimax_, kimax_, err, line);
    }

    void relax_integrator(float integrator, float dt, float time_constant,
                           InternalError* err = nullptr, std::uint16_t line = 0) {
        integrator = math::constrain_value(integrator, -kimax_, kimax_, err, line);
        if (math::is_positive(dt)) {
            flags_i_set_ = true;
            integrator_ += (integrator - integrator_) * (dt / (dt + time_constant));
        }
    }

    void set_slew_limit_scale(std::int8_t scale) { slew_limit_scale_ = scale; }

    [[nodiscard]] float get_p() const { return pid_info_.p; }
    [[nodiscard]] float get_i() const { return integrator_; }
    [[nodiscard]] float get_d() const { return pid_info_.d; }
    [[nodiscard]] float get_ff() const { return pid_info_.ff + pid_info_.dff; }
    [[nodiscard]] float get_ff_component() const { return pid_info_.ff; }
    [[nodiscard]] float get_dff_component() const { return pid_info_.dff; }
    [[nodiscard]] float get_slew_rate() const { return slew_limiter_.get_slew_rate(); }
    [[nodiscard]] const PidInfo& get_pid_info() const { return pid_info_; }

    [[nodiscard]] float imax() const { return kimax_; }
    [[nodiscard]] float pdmax() const { return kpdmax_; }

    // Returns the low-pass filter alpha for the given dt at each filter's
    // configured frequency - exposed separately from update_all so callers
    // can inspect/log the effective filtering without running a full step.
    [[nodiscard]] float get_filt_t_alpha(float dt) const { return math::calc_lowpass_alpha_dt(dt, filt_t_hz_); }
    [[nodiscard]] float get_filt_e_alpha(float dt) const { return math::calc_lowpass_alpha_dt(dt, filt_e_hz_); }
    [[nodiscard]] float get_filt_d_alpha(float dt) const { return math::calc_lowpass_alpha_dt(dt, filt_d_hz_); }

    void set_target_rate(float target) { pid_info_.target = target; }
    void set_actual_rate(float actual) { pid_info_.actual = actual; }

    // gain accessors, matching upstream's kP()/kI()/... shape but returning
    // plain float& (there is no AP_Float to return a reference into)
    float& kP() { return kp_; }
    float& kI() { return ki_; }
    float& kD() { return kd_; }
    float& kIMAX() { return kimax_; }
    float& kPDMAX() { return kpdmax_; }
    float& ff() { return kff_; }
    float& kDff() { return kdff_; }
    float& filt_T_hz() { return filt_t_hz_; }
    float& filt_E_hz() { return filt_e_hz_; }
    float& filt_D_hz() { return filt_d_hz_; }
    float& slew_limit() { return slew_rate_max_; }

    // set accessors - upstream's own set_* family. abs()-clamped where
    // upstream's set_kX(v) calls fabsf(v) before storing (imax/pdmax and
    // every filter/slew frequency are magnitude-only quantities).
    void set_kP(float v) { kp_ = v; }
    void set_kI(float v) { ki_ = v; }
    void set_kD(float v) { kd_ = v; }
    void set_ff(float v) { kff_ = v; }
    void set_imax(float v) { kimax_ = std::fabs(v); }
    void set_pdmax(float v) { kpdmax_ = std::fabs(v); }
    void set_filt_T_hz(float v) { filt_t_hz_ = std::fabs(v); }
    void set_filt_E_hz(float v) { filt_e_hz_ = std::fabs(v); }
    void set_filt_D_hz(float v) { filt_d_hz_ = std::fabs(v); }
    void set_slew_limit(float v) { slew_rate_max_ = std::fabs(v); }
    void set_kDff(float v) { kdff_ = v; }

private:
    void update_i(float dt, bool limit, float i_scale) {
        if (!math::is_zero(ki_) && math::is_positive(dt)) {
            if (!limit
                || (math::is_positive(integrator_) && math::is_negative(error_))
                || (math::is_negative(integrator_) && math::is_positive(error_))) {
                integrator_ += (error_ * ki_) * i_scale * dt;
                integrator_ = math::constrain_value(integrator_, -kimax_, kimax_);
            }
        } else {
            integrator_ = 0.0f;
        }
    }

    float kp_;
    float ki_;
    float kd_;
    float kff_;
    float kimax_;
    float filt_t_hz_;
    float filt_e_hz_;
    float filt_d_hz_;
    float slew_rate_max_;
    float slew_rate_tau_;
    float kdff_;
    float kpdmax_ = 0.0f; // AC_PID_PDMAX has no constructor param upstream - defaults 0 (disabled)

    filter::SlewLimiter slew_limiter_; // must be declared after the rate_max/tau it references

    bool flags_reset_filter_ = false;
    bool flags_i_set_ = false;

    float integrator_ = 0.0f;
    float target_ = 0.0f;
    float error_ = 0.0f;
    float derivative_ = 0.0f;
    float target_derivative_ = 0.0f;
    std::int8_t slew_limit_scale_ = 1;

    PidInfo pid_info_;
};

} // namespace fwcpp::pid
