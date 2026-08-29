#pragma once

// Port of AC_PID_Basic — CCP-027 slice 3 (D velocity loop). Rust: ap-pid/pid_basic.rs.

#include <cmath>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/pid/ac_pid.hpp>

namespace fwcpp::pid {

class AcPidBasic {
public:
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float kff = 0.0f;
    float imax = 0.0f;
    float filt_e_hz = 0.0f;
    float filt_d_hz = 0.0f;

    [[nodiscard]] static AcPidBasic with_gains(float kp_in, float ki_in, float kd_in, float kff_in,
                                               float imax_in, float filt_e_hz_in,
                                               float filt_d_hz_in) {
        AcPidBasic p;
        p.kp = kp_in;
        p.ki = ki_in;
        p.kd = kd_in;
        p.kff = kff_in;
        p.imax = std::fabs(imax_in);
        p.filt_e_hz = filt_e_hz_in;
        p.filt_d_hz = filt_d_hz_in;
        p.reset_filter_ = true;
        return p;
    }

    [[nodiscard]] float error() const { return error_; }
    [[nodiscard]] float integrator() const { return integrator_; }
    [[nodiscard]] const PidInfo& info() const { return info_; }

    void reset_filter() { reset_filter_ = true; }
    void reset_i() { integrator_ = 0.0f; }

    void set_integrator(float value) {
        integrator_ = math::constrain_value(value, -imax, imax);
    }

    [[nodiscard]] float update_all(float target, float measurement, float dt, bool limit_neg,
                                 bool limit_pos) {
        if (!std::isfinite(target) || !std::isfinite(measurement)) {
            return 0.0f;
        }

        target_ = target;

        if (reset_filter_) {
            reset_filter_ = false;
            error_ = target_ - measurement;
            derivative_ = 0.0f;
        } else {
            const float error_last = error_;
            error_ += ((target_ - measurement) - error_) * filt_e_alpha(dt);
            if (math::is_positive(dt)) {
                const float derivative = (error_ - error_last) / dt;
                derivative_ += (derivative - derivative_) * filt_d_alpha(dt);
            }
        }

        update_i(dt, limit_neg, limit_pos);

        const float p_out = error_ * kp;
        const float d_out = derivative_ * kd;
        const float ff_out = target_ * kff;

        info_.target = target_;
        info_.actual = measurement;
        info_.error = error_;
        info_.p = p_out;
        info_.i = integrator_;
        info_.d = d_out;
        info_.ff = ff_out;

        return p_out + integrator_ + d_out + ff_out;
    }

private:
    float target_ = 0.0f;
    float error_ = 0.0f;
    float derivative_ = 0.0f;
    float integrator_ = 0.0f;
    bool reset_filter_ = true;
    PidInfo info_{};

    void update_i(float dt, bool limit_neg, bool limit_pos) {
        if (!math::is_zero(ki)) {
            const bool freeze = (limit_neg && math::is_negative(error_)) ||
                                (limit_pos && math::is_positive(error_));
            if (!freeze) {
                integrator_ += error_ * ki * dt;
                integrator_ = math::constrain_value(integrator_, -imax, imax);
            }
        } else {
            integrator_ = 0.0f;
        }
    }

    [[nodiscard]] float filt_e_alpha(float dt) const {
        return math::calc_lowpass_alpha_dt(dt, filt_e_hz);
    }

    [[nodiscard]] float filt_d_alpha(float dt) const {
        return math::calc_lowpass_alpha_dt(dt, filt_d_hz);
    }
};

}  // namespace fwcpp::pid
