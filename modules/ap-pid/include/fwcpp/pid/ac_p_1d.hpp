#pragma once

#include <algorithm>
// Port of AC_P_1D — CCP-027 slice 3 (D position P loop). Rust: ap-pid/p_1d.rs.

#include <fwcpp/math/control.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/scalar.hpp>

namespace fwcpp::pid {

class AcP1d {
public:
    float kp = 0.0f;

    [[nodiscard]] static AcP1d with_kp(float initial_p) {
        AcP1d p;
        p.kp = initial_p;
        return p;
    }

    [[nodiscard]] float error() const { return error_; }
    [[nodiscard]] float error_min() const { return error_min_; }
    [[nodiscard]] float error_max() const { return error_max_; }
    [[nodiscard]] float d1_max() const { return d1_max_; }

    void set_limits(float output_min, float output_max, float d_out_max, float d2_out_max) {
        d1_max_ = 0.0f;
        error_min_ = 0.0f;
        error_max_ = 0.0f;

        if (math::is_positive(d_out_max)) {
            d1_max_ = d_out_max;
        }

        if (math::is_positive(d2_out_max) && math::is_positive(kp)) {
            d1_max_ = std::min(d1_max_, d2_out_max / kp);
        }

        if (math::is_negative(output_min) && math::is_positive(kp)) {
            error_min_ = math::inv_sqrt_controller(output_min, kp, d1_max_);
        }
        if (math::is_positive(output_max) && math::is_positive(kp)) {
            error_max_ = math::inv_sqrt_controller(output_max, kp, d1_max_);
        }
    }

    void set_error_limits(float error_min, float error_max) {
        if (math::is_negative(error_min)) {
            if (!math::is_zero(error_min_)) {
                error_min_ = std::max(error_min_, error_min);
            } else {
                error_min_ = error_min;
            }
        }
        if (math::is_positive(error_max)) {
            if (!math::is_zero(error_max_)) {
                error_max_ = std::min(error_max_, error_max);
            } else {
                error_max_ = error_max;
            }
        }
    }

    [[nodiscard]] float update_all(math::postype_t& target, math::postype_t measurement) {
        error_ = static_cast<float>(target - measurement);

        if (math::is_negative(error_min_) && error_ < error_min_) {
            error_ = error_min_;
            target = measurement + math::postype_t{error_};
        } else if (math::is_positive(error_max_) && error_ > error_max_) {
            error_ = error_max_;
            target = measurement + math::postype_t{error_};
        }

        return math::sqrt_controller(error_, kp, d1_max_, 0.0f);
    }

private:
    float error_ = 0.0f;
    float error_min_ = 0.0f;
    float error_max_ = 0.0f;
    float d1_max_ = 0.0f;
};

}  // namespace fwcpp::pid
