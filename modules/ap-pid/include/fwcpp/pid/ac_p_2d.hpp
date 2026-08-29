#pragma once

#include <algorithm>
// Port of AC_P_2D — CCP-027 slice 2 (NE position P loop). Rust: ap-pid/p_2d.rs.

#include <fwcpp/math/control_vector.hpp>
#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>

namespace fwcpp::pid {

class AcP2d {
public:
    float kp = 0.0f;

    [[nodiscard]] static AcP2d with_kp(float initial_p) {
        AcP2d p;
        p.kp = initial_p;
        return p;
    }

    [[nodiscard]] math::Vector2f error() const { return error_; }
    [[nodiscard]] float error_max() const { return error_max_; }
    [[nodiscard]] float d1_max() const { return d1_max_; }

    void set_limits(float output_max, float d_out_max, float d2_out_max) {
        d1_max_ = 0.0f;
        error_max_ = 0.0f;

        if (math::is_positive(d_out_max)) {
            d1_max_ = d_out_max;
        }

        if (math::is_positive(d2_out_max) && math::is_positive(kp)) {
            d1_max_ = std::min(d1_max_, d2_out_max / kp);
        }

        if (math::is_positive(output_max) && math::is_positive(kp)) {
            error_max_ = math::inv_sqrt_controller(output_max, kp, d1_max_);
        }
    }

    void set_error_max(float error_max) {
        if (math::is_positive(error_max)) {
            if (!math::is_zero(error_max_)) {
                error_max_ = std::min(error_max_, error_max);
            } else {
                error_max_ = error_max;
            }
        }
    }

    [[nodiscard]] math::Vector2f update_all(math::Vector2<math::postype_t>& target,
                                            math::Vector2<math::postype_t> measurement) {
        error_ = math::Vector2f{
            static_cast<float>(target.x - measurement.x),
            static_cast<float>(target.y - measurement.y),
        };

        if (math::is_positive(error_max_) && error_.limit_length(error_max_)) {
            target.x = measurement.x + math::postype_t{error_.x};
            target.y = measurement.y + math::postype_t{error_.y};
        }

        return math::sqrt_controller_xy(error_, kp, d1_max_, 0.0f);
    }

private:
    math::Vector2f error_{};
    float error_max_ = 0.0f;
    float d1_max_ = 0.0f;
};

}  // namespace fwcpp::pid
