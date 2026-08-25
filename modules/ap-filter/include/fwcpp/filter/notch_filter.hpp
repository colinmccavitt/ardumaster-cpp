#pragma once

// Port of Filter/NotchFilter.h + NotchFilter.cpp. CPP-018. Design by
// Leonard Hall. NotchFilterParams (the AP_Param-backed enable/frequency/
// bandwidth/attenuation bundle) NOT ported - AP_Param-specific, same
// reasoning as AC_PID's skipped var_info.
//
// D-015 found and FIXED (fourth occurrence of this bug class in this
// port's own reading of the Filter library - D-013 in DigitalBiquadFilter,
// D-014 in DigitalLPF - matching the Rust port's own D-005 covering this
// same four-class file family, found independently again): upstream's
// NotchFilter has NO CONSTRUCTOR AT ALL, so `initialised`, `need_reset`,
// and the filter coefficients/delay elements are indeterminate on a fresh
// instance. apply()'s branch is `if (!initialised || need_reset)`, so
// `initialised` being false alone is sufficient to force the safe
// passthrough-and-seed path regardless of need_reset's value - but this
// port initializes both explicitly rather than leaning on that, since
// "one of two garbage flags happens not to matter given how the OR
// evaluates" is not a property worth depending on.
//
// LITERAL SAFETY: init_with_A_and_Q's `omega = 2.0 * M_PI * ...` is the
// only ambiguous-literal spot upstream has here. Unlike scalar.cpp's
// wrap_* family or Location::get_bearing, this does NOT need its own
// compiled .cpp: math::pi_constant() (scalar.cpp, exposed for exactly
// this) is a function CALL, not a bare literal token, so its result is
// fixed at scalar.cpp's own compile time regardless of what flags the
// calling translation unit has - no per-TU parsing ambiguity, no ODR risk
// from calling it in a header. (In hindsight, Location::get_bearing could
// have used the same approach and stayed header-only too; moving it to a
// .cpp there was not wrong, just not the minimal fix once pi_constant()
// existed - noted here so the next M_PI-needing function picks the
// simpler path first.)

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::filter {

template <typename T>
class NotchFilter {
public:
    NotchFilter() = default; // see D-015 above: initialised_/need_reset_ have in-class defaults

    NotchFilter(const NotchFilter&) = delete;
    NotchFilter& operator=(const NotchFilter&) = delete;

    static void calculate_a_and_q(float center_freq_hz, float bandwidth_hz, float attenuation_db, float& a, float& q) {
        a = std::pow(10.0f, -attenuation_db / 40.0f);
        if (center_freq_hz > 0.5f * bandwidth_hz) {
            const float octaves = std::log2(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0f)) * 2.0f;
            q = std::sqrt(std::pow(2.0f, octaves)) / (std::pow(2.0f, octaves) - 1.0f);
        } else {
            q = 0.0f;
        }
    }

    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_db) {
        initialised_ = false;
        if (center_freq_hz > 0.5f * bandwidth_hz && center_freq_hz < 0.5f * sample_freq_hz) {
            float a, q;
            calculate_a_and_q(center_freq_hz, bandwidth_hz, attenuation_db, a, q);
            init_with_a_and_q(sample_freq_hz, center_freq_hz, a, q);
        }
    }

    void init_with_a_and_q(float sample_freq_hz, float center_freq_hz, float a, float q) {
        if (initialised_ && math::is_equal(center_freq_hz, center_freq_hz_)
            && math::is_equal(sample_freq_hz, sample_freq_hz_) && math::is_equal(a, a_)) {
            return;
        }

        float new_center_freq = center_freq_hz;

        if (initialised_ && !need_reset_ && !math::is_zero(center_freq_hz_)) {
            new_center_freq = math::constrain_value(
                new_center_freq, center_freq_hz_ * kNotchMaxSlewLower, center_freq_hz_ * kNotchMaxSlewUpper);
        }

        if (math::is_positive(new_center_freq) && new_center_freq < 0.5f * sample_freq_hz && q > 0.0f) {
            const float omega = 2.0f * static_cast<float>(math::pi_constant()) * new_center_freq / sample_freq_hz;
            const float alpha = std::sin(omega) / (2.0f * q);
            b0_ = 1.0f + alpha * (a * a);
            b1_ = -2.0f * std::cos(omega);
            b2_ = 1.0f - alpha * (a * a);
            a1_ = b1_;
            a2_ = 1.0f - alpha;

            const float a0_inv = 1.0f / (1.0f + alpha);
            b0_ *= a0_inv;
            b1_ *= a0_inv;
            b2_ *= a0_inv;
            a1_ *= a0_inv;
            a2_ *= a0_inv;

            center_freq_hz_ = new_center_freq;
            sample_freq_hz_ = sample_freq_hz;
            a_ = a;
            initialised_ = true;
        } else {
            initialised_ = false;
        }
    }

    T apply(const T& sample) {
        if (!initialised_ || need_reset_) {
            signal1_ = sample;
            signal2_ = sample;
            ntchsig1_ = sample;
            ntchsig2_ = sample;
            need_reset_ = false;
            return sample;
        }

        T output = sample * b0_ + ntchsig1_ * b1_ + ntchsig2_ * b2_ - signal1_ * a1_ - signal2_ * a2_;

        ntchsig2_ = ntchsig1_;
        ntchsig1_ = sample;
        signal2_ = signal1_;
        signal1_ = output;
        return output;
    }

    void reset() { need_reset_ = true; }
    void disable() { initialised_ = false; }

    [[nodiscard]] float center_freq_hz() const { return center_freq_hz_; }
    [[nodiscard]] float sample_freq_hz() const { return sample_freq_hz_; }
    [[nodiscard]] bool is_initialised() const { return initialised_; }

    // Upstream returns AP_Logger::quiet_nanf() when not initialised (a
    // logging-subsystem NaN sentinel). AP_Logger doesn't exist in this
    // port; std::nanf("") is the same IEEE mechanism without the
    // logging-specific payload bits AP_Logger's variant sets.
    [[nodiscard]] float logging_frequency() const {
        return initialised_ ? center_freq_hz_ : std::nanf("");
    }

private:
    static constexpr float kNotchMaxSlew = 0.05f;
    static constexpr float kNotchMaxSlewLower = 1.0f - kNotchMaxSlew;
    static constexpr float kNotchMaxSlewUpper = 1.0f / kNotchMaxSlewLower;

    bool initialised_ = false; // see D-015
    bool need_reset_ = false;  // see D-015
    float b0_ = 0.0f, b1_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
    float center_freq_hz_ = 0.0f, sample_freq_hz_ = 0.0f, a_ = 0.0f;
    T ntchsig1_{}, ntchsig2_{}, signal2_{}, signal1_{};
};

using NotchFilterFloat = NotchFilter<float>;
using NotchFilterVector2f = NotchFilter<math::Vector2f>;
using NotchFilterVector3f = NotchFilter<math::Vector3f>;

} // namespace fwcpp::filter
