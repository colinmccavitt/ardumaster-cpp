#pragma once

// Port of Filter/LowPassFilter.h + LowPassFilter.cpp - the first-order
// (single-pole, alpha-blend) low-pass, DISTINCT from LowPassFilter2p's
// second-order biquad (CPP-013). Two APIs upstream provides over the same
// DigitalLPF core: LowPassFilterConstDt (precomputes alpha once from a
// fixed sample rate) and LowPassFilter (recomputes alpha every apply() from
// a supplied dt) - both ported here since they share nearly all their code.
//
// D-014 found and FIXED (same bug class as D-013/Rust's D-005, found
// independently again by reading this file): DigitalLPF's constructor sets
// `output = T()` but never initializes `initialised` - only the two
// reset() overloads do. Fixed the same way: explicit `initialised_ = false`
// in this port's constructor, regression-pinned by a test.

#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::filter {

template <typename T>
class DigitalLPF {
public:
    DigitalLPF() = default;
    DigitalLPF(const DigitalLPF&) = delete;
    DigitalLPF& operator=(const DigitalLPF&) = delete;

    [[nodiscard]] const T& get() const { return output_; }

    void reset(const T& value) {
        output_ = value;
        initialised_ = true;
    }
    void reset() { initialised_ = false; }

protected:
    T apply_alpha(const T& sample, float alpha) {
        output_ += (sample - output_) * alpha;
        if (!initialised_) {
            initialised_ = true;
            output_ = sample;
        }
        return output_;
    }

private:
    T output_{};
    bool initialised_ = false; // see D-014 above
};

template <typename T>
class LowPassFilterConstDt : public DigitalLPF<T> {
public:
    LowPassFilterConstDt() = default;
    LowPassFilterConstDt(float sample_freq, float cutoff_freq) { set_cutoff_frequency(sample_freq, cutoff_freq); }

    LowPassFilterConstDt(const LowPassFilterConstDt&) = delete;
    LowPassFilterConstDt& operator=(const LowPassFilterConstDt&) = delete;

    void set_cutoff_frequency(float sample_freq, float new_cutoff_freq) {
        cutoff_freq_ = new_cutoff_freq;
        if (sample_freq <= 0.0f) {
            alpha_ = 1.0f;
        } else {
            alpha_ = math::calc_lowpass_alpha_dt(1.0f / sample_freq, cutoff_freq_);
        }
    }

    [[nodiscard]] float get_cutoff_freq() const { return cutoff_freq_; }

    T apply(const T& sample) { return this->apply_alpha(sample, alpha_); }

private:
    float cutoff_freq_ = 0.0f;
    float alpha_ = 0.0f;
};

using LowPassFilterConstDtFloat = LowPassFilterConstDt<float>;
using LowPassFilterConstDtVector2f = LowPassFilterConstDt<math::Vector2f>;
using LowPassFilterConstDtVector3f = LowPassFilterConstDt<math::Vector3f>;

template <typename T>
class LowPassFilter : public DigitalLPF<T> {
public:
    LowPassFilter() = default;
    explicit LowPassFilter(float cutoff_freq) { set_cutoff_frequency(cutoff_freq); }

    LowPassFilter(const LowPassFilter&) = delete;
    LowPassFilter& operator=(const LowPassFilter&) = delete;

    void set_cutoff_frequency(float new_cutoff_freq) { cutoff_freq_ = new_cutoff_freq; }
    [[nodiscard]] float get_cutoff_freq() const { return cutoff_freq_; }

    T apply(const T& sample, float dt) {
        const float alpha = math::calc_lowpass_alpha_dt(dt, cutoff_freq_);
        return this->apply_alpha(sample, alpha);
    }

private:
    float cutoff_freq_ = 0.0f;
};

using LowPassFilterFloat = LowPassFilter<float>;
using LowPassFilterVector2f = LowPassFilter<math::Vector2f>;
using LowPassFilterVector3f = LowPassFilter<math::Vector3f>;

} // namespace fwcpp::filter
