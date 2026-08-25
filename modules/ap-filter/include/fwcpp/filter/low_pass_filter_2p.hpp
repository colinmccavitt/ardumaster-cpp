#pragma once

// Port of Filter/LowPassFilter2p.h + LowPassFilter2p.cpp. CPP-013.
//
// D-013 (registered independently here; same bug class the Rust port found
// repeatedly across this same file family - DigitalLPF, DigitalBiquadFilter,
// NotchFilter, DerivativeFilter - and called D-005 there): upstream's
// DigitalBiquadFilter constructor sets _delay_element_1/_2 to T() but never
// initializes `initialised`. Confirmed by reading LowPassFilter2p.cpp
// directly: the member is only ever assigned inside the two reset()
// overloads, never in the constructor. A freshly-constructed filter's
// initialised flag is genuinely indeterminate - reading it is itself
// undefined behavior, and if it happens to read as `true`, apply()'s
// `if (!initialised) { reset(sample, params); }` skips the seeding reset
// entirely, so the filter's first output is computed from delay elements
// that were never meaningfully set.
//
// FIXED, not reproduced: `initialised` is explicitly false in this port's
// constructor. ADR-0007 permits fixing inherited bugs when doing so doesn't
// change intentional behavior - there is no "intentional" value for reading
// an uninitialized bool, so there is nothing to preserve by reproducing the
// bug, unlike D-003's epsilon choice (which trades one deliberate behavior
// for another). The alternative - deliberately leaving a member
// uninitialized in a from-scratch C++ port - would be manufacturing new UB
// to match old UB, which is not what "reproduce upstream" means.
//
// LITERAL SAFETY: compute_biquad_params (below, in low_pass_filter_2p.cpp)
// has upstream's `sample_freq * 0.4` (0.4 not exact in either precision)
// and bare M_PI references - needs the compiled-.cpp treatment, same
// pattern as scalar.cpp's wrap_* family. It is pulled out as a plain
// function rather than kept as a per-T static template member (upstream's
// own shape): the computation doesn't touch T at all, so upstream's
// version silently compiles a separate identical copy for every
// instantiation (int, long, float, Vector2f, Vector3f) - a
// modernization within ADR-0012's charter, not a behavior change.
// DigitalBiquadFilter<T>::compute_params still exists as upstream declares
// it, forwarding to the shared implementation.

#include <cmath>
#include <cstdint>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::filter {

struct BiquadParams {
    float cutoff_freq = 0.0f;
    float sample_freq = 0.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;
    float b0 = 0.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
};

// Defined in low_pass_filter_2p.cpp - see file banner.
void compute_biquad_params(float sample_freq, float cutoff_freq, BiquadParams& out);

template <typename T>
class DigitalBiquadFilter {
public:
    DigitalBiquadFilter() = default;
    DigitalBiquadFilter(const DigitalBiquadFilter&) = delete;
    DigitalBiquadFilter& operator=(const DigitalBiquadFilter&) = delete;

    T apply(const T& sample, const BiquadParams& params) {
        if (!math::is_positive(params.cutoff_freq) || !math::is_positive(params.sample_freq)) {
            return sample;
        }
        if (!initialised_) {
            reset(sample, params);
        }
        T delay_element_0 = sample - delay_element_1_ * params.a1 - delay_element_2_ * params.a2;
        T output = delay_element_0 * params.b0 + delay_element_1_ * params.b1 + delay_element_2_ * params.b2;
        delay_element_2_ = delay_element_1_;
        delay_element_1_ = delay_element_0;
        return output;
    }

    void reset() { initialised_ = false; }

    void reset(const T& value, const BiquadParams& params) {
        // The reciprocal is a scalar float (matches upstream: `value *
        // (1.0 / (1 + a1 + a2))` - the parenthesized part is a plain
        // number, not T-typed, which matters when T is Vector2f/Vector3f
        // and has no single-argument constructor to receive a bare T(1.0)).
        // 1.0 itself is exact in every float format, so no flag-precision
        // concern the way compute_biquad_params's 0.4 has.
        const float recip = 1.0f / (1.0f + params.a1 + params.a2);
        delay_element_1_ = delay_element_2_ = value * recip;
        initialised_ = true;
    }

    static void compute_params(float sample_freq, float cutoff_freq, BiquadParams& out) {
        compute_biquad_params(sample_freq, cutoff_freq, out);
    }

private:
    T delay_element_1_{};
    T delay_element_2_{};
    bool initialised_ = false; // see D-013 above
};

template <typename T>
class LowPassFilter2p {
public:
    LowPassFilter2p() = default;
    LowPassFilter2p(float sample_freq, float cutoff_freq) { set_cutoff_frequency(sample_freq, cutoff_freq); }

    LowPassFilter2p(const LowPassFilter2p&) = delete;
    LowPassFilter2p& operator=(const LowPassFilter2p&) = delete;

    void set_cutoff_frequency(float sample_freq, float cutoff_freq) {
        compute_biquad_params(sample_freq, cutoff_freq, params_);
    }

    [[nodiscard]] float get_cutoff_freq() const { return params_.cutoff_freq; }
    [[nodiscard]] float get_sample_freq() const { return params_.sample_freq; }

    T apply(const T& sample) { return filter_.apply(sample, params_); }
    void reset() { filter_.reset(); }
    void reset(const T& value) { filter_.reset(value, params_); }

protected:
    BiquadParams params_;

private:
    DigitalBiquadFilter<T> filter_;
};

using LowPassFilter2pInt = LowPassFilter2p<int>;
using LowPassFilter2pLong = LowPassFilter2p<long>;
using LowPassFilter2pFloat = LowPassFilter2p<float>;
using LowPassFilter2pVector2f = LowPassFilter2p<math::Vector2f>;
using LowPassFilter2pVector3f = LowPassFilter2p<math::Vector3f>;

} // namespace fwcpp::filter
