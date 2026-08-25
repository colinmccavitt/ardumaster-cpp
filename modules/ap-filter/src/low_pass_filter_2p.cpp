// Definitions for the compute_biquad_params free function declared in
// low_pass_filter_2p.hpp. Compiled under fwcpp_upstream_flags - the same
// single-translation-unit treatment scalar.cpp's wrap_* family gets, for
// the same reason: `sample_freq * 0.4` and the bare M_PI references below
// are ambiguous-literal territory under -fsingle-precision-constant.

#include <fwcpp/filter/low_pass_filter_2p.hpp>

namespace fwcpp::filter {

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884; // matches scalar.cpp's kPi
}

void compute_biquad_params(float sample_freq, float cutoff_freq, BiquadParams& out) {
    // Keep well under Nyquist limit.
    const float nyquist_limit = sample_freq * 0.4f;
    out.cutoff_freq = cutoff_freq < nyquist_limit ? cutoff_freq : nyquist_limit;
    out.sample_freq = sample_freq;
    if (!math::is_positive(out.cutoff_freq)) {
        // Zero cutoff means pass-through - matches upstream's early return,
        // leaving a1/a2/b0/b1/b2 at their BiquadParams default (0), which
        // is what DigitalBiquadFilter::apply's is_positive(cutoff_freq)
        // guard checks for anyway.
        return;
    }

    const float fr = out.sample_freq / out.cutoff_freq;
    const float ohm = std::tan(static_cast<float>(kPi) / fr);
    const float c = 1.0f + 2.0f * std::cos(static_cast<float>(kPi) / 4.0f) * ohm + ohm * ohm;

    out.b0 = ohm * ohm / c;
    out.b1 = 2.0f * out.b0;
    out.b2 = out.b0;
    out.a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    out.a2 = (1.0f - 2.0f * std::cos(static_cast<float>(kPi) / 4.0f) * ohm + ohm * ohm) / c;
}

} // namespace fwcpp::filter
