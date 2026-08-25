// Tests for fwcpp::filter::LowPassFilter2p / DigitalBiquadFilter (CPP-013).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/filter/low_pass_filter_2p.hpp>

#include <cmath>

using namespace fwcpp::filter;

TEST_CASE("zero or negative cutoff/sample freq makes the filter a pass-through", "[lpf2p]") {
    LowPassFilter2pFloat f(100.0f, 0.0f); // zero cutoff
    REQUIRE(f.apply(5.0f) == 5.0f);
    REQUIRE(f.apply(-3.0f) == -3.0f);
}

TEST_CASE("D-013: a freshly constructed filter seeds itself on the first apply, not garbage", "[lpf2p][D-013]") {
    // This is the regression test for the fix: upstream's `initialised`
    // member is never set in the constructor, so whether the first apply()
    // seeds correctly depends on uninitialized memory. This port
    // explicitly initializes it false, so the first apply() ALWAYS seeds.
    // If that fix regressed, this test would be flaky (pass or fail
    // depending on memory contents) rather than reliably failing - a
    // reliable pass here is itself evidence the fix is in place.
    LowPassFilter2pFloat f(100.0f, 10.0f);
    float first_output = f.apply(50.0f);
    // On first call with a non-trivial signal, output should already
    // reflect the seeded value, not a transient from a zero-initialized
    // delay line racing to catch up over many samples.
    REQUIRE(first_output == Catch::Approx(50.0f).margin(1.0f));
}

TEST_CASE("a constant input converges to itself (unity DC gain)", "[lpf2p]") {
    LowPassFilter2pFloat f(100.0f, 10.0f);
    float out = 0.0f;
    for (int i = 0; i < 50; ++i) {
        out = f.apply(20.0f);
    }
    REQUIRE(out == Catch::Approx(20.0f).margin(0.01f));
}

TEST_CASE("reset(value) seeds the filter so the next apply starts near value", "[lpf2p]") {
    LowPassFilter2pFloat f(100.0f, 10.0f);
    f.reset(75.0f);
    float out = f.apply(75.0f);
    REQUIRE(out == Catch::Approx(75.0f).margin(0.01f));
}

TEST_CASE("reset() with no args forces the next apply to re-seed", "[lpf2p]") {
    LowPassFilter2pFloat f(100.0f, 10.0f);
    f.apply(10.0f);
    f.apply(10.0f);
    f.reset();
    // After a bare reset(), the next apply() should re-seed near ITS
    // input, not continue from the old converged state.
    float out = f.apply(999.0f);
    REQUIRE(out == Catch::Approx(999.0f).margin(5.0f));
}

TEST_CASE("get_cutoff_freq / get_sample_freq report what was configured", "[lpf2p]") {
    LowPassFilter2pFloat f(100.0f, 10.0f);
    REQUIRE(f.get_sample_freq() == Catch::Approx(100.0f));
    REQUIRE(f.get_cutoff_freq() == Catch::Approx(10.0f));
}

TEST_CASE("cutoff is clamped to 0.4x sample frequency (Nyquist margin)", "[lpf2p]") {
    LowPassFilter2pFloat f(100.0f, 1000.0f); // requested cutoff far above Nyquist
    REQUIRE(f.get_cutoff_freq() == Catch::Approx(40.0f)); // clamped to 100*0.4
}

TEST_CASE("a low-frequency signal passes through with little attenuation", "[lpf2p]") {
    const float sample_freq = 1000.0f;
    const float cutoff = 50.0f;
    const float signal_freq = 1.0f; // well below cutoff
    LowPassFilter2pFloat f(sample_freq, cutoff);

    float max_out = 0.0f;
    const int n = 2000;
    for (int i = 0; i < n; ++i) {
        float t = static_cast<float>(i) / sample_freq;
        float sample = std::sin(2.0f * 3.14159265f * signal_freq * t);
        float out = f.apply(sample);
        if (i > n / 2) { // past the settling transient
            max_out = std::max(max_out, std::fabs(out));
        }
    }
    REQUIRE(max_out > 0.9f); // amplitude ~1, little attenuation expected
}

TEST_CASE("a high-frequency signal is significantly attenuated", "[lpf2p]") {
    const float sample_freq = 1000.0f;
    const float cutoff = 5.0f;
    const float signal_freq = 200.0f; // well above cutoff
    LowPassFilter2pFloat f(sample_freq, cutoff);

    float max_out = 0.0f;
    const int n = 2000;
    for (int i = 0; i < n; ++i) {
        float t = static_cast<float>(i) / sample_freq;
        float sample = std::sin(2.0f * 3.14159265f * signal_freq * t);
        float out = f.apply(sample);
        if (i > n / 2) {
            max_out = std::max(max_out, std::fabs(out));
        }
    }
    REQUIRE(max_out < 0.3f); // significantly attenuated from amplitude 1
}

TEST_CASE("LowPassFilter2p works with Vector2f, filtering each component", "[lpf2p]") {
    LowPassFilter2pVector2f f(100.0f, 10.0f);
    fwcpp::math::Vector2f out;
    for (int i = 0; i < 50; ++i) {
        out = f.apply(fwcpp::math::Vector2f(3.0f, -4.0f));
    }
    REQUIRE(out.x == Catch::Approx(3.0f).margin(0.01f));
    REQUIRE(out.y == Catch::Approx(-4.0f).margin(0.01f));
}

TEST_CASE("DigitalBiquadFilter used directly (not through LowPassFilter2p) also self-seeds", "[lpf2p]") {
    fwcpp::filter::BiquadParams params;
    fwcpp::filter::compute_biquad_params(100.0f, 10.0f, params);
    fwcpp::filter::DigitalBiquadFilter<float> bq;
    float out = bq.apply(42.0f, params);
    REQUIRE(out == Catch::Approx(42.0f).margin(1.0f));
}
