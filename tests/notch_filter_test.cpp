// Tests for fwcpp::filter::NotchFilter (CPP-018).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/filter/notch_filter.hpp>

#include <cmath>

using namespace fwcpp::filter;

TEST_CASE("D-015: an un-initialised filter passes samples through unchanged, not garbage", "[notch][D-015]") {
    NotchFilterFloat f;
    REQUIRE_FALSE(f.is_initialised());
    REQUIRE(f.apply(5.0f) == 5.0f);
    REQUIRE(f.apply(-3.0f) == -3.0f);
    REQUIRE(f.apply(0.0f) == 0.0f);
}

TEST_CASE("init with valid parameters initialises the filter", "[notch]") {
    NotchFilterFloat f;
    f.init(1000.0f, 100.0f, 20.0f, 40.0f); // sample 1kHz, center 100Hz, bw 20Hz, 40dB atten
    REQUIRE(f.is_initialised());
    REQUIRE(f.center_freq_hz() == Catch::Approx(100.0f));
    REQUIRE(f.sample_freq_hz() == Catch::Approx(1000.0f));
}

TEST_CASE("init with center frequency above Nyquist leaves the filter uninitialised", "[notch]") {
    NotchFilterFloat f;
    f.init(1000.0f, 600.0f, 20.0f, 40.0f); // 600Hz center > 500Hz Nyquist
    REQUIRE_FALSE(f.is_initialised());
}

TEST_CASE("init with center frequency too close to bandwidth leaves the filter uninitialised", "[notch]") {
    NotchFilterFloat f;
    f.init(1000.0f, 5.0f, 20.0f, 40.0f); // center(5) <= 0.5*bandwidth(10)
    REQUIRE_FALSE(f.is_initialised());
}

TEST_CASE("disable() forces the next apply to passthrough and re-seed", "[notch]") {
    NotchFilterFloat f;
    f.init(1000.0f, 100.0f, 20.0f, 40.0f);
    REQUIRE(f.is_initialised());
    f.disable();
    REQUIRE_FALSE(f.is_initialised());
    REQUIRE(f.apply(42.0f) == 42.0f);
}

TEST_CASE("reset() forces exactly the next apply to re-seed, then resumes normal filtering", "[notch]") {
    NotchFilterFloat f;
    f.init(1000.0f, 100.0f, 20.0f, 40.0f);
    f.apply(1.0f);
    f.apply(1.0f);
    f.reset();
    REQUIRE(f.apply(999.0f) == 999.0f); // reseed sample passes through unchanged
    // subsequent calls filter normally again (don't re-seed every time)
    float out = f.apply(999.0f);
    REQUIRE(out != 0.0f); // sanity: producing real filtered output, not stuck
}

TEST_CASE("a signal at the notch center frequency is significantly attenuated", "[notch]") {
    const float sample_freq = 1000.0f;
    const float center = 50.0f;
    NotchFilterFloat f;
    f.init(sample_freq, center, 10.0f, 40.0f);

    float max_amplitude = 0.0f;
    const int n = 2000;
    for (int i = 0; i < n; ++i) {
        float t = static_cast<float>(i) / sample_freq;
        float sample = std::sin(2.0f * 3.14159265f * center * t);
        float out = f.apply(sample);
        if (i > n / 2) {
            max_amplitude = std::max(max_amplitude, std::fabs(out));
        }
    }
    REQUIRE(max_amplitude < 0.3f); // strongly attenuated at the notch center
}

TEST_CASE("a signal far from the notch center frequency passes through with little attenuation", "[notch]") {
    const float sample_freq = 1000.0f;
    const float center = 50.0f;
    const float signal_freq = 300.0f; // well away from the notch
    NotchFilterFloat f;
    f.init(sample_freq, center, 10.0f, 40.0f);

    float max_amplitude = 0.0f;
    const int n = 2000;
    for (int i = 0; i < n; ++i) {
        float t = static_cast<float>(i) / sample_freq;
        float sample = std::sin(2.0f * 3.14159265f * signal_freq * t);
        float out = f.apply(sample);
        if (i > n / 2) {
            max_amplitude = std::max(max_amplitude, std::fabs(out));
        }
    }
    REQUIRE(max_amplitude > 0.7f); // largely unattenuated far from the notch
}

TEST_CASE("calculate_a_and_q produces a positive Q when center exceeds half the bandwidth", "[notch]") {
    float a = 0.0f, q = 0.0f;
    NotchFilterFloat::calculate_a_and_q(100.0f, 20.0f, 40.0f, a, q);
    REQUIRE(a > 0.0f);
    REQUIRE(q > 0.0f);
}

TEST_CASE("calculate_a_and_q gives Q=0 when center does not exceed half the bandwidth", "[notch]") {
    float a = 0.0f, q = 0.0f;
    NotchFilterFloat::calculate_a_and_q(5.0f, 20.0f, 40.0f, a, q); // 5 <= 0.5*20=10
    REQUIRE(q == 0.0f);
}

TEST_CASE("NotchFilter works with Vector2f, filtering each component", "[notch]") {
    // Note: init() alone does not set need_reset_ (only reset() does), so
    // the very first apply() after init() runs the real biquad computation
    // against zero-initialized delay elements - a genuine startup
    // transient, not a seeded passthrough. Call reset() explicitly to get
    // the seeded-passthrough behavior this test wants to check.
    NotchFilterVector2f f;
    f.init(1000.0f, 50.0f, 10.0f, 40.0f);
    f.reset();
    fwcpp::math::Vector2f out = f.apply(fwcpp::math::Vector2f(3.0f, -4.0f));
    REQUIRE(out == fwcpp::math::Vector2f(3.0f, -4.0f)); // seeded passthrough
}
