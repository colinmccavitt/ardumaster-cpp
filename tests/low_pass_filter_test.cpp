// Tests for fwcpp::filter::LowPassFilter / LowPassFilterConstDt (CPP-014).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/filter/low_pass_filter.hpp>

using namespace fwcpp::filter;

TEST_CASE("D-014: a freshly constructed filter seeds on first apply, not garbage", "[lpf][D-014]") {
    LowPassFilterFloat f(10.0f);
    float first = f.apply(50.0f, 0.01f);
    REQUIRE(first == 50.0f); // exact: first sample always becomes the seed
}

TEST_CASE("LowPassFilter converges to a constant input", "[lpf]") {
    LowPassFilterFloat f(10.0f);
    float out = 0.0f;
    for (int i = 0; i < 200; ++i) {
        out = f.apply(20.0f, 0.001f);
    }
    REQUIRE(out == Catch::Approx(20.0f).margin(0.01f));
}

TEST_CASE("LowPassFilter::reset() forces re-seeding on the next apply", "[lpf]") {
    LowPassFilterFloat f(10.0f);
    f.apply(1.0f, 0.01f);
    f.apply(1.0f, 0.01f);
    f.reset();
    REQUIRE(f.apply(500.0f, 0.01f) == 500.0f);
}

TEST_CASE("LowPassFilter::reset(value) seeds directly", "[lpf]") {
    LowPassFilterFloat f(10.0f);
    f.reset(42.0f);
    REQUIRE(f.get() == 42.0f);
}

TEST_CASE("zero cutoff frequency makes the filter track the input immediately", "[lpf]") {
    LowPassFilterFloat f(0.0f); // alpha == 1
    f.apply(1.0f, 0.01f);
    float out = f.apply(99.0f, 0.01f);
    REQUIRE(out == Catch::Approx(99.0f));
}

TEST_CASE("LowPassFilterConstDt precomputes alpha and matches LowPassFilter for equivalent dt", "[lpf]") {
    const float sample_freq = 100.0f;
    const float cutoff = 10.0f;
    const float dt = 1.0f / sample_freq;

    LowPassFilterConstDtFloat const_dt(sample_freq, cutoff);
    LowPassFilterFloat variable_dt(cutoff);

    float out_const = 0.0f, out_var = 0.0f;
    for (int i = 0; i < 50; ++i) {
        out_const = const_dt.apply(20.0f);
        out_var = variable_dt.apply(20.0f, dt);
    }
    REQUIRE(out_const == Catch::Approx(out_var).margin(1e-5f));
}

TEST_CASE("LowPassFilter works with Vector2f, filtering each component", "[lpf]") {
    LowPassFilterVector2f f(10.0f);
    fwcpp::math::Vector2f out;
    for (int i = 0; i < 200; ++i) {
        out = f.apply(fwcpp::math::Vector2f(3.0f, -4.0f), 0.001f);
    }
    REQUIRE(out.x == Catch::Approx(3.0f).margin(0.01f));
    REQUIRE(out.y == Catch::Approx(-4.0f).margin(0.01f));
}

TEST_CASE("calc_lowpass_alpha_dt matches known boundary behaviors", "[lpf]") {
    REQUIRE(fwcpp::math::calc_lowpass_alpha_dt(0.01f, 0.0f) == 1.0f); // zero cutoff -> alpha 1
    REQUIRE(fwcpp::math::calc_lowpass_alpha_dt(0.0f, 10.0f) == 0.0f); // zero dt -> alpha 0
}

TEST_CASE("calc_lowpass_alpha_dt reports invalid negative inputs via InternalError", "[lpf]") {
    fwcpp::InternalError err;
    float alpha = fwcpp::math::calc_lowpass_alpha_dt(-1.0f, 10.0f, &err, 55);
    REQUIRE(alpha == 1.0f);
    REQUIRE(err.has_error(fwcpp::InternalErrorCode::invalid_arg_or_result));
    REQUIRE(err.last_error_line() == 55);
}
