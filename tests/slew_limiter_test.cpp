// Tests for fwcpp::filter::SlewLimiter (CPP-015).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/filter/slew_limiter.hpp>

using fwcpp::filter::SlewLimiter;

TEST_CASE("SlewLimiter::modifier returns 1.0 for non-positive dt", "[slew_limiter]") {
    float max = 10.0f, tau = 1.0f;
    SlewLimiter s(max, tau);
    REQUIRE(s.modifier(5.0f, 0.0f, 1000) == 1.0f);
    REQUIRE(s.modifier(5.0f, -0.01f, 1000) == 1.0f);
}

TEST_CASE("SlewLimiter::modifier returns 1.0 when slew_rate_max is disabled (<=0)", "[slew_limiter]") {
    float max = 0.0f, tau = 1.0f;
    SlewLimiter s(max, tau);
    std::uint32_t t = 0;
    float mod = 1.0f;
    for (int i = 0; i < 20; ++i) {
        mod = s.modifier(static_cast<float>(i) * 1000.0f, 0.01f, t); // huge slew rate
        t += 10;
    }
    REQUIRE(mod == 1.0f);
}

TEST_CASE("SlewLimiter does not reduce output for a slowly-changing signal within the limit", "[slew_limiter]") {
    float max = 1000.0f, tau = 1.0f; // generous limit
    SlewLimiter s(max, tau);
    std::uint32_t t = 0;
    float mod = 0.0f;
    float sample = 0.0f;
    for (int i = 0; i < 200; ++i) {
        sample += 0.01f; // gentle ramp
        mod = s.modifier(sample, 0.01f, t);
        t += 10;
    }
    REQUIRE(mod == Catch::Approx(1.0f).margin(0.05f));
}

TEST_CASE("SlewLimiter reduces the modifier for a sustained excessive slew rate", "[slew_limiter]") {
    float max = 1.0f, tau = 0.5f; // tight limit
    SlewLimiter s(max, tau);
    std::uint32_t t = 0;
    float mod = 1.0f;
    float sample = 0.0f;
    // Drive a rapidly oscillating signal for over a second (multiple
    // WINDOW_MS=300ms windows) so the exceedance-event history actually
    // accumulates - a single spike isn't enough by this algorithm's design.
    for (int i = 0; i < 200; ++i) {
        sample = (i % 2 == 0) ? 100.0f : -100.0f; // huge, fast oscillation
        mod = s.modifier(sample, 0.01f, t);
        t += 10; // 10ms steps, 2000ms total
    }
    REQUIRE(mod < 1.0f);
    REQUIRE(mod > 0.0f); // never fully zeroes output, matches the formula's asymptote
}

TEST_CASE("SlewLimiter::get_slew_rate reflects a measured rate after driving the filter", "[slew_limiter]") {
    float max = 1000.0f, tau = 1.0f;
    SlewLimiter s(max, tau);
    std::uint32_t t = 0;
    float sample = 0.0f;
    for (int i = 0; i < 50; ++i) {
        sample += 10.0f;
        s.modifier(sample, 0.01f, t);
        t += 10;
    }
    REQUIRE(s.get_slew_rate() > 0.0f);
}

TEST_CASE("SlewLimiter references live parameter values, not a snapshot at construction", "[slew_limiter]") {
    float max = 5.0f, tau = 1.0f;
    SlewLimiter s(max, tau);
    max = 0.0f; // disable the limit AFTER construction
    std::uint32_t t = 0;
    float mod = s.modifier(1000.0f, 0.01f, t);
    // With max now 0, the early "slew_rate_max_ <= 0" branch should fire.
    REQUIRE(mod == 1.0f);
}
