// Tests for fwcpp::Float16 (CPP-012), checked against known IEEE
// binary16 bit patterns (not bfloat16 - see float16.hpp's file banner).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/float16.hpp>

#include <cmath>
#include <limits>

using fwcpp::Float16;

TEST_CASE("Float16::set encodes known values to their standard binary16 bit patterns", "[float16]") {
    Float16 h;
    h.set(0.0f);
    REQUIRE(h.v16 == 0x0000);
    h.set(-0.0f);
    REQUIRE(h.v16 == 0x8000);
    h.set(1.0f);
    REQUIRE(h.v16 == 0x3C00);
    h.set(-1.0f);
    REQUIRE(h.v16 == 0xBC00);
    h.set(2.0f);
    REQUIRE(h.v16 == 0x4000);
    h.set(0.5f);
    REQUIRE(h.v16 == 0x3800);
}

TEST_CASE("Float16::get decodes known bit patterns to their standard values", "[float16]") {
    Float16 h;
    h.v16 = 0x3C00;
    REQUIRE(h.get() == 1.0f);
    h.v16 = 0xBC00;
    REQUIRE(h.get() == -1.0f);
    h.v16 = 0x4000;
    REQUIRE(h.get() == 2.0f);
    h.v16 = 0x0000;
    REQUIRE(h.get() == 0.0f);
}

TEST_CASE("Float16 round-trips exactly-representable values", "[float16]") {
    for (float v : {0.0f, 1.0f, -1.0f, 2.0f, 0.5f, -0.5f, 100.0f, -100.0f}) {
        Float16 h;
        h.set(v);
        REQUIRE(h.get() == Catch::Approx(v));
    }
}

TEST_CASE("Float16 round-trips a non-exact value within half-precision tolerance", "[float16]") {
    // Half precision has ~3 decimal digits of precision (10 mantissa bits).
    Float16 h;
    h.set(3.14159f);
    REQUIRE(h.get() == Catch::Approx(3.14159f).margin(0.01f));
}

TEST_CASE("Float16 saturates a too-large finite value to infinity", "[float16]") {
    Float16 h;
    h.set(1.0e10f); // far beyond binary16's max (~65504)
    REQUIRE(h.v16 == 0x7C00); // +infinity
    REQUIRE(std::isinf(h.get()));
    REQUIRE(h.get() > 0.0f);
}

TEST_CASE("Float16 saturates a large negative value to negative infinity", "[float16]") {
    Float16 h;
    h.set(-1.0e10f);
    REQUIRE(h.v16 == 0xFC00);
    REQUIRE(std::isinf(h.get()));
    REQUIRE(h.get() < 0.0f);
}

TEST_CASE("Float16 preserves actual infinity", "[float16]") {
    Float16 h;
    h.set(std::numeric_limits<float>::infinity());
    REQUIRE(h.v16 == 0x7C00);
    REQUIRE(h.get() == std::numeric_limits<float>::infinity());

    Float16 neg;
    neg.set(-std::numeric_limits<float>::infinity());
    REQUIRE(neg.v16 == 0xFC00);
}

TEST_CASE("Float16 default-constructed value decodes to zero", "[float16]") {
    Float16 h;
    REQUIRE(h.v16 == 0);
    REQUIRE(h.get() == 0.0f);
}
