// Tests for matrix_alg.hpp (CPP-004 continuation): mat_mul, mat_identity,
// mat_inverse's 3x3/4x4 closed-form paths.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/matrix_alg.hpp>

using namespace fwcpp::math;

TEST_CASE("mat_identity produces an identity matrix for arbitrary n", "[matrix_alg]") {
    float m[9];
    mat_identity(m, 3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            REQUIRE(m[i * 3 + j] == (i == j ? 1.0f : 0.0f));
        }
    }
}

TEST_CASE("mat_mul: identity is the multiplicative identity", "[matrix_alg]") {
    float a[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    float id[9];
    mat_identity(id, 3);
    float c[9];
    mat_mul(a, id, c, 3);
    for (int i = 0; i < 9; ++i) {
        REQUIRE(c[i] == Catch::Approx(a[i]));
    }
}

TEST_CASE("mat_mul matches hand-computed 2x2 product", "[matrix_alg]") {
    // [1 2] [5 6]   [19 22]
    // [3 4] [7 8] = [43 50]
    float a[4] = {1, 2, 3, 4};
    float b[4] = {5, 6, 7, 8};
    float c[4];
    mat_mul(a, b, c, 2);
    REQUIRE(c[0] == Catch::Approx(19.0f));
    REQUIRE(c[1] == Catch::Approx(22.0f));
    REQUIRE(c[2] == Catch::Approx(43.0f));
    REQUIRE(c[3] == Catch::Approx(50.0f));
}

TEST_CASE("inverse3x3 of the identity is the identity", "[matrix_alg]") {
    std::array<float, 9> id = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::array<float, 9> out{};
    REQUIRE(inverse3x3(id, out));
    for (int i = 0; i < 9; ++i) {
        REQUIRE(out[i] == Catch::Approx(id[i]));
    }
}

TEST_CASE("inverse3x3 of a known matrix matches the hand-computed inverse", "[matrix_alg]") {
    // A simple, well-conditioned, hand-verifiable case: a diagonal scale
    // matrix. Its inverse is just the reciprocal diagonal.
    std::array<float, 9> a = {2, 0, 0, 0, 4, 0, 0, 0, 5};
    std::array<float, 9> out{};
    REQUIRE(inverse3x3(a, out));
    REQUIRE(out[0] == Catch::Approx(0.5f));
    REQUIRE(out[4] == Catch::Approx(0.25f));
    REQUIRE(out[8] == Catch::Approx(0.2f));
    REQUIRE(out[1] == Catch::Approx(0.0f).margin(1e-6));
}

TEST_CASE("inverse3x3 round-trips: A * inverse(A) is the identity", "[matrix_alg]") {
    // A genuinely non-trivial, non-diagonal, well-conditioned matrix.
    std::array<float, 9> a = {4, 7, 2, 3, 5, 1, 2, 3, 4};
    std::array<float, 9> inv{};
    REQUIRE(inverse3x3(a, inv));

    float product[9];
    mat_mul(a.data(), inv.data(), product, 3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float expected = (i == j) ? 1.0f : 0.0f;
            REQUIRE(product[i * 3 + j] == Catch::Approx(expected).margin(1e-5));
        }
    }
}

TEST_CASE("inverse3x3 rejects a singular matrix", "[matrix_alg]") {
    // Row 3 = Row 1 + Row 2: determinant is exactly zero.
    std::array<float, 9> singular = {1, 2, 3, 4, 5, 6, 5, 7, 9};
    std::array<float, 9> out{};
    REQUIRE_FALSE(inverse3x3(singular, out));
}

TEST_CASE("inverse4x4 of the identity is the identity", "[matrix_alg]") {
    std::array<float, 16> id = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    std::array<float, 16> out{};
    REQUIRE(inverse4x4(id, out));
    for (int i = 0; i < 16; ++i) {
        REQUIRE(out[i] == Catch::Approx(id[i]));
    }
}

TEST_CASE("inverse4x4 round-trips: A * inverse(A) is the identity", "[matrix_alg]") {
    std::array<float, 16> a = {
        2, 0, 0, 1,
        0, 3, 0, 0,
        0, 0, 1, 0,
        1, 0, 0, 4,
    };
    std::array<float, 16> inv{};
    REQUIRE(inverse4x4(a, inv));

    float product[16];
    mat_mul(a.data(), inv.data(), product, 4);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float expected = (i == j) ? 1.0f : 0.0f;
            REQUIRE(product[i * 4 + j] == Catch::Approx(expected).margin(1e-5));
        }
    }
}

TEST_CASE("inverse4x4 rejects a singular matrix", "[matrix_alg]") {
    std::array<float, 16> singular{}; // all zero: certainly singular
    std::array<float, 16> out{};
    REQUIRE_FALSE(inverse4x4(singular, out));
}

TEST_CASE("mat_inverse dispatches to the 3x3 and 4x4 closed forms", "[matrix_alg]") {
    float a3[9] = {2, 0, 0, 0, 4, 0, 0, 0, 5};
    float out3[9];
    REQUIRE(mat_inverse(a3, out3, 3));
    REQUIRE(out3[0] == Catch::Approx(0.5f));

    float id4[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    float out4[16];
    REQUIRE(mat_inverse(id4, out4, 4));
    REQUIRE(out4[0] == Catch::Approx(1.0f));
}

TEST_CASE("mat_inverse returns false for a dimension it doesn't support, not a wrong answer", "[matrix_alg]") {
    // Scope decision documented in matrix_alg.hpp: the general-N LU path
    // heap-allocates (ADR-0012 decision 4), so it isn't ported. This test
    // pins the honest "unsupported" contract so a future silent regression
    // (e.g. someone wiring up a naive N-dim path that happens to allocate)
    // gets caught by a design-intent test, not just a missing feature.
    float a[25] = {}; // 5x5, arbitrary content - never reached, just needs valid storage
    float out[25];
    REQUIRE_FALSE(mat_inverse(a, out, 5));
}

TEST_CASE("mat_inverse works in double precision too", "[matrix_alg]") {
    std::array<double, 9> a = {2, 0, 0, 0, 4, 0, 0, 0, 5};
    std::array<double, 9> out{};
    REQUIRE(inverse3x3(a, out));
    REQUIRE(out[0] == Catch::Approx(0.5));
}
