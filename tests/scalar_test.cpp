// Parity tests for fwcpp::math scalar helpers (CPP-004), checked against
// upstream test vectors (AP_Math/tests/test_math.cpp,
// AP_Math/tests/test_math_double.cpp, read directly from the pinned
// Plane-4.7.0 worktree) plus values chosen to pin specific upstream
// decisions this port reproduces rather than "fixes" (see scalar.hpp's
// D-003 comment).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/scalar.hpp>

#include <cfloat>
#include <cmath>

using namespace fwcpp::math;

TEST_CASE("is_zero(float) matches upstream test_math.cpp vectors", "[scalar]") {
    REQUIRE_FALSE(is_zero(0.1f));
    REQUIRE_FALSE(is_zero(0.0001f));
    REQUIRE(is_zero(0.0f));
    REQUIRE(is_zero(FLT_MIN));
    REQUIRE(is_zero(-FLT_MIN));
}

TEST_CASE("is_zero(double) matches upstream test_math_double.cpp vectors", "[scalar]") {
    REQUIRE_FALSE(is_zero(0.1));
    REQUIRE_FALSE(is_zero(0.0001));
    REQUIRE(is_zero(0.0));
    REQUIRE(is_zero(DBL_MIN));
    REQUIRE(is_zero(-DBL_MIN));
}

TEST_CASE("D-003: is_zero(double) uses FLT_EPSILON, not DBL_EPSILON - reproduced, not fixed", "[scalar][D-003]") {
    // 1e-10 sits between DBL_EPSILON (~2.22e-16) and FLT_EPSILON (~1.19e-7).
    // Upstream's actual (if debatably inconsistent) behavior calls this
    // zero. A "corrected" DBL_EPSILON-based implementation would not. This
    // test exists specifically so a future edit that "fixes" the epsilon
    // fails loudly - see D-003's reasoning in scalar.hpp and FW-035's
    // ticket notes for why upstream's choice is reproduced deliberately.
    REQUIRE(is_zero(1e-10));
    REQUIRE_FALSE(is_zero(1e-6)); // above FLT_EPSILON, correctly non-zero either way
}

TEST_CASE("is_positive / is_negative use FLT_EPSILON for both float and double", "[scalar]") {
    REQUIRE(is_positive(1.0f));
    REQUIRE_FALSE(is_positive(0.0f));
    REQUIRE_FALSE(is_positive(-1.0f));
    REQUIRE(is_negative(-1.0f));
    REQUIRE_FALSE(is_negative(0.0f));
}

TEST_CASE("is_positive(double) boundary matches the FLT_EPSILON threshold", "[scalar]") {
    // Mirrors is_zero's D-003 threshold: is_positive(double) also compares
    // against FLT_EPSILON (~1.19e-7), so a double smaller than that is NOT
    // positive even though it is a genuine nonzero positive number.
    REQUIRE_FALSE(is_positive(1e-10));
    REQUIRE(is_positive(1e-6));
}

TEST_CASE("is_equal: integral overload is exact", "[scalar]") {
    REQUIRE(is_equal(5, 5));
    REQUIRE_FALSE(is_equal(5, 6));
}

TEST_CASE("is_equal: floating overload uses the type-correct epsilon (unlike is_zero)", "[scalar]") {
    // Contrast with is_zero/is_positive: is_equal is NOT the D-003 case.
    // Upstream genuinely uses DBL_EPSILON here for the double instantiation.
    REQUIRE_FALSE(is_equal(1e-10, 0.0)); // 1e-10 > DBL_EPSILON, so NOT equal
    REQUIRE(is_equal(1e-17, 0.0));       // below DBL_EPSILON, equal
    REQUIRE(is_equal(1.0f, 1.0f + FLT_EPSILON / 2));
}

TEST_CASE("wrap_360 and wrap_360_cd wrap into the 0 to 360 range", "[scalar]") {
    REQUIRE(wrap_360(370.0f) == Catch::Approx(10.0f));
    REQUIRE(wrap_360(-10.0f) == Catch::Approx(350.0f));
    REQUIRE(wrap_360(0.0f) == Catch::Approx(0.0f));
    REQUIRE(wrap_360_cd(37000.0f) == Catch::Approx(1000.0f));
    REQUIRE(wrap_360(370) == 10);
    REQUIRE(wrap_360(-10) == 350);
}

TEST_CASE("wrap_180 and wrap_180_cd wrap symmetrically around zero", "[scalar]") {
    REQUIRE(wrap_180(190.0f) == Catch::Approx(-170.0f));
    REQUIRE(wrap_180(170.0f) == Catch::Approx(170.0f));
    REQUIRE(wrap_180_cd(19000.0f) == Catch::Approx(-17000.0f));
}

TEST_CASE("wrap_2PI and wrap_PI wrap radians symmetrically around zero", "[scalar]") {
    REQUIRE(wrap_2PI(static_cast<float>(-M_PI / 2)) == Catch::Approx(static_cast<float>(3 * M_PI / 2)));
    REQUIRE(wrap_PI(static_cast<float>(3 * M_PI / 2)) == Catch::Approx(static_cast<float>(-M_PI / 2)));
}

TEST_CASE("safe_asin clamps out-of-domain inputs instead of returning NaN", "[scalar]") {
    REQUIRE(safe_asin(0.0f) == Catch::Approx(0.0f));
    REQUIRE(safe_asin(1.0f) == Catch::Approx(static_cast<float>(M_PI_2)));
    REQUIRE(safe_asin(2.0f) == Catch::Approx(static_cast<float>(M_PI_2))); // clamped, not NaN
    REQUIRE(safe_asin(-2.0f) == Catch::Approx(static_cast<float>(-M_PI_2)));
    REQUIRE(safe_asin(std::nanf("")) == 0.0f);
    REQUIRE(safe_asin(1) == Catch::Approx(static_cast<float>(M_PI_2))); // integral T
}

TEST_CASE("safe_sqrt returns zero for negative or NaN input instead of NaN", "[scalar]") {
    REQUIRE(safe_sqrt(4.0f) == Catch::Approx(2.0f));
    REQUIRE(safe_sqrt(0.0f) == Catch::Approx(0.0f));
    REQUIRE(safe_sqrt(-1.0f) == 0.0f);
    REQUIRE(safe_sqrt(std::nanf("")) == 0.0f);
    REQUIRE(safe_sqrt(4) == Catch::Approx(2.0f)); // integral T
}

TEST_CASE("constrain_value clamps within range and passes through inside it", "[scalar]") {
    REQUIRE(constrain_value(5.0f, 0.0f, 10.0f) == 5.0f);
    REQUIRE(constrain_value(-5.0f, 0.0f, 10.0f) == 0.0f);
    REQUIRE(constrain_value(15.0f, 0.0f, 10.0f) == 10.0f);
}

TEST_CASE("constrain_value on NaN returns the midpoint, matching upstream's own choice", "[scalar]") {
    float result = constrain_value(std::nanf(""), 0.0f, 10.0f);
    REQUIRE(result == 5.0f);
}

namespace {
class RecordingSink : public fwcpp::math::ConstrainNanSink {
public:
    int calls = 0;
    uint32_t last_line = 0;
    void on_constrain_nan(uint32_t line) override {
        ++calls;
        last_line = line;
    }
};
} // namespace

TEST_CASE("constrain_value reports NaN through an explicit sink, never a singleton", "[scalar]") {
    RecordingSink sink;
    float result = constrain_value(std::nanf(""), 0.0f, 10.0f, &sink, 42);
    REQUIRE(result == 5.0f);
    REQUIRE(sink.calls == 1);
    REQUIRE(sink.last_line == 42);
}

TEST_CASE("constrain_value with a null sink does not crash - matches a build with reporting disabled", "[scalar]") {
    REQUIRE(constrain_value(std::nanf(""), 0.0f, 10.0f, nullptr) == 5.0f);
}

TEST_CASE("constrain_value never invokes the sink off the NaN path", "[scalar]") {
    RecordingSink sink;
    float result = constrain_value(5.0f, 0.0f, 10.0f, &sink);
    REQUIRE(result == 5.0f);
    REQUIRE(sink.calls == 0);
}
