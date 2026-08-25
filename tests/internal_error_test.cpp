// Tests for fwcpp::InternalError (CPP-005).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/internal_error.hpp>

using fwcpp::InternalError;
using fwcpp::InternalErrorCode;

TEST_CASE("InternalError starts with no errors recorded", "[internal_error]") {
    InternalError e;
    REQUIRE(e.errors() == 0);
    REQUIRE(e.count() == 0);
    REQUIRE(e.last_error_line() == 0);
    REQUIRE_FALSE(e.has_error(InternalErrorCode::constraining_nan));
}

TEST_CASE("InternalError::record sets the bit and remembers the line", "[internal_error]") {
    InternalError e;
    e.record(InternalErrorCode::constraining_nan, 42);
    REQUIRE(e.has_error(InternalErrorCode::constraining_nan));
    REQUIRE(e.count() == 1);
    REQUIRE(e.last_error_line() == 42);
}

TEST_CASE("InternalError::record accumulates distinct codes as a bitmask", "[internal_error]") {
    InternalError e;
    e.record(InternalErrorCode::constraining_nan, 1);
    e.record(InternalErrorCode::flow_of_control, 2);
    REQUIRE(e.has_error(InternalErrorCode::constraining_nan));
    REQUIRE(e.has_error(InternalErrorCode::flow_of_control));
    REQUIRE_FALSE(e.has_error(InternalErrorCode::bad_rotation));
    REQUIRE(e.count() == 2);
}

TEST_CASE("InternalError::record on a repeated code still increments count, mask stays one bit", "[internal_error]") {
    InternalError e;
    e.record(InternalErrorCode::constraining_nan, 1);
    e.record(InternalErrorCode::constraining_nan, 2);
    e.record(InternalErrorCode::constraining_nan, 3);
    REQUIRE(e.count() == 3); // every call counted...
    REQUIRE(e.errors() == static_cast<std::uint32_t>(InternalErrorCode::constraining_nan)); // ...but one bit
    REQUIRE(e.last_error_line() == 3); // most recent call site wins
}
