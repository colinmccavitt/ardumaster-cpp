// Unit tests for fwcpp::Result. Not a parity test - Result<T,E> has no
// upstream counterpart (ADR-0012 decision 3), so there is nothing to compare
// against. This proves the type's own contract instead.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/result.hpp>

#include <string>

using fwcpp::Err;
using fwcpp::Result;

namespace {
enum class TestError { BadInput, OutOfRange };
}

TEST_CASE("Result holds a value on the success path", "[result]") {
    Result<int, TestError> r(42);
    REQUIRE(r.has_value());
    REQUIRE(static_cast<bool>(r));
    REQUIRE(r.value() == 42);
}

TEST_CASE("Result holds an error on the failure path", "[result]") {
    Result<int, TestError> r = Err(TestError::OutOfRange);
    REQUIRE_FALSE(r.has_value());
    REQUIRE_FALSE(static_cast<bool>(r));
    REQUIRE(r.error() == TestError::OutOfRange);
}

TEST_CASE("value_or falls back only on the error path", "[result]") {
    Result<int, TestError> ok(7);
    Result<int, TestError> bad = Err(TestError::BadInput);
    REQUIRE(ok.value_or(-1) == 7);
    REQUIRE(bad.value_or(-1) == -1);
}

TEST_CASE("move construction transfers the active member correctly", "[result]") {
    Result<std::string, TestError> a(std::string("hello"));
    Result<std::string, TestError> b(std::move(a));
    REQUIRE(b.has_value());
    REQUIRE(b.value() == "hello");

    Result<std::string, TestError> c = Err(TestError::BadInput);
    Result<std::string, TestError> d(std::move(c));
    REQUIRE_FALSE(d.has_value());
    REQUIRE(d.error() == TestError::BadInput);
}

TEST_CASE("move assignment destroys the previously active member", "[result]") {
    Result<std::string, TestError> a(std::string("first"));
    Result<std::string, TestError> b = Err(TestError::OutOfRange);
    b = std::move(a);
    REQUIRE(b.has_value());
    REQUIRE(b.value() == "first");
}

TEST_CASE("void specialization carries only success/failure, no payload", "[result]") {
    Result<void, TestError> ok;
    Result<void, TestError> bad = Err(TestError::BadInput);
    REQUIRE(ok.has_value());
    REQUIRE_FALSE(bad.has_value());
    REQUIRE(bad.error() == TestError::BadInput);
}
