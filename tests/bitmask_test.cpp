// Tests for fwcpp::Bitmask<N> (CPP-010).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/bitmask.hpp>

using fwcpp::Bitmask;
using fwcpp::InternalError;
using fwcpp::InternalErrorCode;

TEST_CASE("Bitmask starts all-clear", "[bitmask]") {
    Bitmask<40> b;
    REQUIRE(b.empty());
    REQUIRE(b.count() == 0);
    REQUIRE(b.first_set() == -1);
    REQUIRE(b.size() == 40);
}

TEST_CASE("Bitmask set/get/clear round-trip", "[bitmask]") {
    Bitmask<40> b;
    REQUIRE_FALSE(b.get(5));
    b.set(5);
    REQUIRE(b.get(5));
    REQUIRE_FALSE(b.empty());
    b.clear(5);
    REQUIRE_FALSE(b.get(5));
    REQUIRE(b.empty());
}

TEST_CASE("Bitmask spans multiple 32-bit words correctly", "[bitmask]") {
    Bitmask<40> b; // 2 words: bits 0-31, 32-39
    b.set(31);
    b.set(32);
    b.set(39);
    REQUIRE(b.get(31));
    REQUIRE(b.get(32));
    REQUIRE(b.get(39));
    REQUIRE(b.count() == 3);
}

TEST_CASE("Bitmask::setall sets exactly the valid bits, not the padding", "[bitmask]") {
    Bitmask<40> b; // 40 valid bits, but 2 words = 64 bits of storage
    b.setall();
    REQUIRE(b.count() == 40);
    for (std::uint16_t i = 0; i < 40; ++i) {
        REQUIRE(b.get(i));
    }
}

TEST_CASE("Bitmask::setonoff dispatches to set or clear", "[bitmask]") {
    Bitmask<10> b;
    b.setonoff(3, true);
    REQUIRE(b.get(3));
    b.setonoff(3, false);
    REQUIRE_FALSE(b.get(3));
}

TEST_CASE("Bitmask::first_set finds the lowest set bit across words", "[bitmask]") {
    Bitmask<40> b;
    b.set(35);
    b.set(10);
    REQUIRE(b.first_set() == 10);
}

TEST_CASE("Bitmask equality compares all words", "[bitmask]") {
    Bitmask<40> a;
    Bitmask<40> b;
    a.set(5);
    b.set(5);
    REQUIRE(a == b);
    b.set(6);
    REQUIRE(a != b);
}

TEST_CASE("Bitmask copy assignment copies all bits", "[bitmask]") {
    Bitmask<40> a;
    a.set(5);
    a.set(35);
    Bitmask<40> b;
    b = a;
    REQUIRE(b == a);
    REQUIRE(b.get(5));
    REQUIRE(b.get(35));
}

TEST_CASE("Bitmask array constructor sets the listed bits, skips out-of-range ones", "[bitmask]") {
    const std::uint16_t enabled[] = {2, 5, 999}; // 999 is out of range for a 10-bit mask
    Bitmask<10> b(enabled);
    REQUIRE(b.get(2));
    REQUIRE(b.get(5));
    REQUIRE(b.count() == 2); // 999 silently skipped, not reported (see file banner)
}

TEST_CASE("Bitmask out-of-range access returns a safe default with no sink configured", "[bitmask]") {
    Bitmask<10> b;
    REQUIRE_FALSE(b.get(100)); // pretend not set, not undefined behavior
    b.set(100);                // no-op, no crash
    REQUIRE(b.empty());
}

TEST_CASE("Bitmask out-of-range access reports through the configured InternalError sink", "[bitmask]") {
    Bitmask<10> b;
    InternalError err;
    b.set_error_sink(&err, 77);
    REQUIRE_FALSE(b.get(100));
    REQUIRE(err.has_error(InternalErrorCode::bitmask_range));
    REQUIRE(err.last_error_line() == 77);

    b.set(200); // also out of range
    REQUIRE(err.count() == 2);
}

TEST_CASE("Bitmask valid access never touches the sink", "[bitmask]") {
    Bitmask<10> b;
    InternalError err;
    b.set_error_sink(&err);
    b.set(3);
    REQUIRE(b.get(3));
    REQUIRE(err.count() == 0);
}
