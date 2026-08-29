// CPP-088 slice 1: in-memory Semaphore take/give and BinarySemaphore.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/semaphore.hpp>

using fwcpp::hal::BinarySemaphore;
using fwcpp::hal::Semaphore;
using fwcpp::hal::kSemaphoreBlockForeverMs;

TEST_CASE("semaphore take/give round-trip", "[hal][semaphore]") {
    Semaphore sem;
    REQUIRE(sem.depth() == 0);
    REQUIRE(sem.take(10));
    REQUIRE(sem.depth() == 1);
    REQUIRE(sem.give());
    REQUIRE(sem.depth() == 0);
}

TEST_CASE("semaphore take_nonblocking grants when free", "[hal][semaphore]") {
    Semaphore sem;
    REQUIRE(sem.take_nonblocking());
    REQUIRE(sem.depth() == 1);
    REQUIRE(sem.give());
}

TEST_CASE("semaphore is recursive and give fails when free", "[hal][semaphore]") {
    Semaphore sem;
    REQUIRE(sem.take(1));
    REQUIRE(sem.take_nonblocking());
    REQUIRE(sem.depth() == 2);
    REQUIRE(sem.give());
    REQUIRE(sem.depth() == 1);
    REQUIRE(sem.give());
    REQUIRE(sem.depth() == 0);
    REQUIRE_FALSE(sem.give());
}

TEST_CASE("semaphore take_nonblocking fails when contended", "[hal][semaphore]") {
    Semaphore sem;
    sem.set_contended(true);
    REQUIRE(sem.is_contended());
    REQUIRE_FALSE(sem.take_nonblocking());
    REQUIRE(sem.depth() == 0);
    REQUIRE_FALSE(sem.take(5));
    REQUIRE_FALSE(sem.take(kSemaphoreBlockForeverMs));
}

TEST_CASE("binary semaphore wait/signal is a pending flag", "[hal][semaphore]") {
    BinarySemaphore sem;
    REQUIRE_FALSE(sem.is_pending());
    REQUIRE_FALSE(sem.wait_nonblocking());
    sem.signal();
    REQUIRE(sem.is_pending());
    REQUIRE(sem.wait(10));
    REQUIRE_FALSE(sem.is_pending());
    sem.signal();
    sem.signal();
    REQUIRE(sem.wait_blocking());
    REQUIRE_FALSE(sem.wait_nonblocking());
}
