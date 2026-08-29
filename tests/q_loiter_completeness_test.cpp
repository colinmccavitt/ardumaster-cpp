#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_loiter/q_loiter_completeness.hpp>

using fwcpp::q_loiter::PortStatus;
using fwcpp::q_loiter::completeness_has;
using fwcpp::q_loiter::q_loiter_completeness_size;
using fwcpp::q_loiter::this_slice_count;

TEST_CASE("q loiter completeness table", "[q_loiter][completeness]") {
    REQUIRE(q_loiter_completeness_size() >= 10);
    REQUIRE(this_slice_count() >= 10);
    REQUIRE(completeness_has("QLAND run delegates QLOITER", PortStatus::kThisSlice));
}
