#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_loiter/q_loiter_completeness.hpp>

using fwcpp::q_loiter::PortStatus;
using fwcpp::q_loiter::completeness_has;
using fwcpp::q_loiter::on_main_count;
using fwcpp::q_loiter::out_of_scope_count;
using fwcpp::q_loiter::q_loiter_completeness_size;
using fwcpp::q_loiter::remaining_count;
using fwcpp::q_loiter::this_slice_count;

TEST_CASE("q loiter completeness table", "[q_loiter][completeness]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 19);
    REQUIRE(remaining_count() == 0);
    REQUIRE(out_of_scope_count() == 1);
    REQUIRE(q_loiter_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("QLOITER update delegate QStabilize", PortStatus::kThisSlice));
    REQUIRE(completeness_has("QLAND update delegate QStabilize", PortStatus::kThisSlice));
    REQUIRE(completeness_has("QLOITER systemid att offset", PortStatus::kThisSlice));
    REQUIRE(completeness_has("LoiterAltQLand navigate hook", PortStatus::kThisSlice));
    REQUIRE(completeness_has("LoiterAltQLand handle_guided WP", PortStatus::kThisSlice));
    REQUIRE(completeness_has("QLAND landing gear IC engine cut", PortStatus::kThisSlice));
}
