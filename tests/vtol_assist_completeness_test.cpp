#include <catch2/catch_test_macros.hpp>
#include <fwcpp/vtol_assist/vtol_assist_completeness.hpp>

using fwcpp::vtol_assist::PortStatus;
using fwcpp::vtol_assist::completeness_has;
using fwcpp::vtol_assist::on_main_count;
using fwcpp::vtol_assist::remaining_count;
using fwcpp::vtol_assist::this_slice_count;
using fwcpp::vtol_assist::vtol_assist_completeness_size;

TEST_CASE("vtol assist catalog", "[vtol_assist][catalog]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 5);
    REQUIRE(remaining_count() == 8);
    REQUIRE(vtol_assist_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + 1);
    REQUIRE(completeness_has("enable/check gate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("speed assist trigger", PortStatus::kThisSlice));
    REQUIRE(completeness_has("check_VTOL_recovery", PortStatus::kRemaining));
}
