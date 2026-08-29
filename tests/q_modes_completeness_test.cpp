#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_modes/q_modes_completeness.hpp>

using fwcpp::q_modes::PortStatus;
using fwcpp::q_modes::completeness_has;
using fwcpp::q_modes::on_main_count;
using fwcpp::q_modes::q_modes_completeness_size;
using fwcpp::q_modes::remaining_count;
using fwcpp::q_modes::this_slice_count;

TEST_CASE("q modes catalog", "[q_modes][catalog]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 11);
    REQUIRE(remaining_count() == 13);
    REQUIRE(q_modes_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + 1);
    REQUIRE(completeness_has("QStabilize run phase gate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("QHover run phase gate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("QAcro run phase gate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("hold_hover + climb rate", PortStatus::kRemaining));
}
