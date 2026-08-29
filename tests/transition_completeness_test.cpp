#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane_transition/quadplane_transition_completeness.hpp>

using fwcpp::quadplane_transition::PortStatus;
using fwcpp::quadplane_transition::completeness_has;
using fwcpp::quadplane_transition::on_main_count;
using fwcpp::quadplane_transition::quadplane_transition_completeness_size;
using fwcpp::quadplane_transition::remaining_count;
using fwcpp::quadplane_transition::this_slice_count;

TEST_CASE("quadplane transition catalog", "[transition][catalog]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 9);
    REQUIRE(remaining_count() == 4);
    REQUIRE(quadplane_transition_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + 1);
    REQUIRE(completeness_has("SLT_Transition::State enum", PortStatus::kThisSlice));
    REQUIRE(completeness_has("can_transition gate", PortStatus::kThisSlice));
    REQUIRE(completeness_has("update_airspeed_wait", PortStatus::kThisSlice));
}
