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
    REQUIRE(this_slice_count() == 14);
    REQUIRE(remaining_count() == 2);
    REQUIRE(quadplane_transition_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + 1);
    REQUIRE(completeness_has("SLT_Transition::State enum", PortStatus::kThisSlice));
    REQUIRE(completeness_has("set_last_fw_pitch / assist reset on force", PortStatus::kThisSlice));
    REQUIRE(completeness_has("get_mav_vtol_state / TransitionPhase", PortStatus::kThisSlice));
}
