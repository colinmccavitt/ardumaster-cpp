#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane_leftover.hpp>
using namespace fwcpp::quadplane;
TEST_CASE("catalog", "[quadplane][leftover]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() >= 10);
    REQUIRE(remaining_count() >= 8);
    REQUIRE(completeness_has("setup / available / initialised", PortStatus::kThisSlice));
    REQUIRE(completeness_has("setup channels ahrs_view", PortStatus::kThisSlice));
    REQUIRE(completeness_has("wp_nav loiter_nav", PortStatus::kThisSlice));
    REQUIRE(completeness_has("motors_output motor_test", PortStatus::kThisSlice));
    REQUIRE(completeness_has("update transition FSM", PortStatus::kRemaining));
}
