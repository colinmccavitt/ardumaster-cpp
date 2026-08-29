#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane_leftover.hpp>
using namespace fwcpp::quadplane;
TEST_CASE("catalog", "[quadplane][leftover]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() >= 8);
    REQUIRE(remaining_count() >= 10);
    REQUIRE(completeness_has("setup / available / initialised", PortStatus::kThisSlice));
    REQUIRE(completeness_has("update transition FSM", PortStatus::kRemaining));
}