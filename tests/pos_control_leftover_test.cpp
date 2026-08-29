// CCP-027 completeness catalog smoke tests.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/poscontrol/pos_control_leftover.hpp>

using namespace fwcpp::poscontrol;

TEST_CASE("poscontrol catalog counts", "[poscontrol][leftover][catalog]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() >= 9);
    REQUIRE(remaining_count() >= 5);
    REQUIRE(out_of_scope_count() >= 2);
    REQUIRE(pos_control_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());

    REQUIRE(completeness_has("NE_update_controller", PortStatus::kThisSlice));
    REQUIRE(completeness_has("yaw_from_ne_motion", PortStatus::kThisSlice));
    REQUIRE(completeness_has("get_thrust_vector", PortStatus::kThisSlice));
    REQUIRE(completeness_has("AP_SCRIPTING_ENABLED LUA offsets", PortStatus::kOutOfScope));
}
