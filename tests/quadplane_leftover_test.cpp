#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane_leftover.hpp>
using namespace fwcpp::quadplane;
TEST_CASE("catalog", "[quadplane][leftover]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() >= 10);
    REQUIRE(remaining_count() == 2);
    REQUIRE(completeness_has("setup / available / initialised", PortStatus::kThisSlice));
    REQUIRE(completeness_has("setup channels ahrs_view", PortStatus::kThisSlice));
    REQUIRE(completeness_has("wp_nav loiter_nav", PortStatus::kThisSlice));
    REQUIRE(completeness_has("mode_enter poscontrol FSM", PortStatus::kThisSlice));
    REQUIRE(completeness_has("motors_output motor_test", PortStatus::kThisSlice));
    REQUIRE(completeness_has("update transition FSM", PortStatus::kThisSlice));
    REQUIRE(completeness_has("vtol controllers landing", PortStatus::kThisSlice));
    REQUIRE(completeness_has("guided in_vtol_mode", PortStatus::kThisSlice));
    REQUIRE(completeness_has("air_mode active latch", PortStatus::kThisSlice));
    REQUIRE(completeness_has("AUTO VTOL mission", PortStatus::kThisSlice));
    REQUIRE(completeness_has("TECS stick mixing", PortStatus::kThisSlice));
    REQUIRE(completeness_has("tailsitter tiltrotor", PortStatus::kThisSlice));
    REQUIRE(completeness_has("get_singleton", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("AP_Param var_info", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("hover/pilot-input", PortStatus::kThisSlice));
    REQUIRE(completeness_has("land detector", PortStatus::kRemaining));
    REQUIRE(completeness_has("takeoff_controller", PortStatus::kRemaining));
}
