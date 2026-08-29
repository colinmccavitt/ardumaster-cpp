#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane_leftover.hpp>
using namespace fwcpp::quadplane;
TEST_CASE("catalog", "[quadplane][leftover]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() >= 10);
    REQUIRE(remaining_count() >= 4);
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
    REQUIRE(completeness_has("land detector", PortStatus::kThisSlice));
    REQUIRE(completeness_has("takeoff_controller", PortStatus::kThisSlice));
    REQUIRE(completeness_has("control_auto", PortStatus::kThisSlice));
    REQUIRE(completeness_has("hold_stabilize / run_z_controller / multicopter_attitude_rate_update",
                             PortStatus::kThisSlice));
    REQUIRE(completeness_has("run_xy_controller / set_climb_rate_ms / assign_tilt_to_fwd_thr",
                             PortStatus::kThisSlice));
    REQUIRE(completeness_has("update_throttle_hover / update_throttle_suppression / update_throttle_mix",
                             PortStatus::kThisSlice));
    REQUIRE(completeness_has("HAL_GYROFFT", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("guided_start / guided_update / do_user_takeoff", PortStatus::kThisSlice));
    REQUIRE(completeness_has("landing_descent_rate_ms / abort_landing / update_land_positioning",
                             PortStatus::kRemaining));
    REQUIRE(completeness_has("assist_climb_rate_cms / weathervane yaw / is_flying_vtol",
                             PortStatus::kRemaining));
    REQUIRE(completeness_has("vtol_position_controller body", PortStatus::kRemaining));
    REQUIRE(completeness_has("verify_vtol_land body leftovers", PortStatus::kRemaining));
    REQUIRE(completeness_has("Log_Write_*", PortStatus::kOutOfScope));
}
