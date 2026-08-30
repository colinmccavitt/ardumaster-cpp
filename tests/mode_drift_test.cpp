#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/mode_drift.hpp>
#include <fwcpp/copter/mode_loiter_leftover.hpp>

using fwcpp::copter::DesiredSpoolState;
using fwcpp::copter::ModeDrift;
using fwcpp::copter::SpoolState;
using fwcpp::copter::loiter::PortStatus;
using fwcpp::copter::loiter::completeness_has;
using fwcpp::copter::loiter::completeness_size;
using fwcpp::copter::loiter::on_main_count;
using fwcpp::copter::loiter::out_of_scope_count;
using fwcpp::copter::loiter::remaining_count;
using fwcpp::copter::loiter::this_slice_count;

TEST_CASE("ModeDrift leftover_init returns true", "[copter][drift]") {
    ModeDrift mode;
    REQUIRE(mode.leftover_init(false));
    REQUIRE(mode.leftover_init(true));
    REQUIRE(mode.mode_number() == fwcpp::copter::Mode::Number::DRIFT);
    REQUIRE(mode.requires_position());
    REQUIRE_FALSE(mode.has_manual_throttle());
}

TEST_CASE("ModeDrift init override calls leftover_init", "[copter][drift]") {
    ModeDrift mode;
    REQUIRE(mode.init(false));
}

TEST_CASE("ModeDrift leftover_run sets lean/vel and spool/attitude flags",
          "[copter][drift]") {
    ModeDrift mode;
    REQUIRE_FALSE(mode.leftover_run_called);
    mode.leftover_run();
    REQUIRE(mode.leftover_run_called);
    REQUIRE(mode.leftover_pilot_lean);
    REQUIRE(mode.leftover_get_vel_ned);
    REQUIRE(mode.leftover_set_desired_spool);
    REQUIRE(mode.leftover_desired_spool == DesiredSpoolState::GROUND_IDLE);
    REQUIRE(mode.leftover_reset_yaw_target_and_rate);
    REQUIRE(mode.leftover_reset_I);
    REQUIRE_FALSE(mode.leftover_reset_I_smoothly);
    REQUIRE_FALSE(mode.leftover_clear_land_complete);
    REQUIRE(mode.leftover_attitude);
    REQUIRE(mode.leftover_throttle_assist);
    REQUIRE(mode.leftover_set_throttle_out);
}

TEST_CASE("ModeDrift leftover_run throttle unlimited spool desired",
          "[copter][drift]") {
    ModeDrift mode;
    mode.leftover_throttle_zero = false;
    mode.leftover_spool_state = SpoolState::THROTTLE_UNLIMITED;
    mode.leftover_run();
    REQUIRE(mode.leftover_desired_spool == DesiredSpoolState::THROTTLE_UNLIMITED);
    REQUIRE_FALSE(mode.leftover_reset_yaw_target_and_rate);
    REQUIRE_FALSE(mode.leftover_reset_I);
    REQUIRE(mode.leftover_clear_land_complete);
}

TEST_CASE("ModeDrift leftover_run Ground_Idle resets I smoothly",
          "[copter][drift]") {
    ModeDrift mode;
    mode.leftover_spool_state = SpoolState::GROUND_IDLE;
    mode.leftover_run();
    REQUIRE(mode.leftover_reset_yaw_target_and_rate);
    REQUIRE(mode.leftover_reset_I_smoothly);
    REQUIRE_FALSE(mode.leftover_reset_I);
}

TEST_CASE("ModeDrift leftover_run THROTTLE_UNLIMITED keeps land when lower limit",
          "[copter][drift]") {
    ModeDrift mode;
    mode.leftover_throttle_zero = false;
    mode.leftover_spool_state = SpoolState::THROTTLE_UNLIMITED;
    mode.leftover_throttle_lower_limit = true;
    mode.leftover_run();
    REQUIRE_FALSE(mode.leftover_clear_land_complete);
}

TEST_CASE("ModeDrift leftover_run clears stale flags on re-entry",
          "[copter][drift]") {
    ModeDrift mode;
    mode.leftover_spool_state = SpoolState::SHUT_DOWN;
    mode.leftover_run();
    REQUIRE(mode.leftover_reset_I);

    mode.leftover_spool_state = SpoolState::SPOOLING_UP;
    mode.leftover_run();
    REQUIRE(mode.leftover_run_called);
    REQUIRE(mode.leftover_pilot_lean);
    REQUIRE(mode.leftover_get_vel_ned);
    REQUIRE_FALSE(mode.leftover_reset_I);
    REQUIRE_FALSE(mode.leftover_reset_yaw_target_and_rate);
    REQUIRE(mode.leftover_attitude);
}

TEST_CASE("ModeDrift run override calls leftover_run", "[copter][drift]") {
    ModeDrift mode;
    mode.run();
    REQUIRE(mode.leftover_run_called);
    REQUIRE(mode.leftover_pilot_lean);
    REQUIRE(mode.leftover_get_vel_ned);
    REQUIRE(mode.leftover_attitude);
}

TEST_CASE("drift leftover catalog remaining_count", "[copter][drift][leftover]") {
    REQUIRE(remaining_count() == 1);
    REQUIRE(this_slice_count() == 4);
    REQUIRE(on_main_count() == 2);
    REQUIRE(out_of_scope_count() == 3);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModeLoiter::init", PortStatus::kOnMain));
    REQUIRE(completeness_has("ModeLoiter::run", PortStatus::kOnMain));
    REQUIRE(completeness_has("ModePosHold::init", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModePosHold::run", PortStatus::kRemaining));
    REQUIRE(completeness_has("ModeDrift::init", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModeDrift::run", PortStatus::kThisSlice));
    REQUIRE(completeness_has("precision_loiter", PortStatus::kOutOfScope));
    REQUIRE_FALSE(completeness_has("ModeDrift", PortStatus::kRemaining));
}
