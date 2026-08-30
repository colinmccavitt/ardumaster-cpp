#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/mode_loiter.hpp>
#include <fwcpp/copter/mode_loiter_leftover.hpp>

using fwcpp::copter::AltHoldModeState;
using fwcpp::copter::ModeLoiter;
using fwcpp::copter::SpoolState;
using fwcpp::copter::loiter::PortStatus;
using fwcpp::copter::loiter::completeness_has;
using fwcpp::copter::loiter::completeness_size;
using fwcpp::copter::loiter::on_main_count;
using fwcpp::copter::loiter::out_of_scope_count;
using fwcpp::copter::loiter::remaining_count;
using fwcpp::copter::loiter::this_slice_count;

TEST_CASE("ModeLoiter leftover_init sets init call-site flags", "[copter][loiter]") {
    ModeLoiter mode;
    REQUIRE(mode.leftover_d_is_active);
    REQUIRE(mode.leftover_init(false));
    REQUIRE(mode.leftover_update_simple_mode);
    REQUIRE(mode.leftover_pilot_lean);
    REQUIRE(mode.leftover_loiter_set_pilot_accel);
    REQUIRE(mode.leftover_loiter_init_target);
    REQUIRE_FALSE(mode.leftover_d_init);
    REQUIRE(mode.leftover_d_set_max);
    REQUIRE(mode.mode_number() == fwcpp::copter::Mode::Number::LOITER);
    REQUIRE(mode.requires_position());
    REQUIRE_FALSE(mode.has_manual_throttle());
}

TEST_CASE("ModeLoiter leftover_init D_init when D inactive", "[copter][loiter]") {
    ModeLoiter mode;
    mode.leftover_d_is_active = false;
    REQUIRE(mode.leftover_init(true));
    REQUIRE(mode.leftover_update_simple_mode);
    REQUIRE(mode.leftover_pilot_lean);
    REQUIRE(mode.leftover_loiter_set_pilot_accel);
    REQUIRE(mode.leftover_loiter_init_target);
    REQUIRE(mode.leftover_d_init);
    REQUIRE(mode.leftover_d_set_max);
}

TEST_CASE("ModeLoiter init override calls leftover_init", "[copter][loiter]") {
    ModeLoiter mode;
    mode.leftover_d_is_active = false;
    REQUIRE(mode.init(false));
    REQUIRE(mode.leftover_d_init);
    REQUIRE(mode.leftover_d_set_max);
}

TEST_CASE("ModeLoiter leftover_run common path and MotorStopped flags", "[copter][loiter]") {
    ModeLoiter mode;
    // Default injects: !armed + SHUT_DOWN spool → MotorStopped.
    REQUIRE_FALSE(mode.leftover_run_called);
    mode.leftover_run();
    REQUIRE(mode.leftover_run_called);
    REQUIRE(mode.leftover_d_set_max);
    REQUIRE(mode.leftover_update_simple_mode);
    REQUIRE(mode.leftover_pilot_lean);
    REQUIRE(mode.leftover_loiter_set_pilot_accel);
    REQUIRE(mode.leftover_pilot_yaw);
    REQUIRE(mode.leftover_pilot_climb);
    REQUIRE_FALSE(mode.leftover_soften_for_landing);
    REQUIRE(mode.leftover_get_alt_hold_state);
    REQUIRE(mode.leftover_loiter_state == AltHoldModeState::MotorStopped);
    REQUIRE(mode.leftover_reset_I);
    REQUIRE(mode.leftover_reset_yaw_target_and_rate);
    REQUIRE(mode.leftover_d_relax);
    REQUIRE(mode.leftover_loiter_init_target);
    REQUIRE_FALSE(mode.leftover_reset_I_smoothly);
    REQUIRE_FALSE(mode.leftover_takeoff_start);
    REQUIRE_FALSE(mode.leftover_loiter_update);
    REQUIRE(mode.leftover_attitude);
    REQUIRE(mode.leftover_d_update_controller);
}

TEST_CASE("ModeLoiter leftover_run soften_for_landing when land_complete_maybe",
          "[copter][loiter]") {
    ModeLoiter mode;
    mode.leftover_land_complete_maybe = true;
    mode.leftover_run();
    REQUIRE(mode.leftover_soften_for_landing);
    REQUIRE(mode.leftover_d_set_max);
    REQUIRE(mode.leftover_get_alt_hold_state);
}

TEST_CASE("ModeLoiter leftover_run Landed_Ground_Idle flags", "[copter][loiter]") {
    ModeLoiter mode;
    mode.leftover_armed = false;
    mode.leftover_spool_state = SpoolState::GROUND_IDLE;
    mode.leftover_run();
    REQUIRE(mode.leftover_loiter_state == AltHoldModeState::Landed_Ground_Idle);
    REQUIRE(mode.leftover_reset_yaw_target_and_rate);
    REQUIRE(mode.leftover_reset_I_smoothly);
    REQUIRE(mode.leftover_loiter_init_target);
    REQUIRE(mode.leftover_d_relax);
    REQUIRE_FALSE(mode.leftover_reset_I);
    REQUIRE(mode.leftover_attitude);
    REQUIRE(mode.leftover_d_update_controller);
}

TEST_CASE("ModeLoiter leftover_run Landed_Pre_Takeoff flags", "[copter][loiter]") {
    ModeLoiter mode;
    mode.leftover_armed = false;
    mode.leftover_spool_state = SpoolState::THROTTLE_UNLIMITED;
    mode.leftover_run();
    REQUIRE(mode.leftover_loiter_state == AltHoldModeState::Landed_Pre_Takeoff);
    REQUIRE_FALSE(mode.leftover_reset_yaw_target_and_rate);
    REQUIRE(mode.leftover_reset_I_smoothly);
    REQUIRE(mode.leftover_loiter_init_target);
    REQUIRE(mode.leftover_d_relax);
    REQUIRE_FALSE(mode.leftover_reset_I);
}

TEST_CASE("ModeLoiter leftover_run Takeoff flags start when not running", "[copter][loiter]") {
    ModeLoiter mode;
    mode.leftover_armed = true;
    mode.leftover_takeoff_triggered = true;
    mode.leftover_takeoff_running = false;
    mode.leftover_run();
    REQUIRE(mode.leftover_loiter_state == AltHoldModeState::Takeoff);
    REQUIRE(mode.leftover_takeoff_start);
    REQUIRE(mode.leftover_avoidance);
    REQUIRE(mode.leftover_do_pilot_takeoff);
    REQUIRE(mode.leftover_loiter_update);
    REQUIRE_FALSE(mode.leftover_d_set_pos_from_climb);
    REQUIRE_FALSE(mode.leftover_surface_tracking);
    REQUIRE(mode.leftover_attitude);
}

TEST_CASE("ModeLoiter leftover_run Takeoff skips start when already running",
          "[copter][loiter]") {
    ModeLoiter mode;
    mode.leftover_armed = true;
    mode.leftover_takeoff_running = true;
    mode.leftover_run();
    REQUIRE(mode.leftover_loiter_state == AltHoldModeState::Takeoff);
    REQUIRE_FALSE(mode.leftover_takeoff_start);
    REQUIRE(mode.leftover_avoidance);
    REQUIRE(mode.leftover_do_pilot_takeoff);
    REQUIRE(mode.leftover_loiter_update);
}

TEST_CASE("ModeLoiter leftover_run Flying flags", "[copter][loiter]") {
    ModeLoiter mode;
    mode.leftover_armed = true;
    mode.leftover_auto_armed = true;
    mode.leftover_land_complete = false;
    mode.leftover_spool_state = SpoolState::THROTTLE_UNLIMITED;
    mode.leftover_run();
    REQUIRE(mode.leftover_loiter_state == AltHoldModeState::Flying);
    REQUIRE(mode.leftover_loiter_update);
    REQUIRE(mode.leftover_avoidance);
    REQUIRE(mode.leftover_surface_tracking);
    REQUIRE(mode.leftover_d_set_pos_from_climb);
    REQUIRE_FALSE(mode.leftover_takeoff_start);
    REQUIRE_FALSE(mode.leftover_loiter_init_target);
    REQUIRE_FALSE(mode.leftover_reset_I);
    REQUIRE(mode.leftover_attitude);
    REQUIRE(mode.leftover_d_update_controller);
}

TEST_CASE("ModeLoiter leftover_run clears stale state flags on re-entry",
          "[copter][loiter]") {
    ModeLoiter mode;
    mode.leftover_armed = true;
    mode.leftover_auto_armed = true;
    mode.leftover_land_complete = false;
    mode.leftover_spool_state = SpoolState::THROTTLE_UNLIMITED;
    mode.leftover_land_complete_maybe = true;
    mode.leftover_run();
    REQUIRE(mode.leftover_soften_for_landing);
    REQUIRE(mode.leftover_d_set_pos_from_climb);

    mode.leftover_land_complete_maybe = false;
    mode.leftover_armed = false;
    mode.leftover_spool_state = SpoolState::SHUT_DOWN;
    mode.leftover_run();
    REQUIRE_FALSE(mode.leftover_soften_for_landing);
    REQUIRE_FALSE(mode.leftover_d_set_pos_from_climb);
    REQUIRE(mode.leftover_loiter_state == AltHoldModeState::MotorStopped);
    REQUIRE(mode.leftover_reset_I);
}

TEST_CASE("ModeLoiter run override calls leftover_run", "[copter][loiter]") {
    ModeLoiter mode;
    mode.run();
    REQUIRE(mode.leftover_run_called);
    REQUIRE(mode.leftover_d_set_max);
    REQUIRE(mode.leftover_get_alt_hold_state);
    REQUIRE(mode.leftover_attitude);
}

TEST_CASE("loiter leftover catalog remaining_count", "[copter][loiter][leftover]") {
    REQUIRE(remaining_count() == 2);
    REQUIRE(this_slice_count() == 3);
    REQUIRE(on_main_count() == 0);
    REQUIRE(out_of_scope_count() == 3);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModeLoiter::init", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModeLoiter::run", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModePosHold", PortStatus::kRemaining));
    REQUIRE(completeness_has("ModeDrift", PortStatus::kRemaining));
    REQUIRE(completeness_has("precision_loiter", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("fence / avoidance", PortStatus::kOutOfScope));
    REQUIRE_FALSE(completeness_has("ModeLoiter::run", PortStatus::kRemaining));
}
