#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/mode_loiter_leftover.hpp>
#include <fwcpp/copter/mode_poshold.hpp>

using fwcpp::copter::ModePosHold;
using fwcpp::copter::PosHoldRpMode;
using fwcpp::copter::loiter::PortStatus;
using fwcpp::copter::loiter::completeness_has;
using fwcpp::copter::loiter::completeness_size;
using fwcpp::copter::loiter::on_main_count;
using fwcpp::copter::loiter::out_of_scope_count;
using fwcpp::copter::loiter::remaining_count;
using fwcpp::copter::loiter::this_slice_count;

TEST_CASE("ModePosHold leftover_init sets init call-site flags landed",
          "[copter][poshold]") {
    ModePosHold mode;
    REQUIRE(mode.leftover_d_is_active);
    REQUIRE(mode.leftover_land_complete);
    REQUIRE(mode.leftover_init(false));
    REQUIRE(mode.leftover_d_set_max);
    REQUIRE_FALSE(mode.leftover_d_init);
    REQUIRE(mode.leftover_zero_pilot_lean);
    REQUIRE(mode.leftover_brake_gain);
    REQUIRE(mode.leftover_roll_mode == PosHoldRpMode::Loiter);
    REQUIRE(mode.leftover_pitch_mode == PosHoldRpMode::Loiter);
    REQUIRE(mode.leftover_loiter_clear_pilot_accel);
    REQUIRE(mode.leftover_loiter_init_target);
    REQUIRE(mode.leftover_init_wind_comp);
    REQUIRE(mode.mode_number() == fwcpp::copter::Mode::Number::POSHOLD);
    REQUIRE(mode.requires_position());
    REQUIRE_FALSE(mode.has_manual_throttle());
}

TEST_CASE("ModePosHold leftover_init D_init when D inactive", "[copter][poshold]") {
    ModePosHold mode;
    mode.leftover_d_is_active = false;
    REQUIRE(mode.leftover_init(true));
    REQUIRE(mode.leftover_d_set_max);
    REQUIRE(mode.leftover_d_init);
    REQUIRE(mode.leftover_zero_pilot_lean);
    REQUIRE(mode.leftover_brake_gain);
    REQUIRE(mode.leftover_loiter_clear_pilot_accel);
    REQUIRE(mode.leftover_loiter_init_target);
    REQUIRE(mode.leftover_init_wind_comp);
}

TEST_CASE("ModePosHold leftover_init airborne starts pilot override",
          "[copter][poshold]") {
    ModePosHold mode;
    mode.leftover_land_complete = false;
    REQUIRE(mode.leftover_init(false));
    REQUIRE(mode.leftover_roll_mode == PosHoldRpMode::PilotOverride);
    REQUIRE(mode.leftover_pitch_mode == PosHoldRpMode::PilotOverride);
    REQUIRE(mode.leftover_d_set_max);
    REQUIRE(mode.leftover_loiter_init_target);
    REQUIRE(mode.leftover_init_wind_comp);
}

TEST_CASE("ModePosHold init override calls leftover_init", "[copter][poshold]") {
    ModePosHold mode;
    mode.leftover_d_is_active = false;
    mode.leftover_land_complete = false;
    REQUIRE(mode.init(false));
    REQUIRE(mode.leftover_d_init);
    REQUIRE(mode.leftover_d_set_max);
    REQUIRE(mode.leftover_roll_mode == PosHoldRpMode::PilotOverride);
}

TEST_CASE("ModePosHold leftover_run sets call-site flag only", "[copter][poshold]") {
    ModePosHold mode;
    REQUIRE_FALSE(mode.leftover_run_called);
    mode.leftover_run();
    REQUIRE(mode.leftover_run_called);
}

TEST_CASE("ModePosHold run override calls leftover_run", "[copter][poshold]") {
    ModePosHold mode;
    mode.run();
    REQUIRE(mode.leftover_run_called);
}

TEST_CASE("poshold leftover catalog remaining_count", "[copter][poshold][leftover]") {
    REQUIRE(remaining_count() == 2);
    REQUIRE(this_slice_count() == 2);
    REQUIRE(on_main_count() == 2);
    REQUIRE(out_of_scope_count() == 3);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModeLoiter::init", PortStatus::kOnMain));
    REQUIRE(completeness_has("ModeLoiter::run", PortStatus::kOnMain));
    REQUIRE(completeness_has("ModePosHold::init", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModePosHold::run", PortStatus::kRemaining));
    REQUIRE(completeness_has("ModeDrift", PortStatus::kRemaining));
    REQUIRE(completeness_has("precision_loiter", PortStatus::kOutOfScope));
    REQUIRE_FALSE(completeness_has("ModePosHold", PortStatus::kRemaining));
}
