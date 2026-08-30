#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/mode_loiter.hpp>
#include <fwcpp/copter/mode_loiter_leftover.hpp>

using fwcpp::copter::ModeLoiter;
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

TEST_CASE("ModeLoiter leftover_run sets call-site flag only", "[copter][loiter]") {
    ModeLoiter mode;
    REQUIRE_FALSE(mode.leftover_run_called);
    mode.leftover_run();
    REQUIRE(mode.leftover_run_called);
}

TEST_CASE("ModeLoiter run override calls leftover_run", "[copter][loiter]") {
    ModeLoiter mode;
    mode.run();
    REQUIRE(mode.leftover_run_called);
}

TEST_CASE("loiter leftover catalog remaining_count", "[copter][loiter][leftover]") {
    REQUIRE(remaining_count() == 3);
    REQUIRE(this_slice_count() == 2);
    REQUIRE(on_main_count() == 0);
    REQUIRE(out_of_scope_count() == 3);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModeLoiter::init", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModeLoiter::run", PortStatus::kRemaining));
    REQUIRE(completeness_has("ModePosHold", PortStatus::kRemaining));
    REQUIRE(completeness_has("ModeDrift", PortStatus::kRemaining));
    REQUIRE(completeness_has("precision_loiter", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("fence / avoidance", PortStatus::kOutOfScope));
    REQUIRE_FALSE(completeness_has("ModeLoiter::run", PortStatus::kThisSlice));
}
