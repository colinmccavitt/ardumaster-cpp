#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/failsafe.hpp>
#include <fwcpp/copter/failsafe_leftover.hpp>

using fwcpp::copter::FailsafeEffects;
using fwcpp::copter::FailsafeInputs;
using fwcpp::copter::leftover_failsafe_radio_check;
using fwcpp::copter::leftover_set_mode_rtl_or_land;
using fwcpp::copter::failsafe::PortStatus;
using fwcpp::copter::failsafe::completeness_has;
using fwcpp::copter::failsafe::completeness_size;
using fwcpp::copter::failsafe::on_main_count;
using fwcpp::copter::failsafe::out_of_scope_count;
using fwcpp::copter::failsafe::remaining_count;
using fwcpp::copter::failsafe::this_slice_count;

TEST_CASE("leftover_failsafe_radio_check disarmed ignores radio failsafe",
          "[copter][failsafe]") {
    FailsafeInputs in{};
    in.motors_armed = false;
    in.radio_failsafe = true;
    FailsafeEffects fx{};
    leftover_failsafe_radio_check(in, fx);
    REQUIRE_FALSE(fx.radio_failsafe_acted);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE_FALSE(fx.gcs_announce_radio_failsafe);
    REQUIRE_FALSE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_failsafe_radio_check armed without radio inject is quiet",
          "[copter][failsafe]") {
    FailsafeInputs in{};
    in.motors_armed = true;
    in.radio_failsafe = false;
    FailsafeEffects fx{};
    leftover_failsafe_radio_check(in, fx);
    REQUIRE_FALSE(fx.radio_failsafe_acted);
    REQUIRE_FALSE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE_FALSE(fx.gcs_announce_radio_failsafe);
}

TEST_CASE("leftover_failsafe_radio_check armed + radio inject sets RTL-or-land flags",
          "[copter][failsafe]") {
    FailsafeInputs in{};
    in.motors_armed = true;
    in.radio_failsafe = true;
    FailsafeEffects fx{};
    leftover_failsafe_radio_check(in, fx);
    REQUIRE(fx.radio_failsafe_acted);
    REQUIRE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE(fx.gcs_announce_radio_failsafe);
    REQUIRE(fx.notify_failsafe_mode_change);
}

TEST_CASE("leftover_set_mode_rtl_or_land sets mode-change flags only",
          "[copter][failsafe]") {
    FailsafeEffects fx{};
    leftover_set_mode_rtl_or_land(fx);
    REQUIRE(fx.leftover_set_mode_rtl_or_land);
    REQUIRE(fx.notify_failsafe_mode_change);
    REQUIRE_FALSE(fx.gcs_announce_radio_failsafe);
    REQUIRE_FALSE(fx.radio_failsafe_acted);
}

TEST_CASE("failsafe leftover catalog remaining_count",
          "[copter][failsafe][leftover]") {
    REQUIRE(remaining_count() == 6);
    REQUIRE(this_slice_count() == 3);
    REQUIRE(on_main_count() == 2);
    REQUIRE(out_of_scope_count() == 2);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_failsafe_radio_check", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_set_mode_rtl_or_land", PortStatus::kThisSlice));
    REQUIRE(completeness_has("failsafe_enable call site", PortStatus::kOnMain));
    REQUIRE(completeness_has("ModeRTL / ModeLand", PortStatus::kOnMain));
    REQUIRE(completeness_has("failsafe_radio_on_event override ladder", PortStatus::kRemaining));
    REQUIRE(completeness_has("failsafe_gcs_check / failsafe_gcs_on_event", PortStatus::kRemaining));
    REQUIRE(completeness_has("crash_check / thrust_loss / yaw_imbalance", PortStatus::kRemaining));
    REQUIRE(completeness_has("failsafe.cpp CPU watchdog", PortStatus::kRemaining));
    REQUIRE(completeness_has("GCS / Notify / logger objects", PortStatus::kOutOfScope));
    REQUIRE_FALSE(completeness_has("crash_check / thrust_loss / yaw_imbalance",
                                    PortStatus::kThisSlice));
}
