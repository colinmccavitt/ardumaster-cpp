#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/land_detector.hpp>

using fwcpp::copter::LandDetectorEffects;
using fwcpp::copter::LandDetectorInputs;
using fwcpp::copter::leftover_update_land_and_crash_detectors;
using fwcpp::copter::leftover_update_land_detector;
using fwcpp::copter::land_detector::PortStatus;
using fwcpp::copter::land_detector::completeness_has;
using fwcpp::copter::land_detector::completeness_size;
using fwcpp::copter::land_detector::on_main_count;
using fwcpp::copter::land_detector::out_of_scope_count;
using fwcpp::copter::land_detector::remaining_count;
using fwcpp::copter::land_detector::this_slice_count;

TEST_CASE("leftover_update_land_detector disarmed sets land_complete",
          "[copter][land_detector]") {
    LandDetectorInputs in{};
    in.motors_armed = false;
    in.land_complete = false;
    LandDetectorEffects fx{};
    leftover_update_land_detector(in, fx);
    REQUIRE(fx.land_complete);
    REQUIRE(fx.land_complete_set);
    REQUIRE_FALSE(fx.descent_check_inject);
    REQUIRE_FALSE(fx.throttle_check_inject);
}

TEST_CASE("leftover_update_land_detector armed uses descent/throttle injects",
          "[copter][land_detector]") {
    LandDetectorInputs in{};
    in.motors_armed = true;
    in.land_complete = false;
    in.descent_rate_low = true;
    in.throttle_zero = true;
    LandDetectorEffects fx{};
    leftover_update_land_detector(in, fx);
    REQUIRE_FALSE(fx.land_complete_set);
    REQUIRE_FALSE(fx.land_complete);
    REQUIRE(fx.descent_check_inject);
    REQUIRE(fx.throttle_check_inject);
}

TEST_CASE("leftover_update_land_and_crash_detectors filter + no crash body",
          "[copter][land_detector]") {
    LandDetectorInputs in{};
    in.motors_armed = false;
    in.accel_ef_z_plus_g = 9.80665f;
    LandDetectorEffects fx{};
    leftover_update_land_and_crash_detectors(in, fx);
    REQUIRE(fx.land_accel_filter_applied);
    REQUIRE(fx.update_land_detector_ran);
    REQUIRE(fx.land_complete_set);
    REQUIRE(fx.land_complete);
    REQUIRE(fx.accel_ef_z_plus_g == Catch::Approx(9.80665f));
    REQUIRE_FALSE(fx.crash_check_ran);
    REQUIRE_FALSE(fx.thrust_loss_check_ran);
    REQUIRE_FALSE(fx.yaw_imbalance_check_ran);
}

TEST_CASE("land_detector leftover catalog remaining_count",
          "[copter][land_detector][leftover]") {
    REQUIRE(remaining_count() == 4);
    REQUIRE(this_slice_count() == 6);
    REQUIRE(on_main_count() == 2);
    REQUIRE(out_of_scope_count() == 3);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_update_land_and_crash_detectors", PortStatus::kThisSlice));
    REQUIRE(completeness_has("leftover_update_land_detector", PortStatus::kThisSlice));
    REQUIRE(completeness_has("takeoff helpers", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Mode::_TakeOff::start_m", PortStatus::kThisSlice));
    REQUIRE(completeness_has("do_pilot_takeoff_ms body", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModeRTL", PortStatus::kOnMain));
    REQUIRE(completeness_has("ModeLand", PortStatus::kOnMain));
    REQUIRE(completeness_has("land_run_normal body", PortStatus::kRemaining));
    REQUIRE(completeness_has("crash_check / thrust_loss / yaw_imbalance", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("takeoff helpers", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("Mode::_TakeOff::start_m", PortStatus::kRemaining));
    REQUIRE_FALSE(completeness_has("do_pilot_takeoff_ms body", PortStatus::kRemaining));
}
