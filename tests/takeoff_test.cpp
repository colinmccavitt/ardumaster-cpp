#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/land_detector.hpp>
#include <fwcpp/copter/takeoff.hpp>

using fwcpp::copter::UserTakeoffEffects;
using fwcpp::copter::UserTakeoffInputs;
using fwcpp::copter::leftover_do_user_takeoff_U_m;
using fwcpp::copter::land_detector::PortStatus;
using fwcpp::copter::land_detector::completeness_has;
using fwcpp::copter::land_detector::remaining_count;
using fwcpp::copter::land_detector::this_slice_count;

namespace {

UserTakeoffInputs ok_inputs() {
    UserTakeoffInputs in{};
    in.motors_armed = true;
    in.land_complete = true;
    in.has_user_takeoff = true;
    in.takeoff_alt_m = 10.0f;
    in.current_alt_m = 0.0f;
    in.interlock = true;
    in.using_interlock = false;
    return in;
}

}  // namespace

TEST_CASE("leftover_do_user_takeoff_U_m success sets start_m and auto_armed",
          "[copter][takeoff]") {
    auto in = ok_inputs();
    UserTakeoffEffects fx{};
    REQUIRE(leftover_do_user_takeoff_U_m(in, fx));
    REQUIRE(fx.leftover_takeoff_start_m);
    REQUIRE(fx.set_auto_armed);
}

TEST_CASE("leftover_do_user_takeoff_U_m rejects !armed", "[copter][takeoff]") {
    auto in = ok_inputs();
    in.motors_armed = false;
    UserTakeoffEffects fx{};
    REQUIRE_FALSE(leftover_do_user_takeoff_U_m(in, fx));
    REQUIRE_FALSE(fx.leftover_takeoff_start_m);
    REQUIRE_FALSE(fx.set_auto_armed);
}

TEST_CASE("leftover_do_user_takeoff_U_m rejects !land_complete", "[copter][takeoff]") {
    auto in = ok_inputs();
    in.land_complete = false;
    UserTakeoffEffects fx{};
    REQUIRE_FALSE(leftover_do_user_takeoff_U_m(in, fx));
    REQUIRE_FALSE(fx.leftover_takeoff_start_m);
}

TEST_CASE("leftover_do_user_takeoff_U_m rejects !has_user_takeoff", "[copter][takeoff]") {
    auto in = ok_inputs();
    in.has_user_takeoff = false;
    UserTakeoffEffects fx{};
    REQUIRE_FALSE(leftover_do_user_takeoff_U_m(in, fx));
    REQUIRE_FALSE(fx.leftover_takeoff_start_m);
}

TEST_CASE("leftover_do_user_takeoff_U_m rejects takeoff_alt <= current_alt",
          "[copter][takeoff]") {
    auto in = ok_inputs();
    in.takeoff_alt_m = 5.0f;
    in.current_alt_m = 5.0f;
    UserTakeoffEffects fx{};
    REQUIRE_FALSE(leftover_do_user_takeoff_U_m(in, fx));
    REQUIRE_FALSE(fx.leftover_takeoff_start_m);

    in.takeoff_alt_m = 4.0f;
    REQUIRE_FALSE(leftover_do_user_takeoff_U_m(in, fx));
}

TEST_CASE("leftover_do_user_takeoff_U_m rejects interlock disabled when using",
          "[copter][takeoff]") {
    auto in = ok_inputs();
    in.using_interlock = true;
    in.interlock = false;
    UserTakeoffEffects fx{};
    REQUIRE_FALSE(leftover_do_user_takeoff_U_m(in, fx));
    REQUIRE_FALSE(fx.leftover_takeoff_start_m);
}

TEST_CASE("leftover_do_user_takeoff_U_m allows interlock enabled when using",
          "[copter][takeoff]") {
    auto in = ok_inputs();
    in.using_interlock = true;
    in.interlock = true;
    UserTakeoffEffects fx{};
    REQUIRE(leftover_do_user_takeoff_U_m(in, fx));
    REQUIRE(fx.leftover_takeoff_start_m);
    REQUIRE(fx.set_auto_armed);
}

TEST_CASE("takeoff helpers catalog moved to this slice", "[copter][takeoff][leftover]") {
    REQUIRE(remaining_count() == 4);
    REQUIRE(this_slice_count() == 4);
    REQUIRE(completeness_has("takeoff helpers", PortStatus::kThisSlice));
    REQUIRE_FALSE(completeness_has("takeoff helpers", PortStatus::kRemaining));
}
