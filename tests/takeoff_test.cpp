#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/copter/land_detector.hpp>
#include <fwcpp/copter/takeoff.hpp>

using fwcpp::copter::PilotTakeoffEffects;
using fwcpp::copter::PilotTakeoffInputs;
using fwcpp::copter::TakeOffState;
using fwcpp::copter::UserTakeoffEffects;
using fwcpp::copter::UserTakeoffInputs;
using fwcpp::copter::leftover_do_pilot_takeoff_ms;
using fwcpp::copter::leftover_do_user_takeoff_U_m;
using fwcpp::copter::leftover_takeoff_start_m;
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

TEST_CASE("leftover_takeoff_start_m sets running start and complete alt",
          "[copter][takeoff]") {
    TakeOffState st{};
    leftover_takeoff_start_m(st, 5.0f, 12.0f);
    REQUIRE(st._running);
    REQUIRE(st.start_alt == Catch::Approx(12.0f));
    REQUIRE(st.complete_alt == Catch::Approx(17.0f));
}

TEST_CASE("leftover_do_user_takeoff_U_m success sets start_m and auto_armed",
          "[copter][takeoff]") {
    auto in = ok_inputs();
    UserTakeoffEffects fx{};
    REQUIRE(leftover_do_user_takeoff_U_m(in, fx));
    REQUIRE(fx.leftover_takeoff_start_m);
    REQUIRE(fx.set_auto_armed);
}

TEST_CASE("leftover_do_user_takeoff_U_m success wires start_m when state given",
          "[copter][takeoff]") {
    auto in = ok_inputs();
    in.takeoff_alt_m = 8.0f;
    UserTakeoffEffects fx{};
    TakeOffState st{};
    REQUIRE(leftover_do_user_takeoff_U_m(in, fx, &st, 3.5f));
    REQUIRE(fx.leftover_takeoff_start_m);
    REQUIRE(fx.set_auto_armed);
    REQUIRE(st._running);
    REQUIRE(st.start_alt == Catch::Approx(3.5f));
    REQUIRE(st.complete_alt == Catch::Approx(11.5f));
}

TEST_CASE("leftover_do_user_takeoff_U_m success without state leaves TakeOffState",
          "[copter][takeoff]") {
    auto in = ok_inputs();
    UserTakeoffEffects fx{};
    TakeOffState st{};
    REQUIRE(leftover_do_user_takeoff_U_m(in, fx));
    REQUIRE(fx.leftover_takeoff_start_m);
    REQUIRE_FALSE(st._running);
    REQUIRE(st.start_alt == Catch::Approx(0.0f));
    REQUIRE(st.complete_alt == Catch::Approx(0.0f));
}

TEST_CASE("leftover_do_user_takeoff_U_m rejects !armed", "[copter][takeoff]") {
    auto in = ok_inputs();
    in.motors_armed = false;
    UserTakeoffEffects fx{};
    TakeOffState st{};
    REQUIRE_FALSE(leftover_do_user_takeoff_U_m(in, fx, &st, 1.0f));
    REQUIRE_FALSE(fx.leftover_takeoff_start_m);
    REQUIRE_FALSE(fx.set_auto_armed);
    REQUIRE_FALSE(st._running);
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
    TakeOffState st{};
    REQUIRE(leftover_do_user_takeoff_U_m(in, fx, &st, 2.0f));
    REQUIRE(fx.leftover_takeoff_start_m);
    REQUIRE(fx.set_auto_armed);
    REQUIRE(st._running);
    REQUIRE(st.start_alt == Catch::Approx(2.0f));
    REQUIRE(st.complete_alt == Catch::Approx(12.0f));
}

TEST_CASE("leftover_do_pilot_takeoff_ms returns when not running",
          "[copter][takeoff]") {
    TakeOffState st{};
    PilotTakeoffInputs in{};
    in.land_complete = true;
    in.clear_land_complete = true;
    PilotTakeoffEffects fx{};
    leftover_do_pilot_takeoff_ms(st, in, fx);
    REQUIRE_FALSE(fx.set_throttle_out);
    REQUIRE_FALSE(fx.D_init_controller);
    REQUIRE_FALSE(fx.set_land_complete_false);
    REQUIRE_FALSE(fx.input_pos_vel_accel_D);
    REQUIRE_FALSE(fx.takeoff_stopped);
    REQUIRE_FALSE(st._running);
}

TEST_CASE("leftover_do_pilot_takeoff_ms land_complete sets throttle and D_init",
          "[copter][takeoff]") {
    TakeOffState st{};
    leftover_takeoff_start_m(st, 10.0f, 0.0f);
    PilotTakeoffInputs in{};
    in.land_complete = true;
    in.clear_land_complete = false;
    PilotTakeoffEffects fx{};
    leftover_do_pilot_takeoff_ms(st, in, fx);
    REQUIRE(fx.set_throttle_out);
    REQUIRE(fx.D_init_controller);
    REQUIRE_FALSE(fx.set_land_complete_false);
    REQUIRE_FALSE(fx.input_pos_vel_accel_D);
    REQUIRE(st._running);
}

TEST_CASE("leftover_do_pilot_takeoff_ms land_complete clear inject clears land",
          "[copter][takeoff]") {
    TakeOffState st{};
    leftover_takeoff_start_m(st, 10.0f, 1.0f);
    PilotTakeoffInputs in{};
    in.land_complete = true;
    in.clear_land_complete = true;
    PilotTakeoffEffects fx{};
    leftover_do_pilot_takeoff_ms(st, in, fx);
    REQUIRE(fx.set_throttle_out);
    REQUIRE(fx.D_init_controller);
    REQUIRE(fx.set_land_complete_false);
    REQUIRE(st._running);
}

TEST_CASE("leftover_do_pilot_takeoff_ms climb path commands pos_vel",
          "[copter][takeoff]") {
    TakeOffState st{};
    leftover_takeoff_start_m(st, 10.0f, 0.0f);
    PilotTakeoffInputs in{};
    in.land_complete = false;
    in.pilot_climb_rate_ms = 2.0f;
    in.pos_desired_U_m = 1.0f;  // far below complete_alt
    PilotTakeoffEffects fx{};
    leftover_do_pilot_takeoff_ms(st, in, fx);
    REQUIRE(fx.input_pos_vel_accel_D);
    REQUIRE_FALSE(fx.set_throttle_out);
    REQUIRE_FALSE(fx.D_init_controller);
    REQUIRE_FALSE(fx.takeoff_stopped);
    REQUIRE(st._running);
}

TEST_CASE("leftover_do_pilot_takeoff_ms stops on negative climb rate",
          "[copter][takeoff]") {
    TakeOffState st{};
    leftover_takeoff_start_m(st, 10.0f, 0.0f);
    PilotTakeoffInputs in{};
    in.land_complete = false;
    in.pilot_climb_rate_ms = -0.1f;
    in.pos_desired_U_m = 1.0f;
    PilotTakeoffEffects fx{};
    leftover_do_pilot_takeoff_ms(st, in, fx);
    REQUIRE(fx.input_pos_vel_accel_D);
    REQUIRE(fx.takeoff_stopped);
    REQUIRE_FALSE(st._running);
}

TEST_CASE("leftover_do_pilot_takeoff_ms stops when near complete alt",
          "[copter][takeoff]") {
    TakeOffState st{};
    leftover_takeoff_start_m(st, 10.0f, 0.0f);  // complete_alt = 10
    PilotTakeoffInputs in{};
    in.land_complete = false;
    in.pilot_climb_rate_ms = 1.0f;
    // (complete - start) * 0.999 = 9.99; pos_desired - start = 9.995 → stop
    in.pos_desired_U_m = 9.995f;
    PilotTakeoffEffects fx{};
    leftover_do_pilot_takeoff_ms(st, in, fx);
    REQUIRE(fx.input_pos_vel_accel_D);
    REQUIRE(fx.takeoff_stopped);
    REQUIRE_FALSE(st._running);
}

TEST_CASE("do_pilot_takeoff_ms catalog moved to this slice",
          "[copter][takeoff][leftover]") {
    REQUIRE(remaining_count() == 4);
    REQUIRE(this_slice_count() == 6);
    REQUIRE(completeness_has("takeoff helpers", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Mode::_TakeOff::start_m", PortStatus::kThisSlice));
    REQUIRE(completeness_has("do_pilot_takeoff_ms body", PortStatus::kThisSlice));
    REQUIRE_FALSE(completeness_has("do_pilot_takeoff_ms body", PortStatus::kRemaining));
}
