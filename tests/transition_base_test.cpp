#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane_transition/transition_base.hpp>

using fwcpp::quadplane_transition::TransitionBaseDefaults;
using fwcpp::quadplane_transition::TransitionState;
using fwcpp::quadplane_transition::kQOptionsLevelTransition;
using fwcpp::quadplane_transition::slt_allow_update_throttle_mix;
using fwcpp::quadplane_transition::slt_set_fw_roll_limit;
using fwcpp::quadplane_transition::slt_set_fw_roll_pitch;

TEST_CASE("transition base yaw default", "[transition][base]") {
    float yaw = 1.f;
    REQUIRE_FALSE(TransitionBaseDefaults::update_yaw_target(yaw));
}

TEST_CASE("slt throttle mix gate", "[transition][base]") {
    REQUIRE_FALSE(slt_allow_update_throttle_mix(TransitionState::kAirspeedWait, true));
    REQUIRE_FALSE(slt_allow_update_throttle_mix(TransitionState::kTimer, true));
    REQUIRE(slt_allow_update_throttle_mix(TransitionState::kDone, true));
}

TEST_CASE("slt level roll limit", "[transition][base]") {
    int32_t roll = 4500;
    REQUIRE(slt_set_fw_roll_limit(roll, TransitionState::kTimer, true, kQOptionsLevelTransition, 1500));
    REQUIRE(roll == 1500);
}

TEST_CASE("slt fw roll pitch envelope", "[transition][base]") {
    int32_t pitch = 8000;
    int32_t roll = 0;
    auto r = slt_set_fw_roll_pitch(pitch, roll, TransitionState::kAirspeedWait, false, false, true, 1.0f, 3.0f);
    REQUIRE(r.tecs_pitch_max_rad == 0.f);
    REQUIRE(pitch == 0);
    r = slt_set_fw_roll_pitch(pitch, roll, TransitionState::kTimer, false, false, true, 5.0f, 3.0f);
    REQUIRE(r.tecs_pitch_max_rad == 8.f);
}
