#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane_transition/transition_update_loop.hpp>

using fwcpp::quadplane_transition::DesiredSpoolState;
using fwcpp::quadplane_transition::SltTransition;
using fwcpp::quadplane_transition::SltUpdateInputs;
using fwcpp::quadplane_transition::TransitionState;
using fwcpp::quadplane_transition::run_slt_update;

TEST_CASE("slt update requests synthetic airspeed in transition", "[transition][update_loop]") {
    SltTransition fsm = SltTransition::with_defaults();
    SltUpdateInputs in{};
    in.now_ms = 100;
    in.should_assist = true;
    in.have_airspeed = true;
    in.airspeed_ms = 5.f;
    in.airspeed_min_ms = 10.f;
    auto out = run_slt_update(fsm, in);
    REQUIRE(out.use_synthetic_airspeed);
    REQUIRE(out.hold_hover);
    REQUIRE(out.set_throttle_mix_max);
    REQUIRE(out.reset_pitch_roll_i);
}

TEST_CASE("slt update done shuts down spool", "[transition][update_loop]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.mark_transition_done();
    SltUpdateInputs in{};
    in.now_ms = 1;
    auto out = run_slt_update(fsm, in);
    REQUIRE_FALSE(out.use_synthetic_airspeed);
    REQUIRE(out.spool == DesiredSpoolState::kShutDown);
    REQUIRE(out.call_motors_output);
    REQUIRE(out.stamp_last_fw_pitch);
}

TEST_CASE("slt update timer throttle mix scales", "[transition][update_loop]") {
    SltTransition fsm = SltTransition::with_defaults();
    fsm.set_transition_time_ms(1000);
    fsm.update_airspeed_wait(0, true, 20.f, 10.f, false);
    fsm.record_motor_throttle(1.0f);
    SltUpdateInputs in{};
    in.now_ms = 500;
    in.tilt_fwd_complete = false;
    auto out = run_slt_update(fsm, in);
    REQUIRE(out.hold_stabilize);
    REQUIRE(out.hold_stabilize_throttle >= 0.01f);
    REQUIRE(out.attitude_throttle_mix == Catch::Approx(0.25f));
    REQUIRE(fsm.state() == TransitionState::kTimer);
}
