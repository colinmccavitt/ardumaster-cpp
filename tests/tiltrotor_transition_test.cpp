#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/quadplane_transition/transition_state.hpp>
#include <fwcpp/tiltrotor/tiltrotor_transition.hpp>

using fwcpp::quadplane_transition::TransitionState;
using fwcpp::tiltrotor::MotorThrustSample;
using fwcpp::tiltrotor::TiltrotorTransitionView;
using fwcpp::tiltrotor::YawTargetSample;
using fwcpp::tiltrotor::YawTargetState;
using fwcpp::tiltrotor::allow_vfwd;
using fwcpp::tiltrotor::show_vtol_view;
using fwcpp::tiltrotor::transition_update_yaw_target;
using fwcpp::tiltrotor::update_yaw_target;
using fwcpp::tiltrotor::use_multirotor_control_in_fwd_transition;

TEST_CASE("tiltrotor transition helpers", "[tiltrotor][transition]") {
    TiltrotorTransitionView view{};
    view.is_vectored = true;
    view.transition_state = TransitionState::kTimer;
    REQUIRE(use_multirotor_control_in_fwd_transition(view));
    REQUIRE(show_vtol_view(false, view));

    view.transition_state = TransitionState::kAirspeedWait;
    REQUIRE(use_multirotor_control_in_fwd_transition(view));
    REQUIRE(show_vtol_view(false, view));

    view.transition_state = TransitionState::kDone;
    REQUIRE_FALSE(use_multirotor_control_in_fwd_transition(view));
    REQUIRE_FALSE(show_vtol_view(false, view));
    REQUIRE(show_vtol_view(true, view));

    view.is_vectored = false;
    view.transition_state = TransitionState::kTimer;
    REQUIRE_FALSE(use_multirotor_control_in_fwd_transition(view));
    REQUIRE_FALSE(show_vtol_view(false, view));

    MotorThrustSample motors{};
    motors.thrust_boost = true;
    motors.roll_factor = 0.5f;
    REQUIRE(allow_vfwd(false, motors, true));
    REQUIRE(allow_vfwd(true, motors, false));
    REQUIRE_FALSE(allow_vfwd(true, motors, true));
    motors.roll_factor = 0.0f;
    REQUIRE(allow_vfwd(true, motors, true));
    motors.thrust_boost = false;
    REQUIRE(allow_vfwd(true, motors, true));

    auto yaw = update_yaw_target({}, YawTargetSample{.now_ms = 200, .ahrs_yaw_sensor_cd = 1000.0f});
    REQUIRE(yaw.transition_yaw_cd == 1000.0f);

    view.is_vectored = true;
    view.transition_state = TransitionState::kTimer;
    YawTargetState state{};
    float yaw_target_cd = 0.0f;
    REQUIRE(transition_update_yaw_target(view, state, YawTargetSample{.now_ms = 200, .ahrs_yaw_sensor_cd = 2500.0f},
                                         yaw_target_cd));
    REQUIRE_THAT(yaw_target_cd, Catch::Matchers::WithinAbs(2500.0f, 1e-6f));

    view.transition_state = TransitionState::kDone;
    REQUIRE_FALSE(transition_update_yaw_target(view, state, {}, yaw_target_cd));
}
