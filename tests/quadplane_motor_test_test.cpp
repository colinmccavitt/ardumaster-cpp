#include <catch2/catch_test_macros.hpp>

#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_motor_test.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>

using fwcpp::quadplane::MotorsOutputAction;
using fwcpp::quadplane::MotorsOutputView;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::motor_test_running;
using fwcpp::quadplane::motor_test_start;
using fwcpp::quadplane::motor_test_stop;

TEST_CASE("motor_test.running gates motors_output", "[quadplane][motor_test]") {
    QuadPlane qp(1);
    REQUIRE(qp.setup());
    REQUIRE_FALSE(qp.motor_test_running());

    MotorsOutputView view{};
    view.armed_and_safety_off = true;
    view.now_ms = 10;
    REQUIRE(qp.motors_output(view).action == MotorsOutputAction::kOutput);

    REQUIRE(qp.start_motor_test());
    REQUIRE(qp.motor_test_running());
    REQUIRE(qp.motor_test_state().running);
    const auto gated = qp.motors_output(view);
    REQUIRE(gated.action == MotorsOutputAction::kMotorTest);
    REQUIRE(gated.motors_output_ran);

    qp.stop_motor_test();
    REQUIRE_FALSE(qp.motor_test_running());
    REQUIRE(qp.motors_output(view).action == MotorsOutputAction::kOutput);
}

TEST_CASE("motor_test.running skips update transition and tiltrotor", "[quadplane][motor_test]") {
    QuadPlane qp(1);
    qp.set_tilt_enable(1);
    REQUIRE(qp.setup());
    REQUIRE(qp.start_motor_test());
    const auto tick = qp.update(fwcpp::quadplane::QuadPlaneUpdateView{
        .now_ms = 1000,
        .armed_and_safety_off = true,
        .in_vtol_mode = false,
    });
    REQUIRE_FALSE(tick.ran_transition_update);
    REQUIRE_FALSE(tick.ran_tiltrotor_update);
}

TEST_CASE("motor_test start requires available", "[quadplane][motor_test]") {
    QuadPlane qp(0);
    REQUIRE_FALSE(qp.start_motor_test());
    REQUIRE_FALSE(qp.motor_test_running());
}

TEST_CASE("motor_test helpers latch running flag", "[quadplane][motor_test]") {
    fwcpp::quadplane::MotorTestState state{};
    REQUIRE_FALSE(motor_test_running(state));
    REQUIRE(motor_test_start(state));
    REQUIRE(motor_test_running(state));
    motor_test_stop(state);
    REQUIRE_FALSE(motor_test_running(state));
}
