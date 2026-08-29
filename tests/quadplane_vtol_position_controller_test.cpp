#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_vtol_position_controller.hpp>

using fwcpp::quadplane::InVtolModeInputs;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::VtolPositionControllerInputs;

static QuadPlane available_qp() {
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    return qp;
}

TEST_CASE("vtol position controller no-op when unavailable", "[quadplane][vtol_pos]") {
    QuadPlane qp{1};
    const auto tick = qp.vtol_position_controller({});
    REQUIRE_FALSE(tick.ran);
}

TEST_CASE("vtol position controller none failsafe to position1", "[quadplane][vtol_pos]") {
    auto qp = available_qp();
    VtolPositionControllerInputs in{.now_ms = 9000, .armed_and_safety_off = true};
    in.in_vtol.available = true;
    const auto tick = qp.vtol_position_controller(in);
    REQUIRE(tick.ran);
    REQUIRE(tick.none_to_position1_failsafe);
    REQUIRE(qp.poscontrol().state == PositionControlState::kPosition1);
    REQUIRE(qp.poscontrol().last_run_ms == 9000);
}

TEST_CASE("vtol position controller approach nvtol failsafe", "[quadplane][vtol_pos]") {
    auto qp = available_qp();
    qp.poscontrol_mut().state = PositionControlState::kApproach;
    VtolPositionControllerInputs in{.now_ms = 1000, .armed_and_safety_off = true};
    in.in_vtol.available = true;
    in.in_vtol.control_is_vtol_mode = true;
    const auto tick = qp.vtol_position_controller(in);
    REQUIRE(tick.approach_nvtol_failsafe);
    REQUIRE(qp.poscontrol().state == PositionControlState::kPosition1);
}
