#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QuadPlane;

static QuadPlane available_qp() {
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    return qp;
}

TEST_CASE("mode_enter lean angle when available", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    qp.set_lean_angle_max_cd(4500);
    qp.mode_enter();
    REQUIRE(qp.lean_angle_max_cd() == 0);
}

TEST_CASE("mode_enter lean angle when unavailable", "[quadplane][poscontrol]") {
    QuadPlane qp{1};
    qp.set_lean_angle_max_cd(4500);
    qp.mode_enter();
    REQUIRE(qp.lean_angle_max_cd() == 4500);
}

TEST_CASE("mode_enter resets poscontrol state", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    qp.poscontrol_mut().state = PositionControlState::kApproach;
    qp.poscontrol_mut().correction_north_m = 12.f;
    qp.poscontrol_mut().mode_enter_cleared = false;
    qp.mode_enter();
    REQUIRE(qp.poscontrol().state == PositionControlState::kNone);
    REQUIRE(qp.poscontrol().correction_north_m == 0.f);
    REQUIRE(qp.poscontrol().mode_enter_cleared);
}

TEST_CASE("mode_enter guided wait takeoff snapshot", "[quadplane][poscontrol]") {
    auto qp = available_qp();
    qp.set_guided_wait_takeoff(true);
    qp.mode_enter();
    REQUIRE(qp.guided_wait_takeoff_on_mode_enter());
    qp.mode_enter();
    REQUIRE_FALSE(qp.guided_wait_takeoff_on_mode_enter());
}