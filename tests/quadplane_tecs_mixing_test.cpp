#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_tecs_mixing.hpp>

using fwcpp::quadplane::InVtolLandDescentInputs;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::ShouldDisableTecsInputs;
using fwcpp::quadplane::compute_in_vtol_land_descent;
using fwcpp::quadplane::kMavCmdNavVtolLand;
using fwcpp::quadplane::should_disable_TECS;

TEST_CASE("land descent qrtl", "[quadplane][tecs]") {
    InVtolLandDescentInputs in{.control_is_qrtl = true, .pos_state = PositionControlState::kLandDescend};
    REQUIRE(compute_in_vtol_land_descent(in));
}

TEST_CASE("should_disable_TECS guided loiter", "[quadplane][tecs]") {
    ShouldDisableTecsInputs in{.control_is_guided = true, .auto_vtol_loiter = true};
    REQUIRE(should_disable_TECS(in));
}

TEST_CASE("QuadPlane should_disable_TECS wires land", "[quadplane][tecs]") {
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    qp.poscontrol_mut().state = PositionControlState::kLandFinal;
    ShouldDisableTecsInputs in{};
    in.land_descent.control_is_qrtl = true;
    in.land_descent.pos_state = qp.poscontrol().state;
    REQUIRE(qp.should_disable_tecs(in));
}
