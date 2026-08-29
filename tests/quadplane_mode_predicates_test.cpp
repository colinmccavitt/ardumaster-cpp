#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_mode_predicates.hpp>

using fwcpp::quadplane::InVtolModeInputs;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::compute_in_vtol_mode;

static InVtolModeInputs base_available() {
    return InVtolModeInputs{.available = true};
}

TEST_CASE("in_vtol_mode unavailable", "[quadplane][predicates]") {
    REQUIRE_FALSE(compute_in_vtol_mode({}));
}

TEST_CASE("in_vtol_mode vtol control mode", "[quadplane][predicates]") {
    auto in = base_available();
    in.control_is_vtol_mode = true;
    REQUIRE(compute_in_vtol_mode(in));
}

TEST_CASE("in_vtol_mode land sequence approach is fw", "[quadplane][predicates]") {
    auto in = base_available();
    in.in_vtol_land_sequence = true;
    in.pos_state = PositionControlState::kApproach;
    REQUIRE_FALSE(compute_in_vtol_mode(in));
    in.pos_state = PositionControlState::kPosition1;
    REQUIRE(compute_in_vtol_mode(in));
}

TEST_CASE("guided vtol loiter past approach", "[quadplane][predicates]") {
    auto in = base_available();
    in.control_is_guided_mode = true;
    in.auto_vtol_loiter = true;
    in.pos_state = PositionControlState::kAirbrake;
    REQUIRE(compute_in_vtol_mode(in));
    in.pos_state = PositionControlState::kApproach;
    REQUIRE_FALSE(compute_in_vtol_mode(in));
}

TEST_CASE("guided takeoff on mode_guided", "[quadplane][predicates]") {
    auto in = base_available();
    in.mode_is_guided = true;
    in.guided_takeoff = true;
    REQUIRE(compute_in_vtol_mode(in));
}

TEST_CASE("QuadPlane in_vtol_mode wires poscontrol state", "[quadplane][predicates]") {
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    InVtolModeInputs view{.control_is_vtol_mode = true};
    REQUIRE(qp.in_vtol_mode(view));
    qp.poscontrol_mut().state = PositionControlState::kApproach;
    view = InVtolModeInputs{.in_vtol_land_sequence = true};
    REQUIRE_FALSE(qp.in_vtol_mode(view));
}
