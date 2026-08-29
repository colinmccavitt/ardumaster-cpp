#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_auto_vtol_mission.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>

using fwcpp::quadplane::DoVtolLandInputs;
using fwcpp::quadplane::DoVtolTakeoffInputs;
using fwcpp::quadplane::HandleDoVtolTransitionInputs;
using fwcpp::quadplane::MavVtolState;
using fwcpp::quadplane::PosControlLandStub;
using fwcpp::quadplane::PosControlState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::VerifyVtolLandInputs;
using fwcpp::quadplane::VerifyVtolTakeoffInputs;
using fwcpp::quadplane::do_vtol_land;
using fwcpp::quadplane::do_vtol_takeoff;
using fwcpp::quadplane::handle_do_vtol_transition;
using fwcpp::quadplane::is_vtol_land;
using fwcpp::quadplane::is_vtol_takeoff;
using fwcpp::quadplane::kMavCmdNavTakeoff;
using fwcpp::quadplane::kMavCmdNavVtolLand;
using fwcpp::quadplane::kMavCmdNavVtolTakeoff;
using fwcpp::quadplane::verify_vtol_land;
using fwcpp::quadplane::verify_vtol_takeoff;

TEST_CASE("is_vtol_takeoff mission ids", "[quadplane][auto_vtol]") {
    REQUIRE(is_vtol_takeoff(kMavCmdNavVtolTakeoff, true, 0));
    REQUIRE(is_vtol_takeoff(kMavCmdNavTakeoff, true, 0));
    REQUIRE_FALSE(is_vtol_takeoff(kMavCmdNavTakeoff, false, 0));
}

TEST_CASE("do_vtol_takeoff verify complete", "[quadplane][auto_vtol]") {
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    auto d = qp.do_vtol_takeoff_mission({.setup_ok = true, .next_wp_alt_cm = 5000, .now_ms = 1000});
    REQUIRE(d.ok);
    REQUIRE_FALSE(qp.throttle_wait());
    auto v = qp.verify_vtol_takeoff_mission({.available = true,
                                             .armed_and_safety_off = true,
                                             .current_alt_cm = 5000,
                                             .target_alt_cm = 5000,
                                             .control_is_auto = true});
    REQUIRE(v.complete);
    REQUIRE(v.reset_tecs);
}

TEST_CASE("handle_do_vtol_transition auto mc", "[quadplane][auto_vtol]") {
    HandleDoVtolTransitionInputs in{.available = true, .control_is_auto = true, .state = MavVtolState::kMc};
    const auto r = handle_do_vtol_transition(in);
    REQUIRE(r.ok);
    REQUIRE(r.auto_vtol_mode);
    REQUIRE(r.clear_fwd_throttle);
}

TEST_CASE("verify_vtol_land unavailable is complete", "[quadplane][auto_vtol]") {
    PosControlState pc{};
    pc.state = fwcpp::quadplane::PositionControlState::kLandDescend;
    PosControlLandStub land{};
    REQUIRE(verify_vtol_land(pc, land, {.available = false}).done);
    REQUIRE_FALSE(verify_vtol_land(pc, land, {.available = true}).done);
}
