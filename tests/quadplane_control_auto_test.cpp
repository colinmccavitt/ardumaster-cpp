#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_control_auto.hpp>

using fwcpp::quadplane::ControlAutoInputs;
using fwcpp::quadplane::DesiredSpoolState;
using fwcpp::quadplane::PosControlLandStub;
using fwcpp::quadplane::PosControlSetStateSink;
using fwcpp::quadplane::PosControlState;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QOption;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::control_auto;
using fwcpp::quadplane::kControlAutoLoiterResetMs;
using fwcpp::quadplane::kMavCmdNavLand;
using fwcpp::quadplane::kMavCmdNavLoiterTime;
using fwcpp::quadplane::kMavCmdNavLoiterToAlt;
using fwcpp::quadplane::kMavCmdNavLoiterTurns;
using fwcpp::quadplane::kMavCmdNavLoiterUnlim;
using fwcpp::quadplane::kMavCmdNavPayloadPlace;
using fwcpp::quadplane::kMavCmdNavTakeoff;
using fwcpp::quadplane::kMavCmdNavVtolLand;
using fwcpp::quadplane::kMavCmdNavVtolTakeoff;
using fwcpp::quadplane::kMavCmdNavWaypoint;
using fwcpp::quadplane::option_is_set;

static ControlAutoInputs available_in(std::uint16_t id, std::uint32_t now_ms = 1000) {
    ControlAutoInputs in{};
    in.available = true;
    in.nav_cmd_id = id;
    in.now_ms = now_ms;
    return in;
}

static void require_no_dispatch(const fwcpp::quadplane::ControlAutoTick& tick) {
    REQUIRE_FALSE(tick.run_takeoff);
    REQUIRE_FALSE(tick.run_vtol_position_controller);
    REQUIRE_FALSE(tick.run_waypoint);
    REQUIRE_FALSE(tick.reset_poscontrol_position1);
    REQUIRE_FALSE(tick.set_spool_unlimited);
    REQUIRE_FALSE(tick.should_run_motors);
}

TEST_CASE("unavailable control_auto is a no-op", "[quadplane][control_auto]") {
    PosControlState pc{};
    pc.state = PositionControlState::kLandDescend;
    pc.last_run_ms = 1;
    PosControlLandStub land{};
    PosControlSetStateSink sink{};
    ControlAutoInputs in = available_in(kMavCmdNavVtolTakeoff, 5000);
    in.available = false;
    in.delay_arming = true;
    in.in_auto_mission_payload_place = true;

    const auto tick = control_auto(pc, land, sink, in);
    REQUIRE(tick.early_return);
    require_no_dispatch(tick);
    REQUIRE(pc.state == PositionControlState::kLandDescend);
    REQUIRE(pc.last_run_ms == 1);

    QuadPlane qp{1};
    REQUIRE_FALSE(qp.available());
    const auto wired = qp.control_auto(available_in(kMavCmdNavWaypoint));
    REQUIRE(wired.early_return);
    require_no_dispatch(wired);
}

TEST_CASE("takeoff vs land vs loiter vs default waypoint", "[quadplane][control_auto]") {
    PosControlState pc{};
    PosControlLandStub land{};
    PosControlSetStateSink sink{};

    auto takeoff = control_auto(pc, land, sink, available_in(kMavCmdNavVtolTakeoff));
    REQUIRE(takeoff.run_takeoff);
    REQUIRE_FALSE(takeoff.run_vtol_position_controller);
    REQUIRE_FALSE(takeoff.run_waypoint);

    takeoff = control_auto(pc, land, sink, available_in(kMavCmdNavTakeoff));
    REQUIRE(takeoff.run_takeoff);

    ControlAutoInputs fw_takeoff = available_in(kMavCmdNavTakeoff);
    fw_takeoff.options = static_cast<std::int32_t>(QOption::kAllowFwTakeoff);
    REQUIRE(option_is_set(fw_takeoff.options, QOption::kAllowFwTakeoff));
    takeoff = control_auto(pc, land, sink, fw_takeoff);
    REQUIRE_FALSE(takeoff.run_takeoff);
    REQUIRE_FALSE(takeoff.run_waypoint);

    auto land_tick = control_auto(pc, land, sink, available_in(kMavCmdNavVtolLand));
    REQUIRE(land_tick.run_vtol_position_controller);
    REQUIRE_FALSE(land_tick.run_takeoff);
    REQUIRE_FALSE(land_tick.run_waypoint);

    land_tick = control_auto(pc, land, sink, available_in(kMavCmdNavPayloadPlace));
    REQUIRE(land_tick.run_vtol_position_controller);

    land_tick = control_auto(pc, land, sink, available_in(kMavCmdNavLand));
    REQUIRE(land_tick.run_vtol_position_controller);

    ControlAutoInputs fw_land = available_in(kMavCmdNavLand);
    fw_land.options = static_cast<std::int32_t>(QOption::kAllowFwLand);
    land_tick = control_auto(pc, land, sink, fw_land);
    REQUIRE_FALSE(land_tick.run_vtol_position_controller);
    REQUIRE_FALSE(land_tick.run_waypoint);

    for (const auto id : {kMavCmdNavLoiterUnlim, kMavCmdNavLoiterTime, kMavCmdNavLoiterTurns,
                          kMavCmdNavLoiterToAlt}) {
        pc.last_run_ms = 950;
        auto loiter = control_auto(pc, land, sink, available_in(id, 1000));
        REQUIRE(loiter.run_vtol_position_controller);
        REQUIRE_FALSE(loiter.run_takeoff);
        REQUIRE_FALSE(loiter.run_waypoint);
        REQUIRE_FALSE(loiter.reset_poscontrol_position1);
    }

    auto waypoint = control_auto(pc, land, sink, available_in(kMavCmdNavWaypoint));
    REQUIRE(waypoint.run_waypoint);
    REQUIRE_FALSE(waypoint.run_takeoff);
    REQUIRE_FALSE(waypoint.run_vtol_position_controller);

    QuadPlane qp{1};
    REQUIRE(qp.setup());
    const auto wired = qp.control_auto(available_in(kMavCmdNavVtolTakeoff));
    REQUIRE_FALSE(wired.early_return);
    REQUIRE(wired.run_takeoff);
}

TEST_CASE("loiter older than 100ms resets POSITION1", "[quadplane][control_auto]") {
    PosControlState pc{};
    pc.state = PositionControlState::kLandDescend;
    pc.last_run_ms = 100;
    PosControlLandStub land{};
    PosControlSetStateSink sink{};

    ControlAutoInputs in = available_in(kMavCmdNavLoiterUnlim, 100 + kControlAutoLoiterResetMs);
    auto tick = control_auto(pc, land, sink, in);
    REQUIRE(tick.run_vtol_position_controller);
    REQUIRE_FALSE(tick.reset_poscontrol_position1);
    REQUIRE(pc.state == PositionControlState::kLandDescend);

    in.now_ms = 100 + kControlAutoLoiterResetMs + 1;
    tick = control_auto(pc, land, sink, in);
    REQUIRE(tick.reset_poscontrol_position1);
    REQUIRE(tick.run_vtol_position_controller);
    REQUIRE(pc.state == PositionControlState::kPosition1);
    REQUIRE(pc.last_run_ms == in.now_ms);
    REQUIRE(sink.reset_yaw_target);
}

TEST_CASE("dead should_run_motors never spools unlimited", "[quadplane][control_auto]") {
    PosControlState pc{};
    pc.state = PositionControlState::kLandComplete;
    PosControlLandStub land{};
    PosControlSetStateSink sink{};

    ControlAutoInputs in = available_in(kMavCmdNavPayloadPlace);
    in.delay_arming = true;
    in.desired_spool = DesiredSpoolState::kShutDown;
    in.in_auto_mission_payload_place = true;

    auto tick = control_auto(pc, land, sink, in);
    REQUIRE(tick.delay_arming_checked);
    REQUIRE(tick.payload_place_shutdown_checked);
    REQUIRE_FALSE(tick.should_run_motors);
    REQUIRE_FALSE(tick.set_spool_unlimited);
    REQUIRE(tick.run_vtol_position_controller);

    in.delay_arming = false;
    in.desired_spool = DesiredSpoolState::kThrottleUnlimited;
    in.in_auto_mission_payload_place = false;
    pc.state = PositionControlState::kPosition1;
    tick = control_auto(pc, land, sink, in);
    REQUIRE_FALSE(tick.delay_arming_checked);
    REQUIRE_FALSE(tick.payload_place_shutdown_checked);
    REQUIRE_FALSE(tick.should_run_motors);
    REQUIRE_FALSE(tick.set_spool_unlimited);

    pc.state = PositionControlState::kApproach;
    in.delay_arming = true;
    in.desired_spool = DesiredSpoolState::kShutDown;
    in.in_auto_mission_payload_place = true;
    tick = control_auto(pc, land, sink, in);
    REQUIRE_FALSE(tick.delay_arming_checked);
    REQUIRE_FALSE(tick.payload_place_shutdown_checked);
    REQUIRE_FALSE(tick.should_run_motors);
    REQUIRE_FALSE(tick.set_spool_unlimited);
}
