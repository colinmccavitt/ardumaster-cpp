#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_land_detector.hpp>
#include <fwcpp/quadplane/quadplane_poscontrol_fsm.hpp>
#include <fwcpp/quadplane/quadplane_verify_vtol_land.hpp>

using fwcpp::quadplane::LandDetectorInputs;
using fwcpp::quadplane::PosControlLandStub;
using fwcpp::quadplane::PosControlState;
using fwcpp::quadplane::PositionControlState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::VerifyVtolLandInputs;
using fwcpp::quadplane::VerifyVtolLandText;
using fwcpp::quadplane::kCheckLandCompleteTimeoutMs;
using fwcpp::quadplane::kShouldRelaxLowerLimitMs;
using fwcpp::quadplane::kVerifyLandDescendDistM;
using fwcpp::quadplane::verify_vtol_land;

static LandDetectorInputs detector_at_rest(std::uint32_t now_ms, float height_m = 1.0f) {
    return LandDetectorInputs{
        .now_ms = now_ms,
        .throttle_lower = true,
        .throttle_mix_min = true,
        .throttle = 0.0f,
        .height_m = height_m,
        .pilot_correction_active = false,
    };
}

static void arm_detector(PosControlLandStub& land, std::uint32_t lower_ms, std::uint32_t land_ms,
                         float vpos_m) {
    land.lower_limit_start_ms = lower_ms;
    land.land_start_ms = land_ms;
    land.vpos_start_m = vpos_m;
}

static VerifyVtolLandInputs available_in() {
    VerifyVtolLandInputs in{};
    in.available = true;
    in.now_ms = 2000;
    return in;
}

TEST_CASE("verify_vtol_land unavailable is done", "[quadplane][verify_land]") {
    PosControlState pc{};
    PosControlLandStub land{};
    REQUIRE(verify_vtol_land(pc, land, {.available = false}).done);
    REQUIRE(pc.state == PositionControlState::kNone);
}

TEST_CASE("verify_vtol_land POSITION2 to DESCEND on dist and speed", "[quadplane][verify_land]") {
    PosControlState pc{};
    pc.state = PositionControlState::kPosition2;
    pc.target_ned_n_m = 0.f;
    pc.target_ned_e_m = 0.f;
    pc.correction_north_m = 4.f;
    pc.correction_east_m = 5.f;
    PosControlLandStub land{};
    auto in = available_in();
    in.pos_n_m = 1.0f;
    in.pos_e_m = 0.0f;
    in.vel_north_ms = 0.5f;
    in.height_agl_m = 12.0f;
    in.current_alt_m = 40.0f;
    in.mode_auto = true;
    in.landing_gear_enabled = true;
    const auto tick = verify_vtol_land(pc, land, in);
    REQUIRE(pc.state == PositionControlState::kLandDescend);
    REQUIRE(tick.send_text == VerifyVtolLandText::kLandDescendStarted);
    REQUIRE(tick.set_lean_angle_max);
    REQUIRE(tick.lean_angle_max_cd == 0);
    REQUIRE(tick.deploy_landing_gear);
    REQUIRE(tick.set_next_wp_from_mission);
    REQUIRE_FALSE(tick.set_next_wp_from_current);
    REQUIRE_FALSE(tick.copy_alt_from_home);
    REQUIRE(pc.correction_north_m == 0.f);
    REQUIRE(pc.correction_east_m == 0.f);
    REQUIRE_FALSE(pc.pilot_correction_done);
    REQUIRE(land.last_land_final_agl_m == 12.0f);
    REQUIRE(land.land_descend_start_alt_m == 40.0f);
    REQUIRE_FALSE(tick.done);
}

TEST_CASE("verify_vtol_land POSITION2 waits for distance", "[quadplane][verify_land]") {
    PosControlState pc{};
    pc.state = PositionControlState::kPosition2;
    PosControlLandStub land{};
    auto in = available_in();
    in.pos_n_m = kVerifyLandDescendDistM + 0.1f;
    const auto tick = verify_vtol_land(pc, land, in);
    REQUIRE(pc.state == PositionControlState::kPosition2);
    REQUIRE_FALSE(tick.set_lean_angle_max);
    REQUIRE_FALSE(tick.done);
}

TEST_CASE("verify_vtol_land POSITION2 correction-done path", "[quadplane][verify_land]") {
    PosControlState pc{};
    pc.state = PositionControlState::kPosition2;
    pc.pilot_correction_done = true;
    pc.pilot_correction_active = true;
    PosControlLandStub land{};
    auto in = available_in();
    in.pos_n_m = 0.f;
    REQUIRE(pc.state == PositionControlState::kPosition2);
    auto tick = verify_vtol_land(pc, land, in);
    REQUIRE(pc.state == PositionControlState::kPosition2);
    REQUIRE_FALSE(tick.set_lean_angle_max);

    pc.pilot_correction_active = false;
    in.mode_auto = false;
    in.landing_gear_enabled = false;
    in.height_agl_m = 8.0f;
    tick = verify_vtol_land(pc, land, in);
    REQUIRE(pc.state == PositionControlState::kLandDescend);
    REQUIRE(tick.send_text == VerifyVtolLandText::kLandDescendStarted);
    REQUIRE_FALSE(tick.deploy_landing_gear);
    REQUIRE(tick.set_next_wp_from_current);
    REQUIRE(tick.copy_alt_from_home);
    REQUIRE_FALSE(tick.set_next_wp_from_mission);
}

TEST_CASE("verify_vtol_land LAND_DESCEND to FINAL via check_land_final", "[quadplane][verify_land]") {
    PosControlState pc{};
    pc.state = PositionControlState::kLandDescend;
    PosControlLandStub land{};
    land.last_land_final_agl_m = 3.0f;
    auto in = available_in();
    in.height_agl_m = 3.1f;
    in.icengine_enabled = true;
    in.land_icengine_cut = true;
    in.detector = detector_at_rest(10, 3.1f);
    const auto tick = verify_vtol_land(pc, land, in);
    REQUIRE(pc.state == PositionControlState::kLandFinal);
    REQUIRE(tick.ice_cut);
    REQUIRE(tick.send_text == VerifyVtolLandText::kLandFinalStarted);
    REQUIRE_FALSE(tick.done);
}

TEST_CASE("verify_vtol_land LAND_ABORT alt continue", "[quadplane][verify_land]") {
    PosControlState pc{};
    pc.state = PositionControlState::kLandAbort;
    PosControlLandStub land{};
    land.land_descend_start_alt_m = 50.0f;
    auto in = available_in();
    in.current_alt_m = 49.9f;
    REQUIRE_FALSE(verify_vtol_land(pc, land, in).done);
    in.current_alt_m = 50.0f;
    REQUIRE(verify_vtol_land(pc, land, in).done);
    REQUIRE(pc.state == PositionControlState::kLandAbort);
}

TEST_CASE("verify_vtol_land payload place abort", "[quadplane][verify_land]") {
    PosControlState pc{};
    pc.state = PositionControlState::kLandDescend;
    PosControlLandStub land{};
    land.land_descend_start_alt_m = 100.0f;
    land.last_land_final_agl_m = 20.0f;
    auto in = available_in();
    in.payload_place = true;
    in.cmd_p1 = 1000;
    in.current_alt_m = 89.0f;
    in.height_agl_m = 20.0f;
    in.detector = detector_at_rest(10, 20.0f);
    const auto tick = verify_vtol_land(pc, land, in);
    REQUIRE(pc.state == PositionControlState::kLandAbort);
    REQUIRE(tick.send_text == VerifyVtolLandText::kPayloadPlaceAborted);
    REQUIRE_FALSE(tick.done);
}

TEST_CASE("verify_vtol_land complete with continue_after_land", "[quadplane][verify_land]") {
    PosControlState pc{};
    pc.state = PositionControlState::kLandFinal;
    PosControlLandStub land{};
    arm_detector(land, 1000, 1000, 0.3f);
    auto in = available_in();
    in.mode_auto = true;
    in.continue_after_land = true;
    in.detector = detector_at_rest(1000 + kCheckLandCompleteTimeoutMs + kShouldRelaxLowerLimitMs, 0.3f);
    in.height_agl_m = 0.3f;
    const auto tick = verify_vtol_land(pc, land, in);
    REQUIRE(tick.complete);
    REQUIRE(tick.done);
    REQUIRE(tick.send_text == VerifyVtolLandText::kMissionContinue);
    REQUIRE_FALSE(tick.disarm);
    REQUIRE(pc.state == PositionControlState::kLandComplete);
}

TEST_CASE("verify_vtol_land QuadPlane wires available", "[quadplane][verify_land]") {
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    qp.poscontrol_mut().state = PositionControlState::kPosition2;
    auto in = available_in();
    in.available = false;
    in.landing_gear_enabled = true;
    in.mode_auto = true;
    in.height_agl_m = 12.0f;
    const auto tick = qp.verify_vtol_land_mission(in);
    REQUIRE(qp.poscontrol().state == PositionControlState::kLandDescend);
    REQUIRE(tick.deploy_landing_gear);
}
