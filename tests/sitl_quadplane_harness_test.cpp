// VCP-011: SitlQuadPlaneHarness sensors + Plane/QuadPlane tick + SIM_QuadPlane.
#include <cmath>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/hal_sitl/sitl_quadplane_harness.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/sim/sim_quadplane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

using fwcpp::hal_sitl::SitlQuadPlaneHarness;
using fwcpp::hal_sitl::sitl_quadplane::PortStatus;
using fwcpp::hal_sitl::sitl_quadplane::completeness_has;
using fwcpp::hal_sitl::sitl_quadplane::completeness_size;
using fwcpp::hal_sitl::sitl_quadplane::on_main_count;
using fwcpp::hal_sitl::sitl_quadplane::out_of_scope_count;
using fwcpp::hal_sitl::sitl_quadplane::remaining_count;
using fwcpp::hal_sitl::sitl_quadplane::this_slice_count;
using fwcpp::quadplane::QuadPlane;
using fwcpp::sim::SimQuadPlane;
using fwcpp::vehicle::ModeFBWA;
using fwcpp::vehicle::Plane;

TEST_CASE("SitlQuadPlaneHarness step ticks Plane and QuadPlane into SimQuadPlane",
          "[quadplane][sitl][vcp-011]") {
    Plane plane;
    ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    SimQuadPlane sim{"quadplane"};
    SitlQuadPlaneHarness harness(plane, qp, sim);
    REQUIRE(harness.tick_count() == 0);
    const float hover = sim.frame().hover_command();
    harness.step(20, 0.0025f, hover, true);
    REQUIRE(harness.tick_count() == 1);
    REQUIRE(qp.available());
}

TEST_CASE("SitlQuadPlaneHarness climb command leaves the ground", "[quadplane][sitl][vcp-011]") {
    Plane plane;
    ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    SimQuadPlane sim{"quadplane"};
    SitlQuadPlaneHarness harness(plane, qp, sim);
    const float climb = sim.frame().hover_command() + 0.20f;
    constexpr float kDt = 0.0025f;
    std::uint32_t now_ms = 0;
    for (int i = 0; i < 1600; ++i) {
        now_ms += 3;
        harness.step(now_ms, kDt, climb, true);
    }
    REQUIRE((-sim.position.z) > 2.0f);
    REQUIRE(std::isfinite(sim.airspeed));
}

TEST_CASE("SitlQuadPlaneHarness leftover catalog remaining_count", "[quadplane][sitl][vcp-011][leftover]") {
    REQUIRE(remaining_count() == 0);
    REQUIRE(this_slice_count() == 7);
    REQUIRE(on_main_count() == 2);
    REQUIRE(out_of_scope_count() == 3);
    REQUIRE(completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("SitlQuadPlaneHarness scaffold", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Plane::tick", PortStatus::kThisSlice));
    REQUIRE(completeness_has("QuadPlane::update", PortStatus::kThisSlice));
    REQUIRE(completeness_has("SIM_QuadPlane plant", PortStatus::kOnMain));
}

TEST_CASE("SimQuadPlane copter_tailsitter frame string sets tailsitter flag and ground behavior",
          "[quadplane][sitl][vcp-012]") {
    // Real upstream libraries/SITL/SIM_QuadPlane.cpp lines 80-84: the
    // "-copter_tailsitter" frame-string suffix selects the "+" motor layout,
    // sets `copter_tailsitter = true`, and sets
    // `ground_behavior = GROUND_BEHAVIOR_TAILSITTER`. Ported at
    // sim_quadplane.hpp lines 58-61 (copter_tailsitter_ / ground_behavior).
    // This is a pure frame-string-parser assertion, independent of flight
    // dynamics: VCP-011's own harness never exercised this suffix before.
    SimQuadPlane sim{"quadplane-copter_tailsitter"};
    REQUIRE(sim.copter_tailsitter());
    REQUIRE(sim.ground_behavior == fwcpp::sim::GroundBehavior::kTailsitter);

    // The bare "quadplane" frame (already covered by the existing tests in
    // this file) must NOT set either, so the assertions above are genuinely
    // exercising the "-copter_tailsitter" suffix branch and not some
    // always-true default.
    SimQuadPlane plain{"quadplane"};
    REQUIRE_FALSE(plain.copter_tailsitter());
    REQUIRE(plain.ground_behavior == fwcpp::sim::GroundBehavior::kNoMovement);
}

TEST_CASE("SitlQuadPlaneHarness scripted flight through copter_tailsitter frame stays numerically sane",
          "[quadplane][sitl][vcp-012]") {
    // Exercises the tailsitter rotation applied at sim_quadplane.hpp line 120
    // (`if (copter_tailsitter_) { ... }`, immediately after
    // frame_.calculate_forces(...) at line 118 - matching real upstream's own
    // structure at SIM_QuadPlane.cpp lines 133-135) for the first time via a
    // scripted flight. This does NOT need to reach a successful hover/climb -
    // that is VCP-007's own already-done Q-mode control-logic scope. Only
    // numerical sanity (finite airspeed/position/attitude, no NaN, no crash)
    // is asserted here.
    Plane plane;
    ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    SimQuadPlane sim{"quadplane-copter_tailsitter"};
    REQUIRE(sim.copter_tailsitter());
    SitlQuadPlaneHarness harness(plane, qp, sim);
    const float hover = sim.frame().hover_command();
    constexpr float kDt = 0.0025f;
    std::uint32_t now_ms = 0;
    for (int i = 0; i < 500; ++i) {
        now_ms += 3;
        harness.step(now_ms, kDt, hover, true);

        REQUIRE(std::isfinite(sim.airspeed));
        REQUIRE(std::isfinite(sim.position.x));
        REQUIRE(std::isfinite(sim.position.y));
        REQUIRE(std::isfinite(sim.position.z));

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        sim.dcm.to_euler(&roll, &pitch, &yaw);
        REQUIRE(std::isfinite(roll));
        REQUIRE(std::isfinite(pitch));
        REQUIRE(std::isfinite(yaw));
    }
    REQUIRE(harness.tick_count() == 500);
}

TEST_CASE("SimQuadPlane -tilttrivec frame selects a genuinely different motor layout",
          "[quadplane][sitl][vcp-012]") {
    // Real upstream libraries/SITL/SIM_QuadPlane.cpp lines 54-56: the
    // "-tilttrivec" suffix selects the distinct "tilttrivec" frame_type (a
    // 3-motor tilt-tri layout, real SIM_Frame.cpp frame table entry
    // {"tilttrivec", 3, tilttri_vectored_motors}) instead of the default
    // 4-motor "x" quad layout used by the bare "quadplane" frame string. The
    // rotation block at sim_quadplane.hpp line 120 checks ONLY the
    // tailsitter flag, so tilt-rotor variants carry no rotation - their
    // distinguishing behavior lives entirely in the frame's own motor
    // geometry, which is what this test checks directly (real motor count
    // and layout, not a silent fallthrough to the default frame).
    SimQuadPlane tilttrivec{"quadplane-tilttrivec"};
    SimQuadPlane plain{"quadplane"};

    REQUIRE_FALSE(tilttrivec.copter_tailsitter());
    REQUIRE(tilttrivec.frame().num_motors == 3);
    REQUIRE(plain.frame().num_motors == 4);
    REQUIRE(tilttrivec.frame().num_motors != plain.frame().num_motors);
}

TEST_CASE("SitlQuadPlaneHarness scripted flight through -tilttrivec frame stays numerically sane",
          "[quadplane][sitl][vcp-012]") {
    // Scripted-flight numerical-sanity coverage for the tilt-rotor branch,
    // matching the tailsitter scripted-flight test above. No hover/climb bar
    // is required (VCP-007 scope); only finite, non-NaN telemetry throughout.
    Plane plane;
    ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    SimQuadPlane sim{"quadplane-tilttrivec"};
    REQUIRE(sim.frame().num_motors == 3);
    SitlQuadPlaneHarness harness(plane, qp, sim);
    const float hover = sim.frame().hover_command();
    constexpr float kDt = 0.0025f;
    std::uint32_t now_ms = 0;
    for (int i = 0; i < 500; ++i) {
        now_ms += 3;
        harness.step(now_ms, kDt, hover, true);

        REQUIRE(std::isfinite(sim.airspeed));
        REQUIRE(std::isfinite(sim.position.x));
        REQUIRE(std::isfinite(sim.position.y));
        REQUIRE(std::isfinite(sim.position.z));

        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
        sim.dcm.to_euler(&roll, &pitch, &yaw);
        REQUIRE(std::isfinite(roll));
        REQUIRE(std::isfinite(pitch));
        REQUIRE(std::isfinite(yaw));
    }
    REQUIRE(harness.tick_count() == 500);
}
