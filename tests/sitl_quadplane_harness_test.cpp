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
