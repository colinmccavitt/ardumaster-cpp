#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_air_mode.hpp>

using fwcpp::quadplane::AirMode;
using fwcpp::quadplane::AirModeAuxPos;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::air_mode_active;

TEST_CASE("air_mode_active on and assisted only", "[quadplane][air_mode]") {
    REQUIRE(air_mode_active(AirMode::kOn, false));
    REQUIRE_FALSE(air_mode_active(AirMode::kOff, true));
    REQUIRE(air_mode_active(AirMode::kAssistedFlightOnly, true));
    REQUIRE_FALSE(air_mode_active(AirMode::kAssistedFlightOnly, false));
}

TEST_CASE("QuadPlane air_mode aux latch", "[quadplane][air_mode]") {
    QuadPlane qp{1};
    qp.set_air_mode(AirMode::kOff);
    qp.set_throttle_wait(true);
    qp.apply_air_mode_aux(AirModeAuxPos::kHigh);
    REQUIRE(qp.air_mode() == AirMode::kOn);
    REQUIRE_FALSE(qp.throttle_wait());
    qp.apply_air_mode_aux(AirModeAuxPos::kLow);
    REQUIRE(qp.air_mode() == AirMode::kOff);
}

TEST_CASE("armdisarm airmode latch", "[quadplane][air_mode]") {
    QuadPlane qp{1};
    qp.apply_armdisarm_airmode_latch(false);
    REQUIRE(qp.air_mode() == AirMode::kOff);
    qp.apply_armdisarm_airmode_latch(true);
    REQUIRE(qp.air_mode() == AirMode::kOn);
    REQUIRE(qp.air_mode_active());
}
