#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qrtl/qrtl_geometry.hpp>

using Catch::Approx;
using fwcpp::qrtl::QrtlDestination;
using fwcpp::qrtl::calc_best_rally_or_home;
using fwcpp::qrtl::qrtl_climb_cone_target_alt_m;
using fwcpp::qrtl::qrtl_min_climb_m;
using fwcpp::qrtl::qrtl_vtol_return_radius_m;

TEST_CASE("qrtl geometry helpers", "[qrtl][geometry]") {
    REQUIRE(qrtl_vtol_return_radius_m(60.0F, 0.0F) == Approx(90.0F));
    REQUIRE(qrtl_vtol_return_radius_m(-80.0F, 40.0F) == Approx(120.0F));
    REQUIRE(qrtl_min_climb_m(10.0F, 6.0F, 15.0F) == Approx(10.0F));
    REQUIRE(qrtl_min_climb_m(3.0F, 6.0F, 15.0F) == Approx(6.0F));
    REQUIRE(qrtl_climb_cone_target_alt_m(15.0F, 200.0F, 90.0F, 10.0F) == Approx(15.0F));
    REQUIRE(calc_best_rally_or_home(100.0F, 50.0F, true, true) == QrtlDestination::kRally);
    REQUIRE(calc_best_rally_or_home(100.0F, 150.0F, true, true) == QrtlDestination::kHome);
}
