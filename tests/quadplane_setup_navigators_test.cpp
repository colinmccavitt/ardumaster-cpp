#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_setup_navigators.hpp>
#include <fwcpp/wpnav/wpnav.hpp>

using fwcpp::math::Vector3;
using fwcpp::quadplane::LoiterNavStub;
using fwcpp::quadplane::NavigatorDeps;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::QuadPlaneSetupInputs;
using fwcpp::quadplane::WpNavSetupInputs;
using fwcpp::quadplane::navigator_deps_ready;
using fwcpp::quadplane::wire_setup_navigators;
using fwcpp::wpnav::WpNav;

TEST_CASE("navigator deps gate", "[quadplane][setup][nav]") {
    REQUIRE_FALSE(navigator_deps_ready(NavigatorDeps{}));
    REQUIRE(navigator_deps_ready(NavigatorDeps{true, true, true}));
    REQUIRE_FALSE(wire_setup_navigators(NavigatorDeps{}, WpNavSetupInputs{}).ok);
}

TEST_CASE("wire_setup_navigators builds stubs", "[quadplane][setup][nav]") {
    const Vector3<float> stop{1.f, 2.f, -3.f};
    WpNavSetupInputs wp{};
    wp.init_speed_ms = 4.f;
    wp.stopping_point_ned_m = stop;
    wp.now_ms = 42;
    const auto r = wire_setup_navigators(NavigatorDeps{true, true, true}, wp);
    REQUIRE(r.ok);
    REQUIRE(r.loiter_nav.created);
    REQUIRE(r.loiter_nav.ahrs_view_wired);
    REQUIRE(r.loiter_nav.pos_control_wired);
    REQUIRE(r.loiter_nav.attitude_control_wired);
    REQUIRE(r.wp_and_spline_inited);
    REQUIRE(r.wp_nav.pos_control_stopping_point_inited());
    REQUIRE(r.wp_nav.wp_destination_ned_m().x == Catch::Approx(stop.x));
}

TEST_CASE("QuadPlane setup creates navigator stubs", "[quadplane][setup][nav]") {
    QuadPlane qp{1};
    QuadPlaneSetupInputs in{};
    in.wp_nav.init_speed_ms = 6.f;
    in.wp_nav.stopping_point_ned_m = Vector3<float>{10.f, 0.f, 5.f};
    in.wp_nav.now_ms = 100;
    REQUIRE(qp.setup(in));
    REQUIRE(qp.wp_nav_inited());
    REQUIRE(qp.loiter_nav_inited());
    REQUIRE(qp.loiter_nav().created);
    REQUIRE(qp.wp_nav().pos_control_stopping_point_inited());
    REQUIRE(qp.wp_nav().desired_speed_ne_ms() == Catch::Approx(6.f));
}

TEST_CASE("setup leaves navigators unallocated when disabled", "[quadplane][setup][nav]") {
    QuadPlane qp{};
    REQUIRE_FALSE(qp.setup());
    REQUIRE_FALSE(qp.wp_nav_inited());
    REQUIRE_FALSE(qp.loiter_nav_inited());
}
