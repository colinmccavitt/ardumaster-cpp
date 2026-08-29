#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qrtl/mode_qrtl_enter.hpp>

using Catch::Approx;
using fwcpp::qrtl::QrtlEnterAction;
using fwcpp::qrtl::QrtlEnterEffects;
using fwcpp::qrtl::QrtlEnterView;
using fwcpp::qrtl::QrtlSubMode;
using fwcpp::qrtl::qrtl_enter;

TEST_CASE("qrtl enter paths", "[qrtl][enter]") {
    QrtlEnterEffects effects{};
    QrtlEnterView view{};
    auto r = qrtl_enter(view, true, effects);
    REQUIRE(r.action == QrtlEnterAction::kQLandInstead);
    REQUIRE(effects.request_qland_instead);

    effects = {};
    r = qrtl_enter(view, false, effects);
    REQUIRE(r.action == QrtlEnterAction::kClimb);
    REQUIRE(r.submode == QrtlSubMode::kClimb);
    REQUIRE(r.dist_to_climb_m == Approx(10.0F));

    view.relative_ground_alt_m = 20.0F;
    view.current_alt_abs_cm = 2000;
    effects = {};
    r = qrtl_enter(view, false, effects);
    REQUIRE(r.action == QrtlEnterAction::kRtl);
    REQUIRE(effects.do_rtl);
    REQUIRE(effects.poscontrol_init_approach);

    view.home_dist_m = 50.0F;
    view.relative_ground_alt_m = 12.0F;
    view.current_alt_abs_cm = 1200;
    effects = {};
    r = qrtl_enter(view, false, effects);
    REQUIRE(effects.set_position1);
    REQUIRE(effects.do_rtl);
}
