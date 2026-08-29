#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_loiter/mode_qland_enter.hpp>

using Catch::Approx;
using fwcpp::q_loiter::QLandEnterEffects;
using fwcpp::q_loiter::QLandEnterInputs;
using fwcpp::q_loiter::QPosLandState;
using fwcpp::q_loiter::qland_enter;

TEST_CASE("qland enter", "[q_loiter][enter]") {
    QLandEnterEffects effects{};
    QLandEnterInputs in{};
    in.relative_ground_alt_m = 8.5F;
    in.landing_gear_enabled = true;
    auto r = qland_enter(in, effects);
    REQUIRE(r.entered);
    REQUIRE(r.pos_state == QPosLandState::kLandDescend);
    REQUIRE(r.last_land_final_agl_m == Approx(8.5F));
    REQUIRE(effects.poscontrol_land_descend);
    REQUIRE(effects.deploy_landing_gear);
    REQUIRE(effects.qloiter.loiter_init_target);
}
