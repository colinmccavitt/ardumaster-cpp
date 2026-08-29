#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_loiter/mode_qloiter_enter.hpp>

using fwcpp::q_loiter::QLoiterEnterEffects;
using fwcpp::q_loiter::QLoiterEnterInputs;
using fwcpp::q_loiter::qloiter_enter;

TEST_CASE("qloiter enter effects", "[q_loiter][enter]") {
    QLoiterEnterEffects effects{};
    QLoiterEnterInputs in{};
    in.now_ms = 1234;
    auto r = qloiter_enter(in, effects);
    REQUIRE(r.entered);
    REQUIRE(r.last_loiter_ms == 1234);
    REQUIRE(effects.loiter_init_target);
    REQUIRE(effects.init_throttle_wait);
    REQUIRE(effects.set_pos_z_limits);
}
