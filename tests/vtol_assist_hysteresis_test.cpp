#include <catch2/catch_test_macros.hpp>
#include <fwcpp/vtol_assist/assist_hysteresis.hpp>

using fwcpp::vtol_assist::AssistHysteresis;
using fwcpp::vtol_assist::clear_delay_ms;
using fwcpp::vtol_assist::trigger_delay_ms;

TEST_CASE("assist hysteresis trigger edge", "[vtol_assist][hysteresis]") {
    AssistHysteresis h;
    const auto trig = trigger_delay_ms(0.5f);
    const auto clear = clear_delay_ms(0.5f);
    REQUIRE(trig == 500u);
    REQUIRE(clear == 1000u);
    REQUIRE_FALSE(h.update(true, 100, trig, clear));
    REQUIRE(h.update(true, 601, trig, clear));
    REQUIRE(h.is_active());
    REQUIRE_FALSE(h.update(true, 700, trig, clear));
    REQUIRE_FALSE(h.update(false, 1701, trig, clear));
    REQUIRE_FALSE(h.is_active());
}
