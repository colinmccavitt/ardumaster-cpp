#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_loiter/mode_qloiter_meta.hpp>
#include <fwcpp/q_loiter/mode_qland_meta.hpp>
#include <fwcpp/q_loiter/mode_loiter_alt_qland_meta.hpp>

using fwcpp::q_loiter::kModeLoiterAltQlandNumber;
using fwcpp::q_loiter::kModeQlandNumber;
using fwcpp::q_loiter::kModeQloiterNumber;

TEST_CASE("q loiter mode numbers", "[q_loiter][meta]") {
    REQUIRE(kModeQloiterNumber == 19);
    REQUIRE(kModeQlandNumber == 20);
    REQUIRE(kModeLoiterAltQlandNumber == 25);
}
