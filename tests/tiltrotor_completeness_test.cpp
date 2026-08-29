#include <catch2/catch_test_macros.hpp>
#include <fwcpp/tiltrotor/tiltrotor_completeness.hpp>

using fwcpp::tiltrotor::PortStatus;
using fwcpp::tiltrotor::completeness_has;
using fwcpp::tiltrotor::on_main_count;
using fwcpp::tiltrotor::remaining_count;
using fwcpp::tiltrotor::tiltrotor_completeness_size;
using fwcpp::tiltrotor::this_slice_count;

TEST_CASE("tiltrotor catalog", "[tiltrotor][catalog]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 10);
    REQUIRE(remaining_count() == 20);
    REQUIRE(tiltrotor_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + 3);
    REQUIRE(completeness_has("setup enable heuristic", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Tiltrotor::update", PortStatus::kRemaining));
}
