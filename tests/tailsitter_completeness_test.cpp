#include <catch2/catch_test_macros.hpp>
#include <fwcpp/tailsitter/tailsitter_completeness.hpp>

using fwcpp::tailsitter::PortStatus;
using fwcpp::tailsitter::completeness_has;
using fwcpp::tailsitter::on_main_count;
using fwcpp::tailsitter::remaining_count;
using fwcpp::tailsitter::tailsitter_completeness_size;
using fwcpp::tailsitter::this_slice_count;

TEST_CASE("tailsitter catalog", "[tailsitter][catalog]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 23);
    REQUIRE(remaining_count() == 7);
    REQUIRE(tailsitter_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + 4);
    REQUIRE(completeness_has("transition_fw_complete", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Tailsitter::output", PortStatus::kRemaining));
}
