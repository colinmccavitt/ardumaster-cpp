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
    REQUIRE(this_slice_count() == 37);
    REQUIRE(remaining_count() == 2);
    REQUIRE(tailsitter_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + 5);
    REQUIRE(completeness_has("output FW motor_mask", PortStatus::kThisSlice));
    REQUIRE(completeness_has("output hover vectored pitch", PortStatus::kThisSlice));
    REQUIRE(completeness_has("output Q assist motors-only I-relax", PortStatus::kThisSlice));
    REQUIRE(completeness_has("setup SRV surface flags", PortStatus::kThisSlice));
    REQUIRE(completeness_has("enable==2 assist/airmode/arm", PortStatus::kThisSlice));
    REQUIRE(completeness_has("transition_rate_fw auto-set", PortStatus::kThisSlice));
    REQUIRE(completeness_has("speed_scaling", PortStatus::kRemaining));
    REQUIRE(completeness_has("relax_pitch", PortStatus::kRemaining));
    REQUIRE(completeness_has("write_log TSIT", PortStatus::kOutOfScope));
}
