#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qautotune/qautotune_completeness.hpp>

using fwcpp::qautotune::PortStatus;
using fwcpp::qautotune::completeness_has;
using fwcpp::qautotune::qautotune_completeness_size;
using fwcpp::qautotune::this_slice_count;

TEST_CASE("qautotune completeness table", "[qautotune][completeness]") {
    REQUIRE(qautotune_completeness_size() >= 14);
    REQUIRE(this_slice_count() >= 14);
    REQUIRE(completeness_has("ModeQAutotune::run qautotune.run hook", PortStatus::kThisSlice));
}
