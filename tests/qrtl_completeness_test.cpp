#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qrtl/qrtl_completeness.hpp>

using fwcpp::qrtl::PortStatus;
using fwcpp::qrtl::completeness_has;
using fwcpp::qrtl::on_main_count;
using fwcpp::qrtl::qrtl_completeness_size;
using fwcpp::qrtl::remaining_count;
using fwcpp::qrtl::this_slice_count;

TEST_CASE("qrtl catalog", "[qrtl][catalog]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 14);
    REQUIRE(remaining_count() == 3);
    REQUIRE(qrtl_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + 1);
    REQUIRE(completeness_has("_enter climb submode", PortStatus::kThisSlice));
    REQUIRE(completeness_has("run climb tick", PortStatus::kThisSlice));
}