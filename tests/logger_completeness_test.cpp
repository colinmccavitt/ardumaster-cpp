#include <catch2/catch_test_macros.hpp>
#include <fwcpp/logger/completeness.hpp>

using fwcpp::logger::PortStatus;
using fwcpp::logger::completeness_has;
using fwcpp::logger::logger_completeness_size;
using fwcpp::logger::on_main_count;
using fwcpp::logger::out_of_scope_count;
using fwcpp::logger::remaining_count;
using fwcpp::logger::this_slice_count;

TEST_CASE("logger completeness table lists this slice vs remaining", "[logger][completeness]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 3);
    REQUIRE(remaining_count() > 0);
    REQUIRE(remaining_count() == 6);
    REQUIRE(out_of_scope_count() == 0);
    REQUIRE(logger_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("backend", PortStatus::kThisSlice));
    REQUIRE(completeness_has("drop", PortStatus::kThisSlice));
    REQUIRE(completeness_has("completeness catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("DataFlash page map", PortStatus::kRemaining));
    REQUIRE(completeness_has("POSIX/SD file backend", PortStatus::kRemaining));
    REQUIRE(completeness_has("FMT registry", PortStatus::kRemaining));
    REQUIRE(completeness_has("streaming", PortStatus::kRemaining));
    REQUIRE(completeness_has("transfer", PortStatus::kRemaining));
    REQUIRE(completeness_has("erase/rotate", PortStatus::kRemaining));
}
