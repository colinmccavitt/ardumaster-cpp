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
    REQUIRE(on_main_count() == 6);
    REQUIRE(this_slice_count() == 1);
    REQUIRE(remaining_count() > 0);
    REQUIRE(remaining_count() == 3);
    REQUIRE(out_of_scope_count() == 0);
    REQUIRE(logger_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("backend", PortStatus::kOnMain));
    REQUIRE(completeness_has("drop", PortStatus::kOnMain));
    REQUIRE(completeness_has("completeness catalog", PortStatus::kOnMain));
    REQUIRE(completeness_has("DataFlash page map", PortStatus::kRemaining));
    REQUIRE(completeness_has("POSIX/SD file backend", PortStatus::kOnMain));
    REQUIRE(completeness_has("FMT registry", PortStatus::kOnMain));
    REQUIRE(completeness_has("streaming", PortStatus::kOnMain));
    REQUIRE(completeness_has("transfer", PortStatus::kRemaining));
    REQUIRE(completeness_has("EraseAll", PortStatus::kThisSlice));
    REQUIRE(completeness_has("max-files rotation", PortStatus::kRemaining));
}
