#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qautotune/qautotune_completeness.hpp>

using fwcpp::qautotune::PortStatus;
using fwcpp::qautotune::completeness_has;
using fwcpp::qautotune::on_main_count;
using fwcpp::qautotune::out_of_scope_count;
using fwcpp::qautotune::qautotune_completeness_size;
using fwcpp::qautotune::remaining_count;
using fwcpp::qautotune::this_slice_count;

TEST_CASE("qautotune completeness table", "[qautotune][completeness]") {
    REQUIRE(on_main_count() == 0);
    REQUIRE(this_slice_count() == 16);
    REQUIRE(remaining_count() == 0);
    REQUIRE(out_of_scope_count() == 6);
    REQUIRE(qautotune_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("QAutoTune::init_internals wiring", PortStatus::kThisSlice));
    REQUIRE(completeness_has("QAutoTune::log_pids", PortStatus::kThisSlice));
    REQUIRE(completeness_has("ModeQAutotune::run qautotune.run hook", PortStatus::kThisSlice));
    REQUIRE(completeness_has("QAutoTune::init_internals body", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("QAutoTune::run autotune FSM", PortStatus::kOutOfScope));
    REQUIRE(completeness_has("QAutoTune::stop cleanup", PortStatus::kOutOfScope));
}
