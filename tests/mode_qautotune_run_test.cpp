#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qautotune/qautotune.hpp>

using fwcpp::qautotune::QAutotuneRunInputs;
using fwcpp::qautotune::QAutotuneRunPhase;
using fwcpp::qautotune::qautotune_run;

TEST_CASE("qautotune run tailsitter transition delegates FW", "[qautotune][run]") {
    QAutotuneRunInputs in{};
    in.tailsitter_in_vtol_transition = true;
    const auto r = qautotune_run(in);
    REQUIRE(r.delegate_mode_run);
    REQUIRE(r.phase == QAutotuneRunPhase::kFwTransitionControllers);
}

TEST_CASE("qautotune run normal path", "[qautotune][run]") {
    QAutotuneRunInputs in{};
    const auto r = qautotune_run(in);
    REQUIRE(r.run_qautotune);
    REQUIRE(r.fw_followup.stabilize_roll);
    REQUIRE(r.fw_followup.center_rudder);
}
