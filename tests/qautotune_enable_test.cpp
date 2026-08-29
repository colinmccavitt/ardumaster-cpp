#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qautotune/qautotune.hpp>

using fwcpp::qautotune::HalBoard;
using fwcpp::qautotune::QAutotuneEnableInputs;
using fwcpp::qautotune::QAutotuneGate;

TEST_CASE("qautotune enabled on SITL quadplane", "[qautotune][enable]") {
    QAutotuneEnableInputs in{};
    in.hal_quadplane_enabled = true;
    in.board = HalBoard::kSitl;
    REQUIRE(QAutotuneGate::from_inputs(in).enabled());
}

TEST_CASE("qautotune disabled without quadplane", "[qautotune][enable]") {
    QAutotuneEnableInputs in{};
    in.hal_quadplane_enabled = false;
    in.board = HalBoard::kSitl;
    REQUIRE_FALSE(QAutotuneGate::from_inputs(in).enabled());
}
