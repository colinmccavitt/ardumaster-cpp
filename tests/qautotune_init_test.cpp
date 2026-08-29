#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qautotune/qautotune.hpp>

using fwcpp::qautotune::QAutotuneInitInputs;
using fwcpp::qautotune::resolve_qautotune_init;

TEST_CASE("qautotune init requires quadplane", "[qautotune][init]") {
    QAutotuneInitInputs in{};
    in.quadplane_available = false;
    REQUIRE_FALSE(resolve_qautotune_init(in).ok);
}

TEST_CASE("qautotune init position hold from qloiter", "[qautotune][init]") {
    QAutotuneInitInputs in{};
    in.quadplane_available = true;
    in.previous_mode_was_qloiter = true;
    const auto r = resolve_qautotune_init(in);
    REQUIRE(r.ok);
    REQUIRE(r.position_hold);
}
