#include <catch2/catch_test_macros.hpp>
#include <fwcpp/qautotune/qautotune_init_internals.hpp>

using fwcpp::qautotune::QAutotuneInitInputs;
using fwcpp::qautotune::QAutotuneInitInternalsArgs;
using fwcpp::qautotune::resolve_qautotune_init_internals;

TEST_CASE("qautotune init_internals unavailable skips wiring", "[qautotune][init]") {
    QAutotuneInitInputs init{};
    init.quadplane_available = false;
    QAutotuneInitInternalsArgs handles{};
    handles.attitude_control = reinterpret_cast<void*>(0x1);
    handles.pos_control = reinterpret_cast<void*>(0x2);
    handles.ahrs_view = reinterpret_cast<void*>(0x3);

    const auto r = resolve_qautotune_init_internals(init, handles);
    REQUIRE_FALSE(r.ok);
    REQUIRE_FALSE(r.args.position_hold);
    REQUIRE(r.args.attitude_control == nullptr);
    REQUIRE(r.args.pos_control == nullptr);
    REQUIRE(r.args.ahrs_view == nullptr);
}

TEST_CASE("qautotune init_internals wires four args from qloiter", "[qautotune][init]") {
    QAutotuneInitInputs init{};
    init.quadplane_available = true;
    init.previous_mode_was_qloiter = true;
    QAutotuneInitInternalsArgs handles{};
    handles.attitude_control = reinterpret_cast<void*>(0x1);
    handles.pos_control = reinterpret_cast<void*>(0x2);
    handles.ahrs_view = reinterpret_cast<void*>(0x3);

    const auto r = resolve_qautotune_init_internals(init, handles);
    REQUIRE(r.ok);
    REQUIRE(r.args.position_hold);
    REQUIRE(r.args.attitude_control == handles.attitude_control);
    REQUIRE(r.args.pos_control == handles.pos_control);
    REQUIRE(r.args.ahrs_view == handles.ahrs_view);
}

TEST_CASE("qautotune init_internals position_hold false off qloiter", "[qautotune][init]") {
    QAutotuneInitInputs init{};
    init.quadplane_available = true;
    init.previous_mode_was_qloiter = false;
    QAutotuneInitInternalsArgs handles{};
    handles.attitude_control = reinterpret_cast<void*>(0x4);

    const auto r = resolve_qautotune_init_internals(init, handles);
    REQUIRE(r.ok);
    REQUIRE_FALSE(r.args.position_hold);
    REQUIRE(r.args.attitude_control == handles.attitude_control);
}
