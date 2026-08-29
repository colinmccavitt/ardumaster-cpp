#include <catch2/catch_test_macros.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_options.hpp>

using fwcpp::quadplane::QOption;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::kQOptionsDefault;

TEST_CASE("Q_OPTIONS defaults and parsing", "[quadplane][options]") {
    QuadPlane qp;
    REQUIRE(qp.options() == kQOptionsDefault);
    REQUIRE_FALSE(qp.option_is_set(QOption::kQAssistForceEnable));
    qp.set_options(static_cast<std::int32_t>(QOption::kAirmodeUnused));
    REQUIRE(qp.option_is_set(QOption::kAirmodeUnused));
    REQUIRE_FALSE(qp.option_is_set(QOption::kDisableGroundEffectComp));
    qp.set_options(static_cast<std::int32_t>(QOption::kQAssistForceEnable));
    REQUIRE(qp.q_assist_force_enable_from_options());
}

TEST_CASE("motors_init records frame class and type", "[quadplane][motors]") {
    QuadPlane qp{1};
    qp.set_frame_class(2);
    qp.set_frame_type(3);
    REQUIRE(qp.setup());
    const auto params = qp.motors_init_params_if_inited();
    REQUIRE(params.has_value());
    REQUIRE(params->frame_class == 2);
    REQUIRE(params->frame_type == 3);
}