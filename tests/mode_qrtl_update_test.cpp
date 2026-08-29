#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/qrtl/mode_qrtl_target_altitude.hpp>
#include <fwcpp/qrtl/mode_qrtl_update.hpp>

using Catch::Approx;
using fwcpp::quadplane::PositionControlState;
using fwcpp::qrtl::QrtlSubMode;
using fwcpp::qrtl::QrtlTargetAltAction;
using fwcpp::qrtl::QrtlTargetAltView;
using fwcpp::qrtl::qrtl_allows_throttle_nudging;
using fwcpp::qrtl::qrtl_approach_alt_offset_m;
using fwcpp::qrtl::qrtl_update_actions;
using fwcpp::qrtl::qrtl_update_target_altitude;

TEST_CASE("qrtl update delegates QStabilize", "[qrtl][update]") {
    const auto actions = qrtl_update_actions();
    REQUIRE(actions.delegate_qstabilize_update);
}

TEST_CASE("qrtl allows throttle nudging only RTL approach", "[qrtl][update]") {
    REQUIRE_FALSE(qrtl_allows_throttle_nudging(QrtlSubMode::kClimb, PositionControlState::kApproach));
    REQUIRE_FALSE(qrtl_allows_throttle_nudging(QrtlSubMode::kRtl, PositionControlState::kPosition1));
    REQUIRE(qrtl_allows_throttle_nudging(QrtlSubMode::kRtl, PositionControlState::kApproach));
}

TEST_CASE("qrtl update_target_altitude delegates outside approach", "[qrtl][update]") {
    QrtlTargetAltView view{};
    view.submode = QrtlSubMode::kClimb;
    view.poscontrol_state = PositionControlState::kApproach;
    REQUIRE(qrtl_update_target_altitude(view).action == QrtlTargetAltAction::kDelegateBase);

    view.submode = QrtlSubMode::kRtl;
    view.poscontrol_state = PositionControlState::kAirbrake;
    REQUIRE(qrtl_update_target_altitude(view).action == QrtlTargetAltAction::kDelegateBase);
}

TEST_CASE("qrtl approach altitude profile", "[qrtl][update]") {
    QrtlTargetAltView view{};
    view.submode = QrtlSubMode::kRtl;
    view.poscontrol_state = PositionControlState::kApproach;
    view.loiter_radius_m = 50.0F;
    view.rtl_radius_m = 50.0F;
    view.rtl_altitude_m = 60.0F;
    view.qrtl_alt_m = 20.0F;
    view.max_sinkrate_ms = 2.0F;
    view.airspeed_cruise_ms = 10.0F;

    view.wp_distance_m = 100.0F;
    const float radius = 50.0F;
    const float rad_min = 2.0F * radius;
    const float rtl_alt_delta = 40.0F;
    const float expected_far = fwcpp::math::linear_interpolate(0.0F, rtl_alt_delta, 100.0F, rad_min, 20.0F * radius);
    REQUIRE(qrtl_approach_alt_offset_m(view) == Approx(expected_far));

    view.wp_distance_m = rad_min;
    REQUIRE(qrtl_approach_alt_offset_m(view) == Approx(0.0F));

    const auto result = qrtl_update_target_altitude(view);
    REQUIRE(result.action == QrtlTargetAltAction::kApproachProfile);
    REQUIRE(result.approach_offset_up_m == Approx(0.0F));
}
