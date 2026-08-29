#include <catch2/catch_test_macros.hpp>
#include <fwcpp/vtol_assist/assist_gcs_getters.hpp>
#include <fwcpp/vtol_assist/assist_hysteresis.hpp>
#include <fwcpp/vtol_assist/should_assist.hpp>

using fwcpp::vtol_assist::AssistActiveLatch;
using fwcpp::vtol_assist::ShouldAssistHysteresis;
using fwcpp::vtol_assist::ShouldAssistResult;
using fwcpp::vtol_assist::assist_active_latch_from_result;
using fwcpp::vtol_assist::assist_gcs_statustext_from_edges;
using fwcpp::vtol_assist::in_alt_assist;
using fwcpp::vtol_assist::in_angle_assist;
using fwcpp::vtol_assist::in_force_assist;
using fwcpp::vtol_assist::in_speed_assist;

TEST_CASE("in_*_assist getters mirror upstream", "[vtol_assist][gcs][getters]") {
    ShouldAssistHysteresis hysteresis{};
    hysteresis.alt_error.update(true, 100u, 500u, 1000u);
    hysteresis.angle_error.update(true, 100u, 500u, 1000u);
    hysteresis.alt_error.update(true, 700u, 500u, 1000u);
    hysteresis.angle_error.update(true, 700u, 500u, 1000u);
    REQUIRE(hysteresis.alt_error.is_active());
    REQUIRE(hysteresis.angle_error.is_active());

    ShouldAssistResult result{};
    result.force_assist = true;
    result.speed_assist = true;
    result.alt_assist = true;
    result.angle_assist = true;

    AssistActiveLatch latch = assist_active_latch_from_result(result, hysteresis);
    REQUIRE(in_force_assist(latch));
    REQUIRE(in_speed_assist(latch));
    REQUIRE(in_alt_assist(latch));
    REQUIRE(in_angle_assist(latch));
}

TEST_CASE("assist gcs statustext stubs on first edge", "[vtol_assist][gcs][getters]") {
    ShouldAssistResult result{};
    result.alt_assist_first_edge = true;
    result.angle_assist_first_edge = true;
    auto msgs = assist_gcs_statustext_from_edges(result, 12.5f, -10, 5);
    REQUIRE(msgs.alt_assist.has_value());
    REQUIRE(msgs.alt_assist->find("Alt assist") != std::string::npos);
    REQUIRE(msgs.alt_assist->find("12.5") != std::string::npos);
    REQUIRE(msgs.angle_assist.has_value());
    REQUIRE(msgs.angle_assist->find("Angle assist") != std::string::npos);
    REQUIRE(msgs.angle_assist->find("r=-10") != std::string::npos);
    REQUIRE(msgs.angle_assist->find("p=5") != std::string::npos);

    result.alt_assist_first_edge = false;
    result.angle_assist_first_edge = false;
    msgs = assist_gcs_statustext_from_edges(result, 1.0f, 0, 0);
    REQUIRE_FALSE(msgs.alt_assist.has_value());
    REQUIRE_FALSE(msgs.angle_assist.has_value());
}