#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_modes/q_modes.hpp>

using fwcpp::q_modes::QAcroRunPhase;
using fwcpp::q_modes::QHoverRunPhase;
using fwcpp::q_modes::QModeNumber;
using fwcpp::q_modes::QStabilizeRunPhase;
using fwcpp::q_modes::qacro_body_rates_from_sticks;
using fwcpp::q_modes::qacro_enter;
using fwcpp::q_modes::qacro_run_actions;
using fwcpp::q_modes::qacro_run_phase;
using fwcpp::q_modes::qhover_enter;
using fwcpp::q_modes::qhover_run_actions;
using fwcpp::q_modes::qhover_run_phase;
using fwcpp::q_modes::qstabilize_enter;
using fwcpp::q_modes::qstabilize_run_actions;
using fwcpp::q_modes::qstabilize_run_phase;
using fwcpp::q_modes::QAcroRunInputs;
using fwcpp::q_modes::QHoverRunInputs;
using fwcpp::q_modes::QStabilizeRunInputs;

TEST_CASE("q mode numbers match upstream", "[q_modes][meta]") {
    REQUIRE(static_cast<std::uint8_t>(QModeNumber::kQstabilize) == 17);
    REQUIRE(static_cast<std::uint8_t>(QModeNumber::kQhover) == 18);
    REQUIRE(static_cast<std::uint8_t>(QModeNumber::kQacro) == 23);
}

TEST_CASE("qstabilize run branches", "[q_modes][run]") {
    REQUIRE(qstabilize_enter().throttle_wait == false);
    QStabilizeRunInputs in{};
    REQUIRE(qstabilize_run_phase(in) == QStabilizeRunPhase::kNormal);
    auto actions = qstabilize_run_actions(QStabilizeRunPhase::kNormal);
    REQUIRE(actions.hold_stabilize);
    REQUIRE(actions.center_rudder);
    in.esc_calibration = 1;
    REQUIRE(qstabilize_run_phase(in) == QStabilizeRunPhase::kEscCalibration);
    in.esc_calibration = 0;
    in.tailsitter_in_vtol_transition = true;
    REQUIRE(qstabilize_run_phase(in) == QStabilizeRunPhase::kFwTransitionControllers);
}

TEST_CASE("qhover run branches", "[q_modes][run]") {
    REQUIRE(qhover_enter().climb_rate_ms == 0.0f);
    QHoverRunInputs in{};
    in.throttle_wait = true;
    REQUIRE(qhover_run_phase(in) == QHoverRunPhase::kThrottleWait);
    auto wait = qhover_run_actions(QHoverRunPhase::kThrottleWait);
    REQUIRE(wait.ground_idle_spool);
    REQUIRE(wait.relax_pos_z);
    in.throttle_wait = false;
    REQUIRE(qhover_run_phase(in) == QHoverRunPhase::kHoldHover);
    REQUIRE(qhover_run_actions(QHoverRunPhase::kHoldHover).hold_hover);
}

TEST_CASE("qacro run and tailsitter rates", "[q_modes][run]") {
    REQUIRE(qacro_enter().force_transition_complete);
    QAcroRunInputs in{};
    in.throttle_wait = true;
    REQUIRE(qacro_run_phase(in) == QAcroRunPhase::kThrottleWait);
    in.throttle_wait = false;
    REQUIRE(qacro_run_phase(in) == QAcroRunPhase::kAcroRates);
    REQUIRE(qacro_run_actions(QAcroRunPhase::kAcroRates).run_mode_acro_fw);
    auto ts = qacro_body_rates_from_sticks(1.0f, 0.5f, -0.25f, 100.0f, 80.0f, 60.0f, true);
    REQUIRE(ts.roll_cds == -1500.0f);
    REQUIRE(ts.pitch_cds == 4000.0f);
    REQUIRE(ts.yaw_cds == -10000.0f);
}
