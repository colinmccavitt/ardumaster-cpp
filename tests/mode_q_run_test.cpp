#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_modes/q_modes.hpp>

using fwcpp::q_modes::QAcroRateInputVariant;
using fwcpp::q_modes::QAcroRunPhase;
using fwcpp::q_modes::QHoverRunPhase;
using fwcpp::q_modes::QModeNumber;
using fwcpp::q_modes::QStabilizeRunPhase;
using fwcpp::q_modes::qacro_body_rates_from_sticks;
using fwcpp::q_modes::qacro_enter;
using fwcpp::q_modes::qacro_rate_input_variant;
using fwcpp::q_modes::qacro_run;
using fwcpp::q_modes::qacro_run_actions;
using fwcpp::q_modes::qacro_run_phase;
using fwcpp::q_modes::qhover_enter;
using fwcpp::q_modes::qhover_run;
using fwcpp::q_modes::qhover_run_actions;
using fwcpp::q_modes::qhover_run_phase;
using fwcpp::q_modes::qstabilize_enter;
using fwcpp::q_modes::qstabilize_run;
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
    auto actions = qstabilize_run_actions(QStabilizeRunPhase::kNormal, 0.75f);
    REQUIRE(actions.hold_stabilize);
    REQUIRE(actions.hold_stabilize_throttle == 0.75f);
    REQUIRE(actions.center_rudder);
    in.esc_calibration = 1;
    REQUIRE(qstabilize_run_phase(in) == QStabilizeRunPhase::kEscCalibration);
    in.esc_calibration = 0;
    in.tailsitter_in_vtol_transition = true;
    REQUIRE(qstabilize_run_phase(in) == QStabilizeRunPhase::kFwTransitionControllers);
}

TEST_CASE("qstabilize run orchestrator", "[q_modes][run]") {
    QStabilizeRunInputs in{};
    in.pilot_throttle_scaled = 0.5f;
    auto normal = qstabilize_run(in);
    REQUIRE(normal.actions.assign_tilt_to_fwd_thr);
    REQUIRE(normal.actions.hold_stabilize_throttle == 0.5f);
    REQUIRE(normal.fw_followup.stabilize_roll);
    in.esc_calibration = 2;
    auto esc = qstabilize_run(in);
    REQUIRE(esc.actions.run_esc_calibration);
    REQUIRE(esc.actions.stabilize_fw_surfaces);
    in.esc_calibration = 0;
    in.tailsitter_in_vtol_transition = true;
    REQUIRE(qstabilize_run(in).delegate_mode_run);
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
    auto hold = qhover_run_actions(QHoverRunPhase::kHoldHover, 42.0f);
    REQUIRE(hold.hold_hover);
    REQUIRE(hold.hold_hover_climb_rate_cms == 42.0f);
}

TEST_CASE("qhover run orchestrator", "[q_modes][run]") {
    QHoverRunInputs in{};
    in.pilot_climb_rate_cms = 100.0f;
    auto r = qhover_run(in);
    REQUIRE(r.actions.check_vtol_recovery);
    REQUIRE(r.actions.output_spin_recovery);
    REQUIRE(r.actions.hold_hover_climb_rate_cms == 100.0f);
    REQUIRE(r.fw_followup.center_rudder);
}

TEST_CASE("qacro run and tailsitter rates", "[q_modes][run]") {
    REQUIRE(qacro_enter().force_transition_complete);
    QAcroRunInputs in{};
    in.throttle_wait = true;
    REQUIRE(qacro_run_phase(in) == QAcroRunPhase::kThrottleWait);
    in.throttle_wait = false;
    REQUIRE(qacro_run_phase(in) == QAcroRunPhase::kAcroRates);
    REQUIRE(qacro_run_actions(QAcroRunPhase::kAcroRates, in).run_mode_acro_fw);
    auto ts = qacro_body_rates_from_sticks(1.0f, 0.5f, -0.25f, 100.0f, 80.0f, 60.0f, true);
    REQUIRE(ts.roll_cds == -1500.0f);
    REQUIRE(ts.pitch_cds == 4000.0f);
    REQUIRE(ts.yaw_cds == -10000.0f);
}

TEST_CASE("qacro run orchestrator locking variant", "[q_modes][run]") {
    QAcroRunInputs in{};
    in.acro_locking = true;
    in.roll_norm = 1.0f;
    in.acro_roll_rate = 100.0f;
    in.pilot_throttle_scaled = 0.8f;
    auto r = qacro_run(in);
    REQUIRE(r.actions.rate_input == QAcroRateInputVariant::kLocking3);
    REQUIRE(r.actions.throttle_out == 0.8f);
    REQUIRE(r.actions.rates.roll_cds == 10000.0f);
    in.acro_locking = false;
    REQUIRE(qacro_rate_input_variant(false) == QAcroRateInputVariant::kNoLocking2);
}
