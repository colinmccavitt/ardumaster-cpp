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

using fwcpp::q_modes::LimitedRollPitchInputs;
using fwcpp::q_modes::QAcroUpdateInputs;
using fwcpp::q_modes::QHoverEnterEffects;
using fwcpp::q_modes::QHoverEnterInputs;
using fwcpp::q_modes::QStabilizeUpdateInputs;
using fwcpp::q_modes::TailsitterRollPitchInputs;
using fwcpp::q_modes::qacro_update;
using fwcpp::q_modes::qhover_enter_apply_d_limits;
using fwcpp::q_modes::qhover_update;
using fwcpp::q_modes::qstabilize_update;
using fwcpp::q_modes::set_limited_roll_pitch;
using fwcpp::q_modes::set_tailsitter_roll_pitch;
using fwcpp::poscontrol::DLimits;
using fwcpp::pid::AcP1d;
using fwcpp::pid::AcPid;


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

TEST_CASE("qstabilize update and roll pitch helpers", "[q_modes][update]") {
    QStabilizeUpdateInputs in{};
    in.roll_control_in = 4500.0f;
    in.pitch_control_in = -2250.0f;
    in.roll_limit_cd = 3000.0f;
    in.pitch_limit_min_deg = 25.0f;
    in.lean_angle_max_cd = 4500.0f;
    auto upd = qstabilize_update(in);
    REQUIRE(upd.nav.nav_roll_cd == 3000.0f);
    REQUIRE(upd.nav.nav_pitch_cd == 1250.0f);

    in.ignore_fw_angle_limits_in_q_modes = true;
    upd = qstabilize_update(in);
    REQUIRE(upd.nav.nav_roll_cd == 4500.0f);

    in.tailsitter_active = true;
    in.tailsitter_max_roll_angle_deg = 30.0f;
    in.roll_control_in = 4500.0f;
    auto ts = set_tailsitter_roll_pitch({1.0f, 0.5f, 30.0f, 4500.0f});
    REQUIRE(ts.nav.nav_roll_cd == 3000.0f);
    REQUIRE(ts.apply_vtol_roll_pitch_limit);
    REQUIRE(qhover_update(in).nav.nav_roll_cd == 3000.0f);
}

TEST_CASE("qhover enter D limits wiring", "[q_modes][update]") {
    QHoverEnterInputs in{2.0f, 3.0f, 1.5f};
    QHoverEnterEffects effects{};
    (void)qhover_enter(in, effects);
    REQUIRE(effects.set_d_max_speed_accel);
    REQUIRE(effects.pilot_speed_z_max_up_ms == 3.0f);
    AcP1d pos_p{};
    AcPid accel = AcPid(AcPid::Gains{.p = 0.05f, .imax = 0.8f});
    auto applied = qhover_enter_apply_d_limits(DLimits::defaults(), pos_p, accel, effects);
    REQUIRE(applied.applied_max_speed_accel);
    REQUIRE(applied.applied_correction_speed_accel);
    REQUIRE(applied.limits.vel_max_up_ms == 3.0f);
}

TEST_CASE("qacro update att target", "[q_modes][update]") {
    QAcroUpdateInputs in{1200.0f, -800.0f, 0.0f};
    auto out = qacro_update(in);
    REQUIRE(out.nav_roll_cd == 1200.0f);
    REQUIRE(out.nav_pitch_cd == -800.0f);
}
