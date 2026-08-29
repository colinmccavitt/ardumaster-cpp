#include <catch2/catch_test_macros.hpp>
#include <fwcpp/vtol_assist/assist_recovery.hpp>

using fwcpp::vtol_assist::AssistRecoveryActions;
using fwcpp::vtol_assist::AssistRecoveryInputs;
using fwcpp::vtol_assist::AssistRecoveryLatch;
using fwcpp::vtol_assist::AssistOption;
using fwcpp::vtol_assist::SpinRecoveryOutputs;
using fwcpp::vtol_assist::VtolAssist;
using fwcpp::vtol_assist::check_vtol_recovery;
using fwcpp::vtol_assist::kAssistServoMax;
using fwcpp::vtol_assist::output_spin_recovery;
using fwcpp::math::radians;

static AssistRecoveryInputs base_inputs() {
    AssistRecoveryInputs in{};
    in.lean_angle_max_cd = 3000.0f;
    in.wp_default_speed_ne_ms = 5.0f;
    return in;
}

TEST_CASE("fw recovery blocked by option or mode", "[vtol_assist][recovery]") {
    VtolAssist assist = VtolAssist::with_defaults();
    assist.set_options(static_cast<std::int16_t>(AssistOption::kFwForceDisabled));
    AssistRecoveryLatch latch{};
    latch.force_fw_control_recovery = true;
    latch.in_spin_recovery = true;
    AssistRecoveryActions actions{};
    auto in = base_inputs();
    in.roll_sensor_cd = 9000.0f;
    REQUIRE_FALSE(check_vtol_recovery(assist, in, latch, actions));
    REQUIRE_FALSE(latch.force_fw_control_recovery);
    REQUIRE_FALSE(latch.in_spin_recovery);

    assist.set_options(0);
    in.in_qacro_mode = true;
    latch.force_fw_control_recovery = true;
    REQUIRE_FALSE(check_vtol_recovery(assist, in, latch, actions));
    REQUIRE_FALSE(latch.force_fw_control_recovery);
}

TEST_CASE("fw recovery trigger and release", "[vtol_assist][recovery]") {
    VtolAssist assist = VtolAssist::with_defaults();
    AssistRecoveryLatch latch{};
    AssistRecoveryActions actions{};
    auto in = base_inputs();
    in.roll_sensor_cd = 7000.0f;
    in.pitch_sensor_cd = 0.0f;
    REQUIRE(check_vtol_recovery(assist, in, latch, actions));
    REQUIRE(latch.force_fw_control_recovery);
    REQUIRE_FALSE(actions.reset_attitude_target_and_rate);

    in.roll_sensor_cd = 2000.0f;
    in.groundspeed_ms = 10.0f;
    REQUIRE_FALSE(check_vtol_recovery(assist, in, latch, actions));
    REQUIRE_FALSE(latch.force_fw_control_recovery);
    REQUIRE(actions.reset_attitude_target_and_rate);
    REQUIRE(actions.pos_control_d_init);
    REQUIRE(actions.pos_control_ne_init);

    in.groundspeed_ms = 1.0f;
    latch.force_fw_control_recovery = true;
    in.roll_sensor_cd = 2500.0f;
    actions = {};
    REQUIRE_FALSE(check_vtol_recovery(assist, in, latch, actions));
    REQUIRE(actions.reset_attitude_target_and_rate);
    REQUIRE_FALSE(actions.pos_control_d_init);
}

TEST_CASE("spin recovery latch", "[vtol_assist][recovery][spin]") {
    VtolAssist assist = VtolAssist::with_defaults();
    AssistRecoveryLatch latch{};
    AssistRecoveryActions actions{};
    auto in = base_inputs();
    in.roll_sensor_cd = 7000.0f;
    in.gyro = {radians(-35.0f), radians(-35.0f), radians(15.0f)};
    in.pitch_deg = -50.0f;
    REQUIRE(check_vtol_recovery(assist, in, latch, actions));
    REQUIRE(latch.force_fw_control_recovery);
    REQUIRE(latch.in_spin_recovery);

    assist.set_options(static_cast<std::int16_t>(AssistOption::kSpinDisabled));
    REQUIRE(check_vtol_recovery(assist, in, latch, actions));
    REQUIRE_FALSE(latch.in_spin_recovery);
}

TEST_CASE("output spin recovery surfaces", "[vtol_assist][recovery][spin]") {
    AssistRecoveryLatch latch{};
    latch.in_spin_recovery = true;
    AssistRecoveryInputs in{};
    in.vtol_motors_throttle_unlimited = true;
    in.gyro = {0.0f, 0.0f, 1.0f};
    SpinRecoveryOutputs out = output_spin_recovery(in, latch);
    REQUIRE(out.apply_surfaces);
    REQUIRE(out.rudder_scaled == -kAssistServoMax);
    REQUIRE(out.elevator_scaled == 0.0f);
    REQUIRE(latch.in_spin_recovery);

    in.gyro.z = -1.0f;
    out = output_spin_recovery(in, latch);
    REQUIRE(out.rudder_scaled == kAssistServoMax);

    in.vtol_motors_throttle_unlimited = false;
    out = output_spin_recovery(in, latch);
    REQUIRE(out.cleared_in_spin_recovery);
    REQUIRE_FALSE(latch.in_spin_recovery);
    REQUIRE_FALSE(out.apply_surfaces);
}