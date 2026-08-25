// Tests for fwcpp::fw_control::{FwController, RollController,
// PitchController, YawController} (CPP-032).
//
// Style note: mirrors tecs_test.cpp/l1_control_test.cpp - drives each
// class through its public entry points and reads back public
// accessors (get_pid_info(), rate_pid()) for white-box checks of the
// internal PID state.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/fw_control/fw_controller.hpp>
#include <fwcpp/fw_control/pitch_controller.hpp>
#include <fwcpp/fw_control/roll_controller.hpp>
#include <fwcpp/fw_control/yaw_controller.hpp>

using namespace fwcpp::fw_control;

namespace {

RateLoopInputs make_rate_inputs(float measured_rate, float airspeed, float eas2tas, float dt, std::uint32_t now_ms) {
    RateLoopInputs in;
    in.measured_rate = measured_rate;
    in.airspeed = airspeed;
    in.eas2tas = eas2tas;
    in.dt = dt;
    in.now_ms = now_ms;
    return in;
}

} // namespace

// ---------------------------------------------------------------------
// RollController
// ---------------------------------------------------------------------

TEST_CASE("RollController: positive/negative angle error produces same-sign servo output", "[fw_control][roll]") {
    RollController::Gains gains;
    FwAparm aparm;
    RollController roll(gains, aparm);

    RateLoopInputs in = make_rate_inputs(0.0f, 15.0f, 1.0f, 0.02f, 1000);
    float out_pos = roll.get_servo_out(1000, 1.0f, false, false, in); // +10 deg error
    REQUIRE(out_pos > 0.0f);

    RollController roll2(gains, aparm);
    float out_neg = roll2.get_servo_out(-1000, 1.0f, false, false, in); // -10 deg error
    REQUIRE(out_neg < 0.0f);
}

TEST_CASE("RollController: rmax_pos clamps the demanded rate before the rate loop even sees it", "[fw_control][roll]") {
    RollController::Gains gains;
    gains.rmax_pos = 30.0f; // deg/s
    FwAparm aparm;
    RollController roll(gains, aparm);

    RateLoopInputs in = make_rate_inputs(0.0f, 15.0f, 1.0f, 0.02f, 1000);
    // angle_err = 90 deg, tau = 0.5 -> naive desired_rate = 180 deg/s, well past rmax_pos
    roll.get_servo_out(9000, 1.0f, false, false, in);
    REQUIRE(roll.get_pid_info().target == Catch::Approx(30.0f));

    RollController roll2(gains, aparm);
    roll2.get_servo_out(-9000, 1.0f, false, false, in);
    REQUIRE(roll2.get_pid_info().target == Catch::Approx(-30.0f));
}

TEST_CASE("RollController: near-180-degree indecision correction flips the demanded rate's sign", "[fw_control][roll]") {
    RollController::Gains gains; // rmax_pos = 0 (disabled) so it doesn't interfere
    FwAparm aparm;
    RollController roll(gains, aparm);

    RateLoopInputs in = make_rate_inputs(0.0f, 15.0f, 1.0f, 0.02f, 1000);

    // First call: small negative angle error establishes a negative
    // last_desired_rate (stored as pid_info().target) with no indecision
    // logic involved (|angle_err_deg| well under the 160 deg threshold).
    roll.get_servo_out(-1000, 1.0f, false, false, in); // -10 deg -> desired_rate -20 deg/s
    REQUIRE(roll.get_pid_info().target == Catch::Approx(-20.0f));

    // Second call: a 170 degree error naively demands a LARGE POSITIVE
    // rate (170/0.5 = 340 deg/s) - but since that disagrees in sign with
    // the previous call's demanded rate, and |angle_err_deg| (170) is
    // both > the 160 deg indecision threshold and <= 180, the correction
    // engages: it flips the sign negative and scales up the magnitude by
    // (170 + (180-170)*2)/170 = 190/170.
    in.now_ms = 1020;
    roll.get_servo_out(17000, 1.0f, false, false, in);
    const float expected = -(170.0f / 0.5f) * (190.0f / 170.0f); // -380
    REQUIRE(roll.get_pid_info().target == Catch::Approx(expected));
    REQUIRE(roll.get_pid_info().target < 0.0f); // the whole point: sign got flipped from the naive +340
}

TEST_CASE("RollController: underspeed locks the rate integrator", "[fw_control][roll]") {
    RollController::Gains gains;
    gains.rate_pid.i = 0.5f; // need a real I gain for this test to be meaningful
    FwAparm aparm; // airspeed_min = 9.0f default

    RollController underspeed_roll(gains, aparm);
    RollController normal_roll(gains, aparm);

    std::uint32_t t = 1000;
    for (int i = 0; i < 20; ++i) {
        RateLoopInputs in_under = make_rate_inputs(0.0f, 5.0f, 1.0f, 0.02f, t); // below airspeed_min
        RateLoopInputs in_normal = make_rate_inputs(0.0f, 15.0f, 1.0f, 0.02f, t); // above airspeed_min
        underspeed_roll.get_servo_out(2000, 1.0f, false, false, in_under); // constant nonzero angle error
        normal_roll.get_servo_out(2000, 1.0f, false, false, in_normal);
        t += 20;
    }

    // Underspeed: is_underspeed() true every tick -> the integrator is
    // restored to its pre-update value (old_i) every single call, so it
    // never moves off its zero starting point.
    REQUIRE(underspeed_roll.rate_pid().get_i() == Catch::Approx(0.0f));
    // Normal speed: the same constant error accumulates over 20 ticks.
    REQUIRE(normal_roll.rate_pid().get_i() != Catch::Approx(0.0f));
}

// ---------------------------------------------------------------------
// FwController: shared rate-loop behaviors (ground mode, FF/DFF scaling)
// ---------------------------------------------------------------------

TEST_CASE("FwController: ground_mode subtracts D + half of P from the summed output", "[fw_control]") {
    fwcpp::pid::AcPid::Gains g{
        .p = 0.2f, .i = 0.0f, .d = 0.1f, .ff = 0.3f, .imax = 1.0f, .filt_t_hz = 0.0f,
        .filt_e_hz = 0.0f,    .filt_d_hz = 0.0f,     .srmax = 0.0f, .srtau = 1.0f,
    };

    FwController normal(g);
    FwController grounded(g);

    RateLoopInputs in = make_rate_inputs(0.1f, 15.0f, 1.0f, 0.02f, 1000);

    const float out_normal = normal.get_rate_out_full(20.0f, 1.0f, false, false, false, in);
    const float out_grounded = grounded.get_rate_out_full(20.0f, 1.0f, false, false, true, in);

    // pid_info fields (ff/p/i/d/dff) are identical regardless of
    // ground_mode - only the final summation differs.
    const auto& info = normal.get_pid_info();
    REQUIRE(grounded.get_pid_info().p == Catch::Approx(info.p));
    REQUIRE(grounded.get_pid_info().d == Catch::Approx(info.d));

    const float suppressed = (out_normal - out_grounded) / 100.0f; // undo the *100 centidegree scale
    REQUIRE(suppressed == Catch::Approx(info.d + 0.5f * info.p));
}

TEST_CASE("FwController: FF output scales linearly with scaler and inversely with eas2tas", "[fw_control]") {
    fwcpp::pid::AcPid::Gains g{
        .p = 0.0f, .i = 0.0f, .d = 0.0f, .ff = 1.0f, .imax = 1.0f, .filt_t_hz = 0.0f,
        .filt_e_hz = 0.0f,    .filt_d_hz = 0.0f,     .srmax = 0.0f, .srtau = 1.0f,
    };

    // ff_final = desired_rate * kff * scaler / eas2tas (derived in
    // fw_controller.hpp's own get_rate_out_full - see the scaling
    // comment there). Checked here against three independent
    // (scaler, eas2tas) pairs with kff = 1, desired_rate = 10.
    struct Case {
        float scaler;
        float eas2tas;
    };
    for (const Case& c : {Case{1.0f, 1.0f}, Case{2.0f, 1.0f}, Case{1.0f, 2.0f}}) {
        FwController fw(g);
        RateLoopInputs in = make_rate_inputs(0.0f, 15.0f, c.eas2tas, 0.02f, 1000);
        fw.get_rate_out_full(10.0f, c.scaler, /*disable_integrator=*/true, false, false, in);
        const float expected_ff = 10.0f * c.scaler / c.eas2tas;
        REQUIRE(fw.get_pid_info().ff == Catch::Approx(expected_ff));
    }
}

TEST_CASE("FwController: DFF output scales the same way as FF (by scaler, inversely by eas2tas)", "[fw_control]") {
    fwcpp::pid::AcPid::Gains g{
        .p = 0.0f,        .i = 0.0f,         .d = 0.0f, .ff = 0.0f, .imax = 1.0f, .filt_t_hz = 0.0f,
        .filt_e_hz = 0.0f, .filt_d_hz = 0.0f, .srmax = 0.0f, .srtau = 1.0f, .dff = 1.0f,
    };

    auto run = [&](float scaler, float eas2tas) {
        FwController fw(g);
        RateLoopInputs in1 = make_rate_inputs(0.0f, 15.0f, eas2tas, 0.02f, 1000);
        fw.get_rate_out_full(10.0f, scaler, true, false, false, in1); // reset call, target_derivative_ = 0
        RateLoopInputs in2 = make_rate_inputs(0.0f, 15.0f, eas2tas, 0.02f, 1020);
        fw.get_rate_out_full(20.0f, scaler, true, false, false, in2); // target changes -> nonzero dff
        return fw.get_pid_info().dff;
    };

    const float dff_baseline = run(1.0f, 1.0f);
    const float dff_double_scaler = run(2.0f, 1.0f);
    REQUIRE(dff_baseline != Catch::Approx(0.0f));
    // Doubling scaler doubles the AcPid target fed in (scaler^2 term)
    // relative to the scaler*eas2tas divisor, i.e. net one extra factor
    // of scaler - see fw_controller.hpp's scaling comment.
    REQUIRE(dff_double_scaler / dff_baseline == Catch::Approx(2.0f).margin(0.05f));
}

// ---------------------------------------------------------------------
// PitchController
// ---------------------------------------------------------------------

TEST_CASE("PitchController: positive/negative angle error produces same-sign servo output", "[fw_control][pitch]") {
    PitchController::Gains gains;
    FwAparm aparm;
    PitchController pitch(gains, aparm);

    PitchController::Gains g2;
    PitchController pitch2(g2, aparm);

    fwcpp::fw_control::PitchInputs in;
    in.measured_rate = 0.0f;
    in.airspeed = 15.0f;
    in.eas2tas = 1.0f;
    in.dt = 0.02f;
    in.now_ms = 1000;
    in.bank_angle_rad = 0.0f; // wings level: no turn-coordination offset to complicate the sign check
    in.pitch_rad = 0.0f;

    float out_pos = pitch.get_servo_out(1000, 1.0f, false, false, in); // +10 deg
    REQUIRE(out_pos > 0.0f);

    float out_neg = pitch2.get_servo_out(-1000, 1.0f, false, false, in); // -10 deg
    REQUIRE(out_neg < 0.0f);
}

TEST_CASE("PitchController: banking engages a nonzero turn-coordination rate offset even with zero angle error",
          "[fw_control][pitch]") {
    PitchController::Gains gains;
    FwAparm aparm;
    PitchController level(gains, aparm);
    PitchController banked(gains, aparm);

    fwcpp::fw_control::PitchInputs in_level;
    in_level.airspeed = 15.0f;
    in_level.eas2tas = 1.0f;
    in_level.dt = 0.02f;
    in_level.now_ms = 1000;
    in_level.bank_angle_rad = 0.0f;

    fwcpp::fw_control::PitchInputs in_banked = in_level;
    in_banked.bank_angle_rad = fwcpp::math::radians(45.0f);

    level.get_servo_out(0, 1.0f, false, false, in_level);
    banked.get_servo_out(0, 1.0f, false, false, in_banked);

    // Zero angle error -> desired_rate is purely the coordination offset.
    REQUIRE(level.get_pid_info().target == Catch::Approx(0.0f));
    REQUIRE(banked.get_pid_info().target > 0.0f); // nose must rise to hold height in the bank
}

TEST_CASE("PitchController: rmax_pos/rmax_neg clamp the demanded rate", "[fw_control][pitch]") {
    PitchController::Gains gains;
    gains.rmax_pos = 15.0f;
    gains.rmax_neg = 10.0f;
    FwAparm aparm;
    PitchController pitch_up(gains, aparm);
    PitchController pitch_down(gains, aparm);

    fwcpp::fw_control::PitchInputs in;
    in.airspeed = 15.0f;
    in.eas2tas = 1.0f;
    in.dt = 0.02f;
    in.now_ms = 1000;

    pitch_up.get_servo_out(9000, 1.0f, false, false, in); // +90 deg -> naive +180 deg/s
    REQUIRE(pitch_up.get_pid_info().target == Catch::Approx(15.0f));

    pitch_down.get_servo_out(-9000, 1.0f, false, false, in); // -90 deg -> naive -180 deg/s
    REQUIRE(pitch_down.get_pid_info().target == Catch::Approx(-10.0f));
}

TEST_CASE("PitchController: excessive bank angle linearly reduces pitch authority toward zero", "[fw_control][pitch]") {
    PitchController::Gains gains;
    FwAparm aparm; // roll_limit_deg = 45.0f default -> margin = 50 deg

    PitchController moderate_bank(gains, aparm);
    PitchController steep_bank(gains, aparm);

    fwcpp::fw_control::PitchInputs in;
    in.airspeed = 15.0f;
    in.eas2tas = 1.0f;
    in.dt = 0.02f;
    in.now_ms = 1000;
    in.bank_angle_rad = fwcpp::math::radians(10.0f); // well under the 50 deg margin: no reduction
    in.pitch_rad = 0.0f;

    // A large angle error (90 deg -> 180 deg/s base demand) dominates
    // both scenarios' own turn-coordination offset, isolating the
    // roll-limit blend's effect from _get_coordination_rate_offset()'s
    // separate (and, near 70-80 deg of bank, much larger) contribution.
    moderate_bank.get_servo_out(9000, 1.0f, false, false, in);
    const float target_moderate = moderate_bank.get_pid_info().target;

    in.bank_angle_rad = fwcpp::math::radians(70.0f); // past the margin: roll_prop = (70-50)/(90-50) = 0.5
    steep_bank.get_servo_out(9000, 1.0f, false, false, in);
    const float target_steep = steep_bank.get_pid_info().target;

    // The steep case's demanded rate must be markedly smaller (roughly
    // half, modulo each case's own additive coordination-offset term)
    // due to the roll-limit blend.
    REQUIRE(std::fabs(target_steep) < std::fabs(target_moderate));
}

// ---------------------------------------------------------------------
// YawController - REAL contract: get_servo_out() is a sideslip/turn-
// coordination controller with NO angle-error input at all (see
// yaw_controller.hpp's file banner for how this was verified by reading
// upstream, not assumed). get_rate_out() is the separate direct-rate
// aerobatic entry point that DOES reuse the shared AC_PID rate loop.
// ---------------------------------------------------------------------

TEST_CASE("YawController: enabled()/rate_control_enabled() reflect the configured gains", "[fw_control][yaw]") {
    YawController::Gains gains;
    FwAparm aparm;

    gains.k_d = 0.0f;
    gains.rate_enable = false;
    {
        YawController yaw(gains, aparm);
        REQUIRE_FALSE(yaw.enabled());
        REQUIRE_FALSE(yaw.rate_control_enabled());
    }

    gains.k_d = 0.5f;
    {
        YawController yaw(gains, aparm);
        REQUIRE(yaw.enabled()); // damping alone is enough
        REQUIRE_FALSE(yaw.rate_control_enabled());
    }

    gains.k_d = 0.0f;
    gains.rate_enable = true;
    {
        YawController yaw(gains, aparm);
        REQUIRE(yaw.enabled());
        REQUIRE(yaw.rate_control_enabled());
    }
}

TEST_CASE("YawController: get_servo_out returns exactly 0 when yaw damping is disabled, regardless of inputs",
          "[fw_control][yaw]") {
    YawController::Gains gains; // k_d = 0.0f default
    FwAparm aparm;
    YawController yaw(gains, aparm);

    YawCoordinationInputs in;
    in.bank_angle_rad = fwcpp::math::radians(30.0f);
    in.gyro_z = 0.1f;
    in.accel_y = 5.0f; // plenty of lateral acceleration to react to, if damping were enabled
    in.airspeed_valid = true;
    in.airspeed_eas = 15.0f;
    in.now_ms = 1000;

    REQUIRE(yaw.get_servo_out(1.0f, false, in) == 0);
}

TEST_CASE("YawController: sideslip integrator accumulates against sustained lateral acceleration", "[fw_control][yaw]") {
    YawController::Gains gains;
    gains.k_a = 1.0f;
    gains.k_i = 1.0f;
    gains.k_d = 0.5f; // must be nonzero for the integrator to run and for output to be nonzero
    FwAparm aparm;
    YawController yaw(gains, aparm);

    YawCoordinationInputs in;
    in.bank_angle_rad = 0.0f;
    in.gyro_z = 0.0f;
    in.accel_y = 2.0f; // sustained positive lateral acceleration (sideslip)
    in.airspeed_valid = true;
    in.airspeed_eas = 15.0f;

    std::int32_t first_out = 0;
    std::int32_t last_out = 0;
    std::uint32_t t = 1000;
    for (int i = 0; i < 10; ++i) {
        in.now_ms = t;
        std::int32_t out = yaw.get_servo_out(1.0f, false, in);
        if (i == 0) {
            first_out = out;
        }
        last_out = out;
        t += 20;
    }

    // integ_in = -k_i * (k_a * accel_y + rate_hp_out); with accel_y > 0
    // and k_a,k_i > 0 the integrator (and hence the magnitude of the
    // rudder correction opposing the sideslip) must grow over time from
    // a sustained, unchanging input.
    REQUIRE(std::abs(last_out) > std::abs(first_out));
    REQUIRE(last_out < 0); // positive accel_y -> negative integ_in -> negative integrator -> negative rudder demand
}

TEST_CASE("YawController: get_rate_out (aerobatic direct-rate entry) FF scales by scaler/eas2tas like roll/pitch",
          "[fw_control][yaw]") {
    YawController::Gains gains;
    gains.rate_pid.p = 0.0f;
    gains.rate_pid.i = 0.0f;
    gains.rate_pid.d = 0.0f;
    gains.rate_pid.ff = 1.0f;
    FwAparm aparm;

    YawController yaw_a(gains, aparm);
    YawController yaw_b(gains, aparm);

    RateLoopInputs in_a = make_rate_inputs(0.0f, 15.0f, 1.0f, 0.02f, 1000);
    RateLoopInputs in_b = make_rate_inputs(0.0f, 15.0f, 2.0f, 0.02f, 1000); // double eas2tas

    yaw_a.get_rate_out(10.0f, 1.0f, true, in_a);
    yaw_b.get_rate_out(10.0f, 1.0f, true, in_b);

    // ff = desired_rate * kff * scaler / eas2tas -> doubling eas2tas halves ff.
    REQUIRE(yaw_a.get_pid_info().ff == Catch::Approx(2.0f * yaw_b.get_pid_info().ff));
}

TEST_CASE("YawController: decay_I only decays the logged pid_info, not the real rate integrator (upstream quirk)",
          "[fw_control][yaw]") {
    YawController::Gains gains;
    gains.rate_pid.i = 1.0f;
    FwAparm aparm;
    YawController yaw(gains, aparm);

    RateLoopInputs in = make_rate_inputs(0.0f, 15.0f, 1.0f, 0.02f, 1000);
    yaw.get_rate_out(20.0f, 1.0f, false, in); // build up a real, nonzero rate_pid integrator

    const float real_integrator_before = yaw.rate_pid().get_i();
    const float logged_i_before = yaw.get_pid_info().i;
    REQUIRE(logged_i_before != Catch::Approx(0.0f));

    yaw.decay_I();

    REQUIRE(yaw.get_pid_info().i == Catch::Approx(logged_i_before * 0.995f));
    // The real rate_pid integrator is untouched - see yaw_controller.hpp's
    // file banner for why this faithfully reproduces upstream rather than
    // "fixing" it to match FwController::decay_i()'s behavior.
    REQUIRE(yaw.rate_pid().get_i() == Catch::Approx(real_integrator_before));
}

TEST_CASE("YawController: reset_I zeroes the logged integrator, the real rate_pid integrator, and the sideslip integrator",
          "[fw_control][yaw]") {
    YawController::Gains gains;
    gains.k_a = 1.0f;
    gains.k_i = 1.0f;
    gains.k_d = 0.5f;
    gains.rate_pid.i = 1.0f;
    FwAparm aparm;
    YawController yaw(gains, aparm);

    YawCoordinationInputs coord_in;
    coord_in.accel_y = 2.0f;
    coord_in.airspeed_valid = true;
    coord_in.airspeed_eas = 15.0f;
    coord_in.now_ms = 1000;
    yaw.get_servo_out(1.0f, false, coord_in); // build up the sideslip integrator

    RateLoopInputs rate_in = make_rate_inputs(0.0f, 15.0f, 1.0f, 0.02f, 1020);
    yaw.get_rate_out(20.0f, 1.0f, false, rate_in); // build up the real rate_pid integrator

    yaw.reset_I();

    REQUIRE(yaw.get_pid_info().i == Catch::Approx(0.0f));
    REQUIRE(yaw.rate_pid().get_i() == Catch::Approx(0.0f));

    // With the sideslip integrator zeroed, the very next get_servo_out
    // call (fresh dt, so no immediate re-accumulation this same tick)
    // should not carry forward the old integrator's contribution.
    coord_in.now_ms = 1021; // 21ms after the original coordination call
    const std::int32_t out_after_reset = yaw.get_servo_out(1.0f, false, coord_in);
    // Starting from a zeroed sideslip integrator, one 21ms tick of
    // integ_in = -k_i*(k_a*accel_y) = -2.0 only moves the integrator by
    // -0.042, nowhere near enough to produce a large rudder demand.
    REQUIRE(std::abs(out_after_reset) < 50);
}
