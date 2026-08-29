// CCP-027 slice 5: DLimits, D init/relax, throttle paths (ADR-0012 explicit params).

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/poscontrol/pos_control.hpp>
#include <fwcpp/pid/ac_p_1d.hpp>
#include <fwcpp/pid/ac_pid.hpp>
#include <fwcpp/pid/ac_pid_basic.hpp>

using namespace fwcpp::poscontrol;
using fwcpp::pid::AcP1d;
using fwcpp::pid::AcPid;
using fwcpp::pid::AcPidBasic;

static DInitInputs d_init_inputs() {
    DInitInputs inp{};
    inp.estimates.pos_m = 12.0;
    inp.estimates.vel_ms = 1.5f;
    inp.estimated_accel_d_mss = 0.2f;
    inp.throttle_in = 0.55f;
    inp.throttle_hover = 0.4f;
    inp.accel_max_d_mss = 2.5f;
    inp.now_ms = 5000;
    inp.ticks = 20;
    inp.last_update_ticks = 19;
    inp.position_d_reset_count = 3;
    return inp;
}

TEST_CASE("DLimits d_set_max_speed_accel respects zeros and filter cap", "[poscontrol][d][limits]") {
    DLimits limits = DLimits::defaults();
    AcPid accel = AcPid(AcPid::Gains{.p = 0.05f, .imax = 0.8f});
    accel.set_filt_T_hz(10.0f);
    accel.set_filt_E_hz(0.0f);

    limits = d_set_max_speed_accel_m(limits, 0.0f, 3.0f, 0.0f, 8.0f, accel);
    REQUIRE(limits.vel_max_down_ms == Catch::Approx(kPoscontrolSpeedDownMs));
    REQUIRE(limits.vel_max_up_ms == Catch::Approx(3.0f));

    limits = d_set_max_speed_accel_m(limits, -2.0f, 0.0f, 4.0f, 8.0f, accel);
    REQUIRE(limits.vel_max_down_ms == Catch::Approx(2.0f));
    REQUIRE(limits.accel_max_d_mss == Catch::Approx(4.0f));
    REQUIRE(limits.jerk_max_d_msss == Catch::Approx(8.0f));
    REQUIRE(limits.jerk_max_d_msss > 0.0f);

    const DLimits from_cm = d_set_max_speed_accel_cm(DLimits{}, 150.0f, 250.0f, 250.0f, 5.0f, accel);
    REQUIRE(from_cm.vel_max_down_ms == Catch::Approx(1.5f));
    REQUIRE(from_cm.vel_max_up_ms == Catch::Approx(2.5f));
}

TEST_CASE("D correction limits map to AcP1d", "[poscontrol][d][limits]") {
    AcP1d pos_p = AcP1d::with_kp(1.0f);
    d_set_correction_speed_accel_m(pos_p, 1.5f, 2.5f, 3.0f);
    REQUIRE(pos_p.error_min() < 0.0f);
    REQUIRE(pos_p.error_max() > 0.0f);
    AcP1d pos_cm = AcP1d::with_kp(1.0f);
    d_set_correction_speed_accel_cm(pos_cm, 150.0f, 250.0f, 300.0f);
    REQUIRE(pos_cm.error_max() == Catch::Approx(pos_p.error_max()).margin(1e-5f));
}

TEST_CASE("calculate_d_overspeed_gain scales descent and climb", "[poscontrol][d][limits]") {
    REQUIRE(calculate_d_overspeed_gain(0.0f, 2.0f, 2.0f) == Catch::Approx(1.0f));
    REQUIRE(calculate_d_overspeed_gain(4.0f, 2.0f, 2.0f) ==
            Catch::Approx(kPoscontrolOverspeedGainU * 2.0f));
    REQUIRE(calculate_d_overspeed_gain(-4.0f, 2.0f, 2.0f) ==
            Catch::Approx(kPoscontrolOverspeedGainU * 2.0f));
}

TEST_CASE("stopping_point_d matches upstream clamp", "[poscontrol][d][init]") {
    const fwcpp::math::postype_t stop =
        stopping_point_d(10.0, 1.0, 2.0f, 0.5f, 1.0f, 2.5f);
    REQUIRE(static_cast<float>(stop) > 8.0f);
    const fwcpp::math::postype_t at_rest = stopping_point_d(5.0, 0.0, 0.0f, 0.0f, 1.0f, 2.5f);
    REQUIRE(static_cast<float>(at_rest) == Catch::Approx(5.0f));
}

TEST_CASE("D init_controller seeds PID state", "[poscontrol][d][init]") {
    PosControlD d{};
    DOffsetState offsets{};
    offsets.target.pos_m = 2.0;
    offsets.target.vel_ms = 0.5f;
    offsets.target_ms = 5000;
    AcPidBasic vel_pid = AcPidBasic::with_gains(5.0f, 0.1f, 0.0f, 0.0f, 10.0f, 5.0f, 5.0f);
    AcPid accel_pid = AcPid(AcPid::Gains{.p = 0.1f, .i = 0.05f, .ff = 0.02f, .imax = 0.8f});
    const DInitOutput out = d.init_controller(offsets, vel_pid, accel_pid, d_init_inputs());

    REQUIRE(out.pos_target_m == Catch::Approx(12.0).margin(1e-5));
    REQUIRE(static_cast<float>(d.pos_desired_m) == Catch::Approx(10.0f).margin(1e-5f));
    REQUIRE(d.vel_desired_ms == Catch::Approx(1.0f).margin(1e-5f));
    REQUIRE(out.accel_target_mss == Catch::Approx(0.2f).margin(1e-5f));
    REQUIRE(out.last_update_ticks == 20);
    const float expected_i = -(0.55f - 0.4f) - 0.1f * (0.2f - 0.2f) - 0.02f * 0.2f;
    REQUIRE(accel_pid.get_i() == Catch::Approx(expected_i).margin(1e-5f));
    REQUIRE(vel_pid.integrator() == Catch::Approx(0.0f));
}

TEST_CASE("D init no descent clamps positive down motion", "[poscontrol][d][init]") {
    PosControlD d{};
    DOffsetState offsets{};
    AcPidBasic vel_pid = AcPidBasic::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = AcPid(AcPid::Gains{.p = 0.1f, .imax = 0.8f});
    DInitInputs inp = d_init_inputs();
    inp.estimates.vel_ms = 2.0f;
    inp.estimated_accel_d_mss = 1.0f;
    const DInitOutput out = d.init_controller_no_descent(offsets, vel_pid, accel_pid, inp);
    REQUIRE(out.vel_target_ms == Catch::Approx(0.0f));
    REQUIRE(d.vel_desired_ms <= 0.0f);
    REQUIRE(out.accel_target_mss == Catch::Approx(0.0f));
}

TEST_CASE("D init stopping point zeros desired motion", "[poscontrol][d][init]") {
    PosControlD d{};
    DOffsetState offsets{};
    AcPidBasic vel_pid = AcPidBasic::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = AcPid(AcPid::Gains{.p = 0.1f, .imax = 0.8f});
    DLimits limits = DLimits::defaults();
    const DInitOutput out =
        d.init_controller_stopping_point(offsets, vel_pid, accel_pid, d_init_inputs(), 1.0f, limits);
    REQUIRE(d.vel_desired_ms == Catch::Approx(0.0f));
    REQUIRE(d.accel_desired_mss == Catch::Approx(0.0f));
    REQUIRE(static_cast<float>(out.pos_target_m) ==
            static_cast<float>(d.pos_desired_m + offsets.current.pos_m));
}

TEST_CASE("d_relax_controller decays accel integrator", "[poscontrol][d][relax]") {
    PosControlD d{};
    DOffsetState offsets{};
    AcPidBasic vel_pid = AcPidBasic::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = AcPid(AcPid::Gains{.p = 0.1f, .imax = 0.8f});
    DInitInputs inp = d_init_inputs();
    (void)d.init_controller(offsets, vel_pid, accel_pid, inp);
    const float i_after_init = accel_pid.get_i();
    d_relax_controller(d, accel_pid, 0.02f, 0.5f, 0.4f, offsets, vel_pid, inp);
    const float target_i = -(0.5f - 0.4f);
    const float blend = 0.02f / (0.02f + kPoscontrolRelaxTc);
    REQUIRE(accel_pid.get_i() == Catch::Approx(i_after_init + (target_i - i_after_init) * blend).margin(1e-5f));
}

TEST_CASE("D input_accel applies overspeed jerk scale", "[poscontrol][d][input]") {
    PosControlD d{};
    d.vel_desired_ms = 5.0f;
    d.accel_desired_mss = 0.0f;
    DLimits limits = DLimits::defaults();
    limits.vel_max_down_ms = 2.5f;
    d.input_accel(1.0f, limits, 0.02f, 0.0f, 0.0f);
    REQUIRE(d.accel_desired_mss > 0.0f);
}

TEST_CASE("throttle_with_vibration_override integrator bump", "[poscontrol][d][throttle]") {
    AcPid accel = AcPid(AcPid::Gains{.p = 1.0f, .imax = 1.0f});
    accel.set_integrator(0.1f);
    const float out = throttle_with_vibration_override(accel, -0.5f, 2.0f, 0.3f, 0.02f, 0.4f, false);
    const float expected_i = 0.1f + 0.02f * 0.4f * (-0.5f) * 2.0f * kPoscontrolVibeCompIGain;
    REQUIRE(accel.get_i() == Catch::Approx(expected_i).margin(1e-5f));
    REQUIRE(out == Catch::Approx(kPoscontrolVibeCompPGain * 0.4f * 0.3f + expected_i).margin(1e-5f));
}