// CCP-027 slice 3: D_update_controller PID path (Rust pos_control_ne parity cases).

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

static DUpdateInputs default_d_inputs() {
    DUpdateInputs inp{};
    inp.dt = 0.02f;
    inp.vel_max_down_ms = 2.5f;
    return inp;
}

static AcPid accel_p(float p) {
    return AcPid(AcPid::Gains{.p = p, .imax = 1.0f});
}

TEST_CASE("D update_controller P then velocity PID then accel PID", "[poscontrol][d]") {
    PosControlD d{};
    d.pos_desired_m = 1.0;
    AcP1d pos_p = AcP1d::with_kp(1.0f);
    AcPidBasic vel_pid = AcPidBasic::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = accel_p(1.0f);
    const DUpdateOutput out = d.update_controller(pos_p, vel_pid, accel_pid, default_d_inputs());

    REQUIRE(out.vel_target_ms == Catch::Approx(1.0f).margin(1e-5f));
    REQUIRE(out.accel_target_mss == Catch::Approx(1.0f).margin(1e-5f));
    REQUIRE(out.thrust_d_norm == Catch::Approx(1.0f).margin(1e-5f));
    REQUIRE(out.throttle_out == Catch::Approx(-1.0f).margin(1e-5f));
}

TEST_CASE("D update_controller AHRS scale on both outer loops", "[poscontrol][d]") {
    PosControlD d{};
    d.pos_desired_m = 1.0;
    AcP1d pos_p = AcP1d::with_kp(1.0f);
    AcPidBasic vel_pid = AcPidBasic::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = accel_p(1.0f);
    DUpdateInputs inp = default_d_inputs();
    inp.ahrs_control_scale_z = 0.5f;
    const DUpdateOutput out = d.update_controller(pos_p, vel_pid, accel_pid, inp);

    REQUIRE(out.vel_target_ms == Catch::Approx(0.5f).margin(1e-5f));
    REQUIRE(out.accel_target_mss == Catch::Approx(0.25f).margin(1e-5f));
    REQUIRE(out.thrust_d_norm == Catch::Approx(0.25f).margin(1e-5f));
}

TEST_CASE("D offsets and terrain feedforward", "[poscontrol][d]") {
    PosControlD d{};
    d.vel_desired_ms = 0.5f;
    d.accel_desired_mss = 0.25f;
    AcP1d pos_p = AcP1d::with_kp(0.0f);
    AcPidBasic vel_pid = AcPidBasic::with_gains(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = accel_p(0.0f);
    DUpdateInputs inp = default_d_inputs();
    inp.offsets = {3.0, 0.1f, 0.05f};
    inp.terrain = {1.0, 0.2f, -0.05f};
    const DUpdateOutput out = d.update_controller(pos_p, vel_pid, accel_pid, inp);

    REQUIRE(static_cast<float>(out.pos_target_m) == Catch::Approx(4.0f).margin(1e-5f));
    REQUIRE(out.vel_target_ms == Catch::Approx(0.8f).margin(1e-5f));
    REQUIRE(out.accel_target_mss == Catch::Approx(0.25f).margin(1e-5f));
}

TEST_CASE("D position clamp rewrites desired", "[poscontrol][d]") {
    PosControlD d{};
    d.pos_desired_m = 100.0;
    AcP1d pos_p = AcP1d::with_kp(1.0f);
    pos_p.set_limits(-2.0f, 2.0f, 0.0f, 0.0f);
    AcPidBasic vel_pid = AcPidBasic::with_gains(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = accel_p(0.0f);
    DUpdateInputs inp = default_d_inputs();
    inp.offsets.pos_m = 1.0;
    inp.terrain.pos_m = 0.5;
    const DUpdateOutput out = d.update_controller(pos_p, vel_pid, accel_pid, inp);

    REQUIRE(std::fabs(static_cast<float>(out.pos_target_m)) < 10.0f);
    REQUIRE(static_cast<float>(d.pos_desired_m) ==
            Catch::Approx(static_cast<float>(out.pos_target_m) - 1.5f).margin(1e-5f));
}

TEST_CASE("D hover throttle subtracted and sign flipped", "[poscontrol][d]") {
    PosControlD d{};
    AcP1d pos_p = AcP1d::with_kp(0.0f);
    AcPidBasic vel_pid = AcPidBasic::with_gains(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = accel_p(0.0f);
    DUpdateInputs inp = default_d_inputs();
    inp.throttle_hover = 0.4f;
    const DUpdateOutput out = d.update_controller(pos_p, vel_pid, accel_pid, inp);

    REQUIRE(out.thrust_d_norm == Catch::Approx(-0.4f).margin(1e-5f));
    REQUIRE(out.throttle_out == Catch::Approx(0.4f).margin(1e-5f));
}

TEST_CASE("D hover raises accel PID imax", "[poscontrol][d]") {
    PosControlD d{};
    AcP1d pos_p = AcP1d::with_kp(0.0f);
    AcPidBasic vel_pid = AcPidBasic::with_gains(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = accel_p(0.0f);
    accel_pid.set_imax(0.1f);
    DUpdateInputs inp = default_d_inputs();
    inp.throttle_hover = 0.35f;
    (void)d.update_controller(pos_p, vel_pid, accel_pid, inp);
    REQUIRE(accel_pid.imax() == Catch::Approx(0.35f).margin(1e-6f));
}

TEST_CASE("D throttle limits set vertical limit vector", "[poscontrol][d]") {
    PosControlD d{};
    AcP1d pos_p = AcP1d::with_kp(0.0f);
    AcPidBasic vel_pid = AcPidBasic::with_gains(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = accel_p(0.0f);
    DUpdateInputs inp = default_d_inputs();

    inp.throttle_upper = true;
    DUpdateOutput out = d.update_controller(pos_p, vel_pid, accel_pid, inp);
    REQUIRE(out.limit == Catch::Approx(-1.0f));
    REQUIRE(d.limit == Catch::Approx(-1.0f));

    inp.throttle_upper = false;
    inp.throttle_lower = true;
    out = d.update_controller(pos_p, vel_pid, accel_pid, inp);
    REQUIRE(out.limit == Catch::Approx(1.0f));

    inp.throttle_lower = false;
    out = d.update_controller(pos_p, vel_pid, accel_pid, inp);
    REQUIRE(out.limit == Catch::Approx(0.0f));
}

TEST_CASE("D vibe compensation ignores acceleration measurement", "[poscontrol][d]") {
    PosControlD d{};
    d.pos_desired_m = 1.0;
    AcP1d pos_p = AcP1d::with_kp(1.0f);
    AcPidBasic vel_pid = AcPidBasic::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = accel_p(1.0f);
    DUpdateInputs inp = default_d_inputs();
    inp.vibe_comp_enabled = true;
    inp.throttle_hover = 0.4f;
    inp.estimated_accel_d_mss = 50.0f;
    const DUpdateOutput out = d.update_controller(pos_p, vel_pid, accel_pid, inp);

    const float vibe_i = 0.02f * 0.4f * 1.0f * 1.0f * 0.125f;
    const float expected_thrust = 0.250f * 0.4f * 1.0f + vibe_i - 0.4f;
    REQUIRE(out.thrust_d_norm == Catch::Approx(expected_thrust).margin(1e-5f));
}

TEST_CASE("D health ratio walks and clamps", "[poscontrol][d]") {
    PosControlD d{};
    REQUIRE(d.vel_d_control_ratio == Catch::Approx(2.0f));
    d.pos_desired_m = 10.0;
    AcP1d pos_p = AcP1d::with_kp(1.0f);
    AcPidBasic vel_pid = AcPidBasic::with_gains(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    AcPid accel_pid = accel_p(0.0f);
    DUpdateInputs inp = default_d_inputs();
    inp.vel_max_down_ms = 1.0f;
    DUpdateOutput out = d.update_controller(pos_p, vel_pid, accel_pid, inp);
    REQUIRE(out.vel_d_control_ratio == Catch::Approx(2.0f - 0.019f).margin(1e-5f));

    d.vel_d_control_ratio = 0.0f;
    d.pos_desired_m = -10.0;
    pos_p = AcP1d::with_kp(1.0f);
    vel_pid = AcPidBasic::with_gains(0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    out = d.update_controller(pos_p, vel_pid, accel_pid, inp);
    REQUIRE(out.vel_d_control_ratio == Catch::Approx(0.021f).margin(1e-5f));
}
