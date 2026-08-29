// CCP-027 slice 8: PosControl class shell smoke tests.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/poscontrol/pos_control.hpp>
#include <fwcpp/pid/ac_p_2d.hpp>
#include <fwcpp/pid/ac_pid_2d.hpp>

using namespace fwcpp::poscontrol;
using fwcpp::pid::AcP2d;
using fwcpp::pid::AcPid2d;

TEST_CASE("PosControl shell dt and estimates", "[poscontrol][class]") {
    PosControlInjectedDeps deps{};
    deps.attitude_lean_angle_max_rad = 0.5f;
    PosControl pc(deps);

    pc.set_dt_s(0.02f);
    REQUIRE(pc.get_dt_s() == Catch::Approx(0.02f));

    AhrsPosControlEstimateInputs ahrs{};
    ahrs.pos_ned_valid = true;
    ahrs.pos_ned_m = {1.0, 2.0, -3.0};
    ahrs.vel_ned_valid = true;
    ahrs.vel_ned_ms = {0.1f, 0.2f, -0.3f};
    pc.update_estimates(ahrs);

    REQUIRE(static_cast<float>(pc.get_estimates().pos_m.x) == Catch::Approx(1.0f));
    REQUIRE(pc.get_estimates().vel_ms.z == Catch::Approx(-0.3f));
    REQUIRE(pc.get_lean_angle_max_rad() == Catch::Approx(0.5f));
}

TEST_CASE("PosControl shell NE update matches free function path", "[poscontrol][class]") {
    PosControlInjectedDeps deps{};
    deps.attitude_lean_angle_max_rad = 0.8f;
    deps.ticks = 10;
    PosControl pc(deps);
    pc.set_dt_s(0.02f);
    pc.ne_set_max_speed_accel_m(10.0f, 2.0f);

    pc.ne().pos_desired_m = {1.0, 0.0};
    AcP2d pos_p = AcP2d::with_kp(kNePosP);
    AcPid2d vel_pid = AcPid2d::with_gains(1.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    NeDisturbance disturb{};

    NeUpdateInputs inp{};
    inp.dt = 0.02f;
    inp.vel_max_ne_ms = 10.0f;
    inp.lean_angle_max_rad = 0.8f;
    inp.att_yaw_target_rad = 0.3f;
    const NeUpdateOutput direct = pc.ne().update_controller(pos_p, vel_pid, inp, disturb);

    pc.ne().pos_desired_m = {1.0, 0.0};
    deps.att_yaw_target_rad = 0.3f;
    pc.set_injected_deps(deps);
    const NeUpdateOutput wrapped = pc.ne_update_controller();

    REQUIRE(wrapped.vel_target_ms.x == Catch::Approx(direct.vel_target_ms.x).margin(1e-5f));
    REQUIRE(pc.ne_is_active());
}