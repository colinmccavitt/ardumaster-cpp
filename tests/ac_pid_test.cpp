// Tests for fwcpp::pid::AcPid (CPP-016 slice 1).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/pid/ac_pid.hpp>

#include <cmath>

using fwcpp::pid::AcPid;

namespace {
AcPid make_pid(float p, float i, float d, float imax = 100.0f) {
    AcPid::Gains g;
    g.p = p;
    g.i = i;
    g.d = d;
    g.imax = imax;
    g.filt_t_hz = 0.0f; // disable target filter for predictable single-step tests
    g.filt_e_hz = 0.0f;
    g.filt_d_hz = 0.0f;
    return AcPid(g);
}
} // namespace

TEST_CASE("update_all returns 0 for NaN or infinite input", "[ac_pid]") {
    AcPid pid = make_pid(1.0f, 0.0f, 0.0f);
    REQUIRE(pid.update_all(std::nanf(""), 0.0f, 0.01f, 1000) == 0.0f);
    REQUIRE(pid.update_all(0.0f, std::numeric_limits<float>::infinity(), 0.01f, 1000) == 0.0f);
}

TEST_CASE("a pure-P controller outputs kp * error, ignoring filters on the first (reset) call", "[ac_pid]") {
    AcPid pid = make_pid(2.0f, 0.0f, 0.0f);
    float out = pid.update_all(10.0f, 4.0f, 0.01f, 1000); // error = target - measurement = 6
    REQUIRE(out == Catch::Approx(12.0f)); // 2 * 6
}

TEST_CASE("reset_filter forces the next update_all to treat its input as a fresh seed", "[ac_pid]") {
    AcPid pid = make_pid(1.0f, 0.0f, 0.0f);
    pid.update_all(10.0f, 0.0f, 0.01f, 1000);
    pid.update_all(10.0f, 0.0f, 0.01f, 1010);
    pid.reset_filter();
    float out = pid.update_all(100.0f, 0.0f, 0.01f, 1020);
    REQUIRE(out == Catch::Approx(100.0f)); // fresh seed: target snaps to 100, error = 100
}

TEST_CASE("a pure-I controller integrates error over time", "[ac_pid]") {
    AcPid pid = make_pid(0.0f, 10.0f, 0.0f);
    float out = 0.0f;
    std::uint32_t t = 1000;
    for (int i = 0; i < 10; ++i) {
        out = pid.update_all(1.0f, 0.0f, 0.1f, t); // constant error 1.0, dt 0.1
        t += 100;
    }
    // integrator += error * ki * dt each step = 1*10*0.1 = 1.0 per step, 10 steps = 10.0
    REQUIRE(out == Catch::Approx(10.0f).margin(0.01f));
}

TEST_CASE("integrator clamps to +-imax", "[ac_pid]") {
    AcPid pid = make_pid(0.0f, 100.0f, 0.0f, 5.0f); // imax = 5
    float out = 0.0f;
    std::uint32_t t = 1000;
    for (int i = 0; i < 50; ++i) {
        out = pid.update_all(1.0f, 0.0f, 0.1f, t);
        t += 100;
    }
    REQUIRE(out == Catch::Approx(5.0f).margin(0.001f)); // clamped, not runaway
    REQUIRE(pid.get_i() <= 5.0f);
}

TEST_CASE("with limit=true, the integrator can shrink but not grow when error and integrator agree in sign", "[ac_pid]") {
    AcPid pid = make_pid(0.0f, 10.0f, 0.0f, 100.0f);
    std::uint32_t t = 1000;
    // Build up a positive integrator first, unlimited.
    for (int i = 0; i < 5; ++i) {
        pid.update_all(1.0f, 0.0f, 0.1f, t, false);
        t += 100;
    }
    float i_before = pid.get_i();
    REQUIRE(i_before > 0.0f);

    // Now with limit=true and error STILL positive (same sign as integrator):
    // growth should be blocked.
    pid.update_all(1.0f, 0.0f, 0.1f, t, true);
    REQUIRE(pid.get_i() == Catch::Approx(i_before)); // did not grow further
}

TEST_CASE("reset_i zeroes the integrator immediately", "[ac_pid]") {
    AcPid pid = make_pid(0.0f, 10.0f, 0.0f);
    std::uint32_t t = 1000;
    for (int i = 0; i < 5; ++i) {
        pid.update_all(1.0f, 0.0f, 0.1f, t);
        t += 100;
    }
    REQUIRE(pid.get_i() != 0.0f);
    pid.reset_i();
    REQUIRE(pid.get_i() == 0.0f);
}

TEST_CASE("set_integrator clamps to imax", "[ac_pid]") {
    AcPid pid = make_pid(0.0f, 1.0f, 0.0f, 10.0f);
    pid.set_integrator(999.0f);
    REQUIRE(pid.get_i() == Catch::Approx(10.0f)); // clamped
}

TEST_CASE("i_term_set flag clears after being read via update_all", "[ac_pid]") {
    AcPid pid = make_pid(0.0f, 1.0f, 0.0f, 10.0f);
    pid.set_integrator(5.0f);
    pid.update_all(0.0f, 0.0f, 0.01f, 1000);
    REQUIRE(pid.get_pid_info().i_term_set);
    pid.update_all(0.0f, 0.0f, 0.01f, 1010);
    REQUIRE_FALSE(pid.get_pid_info().i_term_set); // cleared after the first read
}

TEST_CASE("relax_integrator moves the integrator toward a target with a time constant", "[ac_pid]") {
    AcPid pid = make_pid(0.0f, 1.0f, 0.0f, 100.0f);
    pid.set_integrator(0.0f);
    pid.relax_integrator(10.0f, 0.1f, 1.0f); // dt=0.1, tau=1.0 -> moves 1/11 of the way
    REQUIRE(pid.get_i() == Catch::Approx(10.0f * (0.1f / 1.1f)).margin(0.001f));
}

TEST_CASE("get_ff and get_ff_component/get_dff_component report feed-forward terms", "[ac_pid]") {
    AcPid::Gains g;
    g.ff = 2.0f;
    g.filt_t_hz = 0.0f;
    g.filt_e_hz = 0.0f;
    g.filt_d_hz = 0.0f;
    AcPid pid(g);
    pid.update_all(5.0f, 5.0f, 0.01f, 1000); // reset call: target snaps to 5
    REQUIRE(pid.get_ff_component() == Catch::Approx(10.0f)); // target(5) * ff(2)
}

TEST_CASE("PD sum limit (kPDMAX) scales P and D down together when exceeded", "[ac_pid]") {
    AcPid pid = make_pid(10.0f, 0.0f, 0.0f);
    pid.kPDMAX() = 5.0f; // enable PD sum limiting
    float out = pid.update_all(10.0f, 0.0f, 0.01f, 1000); // raw P would be 100
    REQUIRE(std::fabs(out) <= 5.0f + 0.01f);
    REQUIRE(pid.get_pid_info().pd_limit);
}
