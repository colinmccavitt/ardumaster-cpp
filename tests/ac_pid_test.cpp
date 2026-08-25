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

TEST_CASE("update_error reuses update_all with target forced to zero", "[ac_pid][update_error]") {
    // error=6 should produce the same P-term as update_all(target=6,
    // measurement=0) - update_error's whole contract is "error input only,
    // target assumed zero".
    AcPid pid_a = make_pid(2.0f, 0.0f, 0.0f);
    AcPid pid_b = make_pid(2.0f, 0.0f, 0.0f);
    float out_a = pid_a.update_all(6.0f, 0.0f, 0.01f, 1000);
    float out_b = pid_b.update_error(6.0f, 0.01f, 1000);
    REQUIRE(out_b == Catch::Approx(out_a));
    REQUIRE(pid_b.get_pid_info().target == Catch::Approx(0.0f));
    REQUIRE(pid_b.get_pid_info().actual == Catch::Approx(0.0f));
}

TEST_CASE("update_error returns 0 for NaN or infinite input", "[ac_pid][update_error]") {
    AcPid pid = make_pid(1.0f, 0.0f, 0.0f);
    REQUIRE(pid.update_error(std::nanf(""), 0.01f, 1000) == 0.0f);
    REQUIRE(pid.update_error(std::numeric_limits<float>::infinity(), 0.01f, 1000) == 0.0f);
}

TEST_CASE("get_filt_t/e/d_alpha match calc_lowpass_alpha_dt at the configured frequencies", "[ac_pid][alpha]") {
    AcPid::Gains g;
    g.p = 1.0f;
    g.filt_t_hz = 10.0f;
    g.filt_e_hz = 20.0f;
    g.filt_d_hz = 30.0f;
    AcPid pid(g);
    REQUIRE(pid.get_filt_t_alpha(0.01f) == Catch::Approx(fwcpp::math::calc_lowpass_alpha_dt(0.01f, 10.0f)));
    REQUIRE(pid.get_filt_e_alpha(0.01f) == Catch::Approx(fwcpp::math::calc_lowpass_alpha_dt(0.01f, 20.0f)));
    REQUIRE(pid.get_filt_d_alpha(0.01f) == Catch::Approx(fwcpp::math::calc_lowpass_alpha_dt(0.01f, 30.0f)));
}

TEST_CASE("set_ accessors update the same storage the getters read from", "[ac_pid][setters]") {
    AcPid pid = make_pid(1.0f, 1.0f, 1.0f);
    pid.set_kP(3.0f);
    pid.set_kI(4.0f);
    pid.set_kD(5.0f);
    pid.set_ff(6.0f);
    pid.set_kDff(7.0f);
    REQUIRE(pid.kP() == Catch::Approx(3.0f));
    REQUIRE(pid.kI() == Catch::Approx(4.0f));
    REQUIRE(pid.kD() == Catch::Approx(5.0f));
    REQUIRE(pid.ff() == Catch::Approx(6.0f));
    REQUIRE(pid.kDff() == Catch::Approx(7.0f));
}

TEST_CASE("set_imax/set_pdmax/set_filt_*_hz/set_slew_limit take the absolute value, matching upstream's fabsf", "[ac_pid][setters]") {
    AcPid pid = make_pid(1.0f, 1.0f, 1.0f);
    pid.set_imax(-10.0f);
    pid.set_pdmax(-5.0f);
    pid.set_filt_T_hz(-1.0f);
    pid.set_filt_E_hz(-2.0f);
    pid.set_filt_D_hz(-3.0f);
    pid.set_slew_limit(-4.0f);
    REQUIRE(pid.imax() == Catch::Approx(10.0f));
    REQUIRE(pid.pdmax() == Catch::Approx(5.0f));
    REQUIRE(pid.filt_T_hz() == Catch::Approx(1.0f));
    REQUIRE(pid.filt_E_hz() == Catch::Approx(2.0f));
    REQUIRE(pid.filt_D_hz() == Catch::Approx(3.0f));
    REQUIRE(pid.slew_limit() == Catch::Approx(4.0f));
}

TEST_CASE("set_slew_limit actually affects the embedded slew limiter (reference-aliased, not copied)", "[ac_pid][setters]") {
    // slew_limit() and set_slew_limit() mutate the very float the embedded
    // SlewLimiter holds a const-reference to (see slew_limiter.hpp) -
    // changing it after construction must change limiter behavior, not
    // just the accessor's own return value.
    AcPid::Gains g;
    g.p = 10.0f;
    g.filt_t_hz = 0.0f;
    g.filt_e_hz = 0.0f;
    g.filt_d_hz = 0.0f;
    g.srmax = 1.0f; // start with slew limiting enabled
    AcPid pid(g);
    REQUIRE(pid.slew_limit() == Catch::Approx(1.0f));

    pid.set_slew_limit(0.0f); // disabled: modifier always returns 1 (no scaling)
    REQUIRE(pid.slew_limit() == Catch::Approx(0.0f));
    float out_unlimited = pid.update_all(10.0f, 0.0f, 0.01f, 1000); // raw P = 100
    REQUIRE(out_unlimited == Catch::Approx(100.0f));
}

TEST_CASE("set_notch_sample_rate with both filter indices 0 is a no-op", "[ac_pid][notch]") {
    AcPid pid = make_pid(1.0f, 0.0f, 0.0f);
    fwcpp::filter::FilterRegistry registry;
    pid.set_notch_sample_rate(400.0f, registry);
    REQUIRE_FALSE(pid.target_notch_active());
    REQUIRE_FALSE(pid.error_notch_active());
}

TEST_CASE("set_notch_sample_rate activates the target notch when its registry slot is configured", "[ac_pid][notch]") {
    AcPid pid = make_pid(1.0f, 0.0f, 0.0f);
    fwcpp::filter::FilterRegistry registry;
    registry.get_notch_filter(1)->center_freq_hz_.set(80.0f); // quality/attenuation stay at their nonzero defaults

    pid.set_notch_t_filter(1);
    pid.set_notch_sample_rate(400.0f, registry);
    REQUIRE(pid.target_notch_active());
    REQUIRE_FALSE(pid.error_notch_active());
}

TEST_CASE("set_notch_sample_rate leaves the filter inactive when its registry slot is unconfigured (center_freq_hz still 0)", "[ac_pid][notch]") {
    AcPid pid = make_pid(1.0f, 0.0f, 0.0f);
    fwcpp::filter::FilterRegistry registry; // slot 1 left at defaults: center_freq_hz == 0

    pid.set_notch_t_filter(1);
    pid.set_notch_sample_rate(400.0f, registry);
    REQUIRE_FALSE(pid.target_notch_active()); // setup_notch_filter returned false -> disabled
}

TEST_CASE("set_notch_sample_rate with an out-of-range index leaves previously-established state untouched", "[ac_pid][notch]") {
    // Matches upstream exactly: AP::filters().get_filter() returning
    // nullptr for an out-of-range index does NOT touch the existing
    // _target_notch/_notch_T_filter state at all - only an in-range slot
    // whose OWN setup fails causes a disable.
    AcPid pid = make_pid(1.0f, 0.0f, 0.0f);
    fwcpp::filter::FilterRegistry registry;
    registry.get_notch_filter(1)->center_freq_hz_.set(80.0f);

    pid.set_notch_t_filter(1);
    pid.set_notch_sample_rate(400.0f, registry);
    REQUIRE(pid.target_notch_active());

    // Now call again with an index the registry doesn't have.
    pid.set_notch_t_filter(200);
    pid.set_notch_sample_rate(400.0f, registry);
    REQUIRE(pid.target_notch_active()); // still true - untouched by the out-of-range lookup
}

TEST_CASE("an active target notch filter measurably changes update_all's output vs an unfiltered controller", "[ac_pid][notch]") {
    fwcpp::filter::FilterRegistry registry;
    registry.get_notch_filter(1)->center_freq_hz_.set(50.0f);

    AcPid filtered = make_pid(1.0f, 0.0f, 0.0f);
    filtered.set_notch_t_filter(1);
    filtered.set_notch_sample_rate(400.0f, registry);

    AcPid unfiltered = make_pid(1.0f, 0.0f, 0.0f);

    // Feed a step target through both on the very first (reset) call -
    // the notch filter's own reset() forces a passthrough seed, so the
    // two should still agree on this first sample...
    float out_filtered_first = filtered.update_all(10.0f, 0.0f, 0.0025f, 1000);
    float out_unfiltered_first = unfiltered.update_all(10.0f, 0.0f, 0.0025f, 1000);
    REQUIRE(out_filtered_first == Catch::Approx(out_unfiltered_first));

    // ...but a subsequent oscillating target exercises the notch's real
    // filtering behavior, which a plain low-pass-only controller doesn't
    // have - the two should now diverge.
    float out_filtered_second = filtered.update_all(20.0f, 0.0f, 0.0025f, 1004);
    float out_unfiltered_second = unfiltered.update_all(20.0f, 0.0f, 0.0025f, 1004);
    REQUIRE(out_filtered_second != Catch::Approx(out_unfiltered_second).margin(0.0001f));
}
