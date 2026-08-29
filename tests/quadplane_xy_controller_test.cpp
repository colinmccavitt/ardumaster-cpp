#include <algorithm>
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_xy_controller.hpp>

using Catch::Approx;
using fwcpp::quadplane::ActiveFwdThr;
using fwcpp::quadplane::AssignTiltInputs;
using fwcpp::quadplane::FwdThrUse;
using fwcpp::quadplane::FwdTiltState;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::VfwdMethodInputs;
using fwcpp::quadplane::XyControllerInputs;
using fwcpp::quadplane::accel_mss_to_angle_deg;
using fwcpp::quadplane::assign_tilt_to_fwd_thr;
using fwcpp::quadplane::get_vfwd_method;
using fwcpp::quadplane::kDefaultSpeedNeMs;
using fwcpp::quadplane::kGravityMss;
using fwcpp::quadplane::kLeanAngleMaxCdCap;
using fwcpp::quadplane::kQBckPitchLimDefault;
using fwcpp::quadplane::kQFwdPitchLimDefault;
using fwcpp::quadplane::kQFwdThrGainDefault;
using fwcpp::quadplane::kWpAccelMssDefault;
using fwcpp::quadplane::run_xy_controller;
using fwcpp::quadplane::set_climb_rate_ms;

static VfwdMethodInputs vfwd_new() {
    VfwdMethodInputs in{};
    in.allow_vfwd = true;
    in.q_fwd_thr_gain = kQFwdThrGainDefault;
    in.q_fwd_thr_use = FwdThrUse::kAll;
    return in;
}

static AssignTiltInputs tilt_new(float nav_pitch_cd) {
    AssignTiltInputs in{};
    in.vfwd = vfwd_new();
    in.nav_pitch_cd = nav_pitch_cd;
    in.q_fwd_thr_gain = kQFwdThrGainDefault;
    in.q_fwd_pitch_lim = kQFwdPitchLimDefault;
    in.q_bck_pitch_lim = kQBckPitchLimDefault;
    in.angle_max_cd = 4500.0f;
    in.in_vtol_land_approach = true;
    in.have_airspeed = false;
    in.tiltrotor_enabled = true;
    return in;
}

TEST_CASE("set_climb_rate_ms inverts climb to D velocity", "[quadplane][xy][climb]") {
    auto up = set_climb_rate_ms(2.5f);
    REQUIRE(up.input_vel_accel_D_m);
    REQUIRE(up.vel_d_m == Approx(-2.5f));
    REQUIRE(up.accel_d_mss == Approx(0.0f));
    REQUIRE_FALSE(up.inhibit_descent_limit);

    auto down = set_climb_rate_ms(-1.25f);
    REQUIRE(down.input_vel_accel_D_m);
    REQUIRE(down.vel_d_m == Approx(1.25f));

    REQUIRE(set_climb_rate_ms(0.0f).vel_d_m == Approx(0.0f));

    QuadPlane qp{1};
    REQUIRE(qp.set_climb_rate_ms(3.0f).vel_d_m == Approx(-3.0f));
}

TEST_CASE("run_xy_controller accel override, 0.95 external limit, inactive init",
          "[quadplane][xy]") {
    XyControllerInputs in{};
    in.wp_accel_mss = kWpAccelMssDefault;
    in.default_speed_ne_ms = kDefaultSpeedNeMs;
    in.lean_angle_max_cd = 2000.0f;
    in.ne_is_active = false;

    auto tick = run_xy_controller(in);
    REQUIRE(tick.accel_mss == Approx(kWpAccelMssDefault));
    REQUIRE(tick.speed_ms == Approx(kDefaultSpeedNeMs));
    REQUIRE(tick.ne_set_max_speed_accel);
    REQUIRE(tick.ne_set_correction_speed_accel);
    REQUIRE(tick.ne_init_controller);
    REQUIRE(tick.ne_update_controller);
    REQUIRE_FALSE(tick.ne_set_externally_limited);
    REQUIRE(tick.set_lean_angle_max_cd);
    REQUIRE(tick.lean_angle_max_cd == Approx(2000.0f));

    in.accel_limit_mss = 4.0f;
    tick = run_xy_controller(in);
    REQUIRE(tick.accel_mss == Approx(4.0f));
    const float lean_from_limit = accel_mss_to_angle_deg(4.0f) * 100.0f;
    REQUIRE(tick.lean_angle_max_cd ==
            Approx(std::min(kLeanAngleMaxCdCap, std::max(lean_from_limit, 2000.0f))));

    in.wp_accel_mss = 5.0f;
    in.accel_limit_mss = 3.0f;
    tick = run_xy_controller(in);
    REQUIRE(tick.accel_mss == Approx(5.0f));
    REQUIRE(tick.lean_angle_max_cd ==
            Approx(std::min(kLeanAngleMaxCdCap, std::max(accel_mss_to_angle_deg(3.0f) * 100.0f, 2000.0f))));

    in.accel_limit_mss = kGravityMss;
    in.lean_angle_max_cd = 1000.0f;
    tick = run_xy_controller(in);
    REQUIRE(accel_mss_to_angle_deg(kGravityMss) == Approx(45.0f).margin(1e-4f));
    REQUIRE(tick.lean_angle_max_cd == Approx(kLeanAngleMaxCdCap));

    in.q_fwd_throttle = 0.95f;
    REQUIRE_FALSE(run_xy_controller(in).ne_set_externally_limited);
    in.q_fwd_throttle = 0.951f;
    REQUIRE(run_xy_controller(in).ne_set_externally_limited);

    in.ne_is_active = true;
    tick = run_xy_controller(in);
    REQUIRE_FALSE(tick.ne_init_controller);
    REQUIRE(tick.ne_update_controller);

    QuadPlane qp{1};
    qp.fwd_tilt_mut().q_fwd_throttle = 0.99f;
    XyControllerInputs wired{};
    wired.ne_is_active = false;
    const auto qp_tick = qp.run_xy_controller(wired);
    REQUIRE(qp_tick.ne_init_controller);
    REQUIRE(qp_tick.ne_set_externally_limited);
    REQUIRE(qp_tick.accel_mss == Approx(kWpAccelMssDefault));
}

TEST_CASE("get_vfwd_method NONE NEW OLD branches", "[quadplane][xy][vfwd]") {
    VfwdMethodInputs in = vfwd_new();
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNew);

    in.allow_vfwd = false;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNone);
    in.allow_vfwd = true;

    in.ice_blocks_fwd = true;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNone);
    in.ice_blocks_fwd = false;

    in.in_qautotune = true;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNone);
    in.in_qautotune = false;

    in.q_fwd_thr_use = FwdThrUse::kOff;
    in.vfwd_enable_active = true;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNew);

    in.vfwd_enable_active = false;
    in.q_fwd_thr_use = FwdThrUse::kAll;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNew);

    in.q_fwd_thr_use = FwdThrUse::kPosctrl;
    in.ne_is_active = true;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNew);

    in.ne_is_active = false;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNone);

    in.q_fwd_thr_gain = 0.0f;
    in.vfwd_enable_active = true;
    in.q_fwd_thr_use = FwdThrUse::kAll;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNone);

    in.vfwd_enable_active = false;
    in.q_fwd_thr_use = FwdThrUse::kOff;
    in.vel_forward_gain = 0.5f;
    in.ne_is_active = true;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kOld);

    in.ne_is_active = false;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNone);

    in.vel_forward_gain = 0.0f;
    in.ne_is_active = true;
    REQUIRE(get_vfwd_method(in) == ActiveFwdThr::kNone);

    QuadPlane qp{1};
    qp.set_q_fwd_thr_use(FwdThrUse::kAll);
    VfwdMethodInputs wired{};
    REQUIRE(qp.get_vfwd_method(wired) == ActiveFwdThr::kNew);
    wired.ice_blocks_fwd = true;
    REQUIRE(qp.get_vfwd_method(wired) == ActiveFwdThr::kNone);
}

TEST_CASE("assign_tilt_to_fwd_thr early zero when not NEW", "[quadplane][xy][tilt]") {
    FwdTiltState state{};
    state.q_fwd_throttle = 0.4f;
    state.q_fwd_pitch_lim_cd = 900.0f;
    AssignTiltInputs in{};
    in.q_fwd_pitch_lim = 3.5f;
    in.nav_pitch_cd = -2000.0f;
    const auto tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE(tick.early_return);
    REQUIRE(state.q_fwd_throttle == Approx(0.0f));
    REQUIRE(state.q_fwd_pitch_lim_cd == Approx(350.0f));
    REQUIRE(tick.nav_pitch_cd == Approx(-2000.0f));

    QuadPlane qp{1};
    qp.fwd_tilt_mut().q_fwd_throttle = 0.2f;
    const auto qp_tick = qp.assign_tilt_to_fwd_thr({});
    REQUIRE(qp_tick.early_return);
    REQUIRE(qp.fwd_tilt().q_fwd_throttle == Approx(0.0f));
    REQUIRE(qp.fwd_tilt().q_fwd_pitch_lim_cd == Approx(100.0f * kQFwdPitchLimDefault));
}

TEST_CASE("assign_tilt_to_fwd_thr tan throttle from pitch", "[quadplane][xy][tilt]") {
    FwdTiltState state{};
    auto in = tilt_new(-1000.0f);
    auto tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE_FALSE(tick.early_return);
    const float expected = std::min(kQFwdThrGainDefault * std::tan(fwcpp::math::radians(10.0f)), 1.0f);
    REQUIRE(state.q_fwd_throttle == Approx(expected));
    REQUIRE(tick.fwd_thr_scaler == Approx(1.0f));

    state = {};
    in.nav_pitch_cd = -4500.0f;
    tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE(state.q_fwd_throttle == Approx(1.0f));

    state = {};
    in.nav_pitch_cd = 800.0f;
    tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE(state.q_fwd_throttle == Approx(0.0f));
}

TEST_CASE("assign_tilt_to_fwd_thr ground scaler vs land approach", "[quadplane][xy][tilt]") {
    FwdTiltState state{};
    auto in = tilt_new(-1000.0f);
    in.in_vtol_land_approach = false;
    in.vel_forward_alt_cutoff_m = 2.0f;
    const float unscaled = std::min(kQFwdThrGainDefault * std::tan(fwcpp::math::radians(10.0f)), 1.0f);

    in.height_above_ground_m = 2.0f;
    auto tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE(tick.fwd_thr_scaler == Approx(0.0f));
    REQUIRE(state.q_fwd_throttle == Approx(0.0f));
    REQUIRE(tick.nav_pitch_lower_limit_cd == Approx(-4500.0f));

    state = {};
    in.height_above_ground_m = 4.0f;
    tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE(tick.fwd_thr_scaler == Approx(1.0f));
    REQUIRE(state.q_fwd_throttle == Approx(unscaled));

    state = {};
    in.height_above_ground_m = 3.0f;
    tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE(tick.fwd_thr_scaler == Approx(0.5f));
    REQUIRE(state.q_fwd_throttle == Approx(unscaled * 0.5f));

    state = {};
    in.in_vtol_land_approach = true;
    in.height_above_ground_m = 0.0f;
    tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE(tick.fwd_thr_scaler == Approx(1.0f));
    REQUIRE(state.q_fwd_throttle == Approx(unscaled));
}

TEST_CASE("assign_tilt_to_fwd_thr airspeed braking first-order filter", "[quadplane][xy][tilt]") {
    FwdTiltState state{};
    auto in = tilt_new(4000.0f);
    in.have_airspeed = true;
    in.aspeed = 20.0f;
    in.airspeed_min = 10.0f;
    in.now_ms = 1000;
    in.q_bck_pitch_lim = 10.0f;

    auto tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE(state.q_pitch_limit_update_ms == 1000);
    const float scaler = (10.0f / 20.0f) * (10.0f / 20.0f);
    REQUIRE(tick.nav_pitch_upper_limit_cd == Approx(1000.0f * scaler));
    const float coef = 1000.0f / (1000.0f + 0.5f);
    REQUIRE(state.q_bck_pitch_lim_cd == Approx(coef * 250.0f));
    REQUIRE(tick.nav_pitch_cd ==
            Approx(std::min(4000.0f, static_cast<float>(static_cast<std::int32_t>(state.q_bck_pitch_lim_cd)))));

    in.now_ms = 1000;
    const float held = state.q_bck_pitch_lim_cd;
    tick = assign_tilt_to_fwd_thr(state, in);
    REQUIRE(state.q_bck_pitch_lim_cd == Approx(held));

    FwdTiltState no_as{};
    in.have_airspeed = false;
    in.nav_pitch_cd = 4000.0f;
    tick = assign_tilt_to_fwd_thr(no_as, in);
    REQUIRE(tick.nav_pitch_cd == Approx(4000.0f));
    REQUIRE(no_as.q_pitch_limit_update_ms == 0);

    FwdTiltState no_lim{};
    in.have_airspeed = true;
    in.q_bck_pitch_lim = 0.0f;
    tick = assign_tilt_to_fwd_thr(no_lim, in);
    REQUIRE(tick.nav_pitch_cd == Approx(4000.0f));
}

TEST_CASE("assign_tilt_to_fwd_thr tiltrotor skips fwd-limit slew", "[quadplane][xy][tilt]") {
    FwdTiltState tilt{};
    tilt.q_fwd_pitch_lim_cd = 300.0f;
    auto in = tilt_new(-5000.0f);
    in.tiltrotor_enabled = true;
    in.ne_is_active = true;
    in.fwd_pitch_is_limited = true;
    in.g_dt = 1.0f;
    in.angle_max_cd = 4500.0f;
    assign_tilt_to_fwd_thr(tilt, in);
    REQUIRE(tilt.q_fwd_pitch_lim_cd == Approx(300.0f));

    FwdTiltState slew{};
    slew.q_fwd_pitch_lim_cd = 300.0f;
    in.tiltrotor_enabled = false;
    assign_tilt_to_fwd_thr(slew, in);
    const float range = 4500.0f - 300.0f;
    const float delta_max = 0.1f * range * 1.0f;
    REQUIRE(slew.q_fwd_pitch_lim_cd == Approx(300.0f + delta_max));

    FwdTiltState clamped{};
    clamped.q_fwd_pitch_lim_cd = 300.0f;
    in.nav_pitch_cd = -400.0f;
    assign_tilt_to_fwd_thr(clamped, in);
    REQUIRE(clamped.q_fwd_pitch_lim_cd == Approx(400.0f));

    FwdTiltState no_range{};
    no_range.q_fwd_pitch_lim_cd = 100.0f;
    in.angle_max_cd = 200.0f;
    in.q_fwd_pitch_lim = 3.0f;
    assign_tilt_to_fwd_thr(no_range, in);
    REQUIRE(no_range.q_fwd_pitch_lim_cd == Approx(200.0f));
}
