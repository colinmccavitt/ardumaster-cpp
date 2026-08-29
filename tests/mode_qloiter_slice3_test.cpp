#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/q_loiter/loiter_alt_qland_handle_guided.hpp>
#include <fwcpp/q_loiter/loiter_alt_qland_navigate.hpp>
#include <fwcpp/q_loiter/mode_qland_run.hpp>
#include <fwcpp/q_loiter/mode_qloiter_qland_options.hpp>
#include <fwcpp/q_loiter/mode_qloiter_systemid.hpp>
#include <fwcpp/q_loiter/mode_qloiter_update.hpp>
#include <fwcpp/q_modes/mode_qstabilize.hpp>

using Catch::Approx;
using fwcpp::math::Vector3f;
using fwcpp::q_loiter::GuidedAltFrame;
using fwcpp::q_loiter::LoiterAltQlandEnterInputs;
using fwcpp::q_loiter::LoiterAltQlandSwitchAction;
using fwcpp::q_loiter::LoiterAltQlandSwitchInputs;
using fwcpp::q_loiter::QLoiterQlandOptionalInputs;
using fwcpp::q_loiter::QLoiterSystemidEffects;
using fwcpp::q_loiter::QLoiterSystemidInputs;
using fwcpp::q_loiter::qland_run;
using fwcpp::q_loiter::qloiter_run_view_qland_final;
using fwcpp::q_modes::QStabilizeUpdateInputs;
using fwcpp::q_modes::qstabilize_update;

TEST_CASE("qloiter and qland update both delegate qstabilize", "[q_loiter][update]") {
    QStabilizeUpdateInputs in{};
    in.roll_control_in = 2250.0F;
    in.pitch_control_in = -2250.0F;
    in.ignore_fw_angle_limits_in_q_modes = true;

    const auto expected = qstabilize_update(in);
    const auto qloiter = fwcpp::q_loiter::qloiter_update(in);
    const auto qland = fwcpp::q_loiter::qland_update(in);

    REQUIRE(qloiter.delegate_qstabilize_update);
    REQUIRE(qland.delegate_qstabilize_update);
    REQUIRE(qloiter.qstabilize.nav.nav_roll_cd == Approx(expected.nav.nav_roll_cd));
    REQUIRE(qloiter.qstabilize.nav.nav_pitch_cd == Approx(expected.nav.nav_pitch_cd));
    REQUIRE(qland.qstabilize.nav.nav_roll_cd == Approx(expected.nav.nav_roll_cd));
    REQUIRE(qland.qstabilize.nav.nav_pitch_cd == Approx(expected.nav.nav_pitch_cd));
}

TEST_CASE("qloiter systemid attitude offset is compile-time optional", "[q_loiter][systemid]") {
    const Vector3f target{1.0F, 2.0F, 3.0F};
    QLoiterSystemidInputs in{};
    in.systemid_enabled = true;
    in.attitude_offset_deg = Vector3f{0.5F, -0.25F, 1.0F};
    QLoiterSystemidEffects effects{};
    const auto out = fwcpp::q_loiter::qloiter_apply_systemid_to_target(target, in, effects);
    REQUIRE(effects.vtol_update_called);
    REQUIRE(effects.apply_attitude_offset);
    REQUIRE(out.x == Approx(1.5F));
    REQUIRE(out.y == Approx(1.75F));
    REQUIRE(out.z == Approx(4.0F));

    in.systemid_enabled = false;
    const auto disabled = fwcpp::q_loiter::qloiter_apply_systemid_to_target(target, in, effects);
    REQUIRE_FALSE(effects.vtol_update_called);
    REQUIRE_FALSE(effects.apply_attitude_offset);
    REQUIRE(disabled.x == Approx(1.0F));
    REQUIRE(disabled.y == Approx(2.0F));
    REQUIRE(disabled.z == Approx(3.0F));
}

TEST_CASE("loiter alt qland navigate switch then ModeLoiter", "[q_loiter][loiter_alt][slice3]") {
    LoiterAltQlandSwitchInputs sw{};
    sw.height_above_valid = false;
    sw.reached_loiter_target = true;
    const auto nav = fwcpp::q_loiter::loiter_alt_qland_navigate(sw);
    REQUIRE(nav.request_qland_mode);
    REQUIRE(nav.delegate_mode_loiter_navigate);
    REQUIRE(nav.switch_action == LoiterAltQlandSwitchAction::kSwitchQland);

    sw.height_above_valid = true;
    sw.height_above_m = 8.0F;
    sw.reached_loiter_target = true;
    const auto hold = fwcpp::q_loiter::loiter_alt_qland_navigate(sw);
    REQUIRE_FALSE(hold.request_qland_mode);
    REQUIRE(hold.delegate_mode_loiter_navigate);
    REQUIRE(hold.switch_action == LoiterAltQlandSwitchAction::kNone);
}

TEST_CASE("loiter alt qland handle_guided alt frame then set_guided_WP",
          "[q_loiter][loiter_alt][slice3]") {
    LoiterAltQlandEnterInputs enter{};
    enter.terrain_enabled = true;
    enter.qrtl_alt_m = 22.0F;
    const auto terrain = fwcpp::q_loiter::loiter_alt_qland_handle_guided_request(enter);
    REQUIRE(terrain.set_guided_wp);
    REQUIRE(terrain.alt_m == Approx(22.0F));
    REQUIRE(terrain.alt_frame == GuidedAltFrame::kAboveTerrain);

    enter.terrain_enabled = false;
    const auto home = fwcpp::q_loiter::loiter_alt_qland_handle_guided_request(enter);
    REQUIRE(home.set_guided_wp);
    REQUIRE(home.alt_frame == GuidedAltFrame::kAboveHome);

    const auto no_terrain_lib =
        fwcpp::q_loiter::loiter_alt_qland_handle_guided(15.0F, true, false);
    REQUIRE(no_terrain_lib.set_guided_wp);
    REQUIRE(no_terrain_lib.alt_m == Approx(15.0F));
    REQUIRE(no_terrain_lib.alt_frame == GuidedAltFrame::kAboveHome);
}

TEST_CASE("qland ic engine cut only on land final when enabled", "[q_loiter][qland][options]") {
    QLoiterQlandOptionalInputs in{};
    REQUIRE_FALSE(fwcpp::q_loiter::qloiter_qland_optional_effects(in).cut_ic_engine);

    in.active_control_is_qland = true;
    in.land_final_transition = true;
    in.icengine_enabled = true;
    in.land_icengine_cut = 1;
    REQUIRE(fwcpp::q_loiter::qloiter_qland_optional_effects(in).cut_ic_engine);

    in.land_icengine_cut = 0;
    REQUIRE_FALSE(fwcpp::q_loiter::qloiter_qland_optional_effects(in).cut_ic_engine);
    in.land_icengine_cut = 1;
    in.icengine_enabled = false;
    REQUIRE_FALSE(fwcpp::q_loiter::qloiter_qland_optional_effects(in).cut_ic_engine);
    in.icengine_enabled = true;
    in.land_final_transition = false;
    REQUIRE_FALSE(fwcpp::q_loiter::qloiter_qland_optional_effects(in).cut_ic_engine);
    in.land_final_transition = true;
    in.active_control_is_qland = false;
    REQUIRE_FALSE(fwcpp::q_loiter::qloiter_qland_optional_effects(in).cut_ic_engine);
}

TEST_CASE("qland run land final wires optional ic engine cut", "[q_loiter][qland][run]") {
    auto view = qloiter_run_view_qland_final();
    REQUIRE_FALSE(qland_run(view).qloiter.actions.cut_ic_engine);

    view.icengine_enabled = true;
    view.land_icengine_cut = 1;
    REQUIRE(qland_run(view).qloiter.actions.cut_ic_engine);
}
