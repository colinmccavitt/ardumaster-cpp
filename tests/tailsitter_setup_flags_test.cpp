#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/quadplane/quadplane_air_mode.hpp>
#include <fwcpp/quadplane/quadplane_frame.hpp>
#include <fwcpp/quadplane/quadplane_options.hpp>
#include <fwcpp/tailsitter/tailsitter_defaults.hpp>
#include <fwcpp/tailsitter/tailsitter_setup.hpp>
#include <fwcpp/tailsitter/tailsitter_setup_flags.hpp>
#include <fwcpp/vtol_assist/vtol_assist.hpp>

using fwcpp::quadplane::AirMode;
using fwcpp::quadplane::MotorFrameClass;
using fwcpp::quadplane::QOption;
using fwcpp::quadplane::option_is_set;
using fwcpp::tailsitter::SrvAssigned;
using fwcpp::tailsitter::TailsitterSetupFlagInputs;
using fwcpp::tailsitter::TailsitterSetupInputs;
using fwcpp::tailsitter::apply_enable2_effects;
using fwcpp::tailsitter::kMotorFrameTailsitter;
using fwcpp::tailsitter::kTransitionAngleFwDefault;
using fwcpp::tailsitter::kTransitionRateFwDefault;
using fwcpp::tailsitter::resolve_enable2;
using fwcpp::tailsitter::resolve_setup;
using fwcpp::tailsitter::resolve_setup_flags;
using fwcpp::tailsitter::resolve_setup_with_flags;
using fwcpp::tailsitter::resolve_surface_flags;
using fwcpp::tailsitter::resolve_transition_rate_fw;
using fwcpp::tailsitter::setup_is_vectored;
using fwcpp::vtol_assist::AssistState;

TEST_CASE("tailsitter kMotorFrameTailsitter matches MOTOR_FRAME_TAILSITTER", "[tailsitter][setup]") {
    REQUIRE(kMotorFrameTailsitter == 10);
    REQUIRE(kMotorFrameTailsitter == static_cast<std::uint8_t>(MotorFrameClass::kTailsitter));
}

TEST_CASE("tailsitter unconfigured transition_rate_fw auto-set", "[tailsitter][setup]") {
    const float rate = resolve_transition_rate_fw(false, kTransitionRateFwDefault,
                                                  kTransitionAngleFwDefault, 2000);
    REQUIRE_THAT(rate, Catch::Matchers::WithinAbs(45.0f, 1e-6f));

    const float default_time = resolve_transition_rate_fw(false, 99.0f, 45, 5000);
    REQUIRE_THAT(default_time, Catch::Matchers::WithinAbs(18.0f, 1e-6f));
}

TEST_CASE("tailsitter configured transition_rate_fw is left alone", "[tailsitter][setup]") {
    const float rate = resolve_transition_rate_fw(true, 50.0f, 45, 2000);
    REQUIRE_THAT(rate, Catch::Matchers::WithinAbs(50.0f, 1e-6f));
}

TEST_CASE("tailsitter vectored vs not from frame gain and tilt assignment", "[tailsitter][setup]") {
    REQUIRE(setup_is_vectored(kMotorFrameTailsitter, 0.5f, true, false));
    REQUIRE(setup_is_vectored(kMotorFrameTailsitter, 0.5f, false, true));
    REQUIRE_FALSE(setup_is_vectored(kMotorFrameTailsitter, 0.5f, false, false));
    REQUIRE_FALSE(setup_is_vectored(kMotorFrameTailsitter, 0.0f, true, true));
    REQUIRE_FALSE(setup_is_vectored(1, 0.5f, true, true));
}

TEST_CASE("tailsitter elevon and vtail are OR of left and right", "[tailsitter][setup]") {
    REQUIRE_FALSE(resolve_surface_flags(SrvAssigned{}).elevon);
    REQUIRE_FALSE(resolve_surface_flags(SrvAssigned{}).v_tail);

    REQUIRE(resolve_surface_flags(SrvAssigned{.elevon_left = true}).elevon);
    REQUIRE(resolve_surface_flags(SrvAssigned{.elevon_right = true}).elevon);
    REQUIRE(resolve_surface_flags(SrvAssigned{.elevon_left = true, .elevon_right = true}).elevon);

    REQUIRE(resolve_surface_flags(SrvAssigned{.vtail_left = true}).v_tail);
    REQUIRE(resolve_surface_flags(SrvAssigned{.vtail_right = true}).v_tail);
    REQUIRE(resolve_surface_flags(SrvAssigned{.vtail_left = true, .vtail_right = true}).v_tail);

    const auto both = resolve_surface_flags(SrvAssigned{
        .elevator = true,
        .aileron = true,
        .rudder = true,
        .elevon_left = true,
        .vtail_right = true,
    });
    REQUIRE(both.elevator);
    REQUIRE(both.aileron);
    REQUIRE(both.rudder);
    REQUIRE(both.elevon);
    REQUIRE(both.v_tail);
}

TEST_CASE("tailsitter enable==2 sets assist airmode and arm option", "[tailsitter][setup]") {
    const auto effects = resolve_enable2(2, AssistState::kAssistEnabled, AirMode::kOff, 0);
    REQUIRE(effects.applied);
    REQUIRE(effects.assist_state == AssistState::kForceEnabled);
    REQUIRE(effects.air_mode == AirMode::kAssistedFlightOnly);
    REQUIRE(option_is_set(effects.options, QOption::kOnlyArmInQmodeOrAuto));
}

TEST_CASE("tailsitter enable==1 is a no-op for assist airmode and arm", "[tailsitter][setup]") {
    const std::int32_t prior = 1 << 0;
    const auto effects =
        resolve_enable2(1, AssistState::kAssistEnabled, AirMode::kOff, prior);
    REQUIRE_FALSE(effects.applied);
    REQUIRE(effects.assist_state == AssistState::kAssistEnabled);
    REQUIRE(effects.air_mode == AirMode::kOff);
    REQUIRE(effects.options == prior);
    REQUIRE_FALSE(option_is_set(effects.options, QOption::kOnlyArmInQmodeOrAuto));
}

TEST_CASE("tailsitter enable==2 ORs arm bit onto existing options", "[tailsitter][setup]") {
    const auto effects = resolve_enable2(2, AssistState::kAssistDisabled, AirMode::kOn, 1 << 0);
    REQUIRE(option_is_set(effects.options, QOption::kLevelTransition));
    REQUIRE(option_is_set(effects.options, QOption::kOnlyArmInQmodeOrAuto));
}

TEST_CASE("tailsitter apply_enable2_effects writes only when applied", "[tailsitter][setup]") {
    AssistState assist = AssistState::kAssistEnabled;
    AirMode air = AirMode::kOff;
    std::int32_t options = 0;

    apply_enable2_effects(resolve_enable2(1, assist, air, options), assist, air, options);
    REQUIRE(assist == AssistState::kAssistEnabled);
    REQUIRE(air == AirMode::kOff);
    REQUIRE(options == 0);

    apply_enable2_effects(resolve_enable2(2, assist, air, options), assist, air, options);
    REQUIRE(assist == AssistState::kForceEnabled);
    REQUIRE(air == AirMode::kAssistedFlightOnly);
    REQUIRE(option_is_set(options, QOption::kOnlyArmInQmodeOrAuto));
}

TEST_CASE("tailsitter resolve_setup_flags skips leftovers when enable is off", "[tailsitter][setup]") {
    TailsitterSetupInputs setup_in{};
    setup_in.enable = 0;
    setup_in.frame_class = kMotorFrameTailsitter;
    const auto setup = resolve_setup(setup_in);

    TailsitterSetupFlagInputs flags_in{};
    flags_in.frame_class = kMotorFrameTailsitter;
    flags_in.srv.tilt_left = true;
    flags_in.srv.elevon_left = true;
    flags_in.transition_rate_fw_configured = false;

    const auto flags = resolve_setup_flags(setup, flags_in);
    REQUIRE_THAT(flags.transition_rate_fw, Catch::Matchers::WithinAbs(kTransitionRateFwDefault, 1e-6f));
    REQUIRE_FALSE(flags.transition_rate_fw_auto_set);
    REQUIRE_FALSE(flags.is_vectored);
    REQUIRE_FALSE(flags.surfaces.elevon);
    REQUIRE_FALSE(flags.enable2.applied);
}

TEST_CASE("tailsitter resolve_setup_with_flags composes heuristic and leftovers", "[tailsitter][setup]") {
    TailsitterSetupInputs setup_in{};
    setup_in.enable = 2;
    setup_in.frame_class = kMotorFrameTailsitter;

    TailsitterSetupFlagInputs flags_in{};
    flags_in.frame_class = kMotorFrameTailsitter;
    flags_in.vectored_hover_gain = 0.5f;
    flags_in.srv.tilt_right = true;
    flags_in.srv.elevon_right = true;
    flags_in.srv.vtail_left = true;
    flags_in.transition_rate_fw_configured = false;
    flags_in.transition_angle_fw = 45;
    flags_in.transition_time_ms = 2000;

    const auto both = resolve_setup_with_flags(setup_in, flags_in);
    REQUIRE(both.setup.enable == 2);
    REQUIRE(both.setup.setup_complete);
    REQUIRE(both.flags.transition_rate_fw_auto_set);
    REQUIRE_THAT(both.flags.transition_rate_fw, Catch::Matchers::WithinAbs(45.0f, 1e-6f));
    REQUIRE(both.flags.is_vectored);
    REQUIRE(both.flags.surfaces.elevon);
    REQUIRE(both.flags.surfaces.v_tail);
    REQUIRE(both.flags.enable2.applied);
    REQUIRE(both.flags.enable2.assist_state == AssistState::kForceEnabled);
    REQUIRE(both.flags.enable2.air_mode == AirMode::kAssistedFlightOnly);
}
