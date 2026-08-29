#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fwcpp/tailsitter/tailsitter_transition.hpp>

using fwcpp::tailsitter::CompleteReason;
using fwcpp::tailsitter::TailsitterTransition;
using fwcpp::tailsitter::TailsitterTransitionState;
using fwcpp::tailsitter::TransitionCompleteSample;
using fwcpp::tailsitter::kLastVtolModeMs;
using fwcpp::tailsitter::kPitchCdLimit;
using fwcpp::tailsitter::kRollErrorFloorCd;
using fwcpp::tailsitter::kRollErrorMarginCd;
using fwcpp::tailsitter::kTransitionAngleFwDefault;
using fwcpp::tailsitter::kTransitionRateFwDefault;
using fwcpp::tailsitter::kTransitionTimeoutScale;
using fwcpp::tailsitter::kVtolZeroGroundspeedMs;
using fwcpp::tailsitter::kVtolZeroThrottle;
using fwcpp::tailsitter::roll_error_limit_cd;
using fwcpp::quadplane_transition::kMavVtolStateFw;
using fwcpp::quadplane_transition::kMavVtolStateMc;
using fwcpp::quadplane_transition::kMavVtolStateTransitionToFw;
using fwcpp::quadplane_transition::kMavVtolStateTransitionToMc;

TEST_CASE("tailsitter transition complete predicates", "[tailsitter][transition]") {
    REQUIRE(roll_error_limit_cd(4500) == 4500 + kRollErrorMarginCd);
    REQUIRE(roll_error_limit_cd(0) == kRollErrorFloorCd);

    TailsitterTransition ts{};
    REQUIRE(ts.complete());
    REQUIRE(ts.complete());

    TransitionCompleteSample s{};
    s.armed_and_safety_off = false;
    REQUIRE(ts.transition_fw_complete(s) == CompleteReason::kDisarmed);
    REQUIRE(ts.transition_vtol_complete(s) == CompleteReason::kDisarmed);

    ts.restart(0, 0.0f);
    s.armed_and_safety_off = true;
    s.pitch_cd = -4500;
    REQUIRE(!ts.transition_fw_complete(s).has_value());
    s.pitch_cd = -4501;
    REQUIRE(ts.transition_fw_complete(s) == CompleteReason::kPitch);

    ts.restart(100, 0.0f);
    s.pitch_cd = 0;
    s.now_ms = 100 + 1350;
    REQUIRE(!ts.transition_fw_complete(s).has_value());
    s.now_ms = 100 + 1351;
    REQUIRE(ts.transition_fw_complete(s) == CompleteReason::kTimeout);
}

TEST_CASE("tailsitter transition FSM update paths", "[tailsitter][transition]") {
    TailsitterTransition ts{};
    ts.restart(0, 0.0f);
    TransitionCompleteSample s{};
    s.now_ms = 1000;
    const auto out = ts.update(s, false, 0.35f, 0.20f);
    REQUIRE(out.use_synthetic_airspeed);
    REQUIRE(out.assisted_flight);
    REQUIRE(out.nav_pitch_cd.has_value());
    REQUIRE(*out.nav_pitch_cd == -5000);
    REQUIRE(out.throttle.has_value());
    REQUIRE_THAT(*out.throttle, Catch::Matchers::WithinAbs(0.35, 1e-6f));

    s.now_ms = kLastVtolModeMs + 1;
    s.pitch_cd = 5000;
    const auto vout = ts.vtol_update(s, 1200.0f);
    REQUIRE(vout.completed == CompleteReason::kPitch);
    REQUIRE(vout.start_vtol_limit);
    REQUIRE(ts.active_frwd());

    REQUIRE(ts.get_mav_vtol_state(false) == kMavVtolStateTransitionToFw);
    REQUIRE(ts.get_mav_vtol_state(true) == kMavVtolStateMc);
}

TEST_CASE("tailsitter mav and view helpers", "[tailsitter][transition]") {
    TailsitterTransition ts{};
    REQUIRE(ts.get_mav_vtol_state(false) == kMavVtolStateFw);
    ts.restart(0, 0.0f);
    REQUIRE(ts.show_vtol_view(false));
    REQUIRE(ts.show_vtol_view(true));
}
