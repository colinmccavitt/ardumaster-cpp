#include <catch2/catch_test_macros.hpp>

#include <fwcpp/quadplane/quadplane.hpp>
#include <fwcpp/quadplane/quadplane_motors_output.hpp>
#include <fwcpp/quadplane/quadplane_options.hpp>

using fwcpp::quadplane::DesiredSpoolState;
using fwcpp::quadplane::MotorsOutputAction;
using fwcpp::quadplane::MotorsOutputView;
using fwcpp::quadplane::QOption;
using fwcpp::quadplane::QuadPlane;
using fwcpp::quadplane::att_control_relax_stale;
using fwcpp::quadplane::kAttControlRelaxMs;
using fwcpp::quadplane::kMotorsActiveThrottle;
using fwcpp::quadplane::kMotorsInactiveMs;
using fwcpp::quadplane::motors_inactive;
using fwcpp::quadplane::motors_output_skip_tailsitter_transition;
using fwcpp::quadplane::motors_were_active;
using fwcpp::quadplane::option_is_set;

namespace {

QuadPlane available_qp() {
    QuadPlane qp{1};
    REQUIRE(qp.setup());
    return qp;
}

}  // namespace

TEST_CASE("motors_output gating", "[quadplane][motors_output]") {
    QuadPlane qp = available_qp();
    qp.set_options(static_cast<std::int32_t>(QOption::kDelayArming));
    REQUIRE(option_is_set(qp.options(), QOption::kDelayArming));

    MotorsOutputView view{};
    view.armed_and_safety_off = true;
    view.now_ms = 5000;
    view.motors_throttle = 0.5f;
    view.arming_delay_active = true;

    const auto delay = qp.motors_output(view);
    REQUIRE(delay.action == MotorsOutputAction::kDelayArming);
    REQUIRE(delay.desired_spool == DesiredSpoolState::kShutDown);
    REQUIRE(delay.motors_output_ran);
    REQUIRE_FALSE(delay.rate_controller_ran);

    qp.set_options(0);
    view.arming_delay_active = false;
    view.armed_and_safety_off = false;
    const auto disarmed = qp.motors_output(view);
    REQUIRE(disarmed.action == MotorsOutputAction::kDisarmed);

    view.armed_and_safety_off = true;
    view.esc_calibration_qstabilize = true;
    const auto esc = qp.motors_output(view);
    REQUIRE(esc.action == MotorsOutputAction::kEscCalibration);
    REQUIRE_FALSE(esc.motors_output_ran);

    view.esc_calibration_qstabilize = false;
    view.tailsitter_in_vtol_transition = true;
    REQUIRE(motors_output_skip_tailsitter_transition(true, false));
    REQUIRE_FALSE(motors_output_skip_tailsitter_transition(true, true));
    const auto ts = qp.motors_output(view);
    REQUIRE(ts.action == MotorsOutputAction::kTailsitterTransition);
    REQUIRE_FALSE(ts.motors_output_ran);

    view.tailsitter_in_vtol_transition = false;
    view.now_ms = 10000;
    view.motors_throttle = 0.5f;
    const auto out = qp.motors_output(view);
    REQUIRE(out.action == MotorsOutputAction::kOutput);
    REQUIRE(out.motors_output_ran);
    REQUIRE(out.rate_controller_ran);
    REQUIRE(out.attitude_relaxed);
    REQUIRE(out.motors_inactive);
    REQUIRE(qp.motors_output_state().last_motors_active_ms == 10000);
    REQUIRE(qp.motors_output_state().last_att_control_ms == 10000);

    view.now_ms = 10050;
    view.motors_throttle = 0.f;
    const auto quiet = qp.motors_output(view);
    REQUIRE_FALSE(quiet.motors_inactive);
    REQUIRE_FALSE(quiet.attitude_relaxed);
    REQUIRE(qp.motors_output_state().last_motors_active_ms == 10000);

    REQUIRE(motors_inactive(10000 + kMotorsInactiveMs + 1, 10000));
    REQUIRE_FALSE(motors_inactive(10000 + kMotorsInactiveMs, 10000));
    REQUIRE(att_control_relax_stale(10000 + kAttControlRelaxMs + 1, 10000));
    REQUIRE(motors_were_active(kMotorsActiveThrottle + 0.001f, false));
    REQUIRE(motors_were_active(0.f, true));
    REQUIRE_FALSE(motors_were_active(kMotorsActiveThrottle, false));
}
