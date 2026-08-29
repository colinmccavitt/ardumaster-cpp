#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include <fwcpp/math/scalar.hpp>
#include <fwcpp/tiltrotor/tiltrotor_control.hpp>
#include <fwcpp/tiltrotor/tiltrotor_enable.hpp>

using fwcpp::tiltrotor::ContinuousTiltInputs;
using fwcpp::tiltrotor::ContinuousTiltStrategy;
using fwcpp::tiltrotor::SlewFlightFlags;
using fwcpp::tiltrotor::TiltControlState;
using fwcpp::tiltrotor::TiltRateParams;
using fwcpp::tiltrotor::TiltType;
using fwcpp::tiltrotor::TiltUpdatePath;
using fwcpp::tiltrotor::TiltrotorGate;
using fwcpp::tiltrotor::TiltrotorSetupInputs;
using fwcpp::tiltrotor::VectoringInputs;
using fwcpp::tiltrotor::VectoringPath;
using fwcpp::tiltrotor::apply_binary_slew;
using fwcpp::tiltrotor::apply_slew;
using fwcpp::tiltrotor::binary_slew;
using fwcpp::tiltrotor::binary_update;
using fwcpp::tiltrotor::continuous_target_tilt;
using fwcpp::tiltrotor::continuous_update;
using fwcpp::tiltrotor::fully_fwd;
using fwcpp::tiltrotor::fully_up;
using fwcpp::tiltrotor::get_forward_flight_tilt;
using fwcpp::tiltrotor::get_fully_forward_tilt;
using fwcpp::tiltrotor::kFastTiltMinRateDps;
using fwcpp::tiltrotor::kQOptionDisarmedTilt;
using fwcpp::tiltrotor::kServoMotorTiltScale;
using fwcpp::tiltrotor::kThrottleScaledToUnit;
using fwcpp::tiltrotor::kTiltDelayMs;
using fwcpp::tiltrotor::kTiltElevonScale;
using fwcpp::tiltrotor::kVectoringThrottleScaleMax;
using fwcpp::tiltrotor::kVectoringThrottleScaleMin;
using fwcpp::tiltrotor::resolve_continuous_strategy;
using fwcpp::tiltrotor::resolve_setup;
using fwcpp::tiltrotor::resolve_update_path;
using fwcpp::tiltrotor::slew;
using fwcpp::tiltrotor::tilt_max_change;
using fwcpp::tiltrotor::tilt_over_max_angle;
using fwcpp::tiltrotor::update;
using fwcpp::tiltrotor::vectoring;
using fwcpp::tiltrotor::vectoring_geometry;

TEST_CASE("tiltrotor slew uses flap-range max_change args", "[tiltrotor][control]") {
    TiltRateParams rate{};
    rate.g_dt = 0.02f;
    const float max_change = tilt_max_change(rate, TiltType::kContinuous, true, false, false, true, true, false);
    REQUIRE_THAT(max_change, Catch::Matchers::WithinAbs(40.0f * 0.02f / 90.0f, 1e-6f));

    const auto slewed = apply_slew(0.0f, 1.0f, max_change);
    REQUIRE_THAT(slewed.current_tilt, Catch::Matchers::WithinAbs(max_change, 1e-6f));
    REQUIRE_FALSE(slewed.angle_achieved);
    REQUIRE_THAT(slewed.servo_motor_tilt, Catch::Matchers::WithinAbs(kServoMotorTiltScale * max_change, 1e-4f));

    SlewFlightFlags flags{};
    flags.in_vtol_mode = true;
    const float flap_deg = 9.0f;
    const float newtilt = 0.95f;
    REQUIRE(newtilt > get_fully_forward_tilt(flap_deg));
    const auto via_slew = slew(0.80f, newtilt, rate, TiltType::kContinuous, flap_deg, flags);
    const float expected = tilt_max_change(rate, TiltType::kContinuous, newtilt < 0.80f, true, flags.manual_mode,
                                           flags.armed_and_safety_off, flags.in_vtol_mode, flags.assisted_flight);
    REQUIRE_THAT(via_slew.current_tilt, Catch::Matchers::WithinAbs(0.80f + expected, 1e-6f));
}

TEST_CASE("tiltrotor tilt_max_change fast tilt is 90 dps in manual FW", "[tiltrotor][control]") {
    TiltRateParams rate{};
    rate.max_rate_down_dps = 20.0f;
    rate.g_dt = 0.02f;
    const float slow = tilt_max_change(rate, TiltType::kContinuous, false, false, false, true, true, false);
    REQUIRE_THAT(slow, Catch::Matchers::WithinAbs(20.0f * 0.02f / 90.0f, 1e-6f));

    const float fast = tilt_max_change(rate, TiltType::kContinuous, false, false, true, false, false, false);
    REQUIRE_THAT(fast, Catch::Matchers::WithinAbs(kFastTiltMinRateDps * 0.02f / 90.0f, 1e-6f));

    const float binary = tilt_max_change(rate, TiltType::kBinary, false, false, true, true, false, false);
    REQUIRE_THAT(binary, Catch::Matchers::WithinAbs(20.0f * 0.02f / 90.0f, 1e-6f));
}

TEST_CASE("tiltrotor fully_fwd fully_up and forward tilt", "[tiltrotor][control]") {
    REQUIRE_THAT(get_forward_flight_tilt(0.0f, 0.0f), Catch::Matchers::WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(get_forward_flight_tilt(9.0f, 100.0f), Catch::Matchers::WithinAbs(0.9f, 1e-6f));
    const auto gate = TiltrotorGate::from_setup(resolve_setup(TiltrotorSetupInputs{.tilt_mask = 0x3u}));
    REQUIRE(fully_fwd(gate, 0x3u, 1.0f, 0.0f));
    REQUIRE(fully_up(gate, 0x3u, 0.0f));
    REQUIRE_FALSE(fully_fwd(gate, 0u, 1.0f, 0.0f));
    const auto off = TiltrotorGate::from_setup(resolve_setup(TiltrotorSetupInputs{}));
    REQUIRE_FALSE(fully_up(off, 0x3u, 0.0f));
}

TEST_CASE("tiltrotor tilt_over_max_angle", "[tiltrotor][control]") {
    REQUIRE_FALSE(tilt_over_max_angle(0.4f, 45.0f, 0.0f, 0.0f));
    REQUIRE(tilt_over_max_angle(0.6f, 45.0f, 0.0f, 0.0f));
    REQUIRE_FALSE(tilt_over_max_angle(0.75f, 80.0f, 18.0f, 100.0f));
    REQUIRE(tilt_over_max_angle(0.81f, 80.0f, 18.0f, 100.0f));
}

TEST_CASE("tiltrotor continuous strategy", "[tiltrotor][control]") {
    ContinuousTiltInputs in{};
    REQUIRE(resolve_continuous_strategy(in) == ContinuousTiltStrategy::kAssistedThrottleMap);

    in.in_vtol_mode = false;
    in.armed_and_safety_off = false;
    REQUIRE(resolve_continuous_strategy(in) == ContinuousTiltStrategy::kFixedWingPath);
    in.disarmed_tilt_up_option = true;
    REQUIRE_THAT(continuous_target_tilt(ContinuousTiltStrategy::kFixedWingPath, in),
                 Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    in.armed_and_safety_off = true;
    REQUIRE_THAT(continuous_target_tilt(ContinuousTiltStrategy::kFixedWingPath, in),
                 Catch::Matchers::WithinAbs(1.0f, 1e-6f));

    in = ContinuousTiltInputs{};
    in.qautotune_mode = true;
    REQUIRE(resolve_continuous_strategy(in) == ContinuousTiltStrategy::kQautotuneZero);

    in = ContinuousTiltInputs{};
    in.use_calculated_fwd_thr = true;
    in.flying_vtol = true;
    REQUIRE(resolve_continuous_strategy(in) == ContinuousTiltStrategy::kFwdThrGain);

    in = ContinuousTiltInputs{};
    in.qacro_qstab_qhover_mode = true;
    REQUIRE(resolve_continuous_strategy(in) == ContinuousTiltStrategy::kManualRcModes);

    in = ContinuousTiltInputs{};
    in.assisted_flight = true;
    in.transition_at_or_past_timer = true;
    REQUIRE(resolve_continuous_strategy(in) == ContinuousTiltStrategy::kTransitionAllForward);
}

TEST_CASE("tiltrotor fwd tilt atan path uses max_angle and 1/90", "[tiltrotor][control]") {
    ContinuousTiltInputs in{};
    in.forward_throttle_pct = 100.0f;
    in.max_angle_deg = 45.0f;
    const float expected =
        std::min(fwcpp::math::degrees(std::atan(1.0f)), 45.0f) / 90.0f;
    REQUIRE_THAT(continuous_target_tilt(ContinuousTiltStrategy::kFwdThrGain, in),
                 Catch::Matchers::WithinAbs(expected, 1e-6f));
}

TEST_CASE("tiltrotor continuous_update defaults motors_active false", "[tiltrotor][control]") {
    TiltRateParams rate{};
    ContinuousTiltInputs in{};
    in.qautotune_mode = true;
    TiltControlState state{};
    state.motors_active = true;
    const auto out = continuous_update(state, in, rate, TiltType::kContinuous, 0x3u);
    REQUIRE_FALSE(out.state.motors_active);
    REQUIRE_FALSE(out.motor_mask.apply);
    REQUIRE_THAT(out.state.current_tilt, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("tiltrotor continuous_update FW throttle 0.01 and motor mask", "[tiltrotor][control]") {
    TiltRateParams rate{};
    rate.g_dt = 0.02f;
    ContinuousTiltInputs in{};
    in.in_vtol_mode = false;
    in.armed_and_safety_off = true;
    in.throttle_out_scaled = 80.0f;
    TiltControlState state{};
    state.current_tilt = 1.0f;
    const auto out = continuous_update(state, in, rate, TiltType::kContinuous, 0x5u);
    REQUIRE(out.state.motors_active);
    REQUIRE_THAT(out.state.current_throttle, Catch::Matchers::WithinAbs(80.0f * kThrottleScaledToUnit, 1e-6f));
    REQUIRE(out.motor_mask.apply);
    REQUIRE(out.motor_mask.mask == 0x5u);
    REQUIRE_THAT(out.motor_mask.throttle, Catch::Matchers::WithinAbs(0.8f, 1e-6f));

    in.armed_and_safety_off = false;
    in.disarmed_tilt_up_option = true;
    state.current_tilt = 0.5f;
    const auto disarmed = continuous_update(state, in, rate, TiltType::kContinuous, 0x5u);
    REQUIRE_FALSE(disarmed.state.motors_active);
    REQUIRE_THAT(disarmed.state.current_throttle, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE(disarmed.motor_mask.apply);
    REQUIRE(disarmed.motor_mask.mask == 0u);
}

TEST_CASE("tiltrotor binary_slew is not rate-limited on the servo", "[tiltrotor][control]") {
    TiltRateParams rate{};
    rate.g_dt = 0.02f;
    SlewFlightFlags flags{};
    flags.in_vtol_mode = false;
    const auto forward = binary_slew(0.0f, true, rate, TiltType::kBinary, flags);
    REQUIRE_THAT(forward.servo_motor_tilt, Catch::Matchers::WithinAbs(kServoMotorTiltScale, 1e-6f));
    REQUIRE(forward.current_tilt < 1.0f);
    REQUIRE(forward.current_tilt > 0.0f);

    const auto up = apply_binary_slew(1.0f, false, 0.1f);
    REQUIRE_THAT(up.servo_motor_tilt, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(up.current_tilt, Catch::Matchers::WithinAbs(0.9f, 1e-6f));
}

TEST_CASE("tiltrotor binary_update motor mask only when fully forward", "[tiltrotor][control]") {
    TiltRateParams rate{};
    rate.g_dt = 0.02f;
    ContinuousTiltInputs in{};
    in.in_vtol_mode = false;
    in.throttle_out_scaled = 50.0f;
    TiltControlState state{};
    const auto mid = binary_update(state, in, rate, 0x3u);
    REQUIRE(mid.state.motors_active);
    REQUIRE_FALSE(mid.motor_mask.apply);
    REQUIRE_THAT(mid.servo_motor_tilt, Catch::Matchers::WithinAbs(kServoMotorTiltScale, 1e-6f));

    state.current_tilt = 1.0f;
    const auto fwd = binary_update(state, in, rate, 0x3u);
    REQUIRE(fwd.motor_mask.apply);
    REQUIRE(fwd.motor_mask.mask == 0x3u);
    REQUIRE_THAT(fwd.motor_mask.throttle, Catch::Matchers::WithinAbs(0.5f, 1e-6f));

    in.in_vtol_mode = true;
    const auto vtol = binary_update(state, in, rate, 0x3u);
    REQUIRE(vtol.state.motors_active);
    REQUIRE_FALSE(vtol.motor_mask.apply);
    REQUIRE_THAT(vtol.servo_motor_tilt, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("tiltrotor update path", "[tiltrotor][control]") {
    const auto off = TiltrotorGate::from_setup(resolve_setup(TiltrotorSetupInputs{}));
    REQUIRE(resolve_update_path(off, 0, TiltType::kContinuous) == TiltUpdatePath::kNone);
    const auto gate = TiltrotorGate::from_setup(resolve_setup(TiltrotorSetupInputs{.tilt_mask = 1u}));
    REQUIRE(resolve_update_path(gate, 1u, TiltType::kBinary) == TiltUpdatePath::kBinary);
    REQUIRE(resolve_update_path(gate, 1u, TiltType::kVectoredYaw) == TiltUpdatePath::kContinuousThenVectoring);
    REQUIRE(resolve_update_path(gate, 0u, TiltType::kContinuous) == TiltUpdatePath::kNone);

    TiltRateParams rate{};
    ContinuousTiltInputs in{};
    in.qautotune_mode = true;
    const auto vectored = update(gate, 1u, TiltType::kVectoredYaw, {}, in, rate);
    REQUIRE(vectored.path == TiltUpdatePath::kContinuousThenVectoring);
    REQUIRE(vectored.ran_vectoring);
    REQUIRE_FALSE(vectored.state.motors_active);

    const auto none = update(off, 1u, TiltType::kContinuous, {}, in, rate);
    REQUIRE(none.path == TiltUpdatePath::kNone);
    REQUIRE_FALSE(none.ran_vectoring);
}

TEST_CASE("tiltrotor vectoring base_output geometry", "[tiltrotor][vectoring]") {
    const auto geo = vectoring_geometry(0.5f, 15.0f, 15.0f);
    REQUIRE_THAT(geo.total_angle, Catch::Matchers::WithinAbs(120.0f, 1e-6f));
    REQUIRE_THAT(geo.zero_out, Catch::Matchers::WithinAbs(15.0f / 120.0f, 1e-6f));
    REQUIRE_THAT(geo.fixed_tilt_limit, Catch::Matchers::WithinAbs(15.0f / 120.0f, 1e-6f));
    REQUIRE_THAT(geo.level_out, Catch::Matchers::WithinAbs(1.0f - 15.0f / 120.0f, 1e-6f));
    REQUIRE_THAT(geo.base_output, Catch::Matchers::WithinAbs(0.5f, 1e-6f));
}

TEST_CASE("tiltrotor vectoring disarmed VTOL rudder after tilt delay", "[tiltrotor][vectoring]") {
    VectoringInputs in{};
    in.current_tilt = 0.5f;
    in.tilt_yaw_angle = 15.0f;
    in.fixed_angle = 15.0f;
    in.options = kQOptionDisarmedTilt;
    in.armed_and_safety_off = false;
    in.in_vtol_mode = true;
    in.now_ms = 10000;
    in.last_armed_change_ms = 10000 - (kTiltDelayMs + 1);
    in.rudder_control_in = kTiltElevonScale;
    in.rudder_range = kTiltElevonScale;

    const auto out = vectoring(in);
    REQUIRE(out.path == VectoringPath::kDisarmedVtol);
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(625.0f, 1e-3f));
    REQUIRE_THAT(out.tilt_right, Catch::Matchers::WithinAbs(375.0f, 1e-3f));
    REQUIRE_THAT(out.tilt_rear, Catch::Matchers::WithinAbs(500.0f, 1e-3f));
    REQUIRE_THAT(out.tilt_rear_left, Catch::Matchers::WithinAbs(625.0f, 1e-3f));
    REQUIRE_THAT(out.tilt_rear_right, Catch::Matchers::WithinAbs(375.0f, 1e-3f));
    REQUIRE_FALSE(out.motors_limit_yaw);
}

TEST_CASE("tiltrotor vectoring disarmed delay not elapsed skips stick path", "[tiltrotor][vectoring]") {
    VectoringInputs in{};
    in.current_tilt = 0.5f;
    in.tilt_yaw_angle = 15.0f;
    in.fixed_angle = 15.0f;
    in.options = kQOptionDisarmedTilt;
    in.armed_and_safety_off = false;
    in.in_vtol_mode = true;
    in.now_ms = 10000;
    in.last_armed_change_ms = 10000 - (kTiltDelayMs - 1);
    in.rudder_control_in = kTiltElevonScale;
    in.rudder_range = kTiltElevonScale;
    in.motors_yaw = 1.0f;

    const auto out = vectoring(in);
    REQUIRE(out.path == VectoringPath::kNone);
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(out.tilt_right, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(out.tilt_rear, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_FALSE(out.motors_limit_yaw);
}

TEST_CASE("tiltrotor vectoring no_yaw elevon mix uses manual scaler 1", "[tiltrotor][vectoring]") {
    VectoringInputs in{};
    in.current_tilt = 0.6f;
    in.tilt_yaw_angle = 0.0f;
    in.fixed_angle = 15.0f;
    in.fixed_gain = 1.0f;
    in.manual_mode = true;
    in.fw_vector_throttle_scaling = 4.0f;
    in.speed_scaler = 0.5f;
    in.elevon_right = kTiltElevonScale;
    in.elevon_left = 0.0f;
    in.elevator = kTiltElevonScale;
    REQUIRE(tilt_over_max_angle(in.current_tilt, in.max_angle_deg, in.flap_angle_deg, in.flap_auto_slew_pct));

    const auto geo = vectoring_geometry(in.current_tilt, in.tilt_yaw_angle, in.fixed_angle);
    const float gain = in.fixed_gain * geo.fixed_tilt_limit;
    const auto out = vectoring(in);
    REQUIRE(out.path == VectoringPath::kNoYawFixedWing);
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(kServoMotorTiltScale * (geo.base_output - gain), 1e-3f));
    REQUIRE_THAT(out.tilt_right, Catch::Matchers::WithinAbs(kServoMotorTiltScale * geo.base_output, 1e-3f));
    REQUIRE_THAT(out.tilt_rear_left, Catch::Matchers::WithinAbs(kServoMotorTiltScale * geo.base_output, 1e-3f));
    REQUIRE_THAT(out.tilt_rear_right, Catch::Matchers::WithinAbs(kServoMotorTiltScale * (geo.base_output + gain), 1e-3f));
    REQUIRE_THAT(out.tilt_rear, Catch::Matchers::WithinAbs(kServoMotorTiltScale * (geo.base_output + gain), 1e-3f));
}

TEST_CASE("tiltrotor vectoring throttle_scaler clamps hover/throttle", "[tiltrotor][vectoring]") {
    VectoringInputs in{};
    in.current_tilt = 0.0f;
    in.tilt_yaw_angle = 18.0f;
    in.motors_yaw = 0.2f;
    in.hover_throttle = 0.5f;

    in.throttle_out = 0.2f;
    const auto high = vectoring(in);
    const auto geo = vectoring_geometry(in.current_tilt, in.tilt_yaw_angle, in.fixed_angle);
    const float high_scale = kVectoringThrottleScaleMax * 0.2f;
    REQUIRE_THAT(high.tilt_left,
                 Catch::Matchers::WithinAbs(kServoMotorTiltScale * (geo.base_output + high_scale * geo.zero_out),
                                            1e-3f));

    in.throttle_out = 2.0f;
    const auto low = vectoring(in);
    const float low_scale = kVectoringThrottleScaleMin * 0.2f;
    REQUIRE_THAT(low.tilt_left,
                 Catch::Matchers::WithinAbs(kServoMotorTiltScale * (geo.base_output + low_scale * geo.zero_out),
                                            1e-3f));
    REQUIRE(high.tilt_left > low.tilt_left);
    REQUIRE(high.path == VectoringPath::kVectoredYaw);
}

TEST_CASE("tiltrotor vectoring tilt_scale over one sets yaw limit", "[tiltrotor][vectoring]") {
    VectoringInputs in{};
    in.current_tilt = 0.0f;
    in.tilt_yaw_angle = 18.0f;
    in.motors_yaw = 1.0f;
    in.throttle_out = 0.0f;

    const auto out = vectoring(in);
    REQUIRE(out.motors_limit_yaw);
    REQUIRE(out.path == VectoringPath::kVectoredYaw);
    const auto geo = vectoring_geometry(in.current_tilt, in.tilt_yaw_angle, in.fixed_angle);
    REQUIRE_THAT(out.tilt_left,
                 Catch::Matchers::WithinAbs(kServoMotorTiltScale * (geo.base_output + geo.zero_out), 1e-3f));
    REQUIRE_THAT(out.tilt_right,
                 Catch::Matchers::WithinAbs(kServoMotorTiltScale * (geo.base_output - geo.zero_out), 1e-3f));
}

TEST_CASE("tiltrotor vectoring dual saturation sets yaw limit", "[tiltrotor][vectoring]") {
    VectoringInputs in{};
    in.current_tilt = -0.2f;
    in.tilt_yaw_angle = 0.0f;
    in.motors_yaw = 0.0f;
    REQUIRE_FALSE(tilt_over_max_angle(in.current_tilt, in.max_angle_deg, in.flap_angle_deg, in.flap_auto_slew_pct));

    const auto out = vectoring(in);
    REQUIRE(out.path == VectoringPath::kVectoredYaw);
    REQUIRE(out.motors_limit_yaw);
    REQUIRE_THAT(out.tilt_left, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(out.tilt_right, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(out.tilt_rear, Catch::Matchers::WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("tiltrotor update runs vectoring into TiltUpdateResult", "[tiltrotor][vectoring]") {
    const auto gate = TiltrotorGate::from_setup(resolve_setup(TiltrotorSetupInputs{.tilt_mask = 1u}));
    TiltRateParams rate{};
    ContinuousTiltInputs cin{};
    cin.qautotune_mode = true;
    VectoringInputs vin{};
    vin.current_tilt = 0.5f;
    vin.tilt_yaw_angle = 15.0f;
    vin.fixed_angle = 15.0f;
    vin.options = kQOptionDisarmedTilt;
    vin.armed_and_safety_off = false;
    vin.in_vtol_mode = true;
    vin.now_ms = 4000;
    vin.last_armed_change_ms = 0;
    vin.rudder_control_in = kTiltElevonScale;
    vin.rudder_range = kTiltElevonScale;

    const auto out = update(gate, 1u, TiltType::kVectoredYaw, {}, cin, rate, vin);
    REQUIRE(out.ran_vectoring);
    REQUIRE(out.vectoring.path == VectoringPath::kDisarmedVtol);
    vin.current_tilt = out.state.current_tilt;
    const auto direct = vectoring(vin);
    REQUIRE_THAT(out.vectoring.tilt_left, Catch::Matchers::WithinAbs(direct.tilt_left, 1e-6f));
    REQUIRE_THAT(out.vectoring.tilt_right, Catch::Matchers::WithinAbs(direct.tilt_right, 1e-6f));
}
