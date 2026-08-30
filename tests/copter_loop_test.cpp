#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <string_view>

#include <fwcpp/copter/copter.hpp>
#include <fwcpp/location.hpp>

using fwcpp::AltitudeContext;
using fwcpp::Location;
using fwcpp::copter::CheckEkfResetInputs;
using fwcpp::copter::EKFResetMethod;
using fwcpp::copter::Mode;
using fwcpp::copter::ModeAltHold;
using fwcpp::copter::ModeStabilize;
using fwcpp::copter::ModeSwitchReadInputs;
using fwcpp::copter::ModeSwitchReadLeftover;
using fwcpp::copter::MotorsOutputInputs;
using fwcpp::copter::MotorsOutputMainLeftover;
using fwcpp::copter::PortStatus;
using fwcpp::copter::RateControllerMainInputs;
using fwcpp::copter::ReadInertiaInputs;
using fwcpp::copter::TaskKind;
using fwcpp::copter::SpoolState;
using fwcpp::copter::UpdateFlightModeInputs;
using fwcpp::copter::UpdateHomeFromEkfInputs;
using fwcpp::copter::UpdateAltitudeInputs;
using fwcpp::copter::UpdateBattCompassInputs;
using fwcpp::copter::UpdateLandAndCrashDetectorsInputs;
using fwcpp::copter::UpdateRangefinderTerrainOffsetInputs;
using fwcpp::copter::UpdateThrottleHoverInputs;
using fwcpp::copter::LoopRateLoggingInputs;
using fwcpp::copter::TenHzLoggingLoopInputs;
using fwcpp::copter::completeness_has;
using fwcpp::copter::copter_completeness_size;
using fwcpp::copter::find_scheduler_task;
using fwcpp::copter::first_scheduled_always_on;
using fwcpp::copter::get_scheduler_tasks;
using fwcpp::copter::kArmingDelayMs;
using fwcpp::copter::kCopterLoopRateHz;
using fwcpp::copter::kLoopRateHz;
using fwcpp::copter::kMaskLogPm;
using fwcpp::copter::kRcLoopMaxTimeMicros;
using fwcpp::copter::kRcLoopPriority;
using fwcpp::copter::kRcLoopRateHz;
using fwcpp::copter::kSchedulerTasks;
using fwcpp::copter::check_ekf_reset;
using fwcpp::copter::motors_output;
using fwcpp::copter::motors_output_main;
using fwcpp::copter::on_main_count;
using fwcpp::copter::out_of_scope_count;
using fwcpp::copter::rc_loop;
using fwcpp::copter::read_ahrs;
using fwcpp::copter::read_inertia;
using fwcpp::copter::read_mode_switch;
using fwcpp::copter::remaining_count;
using fwcpp::copter::run_rate_controller_main;
using fwcpp::copter::scheduler_task_count;
using fwcpp::copter::this_slice_count;
using fwcpp::copter::three_hz_loop;
using fwcpp::copter::loop_rate_logging;
using fwcpp::copter::ten_hz_logging_loop;
using fwcpp::copter::throttle_loop;
using fwcpp::copter::kGravityMss;
using fwcpp::copter::update_flight_mode;
using fwcpp::copter::update_home_from_ekf;
using fwcpp::copter::run_nav_updates;
using fwcpp::copter::update_altitude;
using fwcpp::copter::update_batt_compass;
using fwcpp::copter::update_land_and_crash_detectors;
using fwcpp::copter::update_rangefinder_terrain_offset;
using fwcpp::copter::update_throttle_hover;

namespace {

class TestRunMode : public Mode {
public:
    int run_count{0};

    [[nodiscard]] Number mode_number() const override { return Number::STABILIZE; }
    void run() override { ++run_count; }
    [[nodiscard]] bool requires_position() const override { return false; }
    [[nodiscard]] bool has_manual_throttle() const override { return true; }
};

}  // namespace

TEST_CASE("catalog remaining_count stays open after slice 16", "[copter][leftover]") {
    REQUIRE(remaining_count() == 15);
    REQUIRE(this_slice_count() == 2);
    REQUIRE(on_main_count() == 20);
    REQUIRE(copter_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("Copter::rc_loop", PortStatus::kOnMain));
    REQUIRE(completeness_has("RC_Channels::read_mode_switch", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::scheduler_tasks[]", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::get_scheduler_tasks", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::motors_output / motors_output_main", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::read_AHRS", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::throttle_loop", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::run_rate_controller_main", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::read_inertia", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::check_ekf_reset", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::update_flight_mode", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::update_home_from_EKF", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::update_land_and_crash_detectors", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::update_rangefinder_terrain_offset", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::update_batt_compass", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::update_altitude", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::run_nav_updates", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::update_throttle_hover", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::three_hz_loop", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::loop_rate_logging", PortStatus::kOnMain));
    REQUIRE(completeness_has("leftover catalog", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Copter::ten_hz_logging_loop", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Copter::twentyfive_hz_logging", PortStatus::kRemaining));
    REQUIRE(completeness_has("Copter::update_super_simple_bearing", PortStatus::kRemaining));
    REQUIRE(completeness_has("Copter::update_auto_armed", PortStatus::kRemaining));
    REQUIRE(completeness_has("Copter::init_ardupilot", PortStatus::kRemaining));
    REQUIRE(completeness_has("AP:: singletons", PortStatus::kOutOfScope));
}

TEST_CASE("rc_loop is first scheduled always-on row", "[copter][scheduler]") {
    const auto* row = first_scheduled_always_on();
    REQUIRE(row != nullptr);
    REQUIRE(row->name != nullptr);
    REQUIRE(std::string_view(row->name) == "rc_loop");
    REQUIRE(row->kind == TaskKind::kScheduled);
    REQUIRE(row->rate_hz == kRcLoopRateHz);
    REQUIRE(row->max_time_micros == kRcLoopMaxTimeMicros);
    REQUIRE(row->priority == kRcLoopPriority);
    REQUIRE(row->gate == nullptr);

    const auto view = get_scheduler_tasks();
    REQUIRE(view.log_bit == kMaskLogPm);
    REQUIRE(view.task_count == scheduler_task_count());
    REQUIRE(view.tasks == kSchedulerTasks);
    REQUIRE(view.task_count > 1);

    const auto* button = find_scheduler_task("AP_Button::update");
    REQUIRE(button != nullptr);
    REQUIRE(std::string_view(button->gate) == "HAL_BUTTON_ENABLED");
    REQUIRE(button->rate_hz == 5.0f);
    REQUIRE(button->max_time_micros == 100);
    REQUIRE(button->priority == 168);

    REQUIRE(kSchedulerTasks[0].kind == TaskKind::kFast);
    REQUIRE(std::string_view(kSchedulerTasks[0].name) == "AP_InertialSensor::update");
    REQUIRE(kCopterLoopRateHz == 400);
}

TEST_CASE("rc_loop leftover cases", "[copter][rc_loop]") {
    const auto invalid = rc_loop(ModeSwitchReadInputs{
        .has_valid_input = false,
        .flight_mode_channel = static_cast<std::uint8_t>(4),
    });
    REQUIRE(invalid.read_radio);
    REQUIRE(invalid.mode_switch == ModeSwitchReadLeftover::kNoValidInput);
    REQUIRE(read_mode_switch({.has_valid_input = false, .flight_mode_channel = static_cast<std::uint8_t>(4)}) ==
            ModeSwitchReadLeftover::kNoValidInput);

    const auto missing = rc_loop(ModeSwitchReadInputs{.has_valid_input = true, .flight_mode_channel = {}});
    REQUIRE(missing.read_radio);
    REQUIRE(missing.mode_switch == ModeSwitchReadLeftover::kNoChannel);

    const auto ok = rc_loop(ModeSwitchReadInputs{
        .has_valid_input = true,
        .flight_mode_channel = static_cast<std::uint8_t>(5),
    });
    REQUIRE(ok.read_radio);
    REQUIRE(ok.mode_switch == ModeSwitchReadLeftover::kRead);
}

TEST_CASE("motors_output_main skips when rate thread owns it", "[copter][motors_output]") {
    MotorsOutputInputs in{};
    in.using_rate_thread = true;
    in.motors_armed = true;
    const auto skipped = motors_output_main(in);
    REQUIRE(skipped.leftover == MotorsOutputMainLeftover::kSkipped);
    REQUIRE_FALSE(skipped.output.calc_pwm);
    REQUIRE_FALSE(skipped.output.push_srv);

    in.using_rate_thread = false;
    const auto ran = motors_output_main(in);
    REQUIRE(ran.leftover == MotorsOutputMainLeftover::kRan);
    REQUIRE(ran.output.push_srv);
    REQUIRE_FALSE(ran.output.push_rcout);
}

TEST_CASE("motors_output afs crash returns unless landing", "[copter][motors_output]") {
    MotorsOutputInputs in{};
    in.afs_should_crash = true;
    in.afs_terminating_via_landing = false;
    in.motors_armed = true;
    const auto crash = motors_output(in);
    REQUIRE(crash.skip_output);
    REQUIRE_FALSE(crash.calc_pwm);
    REQUIRE_FALSE(crash.cork);
    REQUIRE_FALSE(crash.output_ch_all);
    REQUIRE_FALSE(crash.output_to_motors);
    REQUIRE_FALSE(crash.push_srv);

    in.afs_terminating_via_landing = true;
    const auto landing = motors_output(in);
    REQUIRE_FALSE(landing.skip_output);
    REQUIRE(landing.calc_pwm);
    REQUIRE(landing.cork);
    REQUIRE(landing.output_ch_all);
    REQUIRE(landing.output_to_motors);
}

TEST_CASE("motors_output interlock enable and disable", "[copter][motors_output]") {
    MotorsOutputInputs in{};
    in.motors_armed = true;
    in.using_interlock = true;
    in.motor_interlock_switch = false;
    const auto blocked = motors_output(in);
    REQUIRE_FALSE(blocked.set_interlock_true);
    REQUIRE_FALSE(blocked.set_interlock_false);
    REQUIRE_FALSE(blocked.log_interlock_enabled);
    REQUIRE(blocked.calc_pwm);
    REQUIRE(blocked.cork);
    REQUIRE(blocked.output_ch_all);

    in.motor_interlock_switch = true;
    in.emergency_stop = true;
    const auto estop = motors_output(in);
    REQUIRE_FALSE(estop.set_interlock_true);
    REQUIRE_FALSE(estop.set_interlock_false);

    in.emergency_stop = false;
    const auto on = motors_output(in);
    REQUIRE(on.set_interlock_true);
    REQUIRE_FALSE(on.set_interlock_false);
    REQUIRE(on.log_interlock_enabled);
    REQUIRE_FALSE(on.log_interlock_disabled);

    in.motors_interlock = true;
    in.motors_armed = false;
    const auto off = motors_output(in);
    REQUIRE_FALSE(off.set_interlock_true);
    REQUIRE(off.set_interlock_false);
    REQUIRE(off.log_interlock_disabled);
}

TEST_CASE("motors_output motor_test vs output_to_motors", "[copter][motors_output]") {
    MotorsOutputInputs in{};
    const auto flight = motors_output(in);
    REQUIRE(flight.output_to_motors);
    REQUIRE_FALSE(flight.motor_test_output);

    in.motor_test = true;
    const auto test = motors_output(in);
    REQUIRE(test.motor_test_output);
    REQUIRE_FALSE(test.output_to_motors);
}

TEST_CASE("motors_output full_push srv vs rcout", "[copter][motors_output]") {
    MotorsOutputInputs in{};
    const auto srv = motors_output(in, true);
    REQUIRE(srv.push_srv);
    REQUIRE_FALSE(srv.push_rcout);

    const auto rcout = motors_output(in, false);
    REQUIRE(rcout.push_rcout);
    REQUIRE_FALSE(rcout.push_srv);
}

TEST_CASE("motors_output arming delay timeout and THROW", "[copter][motors_output]") {
    REQUIRE(kArmingDelayMs == 2000);

    MotorsOutputInputs in{};
    in.in_arming_delay = true;
    in.motors_armed = true;
    in.arm_time_ms = 0;
    in.now_ms = kArmingDelayMs;
    const auto held = motors_output(in);
    REQUIRE_FALSE(held.clear_arming_delay);
    REQUIRE_FALSE(held.set_interlock_true);

    in.now_ms = kArmingDelayMs + 1;
    const auto timed_out = motors_output(in);
    REQUIRE(timed_out.clear_arming_delay);
    REQUIRE(timed_out.set_interlock_true);
    REQUIRE(timed_out.log_interlock_enabled);

    in.now_ms = 0;
    in.motors_armed = false;
    const auto disarmed = motors_output(in);
    REQUIRE(disarmed.clear_arming_delay);
    REQUIRE_FALSE(disarmed.set_interlock_true);

    in.motors_armed = true;
    in.mode_is_throw = true;
    const auto throw_mode = motors_output(in);
    REQUIRE(throw_mode.clear_arming_delay);
    REQUIRE(throw_mode.set_interlock_true);
}

TEST_CASE("read_AHRS skip_ins_update is always true", "[copter][read_ahrs]") {
    const auto leftover = read_ahrs();
    REQUIRE(leftover.skip_ins_update);
}

TEST_CASE("throttle_loop leftover always-on mix auto_armed gnd-effect ekf-terrain",
          "[copter][throttle_loop]") {
    const auto leftover = throttle_loop();
    REQUIRE(leftover.update_throttle_mix);
    REQUIRE(leftover.update_auto_armed);
    REQUIRE(leftover.update_ground_effect_detector);
    REQUIRE(leftover.update_ekf_terrain_height_stable);
    REQUIRE_FALSE(leftover.heli_update_rotor_speed_targets);
    REQUIRE_FALSE(leftover.heli_update_landing_swash);

    const auto* row = find_scheduler_task("throttle_loop");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kScheduled);
    REQUIRE(row->rate_hz == 50.0f);
    REQUIRE(row->max_time_micros == 75);
    REQUIRE(row->priority == 6);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("run_rate_controller_main skips rate_controller_run on rate thread",
          "[copter][run_rate_controller]") {
    RateControllerMainInputs in{};
    in.last_loop_time_s = 0.0025f;
    in.using_rate_thread = true;
    const auto skipped = run_rate_controller_main(in);
    REQUIRE(skipped.last_loop_time_s == 0.0025f);
    REQUIRE(skipped.pos_control_set_dt_s);
    REQUIRE(skipped.attitude_control_set_dt_s);
    REQUIRE_FALSE(skipped.motors_set_dt_s);
    REQUIRE_FALSE(skipped.rate_controller_run);
    REQUIRE(skipped.rate_controller_target_reset);

    in.using_rate_thread = false;
    const auto on_main = run_rate_controller_main(in);
    REQUIRE(on_main.pos_control_set_dt_s);
    REQUIRE(on_main.attitude_control_set_dt_s);
    REQUIRE(on_main.motors_set_dt_s);
    REQUIRE(on_main.rate_controller_run);
    REQUIRE(on_main.rate_controller_target_reset);

    const auto* row = find_scheduler_task("run_rate_controller_main");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kFast);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("run_rate_controller_main always rate_controller_target_reset",
          "[copter][run_rate_controller]") {
    REQUIRE(run_rate_controller_main({.using_rate_thread = false}).rate_controller_target_reset);
    REQUIRE(run_rate_controller_main({.using_rate_thread = true}).rate_controller_target_reset);
}

TEST_CASE("read_inertia no-alt early return copies lat/lng only", "[copter][read_inertia]") {
    Location current;
    current.set_alt_m(50.0f, Location::AltFrame::ABOVE_HOME);
    const std::int32_t prior_alt = current.alt;
    REQUIRE(prior_alt == 5000);

    ReadInertiaInputs in{};
    in.high_vibes = true;
    in.follow_enabled = true;
    in.ahrs_lat = -353632621;
    in.ahrs_lng = 1491652374;
    in.has_rel_pos_d = false;
    in.pos_d_m = -12.0f;
    in.home_is_set = true;

    AltitudeContext ctx{};
    ctx.home_is_set = true;
    ctx.origin_is_set = true;

    const auto fx = read_inertia(in, current, ctx);
    REQUIRE(fx.pos_control_update_estimates);
    REQUIRE(fx.high_vibes);
    REQUIRE(fx.update_follow_estimates);
    REQUIRE_FALSE(fx.alt_written);
    REQUIRE_FALSE(fx.set_alt_above_home_fallback);
    REQUIRE(current.lat == -353632621);
    REQUIRE(current.lng == 1491652374);
    REQUIRE(current.alt == prior_alt);
    REQUIRE(current.get_alt_frame() == Location::AltFrame::ABOVE_HOME);
}

TEST_CASE("read_inertia home not set falls back to origin metres as above-home",
          "[copter][read_inertia]") {
    Location current;
    current.alt = 999;
    ReadInertiaInputs in{};
    in.high_vibes = false;
    in.ahrs_lat = 100;
    in.ahrs_lng = 200;
    in.has_rel_pos_d = true;
    in.pos_d_m = -10.0f;
    in.home_is_set = false;

    AltitudeContext ctx{};
    ctx.home_is_set = false;
    ctx.origin_is_set = true;
    ctx.ekf_origin = Location(0, 0, 8000, Location::AltFrame::ABSOLUTE);

    const auto fx = read_inertia(in, current, ctx);
    REQUIRE(fx.pos_control_update_estimates);
    REQUIRE_FALSE(fx.high_vibes);
    REQUIRE_FALSE(fx.update_follow_estimates);
    REQUIRE(fx.alt_written);
    REQUIRE(fx.set_alt_above_home_fallback);
    REQUIRE(current.lat == 100);
    REQUIRE(current.lng == 200);
    REQUIRE(current.alt == 1000);
    REQUIRE(current.get_alt_frame() == Location::AltFrame::ABOVE_HOME);
}

TEST_CASE("read_inertia home set converts ABOVE_ORIGIN to ABOVE_HOME",
          "[copter][read_inertia]") {
    Location current;
    ReadInertiaInputs in{};
    in.ahrs_lat = 11;
    in.ahrs_lng = 22;
    in.has_rel_pos_d = true;
    in.pos_d_m = -10.0f;
    in.home_is_set = true;

    AltitudeContext ctx{};
    ctx.home_is_set = true;
    ctx.home = Location(0, 0, 10000, Location::AltFrame::ABSOLUTE);
    ctx.origin_is_set = true;
    ctx.ekf_origin = Location(0, 0, 8000, Location::AltFrame::ABSOLUTE);

    const auto fx = read_inertia(in, current, ctx);
    REQUIRE(fx.alt_written);
    REQUIRE_FALSE(fx.set_alt_above_home_fallback);
    REQUIRE(current.lat == 11);
    REQUIRE(current.lng == 22);
    // 10 m above origin; origin 80 m AMSL, home 100 m AMSL → -10 m above home.
    REQUIRE(current.alt == -1000);
    REQUIRE(current.get_alt_frame() == Location::AltFrame::ABOVE_HOME);
}

TEST_CASE("read_inertia home set but change_alt_frame fails uses origin metres",
          "[copter][read_inertia]") {
    Location current;
    ReadInertiaInputs in{};
    in.ahrs_lat = 1;
    in.ahrs_lng = 2;
    in.has_rel_pos_d = true;
    in.pos_d_m = -5.0f;
    in.home_is_set = true;

    AltitudeContext ctx{};
    ctx.home_is_set = true;
    ctx.home = Location(0, 0, 10000, Location::AltFrame::ABSOLUTE);
    ctx.origin_is_set = false;

    const auto fx = read_inertia(in, current, ctx);
    REQUIRE(fx.alt_written);
    REQUIRE(fx.set_alt_above_home_fallback);
    REQUIRE(current.alt == 500);
    REQUIRE(current.get_alt_frame() == Location::AltFrame::ABOVE_HOME);
}

TEST_CASE("read_inertia high_vibes passed through to pos_control leftover",
          "[copter][read_inertia]") {
    Location current;
    AltitudeContext ctx{};
    ReadInertiaInputs in{};
    in.has_rel_pos_d = false;

    in.high_vibes = true;
    REQUIRE(read_inertia(in, current, ctx).high_vibes);
    in.high_vibes = false;
    REQUIRE_FALSE(read_inertia(in, current, ctx).high_vibes);

    const auto* row = find_scheduler_task("read_inertia");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kFast);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("check_ekf_reset yaw reset edge sets inertial_frame_reset and log",
          "[copter][check_ekf_reset]") {
    CheckEkfResetInputs in{};
    in.last_yaw_reset_ms = 0;
    in.new_ekf_yaw_reset_ms = 1234;
    in.yaw_angle_change_rad = 0.25f;
    in.primary_core_index = 0;
    in.new_primary_core_index = 0;

    const auto fx = check_ekf_reset(in);
    REQUIRE(fx.inertial_frame_reset);
    REQUIRE(fx.last_yaw_reset_ms == 1234);
    REQUIRE(fx.log_ekf_yaw_reset);
    REQUIRE(fx.primary_core_index == 0);
    REQUIRE_FALSE(fx.log_ekf_primary_error);
    REQUIRE_FALSE(fx.gcs_text);

    const auto* row = find_scheduler_task("check_ekf_reset");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kFast);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("check_ekf_reset same-ms is a no-op", "[copter][check_ekf_reset]") {
    CheckEkfResetInputs in{};
    in.last_yaw_reset_ms = 500;
    in.new_ekf_yaw_reset_ms = 500;
    in.yaw_angle_change_rad = 1.0f;
    in.primary_core_index = 1;
    in.new_primary_core_index = 1;

    const auto fx = check_ekf_reset(in);
    REQUIRE_FALSE(fx.inertial_frame_reset);
    REQUIRE(fx.last_yaw_reset_ms == 500);
    REQUIRE_FALSE(fx.log_ekf_yaw_reset);
    REQUIRE(fx.primary_core_index == 1);
    REQUIRE_FALSE(fx.log_ekf_primary_error);
    REQUIRE_FALSE(fx.gcs_text);
}

TEST_CASE("check_ekf_reset primary -1 is ignored", "[copter][check_ekf_reset]") {
    CheckEkfResetInputs in{};
    in.last_yaw_reset_ms = 10;
    in.new_ekf_yaw_reset_ms = 10;
    in.primary_core_index = 0;
    in.new_primary_core_index = -1;

    const auto fx = check_ekf_reset(in);
    REQUIRE_FALSE(fx.inertial_frame_reset);
    REQUIRE(fx.last_yaw_reset_ms == 10);
    REQUIRE_FALSE(fx.log_ekf_yaw_reset);
    REQUIRE(fx.primary_core_index == 0);
    REQUIRE_FALSE(fx.log_ekf_primary_error);
    REQUIRE_FALSE(fx.gcs_text);
}

TEST_CASE("check_ekf_reset primary change logs EKF_PRIMARY and gcs_text",
          "[copter][check_ekf_reset]") {
    CheckEkfResetInputs in{};
    in.last_yaw_reset_ms = 42;
    in.new_ekf_yaw_reset_ms = 42;
    in.primary_core_index = 0;
    in.new_primary_core_index = 1;

    const auto fx = check_ekf_reset(in);
    REQUIRE(fx.inertial_frame_reset);
    REQUIRE(fx.last_yaw_reset_ms == 42);
    REQUIRE_FALSE(fx.log_ekf_yaw_reset);
    REQUIRE(fx.primary_core_index == 1);
    REQUIRE(fx.log_ekf_primary_error);
    REQUIRE(fx.gcs_text);
}

TEST_CASE("update_flight_mode land_complete drives landed_gain_reduction",
          "[copter][update_flight_mode]") {
    TestRunMode mode;
    UpdateFlightModeInputs in{};
    in.current = &mode;
    in.land_complete = true;

    const auto landed = update_flight_mode(in);
    REQUIRE(landed.invalidate_surface_tracking);
    REQUIRE(landed.landed_gain_reduction);
    REQUIRE(landed.ekf_reset_method == EKFResetMethod::MoveTarget);

    in.land_complete = false;
    const auto airborne = update_flight_mode(in);
    REQUIRE(airborne.invalidate_surface_tracking);
    REQUIRE_FALSE(airborne.landed_gain_reduction);

    const auto* row = find_scheduler_task("update_flight_mode");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kFast);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("update_flight_mode ekf reset method MoveVehicle vs MoveTarget",
          "[copter][update_flight_mode]") {
    TestRunMode mode;
    UpdateFlightModeInputs in{};
    in.current = &mode;
    in.move_vehicle_on_ekf_reset = true;

    const auto move_vehicle = update_flight_mode(in);
    REQUIRE(move_vehicle.ekf_reset_method == EKFResetMethod::MoveVehicle);

    in.move_vehicle_on_ekf_reset = false;
    const auto move_target = update_flight_mode(in);
    REQUIRE(move_target.ekf_reset_method == EKFResetMethod::MoveTarget);
}

TEST_CASE("update_flight_mode invokes current run including Stabilize AltHold no-ops",
          "[copter][update_flight_mode]") {
    TestRunMode mode;
    UpdateFlightModeInputs in{};
    in.current = &mode;

    const auto counted = update_flight_mode(in);
    REQUIRE(counted.run_called);
    REQUIRE(mode.run_count == 1);
    (void)update_flight_mode(in);
    REQUIRE(mode.run_count == 2);

    ModeStabilize stabilize;
    ModeAltHold althold;
    in.current = &stabilize;
    REQUIRE(update_flight_mode(in).run_called);
    in.current = &althold;
    REQUIRE(update_flight_mode(in).run_called);
}

TEST_CASE("update_home_from_ekf home already set is a no-op", "[copter][update_home_from_ekf]") {
    Location home(-353632621, 1491652374, 3300, Location::AltFrame::ABSOLUTE);
    const Location prior = home;

    UpdateHomeFromEkfInputs in{};
    in.home_is_set = true;
    in.armed = true;
    in.get_location_ok = true;
    in.location = Location(99, 88, 1000, Location::AltFrame::ABSOLUTE);
    in.get_origin_ok = true;
    in.origin = Location(1, 2, 5000, Location::AltFrame::ABOVE_ORIGIN);

    const auto fx = update_home_from_ekf(in, home);
    REQUIRE(fx.home_is_set);
    REQUIRE_FALSE(fx.set_home_called);
    REQUIRE_FALSE(fx.set_home_ok);
    REQUIRE_FALSE(fx.lock_home);
    REQUIRE_FALSE(fx.copy_alt_from_origin);
    REQUIRE_FALSE(fx.inflight);
    REQUIRE_FALSE(fx.smart_rtl_set_home);
    REQUIRE(home.lat == prior.lat);
    REQUIRE(home.lng == prior.lng);
    REQUIRE(home.alt == prior.alt);

    const auto* row = find_scheduler_task("update_home_from_EKF");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kFast);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("update_home_from_ekf disarmed + location writes home", "[copter][update_home_from_ekf]") {
    Location home;
    home.alt = 999;

    UpdateHomeFromEkfInputs in{};
    in.home_is_set = false;
    in.armed = false;
    in.get_location_ok = true;
    in.location = Location(-353632621, 1491652374, 1234, Location::AltFrame::ABSOLUTE);
    in.get_origin_ok = true;
    in.origin = Location(0, 0, 8000, Location::AltFrame::ABSOLUTE);

    const auto fx = update_home_from_ekf(in, home);
    REQUIRE(fx.set_home_called);
    REQUIRE(fx.set_home_ok);
    REQUIRE(fx.home_is_set);
    REQUIRE_FALSE(fx.lock_home);
    REQUIRE_FALSE(fx.inflight);
    REQUIRE_FALSE(fx.copy_alt_from_origin);
    REQUIRE_FALSE(fx.smart_rtl_set_home);
    REQUIRE(home.lat == -353632621);
    REQUIRE(home.lng == 1491652374);
    REQUIRE(home.alt == 1234);
}

TEST_CASE("update_home_from_ekf disarmed + no location leaves home unset",
          "[copter][update_home_from_ekf]") {
    Location home;
    home.alt = 999;

    UpdateHomeFromEkfInputs in{};
    in.armed = false;
    in.get_location_ok = false;
    in.location = Location(1, 2, 3, Location::AltFrame::ABSOLUTE);
    in.get_origin_ok = true;

    const auto fx = update_home_from_ekf(in, home);
    REQUIRE_FALSE(fx.set_home_called);
    REQUIRE_FALSE(fx.set_home_ok);
    REQUIRE_FALSE(fx.home_is_set);
    REQUIRE_FALSE(fx.inflight);
    REQUIRE(home.alt == 999);
}

TEST_CASE("update_home_from_ekf armed inflight copies origin alt; missing origin skips",
          "[copter][update_home_from_ekf]") {
    Location home;
    UpdateHomeFromEkfInputs in{};
    in.armed = true;
    in.get_location_ok = true;
    in.location = Location(-353632621, 1491652374, 1000, Location::AltFrame::ABSOLUTE);
    in.get_origin_ok = true;
    in.origin = Location(0, 0, 8000, Location::AltFrame::ABOVE_ORIGIN);

    const auto fx = update_home_from_ekf(in, home);
    REQUIRE(fx.inflight);
    REQUIRE(fx.copy_alt_from_origin);
    REQUIRE(fx.set_home_called);
    REQUIRE(fx.set_home_ok);
    REQUIRE(fx.home_is_set);
    REQUIRE_FALSE(fx.lock_home);
    REQUIRE_FALSE(fx.smart_rtl_set_home);
    REQUIRE(home.lat == -353632621);
    REQUIRE(home.lng == 1491652374);
    REQUIRE(home.alt == 8000);
    REQUIRE(home.get_alt_frame() == Location::AltFrame::ABOVE_ORIGIN);

    Location skipped;
    skipped.alt = 42;
    in.get_origin_ok = false;
    const auto miss = update_home_from_ekf(in, skipped);
    REQUIRE(miss.inflight);
    REQUIRE_FALSE(miss.copy_alt_from_origin);
    REQUIRE_FALSE(miss.set_home_called);
    REQUIRE_FALSE(miss.set_home_ok);
    REQUIRE_FALSE(miss.home_is_set);
    REQUIRE(skipped.alt == 42);
}

TEST_CASE("update_land_and_crash_detectors disarmed sets land_complete",
          "[copter][update_land_and_crash_detectors]") {
    UpdateLandAndCrashDetectorsInputs in{};
    in.land.armed = false;
    in.land.land_complete = false;
    in.land.land_detector_count = 7;

    const auto fx = update_land_and_crash_detectors(in);
    REQUIRE(fx.update_land_detector);
    REQUIRE(fx.filter_apply);
    REQUIRE_FALSE(fx.parachute_check);
    REQUIRE_FALSE(fx.crash_check);
    REQUIRE_FALSE(fx.thrust_loss_check);
    REQUIRE_FALSE(fx.yaw_imbalance_check);
    REQUIRE(fx.land.land_complete);
    REQUIRE(fx.land.land_complete_maybe);
    REQUIRE_FALSE(fx.land.internal_error_flow_of_control);
    REQUIRE(fx.land.land_detector_count == 0);

    const auto* row = find_scheduler_task("update_land_and_crash_detectors");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kFast);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("update_land_and_crash_detectors already landed + high throttle clears land",
          "[copter][update_land_and_crash_detectors]") {
    UpdateLandAndCrashDetectorsInputs in{};
    in.land.armed = true;
    in.land.land_complete = true;
    in.land.is_taking_off = false;
    in.land.throttle_out = 0.5f;
    in.land.non_takeoff_throttle = 0.1f;
    in.land.spool = SpoolState::THROTTLE_UNLIMITED;
    in.land.land_detector_count = 12;

    const auto fx = update_land_and_crash_detectors(in);
    REQUIRE_FALSE(fx.land.land_complete);
    REQUIRE_FALSE(fx.land.land_complete_maybe);
    REQUIRE(fx.land.internal_error_flow_of_control);
    REQUIRE(fx.land.land_detector_count == 0);
    REQUIRE(fx.update_land_detector);
    REQUIRE_FALSE(fx.crash_check);

    in.land.is_taking_off = true;
    const auto taking_off = update_land_and_crash_detectors(in);
    REQUIRE(taking_off.land.land_complete);
    REQUIRE_FALSE(taking_off.land.internal_error_flow_of_control);
    REQUIRE(taking_off.land.land_detector_count == 12);
}

TEST_CASE("update_land_and_crash_detectors standby_active zeros land_detector_count",
          "[copter][update_land_and_crash_detectors]") {
    UpdateLandAndCrashDetectorsInputs in{};
    in.land.armed = true;
    in.land.land_complete = false;
    in.land.standby_active = true;
    in.land.land_detector_count = 9;

    const auto fx = update_land_and_crash_detectors(in);
    REQUIRE(fx.land.land_detector_count == 0);
    REQUIRE_FALSE(fx.land.land_complete);
    REQUIRE_FALSE(fx.land.internal_error_flow_of_control);
}

TEST_CASE("update_land_and_crash_detectors accel_ef z has gravity added",
          "[copter][update_land_and_crash_detectors]") {
    REQUIRE(kGravityMss == 9.80665f);

    UpdateLandAndCrashDetectorsInputs in{};
    in.accel_ef_mss = fwcpp::math::Vector3f{1.0f, 2.0f, -kGravityMss};
    in.land.armed = false;

    const auto fx = update_land_and_crash_detectors(in);
    REQUIRE(fx.accel_ef_mss.x == 1.0f);
    REQUIRE(fx.accel_ef_mss.y == 2.0f);
    REQUIRE(fx.accel_ef_mss.z == 0.0f);
    REQUIRE(fx.filter_apply);
}

TEST_CASE("update_rangefinder_terrain_offset down LPF mutates terrain_u_m",
          "[copter][update_rangefinder_terrain_offset]") {
    // alpha = 0.1 / max(0.2, 0.1) = 0.5
    // down target = 10 - 2 = 8; new = 4 + (8 - 4) * 0.5 = 6
    UpdateRangefinderTerrainOffsetInputs in{};
    in.g_dt = 0.1f;
    in.surftrak_tc = 0.2f;
    in.rangefinder_state.ref_pos_u_m = 10.0f;
    in.rangefinder_state.alt_glitch_protected_m = 2.0f;
    in.rangefinder_state.terrain_u_m = 4.0f;
    in.rangefinder_state.alt_healthy = false;
    in.rangefinder_state.data_stale = false;

    const auto fx = update_rangefinder_terrain_offset(in);
    REQUIRE(in.rangefinder_state.terrain_u_m == 6.0f);
    REQUIRE_FALSE(fx.wp_nav_set_rangefinder_terrain);
    REQUIRE_FALSE(fx.circle_nav_set_rangefinder_terrain);

    const auto* row = find_scheduler_task("update_rangefinder_terrain_offset");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kFast);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("update_rangefinder_terrain_offset up LPF uses plus glitch",
          "[copter][update_rangefinder_terrain_offset]") {
    // alpha = 0.1 / max(0.2, 0.1) = 0.5
    // up target = 10 + 2 = 12; new = 4 + (12 - 4) * 0.5 = 8
    UpdateRangefinderTerrainOffsetInputs in{};
    in.g_dt = 0.1f;
    in.surftrak_tc = 0.2f;
    in.rangefinder_up_state.ref_pos_u_m = 10.0f;
    in.rangefinder_up_state.alt_glitch_protected_m = 2.0f;
    in.rangefinder_up_state.terrain_u_m = 4.0f;

    (void)update_rangefinder_terrain_offset(in);
    REQUIRE(in.rangefinder_up_state.terrain_u_m == 8.0f);
}

TEST_CASE("update_rangefinder_terrain_offset unhealthy and not stale skips wp_nav",
          "[copter][update_rangefinder_terrain_offset]") {
    UpdateRangefinderTerrainOffsetInputs in{};
    in.g_dt = 0.1f;
    in.surftrak_tc = 0.2f;
    in.rangefinder_state.ref_pos_u_m = 10.0f;
    in.rangefinder_state.alt_glitch_protected_m = 2.0f;
    in.rangefinder_state.terrain_u_m = 4.0f;
    in.rangefinder_state.enabled = true;
    in.rangefinder_state.alt_healthy = false;
    in.rangefinder_state.data_stale = false;

    const auto fx = update_rangefinder_terrain_offset(in);
    REQUIRE(in.rangefinder_state.terrain_u_m == 6.0f);
    REQUIRE_FALSE(fx.wp_nav_set_rangefinder_terrain);
    REQUIRE_FALSE(fx.circle_nav_set_rangefinder_terrain);
}

TEST_CASE("update_rangefinder_terrain_offset healthy or stale calls wp_nav",
          "[copter][update_rangefinder_terrain_offset]") {
    UpdateRangefinderTerrainOffsetInputs in{};
    in.g_dt = 0.1f;
    in.surftrak_tc = 0.2f;
    in.rangefinder_state.ref_pos_u_m = 10.0f;
    in.rangefinder_state.alt_glitch_protected_m = 2.0f;
    in.rangefinder_state.terrain_u_m = 4.0f;
    in.rangefinder_state.enabled = true;
    in.rangefinder_state.alt_healthy = true;
    in.rangefinder_state.data_stale = false;

    const auto healthy = update_rangefinder_terrain_offset(in);
    REQUIRE(healthy.wp_nav_set_rangefinder_terrain);
    REQUIRE(healthy.wp_nav_enabled);
    REQUIRE(healthy.wp_nav_alt_healthy);
    REQUIRE(healthy.wp_nav_terrain_u_m == 6.0f);
    REQUIRE(in.rangefinder_state.terrain_u_m == 6.0f);
    REQUIRE_FALSE(healthy.circle_nav_set_rangefinder_terrain);

    in.rangefinder_state.terrain_u_m = 4.0f;
    in.rangefinder_state.enabled = false;
    in.rangefinder_state.alt_healthy = false;
    in.rangefinder_state.data_stale = true;
    const auto stale = update_rangefinder_terrain_offset(in);
    REQUIRE(stale.wp_nav_set_rangefinder_terrain);
    REQUIRE_FALSE(stale.wp_nav_enabled);
    REQUIRE_FALSE(stale.wp_nav_alt_healthy);
    REQUIRE(stale.wp_nav_terrain_u_m == 6.0f);
    REQUIRE_FALSE(stale.circle_nav_set_rangefinder_terrain);
}

TEST_CASE("update_batt_compass always battery_read", "[copter][update_batt_compass]") {
    UpdateBattCompassInputs available{};
    available.compass_available = true;
    available.throttle = 0.4f;
    available.voltage = 12.6f;
    REQUIRE(update_batt_compass(available).battery_read);

    UpdateBattCompassInputs unavailable{};
    unavailable.compass_available = false;
    unavailable.throttle = 0.4f;
    unavailable.voltage = 12.6f;
    REQUIRE(update_batt_compass(unavailable).battery_read);

    const auto* row = find_scheduler_task("update_batt_compass");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kScheduled);
    REQUIRE(row->rate_hz == 10.0f);
    REQUIRE(row->max_time_micros == 120);
    REQUIRE(row->priority == 15);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("update_batt_compass compass available forwards throttle voltage and reads",
          "[copter][update_batt_compass]") {
    UpdateBattCompassInputs in{};
    in.compass_available = true;
    in.throttle = 0.55f;
    in.voltage = 16.8f;

    const auto fx = update_batt_compass(in);
    REQUIRE(fx.battery_read);
    REQUIRE(fx.set_throttle);
    REQUIRE(fx.throttle == 0.55f);
    REQUIRE(fx.set_voltage);
    REQUIRE(fx.voltage == 16.8f);
    REQUIRE(fx.compass_read);
}

TEST_CASE("update_batt_compass compass unavailable skips compass path",
          "[copter][update_batt_compass]") {
    UpdateBattCompassInputs in{};
    in.compass_available = false;
    in.throttle = 0.55f;
    in.voltage = 16.8f;

    const auto fx = update_batt_compass(in);
    REQUIRE(fx.battery_read);
    REQUIRE_FALSE(fx.set_throttle);
    REQUIRE(fx.throttle == 0.0f);
    REQUIRE_FALSE(fx.set_voltage);
    REQUIRE(fx.voltage == 0.0f);
    REQUIRE_FALSE(fx.compass_read);
}

TEST_CASE("update_altitude always read_barometer", "[copter][update_altitude]") {
    const auto empty = update_altitude({});
    REQUIRE(empty.read_barometer);
    REQUIRE(empty.baro_alt_m == 0.0f);

    UpdateAltitudeInputs injected{};
    injected.baro_alt_m = 12.5f;
    const auto stored = update_altitude(injected);
    REQUIRE(stored.read_barometer);
    REQUIRE(stored.baro_alt_m == 12.5f);

    const auto* row = find_scheduler_task("update_altitude");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kScheduled);
    REQUIRE(row->rate_hz == 10.0f);
    REQUIRE(row->max_time_micros == 100);
    REQUIRE(row->priority == 42);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("update_altitude logging not invoked by default", "[copter][update_altitude]") {
    const auto fx = update_altitude({});
    REQUIRE(fx.read_barometer);
    REQUIRE_FALSE(fx.should_log_ctun);
    REQUIRE_FALSE(fx.log_write_control_tuning);
    REQUIRE_FALSE(fx.write_notch_log_messages);
    REQUIRE_FALSE(fx.gyro_fft_write_log_messages);
}

TEST_CASE("update_altitude should_log_ctun leftover records without writing logs",
          "[copter][update_altitude]") {
    UpdateAltitudeInputs in{};
    in.baro_alt_m = 3.0f;
    in.should_log_ctun = true;

    const auto fx = update_altitude(in);
    REQUIRE(fx.read_barometer);
    REQUIRE(fx.baro_alt_m == 3.0f);
    REQUIRE(fx.should_log_ctun);
    REQUIRE_FALSE(fx.log_write_control_tuning);
    REQUIRE_FALSE(fx.write_notch_log_messages);
    REQUIRE_FALSE(fx.gyro_fft_write_log_messages);
}

TEST_CASE("run_nav_updates records update_super_simple_bearing(false)",
          "[copter][run_nav_updates]") {
    const auto fx = run_nav_updates();
    REQUIRE(fx.update_super_simple_bearing);
    REQUIRE_FALSE(fx.force_update);

    const auto* row = find_scheduler_task("run_nav_updates");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kScheduled);
    REQUIRE(row->rate_hz == 50.0f);
    REQUIRE(row->max_time_micros == 100);
    REQUIRE(row->priority == 45);
    REQUIRE(row->gate == nullptr);
}

namespace {

[[nodiscard]] UpdateThrottleHoverInputs level_hover_inputs() {
    UpdateThrottleHoverInputs in{};
    in.armed = true;
    in.velocity_D_ok = true;
    in.throttle = 0.45f;
    return in;
}

void require_early_return(const UpdateThrottleHoverInputs& in) {
    const auto fx = update_throttle_hover(in);
    REQUIRE(fx.early_return);
    REQUIRE_FALSE(fx.motors_update_throttle_hover);
    REQUIRE(fx.hover_dt == 0.0f);
    REQUIRE_FALSE(fx.gyro_fft_update_freq_hover);
}

}  // namespace

TEST_CASE("update_throttle_hover early-returns when not armed",
          "[copter][update_throttle_hover]") {
    auto in = level_hover_inputs();
    in.armed = false;
    require_early_return(in);
}

TEST_CASE("update_throttle_hover early-returns when land_complete",
          "[copter][update_throttle_hover]") {
    auto in = level_hover_inputs();
    in.land_complete = true;
    require_early_return(in);
}

TEST_CASE("update_throttle_hover early-returns when standby_active",
          "[copter][update_throttle_hover]") {
    auto in = level_hover_inputs();
    in.standby_active = true;
    require_early_return(in);
}

TEST_CASE("update_throttle_hover early-returns with manual throttle",
          "[copter][update_throttle_hover]") {
    auto in = level_hover_inputs();
    in.has_manual_throttle = true;
    require_early_return(in);
}

TEST_CASE("update_throttle_hover early-returns in Drift",
          "[copter][update_throttle_hover]") {
    REQUIRE(Mode::Number::DRIFT == static_cast<Mode::Number>(11));
    auto in = level_hover_inputs();
    in.mode_is_drift = true;
    require_early_return(in);
}

TEST_CASE("update_throttle_hover early-returns when climbing or descending",
          "[copter][update_throttle_hover]") {
    auto in = level_hover_inputs();
    in.vel_desired_U_ms = 0.2f;
    require_early_return(in);
}

TEST_CASE("update_throttle_hover early-returns without velocity_D",
          "[copter][update_throttle_hover]") {
    auto in = level_hover_inputs();
    in.velocity_D_ok = false;
    require_early_return(in);
}

TEST_CASE("update_throttle_hover records motors update in level hover",
          "[copter][update_throttle_hover]") {
    const auto fx = update_throttle_hover(level_hover_inputs());
    REQUIRE_FALSE(fx.early_return);
    REQUIRE(fx.motors_update_throttle_hover);
    REQUIRE(fx.hover_dt == 0.01f);
    REQUIRE_FALSE(fx.gyro_fft_update_freq_hover);

    auto trimmed = level_hover_inputs();
    trimmed.roll_rad = fwcpp::math::radians(4.0f);
    trimmed.roll_trim_rad = fwcpp::math::radians(4.0f);
    trimmed.pitch_rad = fwcpp::math::radians(4.0f);
    const auto trim_fx = update_throttle_hover(trimmed);
    REQUIRE_FALSE(trim_fx.early_return);
    REQUIRE(trim_fx.motors_update_throttle_hover);
    REQUIRE(trim_fx.hover_dt == 0.01f);

    const auto* row = find_scheduler_task("update_throttle_hover");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kScheduled);
    REQUIRE(row->rate_hz == 100.0f);
    REQUIRE(row->max_time_micros == 90);
    REQUIRE(row->priority == 48);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("update_throttle_hover skips motors update when not level hover",
          "[copter][update_throttle_hover]") {
    auto zero_thr = level_hover_inputs();
    zero_thr.throttle = 0.0f;
    const auto zero_fx = update_throttle_hover(zero_thr);
    REQUIRE_FALSE(zero_fx.early_return);
    REQUIRE_FALSE(zero_fx.motors_update_throttle_hover);
    REQUIRE(zero_fx.hover_dt == 0.0f);
    REQUIRE_FALSE(zero_fx.gyro_fft_update_freq_hover);

    auto fast_d = level_hover_inputs();
    fast_d.vel_d_ms = 0.6f;
    const auto d_fx = update_throttle_hover(fast_d);
    REQUIRE_FALSE(d_fx.early_return);
    REQUIRE_FALSE(d_fx.motors_update_throttle_hover);

    auto rolled = level_hover_inputs();
    rolled.roll_rad = fwcpp::math::radians(5.0f);
    const auto roll_fx = update_throttle_hover(rolled);
    REQUIRE_FALSE(roll_fx.early_return);
    REQUIRE_FALSE(roll_fx.motors_update_throttle_hover);

    auto pitched = level_hover_inputs();
    pitched.pitch_rad = fwcpp::math::radians(5.0f);
    const auto pitch_fx = update_throttle_hover(pitched);
    REQUIRE_FALSE(pitch_fx.early_return);
    REQUIRE_FALSE(pitch_fx.motors_update_throttle_hover);
}

TEST_CASE("three_hz_loop leftover always-on failsafes and low_alt; tuning remaining",
          "[copter][three_hz_loop]") {
    const auto leftover = three_hz_loop();
    REQUIRE(leftover.failsafe_gcs_check);
    REQUIRE(leftover.failsafe_terrain_check);
    REQUIRE(leftover.failsafe_deadreckon_check);
    REQUIRE(leftover.low_alt_avoidance);
    REQUIRE_FALSE(leftover.transmitter_tuning);

    const auto* row = find_scheduler_task("three_hz_loop");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kScheduled);
    REQUIRE(row->rate_hz == 3.0f);
    REQUIRE(row->max_time_micros == 75);
    REQUIRE(row->priority == 57);
    REQUIRE(row->gate == nullptr);
}

TEST_CASE("loop_rate_logging leftover records SPOL always; attitude/rate/PIDS/IMU gated",
          "[copter][loop_rate_logging]") {
    const auto empty = loop_rate_logging();
    REQUIRE(empty.log_write_spol);
    REQUIRE_FALSE(empty.log_write_attitude);
    REQUIRE_FALSE(empty.log_write_rate);
    REQUIRE_FALSE(empty.log_write_pids);
    REQUIRE_FALSE(empty.write_imu);
    REQUIRE_FALSE(empty.write_notch_log_messages);
    REQUIRE_FALSE(empty.should_log_ftn_fast);

    LoopRateLoggingInputs att{};
    att.should_log_attitude_fast = true;
    const auto att_fx = loop_rate_logging(att);
    REQUIRE(att_fx.log_write_attitude);
    REQUIRE(att_fx.log_write_rate);
    REQUIRE(att_fx.log_write_pids);
    REQUIRE(att_fx.log_write_spol);
    REQUIRE_FALSE(att_fx.write_imu);
    REQUIRE_FALSE(att_fx.write_notch_log_messages);

    LoopRateLoggingInputs logs_att = att;
    logs_att.logs_attitude = true;
    const auto skip = loop_rate_logging(logs_att);
    REQUIRE_FALSE(skip.log_write_attitude);
    REQUIRE_FALSE(skip.log_write_rate);
    REQUIRE_FALSE(skip.log_write_pids);
    REQUIRE(skip.log_write_spol);

    LoopRateLoggingInputs rate_thr = att;
    rate_thr.using_rate_thread = true;
    const auto rate_fx = loop_rate_logging(rate_thr);
    REQUIRE(rate_fx.log_write_attitude);
    REQUIRE_FALSE(rate_fx.log_write_rate);
    REQUIRE_FALSE(rate_fx.log_write_pids);
    REQUIRE(rate_fx.log_write_spol);

    LoopRateLoggingInputs imu{};
    imu.should_log_imu_fast = true;
    const auto imu_fx = loop_rate_logging(imu);
    REQUIRE(imu_fx.write_imu);
    REQUIRE(imu_fx.log_write_spol);
    REQUIRE_FALSE(imu_fx.log_write_attitude);

    LoopRateLoggingInputs ftn{};
    ftn.should_log_ftn_fast = true;
    const auto ftn_fx = loop_rate_logging(ftn);
    REQUIRE(ftn_fx.should_log_ftn_fast);
    REQUIRE_FALSE(ftn_fx.write_notch_log_messages);
    REQUIRE(ftn_fx.log_write_spol);

    const auto* row = find_scheduler_task("loop_rate_logging");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kScheduled);
    REQUIRE(row->rate_hz == kLoopRateHz);
    REQUIRE(row->max_time_micros == 50);
    REQUIRE(row->priority == 75);
    REQUIRE(row->gate != nullptr);
    REQUIRE(std::string_view(row->gate) == "HAL_LOGGING_ENABLED");
}

TEST_CASE("ten_hz_logging_loop leftover always Write_Attitude; other flags gated",
          "[copter][ten_hz_logging_loop]") {
    const auto empty = ten_hz_logging_loop();
    REQUIRE(empty.write_attitude);
    REQUIRE(empty.log_write_pids);
    REQUIRE(empty.log_write_ekf_pos);
    REQUIRE_FALSE(empty.log_write_attitude);
    REQUIRE_FALSE(empty.log_write_rate);
    REQUIRE_FALSE(empty.motors_log_write);
    REQUIRE_FALSE(empty.write_rcin);
    REQUIRE_FALSE(empty.write_rssi);
    REQUIRE_FALSE(empty.write_rcout);
    REQUIRE_FALSE(empty.pos_control_write_log);
    REQUIRE_FALSE(empty.write_vibration);
    REQUIRE_FALSE(empty.proximity_log);
    REQUIRE_FALSE(empty.beacon_log);
    REQUIRE_FALSE(empty.winch_write_log);
    REQUIRE_FALSE(empty.camera_mount_write_log);

    TenHzLoggingLoopInputs med{};
    med.should_log_attitude_med = true;
    const auto med_fx = ten_hz_logging_loop(med);
    REQUIRE(med_fx.write_attitude);
    REQUIRE(med_fx.log_write_attitude);
    REQUIRE(med_fx.log_write_rate);
    REQUIRE(med_fx.log_write_pids);
    REQUIRE(med_fx.log_write_ekf_pos);

    TenHzLoggingLoopInputs fast = med;
    fast.should_log_attitude_fast = true;
    const auto fast_fx = ten_hz_logging_loop(fast);
    REQUIRE(fast_fx.write_attitude);
    REQUIRE_FALSE(fast_fx.log_write_attitude);
    REQUIRE_FALSE(fast_fx.log_write_rate);
    REQUIRE_FALSE(fast_fx.log_write_pids);
    REQUIRE_FALSE(fast_fx.log_write_ekf_pos);

    TenHzLoggingLoopInputs logs = med;
    logs.logs_attitude = true;
    const auto logs_fx = ten_hz_logging_loop(logs);
    REQUIRE(logs_fx.write_attitude);
    REQUIRE_FALSE(logs_fx.log_write_attitude);
    REQUIRE_FALSE(logs_fx.log_write_rate);
    REQUIRE_FALSE(logs_fx.log_write_pids);
    REQUIRE(logs_fx.log_write_ekf_pos);

    TenHzLoggingLoopInputs rate = med;
    rate.using_rate_thread = true;
    const auto rate_fx = ten_hz_logging_loop(rate);
    REQUIRE(rate_fx.write_attitude);
    REQUIRE(rate_fx.log_write_attitude);
    REQUIRE_FALSE(rate_fx.log_write_rate);
    REQUIRE_FALSE(rate_fx.log_write_pids);
    REQUIRE(rate_fx.log_write_ekf_pos);

    TenHzLoggingLoopInputs mot{};
    mot.should_log_motbatt = true;
    const auto mot_fx = ten_hz_logging_loop(mot);
    REQUIRE(mot_fx.write_attitude);
    REQUIRE(mot_fx.motors_log_write);

    TenHzLoggingLoopInputs rcin{};
    rcin.should_log_rcin = true;
    const auto rcin_fx = ten_hz_logging_loop(rcin);
    REQUIRE(rcin_fx.write_rcin);
    REQUIRE_FALSE(rcin_fx.write_rssi);

    TenHzLoggingLoopInputs rcout{};
    rcout.should_log_rcout = true;
    const auto rcout_fx = ten_hz_logging_loop(rcout);
    REQUIRE(rcout_fx.write_rcout);

    TenHzLoggingLoopInputs ntun_pos{};
    ntun_pos.should_log_ntun = true;
    ntun_pos.has_manual_throttle = true;
    ntun_pos.requires_position = true;
    REQUIRE(ten_hz_logging_loop(ntun_pos).pos_control_write_log);

    TenHzLoggingLoopInputs ntun_land{};
    ntun_land.should_log_ntun = true;
    ntun_land.has_manual_throttle = true;
    ntun_land.landing_with_gps = true;
    REQUIRE(ten_hz_logging_loop(ntun_land).pos_control_write_log);

    TenHzLoggingLoopInputs ntun_auto{};
    ntun_auto.should_log_ntun = true;
    REQUIRE(ten_hz_logging_loop(ntun_auto).pos_control_write_log);

    TenHzLoggingLoopInputs ntun_manual{};
    ntun_manual.should_log_ntun = true;
    ntun_manual.has_manual_throttle = true;
    REQUIRE_FALSE(ten_hz_logging_loop(ntun_manual).pos_control_write_log);

    TenHzLoggingLoopInputs imu{};
    imu.should_log_imu = true;
    REQUIRE(ten_hz_logging_loop(imu).write_vibration);
    TenHzLoggingLoopInputs imu_fast{};
    imu_fast.should_log_imu_fast = true;
    REQUIRE(ten_hz_logging_loop(imu_fast).write_vibration);
    TenHzLoggingLoopInputs imu_raw{};
    imu_raw.should_log_imu_raw = true;
    REQUIRE(ten_hz_logging_loop(imu_raw).write_vibration);

    TenHzLoggingLoopInputs leftover{};
    leftover.should_log_ctun = true;
    leftover.should_log_any = true;
    leftover.should_log_camera = true;
    leftover.should_log_rcin = true;
    const auto rem = ten_hz_logging_loop(leftover);
    REQUIRE(rem.should_log_ctun);
    REQUIRE(rem.should_log_any);
    REQUIRE(rem.should_log_camera);
    REQUIRE(rem.write_rcin);
    REQUIRE_FALSE(rem.write_rssi);
    REQUIRE_FALSE(rem.proximity_log);
    REQUIRE_FALSE(rem.beacon_log);
    REQUIRE_FALSE(rem.winch_write_log);
    REQUIRE_FALSE(rem.camera_mount_write_log);

    const auto* row = find_scheduler_task("ten_hz_logging_loop");
    REQUIRE(row != nullptr);
    REQUIRE(row->kind == TaskKind::kScheduled);
    REQUIRE(row->rate_hz == 10.0f);
    REQUIRE(row->max_time_micros == 350);
    REQUIRE(row->priority == 114);
    REQUIRE(row->gate != nullptr);
    REQUIRE(std::string_view(row->gate) == "HAL_LOGGING_ENABLED");
}
