#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <string_view>

#include <fwcpp/copter/copter.hpp>

using fwcpp::copter::ModeSwitchReadInputs;
using fwcpp::copter::ModeSwitchReadLeftover;
using fwcpp::copter::MotorsOutputInputs;
using fwcpp::copter::MotorsOutputMainLeftover;
using fwcpp::copter::PortStatus;
using fwcpp::copter::TaskKind;
using fwcpp::copter::completeness_has;
using fwcpp::copter::copter_completeness_size;
using fwcpp::copter::find_scheduler_task;
using fwcpp::copter::first_scheduled_always_on;
using fwcpp::copter::get_scheduler_tasks;
using fwcpp::copter::kArmingDelayMs;
using fwcpp::copter::kCopterLoopRateHz;
using fwcpp::copter::kMaskLogPm;
using fwcpp::copter::kRcLoopMaxTimeMicros;
using fwcpp::copter::kRcLoopPriority;
using fwcpp::copter::kRcLoopRateHz;
using fwcpp::copter::kSchedulerTasks;
using fwcpp::copter::motors_output;
using fwcpp::copter::motors_output_main;
using fwcpp::copter::on_main_count;
using fwcpp::copter::out_of_scope_count;
using fwcpp::copter::rc_loop;
using fwcpp::copter::read_ahrs;
using fwcpp::copter::read_mode_switch;
using fwcpp::copter::remaining_count;
using fwcpp::copter::scheduler_task_count;
using fwcpp::copter::this_slice_count;
using fwcpp::copter::throttle_loop;

TEST_CASE("catalog remaining_count stays open after slice 3", "[copter][leftover]") {
    REQUIRE(remaining_count() == 29);
    REQUIRE(this_slice_count() >= 1);
    REQUIRE(on_main_count() == 6);
    REQUIRE(copter_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("Copter::rc_loop", PortStatus::kOnMain));
    REQUIRE(completeness_has("RC_Channels::read_mode_switch", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::scheduler_tasks[]", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::get_scheduler_tasks", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::motors_output / motors_output_main", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::read_AHRS", PortStatus::kOnMain));
    REQUIRE(completeness_has("Copter::throttle_loop", PortStatus::kThisSlice));
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
