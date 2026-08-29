#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <string_view>

#include <fwcpp/copter/copter.hpp>

using fwcpp::copter::ModeSwitchReadInputs;
using fwcpp::copter::ModeSwitchReadLeftover;
using fwcpp::copter::PortStatus;
using fwcpp::copter::TaskKind;
using fwcpp::copter::completeness_has;
using fwcpp::copter::copter_completeness_size;
using fwcpp::copter::find_scheduler_task;
using fwcpp::copter::first_scheduled_always_on;
using fwcpp::copter::get_scheduler_tasks;
using fwcpp::copter::kCopterLoopRateHz;
using fwcpp::copter::kMaskLogPm;
using fwcpp::copter::kRcLoopMaxTimeMicros;
using fwcpp::copter::kRcLoopPriority;
using fwcpp::copter::kRcLoopRateHz;
using fwcpp::copter::kSchedulerTasks;
using fwcpp::copter::on_main_count;
using fwcpp::copter::out_of_scope_count;
using fwcpp::copter::rc_loop;
using fwcpp::copter::read_mode_switch;
using fwcpp::copter::remaining_count;
using fwcpp::copter::scheduler_task_count;
using fwcpp::copter::this_slice_count;

TEST_CASE("catalog remaining_count stays open after slice 1", "[copter][leftover]") {
    REQUIRE(remaining_count() >= 1);
    REQUIRE(this_slice_count() >= 1);
    REQUIRE(on_main_count() == 0);
    REQUIRE(copter_completeness_size() ==
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count());
    REQUIRE(completeness_has("Copter::rc_loop", PortStatus::kThisSlice));
    REQUIRE(completeness_has("RC_Channels::read_mode_switch", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Copter::scheduler_tasks[]", PortStatus::kThisSlice));
    REQUIRE(completeness_has("Copter::motors_output / motors_output_main", PortStatus::kRemaining));
    REQUIRE(completeness_has("Copter::read_AHRS", PortStatus::kRemaining));
    REQUIRE(completeness_has("Copter::throttle_loop", PortStatus::kRemaining));
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
