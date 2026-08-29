#pragma once

// Copter::run_rate_controller_main leftover. Upstream
// ArduCopter/Attitude.cpp. No motors / pos_control / attitude_control
// objects — inject last_loop_time_s and using_rate_thread.
//
// pos_control and attitude_control always take the loop dt. motors
// set_dt_s and rate_controller_run run only on the main thread.
// rate_controller_target_reset always runs (sysid / temporary inputs).

namespace fwcpp::copter {

struct RateControllerMainInputs {
    float last_loop_time_s{0};
    bool using_rate_thread{false};
};

struct RateControllerMainEffects {
    float last_loop_time_s{0};
    bool pos_control_set_dt_s{true};
    bool attitude_control_set_dt_s{true};
    bool motors_set_dt_s{false};
    bool rate_controller_run{false};
    bool rate_controller_target_reset{true};
};

[[nodiscard]] inline constexpr RateControllerMainEffects run_rate_controller_main(
    const RateControllerMainInputs& in) {
    const bool run_on_main = !in.using_rate_thread;
    return RateControllerMainEffects{
        .last_loop_time_s = in.last_loop_time_s,
        .pos_control_set_dt_s = true,
        .attitude_control_set_dt_s = true,
        .motors_set_dt_s = run_on_main,
        .rate_controller_run = run_on_main,
        .rate_controller_target_reset = true,
    };
}

}  // namespace fwcpp::copter
