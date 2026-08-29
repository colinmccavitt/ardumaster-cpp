#pragma once

// Copter::motors_output / motors_output_main leftover. Upstream
// ArduCopter/motors.cpp. No motors / SRV / GCS objects — inject flags.
//
// AFS crash that is not a landing terminate returns before PWM / interlock
// / drive. A landing terminate must keep walking the output path.
// Interlock is computed from the *cleared* arming-delay flag so THROW
// and the 2.0 s timeout can spool on the same tick.

#include <cstdint>

namespace fwcpp::copter {

// ArduCopter/config.h ARMING_DELAY_SEC (stock 2.0f). Compared as
// millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f — strictly greater.
inline constexpr float kArmingDelaySec = 2.0f;
inline constexpr std::uint32_t kArmingDelayMs =
    static_cast<std::uint32_t>(kArmingDelaySec * 1.0e3f);

struct MotorsOutputInputs {
    bool using_rate_thread{false};
    bool afs_should_crash{false};
    bool afs_terminating_via_landing{false};
    bool in_arming_delay{false};
    bool motors_armed{false};
    std::uint32_t now_ms{0};
    std::uint32_t arm_time_ms{0};
    bool mode_is_throw{false};
    bool using_interlock{false};
    bool motor_interlock_switch{false};
    bool emergency_stop{false};
    bool motors_interlock{false};
    bool motor_test{false};
};

struct MotorsOutputEffects {
    bool skip_output{false};
    bool clear_arming_delay{false};
    bool calc_pwm{false};
    bool cork{false};
    bool output_ch_all{false};
    bool set_interlock_true{false};
    bool set_interlock_false{false};
    bool log_interlock_enabled{false};
    bool log_interlock_disabled{false};
    bool motor_test_output{false};
    bool output_to_motors{false};
    bool push_srv{false};
    bool push_rcout{false};
};

enum class MotorsOutputMainLeftover : std::uint8_t {
    kSkipped = 0,
    kRan = 1,
};

struct MotorsOutputMainEffects {
    MotorsOutputMainLeftover leftover{MotorsOutputMainLeftover::kSkipped};
    MotorsOutputEffects output{};
};

[[nodiscard]] inline constexpr bool arming_delay_elapsed_ms(std::uint32_t now_ms,
                                                            std::uint32_t arm_time_ms) {
    return (now_ms - arm_time_ms) > kArmingDelayMs;
}

[[nodiscard]] inline constexpr MotorsOutputEffects motors_output(const MotorsOutputInputs& in,
                                                                bool full_push = true) {
    if (in.afs_should_crash && !in.afs_terminating_via_landing) {
        return MotorsOutputEffects{.skip_output = true};
    }

    const bool clear_arming_delay =
        in.in_arming_delay &&
        (!in.motors_armed || arming_delay_elapsed_ms(in.now_ms, in.arm_time_ms) ||
         in.mode_is_throw);
    const bool in_arming_delay = in.in_arming_delay && !clear_arming_delay;

    const bool interlock = in.motors_armed && !in_arming_delay &&
                           (!in.using_interlock || in.motor_interlock_switch) &&
                           !in.emergency_stop;
    const bool set_true = !in.motors_interlock && interlock;
    const bool set_false = in.motors_interlock && !interlock;

    return MotorsOutputEffects{
        .skip_output = false,
        .clear_arming_delay = clear_arming_delay,
        .calc_pwm = true,
        .cork = true,
        .output_ch_all = true,
        .set_interlock_true = set_true,
        .set_interlock_false = set_false,
        .log_interlock_enabled = set_true,
        .log_interlock_disabled = set_false,
        .motor_test_output = in.motor_test,
        .output_to_motors = !in.motor_test,
        .push_srv = full_push,
        .push_rcout = !full_push,
    };
}

// motors_output_main: rate thread owns the FAST_TASK body. Default
// full_push is forced on; only the rate thread passes false.
[[nodiscard]] inline constexpr MotorsOutputMainEffects motors_output_main(
    const MotorsOutputInputs& in) {
    if (in.using_rate_thread) {
        return MotorsOutputMainEffects{
            .leftover = MotorsOutputMainLeftover::kSkipped,
            .output = {},
        };
    }
    return MotorsOutputMainEffects{
        .leftover = MotorsOutputMainLeftover::kRan,
        .output = motors_output(in, true),
    };
}

}  // namespace fwcpp::copter
