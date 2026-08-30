#pragma once

// Copter::loop_rate_logging leftover. Upstream Copter.cpp ~637-656
// (HAL_LOGGING_ENABLED). Inject should_log / mode / rate-thread bits.
// No logger / INS / motors objects — record flags only; do not write
// logs.
//
// if should_log(MASK_LOG_ATTITUDE_FAST) && !flightmode->logs_attitude():
//   Log_Write_Attitude();
//   if (!using_rate_thread) { Log_Write_Rate(); Log_Write_PIDS(); }
//
// AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED write_notch_log_messages
// stays remaining: write_notch_log_messages=false always this slice
// (even if should_log_ftn_fast injected).
//
// if should_log(MASK_LOG_IMU_FAST): record Write_IMU.
// ALWAYS record motors Log_Write_SPOL.

namespace fwcpp::copter {

struct LoopRateLoggingInputs {
    bool should_log_attitude_fast{false};
    bool logs_attitude{false};
    bool using_rate_thread{false};
    bool should_log_ftn_fast{false};  // leftover: record, do not write notch
    bool should_log_imu_fast{false};
};

struct LoopRateLoggingEffects {
    bool should_log_ftn_fast{false};          // leftover recorded, no notch write
    bool log_write_attitude{false};
    bool log_write_rate{false};
    bool log_write_pids{false};
    bool write_notch_log_messages{false};     // remaining
    bool write_imu{false};
    bool log_write_spol{false};
};

[[nodiscard]] inline LoopRateLoggingEffects loop_rate_logging(
    const LoopRateLoggingInputs& in = {}) {
    LoopRateLoggingEffects fx{};
    fx.should_log_ftn_fast = in.should_log_ftn_fast;

    if (in.should_log_attitude_fast && !in.logs_attitude) {
        fx.log_write_attitude = true;
        if (!in.using_rate_thread) {
            fx.log_write_rate = true;
            fx.log_write_pids = true;
        }
    }

    if (in.should_log_imu_fast) {
        fx.write_imu = true;
    }

    fx.log_write_spol = true;
    return fx;
}

}  // namespace fwcpp::copter
