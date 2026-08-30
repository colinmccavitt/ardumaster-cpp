#pragma once

// Copter::twentyfive_hz_logging leftover. Upstream Copter.cpp ~722-738
// (HAL_LOGGING_ENABLED). Inject should_log bits. No logger / INS /
// gyro_fft objects — record flags only; do not write logs.
//
// if should_log(MASK_LOG_ATTITUDE_FAST): Log_Write_EKF_POS();
//
// if should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_FAST):
//   AP::ins().Write_IMU();
//
// HAL_GYROFFT_ENABLED gyro_fft.write_log_messages stays remaining:
// gyro_fft_write_log_messages=false always this slice (even if
// should_log_ftn_fast is injected).

namespace fwcpp::copter {

struct TwentyfiveHzLoggingInputs {
    bool should_log_attitude_fast{false};
    bool should_log_imu{false};
    bool should_log_imu_fast{false};
    bool should_log_ftn_fast{false};  // leftover: record, do not write gyro_fft
};

struct TwentyfiveHzLoggingEffects {
    bool log_write_ekf_pos{false};
    bool write_imu{false};
    bool should_log_ftn_fast{false};            // leftover recorded, no gyro_fft write
    bool gyro_fft_write_log_messages{false};    // remaining
};

[[nodiscard]] inline TwentyfiveHzLoggingEffects twentyfive_hz_logging(
    const TwentyfiveHzLoggingInputs& in = {}) {
    TwentyfiveHzLoggingEffects fx{};
    fx.should_log_ftn_fast = in.should_log_ftn_fast;

    if (in.should_log_attitude_fast) {
        fx.log_write_ekf_pos = true;
    }

    if (in.should_log_imu && !in.should_log_imu_fast) {
        fx.write_imu = true;
    }

    return fx;
}

}  // namespace fwcpp::copter
