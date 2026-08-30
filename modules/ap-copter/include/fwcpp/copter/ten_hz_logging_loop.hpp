#pragma once

// Copter::ten_hz_logging_loop leftover. Upstream Copter.cpp ~658-720
// (HAL_LOGGING_ENABLED). Inject should_log / mode / rate-thread bits.
// No logger / INS / motors / ahrs objects — record flags only; do not
// write logs.
//
// ALWAYS ahrs.Write_Attitude (unconditional first line).
//
// if should_log(MASK_LOG_ATTITUDE_MED) && !ATT_FAST && !logs_attitude():
//   Log_Write_Attitude();
//   if (!using_rate_thread) { Log_Write_Rate(); }
//
// if !ATT_FAST && !logs_attitude() && !using_rate_thread:
//   Log_Write_PIDS();
//
// if !ATT_FAST: Log_Write_EKF_POS();
//
// motors->Log_Write() if should_log(MASK_LOG_MOTBATT). HELI_FRAME
// always-write is out of scope — do not force true for heli.
//
// RCIN / RCOUT / NTUN / IMU vibration as upstream. AP_RSSI_ENABLED,
// HAL_PROXIMITY_ENABLED, AP_BEACON_ENABLED, AP_WINCH_ENABLED,
// HAL_MOUNT_ENABLED stay remaining: write flags stay false.

namespace fwcpp::copter {

struct TenHzLoggingLoopInputs {
    bool should_log_attitude_med{false};
    bool should_log_attitude_fast{false};
    bool logs_attitude{false};
    bool using_rate_thread{false};
    bool should_log_motbatt{false};
    bool should_log_rcin{false};
    bool should_log_rcout{false};
    bool should_log_ntun{false};
    bool requires_position{false};
    bool landing_with_gps{false};
    bool has_manual_throttle{false};
    bool should_log_imu{false};
    bool should_log_imu_fast{false};
    bool should_log_imu_raw{false};
    bool should_log_ctun{false};    // leftover: record, no proximity/beacon
    bool should_log_any{false};     // leftover: record, no winch
    bool should_log_camera{false};  // leftover: record, no mount
};

struct TenHzLoggingLoopEffects {
    bool write_attitude{false};          // ahrs.Write_Attitude — always
    bool log_write_attitude{false};
    bool log_write_rate{false};
    bool log_write_pids{false};
    bool log_write_ekf_pos{false};
    bool motors_log_write{false};
    bool write_rcin{false};
    bool write_rssi{false};              // remaining (AP_RSSI_ENABLED)
    bool write_rcout{false};
    bool pos_control_write_log{false};
    bool write_vibration{false};
    bool should_log_ctun{false};         // leftover recorded
    bool proximity_log{false};           // remaining
    bool beacon_log{false};              // remaining
    bool should_log_any{false};          // leftover recorded
    bool winch_write_log{false};         // remaining
    bool should_log_camera{false};       // leftover recorded
    bool camera_mount_write_log{false};  // remaining
};

[[nodiscard]] inline TenHzLoggingLoopEffects ten_hz_logging_loop(
    const TenHzLoggingLoopInputs& in = {}) {
    TenHzLoggingLoopEffects fx{};

    fx.write_attitude = true;

    if (in.should_log_attitude_med && !in.should_log_attitude_fast && !in.logs_attitude) {
        fx.log_write_attitude = true;
        if (!in.using_rate_thread) {
            fx.log_write_rate = true;
        }
    }

    if (!in.should_log_attitude_fast && !in.logs_attitude && !in.using_rate_thread) {
        fx.log_write_pids = true;
    }

    if (!in.should_log_attitude_fast) {
        fx.log_write_ekf_pos = true;
    }

    if (in.should_log_motbatt) {
        fx.motors_log_write = true;
    }

    if (in.should_log_rcin) {
        fx.write_rcin = true;
    }

    if (in.should_log_rcout) {
        fx.write_rcout = true;
    }

    if (in.should_log_ntun &&
        (in.requires_position || in.landing_with_gps || !in.has_manual_throttle)) {
        fx.pos_control_write_log = true;
    }

    if (in.should_log_imu || in.should_log_imu_fast || in.should_log_imu_raw) {
        fx.write_vibration = true;
    }

    fx.should_log_ctun = in.should_log_ctun;
    fx.should_log_any = in.should_log_any;
    fx.should_log_camera = in.should_log_camera;
    return fx;
}

}  // namespace fwcpp::copter
