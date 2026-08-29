#pragma once

// Copter::scheduler_tasks[] as data. Upstream ArduCopter/Copter.cpp.
// Gated FAST_TASK / SCHED_TASK rows stay in the table with their compile
// gate string so a later slice can turn a #if on without inventing rate
// or priority. ADR-0012: no AP::scheduler singleton.

#include <cstddef>
#include <cstdint>

namespace fwcpp::copter {

inline constexpr float kLoopRateHz = 0.0f;  // upstream LOOP_RATE — every loop
inline constexpr std::uint8_t kFastTaskPri0 = 0;
inline constexpr std::uint16_t kCopterLoopRateHz = 400;
inline constexpr std::uint32_t kMaskLogPm = 1u << 3;

inline constexpr float kRcLoopRateHz = 250.0f;
inline constexpr std::uint16_t kRcLoopMaxTimeMicros = 130;
inline constexpr std::uint8_t kRcLoopPriority = 3;

enum class TaskKind : std::uint8_t {
    kFast = 0,
    kScheduled = 1,
};

struct SchedulerTaskSpec {
    const char* name;
    float rate_hz;
    std::uint16_t max_time_micros;
    std::uint8_t priority;
    TaskKind kind;
    const char* gate;  // nullptr = always compiled
};

struct SchedulerTasksView {
    const SchedulerTaskSpec* tasks;
    std::uint8_t task_count;
    std::uint32_t log_bit;
};

[[nodiscard]] inline constexpr bool name_eq(const char* a, const char* b) {
    while (*a != '\0' && *b != '\0') {
        if (*a != *b) {
            return false;
        }
        ++a;
        ++b;
    }
    return *a == '\0' && *b == '\0';
}

[[nodiscard]] inline constexpr SchedulerTaskSpec fast(const char* name, const char* gate = nullptr) {
    return SchedulerTaskSpec{
        .name = name,
        .rate_hz = kLoopRateHz,
        .max_time_micros = 0,
        .priority = kFastTaskPri0,
        .kind = TaskKind::kFast,
        .gate = gate,
    };
}

[[nodiscard]] inline constexpr SchedulerTaskSpec sched(const char* name, float rate_hz,
                                                      std::uint16_t max_time_micros,
                                                      std::uint8_t priority,
                                                      const char* gate = nullptr) {
    return SchedulerTaskSpec{
        .name = name,
        .rate_hz = rate_hz,
        .max_time_micros = max_time_micros,
        .priority = priority,
        .kind = TaskKind::kScheduled,
        .gate = gate,
    };
}

inline constexpr SchedulerTaskSpec kSchedulerTasks[] = {
    fast("AP_InertialSensor::update"),
    fast("run_rate_controller_main"),
    fast("run_custom_controller", "AC_CUSTOMCONTROL_MULTI_ENABLED"),
    fast("heli_update_autorotation", "HELI_FRAME"),
    fast("motors_output_main"),
    fast("read_AHRS"),
    fast("update_heli_control_dynamics", "HELI_FRAME"),
    fast("read_inertia"),
    fast("check_ekf_reset"),
    fast("update_flight_mode"),
    fast("update_home_from_EKF"),
    fast("update_land_and_crash_detectors"),
    fast("update_rangefinder_terrain_offset"),
    fast("AP_Mount::update_fast", "HAL_MOUNT_ENABLED"),
    fast("Log_Video_Stabilisation", "HAL_LOGGING_ENABLED"),
    sched("rc_loop", kRcLoopRateHz, kRcLoopMaxTimeMicros, kRcLoopPriority),
    sched("throttle_loop", 50.0f, 75, 6),
    sched("fence_check", 25.0f, 100, 7, "AP_FENCE_ENABLED"),
    sched("AP_GPS::update", 50.0f, 200, 9),
    sched("AP_OpticalFlow::update", 200.0f, 160, 12, "AP_OPTICALFLOW_ENABLED"),
    sched("update_batt_compass", 10.0f, 120, 15),
    sched("RC_Channels::read_aux_all", 10.0f, 50, 18),
    sched("ToyMode::update", 10.0f, 50, 24, "TOY_MODE_ENABLED"),
    sched("auto_disarm_check", 10.0f, 50, 27),
    sched("RC_Channels_Copter::auto_trim_run", 10.0f, 75, 30, "AP_COPTER_AHRS_AUTO_TRIM_ENABLED"),
    sched("read_rangefinder", 20.0f, 100, 33, "AP_RANGEFINDER_ENABLED"),
    sched("AP_Proximity::update", 200.0f, 50, 36, "HAL_PROXIMITY_ENABLED"),
    sched("AP_Beacon::update", 400.0f, 50, 39, "AP_BEACON_ENABLED"),
    sched("update_altitude", 10.0f, 100, 42),
    sched("run_nav_updates", 50.0f, 100, 45),
    sched("update_throttle_hover", 100.0f, 90, 48),
    sched("ModeSmartRTL::save_position", 3.0f, 100, 51, "MODE_SMARTRTL_ENABLED"),
    sched("AC_Sprayer::update", 3.0f, 90, 54, "HAL_SPRAYER_ENABLED"),
    sched("three_hz_loop", 3.0f, 75, 57),
    sched("AP_ServoRelayEvents::update_events", 50.0f, 75, 60, "AP_SERVORELAYEVENTS_ENABLED"),
    sched("update_precland", 400.0f, 50, 69, "AC_PRECLAND_ENABLED"),
    sched("check_dynamic_flight", 50.0f, 75, 72, "HELI_FRAME"),
    sched("loop_rate_logging", kLoopRateHz, 50, 75, "HAL_LOGGING_ENABLED"),
    sched("one_hz_loop", 1.0f, 100, 81),
    sched("ekf_check", 10.0f, 75, 84),
    sched("check_vibration", 10.0f, 50, 87),
    sched("gpsglitch_check", 10.0f, 50, 90),
    sched("takeoff_check", 50.0f, 50, 91),
    sched("landinggear_update", 10.0f, 75, 93, "AP_LANDINGGEAR_ENABLED"),
    sched("standby_update", 100.0f, 75, 96),
    sched("lost_vehicle_check", 10.0f, 50, 99),
    sched("GCS::update_receive", 400.0f, 180, 102),
    sched("GCS::update_send", 400.0f, 550, 105),
    sched("AP_Mount::update", 50.0f, 75, 108, "HAL_MOUNT_ENABLED"),
    sched("AP_Camera::update", 50.0f, 75, 111, "AP_CAMERA_ENABLED"),
    sched("ten_hz_logging_loop", 10.0f, 350, 114, "HAL_LOGGING_ENABLED"),
    sched("twentyfive_hz_logging", 25.0f, 110, 117, "HAL_LOGGING_ENABLED"),
    sched("AP_Logger::periodic_tasks", 400.0f, 300, 120, "HAL_LOGGING_ENABLED"),
    sched("AP_InertialSensor::periodic", 400.0f, 50, 123),
    sched("AP_Scheduler::update_logging", 0.1f, 75, 126, "HAL_LOGGING_ENABLED"),
    sched("AP_TempCalibration::update", 10.0f, 100, 135, "AP_TEMPCALIBRATION_ENABLED"),
    sched("avoidance_adsb_update", 10.0f, 100, 138, "HAL_ADSB_ENABLED || AP_ADSB_AVOIDANCE_ENABLED"),
    sched("afs_fs_check", 10.0f, 100, 141, "AP_COPTER_ADVANCED_FAILSAFE_ENABLED"),
    sched("terrain_update", 10.0f, 100, 144, "AP_TERRAIN_AVAILABLE"),
    sched("AP_Winch::update", 50.0f, 50, 150, "AP_WINCH_ENABLED"),
    sched("userhook_FastLoop", 100.0f, 75, 153, "USERHOOK_FASTLOOP"),
    sched("userhook_50Hz", 50.0f, 75, 156, "USERHOOK_50HZLOOP"),
    sched("userhook_MediumLoop", 10.0f, 75, 159, "USERHOOK_MEDIUMLOOP"),
    sched("userhook_SlowLoop", 3.3f, 75, 162, "USERHOOK_SLOWLOOP"),
    sched("userhook_SuperSlowLoop", 1.0f, 75, 165, "USERHOOK_SUPERSLOWLOOP"),
    sched("AP_Button::update", 5.0f, 100, 168, "HAL_BUTTON_ENABLED"),
    sched("update_dynamic_notch_at_specified_rate_main", kLoopRateHz, 200, 215,
          "AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED"),
};

[[nodiscard]] inline constexpr std::size_t scheduler_task_count() {
    return sizeof(kSchedulerTasks) / sizeof(kSchedulerTasks[0]);
}

[[nodiscard]] inline constexpr SchedulerTasksView get_scheduler_tasks() {
    return SchedulerTasksView{
        .tasks = kSchedulerTasks,
        .task_count = static_cast<std::uint8_t>(scheduler_task_count()),
        .log_bit = kMaskLogPm,
    };
}

[[nodiscard]] inline constexpr const SchedulerTaskSpec* find_scheduler_task(const char* name) {
    for (const auto& task : kSchedulerTasks) {
        if (name_eq(task.name, name)) {
            return &task;
        }
    }
    return nullptr;
}

// First SCHED_TASK row that a stock multicopter always compiles.
[[nodiscard]] inline constexpr const SchedulerTaskSpec* first_scheduled_always_on() {
    for (const auto& task : kSchedulerTasks) {
        if (task.kind == TaskKind::kScheduled && task.gate == nullptr) {
            return &task;
        }
    }
    return nullptr;
}

}  // namespace fwcpp::copter
