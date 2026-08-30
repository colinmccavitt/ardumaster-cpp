#pragma once

// Copter::init_ardupilot leftover. Upstream ArduCopter/system.cpp
// ~16-200 (through ap.initialised = true). ESC cal body remaining
// (delay/read_radio/arming/while(1)). No notify / battery /
// barometer / winch /
// rssi / GCS / OSD / SurfaceTracking / RC_Channel / motors /
// SRV_Channels / BoardConfig / AP_Relay / HAL / GPS / compass /
// airspeed / OA / attitude_control / optflow / camera / precland /
// landinggear / rangefinder / proximity / beacon / AP_Mission /
// SmartRTL / AP_Logger / custom_control / INS objects — record
// leftover flags only. Do not invoke the allocate_motors() or
// startup_INS_ground() helper bodies (call-site leftover flags
// only). Do not dump Log_Write_Vehicle_Startup_Messages body or
// FUNCTOR_BIND.
//
// Always-on this slice (AP_WINCH_ENABLED / AP_RSSI_ENABLED are not
// enabled in this port):
//   notify.init()
//   notify_flight_mode()
//   battery.init()
//   barometer.init()
// winch_init and rssi_init stay false (remaining / not enabled).
//
// GCS/OSD remaining this slice (do not invent objects):
//   gcs().setup_uarts()  — leftover gcs_setup_uarts stays false
//   osd.init()           — leftover osd_init stays false
//
// This slice leftover (non-heli, FRAME_CONFIG != HELI_FRAME):
//   update_using_interlock() — using_interlock = motor_interlock_aux
//     (rc().find_channel_for_option(MOTOR_INTERLOCK) != nullptr)
//   init_rc_in() — bind + set_angle/set_range leftover flags
//   allocate_motors() call site — leftover flag only
//   rc().convert_options(ARMDISARM_UNUSED=41, ARMDISARM_AIRMODE=154)
//   rc().init()
//   init_rc_out() leftover flags — motors->init, enable_aux_servos,
//     set_update_rate, convert_pwm_min_max, update_throttle_range,
//     update_aux_servo_function, safety_ignore_mask. Skip heli
//     set_esc_scaling. Do not invent motors / SRV / BoardConfig.
//   esc_calibration_startup_check() — brushed skip only
//     (motors->is_brushed_pwm_type() early return). Non-brushed ESC
//     cal body stays remaining (delay/read_radio/arming/while(1)).
//   ap.initialised_params = true
//   register_timer_failsafe(failsafe_check_static, 1000) — flag only
//   gps.set_log_gps_bit(MASK_LOG_GPS) + gps.init() leftover flags
//   AP::compass().set_log_bit(MASK_LOG_COMPASS) + init leftover flags
//   attitude_control->parameter_sanity_check() leftover flag only
//   barometer.set_log_baro_bit(MASK_LOG_IMU) leftover flag
//   barometer.calibrate() leftover flag
//   mode_auto.mission.init() leftover flag (MODE_AUTO treated enabled)
//   mode_auto.mission.set_log_start_mission_item_bit(MASK_LOG_CMD)
//     leftover flag + mission_log_bit
//   g2.smart_rtl.init() leftover flag (MODE_SMARTRTL treated enabled)
//   logger.setVehicle_Startup_Writer leftover flag only (no functor)
//   startup_INS_ground() call site — leftover flag only (helper
//     body already kOnMain; do not invoke)
//   set_land_complete(true) leftover flag
//   set_land_complete_maybe(true) leftover flag
//   failsafe_enable() leftover flag
//   ins.set_log_raw_bit(MASK_LOG_IMU_RAW) leftover flag + bit
//   motors->output_min() leftover flag only (no motors object)
//   set_mode leftover flags only — uint8 reasons, no real set_mode
//     / mode.hpp. INITIALISED=26; UNAVAILABLE=33 fallback iff
//     !initial_mode_ok. Inject initial_mode (default 0 STABILIZE).
//   pos_variance_filt.set_cutoff_frequency leftover flag
//   vel_variance_filt.set_cutoff_frequency leftover flag
//   ap.initialised leftover flag
// custom_control.init stays false (AC_CUSTOMCONTROL_MULTI remaining).
//
// surface_tracking.init stays false (AP_RANGEFINDER remaining).
// relay.init stays false (AP_RELAY remaining).
// airspeed.set_log_bit stays false (AP_AIRSPEED remaining).
// g2.oa.init stays false (AP_OAPATHPLANNER remaining).
// optflow.init stays false (AP_OPTICALFLOW remaining).
// camera_mount.init stays false (HAL_MOUNT remaining).
// camera.init stays false (AP_CAMERA remaining).
// init_precland stays false (AC_PRECLAND remaining).
// landinggear.init stays false (AP_LANDINGGEAR remaining).
// USERHOOK_INIT stays false (not implemented).
// init_rangefinder stays false (AP_RANGEFINDER remaining).
// g2.proximity.init stays false (HAL_PROXIMITY remaining).
// g2.beacon.init stays false (AP_BEACON remaining).
// The rest of init_ardupilot is ESC cal body only
// (delay/read_radio/arming/while(1)) — catalog row
// "Copter::init_ardupilot rest".

#include <cstdint>

namespace fwcpp::copter {

// ROLL_PITCH_YAW_INPUT_MAX — ArduCopter/config.h ~470-471
inline constexpr std::int16_t kRollPitchYawInputMax = 4500;

// RC_Channel::AUX_FUNC — libraries/RC_Channel/RC_Channel.h
inline constexpr std::uint16_t kAuxArmdisarmUnused = 41;
inline constexpr std::uint16_t kAuxArmdisarmAirmode = 154;

// Default motor PWM when throttle is not configured — radio.cpp init_rc_out
inline constexpr std::int16_t kDefaultPwmMin = 1000;
inline constexpr std::int16_t kDefaultPwmMax = 2000;

// register_timer_failsafe period — ArduCopter/system.cpp ~87
inline constexpr std::uint16_t kFailsafeCheckPeriodUs = 1000;

// ArduCopter/defines.h MASK_LOG_GPS = (1<<2); MASK_LOG_COMPASS = (1<<13);
// MASK_LOG_IMU = (1<<7); MASK_LOG_CMD = (1<<8);
// MASK_LOG_IMU_RAW = (1UL<<19)
inline constexpr std::uint32_t kMaskLogGps = 1u << 2;
inline constexpr std::uint32_t kMaskLogCompass = 1u << 13;
inline constexpr std::uint32_t kMaskLogImu = 1u << 7;
inline constexpr std::uint32_t kMaskLogCmd = 1u << 8;
inline constexpr std::uint32_t kMaskLogImuRaw = 1u << 19;

// ModeReason leftover uint8 values only — do not include mode.hpp.
// libraries/AP_Vehicle/ModeReason.h INITIALISED=26, UNAVAILABLE=33.
inline constexpr std::uint8_t kModeReasonInitialised = 26;
inline constexpr std::uint8_t kModeReasonUnavailable = 33;

struct InitArdupilotInputs {
    bool motor_interlock_aux{false};
    bool throttle_configured{false};
    std::int16_t radio_min{0};
    std::int16_t radio_max{0};
    std::uint16_t rc_speed{0};
    bool is_brushed_pwm{false};  // motors->is_brushed_pwm_type()
    std::uint8_t initial_mode{0};  // g.initial_mode; Mode::Number::STABILIZE=0
    bool initial_mode_ok{true};    // set_mode(initial_mode, INITIALISED) result
};

struct InitArdupilotEffects {
    bool winch_init{false};           // remaining AP_WINCH_ENABLED
    bool notify_init{false};
    bool notify_flight_mode{false};
    bool battery_init{false};
    bool rssi_init{false};            // remaining AP_RSSI_ENABLED
    bool barometer_init{false};
    bool gcs_setup_uarts{false};      // remaining — do not invent GCS
    bool osd_init{false};             // remaining OSD_ENABLED
    bool using_interlock{false};
    bool roll_bind{false};
    bool pitch_bind{false};
    bool throttle_bind{false};
    bool yaw_bind{false};
    std::int16_t roll_angle{0};
    std::int16_t pitch_angle{0};
    std::int16_t yaw_angle{0};
    std::int16_t throttle_range{0};
    bool rc_tuning{false};            // remaining AP_RC_TRANSMITTER_TUNING
    bool rc_tuning2{false};
    bool default_dead_zones{false};
    bool throttle_zero{false};
    bool surface_tracking_init{false};  // remaining AP_RANGEFINDER
    bool allocate_motors_called{false}; // call site only — no helper body
    bool rc_convert_options{false};     // 41 → 154 leftover flag
    bool rc_init{false};
    bool motors_init{false};
    bool enable_aux_servos{false};
    bool set_update_rate{false};
    std::uint16_t rc_speed{0};
    bool convert_pwm_min_max{false};
    std::int16_t convert_pwm_min{0};
    std::int16_t convert_pwm_max{0};
    bool update_throttle_range{false};
    bool update_aux_servo_function{false};
    bool safety_ignore_mask{false};     // flag only — no BoardConfig / motor_mask
    bool esc_cal_skipped{false};        // brushed early return
    bool esc_cal_body{false};           // remaining — delay/read_radio/arming/while(1)
    bool initialised_params{false};
    bool relay_init{false};             // remaining AP_RELAY
    bool register_timer_failsafe{false};
    std::uint16_t register_timer_failsafe_period{0};
    bool gps_set_log_bit{false};
    std::uint32_t gps_log_bit{0};       // MASK_LOG_GPS
    bool gps_init{false};
    bool compass_set_log_bit{false};
    std::uint32_t compass_log_bit{0};   // MASK_LOG_COMPASS
    bool compass_init{false};
    bool airspeed_set_log_bit{false};   // remaining AP_AIRSPEED
    bool oa_init{false};                // remaining AP_OAPATHPLANNER
    bool attitude_parameter_sanity_check{false};
    bool optflow_init{false};           // remaining AP_OPTICALFLOW
    bool camera_mount_init{false};      // remaining HAL_MOUNT
    bool camera_init{false};            // remaining AP_CAMERA
    bool init_precland{false};          // remaining AC_PRECLAND
    bool landinggear_init{false};       // remaining AP_LANDINGGEAR
    bool userhook_init{false};          // remaining USERHOOK_INIT
    bool barometer_set_log_baro_bit{false};
    std::uint32_t baro_log_bit{0};      // MASK_LOG_IMU
    bool barometer_calibrate{false};
    bool init_rangefinder{false};       // remaining AP_RANGEFINDER
    bool proximity_init{false};         // remaining HAL_PROXIMITY
    bool beacon_init{false};            // remaining AP_BEACON
    bool mission_init{false};
    bool mission_set_log_start_mission_item_bit{false};
    std::uint32_t mission_log_bit{0};   // MASK_LOG_CMD
    bool smart_rtl_init{false};
    bool logger_set_vehicle_startup_writer{false};  // flag only
    bool startup_ins_ground_called{false};  // call site only — no helper body
    bool custom_control_init{false};        // remaining AC_CUSTOMCONTROL_MULTI
    bool set_land_complete{false};
    bool set_land_complete_maybe{false};
    bool failsafe_enable{false};
    bool ins_set_log_raw_bit{false};
    std::uint32_t ins_log_raw_bit{0};  // MASK_LOG_IMU_RAW
    bool motors_output_min{false};     // flag only — no motors object
    bool set_mode_initial{false};      // leftover only — no real set_mode
    std::uint8_t leftover_set_mode_reason{0};  // ModeReason::INITIALISED=26
    bool set_mode_stabilize_unavailable{false};
    std::uint8_t leftover_set_mode_unavailable_reason{0};  // UNAVAILABLE=33
    bool pos_variance_filt_set_cutoff{false};
    bool vel_variance_filt_set_cutoff{false};
    bool ap_initialised{false};
};

[[nodiscard]] inline InitArdupilotEffects init_ardupilot(
    const InitArdupilotInputs& in = {}) {
    InitArdupilotEffects fx{};
    fx.notify_init = true;
    fx.notify_flight_mode = true;
    fx.battery_init = true;
    fx.barometer_init = true;
    // gcs_setup_uarts / osd_init stay false (remaining)
    fx.using_interlock = in.motor_interlock_aux;
    // library guarantees non-nullptr
    fx.roll_bind = true;
    fx.pitch_bind = true;
    fx.throttle_bind = true;
    fx.yaw_bind = true;
    fx.roll_angle = kRollPitchYawInputMax;
    fx.pitch_angle = kRollPitchYawInputMax;
    fx.yaw_angle = kRollPitchYawInputMax;
    fx.throttle_range = 1000;
    // rc_tuning / rc_tuning2 stay false
    fx.default_dead_zones = true;
    fx.throttle_zero = true;
    // surface_tracking_init stays false (AP_RANGEFINDER remaining)
    fx.allocate_motors_called = true;
    fx.rc_convert_options = true;
    fx.rc_init = true;
    // init_rc_out leftover (non-heli). No motors / SRV / BoardConfig objects.
    fx.motors_init = true;
    fx.enable_aux_servos = true;
    fx.set_update_rate = true;
    fx.rc_speed = in.rc_speed;
    fx.convert_pwm_min_max = true;
    if (in.throttle_configured) {
        fx.convert_pwm_min = in.radio_min;
        fx.convert_pwm_max = in.radio_max;
    } else {
        fx.convert_pwm_min = kDefaultPwmMin;
        fx.convert_pwm_max = kDefaultPwmMax;
    }
    fx.update_throttle_range = true;
    fx.update_aux_servo_function = true;
    fx.safety_ignore_mask = true;
    // esc_calibration_startup_check leftover — brushed skip only.
    // esc_cal_body stays false (non-brushed body remaining).
    if (in.is_brushed_pwm) {
        fx.esc_cal_skipped = true;
    }
    fx.initialised_params = true;
    // relay_init stays false (AP_RELAY remaining)
    fx.register_timer_failsafe = true;
    fx.register_timer_failsafe_period = kFailsafeCheckPeriodUs;
    // GPS / compass leftover flags only — no objects.
    fx.gps_set_log_bit = true;
    fx.gps_log_bit = kMaskLogGps;
    fx.gps_init = true;
    fx.compass_set_log_bit = true;
    fx.compass_log_bit = kMaskLogCompass;
    fx.compass_init = true;
    // airspeed_set_log_bit / oa_init stay false (remaining)
    fx.attitude_parameter_sanity_check = true;
    // optflow / camera_mount / camera / precland / landinggear stay false
    // userhook_init stays false (USERHOOK_INIT not implemented)
    fx.barometer_set_log_baro_bit = true;
    fx.baro_log_bit = kMaskLogImu;
    fx.barometer_calibrate = true;
    // init_rangefinder / proximity_init / beacon_init stay false (remaining)
    // MODE_AUTO / MODE_SMARTRTL / HAL_LOGGING treated enabled this port.
    fx.mission_init = true;
    fx.mission_set_log_start_mission_item_bit = true;
    fx.mission_log_bit = kMaskLogCmd;
    fx.smart_rtl_init = true;
    fx.logger_set_vehicle_startup_writer = true;
    // startup_INS_ground call site only — do not invoke helper body.
    fx.startup_ins_ground_called = true;
    // custom_control_init stays false (AC_CUSTOMCONTROL_MULTI remaining)
    fx.set_land_complete = true;
    fx.set_land_complete_maybe = true;
    fx.failsafe_enable = true;
    // Tail after failsafe_enable — leftover flags only. No INS /
    // motors objects, no real set_mode (CCP-036), no ESC cal body.
    fx.ins_set_log_raw_bit = true;
    fx.ins_log_raw_bit = kMaskLogImuRaw;
    fx.motors_output_min = true;
    fx.set_mode_initial = true;
    fx.leftover_set_mode_reason = kModeReasonInitialised;
    if (!in.initial_mode_ok) {
        fx.set_mode_stabilize_unavailable = true;
        fx.leftover_set_mode_unavailable_reason = kModeReasonUnavailable;
    }
    (void)in.initial_mode;
    fx.pos_variance_filt_set_cutoff = true;
    fx.vel_variance_filt_set_cutoff = true;
    fx.ap_initialised = true;
    return fx;
}

}  // namespace fwcpp::copter
