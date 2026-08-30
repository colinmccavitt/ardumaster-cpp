#pragma once

// Copter::one_hz_loop leftover. Upstream Copter.cpp ~779-835. Inject
// armed, land_complete, should_log_any, using_rate_thread. No motors /
// logger / notify / scheduler objects — record flags only; do not port
// callee bodies.
//
// ALWAYS: AP::srv().enable_aux_servos(); AP_Notify::flags.flying =
// !ap.land_complete.
//
// if !motors->armed(): update_using_interlock, set_frame_class_and_type,
// update_throttle_range (FRAME_CONFIG != HELI — this port is not heli,
// so always when !armed).
//
// if should_log(MASK_LOG_ANY): record Log_Write_Data AP_STATE leftover
// true. Do not port Copter::ap_value() — own remaining catalog row;
// Log_Write_Data payload stays remaining.
//
// Remaining false this slice (do not invent objects): terrain_logging,
// HAL_ADSB_ENABLED adsb.set_is_flying, custom_control.set_notch_sample_rate,
// AP_INERTIALSENSOR_FAST_SAMPLE_WINDOW_ENABLED rate thread.
//
// if !using_rate_thread: attitude_control->set_notch_sample_rate leftover.
// Always: pos_control->D_get_accel_pid().set_notch_sample_rate leftover.

namespace fwcpp::copter {

struct OneHzLoopInputs {
    bool armed{false};
    bool land_complete{false};
    bool should_log_any{false};
    bool using_rate_thread{false};
};

struct OneHzLoopEffects {
    bool log_write_ap_state{false};
    bool update_using_interlock{false};
    bool set_frame_class_and_type{false};
    bool update_throttle_range{false};
    bool enable_aux_servos{false};
    bool terrain_logging{false};                         // remaining
    bool adsb_set_is_flying{false};                      // remaining HAL_ADSB_ENABLED
    bool notify_flying{false};                           // AP_Notify::flags.flying = !land_complete
    bool attitude_control_set_notch_sample_rate{false};
    bool pos_control_d_accel_pid_set_notch_sample_rate{false};
    bool custom_control_set_notch_sample_rate{false};    // remaining
    bool started_rate_thread{false};                     // remaining FAST_SAMPLE_WINDOW
};

[[nodiscard]] inline OneHzLoopEffects one_hz_loop(const OneHzLoopInputs& in = {}) {
    OneHzLoopEffects fx{};

    if (in.should_log_any) {
        fx.log_write_ap_state = true;
    }

    if (!in.armed) {
        fx.update_using_interlock = true;
        fx.set_frame_class_and_type = true;
        fx.update_throttle_range = true;
    }

    fx.enable_aux_servos = true;
    fx.notify_flying = !in.land_complete;

    if (!in.using_rate_thread) {
        fx.attitude_control_set_notch_sample_rate = true;
    }
    fx.pos_control_d_accel_pid_set_notch_sample_rate = true;

    return fx;
}

}  // namespace fwcpp::copter
