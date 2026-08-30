#pragma once

// Copter::update_altitude leftover. Upstream ArduCopter/Copter.cpp
// ~922-941. Always read_barometer() first (sensors.cpp ~4-8:
// barometer.update(); baro_alt_m = barometer.get_altitude()).
// No barometer / logger / INS / gyro_fft objects — inject baro_alt_m
// and optional should_log_ctun leftover flag.
//
// HAL_LOGGING_ENABLED should_log(MASK_LOG_CTUN) Log_Write_Control_Tuning
// + optional notch / gyro_fft stay remaining: write flags stay false.
// Injected should_log_ctun is recorded only (no log writes).
// run_nav_updates is a later leftover.

namespace fwcpp::copter {

struct UpdateAltitudeInputs {
    float baro_alt_m{0};
    bool should_log_ctun{false};  // leftover: record, do not write logs
};

struct UpdateAltitudeEffects {
    bool read_barometer{false};
    float baro_alt_m{0};
    bool should_log_ctun{false};            // leftover recorded, no Log_Write
    bool log_write_control_tuning{false};   // remaining
    bool write_notch_log_messages{false};   // remaining
    bool gyro_fft_write_log_messages{false};  // remaining
};

[[nodiscard]] inline UpdateAltitudeEffects update_altitude(
    const UpdateAltitudeInputs& in = {}) {
    UpdateAltitudeEffects fx{};
    fx.read_barometer = true;
    fx.baro_alt_m = in.baro_alt_m;
    fx.should_log_ctun = in.should_log_ctun;
    return fx;
}

}  // namespace fwcpp::copter
