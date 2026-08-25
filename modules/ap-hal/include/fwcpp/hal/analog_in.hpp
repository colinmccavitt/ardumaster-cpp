#pragma once

// Port of AP_HAL/AnalogIn.h's observable contract (board_voltage,
// servorail_voltage, per-pin analog reads), matched against what
// AP_HAL_SITL/AnalogIn.cpp actually does rather than the full generic
// multi-backend HAL interface. CPP-025 (AnalogIn slice).
//
// SITL's own AnalogIn (AP_HAL_SITL/AnalogIn.cpp) is NOT a real ADC
// driver: channel(pin) heap-allocates (NEW_NOTHROW) a tiny ADCSource
// object whose only state is the pin number; every read dispatches
// straight into a handful of plain floats sitting on the shared
// SITL_State struct (sonar_pin_voltage, airspeed_pin_voltage[2],
// current_pin_voltage, voltage_pin_voltage, current2_pin_voltage,
// voltage2_pin_voltage), each filled in once per frame by SITL's own
// sensor simulations (rangefinder, airspeed, battery - none of which
// this port has built yet). There is no real ADC, no averaging, and no
// per-channel ownership worth modelling as a separate object:
// read_average()==read_latest() and voltage_average()==voltage_latest()
// in the real backend (SITL never accumulates samples), and the
// AnalogIn/AnalogSource split exists upstream only so hardware backends
// can hold per-channel driver state and to satisfy the abstract
// multi-backend interface - neither applies here. So this port
// flattens the two classes into one plain value holder (matching
// RcInput/RcOutput's own pattern) addressed directly by pin number, and
// avoids the per-channel dynamic allocation entirely (this port
// disallows dynamic allocation - see house rules).
//
// Per-pin dispatch table reproduced exactly from
// ADCSource::voltage_latest() (AP_HAL_SITL/AnalogIn.cpp): pin numbers
// are fixed instrument roles, not a generic addressable array -
//   0  -> sonar_pin_voltage       (rangefinder)
//   1  -> airspeed_pin_voltage[0]
//   2  -> airspeed_pin_voltage[1]
//   12 -> current_pin_voltage     (battery monitor, primary)
//   13 -> voltage_pin_voltage     (battery monitor, primary)
//   14 -> current2_pin_voltage    (battery monitor, secondary)
//   15 -> voltage2_pin_voltage    (battery monitor, secondary)
//   ANALOG_INPUT_BOARD_VCC (254) -> SITL_ADC_MAX_PIN_VALUE, a raw 16-bit
//       ADC count (65535), NOT a voltage - this is a genuine quirk in
//       upstream SITL (voltage_latest() is documented everywhere else
//       as returning a 0-5V voltage; this one case returns the ADC
//       full-scale count instead, in the 0-5V-labelled return slot).
//       Reproduced faithfully since board_voltage() below is the
//       correct/intended way to read board VCC and no real caller is
//       expected to go through channel(ANALOG_INPUT_BOARD_VCC) - but
//       flagged here per this port's divergence-tracking convention so
//       nobody "fixes" it thinking it's this port's own bug.
//   ANALOG_INPUT_NONE (255) or any other pin -> 0.0f (matches
//       upstream's switch-default).
//
// read_latest()/read_average() reproduce ADCSource's
// VOLTAGE_TO_PIN_VALUE() digitisation: voltage_latest() rescaled to a
// 16-bit (0-65535) raw ADC count assuming a 5.0V full-scale reference,
// clamped to range - this is real, meaningful SITL behavior (what a
// "raw ADC read" caller sees, as opposed to a "give me volts" caller),
// not a hardware-only concern, so it is reproduced rather than dropped.
//
// Deliberately left out of scope (SITL's own AnalogIn never overrides
// them - they are pure hardware/board-config concerns with zero
// simulated behavior to port):
//   - servorail_voltage(): SITL doesn't override the base class's
//     `return 0` - there is no simulated servo rail model. Reproduced
//     here as a fixed 0.0f for the same reason, not because this port
//     forgot to implement it.
//   - power_status_flags()/accumulated_power_status_flags(): same -
//     base-class `return 0`, no SITL override, no MAV_POWER_STATUS
//     simulation exists.
//   - valid_analog_pin(): base-class `return false`, no SITL override.
//   - mcu_temperature()/mcu_voltage()/mcu_voltage_max()/mcu_voltage_min():
//     gated behind HAL_WITH_MCU_MONITORING upstream and never
//     implemented by SITL's AnalogIn at all.
//   - set_pin()/channel() as a stateful per-channel handle: see above -
//     flattened away, callers pass the pin number directly instead.
//   - init(): SITL's AnalogIn::init() is an empty no-op.

#include <algorithm>
#include <array>
#include <cstdint>

namespace fwcpp::hal {

// Mirrors AP_HAL_SITL/AnalogIn.h's SITL_ADC_* constants.
inline constexpr std::uint16_t kAnalogAdcMaxPinValue = 65535; // 16-bit resolution, (1<<16)-1
inline constexpr float kAnalogFullScaleVoltage = 5.0f;

// Mirrors AP_HAL/AnalogIn.h's ANALOG_INPUT_* sentinels.
inline constexpr std::uint8_t kAnalogInputBoardVcc = 254;
inline constexpr std::uint8_t kAnalogInputNone = 255;

// Fixed instrument-role pin numbers, matching ADCSource::voltage_latest()'s
// switch statement exactly (see file banner).
inline constexpr std::uint8_t kAnalogPinSonar = 0;
inline constexpr std::uint8_t kAnalogPinAirspeed0 = 1;
inline constexpr std::uint8_t kAnalogPinAirspeed1 = 2;
inline constexpr std::uint8_t kAnalogPinCurrent = 12;
inline constexpr std::uint8_t kAnalogPinVoltage = 13;
inline constexpr std::uint8_t kAnalogPinCurrent2 = 14;
inline constexpr std::uint8_t kAnalogPinVoltage2 = 15;

class AnalogIn {
public:
    // Board 5V rail voltage in volts. SITL hardcodes this rather than
    // simulating a rail sensor (AnalogIn::board_voltage() override is
    // always `return 5.0f`) - no injection point exists upstream, so
    // none is offered here either.
    [[nodiscard]] float board_voltage() const { return kAnalogFullScaleVoltage; }

    // Servo rail voltage in volts. Always 0: SITL never overrides the
    // base class's default (see file banner) - there is no simulated
    // servo rail model to port.
    [[nodiscard]] float servorail_voltage() const { return 0.0f; }

    // --- Per-pin instrument injection, mirroring the SITL_State fields
    // ADCSource::voltage_latest() reads (see file banner for the pin
    // table). A test harness, or a future sensor-simulation port, sets
    // these the same way it would poke SITL_State directly.
    void set_sonar_pin_voltage(float volts) { sonar_pin_voltage_ = volts; }

    void set_airspeed_pin_voltage(std::uint8_t sensor_index, float volts) {
        if (sensor_index < airspeed_pin_voltage_.size()) {
            airspeed_pin_voltage_[sensor_index] = volts;
        }
    }

    void set_current_pin_voltage(float volts) { current_pin_voltage_ = volts; }
    void set_voltage_pin_voltage(float volts) { voltage_pin_voltage_ = volts; }
    void set_current2_pin_voltage(float volts) { current2_pin_voltage_ = volts; }
    void set_voltage2_pin_voltage(float volts) { voltage2_pin_voltage_ = volts; }

    // Simulated voltage on a given "pin", matching
    // ADCSource::voltage_latest() exactly (including the
    // ANALOG_INPUT_BOARD_VCC quirk - see file banner).
    [[nodiscard]] float voltage_latest(std::uint8_t pin) const {
        switch (pin) {
            case kAnalogInputBoardVcc:
                return static_cast<float>(kAnalogAdcMaxPinValue);
            case kAnalogPinSonar:
                return sonar_pin_voltage_;
            case kAnalogPinAirspeed0:
                return airspeed_pin_voltage_[0];
            case kAnalogPinAirspeed1:
                return airspeed_pin_voltage_[1];
            case kAnalogPinCurrent:
                return current_pin_voltage_;
            case kAnalogPinVoltage:
                return voltage_pin_voltage_;
            case kAnalogPinCurrent2:
                return current2_pin_voltage_;
            case kAnalogPinVoltage2:
                return voltage2_pin_voltage_;
            default:
                return 0.0f;
        }
    }

    // SITL never distinguishes "latest sample" from "averaged samples"
    // (no accumulation happens) - both read the same value, matching
    // ADCSource::read_average()/voltage_average() delegating straight
    // to their _latest() counterparts.
    [[nodiscard]] float voltage_average(std::uint8_t pin) const { return voltage_latest(pin); }

    // Matches ADCSource::voltage_average_ratiometric(), which is just
    // voltage_average() again - SITL has no separate ratiometric model.
    [[nodiscard]] float voltage_average_ratiometric(std::uint8_t pin) const {
        return voltage_average(pin);
    }

    // Raw ADC count a caller would see from read_latest()/read_average():
    // voltage_latest() rescaled from [0, 5V] to a 16-bit count and
    // clamped, matching ADCSource's VOLTAGE_TO_PIN_VALUE() macro exactly.
    [[nodiscard]] float read_latest(std::uint8_t pin) const {
        const float scaled =
            voltage_latest(pin) * (static_cast<float>(kAnalogAdcMaxPinValue) / kAnalogFullScaleVoltage);
        return std::clamp(scaled, 0.0f, static_cast<float>(kAnalogAdcMaxPinValue));
    }

    [[nodiscard]] float read_average(std::uint8_t pin) const { return read_latest(pin); }

private:
    float sonar_pin_voltage_ = 0.0f;
    std::array<float, 2> airspeed_pin_voltage_{};
    float current_pin_voltage_ = 0.0f;
    float voltage_pin_voltage_ = 0.0f;
    float current2_pin_voltage_ = 0.0f;
    float voltage2_pin_voltage_ = 0.0f;
};

} // namespace fwcpp::hal
