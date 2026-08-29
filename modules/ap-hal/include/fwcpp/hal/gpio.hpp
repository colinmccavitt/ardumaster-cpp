#pragma once

// Port of AP_HAL/GPIO.h's digital pin contract, matched against what
// AP_HAL_SITL/GPIO.cpp actually does rather than the full generic
// multi-backend HAL interface. CPP-088 slice 1.
//
// SITL's GPIO is an in-memory 16-bit pin_mask on SITL_State (valid_pin
// is `pin < 16`). pinMode only tracks a write-enable bitmask for pins
// 0-7; pins 8-15 always accept write(). write() on a pin 0-7 that is
// not write-enabled is ignored (upstream comment: "ignore setting of
// pull-up resistors"). read()/write() encode levels as uint8_t 0/1;
// any non-zero write value is treated as high.
//
// analogPinToDigital is not present on AP_HAL::GPIO or HALSITL::GPIO
// (Plane-4.7.0) and is not invented here.
//
// Deliberately not a virtual AP_HAL::GPIO base (ADR-0012, no RTTI /
// no singleton manager). channel() is omitted: SITL heap-allocates a
// DigitalSource* and this port forbids flight-path allocation.
// Weight-on-wheels (wow_pin vs simulated altitude) is a SITL_State
// sensor hook, not a GPIO primitive, and is left for a sim consumer.

#include <array>
#include <cstdint>

namespace fwcpp::hal {

// Upstream HAL_GPIO_INPUT / HAL_GPIO_OUTPUT / HAL_GPIO_ALT.
enum class PinMode : std::uint8_t {
    kInput = 0,
    kOutput = 1,
    kAlt = 2,
};

// HALSITL::GPIO::valid_pin: pin < 16 (sizeof(pin_mask) * 8).
inline constexpr std::uint8_t kGpioPinCount = 16;

// SITL pinMode only updates the write-enable mask for pins 0-7.
inline constexpr std::uint8_t kGpioWriteMaskPinCount = 8;

class Gpio {
public:
    // SITL GPIO::init() is an empty no-op; we only record that it ran
    // so tests can see the same "called once at bring-up" seam.
    void init() { inited_ = true; }

    [[nodiscard]] bool is_initialized() const { return inited_; }

    [[nodiscard]] bool valid_pin(std::uint8_t pin) const { return pin < kGpioPinCount; }

    // Upstream pinMode(pin, output). ALT is non-zero, so SITL treats it
    // as write-enabled the same way as OUTPUT.
    void set_pin_mode(std::uint8_t pin, PinMode mode) {
        if (!valid_pin(pin)) {
            return;
        }
        modes_[pin] = mode;
    }

    [[nodiscard]] PinMode mode_of(std::uint8_t pin) const {
        return valid_pin(pin) ? modes_[pin] : PinMode::kInput;
    }

    [[nodiscard]] std::uint8_t read(std::uint8_t pin) const {
        if (!valid_pin(pin)) {
            return 0;
        }
        return static_cast<std::uint8_t>((pin_mask_ & (1U << pin)) != 0 ? 1 : 0);
    }

    void write(std::uint8_t pin, std::uint8_t value) {
        if (!valid_pin(pin)) {
            return;
        }
        // SITL: pins 0-7 ignore writes unless pinMode set a write bit.
        if (pin < kGpioWriteMaskPinCount && !write_enabled(pin)) {
            return;
        }
        if (value != 0) {
            pin_mask_ = static_cast<std::uint16_t>(pin_mask_ | (1U << pin));
        } else {
            pin_mask_ = static_cast<std::uint16_t>(pin_mask_ & ~(1U << pin));
        }
    }

    void toggle(std::uint8_t pin) { write(pin, static_cast<std::uint8_t>(read(pin) == 0 ? 1 : 0)); }

    // SITL GPIO::usb_connected() always returns false.
    [[nodiscard]] bool usb_connected() const { return false; }

private:
    [[nodiscard]] bool write_enabled(std::uint8_t pin) const {
        const PinMode mode = modes_[pin];
        return mode == PinMode::kOutput || mode == PinMode::kAlt;
    }

    std::array<PinMode, kGpioPinCount> modes_{};
    std::uint16_t pin_mask_ = 0;
    bool inited_ = false;
};

}  // namespace fwcpp::hal
