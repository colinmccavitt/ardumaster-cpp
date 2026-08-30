#pragma once

// Compile-only ESP32 HalContext-shaped bundle (CPP-089 slice 4).
//
// Upstream HAL_ESP32 (AP_HAL_ESP32/HAL_ESP32_Class.h/.cpp) constructs a
// singleton HAL from static ESP32::UARTDriver cons/serial2/serial3,
// ESP32::WiFiDriver (or WiFiUdpDriver) serial1, GPIO, RCInput,
// RCOutput, Storage, Scheduler, and Util, then run() starts ESP32
// tasks via Scheduler::init(). This port does not reproduce that
// constructor wiring or those FreeRTOS tasks (ADR-0012: no singletons;
// inject HalContext). ESP-IDF headers, uart_driver_install, and
// WiFiDriver are not included.
//
// This type reuses the existing SITL-subset seams behind BoardKind::kEsp32:
//   UartDriver     — in-memory RX/TX rings (uart_driver.hpp, CPP-025)
//   Gpio           — 16-pin in-memory table (gpio.hpp, CPP-088)
//   RcInput/Output — injected channel values (rc_input.hpp / rc_output.hpp)
//   RawStorage     — in-memory param buffer (param/storage.hpp, CPP-020)
//   Watchdog       — compile-only stub (watchdog.hpp, CPP-089 slice 1)
// Time is an injected now_ms, same as SITL HalContext, LinuxHalContext,
// and ChibiOSHalContext (no esp_timer_get_time / AP_HAL::millis()).
//
// Deliberately not ported (disclose vs upstream ESP32):
//   - ESP32::UARTDriver uart_param_config / uart_set_pin /
//     uart_driver_install (ESP-IDF driver/uart.h, not this port's
//     in-memory UartDriver)
//   - ESP32::WiFiDriver / WiFiUdpDriver TCP/UDP AP (192.168.4.1:5760)
//   - ESP32::GPIO gpio_set_direction / gpio_get_level (driver/gpio.h)
//   - AP_HAL_ESP32/system.cpp esp_timer_get_time millis()
//   - ESP32::Storage NVS / SdCard flush
//   - Scheduler FreeRTOS xTaskCreate / run() task start

#include <cstdint>

#include <fwcpp/hal/board.hpp>
#include <fwcpp/hal/gpio.hpp>
#include <fwcpp/hal/rc_input.hpp>
#include <fwcpp/hal/rc_output.hpp>
#include <fwcpp/hal/uart_driver.hpp>
#include <fwcpp/hal/watchdog.hpp>
#include <fwcpp/param/storage.hpp>

namespace fwcpp::hal {

class ESP32HalContext {
public:
    explicit ESP32HalContext(std::uint32_t now_ms = 0) : now_ms_(now_ms) {}

    [[nodiscard]] static constexpr BoardKind board_kind() { return BoardKind::kEsp32; }

    // Injected time. Upstream ESP32 millis() is esp_timer_get_time;
    // SITL, LinuxHalContext, ChibiOSHalContext, and this bundle take
    // now_ms from the caller.
    void set_now_ms(std::uint32_t now_ms) { now_ms_ = now_ms; }
    [[nodiscard]] std::uint32_t now_ms() const { return now_ms_; }

    UartDriver<> uart;
    Gpio gpio;
    RcInput rc_input;
    RcOutput rc_output;
    storage::RawStorage storage;
    Watchdog watchdog;

private:
    std::uint32_t now_ms_ = 0;
};

}  // namespace fwcpp::hal
