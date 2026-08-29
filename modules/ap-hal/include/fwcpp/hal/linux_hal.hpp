#pragma once

// Compile-only Linux HalContext-shaped bundle (CPP-089 slice 2).
//
// Upstream HAL_Linux (AP_HAL_Linux/HAL_Linux_Class.h/.cpp) is a
// CONFIG_HAL_BOARD_SUBTYPE switch that constructs a singleton HAL from
// board-specific UARTDriver, GPIO_*, RCInput_*, RCOutput_*, Storage, and
// AP_HAL::millis() (system.cpp clock_gettime(CLOCK_MONOTONIC)). This port
// does not reproduce that preprocessor board matrix or those syscalls
// (ADR-0012: no singletons; inject HalContext).
//
// This type reuses the existing SITL-subset seams behind BoardKind::kLinux:
//   UartDriver     — in-memory RX/TX rings (uart_driver.hpp, CPP-025)
//   Gpio           — 16-pin in-memory table (gpio.hpp, CPP-088)
//   RcInput/Output — injected channel values (rc_input.hpp / rc_output.hpp)
//   RawStorage     — in-memory param buffer (param/storage.hpp, CPP-020)
//   Watchdog       — compile-only stub (watchdog.hpp, CPP-089 slice 1)
// Time is an injected now_ms, same as SITL HalContext (no wall-clock HAL
// source). No /dev/tty, no sysfs GPIO, no clock_gettime.
//
// Deliberately not ported (disclose vs upstream Linux):
//   - Linux::UARTDriver::_begin / UARTDevice termios2 (termios.h,
//     ioctl TIOC* / TCGETS2) and set_device_path("/dev/tty*")
//   - Linux::GPIO_Sysfs export/open of /sys/class/gpio (and GPIO_RPI mmap)
//   - AP_HAL_Linux/system.cpp clock_gettime(CLOCK_MONOTONIC) millis()
//   - Linux::Storage dirty-line flush to an on-disk fd
//   - Board-specific RCInput_PRU / RCOutput_PCA9685 / etc.

#include <cstdint>

#include <fwcpp/hal/board.hpp>
#include <fwcpp/hal/gpio.hpp>
#include <fwcpp/hal/rc_input.hpp>
#include <fwcpp/hal/rc_output.hpp>
#include <fwcpp/hal/uart_driver.hpp>
#include <fwcpp/hal/watchdog.hpp>
#include <fwcpp/param/storage.hpp>

namespace fwcpp::hal {

class LinuxHalContext {
public:
    explicit LinuxHalContext(std::uint32_t now_ms = 0) : now_ms_(now_ms) {}

    [[nodiscard]] static constexpr BoardKind board_kind() { return BoardKind::kLinux; }

    // Injected time. Upstream Linux millis() is clock_gettime; SITL and
    // this bundle take now_ms from the caller instead.
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
