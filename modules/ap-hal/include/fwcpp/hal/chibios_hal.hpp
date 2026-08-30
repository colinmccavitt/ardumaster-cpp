#pragma once

// Compile-only ChibiOS HalContext-shaped bundle (CPP-089 slice 3).
//
// Upstream HAL_ChibiOS (AP_HAL_ChibiOS/HAL_ChibiOS_Class.h/.cpp)
// constructs a singleton HAL from static ChibiOS::UARTDriver serial0..9,
// GPIO, RCInput, RCOutput, Storage, Scheduler, and Util, then run()
// starts ChibiOS threads (chThdSetPriority / chThdCreateStatic),
// Shared_DMA, FATFS/sdcard, and stm32_watchdog_init (IWDG). This port
// does not reproduce that constructor wiring or those threads
// (ADR-0012: no singletons; inject HalContext). ch.h, hwdef.h, FATFS,
// and DMA are not included.
//
// This type reuses the existing SITL-subset seams behind BoardKind::kChibiOS:
//   UartDriver     — in-memory RX/TX rings (uart_driver.hpp, CPP-025)
//   Gpio           — 16-pin in-memory table (gpio.hpp, CPP-088)
//   RcInput/Output — injected channel values (rc_input.hpp / rc_output.hpp)
//   RawStorage     — in-memory param buffer (param/storage.hpp, CPP-020)
//   Watchdog       — compile-only stub (watchdog.hpp, CPP-089 slice 1)
// Time is an injected now_ms, same as SITL HalContext and LinuxHalContext
// (no ChibiOS chVTGetSystemTimeX / AP_HAL::millis()).
//
// Deliberately not ported (disclose vs upstream ChibiOS):
//   - ChibiOS::UARTDriver SD/SDU serial drivers (hal.h SerialDriver,
//     not this port's in-memory UartDriver)
//   - ChibiOS::GPIO palReadLine / palWriteLine / hwdef pin tables
//   - stm32_watchdog_init / pat IWDG MMIO (watchdog.c IWDGD.KR)
//   - hwdef.dat pin/peripheral generation (hwdef.h)
//   - Scheduler thread / DMA / FATFS (out of this ticket's HAL surface)

#include <cstdint>

#include <fwcpp/hal/board.hpp>
#include <fwcpp/hal/gpio.hpp>
#include <fwcpp/hal/rc_input.hpp>
#include <fwcpp/hal/rc_output.hpp>
#include <fwcpp/hal/uart_driver.hpp>
#include <fwcpp/hal/watchdog.hpp>
#include <fwcpp/param/storage.hpp>

namespace fwcpp::hal {

class ChibiOSHalContext {
public:
    explicit ChibiOSHalContext(std::uint32_t now_ms = 0) : now_ms_(now_ms) {}

    [[nodiscard]] static constexpr BoardKind board_kind() { return BoardKind::kChibiOS; }

    // Injected time. Upstream ChibiOS millis() is chVTGetSystemTimeX;
    // SITL, LinuxHalContext, and this bundle take now_ms from the caller.
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
