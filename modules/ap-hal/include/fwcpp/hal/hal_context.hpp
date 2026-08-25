#pragma once

// Explicit HAL context bundling this port's SITL-subset AP_HAL surface
// into one caller-owned object, replacing upstream's hal.* singleton
// access (AP_HAL::get_HAL()). CPP-025, final slice.
//
// Every member here was ported independently against SITL's own real
// backend (see each header's own file banner for the upstream source
// it was matched against) - this struct's only job is to bundle them so
// a vehicle skeleton has ONE thing to construct and pass by reference,
// matching this port's standing no-singleton convention (ADR-0012)
// already used throughout (AltitudeContext, L1Inputs, FilterRegistry).
//
// Deliberately excluded:
//   - A wall-clock time source (millis()/micros()). This port never
//     centralizes one - every module that needs "now" takes it as an
//     explicit now_ms/now_us parameter instead (AcPid, L1Control,
//     Scheduler's own TimeSource callable), so there is nothing for
//     HalContext to own here; a caller supplies real or simulated time
//     at the call site.
//   - Multiple UART instances (upstream's serial0..serial9). Only one
//     is wired in - this slice's "minimal UARTDriver for future
//     MAVLink/console" scope (see CPP-025's ticket text) doesn't yet
//     have a caller needing more than one; adding the rest is trivial
//     (another UartDriver<> member) once a real caller exists, not
//     designed in speculatively now.
//   - Scheduler's task tables and last_run array: Scheduler itself
//     (tick counters, loop rate/period) is genuinely HAL-context-shaped
//     state, but task tables are vehicle-specific data owned by the
//     vehicle skeleton, passed into scheduler.run() as spans - bundling
//     them here would mean HalContext owning vehicle-skeleton state,
//     which is exactly the kind of hidden-coupling this port's explicit-
//     context convention exists to avoid.

#include <cstdint>

#include <fwcpp/hal/analog_in.hpp>
#include <fwcpp/hal/rc_input.hpp>
#include <fwcpp/hal/rc_output.hpp>
#include <fwcpp/hal/uart_driver.hpp>
#include <fwcpp/param/storage.hpp>
#include <fwcpp/scheduler/scheduler.hpp>

namespace fwcpp::hal {

class HalContext {
public:
    explicit HalContext(std::uint16_t loop_rate_hz) : scheduler(loop_rate_hz) {}

    RcInput rc_input;
    RcOutput rc_output;
    AnalogIn analog_in;
    UartDriver<> console;
    storage::RawStorage storage;
    scheduler::Scheduler scheduler;
};

} // namespace fwcpp::hal
