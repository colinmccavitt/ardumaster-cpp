#pragma once

// Copter::startup_INS_ground leftover. Upstream ArduCopter/system.cpp
// ~207-218. No AHRS / INS / scheduler objects — record the four
// calls in order as leftover effects:
//   1. ahrs.init()
//   2. ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER)
//   3. ins.init(scheduler.get_loop_rate_hz())
//   4. ahrs.reset()
//
// VehicleClass matches AP_AHRS.h ~711-717 (UNKNOWN=0, GROUND=1,
// COPTER=2, FIXED_WING=3, SUBMARINE=4). Local leftover enum — do
// not pull in a fake AHRS object. Always COPTER: that is what
// Copter::startup_INS_ground does.
//
// loop_rate_hz default 400 is Copter MAIN_LOOP_RATE /
// kCopterLoopRateHz. Always run all four steps (upstream has no
// gates).
//
// Do not port Copter::allocate_motors or Copter::init_ardupilot.

#include <cstdint>

namespace fwcpp::copter {

enum class VehicleClass : std::uint8_t {
    UNKNOWN = 0,
    GROUND = 1,
    COPTER = 2,
    FIXED_WING = 3,
    SUBMARINE = 4,
};

struct StartupInsGroundInputs {
    std::uint16_t loop_rate_hz{400};
};

struct StartupInsGroundEffects {
    bool ahrs_init{false};
    VehicleClass vehicle_class{VehicleClass::UNKNOWN};
    bool ins_init{false};
    std::uint16_t ins_loop_rate_hz{0};
    bool ahrs_reset{false};
};

[[nodiscard]] inline StartupInsGroundEffects startup_ins_ground(
    const StartupInsGroundInputs& in = {}) {
    StartupInsGroundEffects fx{};
    fx.ahrs_init = true;
    fx.vehicle_class = VehicleClass::COPTER;
    fx.ins_init = true;
    fx.ins_loop_rate_hz = in.loop_rate_hz;
    fx.ahrs_reset = true;
    return fx;
}

}  // namespace fwcpp::copter
