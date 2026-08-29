// Tests for fwcpp::hal::HalContext (CPP-025 final slice).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/hal_context.hpp>

using namespace fwcpp::hal;

TEST_CASE("HalContext bundles independently-default-constructed peripherals", "[hal_context]") {
    HalContext hal(400);

    REQUIRE(hal.rc_input.read(0) == 0);
    REQUIRE(hal.rc_output.read(0) == 0);
    REQUIRE(hal.analog_in.board_voltage() == 5.0f);
    REQUIRE(hal.console.available() == 0);
    REQUIRE(hal.storage.size() == 16384);
    REQUIRE(hal.scheduler.loop_rate_hz() == 400);
    REQUIRE(hal.gpio.valid_pin(0));
    REQUIRE(hal.gpio.read(0) == 0);
    REQUIRE(hal.semaphore.depth() == 0);
}

TEST_CASE("HalContext's members are independently usable through the bundle", "[hal_context]") {
    HalContext hal(50);

    hal.rc_input.set_channel(0, 1500);
    REQUIRE(hal.rc_input.read(0) == 1500);

    hal.rc_output.force_safety_off();
    hal.rc_output.write(0, 1600);
    REQUIRE(hal.rc_output.read(0) == 1600);

    hal.analog_in.set_voltage_pin_voltage(11.1f);
    REQUIRE(hal.analog_in.voltage_latest(fwcpp::hal::kAnalogPinVoltage) == 11.1f);

    std::uint8_t injected[3] = {1, 2, 3};
    REQUIRE(hal.console.inject_rx(injected, 3) == 3);
    REQUIRE(hal.console.available() == 3);

    REQUIRE(hal.storage.write_block(0, injected, 3));

    hal.gpio.set_pin_mode(1, PinMode::kOutput);
    hal.gpio.write(1, 1);
    REQUIRE(hal.gpio.read(1) == 1);
    REQUIRE(hal.semaphore.take_nonblocking());
    REQUIRE(hal.semaphore.give());

    hal.scheduler.tick();
    REQUIRE(hal.scheduler.ticks() == 1);
}

TEST_CASE("HalContext's loop_period_us reflects the constructor's loop_rate_hz", "[hal_context]") {
    HalContext hal(1000);
    REQUIRE(hal.scheduler.loop_period_us() == 1000);
}
