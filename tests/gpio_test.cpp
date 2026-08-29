// CPP-088 slice 1: SITL GPIO pin mode / digital read-write.

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/gpio.hpp>

using fwcpp::hal::Gpio;
using fwcpp::hal::PinMode;
using fwcpp::hal::kGpioPinCount;

TEST_CASE("gpio pin write/read round-trips on an output pin", "[hal][gpio]") {
    Gpio gpio;
    gpio.init();
    REQUIRE(gpio.is_initialized());
    REQUIRE(gpio.valid_pin(3));
    REQUIRE_FALSE(gpio.valid_pin(kGpioPinCount));

    gpio.set_pin_mode(3, PinMode::kOutput);
    REQUIRE(gpio.mode_of(3) == PinMode::kOutput);

    gpio.write(3, 1);
    REQUIRE(gpio.read(3) == 1);
    gpio.write(3, 0);
    REQUIRE(gpio.read(3) == 0);

    gpio.write(3, 7);
    REQUIRE(gpio.read(3) == 1);
}

TEST_CASE("gpio pin mode INPUT/OUTPUT/ALT is stored per pin", "[hal][gpio]") {
    Gpio gpio;
    REQUIRE(gpio.mode_of(0) == PinMode::kInput);

    gpio.set_pin_mode(0, PinMode::kOutput);
    gpio.set_pin_mode(1, PinMode::kAlt);
    gpio.set_pin_mode(2, PinMode::kInput);

    REQUIRE(gpio.mode_of(0) == PinMode::kOutput);
    REQUIRE(gpio.mode_of(1) == PinMode::kAlt);
    REQUIRE(gpio.mode_of(2) == PinMode::kInput);
}

TEST_CASE("gpio write on pins 0-7 is ignored until OUTPUT (SITL pull-up)", "[hal][gpio]") {
    Gpio gpio;
    gpio.write(4, 1);
    REQUIRE(gpio.read(4) == 0);

    gpio.set_pin_mode(4, PinMode::kOutput);
    gpio.write(4, 1);
    REQUIRE(gpio.read(4) == 1);

    gpio.toggle(4);
    REQUIRE(gpio.read(4) == 0);
}

TEST_CASE("gpio pins 8-15 accept write without pinMode (SITL)", "[hal][gpio]") {
    Gpio gpio;
    gpio.write(12, 1);
    REQUIRE(gpio.read(12) == 1);
    REQUIRE(gpio.read(16) == 0);
    gpio.write(16, 1);
    REQUIRE(gpio.read(16) == 0);
    REQUIRE_FALSE(gpio.usb_connected());
}
