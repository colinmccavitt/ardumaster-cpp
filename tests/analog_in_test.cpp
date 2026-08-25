// Tests for fwcpp::hal::AnalogIn (CPP-025, AnalogIn slice).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/analog_in.hpp>

using namespace fwcpp::hal;

TEST_CASE("board_voltage is a fixed 5.0V, matching SITL's hardcoded override", "[analog_in]") {
    AnalogIn analog;
    REQUIRE(analog.board_voltage() == Catch::Approx(5.0f));
}

TEST_CASE("servorail_voltage is always 0, matching SITL's unimplemented base-class default", "[analog_in]") {
    AnalogIn analog;
    REQUIRE(analog.servorail_voltage() == Catch::Approx(0.0f));
}

TEST_CASE("unmapped and NONE pins read 0V, matching upstream's switch-default", "[analog_in]") {
    AnalogIn analog;
    REQUIRE(analog.voltage_latest(3) == Catch::Approx(0.0f));
    REQUIRE(analog.voltage_latest(kAnalogInputNone) == Catch::Approx(0.0f));
    REQUIRE(analog.read_latest(3) == Catch::Approx(0.0f));
}

TEST_CASE("sonar pin round-trips an injected voltage", "[analog_in]") {
    AnalogIn analog;
    analog.set_sonar_pin_voltage(3.3f);
    REQUIRE(analog.voltage_latest(kAnalogPinSonar) == Catch::Approx(3.3f));
    REQUIRE(analog.voltage_average(kAnalogPinSonar) == Catch::Approx(3.3f));
    REQUIRE(analog.voltage_average_ratiometric(kAnalogPinSonar) == Catch::Approx(3.3f));
}

TEST_CASE("both airspeed pins round-trip independently", "[analog_in]") {
    AnalogIn analog;
    analog.set_airspeed_pin_voltage(0, 1.1f);
    analog.set_airspeed_pin_voltage(1, 2.2f);
    REQUIRE(analog.voltage_latest(kAnalogPinAirspeed0) == Catch::Approx(1.1f));
    REQUIRE(analog.voltage_latest(kAnalogPinAirspeed1) == Catch::Approx(2.2f));
}

TEST_CASE("airspeed sensor_index out of range is silently ignored, matching this port's bounds-checked injection style",
          "[analog_in]") {
    AnalogIn analog;
    analog.set_airspeed_pin_voltage(2, 9.9f); // only indices 0 and 1 exist
    REQUIRE(analog.voltage_latest(kAnalogPinAirspeed0) == Catch::Approx(0.0f));
    REQUIRE(analog.voltage_latest(kAnalogPinAirspeed1) == Catch::Approx(0.0f));
}

TEST_CASE("battery monitor pins (current/voltage, primary and secondary) round-trip independently", "[analog_in]") {
    AnalogIn analog;
    analog.set_current_pin_voltage(1.0f);
    analog.set_voltage_pin_voltage(2.0f);
    analog.set_current2_pin_voltage(3.0f);
    analog.set_voltage2_pin_voltage(4.0f);

    REQUIRE(analog.voltage_latest(kAnalogPinCurrent) == Catch::Approx(1.0f));
    REQUIRE(analog.voltage_latest(kAnalogPinVoltage) == Catch::Approx(2.0f));
    REQUIRE(analog.voltage_latest(kAnalogPinCurrent2) == Catch::Approx(3.0f));
    REQUIRE(analog.voltage_latest(kAnalogPinVoltage2) == Catch::Approx(4.0f));
}

TEST_CASE("reading ANALOG_INPUT_BOARD_VCC through voltage_latest returns a raw ADC count, not a voltage - "
          "reproducing SITL's own quirk",
          "[analog_in]") {
    AnalogIn analog;
    // See file banner: this is a real upstream inconsistency (mixing
    // domains), reproduced faithfully rather than "fixed" here.
    REQUIRE(analog.voltage_latest(kAnalogInputBoardVcc) == Catch::Approx(65535.0f));
    // board_voltage() remains the correct way to read board VCC as a voltage.
    REQUIRE(analog.board_voltage() == Catch::Approx(5.0f));
}

TEST_CASE("read_latest digitises voltage_latest to a 16-bit ADC count assuming a 5V full scale", "[analog_in]") {
    AnalogIn analog;
    analog.set_voltage_pin_voltage(2.5f); // half of the 5.0V full-scale reference
    REQUIRE(analog.read_latest(kAnalogPinVoltage) == Catch::Approx(32767.5f).margin(0.01f));
    REQUIRE(analog.read_average(kAnalogPinVoltage) == Catch::Approx(analog.read_latest(kAnalogPinVoltage)));
}

TEST_CASE("read_latest clamps out-of-range voltages to the ADC's [0, 65535] count range", "[analog_in]") {
    AnalogIn analog;

    analog.set_voltage_pin_voltage(-1.0f); // below the ADC's representable range
    REQUIRE(analog.read_latest(kAnalogPinVoltage) == Catch::Approx(0.0f));

    analog.set_voltage_pin_voltage(10.0f); // above the 5.0V full-scale reference
    REQUIRE(analog.read_latest(kAnalogPinVoltage) == Catch::Approx(65535.0f));
}

TEST_CASE("read_average and voltage_average always equal their _latest counterparts, matching SITL's lack of "
          "sample accumulation",
          "[analog_in]") {
    AnalogIn analog;
    analog.set_current_pin_voltage(1.7f);
    REQUIRE(analog.read_average(kAnalogPinCurrent) == Catch::Approx(analog.read_latest(kAnalogPinCurrent)));
    REQUIRE(analog.voltage_average(kAnalogPinCurrent) == Catch::Approx(analog.voltage_latest(kAnalogPinCurrent)));
}
