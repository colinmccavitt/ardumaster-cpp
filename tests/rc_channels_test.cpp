// Tests for fwcpp::rc::RcChannels (CPP-027 slice).

#include <array>
#include <cstdint>
#include <optional>

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/rc_input.hpp>
#include <fwcpp/rc/rc_channels.hpp>

using fwcpp::rc::ControlType;
using fwcpp::rc::kNumRcChannels;
using fwcpp::rc::RcChannel;
using fwcpp::rc::RcChannels;

TEST_CASE("channel() defaults ch_in to the channel's own index", "[rc_channels]") {
    RcChannels rc;
    for (std::uint8_t i = 0; i < kNumRcChannels; ++i) {
        REQUIRE(rc.channel(i)->ch_in == i);
    }
}

TEST_CASE("channel() is bounds-checked and returns nullptr out of range", "[rc_channels]") {
    RcChannels rc;
    REQUIRE(rc.channel(0) != nullptr);
    REQUIRE(rc.channel(kNumRcChannels - 1) != nullptr);
    REQUIRE(rc.channel(kNumRcChannels) == nullptr);
    REQUIRE(rc.channel(255) == nullptr);

    const RcChannels& crc = rc;
    REQUIRE(crc.channel(0) != nullptr);
    REQUIRE(crc.channel(kNumRcChannels) == nullptr);
}

TEST_CASE("has_valid_input is false before any input has ever been seen", "[rc_channels]") {
    RcChannels rc;
    REQUIRE_FALSE(rc.has_valid_input());
}

TEST_CASE("read_input returns false and leaves has_valid_input false when RcInput has no new frame", "[rc_channels]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    // No set_channel()/set_channels() call yet -> new_input() is false.
    REQUIRE_FALSE(rc.read_input(rc_input));
    REQUIRE_FALSE(rc.has_valid_input());
}

TEST_CASE("read_input pulls radio_in through from RcInput by each channel's ch_in index", "[rc_channels]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;

    rc_input.set_channel(0, 1000);
    rc_input.set_channel(1, 1500);
    rc_input.set_channel(5, 1999);

    REQUIRE(rc.read_input(rc_input));

    REQUIRE(rc.channel(0)->radio_in == 1000);
    REQUIRE(rc.channel(1)->radio_in == 1500);
    REQUIRE(rc.channel(5)->radio_in == 1999);
    // A channel with no injected value reads RcInput's own default (0).
    REQUIRE(rc.channel(2)->radio_in == 0);
}

TEST_CASE("read_input honors a remapped ch_in rather than always using the array index", "[rc_channels]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;

    // Remap channel 0 to read from RcInput index 7 instead of 0.
    rc.channel(0)->ch_in = 7;
    rc_input.set_channel(7, 1750);
    rc_input.set_channel(0, 1111); // channel 0 should NOT pick this up

    REQUIRE(rc.read_input(rc_input));
    REQUIRE(rc.channel(0)->radio_in == 1750);
}

TEST_CASE("has_valid_input becomes true after read_input observes new input, and stays true", "[rc_channels]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;

    REQUIRE_FALSE(rc.has_valid_input());

    rc_input.set_channel(0, 1500);
    REQUIRE(rc.read_input(rc_input));
    REQUIRE(rc.has_valid_input());

    // new_input() is now false again (RcInput's one-shot latch) - but
    // has_valid_input() must remain true: it never resets once set,
    // matching upstream's _has_ever_seen_rc_input.
    REQUIRE_FALSE(rc.read_input(rc_input));
    REQUIRE(rc.has_valid_input());
}

TEST_CASE("read_input computes control_in via pwm_to_angle for an ANGLE channel", "[rc_channels]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;

    RcChannel* ch = rc.channel(0);
    ch->type_in = ControlType::kAngle;
    ch->radio_min = 1000;
    ch->radio_trim = 1500;
    ch->radio_max = 2000;
    ch->dead_zone = 0;
    ch->high_in = 4500;

    rc_input.set_channel(0, 2000); // full deflection towards radio_max
    REQUIRE(rc.read_input(rc_input));
    REQUIRE(ch->control_in == 4500);
}

TEST_CASE("read_input computes control_in via pwm_to_range for a RANGE channel", "[rc_channels]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;

    RcChannel* ch = rc.channel(1);
    ch->type_in = ControlType::kRange;
    ch->radio_min = 1000;
    ch->radio_trim = 1500;
    ch->radio_max = 2000;
    ch->dead_zone = 0;
    ch->high_in = 100; // e.g. a 0-100 throttle-style range

    rc_input.set_channel(1, 1500); // halfway across the full min/max span
    REQUIRE(rc.read_input(rc_input));
    REQUIRE(ch->control_in == 50);
}

TEST_CASE("get_radio_in zero-fills then copies up to kNumRcChannels entries", "[rc_channels]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    rc_input.set_channel(0, 1100);
    rc_input.set_channel(1, 1200);
    REQUIRE(rc.read_input(rc_input));

    std::array<std::uint16_t, kNumRcChannels> buf;
    buf.fill(0xFFFF);
    const std::uint8_t n = rc.get_radio_in(buf.data(), static_cast<std::uint8_t>(buf.size()));

    REQUIRE(n == kNumRcChannels);
    REQUIRE(buf[0] == 1100);
    REQUIRE(buf[1] == 1200);
    // Every other channel's radio_in is still its untouched default (0).
    for (std::uint8_t i = 2; i < kNumRcChannels; ++i) {
        REQUIRE(buf[i] == 0);
    }
}

TEST_CASE("get_radio_in fills only as many entries as the caller's buffer holds", "[rc_channels]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    rc_input.set_channel(0, 1300);
    rc_input.set_channel(1, 1400);
    rc_input.set_channel(2, 1600); // outside the small buffer below
    REQUIRE(rc.read_input(rc_input));

    std::array<std::uint16_t, 2> small_buf{};
    const std::uint8_t n = rc.get_radio_in(small_buf.data(), static_cast<std::uint8_t>(small_buf.size()));

    REQUIRE(n == 2);
    REQUIRE(small_buf[0] == 1300);
    REQUIRE(small_buf[1] == 1400);
}

TEST_CASE("get_valid_channel_count is the min of kNumRcChannels and RcInput::num_channels", "[rc_channels]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    // fwcpp::hal::RcInput::num_channels() is fixed at kNumRcChannels (32),
    // which is larger than fwcpp::rc::kNumRcChannels (16).
    REQUIRE(rc.get_valid_channel_count(rc_input) == kNumRcChannels);
}

// ---------------------------------------------------------------------
// CPP-031 SLICE 11: flight_mode_channel_number/flight_mode_channel()/
// read_mode_switch() - see rc_channels.hpp's own file banner for the
// full design (why this returns std::optional<std::int8_t> rather than
// dispatching through a virtual callback).
// ---------------------------------------------------------------------

TEST_CASE("flight_mode_channel_number defaults to 8 (ArduPlane's real FLIGHT_MODE_CHANNEL stock default), "
          "1-indexed, resolving to channel(7)",
          "[rc_channels][mode_switch]") {
    RcChannels rc;
    REQUIRE(rc.flight_mode_channel_number == 8);
    REQUIRE(rc.flight_mode_channel() == rc.channel(7));
}

TEST_CASE("flight_mode_channel returns nullptr when unconfigured (<=0) or out of range", "[rc_channels][mode_switch]") {
    RcChannels rc;

    rc.flight_mode_channel_number = 0;
    REQUIRE(rc.flight_mode_channel() == nullptr);

    rc.flight_mode_channel_number = -1;
    REQUIRE(rc.flight_mode_channel() == nullptr);

    rc.flight_mode_channel_number = static_cast<std::int8_t>(kNumRcChannels) + 1; // one past the last valid channel
    REQUIRE(rc.flight_mode_channel() == nullptr);

    rc.flight_mode_channel_number = static_cast<std::int8_t>(kNumRcChannels); // the last valid channel (1-indexed)
    REQUIRE(rc.flight_mode_channel() == rc.channel(kNumRcChannels - 1));
}

TEST_CASE("read_mode_switch returns nullopt before any RC input has ever been seen", "[rc_channels][mode_switch]") {
    RcChannels rc;
    REQUIRE_FALSE(rc.read_mode_switch(0).has_value());
}

TEST_CASE("read_mode_switch returns nullopt when no mode-switch channel is configured", "[rc_channels][mode_switch]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    rc_input.set_channel(7, 1500);
    REQUIRE(rc.read_input(rc_input));

    rc.flight_mode_channel_number = 0; // disabled, matching upstream's own "0 = disabled" convention
    REQUIRE_FALSE(rc.read_mode_switch(1000).has_value());
}

TEST_CASE("read_mode_switch dispatches to the configured channel and requires the full debounce window before "
          "returning a position",
          "[rc_channels][mode_switch]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    rc_input.set_channel(7, 1500); // the default FLTMODE_CH=8 (1-indexed) -> index 7; PWM 1500 -> position 3
    REQUIRE(rc.read_input(rc_input));

    REQUIRE_FALSE(rc.read_mode_switch(0).has_value());   // first observation - debounce not complete
    REQUIRE_FALSE(rc.read_mode_switch(100).has_value()); // still within the 200ms window
    const std::optional<std::int8_t> pos = rc.read_mode_switch(200);
    REQUIRE(pos.has_value());
    REQUIRE(*pos == 3);

    // Once settled, re-reading the SAME position is "no new change" -
    // matches RcChannel::debounce_completed()'s own event (not level)
    // semantics.
    REQUIRE_FALSE(rc.read_mode_switch(1000).has_value());
}

TEST_CASE("read_mode_switch returns nullopt for an out-of-range (error) PWM on the mode-switch channel",
          "[rc_channels][mode_switch]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    rc_input.set_channel(7, 700); // below RC_MIN_LIMIT_PWM (800) - a receiver/wiring error, not a real position
    REQUIRE(rc.read_input(rc_input));

    REQUIRE_FALSE(rc.read_mode_switch(0).has_value());
    REQUIRE_FALSE(rc.read_mode_switch(10000).has_value()); // no amount of elapsed time makes an invalid PWM valid
}

TEST_CASE("read_mode_switch honors a remapped mode-switch channel number, not always index 7",
          "[rc_channels][mode_switch]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    rc.flight_mode_channel_number = 6; // 1-indexed -> index 5
    rc_input.set_channel(5, 1231);     // position 1
    rc_input.set_channel(7, 1231);     // the DEFAULT channel - must NOT be read once remapped
    REQUIRE(rc.read_input(rc_input));

    REQUIRE_FALSE(rc.read_mode_switch(0).has_value());
    const std::optional<std::int8_t> pos = rc.read_mode_switch(200);
    REQUIRE(pos.has_value());
    REQUIRE(*pos == 1);
}
