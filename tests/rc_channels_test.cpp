// Tests for fwcpp::rc::RcChannels (CPP-027 slice).

#include <array>
#include <cstdint>

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
