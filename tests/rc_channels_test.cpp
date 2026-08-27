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

// ---------------------------------------------------------------------
// CPP-037: read_aux_all()/reset_mode_switch() - the RcChannels-level half
// of the 3-position aux-function switch mechanism. See rc_channels.hpp's
// own "CPP-037 ADDENDUM" file banner.
// ---------------------------------------------------------------------

using fwcpp::rc::AuxFunc;
using fwcpp::rc::AuxSwitchPos;

TEST_CASE("read_aux_all does nothing before any RC input has ever been seen", "[rc_channels][aux]") {
    RcChannels rc;
    rc.channel(9)->option = AuxFunc::ArmDisarm;
    rc.channel(9)->radio_in = 1900;
    int calls = 0;
    rc.read_aux_all(0, [&](AuxFunc, AuxSwitchPos) { ++calls; });
    REQUIRE(calls == 0);
}

TEST_CASE("read_aux_all skips every DoNothing-option channel, never invoking the handler for one",
          "[rc_channels][aux]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    for (std::uint8_t i = 0; i < kNumRcChannels; ++i) {
        rc_input.set_channel(i, 1900); // every channel HIGH - would fire if any were configured
    }
    REQUIRE(rc.read_input(rc_input));

    int calls = 0;
    rc.read_aux_all(1000, [&](AuxFunc, AuxSwitchPos) { ++calls; });
    REQUIRE(calls == 0); // no channel has a non-DoNothing option
}

TEST_CASE("read_aux_all dispatches exactly one (AuxFunc, AuxSwitchPos) event per configured channel that debounces "
          "a real change, and requires the full debounce window first",
          "[rc_channels][aux]") {
    RcChannels rc;
    rc.channel(9)->option = AuxFunc::Fbwa; // non-ARM-type: no first-read suppression to work around here
    fwcpp::hal::RcInput rc_input;
    rc_input.set_channel(9, 1900); // HIGH

    REQUIRE(rc.read_input(rc_input));
    int calls = 0;
    rc.read_aux_all(0, [&](AuxFunc, AuxSwitchPos) { ++calls; });
    REQUIRE(calls == 0); // debounce not complete yet

    rc.read_aux_all(199, [&](AuxFunc, AuxSwitchPos) { ++calls; });
    REQUIRE(calls == 0);

    AuxFunc seen_func = AuxFunc::DoNothing;
    AuxSwitchPos seen_pos = AuxSwitchPos::kMiddle;
    rc.read_aux_all(200, [&](AuxFunc func, AuxSwitchPos pos) {
        ++calls;
        seen_func = func;
        seen_pos = pos;
    });
    REQUIRE(calls == 1);
    REQUIRE(seen_func == AuxFunc::Fbwa);
    REQUIRE(seen_pos == AuxSwitchPos::kHigh);

    // Settled - no repeat event for the unchanged position.
    rc.read_aux_all(5000, [&](AuxFunc, AuxSwitchPos) { ++calls; });
    REQUIRE(calls == 1);
}

TEST_CASE("read_aux_all dispatches MULTIPLE independently-configured channels in the same call, each exactly once",
          "[rc_channels][aux]") {
    RcChannels rc;
    rc.channel(9)->option = AuxFunc::ArmDisarm;
    rc.channel(10)->option = AuxFunc::EmergencyLandingEn;
    fwcpp::hal::RcInput rc_input;
    rc_input.set_channel(9, 1000);  // LOW - a real change for ArmDisarm (not its adopted-baseline HIGH)
    rc_input.set_channel(10, 1900); // HIGH
    REQUIRE(rc.read_input(rc_input));

    // First observation of each: ArmDisarm adopts LOW as ITS OWN
    // suppressed baseline (init_position_on_first_radio_read()), while
    // EmergencyLandingEn (not an ARM-type function) just starts its
    // normal debounce window - neither fires yet.
    int calls = 0;
    rc.read_aux_all(0, [&](AuxFunc, AuxSwitchPos) { ++calls; });
    REQUIRE(calls == 0);

    // EmergencyLandingEn's window completes at 200; move ArmDisarm to a
    // real change (HIGH) at the same tick so both are pending together.
    rc_input.set_channel(9, 1900);
    REQUIRE(rc.read_input(rc_input));

    std::array<AuxFunc, 2> seen_funcs{};
    std::array<AuxSwitchPos, 2> seen_pos{};
    calls = 0;
    rc.read_aux_all(200, [&](AuxFunc func, AuxSwitchPos pos) {
        seen_funcs[static_cast<std::size_t>(calls)] = func;
        seen_pos[static_cast<std::size_t>(calls)] = pos;
        ++calls;
    });
    // EmergencyLandingEn's own window (started at t=0) completes exactly
    // at 200; ArmDisarm's NEW window (started at t=200, when it changed
    // away from its adopted LOW baseline) has NOT yet - only one event.
    REQUIRE(calls == 1);
    REQUIRE(seen_funcs[0] == AuxFunc::EmergencyLandingEn);
    REQUIRE(seen_pos[0] == AuxSwitchPos::kHigh);

    calls = 0;
    rc.read_aux_all(400, [&](AuxFunc func, AuxSwitchPos pos) {
        seen_funcs[static_cast<std::size_t>(calls)] = func;
        seen_pos[static_cast<std::size_t>(calls)] = pos;
        ++calls;
    });
    REQUIRE(calls == 1);
    REQUIRE(seen_funcs[0] == AuxFunc::ArmDisarm);
    REQUIRE(seen_pos[0] == AuxSwitchPos::kHigh);
}

TEST_CASE("reset_mode_switch is a no-op when no mode-switch channel is configured", "[rc_channels][aux][mode_switch]") {
    RcChannels rc;
    rc.flight_mode_channel_number = 0;
    rc.reset_mode_switch(1000); // must not crash
}

TEST_CASE("reset_mode_switch clears the flight-mode channel's debounce state, forcing a fresh full debounce window "
          "before its CURRENT (unchanged) physical position is reported again",
          "[rc_channels][aux][mode_switch]") {
    RcChannels rc;
    fwcpp::hal::RcInput rc_input;
    rc_input.set_channel(7, 1500); // position 3, the default FLTMODE_CH=8 -> index 7
    REQUIRE(rc.read_input(rc_input));

    REQUIRE_FALSE(rc.read_mode_switch(0).has_value()); // edge established at t=0
    const std::optional<std::int8_t> settled = rc.read_mode_switch(200);
    REQUIRE(settled.has_value());
    REQUIRE(*settled == 3);
    REQUIRE_FALSE(rc.read_mode_switch(300).has_value()); // settled, no repeat event

    // Reset at t=500, PWM unchanged - the position doesn't need to
    // physically move for this to matter (MODE_SWITCH_RESET/aux-mode-
    // release calls this with the switch never having moved).
    rc.reset_mode_switch(500);

    // Immediately after reset, nothing new (the debounce window for the
    // re-adopted position has only just started, at t=500).
    REQUIRE_FALSE(rc.read_mode_switch(500).has_value());
    REQUIRE_FALSE(rc.read_mode_switch(699).has_value()); // 1ms short of 500+200

    // A full fresh debounce window after the reset - the SAME position 3
    // is reported as a "change" again, exactly like upstream's own
    // reset_mode_switch() + read_mode_switch() sequence.
    const std::optional<std::int8_t> reasserted = rc.read_mode_switch(700);
    REQUIRE(reasserted.has_value());
    REQUIRE(*reasserted == 3);
}

