// Tests for fwcpp::hal::RcInput/RcOutput (CPP-025 slices 1-2).

#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/rc_input.hpp>
#include <fwcpp/hal/rc_output.hpp>

using namespace fwcpp::hal;

TEST_CASE("RcInput starts with all channels zero and no new input", "[rc_input]") {
    RcInput rc;
    REQUIRE(rc.num_channels() == kNumRcChannels);
    REQUIRE(rc.read(0) == 0);
    REQUIRE_FALSE(rc.new_input());
}

TEST_CASE("set_channel updates a single channel and sets new_input", "[rc_input]") {
    RcInput rc;
    rc.set_channel(2, 1500);
    REQUIRE(rc.read(2) == 1500);
    REQUIRE(rc.new_input());
}

TEST_CASE("new_input is true exactly once per update, then false until the next one", "[rc_input]") {
    RcInput rc;
    rc.set_channel(0, 1000);
    REQUIRE(rc.new_input());
    REQUIRE_FALSE(rc.new_input()); // already consumed
    rc.set_channel(0, 1100);
    REQUIRE(rc.new_input());
}

TEST_CASE("set_channels replaces the whole channel array at once", "[rc_input]") {
    RcInput rc;
    std::array<std::uint16_t, kNumRcChannels> values{};
    values[0] = 1000;
    values[3] = 2000;
    rc.set_channels(values);
    REQUIRE(rc.read(0) == 1000);
    REQUIRE(rc.read(3) == 2000);
    REQUIRE(rc.read(1) == 0);
}

TEST_CASE("read(ch) out of range returns 0, matching upstream's own bounds-checked contract", "[rc_input]") {
    RcInput rc;
    REQUIRE(rc.read(kNumRcChannels) == 0);
    REQUIRE(rc.read(255) == 0);
}

TEST_CASE("array read() copies min(len, num_channels) values and returns that count", "[rc_input]") {
    RcInput rc;
    rc.set_channel(0, 111);
    rc.set_channel(1, 222);
    std::uint16_t out[4] = {};
    std::uint8_t n = rc.read(out, 2);
    REQUIRE(n == 2);
    REQUIRE(out[0] == 111);
    REQUIRE(out[1] == 222);
}

TEST_CASE("RcOutput write/read round-trips a single channel when not corked", "[rc_output]") {
    RcOutput out;
    out.force_safety_off(); // RcOutput starts disarmed (matching upstream); arm it so this write isn't zeroed
    out.write(5, 1600);
    REQUIRE(out.read(5) == 1600);
}

TEST_CASE("RcOutput cork buffers writes until push", "[rc_output]") {
    RcOutput out;
    out.force_safety_off(); // armed, so writes aren't zeroed - see safety test below
    out.write(0, 1000);
    out.cork();
    out.write(0, 1500); // buffered, not yet visible
    REQUIRE(out.read(0) == 1000);
    out.push();
    REQUIRE(out.read(0) == 1500);
}

TEST_CASE("RcOutput starts disarmed and zeroes writes unless the channel is in the safety mask", "[rc_output]") {
    RcOutput out;
    REQUIRE(out.safety_state() == SafetyState::kDisarmed);

    out.write(2, 1500); // no safety_mask passed -> zeroed while disarmed
    REQUIRE(out.read(2) == 0);

    out.write(2, 1500, /*safety_mask=*/1U << 2); // channel 2 exempted
    REQUIRE(out.read(2) == 1500);
}

TEST_CASE("force_safety_off allows writes through without a safety_mask", "[rc_output]") {
    RcOutput out;
    out.force_safety_off();
    REQUIRE(out.safety_state() == SafetyState::kArmed);
    out.write(2, 1500);
    REQUIRE(out.read(2) == 1500);
}

TEST_CASE("RcOutput enable_ch/disable_ch tracks the enable mask independently of write/read", "[rc_output]") {
    RcOutput out;
    REQUIRE_FALSE(out.channel_enabled(4));
    out.enable_ch(4);
    REQUIRE(out.channel_enabled(4));
    out.disable_ch(4);
    REQUIRE_FALSE(out.channel_enabled(4));
}

TEST_CASE("RcOutput array read copies exactly the requested (bounded) number of channels", "[rc_output]") {
    RcOutput out;
    out.force_safety_off();
    out.write(0, 100);
    out.write(1, 200);
    std::uint16_t buf[2] = {};
    out.read(buf, 2);
    REQUIRE(buf[0] == 100);
    REQUIRE(buf[1] == 200);
}
