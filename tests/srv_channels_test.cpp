// Tests for fwcpp::srv::SrvChannels (CPP-027, registry "slice 1").

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/hal/rc_output.hpp>
#include <fwcpp/srv/srv_channels.hpp>

using namespace fwcpp::srv;
using fwcpp::hal::RcOutput;

namespace {
// Sets up a channel at index `i` as an angle channel (servo_min/trim/max =
// 1000/1500/2000, high_out = 4500) tagged with `function` - mirrors
// srv_channel_test.cpp's make_angle_channel(), applied in place on the
// registry's owned array.
void configure_angle_channel(SrvChannels& reg, std::uint8_t i, Function function) {
    reg.channels[i].servo_min = 1000;
    reg.channels[i].servo_trim = 1500;
    reg.channels[i].servo_max = 2000;
    reg.channels[i].reversed = false;
    reg.channels[i].set_angle(4500);
    reg.channels[i].function = function;
}
} // namespace

// ---- kNumServoChannels ----------------------------------------------------

TEST_CASE("kNumServoChannels is 32, matching kNumRcOutputChannels", "[srv_channels]") {
    REQUIRE(kNumServoChannels == 32);
    REQUIRE(kNumServoChannels == fwcpp::hal::kNumRcOutputChannels);
    SrvChannels reg;
    REQUIRE(reg.channels.size() == 32);
}

// ---- find_first_channel / function lookup ---------------------------------

TEST_CASE("find_first_channel finds the single channel tagged with a function", "[srv_channels][lookup]") {
    SrvChannels reg;
    configure_angle_channel(reg, 3, Function::kElevator);

    std::uint8_t chan = 0;
    REQUIRE(reg.find_first_channel(Function::kElevator, chan));
    REQUIRE(chan == 3);
}

TEST_CASE("find_first_channel returns false when no channel has the function", "[srv_channels][lookup]") {
    SrvChannels reg;
    std::uint8_t chan = 0;
    REQUIRE_FALSE(reg.find_first_channel(Function::kRudder, chan));
}

TEST_CASE("find_first_channel returns the LOWEST index when multiple channels share a function", "[srv_channels][lookup]") {
    SrvChannels reg;
    // Two ailerons on channels 5 and 1 - a real multi-channel-shares-one-
    // function case (e.g. left/right aileron both tagged k_aileron).
    configure_angle_channel(reg, 5, Function::kAileron);
    configure_angle_channel(reg, 1, Function::kAileron);

    std::uint8_t chan = 0;
    REQUIRE(reg.find_first_channel(Function::kAileron, chan));
    REQUIRE(chan == 1);
}

TEST_CASE("find_first_channel returns false for an invalid function (kGpio)", "[srv_channels][lookup]") {
    SrvChannels reg;
    configure_angle_channel(reg, 0, Function::kGpio);
    std::uint8_t chan = 0;
    REQUIRE_FALSE(reg.find_first_channel(Function::kGpio, chan));
}

// ---- get_output_channel_mask ----------------------------------------------

TEST_CASE("get_output_channel_mask is 0 when no channel has the function", "[srv_channels][mask]") {
    SrvChannels reg;
    REQUIRE(reg.get_output_channel_mask(Function::kRudder) == 0U);
}

TEST_CASE("get_output_channel_mask sets exactly the bits of matching channel indices", "[srv_channels][mask]") {
    SrvChannels reg;
    configure_angle_channel(reg, 0, Function::kFlaperonLeft);
    configure_angle_channel(reg, 2, Function::kFlaperonLeft);
    configure_angle_channel(reg, 31, Function::kFlaperonLeft);
    configure_angle_channel(reg, 1, Function::kFlaperonRight); // distractor, different function

    const std::uint32_t mask = reg.get_output_channel_mask(Function::kFlaperonLeft);
    REQUIRE(mask == ((1U << 0) | (1U << 2) | (1U << 31)));

    const std::uint32_t right_mask = reg.get_output_channel_mask(Function::kFlaperonRight);
    REQUIRE(right_mask == (1U << 1));
}

// ---- set_output_pwm / get_output_pwm_chan (ALL matches) --------------------

TEST_CASE("set_output_pwm writes the pwm to ALL channels sharing the function", "[srv_channels][pwm]") {
    SrvChannels reg;
    configure_angle_channel(reg, 4, Function::kAileron);
    configure_angle_channel(reg, 7, Function::kAileron);
    configure_angle_channel(reg, 2, Function::kElevator); // distractor

    reg.set_output_pwm(Function::kAileron, 1234);

    std::uint16_t v = 0;
    REQUIRE(reg.get_output_pwm_chan(4, v));
    REQUIRE(v == 1234);
    REQUIRE(reg.get_output_pwm_chan(7, v));
    REQUIRE(v == 1234);
    REQUIRE(reg.get_output_pwm_chan(2, v));
    REQUIRE(v == 0); // untouched - different function
}

TEST_CASE("set_output_pwm on an unassigned function touches no channel", "[srv_channels][pwm]") {
    SrvChannels reg;
    configure_angle_channel(reg, 0, Function::kElevator);
    reg.set_output_pwm(Function::kRudder, 1600); // no channel has kRudder

    std::uint16_t v = 0;
    REQUIRE(reg.get_output_pwm_chan(0, v));
    REQUIRE(v == 0);
}

// ---- get_output_pwm (FIRST match only, with recompute side effect) --------

TEST_CASE("get_output_pwm returns false when no channel has the function", "[srv_channels][pwm]") {
    SrvChannels reg;
    std::uint16_t v = 999;
    REQUIRE_FALSE(reg.get_output_pwm(Function::kRudder, v));
    REQUIRE(v == 999); // untouched on failure
}

TEST_CASE("get_output_pwm reads back only the FIRST matching channel", "[srv_channels][pwm]") {
    SrvChannels reg;
    configure_angle_channel(reg, 6, Function::kAileron);
    configure_angle_channel(reg, 3, Function::kAileron);
    // Give the two matching channels visibly different raw pwm so we can
    // tell which one get_output_pwm actually read.
    reg.channels[6].set_output_pwm(1800);
    reg.channels[3].set_output_pwm(1200);

    std::uint16_t v = 0;
    REQUIRE(reg.get_output_pwm(Function::kAileron, v));
    // Lowest index (3) wins, and get_output_pwm recomputes it from the
    // per-function scaled cache (default 0.0, i.e. trim for an angle
    // channel) rather than returning the 1200 that was just written via
    // set_output_pwm - this is upstream's real (surprising) recompute
    // side effect, see srv_channels.hpp's file banner finding #1.
    REQUIRE(v == 1500);
    REQUIRE(reg.channels[3].get_output_pwm() == 1500);
    REQUIRE(reg.channels[6].get_output_pwm() == 1800); // channel 6 untouched
}

TEST_CASE("get_output_pwm reflects a prior set_output_scaled on the function", "[srv_channels][pwm]") {
    SrvChannels reg;
    configure_angle_channel(reg, 0, Function::kElevator);
    reg.set_output_scaled(Function::kElevator, 4500.0f); // full deflection

    std::uint16_t v = 0;
    REQUIRE(reg.get_output_pwm(Function::kElevator, v));
    REQUIRE(v == 2000); // servo_max
}

// ---- set_output_pwm_chan / get_output_pwm_chan (direct index, bounds-checked)

TEST_CASE("set_output_pwm_chan/get_output_pwm_chan round-trip a raw pwm by index", "[srv_channels][chan]") {
    SrvChannels reg;
    reg.set_output_pwm_chan(10, 1750);

    std::uint16_t v = 0;
    REQUIRE(reg.get_output_pwm_chan(10, v));
    REQUIRE(v == 1750);
}

TEST_CASE("set_output_pwm_chan out of range is a silent no-op", "[srv_channels][chan][bounds]") {
    SrvChannels reg;
    reg.set_output_pwm_chan(32, 1750); // kNumServoChannels == 32, so 32 is out of range
    reg.set_output_pwm_chan(255, 1750);
    // Nothing should have crashed or corrupted channel 0/31.
    std::uint16_t v = 0;
    REQUIRE(reg.get_output_pwm_chan(0, v));
    REQUIRE(v == 0);
    REQUIRE(reg.get_output_pwm_chan(31, v));
    REQUIRE(v == 0);
}

TEST_CASE("get_output_pwm_chan out of range returns false and leaves value untouched", "[srv_channels][chan][bounds]") {
    SrvChannels reg;
    std::uint16_t v = 4242;
    REQUIRE_FALSE(reg.get_output_pwm_chan(32, v));
    REQUIRE(v == 4242);
    REQUIRE_FALSE(reg.get_output_pwm_chan(255, v));
    REQUIRE(v == 4242);
}

// ---- set_output_norm (ALL matches) / get_output_norm (FIRST match) --------

TEST_CASE("set_output_norm writes to ALL channels sharing the function", "[srv_channels][norm]") {
    SrvChannels reg;
    configure_angle_channel(reg, 0, Function::kAileron);
    configure_angle_channel(reg, 1, Function::kAileron);

    reg.set_output_norm(Function::kAileron, 1.0f);

    REQUIRE(reg.channels[0].get_output_norm() == Catch::Approx(1.0f).margin(0.01f));
    REQUIRE(reg.channels[1].get_output_norm() == Catch::Approx(1.0f).margin(0.01f));
}

TEST_CASE("get_output_norm returns 0 when no channel has the function", "[srv_channels][norm]") {
    SrvChannels reg;
    REQUIRE(reg.get_output_norm(Function::kRudder) == 0.0f);
}

TEST_CASE("get_output_norm reads back only the FIRST matching channel", "[srv_channels][norm]") {
    SrvChannels reg;
    configure_angle_channel(reg, 6, Function::kElevator);
    configure_angle_channel(reg, 3, Function::kElevator);
    reg.channels[6].set_output_norm(1.0f);
    reg.channels[3].set_output_norm(-1.0f);

    // Same recompute-from-cache side effect as get_output_pwm: with
    // nothing set via set_output_scaled, the function's cached scaled
    // value is still 0.0, so channel 3 (lowest index) gets recomputed back
    // to its midpoint (norm 0), NOT the -1.0 that set_output_norm just
    // wrote to it directly.
    REQUIRE(reg.get_output_norm(Function::kElevator) == Catch::Approx(0.0f).margin(0.01f));
    REQUIRE(reg.channels[6].get_output_norm() == Catch::Approx(1.0f).margin(0.01f)); // untouched
}

// ---- set_output_scaled / get_output_scaled ---------------------------------

TEST_CASE("get_output_scaled defaults to 0 for a function nothing has set", "[srv_channels][scaled]") {
    SrvChannels reg;
    REQUIRE(reg.get_output_scaled(Function::kThrottle) == 0.0f);
}

TEST_CASE("set_output_scaled/get_output_scaled round-trip independent of channel assignment", "[srv_channels][scaled]") {
    SrvChannels reg;
    // No channel is tagged kThrottle at all - upstream's real semantics
    // (file banner finding #2) are that the scaled-value cache is keyed
    // purely by Function, not by "does some channel currently have it".
    reg.set_output_scaled(Function::kThrottle, 0.75f);
    REQUIRE(reg.get_output_scaled(Function::kThrottle) == Catch::Approx(0.75f));
}

TEST_CASE("get_output_scaled returns 0 for an invalid function (kGpio)", "[srv_channels][scaled]") {
    SrvChannels reg;
    reg.set_output_scaled(Function::kGpio, 5.0f); // no-op, kGpio is not valid_function
    REQUIRE(reg.get_output_scaled(Function::kGpio) == 0.0f);
}

TEST_CASE("set_output_scaled fans out calc_pwm to ALL channels sharing the function", "[srv_channels][scaled]") {
    SrvChannels reg;
    configure_angle_channel(reg, 0, Function::kElevator);
    configure_angle_channel(reg, 5, Function::kElevator);
    configure_angle_channel(reg, 2, Function::kAileron); // distractor

    reg.set_output_scaled(Function::kElevator, -4500.0f); // full negative deflection

    REQUIRE(reg.channels[0].get_output_pwm() == 1000); // servo_min
    REQUIRE(reg.channels[5].get_output_pwm() == 1000);
    REQUIRE(reg.channels[2].get_output_pwm() == 0); // untouched - different function
}

// ---- set_default_function --------------------------------------------------

TEST_CASE("set_default_function sets the function field directly, bounds-checked", "[srv_channels][default_function]") {
    SrvChannels reg;
    reg.set_default_function(9, Function::kSteering);
    REQUIRE(reg.channels[9].function == Function::kSteering);
}

TEST_CASE("set_default_function out of range is a silent no-op", "[srv_channels][default_function][bounds]") {
    SrvChannels reg;
    reg.set_default_function(32, Function::kSteering);
    reg.set_default_function(255, Function::kSteering);
    // Nothing to observe directly (index is out of range by construction);
    // this just documents that it must not crash under ASan.
    SUCCEED();
}

// ---- output_ch_all ----------------------------------------------------------

TEST_CASE("output_ch_all writes every channel's output_pwm through to RcOutput", "[srv_channels][output]") {
    SrvChannels reg;
    RcOutput out;
    out.force_safety_off(); // RcOutput starts disarmed; see hal_context_test.cpp/rc_test.cpp precedent

    configure_angle_channel(reg, 0, Function::kAileron);
    configure_angle_channel(reg, 1, Function::kElevator);
    reg.channels[0].set_output_pwm(1650);
    reg.channels[1].set_output_pwm(1350);

    reg.output_ch_all(out);

    REQUIRE(out.read(0) == 1650);
    REQUIRE(out.read(1) == 1350);
}

TEST_CASE("output_ch_all writes zero-initialized channels too (full 32-channel sweep)", "[srv_channels][output]") {
    SrvChannels reg;
    RcOutput out;
    out.force_safety_off();

    reg.channels[17].set_output_pwm(1888);

    reg.output_ch_all(out);

    for (std::uint8_t i = 0; i < kNumServoChannels; ++i) {
        if (i == 17) {
            REQUIRE(out.read(i) == 1888);
        } else {
            REQUIRE(out.read(i) == 0);
        }
    }
}

TEST_CASE("output_ch_all reflects set_output_scaled's immediate calc_pwm fan-out", "[srv_channels][output]") {
    SrvChannels reg;
    RcOutput out;
    out.force_safety_off();

    configure_angle_channel(reg, 4, Function::kRudder);
    reg.set_output_scaled(Function::kRudder, 2250.0f); // halfway trim->max

    reg.output_ch_all(out);

    REQUIRE(out.read(4) == 1750);
}

TEST_CASE("output_ch_all respects RcOutput's disarmed safety state (0 output while disarmed)", "[srv_channels][output]") {
    SrvChannels reg;
    RcOutput out; // safety left on (disarmed) - default
    configure_angle_channel(reg, 0, Function::kAileron);
    reg.channels[0].set_output_pwm(1650);

    reg.output_ch_all(out);

    REQUIRE(out.read(0) == 0); // RcOutput::write() forces 0 while disarmed
}
