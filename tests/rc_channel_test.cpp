// Tests for fwcpp::rc::RcChannel (CPP-030 slice 1).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/rc/rc_channel.hpp>

using namespace fwcpp::rc;

namespace {
RcChannel make_channel() {
    RcChannel ch;
    ch.radio_min = 1000;
    ch.radio_trim = 1500;
    ch.radio_max = 2000;
    ch.dead_zone = 0;
    ch.reversed = false;
    ch.high_in = 4500;
    return ch;
}
} // namespace

TEST_CASE("pwm_to_angle returns 0 at trim", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.radio_in = 1500;
    REQUIRE(ch.pwm_to_angle() == 0.0f);
}

TEST_CASE("pwm_to_angle returns +high_in at radio_max and -high_in at radio_min", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.radio_in = 2000;
    REQUIRE(ch.pwm_to_angle() == Catch::Approx(4500.0f));
    ch.radio_in = 1000;
    REQUIRE(ch.pwm_to_angle() == Catch::Approx(-4500.0f));
}

TEST_CASE("pwm_to_angle interpolates linearly between trim and max", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.radio_in = 1750; // halfway between trim(1500) and max(2000)
    REQUIRE(ch.pwm_to_angle() == Catch::Approx(2250.0f)); // half of 4500
}

TEST_CASE("pwm_to_angle with reversed flips the sign", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.reversed = true;
    ch.radio_in = 2000;
    REQUIRE(ch.pwm_to_angle() == Catch::Approx(-4500.0f));
}

TEST_CASE("pwm_to_angle_dz returns 0 within the dead zone around trim", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.dead_zone = 50;
    ch.radio_in = 1520; // within trim +- 50
    REQUIRE(ch.pwm_to_angle() == 0.0f);
    ch.radio_in = 1560; // just outside the dead zone
    REQUIRE(ch.pwm_to_angle() != 0.0f);
}

TEST_CASE("pwm_to_range returns 0 at or below radio_min and high_in at radio_max", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.radio_in = 1000;
    REQUIRE(ch.pwm_to_range() == 0.0f);
    ch.radio_in = 2000;
    REQUIRE(ch.pwm_to_range() == Catch::Approx(4500.0f));
}

TEST_CASE("pwm_to_range interpolates linearly across the full range", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.radio_in = 1500; // halfway between min(1000) and max(2000)
    REQUIRE(ch.pwm_to_range() == Catch::Approx(2250.0f)); // half of 4500
}

TEST_CASE("pwm_to_range with reversed flips which end is 0 vs high_in", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.reversed = true;
    ch.radio_in = 1000; // "low" stick, but reversed -> should read as high_in
    REQUIRE(ch.pwm_to_range() == Catch::Approx(4500.0f));
    ch.radio_in = 2000;
    REQUIRE(ch.pwm_to_range() == Catch::Approx(0.0f));
}

TEST_CASE("get_control_in_zero_dz dispatches by ControlType", "[rc_channel]") {
    RcChannel angle_ch = make_channel();
    angle_ch.type_in = ControlType::kAngle;
    angle_ch.radio_in = 2000;
    REQUIRE(angle_ch.get_control_in_zero_dz() == Catch::Approx(4500.0f));

    RcChannel range_ch = make_channel();
    range_ch.type_in = ControlType::kRange;
    range_ch.radio_in = 1500;
    REQUIRE(range_ch.get_control_in_zero_dz() == Catch::Approx(2250.0f));
}

TEST_CASE("norm_input is 0 at trim, +-1 at min/max", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.radio_in = 1500;
    REQUIRE(ch.norm_input() == 0.0f);
    ch.radio_in = 2000;
    REQUIRE(ch.norm_input() == Catch::Approx(1.0f));
    ch.radio_in = 1000;
    REQUIRE(ch.norm_input() == Catch::Approx(-1.0f));
}

TEST_CASE("norm_input clamps out-of-range radio_in to +-1", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.radio_in = 2500; // beyond radio_max
    REQUIRE(ch.norm_input() == Catch::Approx(1.0f));
}

TEST_CASE("norm_input_dz returns 0 within the dead zone, matches norm_input outside it", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.dead_zone = 100;
    ch.radio_in = 1550; // within trim +- 100
    REQUIRE(ch.norm_input_dz() == 0.0f);

    ch.radio_in = 1750; // outside the dead zone, halfway to max from the dz edge (1600..2000)
    REQUIRE(ch.norm_input_dz() == Catch::Approx(0.375f)); // (1750-1600)/(2000-1600)
}

TEST_CASE("norm_input_ignore_trim ignores trim entirely, using only min/max", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.radio_trim = 1200; // off-center trim - should have NO effect on this function
    ch.radio_in = 1500; // the midpoint of min/max, NOT of trim
    REQUIRE(ch.norm_input_ignore_trim() == Catch::Approx(0.0f));
}

TEST_CASE("norm_input_ignore_trim returns 0 when radio_max <= radio_min (sanity guard)", "[rc_channel]") {
    RcChannel ch = make_channel();
    ch.radio_max = ch.radio_min; // degenerate configuration
    REQUIRE(ch.norm_input_ignore_trim() == 0.0f);
}
