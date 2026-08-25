// Tests for fwcpp::srv::SrvChannel (CPP-027, servo-output-mapping slice).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/srv/srv_channel.hpp>

using namespace fwcpp::srv;

namespace {
SrvChannel make_angle_channel() {
    SrvChannel ch;
    ch.servo_min = 1000;
    ch.servo_trim = 1500;
    ch.servo_max = 2000;
    ch.reversed = false;
    ch.set_angle(4500);
    return ch;
}

SrvChannel make_range_channel() {
    SrvChannel ch;
    ch.servo_min = 1000;
    ch.servo_trim = 1500;
    ch.servo_max = 2000;
    ch.reversed = false;
    ch.set_range(4500);
    return ch;
}
} // namespace

// ---- pwm_from_angle ---------------------------------------------------

TEST_CASE("pwm_from_angle returns trim at 0", "[srv_channel][angle]") {
    SrvChannel ch = make_angle_channel();
    REQUIRE(ch.pwm_from_angle(0.0f) == 1500);
}

TEST_CASE("pwm_from_angle returns servo_max at +high_out and servo_min at -high_out", "[srv_channel][angle]") {
    SrvChannel ch = make_angle_channel();
    REQUIRE(ch.pwm_from_angle(4500.0f) == 2000);
    REQUIRE(ch.pwm_from_angle(-4500.0f) == 1000);
}

TEST_CASE("pwm_from_angle interpolates linearly on each side of trim", "[srv_channel][angle]") {
    SrvChannel ch = make_angle_channel();
    REQUIRE(ch.pwm_from_angle(2250.0f) == 1750);  // halfway trim->max
    REQUIRE(ch.pwm_from_angle(-2250.0f) == 1250); // halfway trim->min
}

TEST_CASE("pwm_from_angle clamps beyond +-high_out", "[srv_channel][angle]") {
    SrvChannel ch = make_angle_channel();
    REQUIRE(ch.pwm_from_angle(9000.0f) == 2000);
    REQUIRE(ch.pwm_from_angle(-9000.0f) == 1000);
}

TEST_CASE("pwm_from_angle with reversed flips the sign (not the value-space)", "[srv_channel][angle]") {
    SrvChannel ch = make_angle_channel();
    ch.reversed = true;
    // A sign flip on a domain symmetric about 0 sends +high_out to
    // servo_min and -high_out to servo_max - the mirror image of the
    // non-reversed case.
    REQUIRE(ch.pwm_from_angle(4500.0f) == 1000);
    REQUIRE(ch.pwm_from_angle(-4500.0f) == 2000);
    // trim is the fixed point of a sign flip, so it is untouched.
    REQUIRE(ch.pwm_from_angle(0.0f) == 1500);
}

TEST_CASE("pwm_from_angle with high_out == 0 returns trim (divide-by-zero guard)", "[srv_channel][angle]") {
    SrvChannel ch = make_angle_channel();
    ch.high_out = 0;
    REQUIRE(ch.pwm_from_angle(1234.0f) == 1500);
    REQUIRE(ch.pwm_from_angle(-1234.0f) == 1500);
}

// ---- pwm_from_range -----------------------------------------------------

TEST_CASE("pwm_from_range returns servo_min at 0 and servo_max at high_out", "[srv_channel][range]") {
    SrvChannel ch = make_range_channel();
    REQUIRE(ch.pwm_from_range(0.0f) == 1000);
    REQUIRE(ch.pwm_from_range(4500.0f) == 2000);
}

TEST_CASE("pwm_from_range interpolates linearly across the full range", "[srv_channel][range]") {
    SrvChannel ch = make_range_channel();
    REQUIRE(ch.pwm_from_range(2250.0f) == 1500); // halfway
}

TEST_CASE("pwm_from_range clamps below 0 and above high_out", "[srv_channel][range]") {
    SrvChannel ch = make_range_channel();
    REQUIRE(ch.pwm_from_range(-100.0f) == 1000);
    REQUIRE(ch.pwm_from_range(9000.0f) == 2000);
}

TEST_CASE("pwm_from_range with reversed flips which end is servo_min vs servo_max (value-space flip)", "[srv_channel][range]") {
    SrvChannel ch = make_range_channel();
    ch.reversed = true;
    // Reversed on a 0..high_out domain reflects through the domain's
    // midpoint (high_out - v), NOT a sign flip - a sign flip would send
    // scaled_value negative and be clamped straight back to 0, which
    // would make "reversed" on a range channel a no-op. This is the
    // property that would catch a sign-flip-instead-of-value-space-flip
    // transcription error.
    REQUIRE(ch.pwm_from_range(0.0f) == 2000);
    REQUIRE(ch.pwm_from_range(4500.0f) == 1000);
    REQUIRE(ch.pwm_from_range(2250.0f) == 1500); // midpoint is its own mirror
}

TEST_CASE("pwm_from_range with servo_max <= servo_min returns servo_min (degenerate guard)", "[srv_channel][range]") {
    SrvChannel ch = make_range_channel();
    ch.servo_max = ch.servo_min; // degenerate configuration
    REQUIRE(ch.pwm_from_range(4500.0f) == ch.servo_min);
}

TEST_CASE("pwm_from_range with high_out == 0 returns servo_min (divide-by-zero guard)", "[srv_channel][range]") {
    SrvChannel ch = make_range_channel();
    ch.high_out = 0;
    REQUIRE(ch.pwm_from_range(123.0f) == ch.servo_min);
}

// ---- pwm_from_scaled_value dispatch --------------------------------------

TEST_CASE("pwm_from_scaled_value dispatches by type_angle", "[srv_channel]") {
    SrvChannel angle_ch = make_angle_channel();
    REQUIRE(angle_ch.pwm_from_scaled_value(4500.0f) == 2000);

    SrvChannel range_ch = make_range_channel();
    REQUIRE(range_ch.pwm_from_scaled_value(4500.0f) == 2000);
    REQUIRE(range_ch.pwm_from_scaled_value(0.0f) == 1000); // would be trim(1500) if mis-dispatched to angle
}

// ---- calc_pwm / set_output_pwm / get_output_pwm --------------------------

TEST_CASE("calc_pwm stores the scaled-to-pwm conversion into output_pwm", "[srv_channel]") {
    SrvChannel ch = make_angle_channel();
    ch.calc_pwm(4500.0f);
    REQUIRE(ch.get_output_pwm() == 2000);
}

TEST_CASE("set_output_pwm/get_output_pwm round-trip a raw pwm value directly", "[srv_channel]") {
    SrvChannel ch = make_angle_channel();
    ch.set_output_pwm(1732);
    REQUIRE(ch.get_output_pwm() == 1732);
}

// ---- get_output_norm ------------------------------------------------------

TEST_CASE("get_output_norm is 0 at the servo_min/servo_max midpoint", "[srv_channel]") {
    SrvChannel ch = make_angle_channel();
    ch.set_output_pwm(1500); // (1000+2000)/2 == 1500
    REQUIRE(ch.get_output_norm() == 0.0f);
}

TEST_CASE("get_output_norm is +-1 at servo_max/servo_min", "[srv_channel]") {
    SrvChannel ch = make_angle_channel();
    ch.set_output_pwm(2000);
    REQUIRE(ch.get_output_norm() == Catch::Approx(1.0f));
    ch.set_output_pwm(1000);
    REQUIRE(ch.get_output_norm() == Catch::Approx(-1.0f));
}

TEST_CASE("get_output_norm flips sign when reversed", "[srv_channel]") {
    SrvChannel ch = make_angle_channel();
    ch.reversed = true;
    ch.set_output_pwm(2000);
    REQUIRE(ch.get_output_norm() == Catch::Approx(-1.0f));
    ch.set_output_pwm(1000);
    REQUIRE(ch.get_output_norm() == Catch::Approx(1.0f));
}

TEST_CASE("get_output_norm returns 0 when servo_max <= servo_min (degenerate guard)", "[srv_channel]") {
    SrvChannel ch = make_angle_channel();
    ch.servo_max = ch.servo_min;
    ch.set_output_pwm(1999);
    REQUIRE(ch.get_output_norm() == 0.0f);
}

// A property that would catch a sign error in EITHER of the two
// independent places `reversed` is applied (the sign flip inside
// pwm_from_angle, or the sign flip inside get_output_norm): with
// servo_trim sitting exactly at the servo_min/servo_max midpoint (as
// make_angle_channel sets up), pwm_from_angle's scaled_value -> pwm
// mapping and get_output_norm's pwm -> norm mapping are each linear and
// odd about that midpoint, so applying the SAME reversed flip in both
// places cancels out over the full round trip - set_output_norm(x)
// followed by get_output_norm() reproduces x regardless of reversed. If
// either flip were dropped, wrongly negated, or applied in only one of
// the two functions, this round trip would break for reversed == true
// while still passing for reversed == false.
TEST_CASE("set_output_norm followed by get_output_norm round-trips through pwm regardless of reversed", "[srv_channel]") {
    for (const bool rev : {false, true}) {
        SrvChannel ch = make_angle_channel();
        ch.reversed = rev;
        ch.set_output_norm(1.0f); // high_out(4500) * 1.0 -> pwm_from_angle(4500)
        REQUIRE(ch.get_output_norm() == Catch::Approx(1.0f).margin(0.01f));

        ch.set_output_norm(-1.0f);
        REQUIRE(ch.get_output_norm() == Catch::Approx(-1.0f).margin(0.01f));
    }
}

// ---- get_limit_pwm ----------------------------------------------------

TEST_CASE("get_limit_pwm returns trim/min/max/zero, honoring reversed for min/max", "[srv_channel]") {
    SrvChannel ch = make_angle_channel();
    REQUIRE(ch.get_limit_pwm(Limit::kTrim) == 1500);
    REQUIRE(ch.get_limit_pwm(Limit::kMin) == 1000);
    REQUIRE(ch.get_limit_pwm(Limit::kMax) == 2000);
    REQUIRE(ch.get_limit_pwm(Limit::kZeroPwm) == 0);

    ch.reversed = true;
    // Reversed swaps which physical PWM endpoint counts as "min" vs "max".
    REQUIRE(ch.get_limit_pwm(Limit::kMin) == 2000);
    REQUIRE(ch.get_limit_pwm(Limit::kMax) == 1000);
    REQUIRE(ch.get_limit_pwm(Limit::kTrim) == 1500); // trim is unaffected by reversed
}

// ---- get_reversed / function tagging -----------------------------------

TEST_CASE("get_reversed reflects the reversed field", "[srv_channel]") {
    SrvChannel ch = make_angle_channel();
    REQUIRE_FALSE(ch.get_reversed());
    ch.reversed = true;
    REQUIRE(ch.get_reversed());
}

TEST_CASE("function defaults to kNone and can be tagged directly", "[srv_channel]") {
    SrvChannel ch = make_angle_channel();
    REQUIRE(ch.function == Function::kNone);
    ch.function = Function::kAileron;
    REQUIRE(ch.function == Function::kAileron);
}
