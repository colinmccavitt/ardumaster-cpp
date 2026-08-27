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

// ---------------------------------------------------------------------
// CPP-031 SLICE 11: read_6pos_switch()/debounce_completed() - the 6-
// position mode-switch discretization + debounce state machine. See
// rc_channel.hpp's own file banner for the upstream citation.
// ---------------------------------------------------------------------

TEST_CASE("read_6pos_switch rejects a pulsewidth at or beyond RC_MIN_LIMIT_PWM/RC_MAX_LIMIT_PWM, and leaves "
          "position untouched",
          "[rc_channel][mode_switch]") {
    RcChannel ch;
    std::int8_t position = 42;

    ch.radio_in = 800; // at RC_MIN_LIMIT_PWM - "<=" makes this an error too
    REQUIRE_FALSE(ch.read_6pos_switch(position, 0));
    REQUIRE(position == 42); // untouched - upstream returns false BEFORE computing position

    ch.radio_in = 2200; // at RC_MAX_LIMIT_PWM - ">=" makes this an error too
    REQUIRE_FALSE(ch.read_6pos_switch(position, 0));
    REQUIRE(position == 42);
}

TEST_CASE("read_6pos_switch discretizes PWM into the 6 fixed positions at the exact upstream breakpoints",
          "[rc_channel][mode_switch]") {
    // position is written unconditionally, before the debounce check ever
    // runs (matches upstream: the return value reflects only whether the
    // position has been DEBOUNCED, not whether it was computed) - so the
    // return value is deliberately ignored here; only the discretization
    // itself is under test. Each call uses a fresh channel to avoid any
    // debounce-state interaction with the position value asserted.
    auto position_for = [](std::uint16_t pwm) {
        RcChannel ch;
        ch.radio_in = static_cast<std::int16_t>(pwm);
        std::int8_t position = -1;
        ch.read_6pos_switch(position, 0);
        return position;
    };

    // just below/at/above every one of the 6 breakpoints.
    REQUIRE(position_for(801) == 0);  // just inside the valid range
    REQUIRE(position_for(1230) == 0); // just below the 0/1 breakpoint
    REQUIRE(position_for(1231) == 1); // at the 0/1 breakpoint
    REQUIRE(position_for(1360) == 1); // just below the 1/2 breakpoint
    REQUIRE(position_for(1361) == 2); // at the 1/2 breakpoint
    REQUIRE(position_for(1490) == 2); // just below the 2/3 breakpoint
    REQUIRE(position_for(1491) == 3); // at the 2/3 breakpoint
    REQUIRE(position_for(1620) == 3); // just below the 3/4 breakpoint
    REQUIRE(position_for(1621) == 4); // at the 3/4 breakpoint
    REQUIRE(position_for(1749) == 4); // just below the 4/5 breakpoint
    REQUIRE(position_for(1750) == 5); // at the 4/5 breakpoint
    REQUIRE(position_for(2199) == 5); // just inside the valid range at the top
}

TEST_CASE("debounce_completed requires the FULL debounce window of stability before firing", "[rc_channel][mode_switch]") {
    RcChannel ch;

    // The very first observation of a position is itself "a change" from
    // the -1/-1 default - it only starts the edge timer, never fires
    // immediately.
    REQUIRE_FALSE(ch.debounce_completed(0, 0));   // t=0ms: edge established
    REQUIRE_FALSE(ch.debounce_completed(0, 100)); // t=100ms: 100ms of 200ms elapsed
    REQUIRE_FALSE(ch.debounce_completed(0, 199)); // t=199ms: 1ms short of the full window
    REQUIRE(ch.debounce_completed(0, 200));       // t=200ms: exactly the full window - fires

    // Once settled, staying at the same (now current) position is no
    // longer a "change" to report - returns false forever after, matching
    // upstream: debounce_completed() reports a POSITION-CHANGE EVENT, not
    // a continuous "yes, we're at this position" level.
    REQUIRE_FALSE(ch.debounce_completed(0, 500));
    REQUIRE_FALSE(ch.debounce_completed(0, 100000));
}

TEST_CASE("debounce_completed: a position that changes and changes back within the debounce window never fires",
          "[rc_channel][mode_switch]") {
    RcChannel ch;

    // Settle on position 3 first, far in the past.
    REQUIRE_FALSE(ch.debounce_completed(3, 0));
    REQUIRE(ch.debounce_completed(3, 200));

    // Wobble 3 -> 4 -> 3, all comfortably inside a single 200ms window -
    // the real state machine under test: a naive "N ms since I first saw
    // something different" counter would fire here; the real one must not.
    REQUIRE_FALSE(ch.debounce_completed(4, 300)); // new edge for position 4 (100ms into what would be its window)
    REQUIRE_FALSE(ch.debounce_completed(4, 450)); // still short of 300+200=500
    REQUIRE_FALSE(ch.debounce_completed(3, 480)); // back to 3 BEFORE 4 ever completed - current_position is
                                                   // still 3 (never left), so this is "no change" again, not
                                                   // a fresh edge for 3.
    REQUIRE_FALSE(ch.debounce_completed(3, 100000)); // long afterwards - still nothing NEW happened

    // Confirm position 4, held for the FULL window uninterrupted this
    // time, does eventually fire - proving the earlier non-firing wasn't
    // simply "4 never works".
    REQUIRE_FALSE(ch.debounce_completed(4, 100100)); // new edge
    REQUIRE(ch.debounce_completed(4, 100300));       // full 200ms held - fires
}

TEST_CASE("read_6pos_switch: the debounce gate is real, not just a discretization pass-through",
          "[rc_channel][mode_switch]") {
    RcChannel ch;
    ch.radio_in = 1500; // position 3
    std::int8_t position = -1;

    // Held steady well short of the debounce window - discretizes
    // correctly but does not yet report a real, actionable change.
    REQUIRE_FALSE(ch.read_6pos_switch(position, 0));
    REQUIRE(position == 3);
    REQUIRE_FALSE(ch.read_6pos_switch(position, 199));
    REQUIRE(position == 3);

    // Held the full window - now reports true.
    REQUIRE(ch.read_6pos_switch(position, 200));
    REQUIRE(position == 3);

    // A wobble to another valid position and back within a new window
    // must not fire either - end-to-end through read_6pos_switch(), not
    // just debounce_completed() in isolation.
    ch.radio_in = 900; // position 0
    REQUIRE_FALSE(ch.read_6pos_switch(position, 300));
    REQUIRE(position == 0);
    ch.radio_in = 1500; // back to position 3 before position 0 ever completed
    REQUIRE_FALSE(ch.read_6pos_switch(position, 450));
    REQUIRE(position == 3);

    // An out-of-range PWM observed mid-debounce is a hard error - it does
    // not participate in the position/debounce state machine at all.
    ch.radio_in = 700;
    REQUIRE_FALSE(ch.read_6pos_switch(position, 460));
    REQUIRE(position == 3); // untouched by the error path
}

// ---------------------------------------------------------------------
// CPP-037: read_3pos_switch()/init_position_on_first_radio_read()/
// read_aux() - the 3-position aux-switch decode mechanism. A SEPARATE
// state machine from read_6pos_switch()/debounce_completed() above -
// see rc_channel.hpp's own "CPP-037 ADDENDUM" file banner.
// ---------------------------------------------------------------------

TEST_CASE("read_3pos_switch rejects a pulsewidth at or beyond RC_MIN_LIMIT_PWM/RC_MAX_LIMIT_PWM",
          "[rc_channel][aux]") {
    RcChannel ch;
    AuxSwitchPos pos = AuxSwitchPos::kMiddle;

    ch.radio_in = 800; // at RC_MIN_LIMIT_PWM
    REQUIRE_FALSE(ch.read_3pos_switch(pos));
    ch.radio_in = 2200; // at RC_MAX_LIMIT_PWM
    REQUIRE_FALSE(ch.read_3pos_switch(pos));
}

TEST_CASE("read_3pos_switch discretizes PWM into LOW/MIDDLE/HIGH at the exact upstream breakpoints (1200/1800)",
          "[rc_channel][aux]") {
    RcChannel ch;
    AuxSwitchPos pos;

    ch.radio_in = 1199;
    REQUIRE(ch.read_3pos_switch(pos));
    REQUIRE(pos == AuxSwitchPos::kLow);

    ch.radio_in = 1200; // AUX_SWITCH_PWM_TRIGGER_LOW itself is NOT low (strict <)
    REQUIRE(ch.read_3pos_switch(pos));
    REQUIRE(pos == AuxSwitchPos::kMiddle);

    ch.radio_in = 1500;
    REQUIRE(ch.read_3pos_switch(pos));
    REQUIRE(pos == AuxSwitchPos::kMiddle);

    ch.radio_in = 1800; // AUX_SWITCH_PWM_TRIGGER_HIGH itself is NOT high (strict >)
    REQUIRE(ch.read_3pos_switch(pos));
    REQUIRE(pos == AuxSwitchPos::kMiddle);

    ch.radio_in = 1801;
    REQUIRE(ch.read_3pos_switch(pos));
    REQUIRE(pos == AuxSwitchPos::kHigh);
}

TEST_CASE("init_position_on_first_radio_read: only ArmDisarm is suppressed - every other real AuxFunc this port "
          "defines fires normally on its first stable read",
          "[rc_channel][aux]") {
    REQUIRE(RcChannel::init_position_on_first_radio_read(AuxFunc::ArmDisarm));

    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::DoNothing));
    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::Rtl));
    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::Auto));
    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::Manual));
    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::Loiter));
    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::Takeoff));
    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::Fbwa));
    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::ModeSwitchReset));
    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::Cruise));
    REQUIRE_FALSE(RcChannel::init_position_on_first_radio_read(AuxFunc::EmergencyLandingEn));
}

TEST_CASE("read_aux: a DoNothing-option channel never reports a change, however the PWM moves", "[rc_channel][aux]") {
    RcChannel ch;
    REQUIRE(ch.option == AuxFunc::DoNothing); // the documented default
    ch.radio_in = 1900;
    REQUIRE_FALSE(ch.read_aux(0).has_value());
    REQUIRE_FALSE(ch.read_aux(1000000).has_value());
}

TEST_CASE("read_aux: a non-ARM-type function's STARTING position still requires the normal debounce window before "
          "firing once - no first-read suppression for it",
          "[rc_channel][aux]") {
    RcChannel ch;
    ch.option = AuxFunc::Fbwa;
    ch.radio_in = 1900; // HIGH from the very first read

    REQUIRE_FALSE(ch.read_aux(0).has_value());   // edge just established
    REQUIRE_FALSE(ch.read_aux(199).has_value()); // 1ms short
    const std::optional<AuxSwitchPos> pos = ch.read_aux(200);
    REQUIRE(pos.has_value());
    REQUIRE(*pos == AuxSwitchPos::kHigh);

    // Not reported again while unchanged (a position-change EVENT, not a
    // level query - same contract as read_6pos_switch() above).
    REQUIRE_FALSE(ch.read_aux(500).has_value());
}

TEST_CASE("read_aux: an ARM-type function (ArmDisarm) starting HIGH on the very first read NEVER fires for that "
          "starting position, even after the normal debounce window elapses - the real 'do not arm on power-up "
          "with the switch already high' suppression",
          "[rc_channel][aux]") {
    RcChannel ch;
    ch.option = AuxFunc::ArmDisarm;
    ch.radio_in = 1900; // HIGH from the very first read

    // Unlike the non-ARM-type test above, this must stay false forever
    // at this position - the baseline was silently adopted, not merely
    // delayed by debounce.
    REQUIRE_FALSE(ch.read_aux(0).has_value());
    REQUIRE_FALSE(ch.read_aux(200).has_value());
    REQUIRE_FALSE(ch.read_aux(1000).has_value());
    REQUIRE_FALSE(ch.read_aux(1000000).has_value());

    // A genuine CHANGE away from that adopted baseline, once, DOES fire
    // after the normal debounce window - the suppression only ever
    // applies to the one starting position, not to the function forever.
    ch.radio_in = 1000; // LOW
    REQUIRE_FALSE(ch.read_aux(1000020).has_value());
    const std::optional<AuxSwitchPos> pos = ch.read_aux(1000220);
    REQUIRE(pos.has_value());
    REQUIRE(*pos == AuxSwitchPos::kLow);
}

TEST_CASE("read_aux: an ARM-type function starting MIDDLE also adopts MIDDLE as its suppressed baseline, not just "
          "HIGH - the suppression is keyed on 'the starting position', not a hardcoded HIGH",
          "[rc_channel][aux]") {
    RcChannel ch;
    ch.option = AuxFunc::ArmDisarm;
    ch.radio_in = 1500; // MIDDLE from the very first read
    REQUIRE_FALSE(ch.read_aux(0).has_value());
    REQUIRE_FALSE(ch.read_aux(1000).has_value());

    // Moving to HIGH is a real change away from the adopted MIDDLE
    // baseline and fires normally once debounced.
    ch.radio_in = 1900;
    REQUIRE_FALSE(ch.read_aux(1020).has_value());
    const std::optional<AuxSwitchPos> pos = ch.read_aux(1220);
    REQUIRE(pos.has_value());
    REQUIRE(*pos == AuxSwitchPos::kHigh);
}

