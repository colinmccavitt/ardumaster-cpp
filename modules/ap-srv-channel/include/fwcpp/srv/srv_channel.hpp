#pragma once

// Port of SRV_Channel/SRV_Channel.h + SRV_Channel.cpp's core value-
// conversion math: pwm_from_range/pwm_from_angle/pwm_from_scaled_value
// (scaled control-law output -> PWM microseconds) and get_output_norm
// (PWM -> normalized -1..1, the closest thing upstream has to an inverse -
// see the note on get_output_norm below for why there is no general
// pwm-to-scaled-value inverse). CPP-027 ("RC_Channel / SRV_Channel: RC
// input decoding and servo output mapping"), servo-output-mapping slice.
//
// SLICE BOUNDARY: this is the pure per-channel scaling math - given one
// channel's own trim/min/max/reversed/high_out/type_angle state (plus,
// same as upstream, a currently-held output_pwm value), convert a scaled
// control-law output to a PWM value and back to a normalized reading.
// Deliberately NOT in this slice, same shape of exclusion RcChannel (see
// fwcpp/rc/rc_channel.hpp) applies to RC_Channels:
//
//   - SRV_Channels (plural): the vehicle-wide array of channels, function
//     assignment/lookup (which physical channel(s) currently have
//     SERVOn_FUNCTION set to k_aileron?), output_ch()'s per-function
//     dispatch loop, AP_Volz/RobotisServo/SBusOut/BLHeli/FETtec protocol
//     backends, and the AP::srv_channels() singleton that owns all of it.
//     This port has a standing no-singleton rule; a caller that needs
//     "find every channel serving function X" owns its own
//     std::array<SrvChannel, N> (or similar) plus each element's
//     `function` field below, and scans it directly - there is no
//     registry object to port, just a loop upstream also expresses as a
//     loop (SRV_Channels::function_channel* helpers), applied to
//     caller-owned storage instead of a global array.
//   - E-stop (SRV_Channels::emergency_stop) and RC-passthrough override
//     (override_active/have_pwm_mask) bookkeeping in calc_pwm/
//     set_output_pwm: both are vehicle-wide policy decisions layered on
//     top of the scaling math, not the scaling math itself. calc_pwm
//     below reproduces only the `output_pwm = pwm_from_scaled_value(...)`
//     line; a caller wanting e-stop/override semantics implements that
//     policy itself and simply doesn't call calc_pwm when it applies.
//   - is_motor/should_e_stop/is_control_surface: these classify a
//     *function number* (not a channel's scaling state) against the full
//     ~190-entry Function enum. Function below intentionally carries only
//     a representative handful of common fixed-wing values (exact
//     upstream numeric values preserved, in case a later slice needs
//     wire/param compatibility) to demonstrate the function-tagging
//     pattern per-channel; porting the full enum plus its classification
//     helpers is separate follow-on work, not scaling math.
//   - aux_servo_function_setup(): upstream's function-number -> (type_angle,
//     high_out) defaulting table. Out of scope for the same reason as
//     RcChannel's ControlType: this port takes type_angle/high_out as
//     values the caller sets directly via set_angle()/set_range(),
//     exactly mirroring upstream's own public set_angle()/set_range() API
//     (which aux_servo_function_setup itself calls into).
//
// AP_Int16 REPLACED WITH PLAIN std::int16_t for servo_min/servo_max/
// servo_trim, and AP_Int8 (0/1 convention) REPLACED WITH PLAIN bool for
// reversed - same AP_Param-not-wired-in-yet precedent as RcChannel's
// radio_min/max/trim and AcPid::Gains (AP_Param itself is ported at
// ap-param, but no controller in this port has been wired to it yet;
// that wiring is a caller-side concern per that module's own file banner).
//
// REGISTERED DIVERGENCE (UB avoidance, not a behavior change for any
// correctly-configured channel): upstream computes the offset-from-
// trim/min term in pwm_from_range/pwm_from_angle as `uint16_t(expr)` and
// adds/subtracts it from an AP_Int16, e.g.
// `servo_min + uint16_t((scaled_value * (servo_max - servo_min)) / high_out)`.
// For a correctly-configured channel (servo_min <= servo_trim <=
// servo_max) `expr` is always >= 0, so the cast is harmless. But if a
// channel is misconfigured with servo_trim < servo_min or servo_trim >
// servo_max, `expr` can go negative, and casting a negative float
// directly to an unsigned integer type is undefined behavior in C++
// (unlike out-of-range *integral*-to-unsigned conversions, which are
// well-defined modulo wraparound). This port computes the same
// expression in std::int32_t (wide enough that the intermediate
// float-to-int cast is always in-range for any PWM-scale input) and only
// narrows to std::uint16_t at the very end, where the conversion is
// integral-to-unsigned and therefore well-defined. Bit-identical result
// to upstream for every correctly-configured channel; the only difference
// is what a misconfigured one does, and "defined wraparound" is
// preferable there to "sanitizer-flagged UB" without upstream ever having
// depended on the UB's specific behavior.

#include <cstdint>

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::srv {

// A representative slice of upstream's ~190-entry SRV_Channel::Function
// enum (SRV_Channel.h) - just enough named, fixed-wing-relevant values to
// demonstrate the "this channel is tagged with a function" pattern that
// the (out-of-scope, see file banner) SRV_Channels registry uses to find
// channels by function. Numeric values are copied verbatim from upstream,
// not renumbered, in case a later slice needs SERVOn_FUNCTION wire/param
// compatibility.
enum class Function : std::int16_t {
    kGpio = -1,
    kNone = 0,
    kManual = 1,
    kFlap = 2,
    kFlapAuto = 3, // CPP-038 addition - upstream: k_flap_auto (SRV_Channel.h), grepped directly, not renumbered.
    kAileron = 4,
    kFlaperonLeft = 24,
    kFlaperonRight = 25,
    kElevator = 19,
    kRudder = 21,
    kSteering = 26,
    kThrottle = 70,
    kElevonLeft = 77,
    kElevonRight = 78,
    kMin = 134,   // always outputs SERVOn_MIN
    kTrim = 135,  // always outputs SERVOn_TRIM
    kMax = 136,   // always outputs SERVOn_MAX
};

// Mirrors SRV_Channel::Limit - used to fetch a fixed PWM (min/max/trim/0)
// for a channel, honoring reversed the same way get_limit_pwm does (a
// reversed channel's "min" output is its servo_max PWM and vice versa).
enum class Limit : std::uint8_t {
    kTrim,
    kMin,
    kMax,
    kZeroPwm,
};

class SrvChannel {
public:
    std::int16_t servo_min = 1100;
    std::int16_t servo_max = 1900;
    std::int16_t servo_trim = 1500;
    bool reversed = false;

    // Which function this channel is currently tagged with. Purely
    // descriptive at this slice's scope - see the SRV_Channels exclusion
    // above for why nothing here searches by it.
    Function function = Function::kNone;

    // true for angle output type (set_angle()), false for range output
    // type (set_range()) - upstream's type_angle bitfield.
    bool type_angle = true;

    // High point of angle (symmetric -high_out..high_out) or range
    // (0..high_out) output, set via set_angle()/set_range().
    std::uint16_t high_out = 0;

    // The channel's currently-held output, in PWM microseconds - upstream
    // SRV_Channel::output_pwm. Written by calc_pwm()/set_output_pwm(),
    // read back by get_output_pwm()/get_output_norm().
    std::uint16_t output_pwm = 0;

    // set_angle()/set_range(): upstream's public API for configuring
    // output shape, called either directly by a caller (lua scripts, GCS
    // do-set-servo handling) or by the out-of-scope
    // aux_servo_function_setup() table lookup. angle's parameter stays
    // int16_t (matching upstream exactly) even though high_out is
    // uint16_t - upstream relies on the implicit conversion because every
    // real angle value (e.g. ANGLE_MAX = 4500) is positive; reproduced as-is
    // rather than "fixed" to uint16_t, since a signed parameter matches
    // every real call site upstream has.
    void set_angle(std::int16_t angle) {
        type_angle = true;
        high_out = static_cast<std::uint16_t>(angle);
    }

    void set_range(std::uint16_t high) {
        type_angle = false;
        high_out = high;
    }

    [[nodiscard]] bool get_reversed() const { return reversed; }

    // convert a 0..high_out scaled value to a pwm (SRV_Channel::pwm_from_range).
    //
    // reversed is applied as a VALUE-SPACE FLIP here (scaled_value ->
    // high_out - scaled_value) rather than a sign flip. This is not an
    // inconsistency with pwm_from_angle below using a sign flip - it is
    // required by the different value domains: range values live in
    // 0..high_out, so negating one would leave [0, high_out] range and
    // land outside servo_min..servo_max on the wrong side, or go negative
    // pre-constrain; reflecting through the domain's midpoint is the only
    // flip that keeps the result in 0..high_out. (Same family of
    // per-function-type reversed handling RcChannel documents for
    // pwm_to_range_dz vs pwm_to_angle_dz.)
    [[nodiscard]] std::uint16_t pwm_from_range(float scaled_value) const {
        if (servo_max <= servo_min || high_out == 0) {
            // Degenerate configuration: upstream collapses to servo_min
            // rather than dividing by (servo_max - servo_min) or high_out.
            return static_cast<std::uint16_t>(servo_min);
        }
        scaled_value = math::constrain_value(scaled_value, 0.0f, static_cast<float>(high_out));
        if (reversed) {
            scaled_value = static_cast<float>(high_out) - scaled_value;
        }
        return static_cast<std::uint16_t>(
            servo_min + static_cast<std::int32_t>((scaled_value * static_cast<float>(servo_max - servo_min)) / static_cast<float>(high_out)));
    }

    // convert a -high_out..high_out scaled value to a pwm (SRV_Channel::pwm_from_angle).
    //
    // reversed is applied as a SIGN FLIP here (scaled_value -> -scaled_value),
    // not a value-space flip. Angle's domain is symmetric about zero, so a
    // sign flip and reflecting through the domain's midpoint (0) are the
    // same operation here - unlike pwm_from_range above, where the domain
    // (0..high_out) is not symmetric about zero and a sign flip would be
    // wrong. See pwm_from_range's comment for the full reasoning; this is
    // the same upstream pattern RcChannel found between its own
    // angle-shaped and range-shaped conversions, reproduced deliberately.
    [[nodiscard]] std::uint16_t pwm_from_angle(float scaled_value) const {
        if (high_out == 0) {
            // Degenerate configuration: upstream returns trim (the
            // channel's neutral position) rather than dividing by high_out.
            return static_cast<std::uint16_t>(servo_trim);
        }
        if (reversed) {
            scaled_value = -scaled_value;
        }
        scaled_value = math::constrain_value(scaled_value, -static_cast<float>(high_out), static_cast<float>(high_out));
        if (scaled_value > 0.0f) {
            return static_cast<std::uint16_t>(
                servo_trim + static_cast<std::int32_t>((scaled_value * static_cast<float>(servo_max - servo_trim)) / static_cast<float>(high_out)));
        }
        return static_cast<std::uint16_t>(
            servo_trim - static_cast<std::int32_t>((-scaled_value * static_cast<float>(servo_trim - servo_min)) / static_cast<float>(high_out)));
    }

    // Dispatches to pwm_from_range or pwm_from_angle depending on
    // type_angle - SRV_Channel::pwm_from_scaled_value.
    [[nodiscard]] std::uint16_t pwm_from_scaled_value(float scaled_value) const {
        return type_angle ? pwm_from_angle(scaled_value) : pwm_from_range(scaled_value);
    }

    // SRV_Channel::calc_pwm, minus the e-stop/override-active bookkeeping
    // (see file banner) - stores the pwm computed from a scaled control-law
    // output into output_pwm.
    void calc_pwm(float output_scaled) { output_pwm = pwm_from_scaled_value(output_scaled); }

    void set_output_pwm(std::uint16_t pwm) { output_pwm = pwm; }

    [[nodiscard]] std::uint16_t get_output_pwm() const { return output_pwm; }

    // set normalised output from -1 to 1, assuming 0 at mid point of
    // servo_min/servo_max - SRV_Channel::set_output_norm. Scales by
    // high_out first exactly like upstream (this only produces a genuinely
    // normalized -1..1 *input* when high_out == 1; for a real angle/range
    // channel this is upstream's own actual behavior - value*high_out is
    // then run back through the type's own scaling, not a bug introduced
    // here).
    void set_output_norm(float value) { set_output_pwm(pwm_from_scaled_value(value * static_cast<float>(high_out))); }

    // get normalised output from -1 to 1, assuming 0 at mid point of
    // servo_min/servo_max - SRV_Channel::get_output_norm.
    //
    // This is the closest thing upstream has to an "inverse" of
    // pwm_from_range/pwm_from_angle, and it is deliberately NOT their
    // exact inverse: it always maps through the servo_min/servo_max
    // midpoint to a -1..1 value regardless of type_angle/high_out/
    // servo_trim, rather than solving pwm_from_angle or pwm_from_range
    // backwards for scaled_value. Upstream provides no general
    // pwm-to-scaled-value inverse (no "angle_from_pwm"/"range_from_pwm");
    // this is the only pwm-reading function it has, reproduced as-is.
    [[nodiscard]] float get_output_norm() const {
        const std::int32_t mid = (static_cast<std::int32_t>(servo_max) + static_cast<std::int32_t>(servo_min)) / 2;
        if (mid <= servo_min) {
            // Degenerate configuration (servo_max <= servo_min): avoid
            // dividing by (mid - servo_min) below, which would be <= 0.
            return 0.0f;
        }
        float ret;
        if (static_cast<std::int32_t>(output_pwm) < mid) {
            ret = static_cast<float>(static_cast<std::int32_t>(output_pwm) - mid) / static_cast<float>(mid - servo_min);
        } else if (static_cast<std::int32_t>(output_pwm) > mid) {
            ret = static_cast<float>(static_cast<std::int32_t>(output_pwm) - mid) / static_cast<float>(servo_max - mid);
        } else {
            ret = 0.0f;
        }
        if (reversed) {
            ret = -ret;
        }
        return ret;
    }

    // Fixed PWM for a given Limit, honoring reversed the same way a
    // reversed channel's "min" is its servo_max PWM (SRV_Channel::get_limit_pwm).
    [[nodiscard]] std::uint16_t get_limit_pwm(Limit limit) const {
        switch (limit) {
        case Limit::kTrim:
            return static_cast<std::uint16_t>(servo_trim);
        case Limit::kMin:
            return static_cast<std::uint16_t>(reversed ? servo_max : servo_min);
        case Limit::kMax:
            return static_cast<std::uint16_t>(reversed ? servo_min : servo_max);
        case Limit::kZeroPwm:
            return 0;
        }
        return 0;
    }
};

} // namespace fwcpp::srv
