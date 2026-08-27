#pragma once

// Port of RC_Channel/RC_Channel.h + RC_Channel.cpp's PWM<->normalized-
// value conversion core: pwm_to_angle*/pwm_to_range*/norm_input*. CPP-030
// slice 1.
//
// SLICE BOUNDARY: this is the pure conversion math - given a channel's
// own radio_in/min/max/trim/dead_zone/reversed state, produce an angle
// (centidegrees, e.g. -4500..4500 for a control surface), a range value
// (0..high_in), or a normalized -1..1 value. Deliberately NOT in this
// slice: RC_Channels (the vehicle-wide array of channels, aux function
// mapping, has_valid_input()/failsafe detection - a real subsystem of
// its own), and norm_input_ignore_trim's bool-returning overload (needs
// rc().has_valid_input(), i.e. that same not-yet-ported subsystem).
//
// AP_Int16 REPLACED WITH PLAIN std::int16_t for radio_min/max/trim -
// same AP_Param-not-wired-in-yet precedent used throughout this port
// (AcPid::Gains, L1Control::Gains) until a caller actually wires AP_Param
// in.
//
// LITERAL SAFETY: no bare ambiguous double literals - every constant is
// an explicit float-suffixed literal, matching upstream's own.
//
// CPP-027 slice: added ch_in/control_in (RC_Channel.h:521,529) and
// update() (RC_Channel.cpp:~303) so fwcpp::rc::RcChannels (rc_channels.hpp,
// same module) can drive a channel from an fwcpp::hal::RcInput. update()
// here deliberately takes the already-read PWM value as a parameter
// rather than pulling from hal.rcin itself: this keeps RcChannel free of
// any HAL dependency (it stays pure per-channel math, as it was before
// this slice) and matches this port's explicit-context convention - the
// registry (which does depend on ap-hal) owns "where does the PWM value
// come from" (RcInput::read(ch_in), or in upstream's case also GCS
// overrides - see rc_channels.hpp's banner for why overrides aren't
// reproduced here). Functionally this is upstream's update() with the
// has_override()/has_had_rc_receiver() gate hoisted to the caller: by
// the time a caller has a PWM value in hand to pass in, upstream would
// already have decided to update, so update() always returns true here.
//
// CPP-031 SLICE 11: added the 6-position mode-switch discretization +
// debounce state machine - upstream: RC_Channel.h's `switch_state`
// (private, RC_Channels/RC_Channel's own "auxiliary switches" section)
// plus RC_Channel.cpp's read_6pos_switch() (~line 596) and
// debounce_completed() (~line 636), both read in full. This is
// deliberately the RAW per-channel primitive only - the orchestration
// that ties it to an actual mode change (upstream's RC_Channel::
// read_mode_switch()/mode_switch_changed(), and RC_Channels::
// read_mode_switch()/flight_mode_channel()) lives one level up, in
// RcChannels::read_mode_switch() (rc_channels.hpp, same module) - see
// that file's own banner for why the split is drawn there instead of
// here (this port has no vehicle-specific RC_Channel subclass to hang a
// virtual mode_switch_changed() off of, so the dispatch step moved to
// the one class that DOES already know which channel is the mode
// switch).
//
// now_ms IS AN EXPLICIT PARAMETER (ADR-0012, matching every other
// now_ms-taking method in this port) rather than upstream's own
// AP_HAL::millis() singleton read inside debounce_completed().
//
// CPP-037: reset_mode_switch() (formerly "not ported here", see the OLD
// note this replaces) is now ported - as RcChannels::reset_mode_switch()
// (rc_channels.hpp, same module), not here. Upstream's own RC_Channel::
// reset_mode_switch() (RC_Channel.cpp ~line 588, read in full) is a tiny
// two-field reset plus an immediate re-read; the "re-read" half needs the
// flight-mode-channel RESOLUTION step (RcChannels::flight_mode_channel())
// this class doesn't have, so - exactly like read_mode_switch() above -
// the whole thing collapsed one level up rather than being split across
// two classes for no reason.
//
// CPP-037 ADDENDUM: the 3-position aux-switch decode mechanism - a
// SEPARATE state machine from read_6pos_switch()/debounce_completed()
// above, sharing only the debounce ALGORITHM (debounce_completed() is
// reused as-is; a real RC channel is configured as either the flight-mode
// switch OR an aux-function switch, never both, so switch_state's single
// set of fields never actually double-books between the two callers -
// verified directly against upstream, which draws exactly this same
// distinction and also reuses one switch_state for both). Upstream:
// RC_Channel::read_3pos_switch() (RC_Channel.cpp ~line 2031),
// RC_Channel::init_position_on_first_radio_read() (~line 1030), and
// RC_Channel::read_aux() (~line 976) - all three read in full. AUX_FUNC/
// AuxSwitchPos: RC_Channel.h's real enums (~300+ values / 3 positions),
// grepped directly.
//
// AuxFunc BELOW IS NOT A FULL PORT OF UPSTREAM'S AUX_FUNC (RC_Channel.h) -
// that enum spans 300+ values across nearly every ArduPilot vehicle type
// and optional subsystem (camera/gripper/sprayer/generator/RunCam/
// quadplane/ADSB-avoidance/soaring/terrain/relays/fence/mission-reset/
// RC-override-enable/FFT-tune/mount/VTX/inverted-flight/reverse-throttle/
// airbrake/EKF-source/compass-learn/... - all real, all absent from this
// enum on purpose, none of their backing subsystems exist in this port
// (CPP-038 addendum: FLAP is the one exception - added below once a real
// consumer, Plane::set_servos_flaps(), needed it). This ticket ports ONLY
// the handful of values with a real dispatch target today (see fwcpp::
// vehicle::Plane::dispatch_aux_function()'s own file banner, plane.hpp,
// for the full named exclusion
// list and per-value upstream trace) - each kept at its REAL upstream
// numeric value (RC_Channel.h, grepped directly) so a value here never
// silently means something different than it does upstream, even though
// the enum itself is a small subset rather than an exhaustive mirror
// (unlike the small, ~5-6-value FsActionShort/FsActionLong enums
// elsewhere in this port, which ARE exhaustive - AUX_FUNC's real size
// makes that impractical and the ticket does not ask for it).
//
// init_position_on_first_radio_read()'s REAL, NARROW suppression set:
// upstream's own switch covers ARMDISARM_AIRMODE/ARMDISARM/ARM_EMERGENCY_
// STOP (all AP_ARMING_ENABLED-gated) and PARACHUTE_RELEASE (HAL_
// PARACHUTE_ENABLED-gated) - RC_Channel.cpp ~line 1030, read directly.
// This port has none of ARMDISARM_AIRMODE (quadplane-only), ARM_
// EMERGENCY_STOP (needs an AP_Notify-style emergency-stop concept this
// port lacks), or PARACHUTE_RELEASE (no parachute subsystem) - only
// AuxFunc::ArmDisarm is a real, in-scope member of this suppression set.
// The REASON this suppression exists at all, ported faithfully: a
// transmitter powered on with the arm switch already HIGH must not
// instantly arm the vehicle - the first-ever successful read of an
// ARM-type aux switch silently ADOPTS whatever position it finds as the
// new baseline (switch_state.current_position AND debounce_position both
// set to it immediately) rather than ever treating that starting position
// as an actionable change, even after the normal debounce window would
// otherwise have let it through. A non-ARM-type function gets NO such
// adoption - its starting position still requires the normal
// kSwitchDebounceTimeMs of stability before firing ONCE, same as any
// later change.
//
// NOT PORTED: the AUX_PWM_TRIGGER_LOW/_HIGH constants (1300/1700,
// RC_Channel.h) and read_aux()'s own AUX_FUNC::VTX_POWER special case
// (upstream's own `else if` branch reading read_6pos_switch() instead of
// read_3pos_switch() for that one function) - no AP_VideoTX subsystem,
// named exclusion. The `reversed`/ALLOW_SWITCH_REV per-channel-reversed-
// switch path inside read_3pos_switch() is also dropped - no RC_Channels::
// Option bitmask subsystem exists in this port, the SAME class of
// exclusion CPP-031 slice 11 already made for its own read_6pos_switch()
// (that method never reads `reversed` either).
//
// Dispatch (upstream's run_aux_function()/do_aux_function()) is
// DELIBERATELY NOT HERE: this port has no vehicle-specific RC_Channel
// subclass to hang a virtual do_aux_function() override on (ADR-0012) -
// read_aux() below returns the new debounced AuxSwitchPos (or nullopt)
// and leaves dispatch to the caller, exactly the same three-layer split
// (RcChannel raw primitive -> RcChannels resolution/orchestration ->
// Plane vehicle-specific dispatch) CPP-031 slice 11 already established
// for the flight-mode-switch channel. See rc_channels.hpp's read_aux_
// all() and plane.hpp's dispatch_aux_function() for the other two layers.

#include <cstdint>
#include <optional>

#include <fwcpp/math/scalar.hpp>

namespace fwcpp::rc {

enum class ControlType : std::uint8_t {
    kAngle = 0,
    kRange = 1,
};

// upstream: RC_Channel::AuxSwitchPos (RC_Channel.h ~line 430) - the
// decoded position of a 3-position aux switch. Ported field-for-field
// (as an ordinary enum rather than the "2-bit" packed storage upstream's
// own comment mentions - that packing is an upstream memory-layout
// micro-optimization with no behavioral effect, not reproduced here).
enum class AuxSwitchPos : std::uint8_t {
    kLow,    // pwm < AUX_SWITCH_PWM_TRIGGER_LOW (1200)
    kMiddle, // AUX_SWITCH_PWM_TRIGGER_LOW <= pwm <= AUX_SWITCH_PWM_TRIGGER_HIGH
    kHigh,   // pwm > AUX_SWITCH_PWM_TRIGGER_HIGH (1800)
};

// upstream: RC_Channel::AUX_FUNC (RC_Channel.h) - see this file's own
// "CPP-037 ADDENDUM" banner above for why this is a small, real SUBSET
// of upstream's 300+-value enum rather than an exhaustive port, and for
// the exact upstream numeric value of every member kept here.
enum class AuxFunc : std::uint16_t {
    DoNothing = 0,            // upstream: DO_NOTHING - aux switch disabled (the default)
    Rtl = 4,                  // upstream: RTL - change to RTL flight mode
    Auto = 16,                // upstream: AUTO - change to auto flight mode
    Manual = 51,              // upstream: MANUAL - manual mode
    Loiter = 56,              // upstream: LOITER - loiter mode
    Takeoff = 77,             // upstream: TAKEOFF - takeoff
    Fbwa = 92,                // upstream: FBWA - Fly-By-Wire-A

    // CPP-042: upstream: FBWA_TAILDRAGGER (RC_Channel.h:269), grepped
    // directly - 95, NOT renumbered. "enables FBWA taildragger takeoff
    // mode... elevator will be forced to TKOFF_TDRAG_ELEV" (upstream's own
    // doc comment). RC_Channel_Plane::do_aux_function()'s own dispatch
    // switch (RC_Channel_Plane.cpp ~line 305) treats this exactly like
    // FLAP above - "break; // input labels, nothing to do" - VERIFIED
    // DIRECTLY. The actual read is the RAW (non-debounced) switch position
    // via RcChannel::read_3pos_switch() below, resolved through
    // RcChannels::channel_for() - see plane.hpp's ModeFBWA::update() (the
    // real consumer, mode.hpp) and its own "CPP-042 ADDENDUM" file banner.
    FbwaTaildragger = 95,

    ModeSwitchReset = 96,     // upstream: MODE_SWITCH_RESET - trigger re-reading of mode switch
    Cruise = 150,             // upstream: CRUISE mode
    ArmDisarm = 153,          // upstream: ARMDISARM (4.2+ value - NOT the UNUSED(41) 4.1-and-lower one)
    EmergencyLandingEn = 157, // upstream: EMERGENCY_LANDING_EN - force long FS action to FBWA for landing out of range

    // CPP-038: upstream: FLAP (RC_Channel.h:381), grepped directly -
    // 208, NOT renumbered. The manual-flap-input channel Plane::
    // set_servos_flaps() resolves via RcChannels::channel_for() (added
    // this ticket, rc_channels.hpp, same module) - see plane.hpp's
    // set_servos_flaps() for the real consumer.
    Flap = 208,
};

class RcChannel {
public:
    std::int16_t radio_in = 0;
    std::int16_t radio_min = 1100;
    std::int16_t radio_max = 1900;
    std::int16_t radio_trim = 1500;
    std::uint16_t dead_zone = 0;
    bool reversed = false;
    std::int16_t high_in = 4500; // e.g. ANGLE_MAX for a control surface channel
    ControlType type_in = ControlType::kAngle;

    // Index into the RC input source (fwcpp::hal::RcInput::read(ch_in)),
    // matching RC_Channel.h:521's ch_in. RcChannels::RcChannels() below
    // defaults every channel's ch_in to its own array index (matching
    // RC_Channels::init()'s `channel(i)->ch_in = i` loop) - overridable
    // afterwards for a non-1:1 mapping, same as upstream allows.
    std::uint8_t ch_in = 0;

    // Cached scaled value from the last update(), matching RC_Channel.h:
    // 529's control_in. int16_t (not float) is upstream's own choice,
    // even though pwm_to_range()/pwm_to_angle() are float-returning -
    // reproduced here with an explicit narrowing cast rather than
    // upstream's implicit one (get_control_in_zero_dz() above stays
    // float-returning since it's a fresh per-call computation, not this
    // cache).
    std::int16_t control_in = 0;

    // Pulls a freshly-read PWM value in as radio_in and recomputes
    // control_in via pwm_to_range() or pwm_to_angle() depending on
    // type_in - matches RC_Channel::update()'s dispatch (RC_Channel.cpp:
    // ~303) minus the override/has_had_rc_receiver gating (see file
    // banner - that decision is the caller's, i.e. RcChannels::
    // read_input() in rc_channels.hpp). Always returns true: reaching
    // this call already means the caller decided a real update happens.
    bool update(std::uint16_t new_radio_in) {
        radio_in = static_cast<std::int16_t>(new_radio_in);
        control_in = static_cast<std::int16_t>(
            type_in == ControlType::kRange ? pwm_to_range() : pwm_to_angle());
        return true;
    }

    // upstream: RC_Channel.h's `switch_state` struct (private section,
    // "support for auxiliary switches"). -1 in either position field
    // means "no position established yet" - matches upstream's own
    // int8_t default-initialization-to-invalid convention (a real 6-
    // position switch only ever reports 0..5).
    struct SwitchState {
        std::int8_t current_position = -1;
        std::int8_t debounce_position = -1;
        std::uint32_t last_edge_time_ms = 0;
        // upstream: switch_state.initialised (RC_Channel.h) - CPP-037.
        // Only read_aux() below consults this (read_6pos_switch() above
        // never did upstream either) - see this file's own "CPP-037
        // ADDENDUM" banner for the first-radio-read suppression this
        // gates.
        bool initialised = false;
    };
    SwitchState switch_state;

    // upstream: RC_Channel::option (RC_Channel.h's AP_Int8, RC_Channel.cpp
    // var_info's "OPTION" param) - CPP-037. Not AP_Param-backed (same
    // established precedent as radio_min/max/trim above) - a plain
    // settable field, default DoNothing, matching upstream's real
    // default (0).
    AuxFunc option = AuxFunc::DoNothing;

    // upstream: RC_Channel::RC_MIN_LIMIT_PWM / RC_MAX_LIMIT_PWM
    // (RC_Channel.h:465,467) - a pulsewidth at or outside these bounds is
    // treated as a receiver/wiring error, not a real switch position.
    static constexpr std::uint16_t kRcMinLimitPwm = 800;
    static constexpr std::uint16_t kRcMaxLimitPwm = 2200;

    // upstream: SWITCH_DEBOUNCE_TIME_MS (RC_Channel.cpp:64).
    static constexpr std::uint32_t kSwitchDebounceTimeMs = 200;

    // upstream: RC_Channel::read_6pos_switch(int8_t& position)
    // (RC_Channel.cpp ~line 596, read in full). Discretizes the current
    // radio_in into one of 6 fixed PWM-breakpoint positions, then runs it
    // through debounce_completed() below - returns false for either an
    // out-of-range (error) pulsewidth OR a position that hasn't been
    // stable for kSwitchDebounceTimeMs yet, exactly like upstream (both
    // cases mean "no real, actionable position change to report").
    bool read_6pos_switch(std::int8_t& position, std::uint32_t now_ms) {
        const std::uint16_t pulsewidth = static_cast<std::uint16_t>(radio_in);
        if (pulsewidth <= kRcMinLimitPwm || pulsewidth >= kRcMaxLimitPwm) {
            return false; // this is an error condition
        }

        if (pulsewidth < 1231) {
            position = 0;
        } else if (pulsewidth < 1361) {
            position = 1;
        } else if (pulsewidth < 1491) {
            position = 2;
        } else if (pulsewidth < 1621) {
            position = 3;
        } else if (pulsewidth < 1750) {
            position = 4;
        } else {
            position = 5;
        }

        return debounce_completed(position, now_ms);
    }

    // upstream: RC_Channel::debounce_completed(int8_t position)
    // (RC_Channel.cpp ~line 636, read in full) - a REAL debounce state
    // machine, not a simple "N ticks past a threshold" counter: a newly-
    // observed position resets a separate edge timer (debounce_position/
    // last_edge_time_ms) every time it CHANGES, so a position that
    // wobbles back and forth within the debounce window never latches -
    // only a position that has held steady for the FULL
    // kSwitchDebounceTimeMs since its own last change is promoted to
    // current_position and reported as a real, actionable change (return
    // true). Ported field-for-field, condition-for-condition.
    bool debounce_completed(std::int8_t position, std::uint32_t now_ms) {
        // switch change not detected
        if (switch_state.current_position == position) {
            // reset debouncing
            switch_state.debounce_position = position;
        } else {
            // switch change detected
            // position not established yet
            if (switch_state.debounce_position != position) {
                switch_state.debounce_position = position;
                switch_state.last_edge_time_ms = now_ms;
            } else if (now_ms - switch_state.last_edge_time_ms >= kSwitchDebounceTimeMs) {
                // position established; debounce completed
                switch_state.current_position = position;
                return true;
            }
        }

        return false;
    }

    // upstream: RC_Channel::AUX_SWITCH_PWM_TRIGGER_LOW/_HIGH (RC_Channel.h
    // :475,477) - CPP-037.
    static constexpr std::uint16_t kAuxSwitchPwmTriggerLow = 1200;
    static constexpr std::uint16_t kAuxSwitchPwmTriggerHigh = 1800;

    // upstream: RC_Channel::read_3pos_switch(AuxSwitchPos&) (RC_Channel.cpp
    // ~line 2031, read in full) - CPP-037. The `reversed`/ALLOW_SWITCH_REV
    // branch is dropped - see this file's own "CPP-037 ADDENDUM" banner.
    bool read_3pos_switch(AuxSwitchPos& ret) const {
        const std::uint16_t in = static_cast<std::uint16_t>(radio_in);
        if (in <= kRcMinLimitPwm || in >= kRcMaxLimitPwm) {
            return false;
        }
        if (in < kAuxSwitchPwmTriggerLow) {
            ret = AuxSwitchPos::kLow;
        } else if (in > kAuxSwitchPwmTriggerHigh) {
            ret = AuxSwitchPos::kHigh;
        } else {
            ret = AuxSwitchPos::kMiddle;
        }
        return true;
    }

    // upstream: RC_Channel::init_position_on_first_radio_read(AUX_FUNC)
    // (RC_Channel.cpp ~line 1030, read in full) - CPP-037. See this file's
    // own "CPP-037 ADDENDUM" banner for the real, narrow suppression set
    // this port has (ArmDisarm only) versus upstream's real four-value one.
    static bool init_position_on_first_radio_read(AuxFunc func) {
        return func == AuxFunc::ArmDisarm;
    }

    // upstream: RC_Channel::read_aux() (RC_Channel.cpp ~line 976, read in
    // full) - CPP-037. Returns the new debounced AuxSwitchPos exactly once
    // per real, actionable switch change; nullopt otherwise (DoNothing
    // option, invalid/out-of-range PWM, or debounce not yet settled - all
    // of upstream's own "don't call run_aux_function()" cases collapse to
    // nullopt here, same shape as RcChannels::read_mode_switch() in
    // rc_channels.hpp). Dispatch itself (upstream's run_aux_function()/
    // do_aux_function()) is the CALLER's job - see this file's own
    // "CPP-037 ADDENDUM" banner for why.
    std::optional<AuxSwitchPos> read_aux(std::uint32_t now_ms) {
        if (option == AuxFunc::DoNothing) {
            // upstream: "may wish to add special cases for other 'AUXSW'
            // things here e.g. RCMAP_ROLL etc once they become options" -
            // no such cases in this port.
            return std::nullopt;
        }

        AuxSwitchPos new_position;
        if (!read_3pos_switch(new_position)) {
            return std::nullopt;
        }

        if (!switch_state.initialised) {
            switch_state.initialised = true;
            if (init_position_on_first_radio_read(option)) {
                switch_state.current_position = static_cast<std::int8_t>(new_position);
                switch_state.debounce_position = static_cast<std::int8_t>(new_position);
            }
        }

        if (!debounce_completed(static_cast<std::int8_t>(new_position), now_ms)) {
            return std::nullopt;
        }

        return new_position;
    }

    // Angle (centidegrees) from radio_in, using an explicit dead_zone and
    // trim rather than this channel's own configured ones - the shared
    // core pwm_to_angle_dz/pwm_to_angle build on.
    [[nodiscard]] float pwm_to_angle_dz_trim(std::uint16_t dz, std::int16_t trim) const {
        const std::int16_t trim_high = static_cast<std::int16_t>(trim + dz);
        const std::int16_t trim_low = static_cast<std::int16_t>(trim - dz);
        const float reverse_mul = reversed ? -1.0f : 1.0f;

        const std::int16_t r_in = math::constrain_value(radio_in, radio_min, radio_max);

        if (r_in > trim_high && radio_max != trim_high) {
            return reverse_mul * (static_cast<float>(high_in) * static_cast<float>(r_in - trim_high)) / static_cast<float>(radio_max - trim_high);
        }
        if (r_in < trim_low && trim_low != radio_min) {
            return reverse_mul * (static_cast<float>(high_in) * static_cast<float>(r_in - trim_low)) / static_cast<float>(trim_low - radio_min);
        }
        return 0.0f;
    }

    [[nodiscard]] float pwm_to_angle_dz(std::uint16_t dz) const { return pwm_to_angle_dz_trim(dz, radio_trim); }
    [[nodiscard]] float pwm_to_angle() const { return pwm_to_angle_dz(dead_zone); }

    // Range value (0..high_in) from radio_in, using an explicit dead_zone.
    [[nodiscard]] float pwm_to_range_dz(std::uint16_t dz) const {
        std::int16_t r_in = math::constrain_value(radio_in, radio_min, radio_max);
        if (reversed) {
            r_in = static_cast<std::int16_t>(radio_max - (r_in - radio_min));
        }
        const std::int16_t trim_low = static_cast<std::int16_t>(radio_min + dz);
        if (r_in > trim_low) {
            return (static_cast<float>(high_in) * static_cast<float>(r_in - trim_low)) / static_cast<float>(radio_max - trim_low);
        }
        return 0.0f;
    }

    [[nodiscard]] float pwm_to_range() const { return pwm_to_range_dz(dead_zone); }

    // Dispatches to pwm_to_range_dz(0) or pwm_to_angle_dz(0) depending on
    // this channel's configured ControlType.
    [[nodiscard]] float get_control_in_zero_dz() const {
        return type_in == ControlType::kRange ? pwm_to_range_dz(0) : pwm_to_angle_dz(0);
    }

    // Normalized (-1..1) input using this channel's own trim as center
    // and dead_zone.
    [[nodiscard]] float norm_input() const {
        const float reverse_mul = reversed ? -1.0f : 1.0f;
        float ret;
        if (radio_in < radio_trim) {
            if (radio_min >= radio_trim) {
                return 0.0f;
            }
            ret = reverse_mul * static_cast<float>(radio_in - radio_trim) / static_cast<float>(radio_trim - radio_min);
        } else {
            if (radio_max <= radio_trim) {
                return 0.0f;
            }
            ret = reverse_mul * static_cast<float>(radio_in - radio_trim) / static_cast<float>(radio_max - radio_trim);
        }
        return math::constrain_value(ret, -1.0f, 1.0f);
    }

    [[nodiscard]] float norm_input_dz() const {
        const std::int16_t dz_min = static_cast<std::int16_t>(radio_trim - dead_zone);
        const std::int16_t dz_max = static_cast<std::int16_t>(radio_trim + dead_zone);
        const float reverse_mul = reversed ? -1.0f : 1.0f;
        float ret;
        if (radio_in < dz_min && dz_min > radio_min) {
            ret = reverse_mul * static_cast<float>(radio_in - dz_min) / static_cast<float>(dz_min - radio_min);
        } else if (radio_in > dz_max && radio_max > dz_max) {
            ret = reverse_mul * static_cast<float>(radio_in - dz_max) / static_cast<float>(radio_max - dz_max);
        } else {
            ret = 0.0f;
        }
        return math::constrain_value(ret, -1.0f, 1.0f);
    }

    // Normalized (-1..1) input using radio_min/radio_max only - trim and
    // dead_zone are not involved at all (matches upstream's own name).
    [[nodiscard]] float norm_input_ignore_trim() const {
        if (radio_max <= radio_min) {
            return 0.0f;
        }
        const float ret = (reversed ? -2.0f : 2.0f)
            * ((static_cast<float>(radio_in - radio_min) / static_cast<float>(radio_max - radio_min)) - 0.5f);
        return math::constrain_value(ret, -1.0f, 1.0f);
    }

    // upstream: RC_Channel::percent_input() (RC_Channel.cpp:488-501, read
    // in full) - CPP-038. VERIFIED DIRECTLY (the ticket's own instruction
    // to double check this, since it is easy to mis-scope): this returns
    // an UNSIGNED 0..100 value - trim and reversed's usual "centered at
    // trim" treatment do NOT apply here at all; only radio_min/radio_max
    // matter, and `reversed` flips which END maps to 0 vs 100, never
    // producing a negative result. Upstream's own real caller
    // (Plane::set_servos_flaps(), servos.cpp:685) assigns this into a
    // SIGNED `int8_t manual_flap_percent` - safe precisely because the
    // returned range (0..100) is always representable as a non-negative
    // int8_t, never because the value itself can go negative.
    [[nodiscard]] std::uint8_t percent_input() const {
        if (radio_in <= radio_min) {
            return reversed ? 100 : 0;
        }
        if (radio_in >= radio_max) {
            return reversed ? 0 : 100;
        }
        std::uint8_t ret = static_cast<std::uint8_t>(
            100.0f * static_cast<float>(radio_in - radio_min) / static_cast<float>(radio_max - radio_min));
        if (reversed) {
            ret = static_cast<std::uint8_t>(100 - ret);
        }
        return ret;
    }
};

} // namespace fwcpp::rc
