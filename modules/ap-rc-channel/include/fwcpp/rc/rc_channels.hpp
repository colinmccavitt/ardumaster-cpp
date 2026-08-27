#pragma once

// Port of RC_Channel/RC_Channel.h's RC_Channels class declaration
// (libraries/RC_Channel/RC_Channel.h:588) and the base-class logic in
// RC_Channel/RC_Channels.cpp: has_valid_input(), get_radio_in(),
// read_input(), get_valid_channel_count(). CPP-027 slice.
//
// (The header banners of this port's earlier RcChannel/SrvChannel core
// math, rc_channel.hpp and srv_channel.hpp, mislabel themselves
// "CPP-030 slice 1/2" - CPP-030 is actually a distinct, unrelated
// tracker ticket (ap-sim flight dynamics simulator). That work was
// really CPP-027 all along, per the tracker's own ticket file and this
// slice's task. Flagging here rather than silently "fixing" someone
// else's already-landed commit banners; a tracker-hygiene fix is a
// separate, out-of-scope change.)
//
// NO SINGLETON (ADR-0012): upstream's RC_Channels is a singleton
// (_singleton, AP_HAL::panic() if constructed twice) with a pure-virtual
// `RC_Channel *channel(uint8_t chan) = 0` that each VEHICLE subclass
// implements to index into its own array (Plane/Copter/Rover each embed
// their own array of RC_Channel subclass and hand back a pointer into
// it). This port has no vehicle subclass hierarchy yet and no singleton
// registry anywhere else (HalContext, FilterRegistry, AltitudeContext,
// L1Inputs all follow the same house rule) - so RcChannels here OWNS a
// fixed std::array<RcChannel, kNumRcChannels> directly, with a plain
// non-virtual bounds-checked channel() accessor. No virtual dispatch, no
// singleton, no rc() free function - a caller constructs one and passes
// it (and an fwcpp::hal::RcInput&) around explicitly.
//
// kNumRcChannels = 16 pins NUM_RC_CHANNELS from RC_Channel.h:30.
//
// read_input()'s "has anything ever come in" latch (has_valid_input()'s
// dependency) is upstream's _has_ever_seen_rc_input (RC_Channel.h, RC_Channels
// private section) - tracked here as has_ever_seen_rc_input_, set true
// exactly once RcInput::new_input() has ever been observed true during a
// read_input() call, and never cleared again (matches upstream: nothing
// ever resets _has_ever_seen_rc_input either).
//
// Deliberately OUT OF SCOPE for this slice (left unimplemented, not
// stubbed - no dead placeholder methods for any of these):
//   - MAVLink/GCS RC overrides (set_override/clear_overrides/
//     has_active_overrides/override_value/has_new_overrides/
//     get_override_mask/gcs_overrides_enabled). This port has no
//     MAVLink/GCS subsystem yet - nothing exists to call these or to
//     receive an override from. RC_Channel::update()'s override branch
//     is correspondingly absent from RcChannel::update() (see that
//     file's banner) - only the receiver-read branch is reproduced.
//   - Aux function dispatch (RC_Channel::AUX_FUNC, find_channel_for_option,
//     init_aux_all, duplicate_options_exist, convert_options). A large
//     separate subsystem (RC-channel-to-auxiliary-switch-function mapping)
//     with its own scope. CPP-037 below ports read_aux_all() (and
//     reset_mode_switch()) - the REST of this list (find_channel_for_
//     option/init_aux_all/duplicate_options_exist/convert_options) is
//     STILL out of scope: find_channel_for_option() has no caller in this
//     port's own scope (nothing here needs to look UP from a function to
//     its channel - dispatch is always driven the other direction, by
//     scanning channels, exactly like read_aux_all() itself does);
//     init_aux_all() exists purely to run each configured aux function
//     ONCE at boot with its resting position (upstream: RC_Channel::
//     init_aux(), which calls do_aux_function_armdisarm()/do_aux_function_
//     change_mode() etc. immediately rather than waiting for read_aux()'s
//     own debounce/first-read-suppression) - this port's own Plane starts
//     from a fixed, known-good default state (control_mode = &mode_manual,
//     armed = false) that a boot-time aux re-application would only ever
//     reproduce or leave unchanged for every real function this ticket
//     wires (ArmDisarm's own init_position_on_first_radio_read()
//     suppression makes init_aux()'s boot-time call a no-op for it
//     anyway), so there is no OBSERVABLE difference for this port's scope
//     to actually test; duplicate_options_exist()/convert_options() are
//     pure AP_Param/GCS-parameter-migration housekeeping with nothing to
//     migrate (no AP_Param backing for `option` at all, matching radio_
//     min/max/trim's own established precedent, rc_channel.hpp).
//   - RSSI / link quality (get_receiver_rssi/get_receiver_link_quality).
//     Hardware-telemetry-adjacent; no receiver-link modeling exists here.
//   - The Option enum / option_is_enabled bitmask (CRSF/FPORT/arming-check
//     toggles). Every one of its bits gates a feature from the
//     subsystems above; nothing to gate yet.
//   - in_rc_failsafe(): upstream's own RC_Channels base-class default is
//     a trivial `return true` (a vehicle subclass overrides it with real
//     failsafe logic derived from RC link state this port doesn't model
//     yet) - genuinely nothing to port beyond a constant, so left out
//     rather than adding a one-line method with no real behavior behind it.
//   - Rudder arm/disarm (rudder_arm_disarm_check, seen_neutral_rudder).
//     A vehicle-arming-logic concern (calls into AP_Arming, which this
//     port hasn't built) - out of scope here.
//   - get_pwm()/lua_rc_channel() (scripting-facing 1-indexed wrappers),
//     get_roll/pitch/yaw/throttle/forward/lateral_channel() (AP_RCMapper-
//     backed axis lookups), get_fs_timeout_ms(), calibrating(),
//     enabled_protocols(), get_aux_cached() (AP_SCRIPTING_ENABLED-gated).
//     All thin wrappers around subsystems (AP_RCMapper, AP_Scripting,
//     AP_Param-backed options) this port hasn't ported; trivial to add
//     once a real caller needs one, not designed in speculatively now.
//
// CPP-031 SLICE 11: added flight_mode_channel_number/flight_mode_channel()/
// read_mode_switch() - upstream: RC_Channels::flight_mode_channel()
// (RC_Channel.cpp ~line 211, read in full) and RC_Channels::
// read_mode_switch() (~line 232, read in full). flight_mode_channel_number
// is upstream's FLTMODE_CH (ArduPlane/Parameters.cpp), an AP_Param-backed
// int - not wired to AP_Param yet (see rc_channel.hpp's own precedent for
// radio_min/max/trim), so it's a plain field here, 1-indexed exactly like
// upstream's own convention (channel 1 is index 0 - flight_mode_channel()
// below subtracts 1, matching upstream's `rc_channel(num-1)`). Default 8:
// ArduPlane's real FLIGHT_MODE_CHANNEL stock default (ArduPlane/config.h,
// grepped directly, NOT assumed to be channel 5 - RC_Channel's own
// standalone example (examples/RC_Channel/RC_Channel.cpp:40) hardcodes 5,
// but that is a bare-library example, not ArduPlane's real vehicle
// default).
//
// READ_MODE_SWITCH() COLLAPSES THREE UPSTREAM METHODS INTO ONE, PER THE
// TICKET'S OWN INSTRUCTION TO DESIGN THE CLEANEST NO-EXCEPTIONS SHAPE:
// upstream splits this across RC_Channels::read_mode_switch() (has_valid_
// input() guard + flight_mode_channel() lookup) calling RC_Channel::
// read_mode_switch() (calls read_6pos_switch(), and on success alone,
// dispatches to a virtual mode_switch_changed(modeswitch_pos_t) - a
// per-vehicle override, ArduPlane's RC_Channel_Plane::mode_switch_changed(),
// control_modes.cpp). This port has no RC_Channel subclass hierarchy (no
// singleton/virtual-dispatch machinery at all - ADR-0012, this file's own
// "NO SINGLETON" note above) for a vehicle to hook a virtual method into,
// so instead of inventing one just to carry a single callback, this method
// returns `std::optional<std::int8_t>` - nullopt for "no actionable change
// this call" (no valid input yet, no channel configured, invalid PWM, or
// debounce not yet complete - all four of upstream's own "don't call
// mode_switch_changed()" cases collapse to the same nullopt here), or the
// new debounced position (0..5) when a real, actionable change occurred.
// The caller (fwcpp::vehicle::Plane::mode_switch_changed(), plane.hpp) is
// the one that knows how to turn a position into an actual set_mode() call
// - exactly upstream's own separation of concerns (RC_Channels doesn't
// know about modes either), just expressed as a return value instead of a
// virtual callback.
//
// flight_mode_channel_conflicts_with_rc_option() is STILL NOT ported, even
// after CPP-037 added a real aux-function subsystem below - it exists
// purely to WARN (a GCS-facing diagnostic, no GCS subsystem here) about a
// channel double-booked between the mode switch and an aux function, and
// checking that requires find_channel_for_option() (a function -> channel
// reverse lookup), itself still out of scope (this file's own "Aux
// function dispatch" exclusion note above) - nothing for this method to
// meaningfully check without it.
//
// =====================================================================
// CPP-037 ADDENDUM: the aux-function dispatch mechanism's RcChannels-level
// half - read_aux_all() (scans every channel with a configured option,
// invoking a caller-supplied handler once per real debounced change) and
// reset_mode_switch() (forces the flight-mode channel's own debounce
// state to restart, the real mechanism behind MODE_SWITCH_RESET and
// do_aux_function_change_mode()'s "give control back to the flight-mode
// switch" behavior). Upstream: RC_Channels::read_aux_all() (RC_Channels.
// cpp ~line 173) and RC_Channels::reset_mode_switch()/RC_Channel::
// reset_mode_switch() (RC_Channels.cpp ~line 223 / RC_Channel.cpp ~line
// 588) - all read in full. The per-channel primitives (read_3pos_switch/
// init_position_on_first_radio_read/read_aux, AuxFunc/AuxSwitchPos) live
// in rc_channel.hpp (same module) - see that file's own "CPP-037
// ADDENDUM" banner for the full design and every named exclusion in the
// AuxFunc enum itself. Vehicle-specific dispatch (which AuxFunc values do
// what) lives in fwcpp::vehicle::Plane::dispatch_aux_function() (plane.
// hpp) - see ITS OWN file banner for the complete, ticket-required list
// of every real upstream AUX_FUNC case this port disclaims rather than
// stubs (camera/gripper/sprayer/generator/RunCam/quadplane/ADSB-avoidance/
// soaring/terrain/relays/fence/mission-reset/RC-override-enable/FFT-tune/
// mount/VTX/inverted/reverse-throttle/airbrake/flap/... and the excluded
// mode-select values ACRO/GUIDED/CIRCLE/TRAINING).
//
// READ_AUX_ALL() USES A TEMPLATED CALLBACK, NOT A VIRTUAL do_aux_
// function() OVERRIDE: same ADR-0012 rationale as read_mode_switch()'s
// own std::optional return above (no vehicle-specific RC_Channel subclass
// to hang a virtual method on), generalized here because MULTIPLE
// channels can each have their own configured option and each
// independently produce a real change in a single call - a single
// std::optional return (read_mode_switch()'s own shape, correct for
// exactly one flight-mode channel) cannot represent that. A template
// avoids both a std::function's heap allocation/vtable indirection and a
// fixed-capacity output buffer nothing in this port's scope needs.
// AP::logger().Write_RCIN()'s need_log bookkeeping is dropped - no
// logging subsystem (long-standing exclusion, this file's banner above).
//
// RESET_MODE_SWITCH() COLLAPSES TWO UPSTREAM METHODS INTO ONE, same
// reasoning as read_mode_switch() above: upstream's RC_Channels::reset_
// mode_switch() (channel resolution) calls RC_Channel::reset_mode_switch()
// (the actual two-field reset + immediate re-read) - collapsed here since
// this port's RcChannel has no reset_mode_switch() of its own (nothing
// else needs one - see rc_channel.hpp's OLD note this replaces). The
// immediate re-read's result is always discarded (see reset_mode_switch()
// below for why it can never itself report an actionable change) -
// matches upstream, whose own call is to a void-returning method for the
// same "just seed last_edge_time_ms now" effect.
//
// TICK() WIRING: mode.hpp's tick() calls read_aux_all() as its own new
// step (immediately after step 1c's mode-switch dispatch) - see that
// file's own "CPP-037 NOTE" for why that relative order matters (a
// pilot's TAKEOFF/etc. aux-engage or MODE_SWITCH_RESET can call THIS
// tick's reset_mode_switch(), and the debounce timer it restarts must
// not be read again until the FOLLOWING tick, matching upstream's own
// real scheduler ordering: read_mode_switch() at priority 7 always runs
// strictly before read_aux_all() at priority 10, both same-rate, Plane.
// cpp's scheduler_tasks[]).

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

#include <fwcpp/hal/rc_input.hpp>
#include <fwcpp/rc/rc_channel.hpp>

namespace fwcpp::rc {

// NUM_RC_CHANNELS, RC_Channel.h:30.
inline constexpr std::uint8_t kNumRcChannels = 16;

class RcChannels {
public:
    // Defaults every channel's ch_in to its own array index, matching
    // RC_Channels::init()'s `channel(i)->ch_in = i` loop (RC_Channels.cpp).
    // Upstream splits construction (RC_Channels::RC_Channels(), which only
    // sets up AP_Param defaults and the singleton) from init() (which sets
    // ch_in and calls init_aux_all()) - collapsed into one constructor
    // here since there is no AP_Param registration step in this port to
    // separate from, and init_aux_all() is out of scope (see file banner).
    RcChannels() {
        for (std::uint8_t i = 0; i < kNumRcChannels; ++i) {
            channels_[i].ch_in = i;
        }
    }

    // Bounds-checked accessor mirroring how RcOutput/AnalogIn handle an
    // out-of-range index (a sentinel/default value there; nullptr here,
    // since a channel is an object rather than a scalar) rather than
    // upstream's own channel(), whose bounds-checking is entirely the
    // vehicle subclass's responsibility and thus varies per vehicle.
    [[nodiscard]] RcChannel* channel(std::uint8_t chan) {
        return chan < kNumRcChannels ? &channels_[chan] : nullptr;
    }
    [[nodiscard]] const RcChannel* channel(std::uint8_t chan) const {
        return chan < kNumRcChannels ? &channels_[chan] : nullptr;
    }

    // Port of RC_Channels::read_input() (RC_Channels.cpp) minus the
    // override/IGNORE_RECEIVER branches (see file banner): pulls each
    // channel's PWM from rc_input at that channel's own ch_in index and
    // recomputes its cached control_in, but only when rc_input reports
    // new_input() - matching upstream's own "nothing to do" early return
    // when no new frame has arrived. Sets the has-ever-seen-input latch
    // (has_valid_input()'s dependency) and returns whether any channel
    // was actually updated, matching upstream's bool return + `success |=`
    // accumulation.
    bool read_input(hal::RcInput& rc_input) {
        if (!rc_input.new_input()) {
            return false;
        }

        has_ever_seen_rc_input_ = true;
        ++input_update_count_; // CPP-031 slice 8 (ap-vehicle RC short failsafe) - see input_update_count() below

        bool success = false;
        for (std::uint8_t i = 0; i < kNumRcChannels; ++i) {
            success |= channels_[i].update(rc_input.read(channels_[i].ch_in));
        }
        return success;
    }

    // CPP-031 slice 8 (ap-vehicle RC short failsafe) addition: a monotonic
    // count of how many times read_input() above has actually processed a
    // genuinely new frame (i.e. how many times it has returned true) -
    // NOT reset or consumed by being read, unlike RcInput::new_input()'s
    // own single-shot flag. Exists so a caller checking "did a new frame
    // arrive since I last looked" (fwcpp::vehicle::Plane::
    // update_throttle_failsafe()) gets a correct answer even when SOME
    // OTHER caller already consumed RcInput::new_input() earlier in the
    // same logical tick - this port's own vehicle_test.cpp set_sticks()
    // helper does exactly that, calling read_input() directly before
    // tick() calls it again, which would otherwise make a second,
    // flag-based check always see "no new input" whether or not one
    // genuinely arrived. Counting updates rather than latching a bool
    // also means two real frames arriving between two checks are not
    // conflated with one - not needed by today's one caller, but the
    // honest, no-information-lost version of "did anything change".
    [[nodiscard]] std::uint32_t input_update_count() const { return input_update_count_; }

    // Port of RC_Channels::has_valid_input() (RC_Channels.cpp) exactly:
    // the base-class implementation is *only* the has-ever-seen-input
    // check (a vehicle subclass upstream layers many more checks -
    // failsafe state, receiver health - on top, but those all live in
    // subsystems out of scope here per the file banner).
    [[nodiscard]] bool has_valid_input() const { return has_ever_seen_rc_input_; }

    // Port of RC_Channels::get_radio_in(uint16_t*, uint8_t) (RC_Channels.cpp):
    // zero-fills the caller's buffer first (matching upstream's memset),
    // then fills up to min(num_channels, kNumRcChannels) entries from
    // each channel's current radio_in, returning the count actually
    // filled. Mirrors rc_output.hpp's own bulk read(period_us*, len)
    // pattern (raw pointer + length, not std::span) for the same
    // contract shape as an existing bulk API in this port.
    std::uint8_t get_radio_in(std::uint16_t* chans, std::uint8_t num_channels) const {
        std::fill_n(chans, num_channels, std::uint16_t{0});

        const std::uint8_t read_channels = std::min<std::uint8_t>(num_channels, kNumRcChannels);
        for (std::uint8_t i = 0; i < read_channels; ++i) {
            chans[i] = static_cast<std::uint16_t>(channels_[i].radio_in);
        }
        return read_channels;
    }

    // Port of RC_Channels::get_valid_channel_count() (RC_Channels.cpp):
    // MIN(NUM_RC_CHANNELS, hal.rcin->num_channels()). Upstream reaches
    // hal.rcin through the singleton; here the caller passes the same
    // RcInput it uses for read_input() explicitly (no singleton - see
    // file banner).
    [[nodiscard]] std::uint8_t get_valid_channel_count(const hal::RcInput& rc_input) const {
        return std::min<std::uint8_t>(kNumRcChannels, rc_input.num_channels());
    }

    // upstream: FLTMODE_CH (ArduPlane/Parameters.cpp) via RC_Channels_Plane::
    // flight_mode_channel_number() (RC_Channel_Plane.cpp:15) - see file
    // banner's "CPP-031 SLICE 11" note for the default-8-not-5 finding.
    // 1-indexed; 0 (or any value outside 1..kNumRcChannels) means "no mode
    // switch channel configured", matching upstream's own `num <= 0`/
    // `num >= NUM_RC_CHANNELS` guards in flight_mode_channel() below.
    std::int8_t flight_mode_channel_number = 8;

    // upstream: RC_Channels::flight_mode_channel() (RC_Channel.cpp ~line
    // 211, read in full).
    [[nodiscard]] RcChannel* flight_mode_channel() {
        if (flight_mode_channel_number <= 0 || flight_mode_channel_number > static_cast<std::int8_t>(kNumRcChannels)) {
            return nullptr;
        }
        return channel(static_cast<std::uint8_t>(flight_mode_channel_number - 1));
    }

    // upstream: RC_Channels::read_mode_switch() (RC_Channel.cpp ~line 232)
    // + RC_Channel::read_mode_switch()/read_6pos_switch() (RC_Channel.cpp),
    // collapsed - see file banner's "CPP-031 SLICE 11" note for why this
    // returns std::optional<std::int8_t> instead of dispatching through a
    // virtual mode_switch_changed(). now_ms is an explicit parameter
    // (ADR-0012) forwarded straight to RcChannel::read_6pos_switch().
    [[nodiscard]] std::optional<std::int8_t> read_mode_switch(std::uint32_t now_ms) {
        if (!has_valid_input()) {
            // exit immediately when no RC input - upstream's own guard.
            return std::nullopt;
        }
        RcChannel* c = flight_mode_channel();
        if (c == nullptr) {
            return std::nullopt;
        }
        std::int8_t position = 0;
        if (!c->read_6pos_switch(position, now_ms)) {
            return std::nullopt;
        }
        return position;
    }

    // upstream: RC_Channels::read_aux_all() (RC_Channels.cpp ~line 173,
    // read in full) - CPP-037, see this file's own "CPP-037 ADDENDUM"
    // banner for the full design. `handler` is invoked as
    // `handler(AuxFunc, AuxSwitchPos)` once per channel that reports a
    // real, debounced change this call - never for a DoNothing channel
    // (skipped before ever calling read_aux(), same short-circuit
    // upstream's own read_aux() does internally) and never more than once
    // per channel per call.
    template <typename Handler>
    void read_aux_all(std::uint32_t now_ms, Handler&& handler) {
        if (!has_valid_input()) {
            // exit immediately when no RC input - upstream's own guard.
            return;
        }
        for (std::uint8_t i = 0; i < kNumRcChannels; ++i) {
            RcChannel& c = channels_[i];
            if (c.option == AuxFunc::DoNothing) {
                continue;
            }
            if (const std::optional<AuxSwitchPos> pos = c.read_aux(now_ms); pos.has_value()) {
                handler(c.option, *pos);
            }
        }
    }

    // upstream: RC_Channels::reset_mode_switch() (RC_Channels.cpp ~line
    // 223) + RC_Channel::reset_mode_switch() (RC_Channel.cpp ~line 588,
    // read in full) - CPP-037, collapsed for the same reason read_mode_
    // switch() above collapses two upstream methods (this file's own
    // "CPP-037 ADDENDUM" banner). Resets the flight-mode channel's
    // debounce state to "no position established" and immediately
    // re-invokes read_mode_switch() purely to seed last_edge_time_ms at
    // THIS instant (matching upstream's own trailing call) - that
    // immediate re-read can never itself report an actionable change (a
    // position cannot already have been stable for kSwitchDebounceTimeMs
    // at the exact instant its own debounce state was just cleared), so
    // its result is discarded, matching upstream discarding its own
    // void-returning call.
    void reset_mode_switch(std::uint32_t now_ms) {
        RcChannel* c = flight_mode_channel();
        if (c == nullptr) {
            return;
        }
        c->switch_state.current_position = -1;
        c->switch_state.debounce_position = -1;
        (void)read_mode_switch(now_ms);
    }

private:
    std::array<RcChannel, kNumRcChannels> channels_{};

    // _has_ever_seen_rc_input, RC_Channel.h (RC_Channels private section).
    bool has_ever_seen_rc_input_ = false;

    // CPP-031 slice 8 addition - see input_update_count() above.
    std::uint32_t input_update_count_ = 0;
};

} // namespace fwcpp::rc
