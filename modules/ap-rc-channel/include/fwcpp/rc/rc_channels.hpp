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
//     init_aux_all/read_aux_all, duplicate_options_exist, convert_options).
//     A large separate subsystem (RC-channel-to-auxiliary-switch-function
//     mapping) with its own scope, not part of this slice.
//   - Mode switch handling (reset_mode_switch/read_mode_switch/
//     flight_mode_channel/flight_mode_channel_number). Needs the
//     vehicle's flight-mode subsystem, which doesn't exist in this port.
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

#include <algorithm>
#include <array>
#include <cstdint>

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

        bool success = false;
        for (std::uint8_t i = 0; i < kNumRcChannels; ++i) {
            success |= channels_[i].update(rc_input.read(channels_[i].ch_in));
        }
        return success;
    }

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

private:
    std::array<RcChannel, kNumRcChannels> channels_{};

    // _has_ever_seen_rc_input, RC_Channel.h (RC_Channels private section).
    bool has_ever_seen_rc_input_ = false;
};

} // namespace fwcpp::rc
