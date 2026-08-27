#pragma once

// Port of SRV_Channel/SRV_Channel.h's `SRV_Channels` class (declared from
// around line 383 through EOF), matched against the real implementations in
// SRV_Channels.cpp and SRV_Channel_aux.cpp (output_ch()/output_ch_all() -
// the code that actually writes computed PWM to RC output hardware).
// CPP-027 ("RC_Channel / SRV_Channel: RC input decoding and servo output
// mapping"), "slice 1" of the registry itself - the vehicle-wide array of
// channels, function-based lookup/dispatch, and hardware output, built on
// top of the already-ported per-channel core in srv_channel.hpp.
//
// NO SINGLETON, EXPLICIT CONTEXT INSTEAD (ADR-0012, this port's standing
// convention - see AltitudeContext, L1Inputs, FilterRegistry, HalContext,
// and this port's own RcOutput/AnalogIn). Upstream's SRV_Channels is
// entirely `static` methods operating on a hidden singleton
// (`obj_channels[NUM_SERVO_CHANNELS]`, `_singleton`, `AP::srv_channels()`).
// This port's SrvChannels instead OWNS a std::array<SrvChannel,
// kNumServoChannels> as a plain public member (`channels` below), and every
// upstream `static` method becomes an ordinary member function taking
// whatever upstream's static function took as its real logical arguments -
// no hidden lookup, no `AP::srv()` accessor.
//
// kNumServoChannels = 32, matching fwcpp::hal::kNumRcOutputChannels
// (rc_output.hpp) - verified against upstream's actual
// NUM_SERVO_CHANNELS resolution in SRV_Channel_config.h:
//   #if HAL_PROGRAM_SIZE_LIMIT_KB > 1024
//       #define NUM_SERVO_CHANNELS 32
//   #else
//       #define NUM_SERVO_CHANNELS 16
//   #endif
// SITL (this port's target) takes the >1024KB branch, so 32 is the real
// upstream contract value for this configuration, not an assumption.
//
// FUNCTION FAN-OUT: upstream allows multiple channels to share one
// function (e.g. two ailerons both tagged k_aileron). Reading each ported
// method's real body (not assuming uniformity - see SRV_Channel_aux.cpp)
// shows the fan-out behavior genuinely differs per method:
//   - set_output_pwm(function, value)   -> ALL matching channels
//   - set_output_norm(function, value)  -> ALL matching channels
//   - set_output_scaled(function, value)-> conceptually ALL matching
//     channels (see below - upstream's real mechanism is different, but
//     ends up applying to every channel with that function)
//   - get_output_pwm(function, value)   -> FIRST matching channel only
//     (upstream's find_channel(), lowest channel index with the function)
//   - get_output_norm(function)         -> FIRST matching channel only
//   - get_output_scaled(function)       -> not channel-indexed at all (see
//     below)
//   - get_output_channel_mask(function) -> ALL matching channels, as a
//     bitmask rather than a value
//
// SURPRISING UPSTREAM FINDING #1 - get_output_pwm/get_output_norm are not
// pure reads: both recompute the first matching channel's output_pwm via
// calc_pwm(cached scaled value) as a side effect before returning it
// (SRV_Channel_aux.cpp: `channels[chan].calc_pwm(functions[function].
// output_scaled); value = channels[chan].get_output_pwm();`). So calling
// get_output_pwm() after a raw set_output_pwm_chan() write can silently
// overwrite that write with whatever the function's last set_output_scaled
// value converts to. Reproduced faithfully below, not smoothed over.
//
// SURPRISING UPSTREAM FINDING #2 - set_output_scaled/get_output_scaled do
// NOT touch any SrvChannel at all. Upstream stores the scaled value in a
// separate function-indexed array (`functions[k_nr_aux_servo_functions]`,
// keyed purely by the Function enum value, entirely independent of which
// channels currently claim that function) and defers turning it into any
// channel's output_pwm until a later, separate global `SRV_Channels::
// calc_pwm()` pass iterates every channel and calls `channels[i].
// calc_pwm(functions[channels[i].function].output_scaled)`. That global
// calc_pwm() driver is not part of this slice's scope (not listed as one
// of the methods to port, and nothing else in scope calls it), so a
// literal port of set_output_scaled alone would be an inert write nothing
// ever reads back into a channel's output_pwm - output_ch_all() would
// never reflect it. JUDGMENT CALL: this port keeps the same
// function-indexed cache (`function_scaled_` below, mirroring upstream's
// `functions[]`) for get_output_scaled's read-back semantics, but
// set_output_scaled ALSO immediately calls calc_pwm(value) on every
// channel currently matching that function, collapsing upstream's
// two-phase store-then-later-pass design into one step. This is the
// smallest change that keeps get_output_scaled's upstream semantics
// (reads the cache, independent of channel assignment) while making
// set_output_scaled observable through output_ch_all without porting the
// registry-wide calc_pwm() driver in this slice.
//
// get_output_scaled's no-match value: upstream's real gate is
// `SRV_Channel::valid_function(fn)`, i.e. `fn >= k_none && fn <
// k_nr_aux_servo_functions` - an index-range check on the function value
// itself, NOT "does any channel currently have this function". (The
// function_scaled_ cache is populated purely by function value, same as
// upstream.) This port's Function enum only defines a representative
// subset of upstream's ~190-entry enum (see srv_channel.hpp's banner), so
// valid_function here checks against kFunctionCacheSize (one past the
// highest value this port's Function enum defines, kMax=136) rather than
// upstream's true k_nr_aux_servo_functions sentinel. For every function
// value this port actually defines, the two checks agree.
//
// get_output_channel_mask: upstream caches this per-function as
// `functions[function].channel_mask`, built and kept in sync by
// update_aux_servo_function()/set_aux_channel_default() - a lazy-cache
// invalidation scheme this slice does not port (it exists to avoid
// rescanning on every call from real-time code; not a scope difference in
// output, just an eager-vs-lazy implementation difference). This port
// recomputes the mask by scanning `channels` directly on every call, which
// yields the identical mask upstream's cache would hold, just computed
// on demand instead of cached and invalidated.
//
// find_channel (private helper, `find_first_channel` below): upstream
// finds the lowest-indexed matching channel via `__builtin_ffs` on a
// cached bitmask; scanning `channels` ascending from index 0 and returning
// the first match is the same "lowest index" result, just without the
// cached-bitmask/ffs implementation detail.
//
// set_default_function: upstream's real version also calls
// `channels[chan].function.set_default(function)` (an AP_Param default,
// not a plain assignment) and updates `function_mask`, a cached
// which-functions-are-in-use bitmask used by function_assigned(). This
// port has no AP_Param wiring into this module yet - same standing
// precedent as AcPid::Gains, L1Control::Gains, and RcChannel's own header
// banner (ap-rc-channel/include/fwcpp/rc/rc_channel.hpp) - so this just
// sets the `.function` field directly, and there is no function_mask to
// maintain since nothing here reads one (get_output_channel_mask scans
// directly instead, see above).
//
// output_ch_all(RcOutput&): ports SRV_Channel::output_ch()'s actual
// write-to-hardware line (`hal.rcout->write(ch_num, output_pwm)`) applied
// to every channel, matching upstream's output_ch_all() loop exactly, but
// EXPLICITLY EXCLUDES two things upstream's real output_ch() also does:
//   - The RC-passthrough special cases: the `k_manual` / `k_rcin1 ...
//     k_rcin16` / `k_rcin1_mapped ... k_rcin16_mapped` switch in
//     SRV_Channel_aux.cpp's output_ch() (around its top), which reads from
//     `rc().channel(passthrough_from)` and overwrites output_pwm from live
//     RC input before writing it out. This needs the RcChannels registry
//     (a separate, possibly-concurrent port effort under ap-rc-channel)
//     wired in, which this slice deliberately does not depend on. A
//     channel configured for one of those functions will, in this port,
//     just output whatever calc_pwm/set_output_pwm last computed for it -
//     never live RC passthrough - until that wiring lands in a later
//     slice.
//   - The `SRV_Channels::disabled_mask` check (`if (!(disabled_mask &
//     (1U<<ch_num))) hal.rcout->write(...)`) - a mask maintained by the
//     AP_BLHeli digital-ESC-protocol backend (set_disabled_channel_mask()),
//     which this port has not ported (see srv_channel.hpp's banner: no
//     AP_Volz/SBusOut/RobotisServo/BLHeli/FETtec protocol backends). Every
//     channel is written unconditionally here; a caller wanting a channel
//     disabled from output can leave it unassigned or write it out of the
//     loop after calling this.
// Neither exclusion changes calc_pwm/set_output_pwm/set_output_norm's own
// results - only which value ultimately reaches the hardware write for a
// passthrough- or BLHeli-disabled channel, and both are genuinely
// out-of-scope subsystems for this slice, not a shortcut on the mapping
// math itself.
//
// DELIBERATELY OUT OF SCOPE for this slice (left unported, not stubbed):
//   - set_slew_rate/get_slew_limited_output_scaled/
//     set_slew_last_scaled_output: this registry-wide, function-indexed
//     slew mechanism is STILL not wired in generically (CPP-038's own
//     decision, see plane.hpp's set_servos_flaps() file-banner note) -
//     CPP-038 needed exactly this mechanism for flap output and, on
//     reading both real upstream sources side by side, found that this
//     module's ORIGINAL note above (claiming fwcpp::filter::SlewLimiter,
//     ap-filter, CPP-015, was the reusable primitive for this) was
//     mistaken: fwcpp::filter::SlewLimiter ports a DIFFERENT upstream
//     file (libraries/Filter/SlewLimiter.h) - a control-theory modifier()
//     that scales down a PID's P+D output, used only by
//     AP_PitchController/AP_RollController's rate loops. SRV_Channel_
//     aux.cpp's own set_slew_rate()/get_slew_limited_output_scaled()
//     (read in full) is algorithmically unrelated: a simple per-function
//     (last_scaled_output, max_change) pair with a constrain_float clamp.
//     Reusing fwcpp::filter::SlewLimiter here would have silently
//     substituted the wrong algorithm. CPP-038 ported the REAL mechanism
//     as a small, new, LOCAL primitive (Plane::FlapSlewState, plane.hpp)
//     scoped to just the two functions ArduPlane's flap code actually
//     needs (k_flap_auto/k_flap) rather than adding a generic registry
//     facility here - see that struct's own file-banner note for the
//     full design rationale and the real, surprising upstream behavior
//     it reproduces (the slew window's reference value is seeded once,
//     at first use, and never advances again on its own). This
//     registry's OWN gap (a genuinely generic, any-function slew list)
//     is still open for a future ticket that needs it for a function
//     other than flap's two.
//   - set_output_min_max/set_output_min_max_defaults/save_output_min_max/
//     save_trim/adjust_trim/auto_trim_enabled: need real AP_Param
//     persistence wiring (parameter save-to-storage) this module doesn't
//     have yet.
//   - set_esc_scaling_for: ESC-specific calibration setup, a
//     vehicle-config-time concern, not core mapping/output.
//   - setup_failsafe_trim_all_non_motors (IO-failsafe setup): needs a real
//     IOMCU/failsafe subsystem this port doesn't have.
//   - set_output_pwm_chan_timeout: needs a scheduled-clear-after-timeout
//     mechanism (a scheduler-integration concern), out of scope for this
//     core slice.

#include <array>
#include <cstddef>
#include <cstdint>

#include <fwcpp/hal/rc_output.hpp>
#include <fwcpp/srv/srv_channel.hpp>

namespace fwcpp::srv {

// Matches upstream NUM_SERVO_CHANNELS's resolved value for this port's
// SITL/Plane target (see file banner) and fwcpp::hal::kNumRcOutputChannels.
inline constexpr std::size_t kNumServoChannels = hal::kNumRcOutputChannels;

// One past the highest Function value this port's (representative subset
// of upstream's) enum defines - see srv_channel.hpp. Sizes the
// per-function scaled-value cache below; see file banner for why this
// exists and how it differs from upstream's true k_nr_aux_servo_functions
// sentinel.
inline constexpr std::size_t kFunctionCacheSize = static_cast<std::size_t>(Function::kMax) + 1U;

class SrvChannels {
public:
    // The vehicle-wide array of servo channels - upstream's
    // `obj_channels[NUM_SERVO_CHANNELS]`, owned directly per this port's
    // no-singleton convention (see file banner) instead of reached through
    // a registry singleton.
    std::array<SrvChannel, kNumServoChannels> channels{};

    // SRV_Channel::valid_function, narrowed to this port's reduced
    // Function enum (see file banner). A function value is valid if it is
    // a defined, non-negative slot in the scaled-value cache below - kGpio
    // (-1) and anything beyond kMax are not.
    [[nodiscard]] static constexpr bool valid_function(Function function) {
        return function >= Function::kNone &&
               static_cast<std::int32_t>(function) < static_cast<std::int32_t>(kFunctionCacheSize);
    }

    // SRV_Channels::find_channel - lowest-indexed channel currently tagged
    // with `function`. Returns false (chan left unmodified) if none match
    // or `function` is not valid_function(); see file banner for why this
    // is a direct scan rather than a cached-bitmask/ffs lookup.
    [[nodiscard]] bool find_first_channel(Function function, std::uint8_t& chan) const {
        if (!valid_function(function)) {
            return false;
        }
        for (std::uint8_t i = 0; i < kNumServoChannels; ++i) {
            if (channels[i].function == function) {
                chan = i;
                return true;
            }
        }
        return false;
    }

    // SRV_Channels::get_output_channel_mask - bitmask of every channel
    // currently tagged with `function`, valid or not (see file banner: an
    // invalid function value, e.g. kGpio, simply yields the mask of
    // channels tagged with that exact value via the same scan, which this
    // port's enum only ever assigns kGpio to - no separate coarse
    // "invalid_mask" bucket is needed here).
    [[nodiscard]] std::uint32_t get_output_channel_mask(Function function) const {
        std::uint32_t mask = 0;
        for (std::uint8_t i = 0; i < kNumServoChannels; ++i) {
            if (channels[i].function == function) {
                mask |= (1U << i);
            }
        }
        return mask;
    }

    // SRV_Channels::set_output_pwm - ALL channels matching `function` get
    // `value` written to their output_pwm directly (SrvChannel::
    // set_output_pwm, no scaling). Upstream also calls channels[i].
    // output_ch() here, writing straight to hal.rcout as an immediate side
    // effect; this port has no RcOutput& in this signature (per this
    // slice's scope), so the write only updates in-memory channel state -
    // the actual hardware write happens later, when the caller invokes
    // output_ch_all(RcOutput&) below. This matches this port's explicit-
    // context convention (hardware access threaded explicitly, not reached
    // implicitly) rather than being a missed step.
    void set_output_pwm(Function function, std::uint16_t value) {
        for (std::uint8_t i = 0; i < kNumServoChannels; ++i) {
            if (channels[i].function == function) {
                channels[i].set_output_pwm(value);
            }
        }
    }

    // SRV_Channels::get_output_pwm - FIRST matching channel only. Not a
    // pure read: recomputes that channel's output_pwm from the function's
    // cached scaled value before returning it, exactly matching upstream's
    // real (surprising) behavior - see file banner finding #1. Returns
    // false (value left unmodified) if no channel currently has this
    // function.
    [[nodiscard]] bool get_output_pwm(Function function, std::uint16_t& value) {
        std::uint8_t chan = 0;
        if (!find_first_channel(function, chan)) {
            return false;
        }
        channels[chan].calc_pwm(function_scaled_[static_cast<std::size_t>(function)]);
        value = channels[chan].get_output_pwm();
        return true;
    }

    // SRV_Channels::set_output_pwm_chan - direct by-index write,
    // bounds-checked (matches upstream's `chan < NUM_SERVO_CHANNELS`
    // guard exactly; out-of-range is a silent no-op upstream too).
    void set_output_pwm_chan(std::uint8_t chan, std::uint16_t value) {
        if (chan < kNumServoChannels) {
            channels[chan].set_output_pwm(value);
        }
    }

    // SRV_Channels::get_output_pwm_chan - direct by-index read,
    // bounds-checked. Returns false (value left unmodified) if chan is out
    // of range.
    [[nodiscard]] bool get_output_pwm_chan(std::uint8_t chan, std::uint16_t& value) const {
        if (chan >= kNumServoChannels) {
            return false;
        }
        value = channels[chan].get_output_pwm();
        return true;
    }

    // SRV_Channels::set_output_norm - ALL matching channels, each via
    // SrvChannel::set_output_norm (immediate: converts to pwm and stores,
    // no cache involved - unlike set_output_scaled below).
    void set_output_norm(Function function, float value) {
        for (std::uint8_t i = 0; i < kNumServoChannels; ++i) {
            if (channels[i].function == function) {
                channels[i].set_output_norm(value);
            }
        }
    }

    // SRV_Channels::get_output_norm - FIRST matching channel only, with
    // the same recompute-before-read side effect as get_output_pwm above
    // (file banner finding #1). Returns 0.0F if no channel currently has
    // this function (matches upstream's `return 0;` on find_channel
    // failure).
    [[nodiscard]] float get_output_norm(Function function) {
        std::uint8_t chan = 0;
        if (!find_first_channel(function, chan)) {
            return 0.0F;
        }
        channels[chan].calc_pwm(function_scaled_[static_cast<std::size_t>(function)]);
        return channels[chan].get_output_norm();
    }

    // SRV_Channels::set_output_scaled - see file banner finding #2 for why
    // this both updates the per-function cache (matching upstream's real
    // read-back semantics for get_output_scaled) AND immediately fans out
    // calc_pwm(value) to every channel currently matching `function`
    // (collapsing upstream's separate, out-of-scope global calc_pwm()
    // pass into this call). A no-op if `function` is not valid_function().
    void set_output_scaled(Function function, float value) {
        if (!valid_function(function)) {
            return;
        }
        function_scaled_[static_cast<std::size_t>(function)] = value;
        for (std::uint8_t i = 0; i < kNumServoChannels; ++i) {
            if (channels[i].function == function) {
                channels[i].calc_pwm(value);
            }
        }
    }

    // SRV_Channels::get_output_scaled - reads the per-function cache
    // directly, independent of channel assignment (file banner finding
    // #2). Returns 0.0F for a function that is not valid_function()
    // (matches upstream's real `return 0;` on that check) - the cache
    // itself also defaults every slot to 0.0F, so a valid function nothing
    // has ever set_output_scaled() also reads back as 0.0F, same as
    // upstream.
    [[nodiscard]] float get_output_scaled(Function function) const {
        if (!valid_function(function)) {
            return 0.0F;
        }
        return function_scaled_[static_cast<std::size_t>(function)];
    }

    // SRV_Channels::set_default_function - bounds-checked, sets `.function`
    // directly. See file banner for why this skips upstream's AP_Param
    // default-value and function_mask bookkeeping (established
    // AP_Param-not-wired-in-yet precedent, not a new judgment call here).
    void set_default_function(std::uint8_t chan, Function function) {
        if (chan < kNumServoChannels) {
            channels[chan].function = function;
        }
    }

    // SRV_Channels::output_ch_all - writes every channel's already-computed
    // output_pwm to the given RcOutput, one call per physical channel
    // index. See file banner for the two things upstream's real
    // output_ch() also does that are explicitly excluded here
    // (RC-passthrough special cases and the BLHeli disabled_mask check).
    void output_ch_all(hal::RcOutput& out) const {
        for (std::uint8_t i = 0; i < kNumServoChannels; ++i) {
            out.write(i, channels[i].output_pwm);
        }
    }

private:
    // Per-function cached scaled value - mirrors upstream's
    // `functions[k_nr_aux_servo_functions]` array (SRV_Channels.h),
    // indexed by Function value rather than by channel. See file banner
    // finding #2 for why set_output_scaled/get_output_scaled need this
    // instead of reading/writing a channel directly.
    std::array<float, kFunctionCacheSize> function_scaled_{};
};

} // namespace fwcpp::srv
