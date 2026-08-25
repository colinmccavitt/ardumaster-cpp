#pragma once

// Port of AP_HAL/RCInput.h's observable contract (num_channels/read/
// new_input/get_rssi), matched against what AP_HAL_SITL/RCInput.cpp
// actually does for SITL rather than the full generic multi-backend HAL
// interface. CPP-025, slice 1. See tracker CPP-025 and the fw-rust ap-hal
// crate (time/rc/serial/analog/context) used as scope precedent.
//
// SITL's own RCInput (AP_HAL_SITL/RCInput.cpp) just delegates
// num_channels/read/new_input straight through to AP_RCProtocol - a
// shared, real serial-protocol decoder (SBUS/PPM/etc, fed by a
// simulated byte stream) every board (real or simulated) uses
// identically. That decoder is a real subsystem of its own, genuinely
// out of scope here (this port has no serial byte stream to decode -
// see CPP-025's own UARTDriver scope, still minimal). What THIS port
// needs is the decoder's OBSERVABLE CONTRACT: channel values that
// something (a test harness, eventually CPP-030's ap-sim oracle, or a
// MAVLink RC_CHANNELS_OVERRIDE handler) can inject directly - matching
// this port's standing pattern of replacing a hardware/protocol
// mechanism with an explicit value holder (RawStorage vs. hal.storage's
// flash emulation, CPP-025's own RcOutput vs. real PWM generation).
//
// new_input() semantics: true exactly once per set_channels() call,
// then false until the next one - the same "has new data arrived since
// I last checked" contract callers need from a decoder's dirty flag,
// without needing this port to reproduce the decoder itself.

#include <algorithm>
#include <array>
#include <cstdint>

namespace fwcpp::hal {

inline constexpr std::uint8_t kNumRcChannels = 32; // matches upstream's SITL_NUM_CHANNELS

class RcInput {
public:
    // Injects a full set of channel values (e.g. from a test harness or
    // a MAVLink override handler) and marks new_input() true until the
    // next call.
    void set_channels(const std::array<std::uint16_t, kNumRcChannels>& channels) {
        channels_ = channels;
        new_input_ = true;
    }

    void set_channel(std::uint8_t ch, std::uint16_t value) {
        if (ch < kNumRcChannels) {
            channels_[ch] = value;
            new_input_ = true;
        }
    }

    [[nodiscard]] bool new_input() {
        const bool result = new_input_;
        new_input_ = false;
        return result;
    }

    [[nodiscard]] std::uint8_t num_channels() const { return kNumRcChannels; }

    [[nodiscard]] std::uint16_t read(std::uint8_t ch) const {
        return ch < kNumRcChannels ? channels_[ch] : 0;
    }

    // Matches upstream's `read(periods*, len)` -> returns the number of
    // channels actually copied (min(len, num_channels())).
    std::uint8_t read(std::uint16_t* periods, std::uint8_t len) const {
        const std::uint8_t n = std::min<std::uint8_t>(len, kNumRcChannels);
        std::copy_n(channels_.begin(), n, periods);
        return n;
    }

    // -1 for unknown, matching upstream's own default (no RSSI source in
    // this port yet).
    [[nodiscard]] std::int16_t get_rssi() const { return -1; }

private:
    std::array<std::uint16_t, kNumRcChannels> channels_{};
    bool new_input_ = false;
};

} // namespace fwcpp::hal
