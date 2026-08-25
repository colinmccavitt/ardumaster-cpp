#pragma once

// Port of AP_HAL/RCOutput.h's observable contract, matched against
// AP_HAL_SITL/RCOutput.cpp (real PWM/DShot generation, set_freq,
// per-channel frequency groups, and serial-LED emulation are hardware
// concerns genuinely out of scope - SITL's own backend just holds a
// plain value array too). CPP-025, slice 2.
//
// cork()/push() reproduced exactly: cork() snapshots the current output
// array into a pending buffer and starts buffering further write()
// calls into it instead of the live array; push() atomically flushes
// the pending buffer back (or, if never corked, write() already went
// straight to the live array) - lets a caller update several channels
// without an observer seeing a torn intermediate state mid-update.
//
// SAFETY STATE reproduced with an explicit safety_mask parameter
// instead of reaching for AP_BoardConfig (a real subsystem this port
// hasn't ported - get_safety_mask() lets specific channels, e.g. for
// lighting, stay live even while disarmed). Matches this port's
// standing explicit-context-over-singleton convention: callers who need
// mask-exempted channels pass the mask directly to write(); the default
// of 0 exempts nothing, matching a board with no configured exceptions.

#include <array>
#include <cstdint>
#include <cstring>

namespace fwcpp::hal {

inline constexpr std::uint8_t kNumRcOutputChannels = 32; // matches SITL_NUM_CHANNELS

enum class SafetyState : std::uint8_t {
    kDisarmed = 0,
    kArmed = 1,
};

class RcOutput {
public:
    // safety_mask: bit i set means channel i stays live even while
    // disarmed (matches AP_BoardConfig::get_safety_mask()'s role -
    // see file banner).
    void write(std::uint8_t ch, std::uint16_t period_us, std::uint32_t safety_mask = 0) {
        if (ch >= kNumRcOutputChannels) {
            return;
        }
        if (safety_state_ == SafetyState::kDisarmed && !((safety_mask >> ch) & 1U)) {
            period_us = 0;
        }
        if (corked_) {
            pending_[ch] = period_us;
        } else {
            outputs_[ch] = period_us;
        }
    }

    [[nodiscard]] std::uint16_t read(std::uint8_t ch) const {
        return ch < kNumRcOutputChannels ? outputs_[ch] : 0;
    }

    void read(std::uint16_t* period_us, std::uint8_t len) const {
        const std::uint8_t n = len < kNumRcOutputChannels ? len : kNumRcOutputChannels;
        std::memcpy(period_us, outputs_.data(), n * sizeof(std::uint16_t));
    }

    void cork() {
        if (!corked_) {
            pending_ = outputs_;
            corked_ = true;
        }
    }

    void push() {
        if (corked_) {
            outputs_ = pending_;
            corked_ = false;
        }
    }

    void enable_ch(std::uint8_t ch) {
        if (ch < kNumRcOutputChannels) {
            enable_mask_ |= (1U << ch);
        }
    }
    void disable_ch(std::uint8_t ch) {
        if (ch < kNumRcOutputChannels) {
            enable_mask_ &= ~(1U << ch);
        }
    }
    [[nodiscard]] bool channel_enabled(std::uint8_t ch) const {
        return ch < kNumRcOutputChannels && ((enable_mask_ >> ch) & 1U);
    }

    void force_safety_on() { safety_state_ = SafetyState::kDisarmed; }
    void force_safety_off() { safety_state_ = SafetyState::kArmed; }
    [[nodiscard]] SafetyState safety_state() const { return safety_state_; }

private:
    std::array<std::uint16_t, kNumRcOutputChannels> outputs_{};
    std::array<std::uint16_t, kNumRcOutputChannels> pending_{};
    bool corked_ = false;
    std::uint32_t enable_mask_ = 0;
    SafetyState safety_state_ = SafetyState::kDisarmed; // matches upstream's own SITL default
};

} // namespace fwcpp::hal
