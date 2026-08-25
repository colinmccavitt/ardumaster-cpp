#pragma once

// Port of AP_Common/Bitmask.h. CPP-010.
//
// Fixed-size (compile-time NUMBITS), no dynamic allocation - bits[] is a
// plain array sized at compile time, matching ADR-0012 decision 4 with no
// scope decision needed (unlike matrix_alg's mat_inverseN).
//
// ERROR REPORTING DESIGN: upstream's validate() calls
// INTERNAL_ERROR(AP_InternalError::error_t::bitmask_range) on an
// out-of-range bit access, reaching the AP::internalerror() singleton.
// ADR-0012 decision 6 forbids that, and CPP-005 built the explicit
// replacement (fwcpp::InternalError) - but Bitmask is different from
// constrain_value/Quaternion::normalize in one respect: it's a data type
// instantiated as an ordinary member variable all over a vehicle (parameter
// enable-flags, mode-available bitmasks, ...), not a single free function
// called at one call site. Threading an `InternalError*` through every
// set/clear/get call, changing their signatures from upstream's, would be
// a much bigger API surface change than constrain_value's single added
// parameter - and ADR-0012 decision 9 (stay close to upstream's surface for
// diffability) argues against that here specifically.
//
// Instead the InternalError* is a member of the Bitmask itself, settable
// once (defaults to nullptr, matching a build with reporting disabled) via
// `set_error_sink`. set/clear/get keep upstream's exact signatures. This is
// still explicit and non-singleton - the pointer lives on the object, set
// by whoever constructs it, never reached through a global - just threaded
// through construction/configuration instead of through every call.

#include <cstdint>
#include <cstring>

#include <fwcpp/internal_error.hpp>

namespace fwcpp {

template <std::uint16_t NumBits>
class Bitmask {
    static constexpr std::uint16_t kNumWords = (NumBits + 31) / 32;

    static_assert(NumBits > 0, "must store something");
    static_assert(NumBits <= INT16_MAX, "must fit in int16_t");

public:
    Bitmask() { clearall(); }

    Bitmask(const Bitmask&) = delete;
    Bitmask& operator=(const Bitmask& other) {
        std::memcpy(bits_, other.bits_, sizeof(bits_));
        return *this;
    }

    [[nodiscard]] bool operator==(const Bitmask& other) const {
        return std::memcmp(bits_, other.bits_, sizeof(bits_)) == 0;
    }
    [[nodiscard]] bool operator!=(const Bitmask& other) const { return !(*this == other); }

    // Construct with some bits pre-enabled. Out-of-range entries in the
    // initializer are silently skipped, matching upstream (which checks
    // `< NumBits` directly here rather than going through set()'s
    // validate/report path - this is upstream's own choice, not this
    // port's: a compile-time-known bad literal in the initializer list
    // doesn't get the same runtime report a bad runtime bit index would).
    template <std::size_t N>
    explicit Bitmask(const std::uint16_t (&enabled_bits)[N]) {
        clearall();
        for (std::size_t i = 0; i < N; ++i) {
            if (enabled_bits[i] < NumBits) {
                set(enabled_bits[i]);
            }
        }
    }

    // See file banner: not part of upstream's API, this port's explicit
    // substitute for upstream reaching AP::internalerror() implicitly.
    void set_error_sink(InternalError* err, std::uint16_t line = 0) {
        error_sink_ = err;
        error_line_ = line;
    }

    void set(std::uint16_t bit) {
        if (!validate(bit)) {
            return;
        }
        bits_[bit / 32] |= (1U << (bit & 0x1f));
    }

    void setall() {
        for (auto& w : bits_) {
            w = 0xffffffffU;
        }
        const std::uint16_t num_valid_bits = NumBits % 32;
        if (num_valid_bits) {
            bits_[kNumWords - 1] = (1U << num_valid_bits) - 1;
        }
    }

    void clear(std::uint16_t bit) {
        if (!validate(bit)) {
            return;
        }
        bits_[bit / 32] &= ~(1U << (bit & 0x1f));
    }

    void setonoff(std::uint16_t bit, bool onoff) {
        if (onoff) {
            set(bit);
        } else {
            clear(bit);
        }
    }

    void clearall() { std::memset(bits_, 0, sizeof(bits_)); }

    [[nodiscard]] bool get(std::uint16_t bit) const {
        if (!validate(bit)) {
            return false;
        }
        return (bits_[bit / 32] & (1U << (bit & 0x1f))) != 0;
    }

    [[nodiscard]] bool empty() const {
        for (auto w : bits_) {
            if (w != 0) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] std::uint16_t count() const {
        std::uint16_t sum = 0;
        for (auto w : bits_) {
            sum += static_cast<std::uint16_t>(__builtin_popcount(w));
        }
        return sum;
    }

    // First set bit, or -1 if none.
    [[nodiscard]] std::int16_t first_set() const {
        for (std::uint16_t i = 0; i < kNumWords; ++i) {
            if (bits_[i] != 0) {
                return static_cast<std::int16_t>(i * 32 + __builtin_ffs(static_cast<int>(bits_[i])) - 1);
            }
        }
        return -1;
    }

    [[nodiscard]] std::uint16_t size() const { return NumBits; }

private:
    bool validate(std::uint16_t bit) const {
        if (bit >= NumBits) {
            if (error_sink_ != nullptr) {
                error_sink_->record(InternalErrorCode::bitmask_range, error_line_);
            }
            return false;
        }
        return true;
    }

    std::uint32_t bits_[kNumWords];
    InternalError* error_sink_ = nullptr;
    std::uint16_t error_line_ = 0;
};

} // namespace fwcpp
