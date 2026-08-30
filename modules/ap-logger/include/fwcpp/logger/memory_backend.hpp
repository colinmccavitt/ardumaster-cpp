#pragma once

// Port of AP_Logger_Backend::WriteBlock and AP_Logger_Block::StartWrite /
// FinishWrite (ticket name: EndWrite). CPP-090, slice 1.
//
// LogBackend is the write-path seam: three function-like methods on the
// concrete MemoryBackend. No vtable (ADR-0012: no RTTI). No filesystem.
//
// MemoryBackend is not a DataFlash device: there is no page erase, wrap,
// or log index. The RAM DataFlash page map (dataflash_page_map.hpp) is a
// separate device model with BufferToPage / PageToBuffer. WriteBlock
// appends into a fixed-capacity buffer (std::array, compile-time cap).
// A write that would overflow is dropped whole and counted (upstream
// _dropped when space < size). No unbounded growth on the flight path.

#include <fwcpp/logger/rate_limiter.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace fwcpp::logger {

// Write-path seam matching AP_Logger_Backend / AP_Logger_Block. Bound as
// methods on MemoryBackend rather than a virtual base.
//   StartWrite(page_adr)  — AP_Logger_Block::StartWrite
//   WriteBlock(bytes)     — AP_Logger_Backend::WriteBlock
//   EndWrite()            — AP_Logger_Block::FinishWrite (ticket: EndWrite)

template <std::size_t Capacity>
class MemoryBackend {
public:
    MemoryBackend() = default;

    void StartWrite(std::uint32_t page_adr) {
        page_adr_ = page_adr;
        writing_ = true;
    }

    // Copy `bytes` at the current offset. Returns true when the whole
    // block was accepted. A full buffer drops the block and increments
    // num_dropped() (upstream _WritePrioritisedBlock when space < size).
    [[nodiscard]] bool WriteBlock(std::span<const std::uint8_t> bytes) {
        const std::size_t n = bytes.size();
        if (n > Capacity || len_ > Capacity - n) {
            if (dropped_ < std::numeric_limits<std::uint32_t>::max()) {
                ++dropped_;
            }
            return false;
        }
        std::copy(bytes.begin(), bytes.end(), storage_.begin() + static_cast<std::ptrdiff_t>(len_));
        len_ += n;
        return true;
    }

    void EndWrite() {
        writing_ = false;
        if (ended_ < std::numeric_limits<std::uint32_t>::max()) {
            ++ended_;
        }
    }

    // WriteStreaming-style path: WritePrioritisedBlock's non-critical
    // rate_limiter->should_log(msgbuf[2], writev_streaming) gate
    // (Backend.cpp ~457-461), using should_log_streaming only. A failed
    // gate drops the block without incrementing num_dropped (rate-limit
    // is not a buffer-full drop). A pass is WriteBlock as usual.
    [[nodiscard]] bool WriteStreaming(std::span<const std::uint8_t> bytes,
                                      std::uint8_t msgid,
                                      std::uint16_t now_ms16,
                                      float rate_hz,
                                      bool log_pause) {
        if (!rate_limiter_.should_log_streaming(msgid, now_ms16, rate_hz, log_pause)) {
            return false;
        }
        return WriteBlock(bytes);
    }

    [[nodiscard]] const RateLimiter& rate_limiter() const { return rate_limiter_; }

    [[nodiscard]] std::span<const std::uint8_t> recorded() const {
        return std::span<const std::uint8_t>(storage_.data(), len_);
    }

    [[nodiscard]] std::uint32_t page_adr() const { return page_adr_; }
    [[nodiscard]] bool is_writing() const { return writing_; }
    [[nodiscard]] std::uint32_t ended_writes() const { return ended_; }
    [[nodiscard]] std::uint32_t num_dropped() const { return dropped_; }
    [[nodiscard]] std::size_t capacity() const { return Capacity; }
    [[nodiscard]] std::size_t size() const { return len_; }

private:
    std::array<std::uint8_t, Capacity> storage_{};
    std::size_t len_{0};
    std::uint32_t page_adr_{0};
    bool writing_{false};
    std::uint32_t ended_{0};
    std::uint32_t dropped_{0};
    RateLimiter rate_limiter_{};
};

}  // namespace fwcpp::logger
