#pragma once

// Port of AP_HAL/UARTDriver.h's byte-stream read/write contract
// (available/read/write/txspace-equivalent), matched against what
// AP_HAL_SITL/UARTDriver actually does rather than the full generic
// interface. CPP-025-style scope precedent: see rc_input.hpp/rc_output.hpp
// in this same module for the established "match SITL's real backend, not
// the generic multi-board interface" methodology.
//
// AP_HAL_SITL::UARTDriver (AP_HAL_SITL/UARTDriver.h/.cpp) holds exactly two
// plain ring buffers as its actual data-carrying state:
//   ByteBuffer _readbuffer{16384};
//   ByteBuffer _writebuffer{16384};
// _write() copies caller bytes into _writebuffer; a background step
// (handle_writing_from_writebuffer_to_device) later drains _writebuffer to
// whatever real transport is configured for that port - a TCP/UDP socket,
// a pty, or (for the console UART, portNumber 0 in SITL_State) a plain
// stdio fd. Symmetrically, _read() pulls from _readbuffer, filled by
// handle_reading_from_device_to_readbuffer() reading off that same
// transport. In other words: the byte-buffer contract is real, shared,
// and transport-independent; the socket/pty/stdio plumbing on the far
// side is genuinely a separate concern SITL itself keeps behind _fd/
// _mc_fd/_listen_fd and _sitlState.
//
// This slice ports the two ring buffers and the read/write/available
// contract built on top of them - deliberately NOT the network transports
// (TCP/UDP/multicast socket setup, pty allocation) or baud-rate timing
// simulation (DataRateLimit) that AP_HAL_SITL::UARTDriver layers on top.
// Those are real, separate subsystems callers can add later as an
// explicit transport object that drains/fills these buffers from the
// outside - exactly the seam SITL's own handle_writing_from_writebuffer_to_device
// / handle_reading_from_device_to_readbuffer methods already are. A test
// harness (or, eventually, that transport) plays that role here via
// inject_rx()/drain_tx().
//
// Buffer-full policy: upstream's own ByteBuffer::write() (AP_HAL/utility/
// RingBuffer.cpp) clamps the requested length to ByteBuffer::space() before
// copying and returns the number of bytes actually accepted - i.e. a
// partial write when the buffer can't hold everything, never an overwrite
// of unread bytes and never silent full-call data corruption. This port
// reproduces that exact policy for both bulk write(buf, len) (returns
// count actually accepted) and single-byte write(uint8_t) (returns false,
// accepts nothing, when the buffer is already full). Overwrite-oldest was
// considered and rejected: it would silently discard bytes a reader has
// not yet seen, which is exactly the "silent data corruption" this port's
// conventions forbid; reject/partial-accept instead makes the caller's
// own txspace()-style check (space_for_write()) the correct way to avoid
// loss, matching upstream's contract.

#include <array>
#include <cstddef>
#include <cstdint>

namespace fwcpp::hal {

// Matches AP_HAL_SITL::UARTDriver's own default buffer size for both
// directions (see file banner). Callers needing a different size (e.g.
// tests exercising full-buffer behavior without writing 16KB) can pass a
// smaller Capacity explicitly - the ring buffer logic is identical either
// way.
inline constexpr std::size_t kDefaultUartBufferBytes = 16384;

// A single fixed-capacity byte ring buffer: the shared shape behind both
// UartDriver's RX and TX buffers (see file banner - upstream also uses one
// ByteBuffer type for both directions of its UARTDriver).
template <std::size_t Capacity>
class ByteRingBuffer {
public:
    static_assert(Capacity > 0, "ring buffer capacity must be nonzero");

    // Bytes currently queued and available to read.
    [[nodiscard]] std::size_t available() const { return count_; }

    // Free space currently available to write (upstream's txspace()/
    // ByteBuffer::space() equivalent).
    [[nodiscard]] std::size_t space() const { return Capacity - count_; }

    [[nodiscard]] bool empty() const { return count_ == 0; }
    [[nodiscard]] bool full() const { return count_ == Capacity; }

    // Single-byte push. Returns false and accepts nothing if full - see
    // file banner for why reject (not overwrite-oldest) is this port's
    // buffer-full policy.
    bool push(std::uint8_t b) {
        if (full()) {
            return false;
        }
        buf_[tail_] = b;
        tail_ = (tail_ + 1) % Capacity;
        ++count_;
        return true;
    }

    // Bulk push. Accepts as many leading bytes of [data, data+len) as fit
    // and returns that count - a partial write on overflow, matching
    // upstream ByteBuffer::write()'s clamp-to-space() behavior exactly
    // (see file banner). Never overwrites unread bytes.
    std::size_t push(const std::uint8_t* data, std::size_t len) {
        const std::size_t n = len < space() ? len : space();
        for (std::size_t i = 0; i < n; ++i) {
            buf_[tail_] = data[i];
            tail_ = (tail_ + 1) % Capacity;
        }
        count_ += n;
        return n;
    }

    // Single-byte pop. Returns false (buffer untouched) if empty.
    bool pop(std::uint8_t& out) {
        if (empty()) {
            return false;
        }
        out = buf_[head_];
        head_ = (head_ + 1) % Capacity;
        --count_;
        return true;
    }

    // Bulk pop. Copies up to len available bytes into data and returns the
    // count actually copied (may be less than len if fewer bytes are
    // queued - matching upstream ByteBuffer::read()'s own behavior).
    std::size_t pop(std::uint8_t* data, std::size_t len) {
        const std::size_t n = len < count_ ? len : count_;
        for (std::size_t i = 0; i < n; ++i) {
            data[i] = buf_[head_];
            head_ = (head_ + 1) % Capacity;
        }
        count_ -= n;
        return n;
    }

    void clear() {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

private:
    std::array<std::uint8_t, Capacity> buf_{};
    std::size_t head_ = 0;  // next index to pop from
    std::size_t tail_ = 0;  // next index to push to
    std::size_t count_ = 0; // bytes queued (avoids the classic
                             // head==tail "empty or full?" ambiguity -
                             // upstream instead reserves one slot,
                             // see RingBuffer.cpp's space()/-1; this
                             // port tracks count explicitly instead,
                             // an equivalent and simpler substitute for a
                             // fixed std::array-backed buffer)
};

// Minimal UARTDriver-equivalent: one RX ring buffer, one TX ring buffer,
// and the read/write/available contract built directly on top of them.
// See file banner for what's deliberately out of scope (network
// transports, baud-rate timing) and the buffer-full policy.
//
// No transport is wired in at this slice - a test harness (see
// uart_driver_test.cpp) or a future transport object drives this class
// from the outside via inject_rx() (simulating bytes having arrived on
// the wire) and drain_tx() (taking bytes this side wrote, as a real
// transport's write-out step would).
template <std::size_t Capacity = kDefaultUartBufferBytes>
class UartDriver {
public:
    // ---- AP_HAL::UARTDriver-equivalent read/write contract ----

    // RX bytes ready to read (upstream's available()).
    [[nodiscard]] std::size_t available() const { return rx_.available(); }

    // TX free space (upstream's txspace()/available_to_write()).
    [[nodiscard]] std::size_t txspace() const { return tx_.space(); }

    // Single-byte read. Returns false with *b left unmodified if RX is
    // empty (upstream's bool read(uint8_t&) overload).
    bool read(std::uint8_t& b) { return rx_.pop(b); }

    // Bulk read. Returns the number of bytes actually copied (may be
    // fewer than count if RX doesn't have that many queued).
    std::size_t read(std::uint8_t* buffer, std::size_t count) {
        return rx_.pop(buffer, count);
    }

    // Single-byte write. Returns false if TX is full (see file banner for
    // the reject-on-full policy).
    bool write(std::uint8_t c) { return tx_.push(c); }

    // Bulk write. Returns the number of bytes actually accepted into TX -
    // a partial write on overflow (see file banner).
    std::size_t write(const std::uint8_t* buffer, std::size_t size) {
        return tx_.push(buffer, size);
    }

    // ---- Test/transport injection-and-drain surface ----
    // Mirrors how RcInput::set_channel lets a test (or, there, a MAVLink
    // override handler) inject values directly instead of this port
    // reproducing the real decoder/transport that would normally produce
    // them (see rc_input.hpp). Here the thing genuinely out of scope is
    // the network/serial transport (see file banner); a test or future
    // transport object plays that role via these two calls.

    // Simulates bytes having arrived on the wire: pushes into RX exactly
    // as a transport's read-in step would. Returns the count actually
    // accepted (partial on overflow, same policy as write()).
    std::size_t inject_rx(const std::uint8_t* data, std::size_t len) {
        return rx_.push(data, len);
    }

    // Drains up to len bytes this side has written to TX, as a real
    // transport's write-out step would - lets a test (or future
    // transport) observe what was sent. Returns the count actually
    // copied out.
    std::size_t drain_tx(std::uint8_t* data, std::size_t len) {
        return tx_.pop(data, len);
    }

    [[nodiscard]] std::size_t rx_available() const { return rx_.available(); }
    [[nodiscard]] std::size_t tx_available() const { return tx_.available(); }

private:
    ByteRingBuffer<Capacity> rx_;
    ByteRingBuffer<Capacity> tx_;
};

} // namespace fwcpp::hal
