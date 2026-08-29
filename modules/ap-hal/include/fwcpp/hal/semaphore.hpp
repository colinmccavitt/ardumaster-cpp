#pragma once

// Port of AP_HAL/Semaphores.h take/give and BinarySemaphore wait/signal,
// as an in-memory counter/flag for SITL unit tests. CPP-088 slice 1.
//
// No OS threads and no locking invented for Plane tick() — a depth
// counter (recursive mutex) and a pending flag (binary) are enough to
// exercise the contract. A contended/unsignaled wait refuses rather
// than sleeping; there is no scheduler here to wake it.
//
// Upstream documents that every AP_HAL::Semaphore is recursive: the
// holder may take again and must give the same number of times.
// take(0) is HAL_SEMAPHORE_BLOCK_FOREVER, not a zero-timeout try —
// the non-blocking path is take_nonblocking(). Binary wait(0) is the
// opposite: non-blocking (wait_nonblocking).

#include <cstdint>

namespace fwcpp::hal {

// Upstream HAL_SEMAPHORE_BLOCK_FOREVER.
inline constexpr std::uint32_t kSemaphoreBlockForeverMs = 0;

class Semaphore {
public:
    [[nodiscard]] bool take(std::uint32_t timeout_ms) {
        if (try_acquire()) {
            return true;
        }
        // Contended and free: a positive timeout still cannot wait in
        // this SITL stub. Block-forever (0) also refuses.
        (void)timeout_ms;
        return false;
    }

    [[nodiscard]] bool take_nonblocking() { return try_acquire(); }

    void take_blocking() { (void)take(kSemaphoreBlockForeverMs); }

    [[nodiscard]] bool give() {
        if (depth_ == 0) {
            return false;
        }
        --depth_;
        return true;
    }

    [[nodiscard]] std::uint32_t depth() const { return depth_; }

    [[nodiscard]] bool is_contended() const { return contended_; }

    // Simulate another owner so take_nonblocking can fail. Contended
    // only applies while free; a recursive take from the holder still
    // succeeds.
    void set_contended(bool contended) { contended_ = contended; }

private:
    [[nodiscard]] bool try_acquire() {
        if (depth_ == 0 && contended_) {
            return false;
        }
        ++depth_;
        return true;
    }

    std::uint32_t depth_ = 0;
    bool contended_ = false;
};

// Upstream AP_HAL::BinarySemaphore. initial_state true means a wait
// immediately after creation does not block.
class BinarySemaphore {
public:
    explicit BinarySemaphore(bool initial_state = false) : pending_(initial_state) {}

    [[nodiscard]] bool wait(std::uint32_t timeout_us) {
        if (pending_) {
            pending_ = false;
            return true;
        }
        (void)timeout_us;
        return false;
    }

    [[nodiscard]] bool wait_blocking() {
        if (pending_) {
            pending_ = false;
            return true;
        }
        return false;
    }

    [[nodiscard]] bool wait_nonblocking() { return wait(0); }

    void signal() { pending_ = true; }

    void signal_isr() { signal(); }

    [[nodiscard]] bool is_pending() const { return pending_; }

private:
    bool pending_ = false;
};

}  // namespace fwcpp::hal
