#pragma once

// Port of AP_HAL::CANIface send/receive as fixed in-memory queues.
// CPP-088 slice 4.
//
// Concrete caller-owned type. No virtual AP_HAL::CANIface base, no
// OwnPtr, no HAL singleton. HAL_CANFD_SUPPORTED is not defined in this
// port, so CANFrame::MaxDataLen is 8 (classic CAN).
//
// send / receive return codes match AP_HAL::CANIface:
//   send:    -1 if !initialized, 0 if TX full, 1 if queued
//   receive: 0 if RX empty, 1 if popped
// init(bitrate) stores the bitrate and returns true. The fdbitrate
// overload ignores fdbitrate (same as HALSITL::CANIface::init).
//
// Loopback (SITL-subset, disclosed vs SocketCAN):
//   A successful send also enqueues the same frame on RX so tests can
//   round-trip without a kernel bus. Upstream HALSITL::CANIface::send
//   pushes TX then _pollWrite/_pollRead against CAN_SocketCAN (PF_CAN
//   SOCK_RAW + ioctl SIOCGIFINDEX + bind vcanN) or CAN_Multicast
//   (UDP 239.65.82.N:57732). Linux SocketCAN defaults CAN_RAW_LOOPBACK,
//   so a sent frame can reappear on RX via the kernel. This port copies
//   the frame in-process and never opens a socket, never ioctl, never
//   poll(). Deadline is stored on the TX item and unused (no poll-write
//   timeout path).
//
// set_event_handle stores a non-owning BinarySemaphore* (slice 1) and
// signals on TX enqueue. select() / poll() / get_stats are not ported.

#include <array>
#include <cstdint>
#include <cstring>

#include <fwcpp/hal/semaphore.hpp>

namespace fwcpp::hal {

inline constexpr std::size_t kCanQueueCapacity = 16;

struct CANFrame {
    static constexpr std::uint32_t MaskStdID = 0x000007FFU;
    static constexpr std::uint32_t MaskExtID = 0x1FFFFFFFU;
    static constexpr std::uint32_t FlagEFF = 1U << 31;
    static constexpr std::uint32_t FlagRTR = 1U << 30;
    static constexpr std::uint32_t FlagERR = 1U << 29;

    static constexpr std::uint8_t NonFDCANMaxDataLen = 8;
    static constexpr std::uint8_t MaxDataLen = 8;

    std::uint32_t id = 0;
    std::uint8_t data[MaxDataLen] = {};
    std::uint8_t dlc = 0;
    bool canfd = false;

    CANFrame() = default;

    CANFrame(std::uint32_t can_id, const std::uint8_t* can_data, std::uint8_t data_len,
             bool canfd_frame = false)
        : id(can_id), canfd(canfd_frame) {
        if (can_data == nullptr || data_len == 0 || data_len > MaxDataLen) {
            dlc = 0;
            return;
        }
        std::memcpy(data, can_data, data_len);
        dlc = data_len;
    }

    [[nodiscard]] bool isExtended() const { return (id & FlagEFF) != 0; }
    [[nodiscard]] bool isRemoteTransmissionRequest() const { return (id & FlagRTR) != 0; }
    [[nodiscard]] bool isErrorFrame() const { return (id & FlagERR) != 0; }

    [[nodiscard]] bool operator==(const CANFrame& rhs) const {
        return id == rhs.id && dlc == rhs.dlc && std::memcmp(data, rhs.data, dlc) == 0;
    }
    [[nodiscard]] bool operator!=(const CANFrame& rhs) const { return !(*this == rhs); }
};

class CanIface {
public:
    using CanIOFlags = std::uint16_t;
    static constexpr CanIOFlags Loopback = 1;
    static constexpr CanIOFlags AbortOnError = 2;
    static constexpr CanIOFlags IsForwardedFrame = 4;

    struct CanRxItem {
        std::uint64_t timestamp_us = 0;
        CanIOFlags flags = 0;
        CANFrame frame;
    };

    struct CanTxItem {
        std::uint64_t deadline = 0;
        CANFrame frame;
        CanIOFlags flags = 0;
    };

    CanIface() = default;

    // Caller-injected monotonic time for receive timestamps. Default 0
    // (no AP_HAL::micros64() in this port).
    void set_now_us(std::uint64_t now_us) { now_us_ = now_us; }
    [[nodiscard]] std::uint64_t now_us() const { return now_us_; }

    [[nodiscard]] bool init(std::uint32_t bitrate) {
        bitrate_ = bitrate;
        initialized_ = true;
        return true;
    }

    [[nodiscard]] bool init(std::uint32_t bitrate, std::uint32_t fdbitrate) {
        (void)fdbitrate;
        return init(bitrate);
    }

    [[nodiscard]] bool is_initialized() const { return initialized_; }
    [[nodiscard]] std::uint32_t bitrate() const { return bitrate_; }

    // Non-owning. Upstream always returns true.
    bool set_event_handle(BinarySemaphore* sem_handle) {
        event_handle_ = sem_handle;
        return true;
    }

    [[nodiscard]] BinarySemaphore* event_handle() const { return event_handle_; }

    [[nodiscard]] std::int16_t send(const CANFrame& frame, std::uint64_t tx_deadline,
                                    CanIOFlags flags) {
        if (!initialized_) {
            return -1;
        }
        if (tx_.full()) {
            return 0;
        }
        CanTxItem item;
        item.deadline = tx_deadline;
        item.frame = frame;
        item.flags = flags;
        tx_.push(item);

        if (!rx_.full()) {
            CanRxItem rx;
            rx.timestamp_us = now_us_;
            rx.flags = static_cast<CanIOFlags>(flags | Loopback);
            rx.frame = frame;
            rx_.push(rx);
        }
        if (event_handle_ != nullptr) {
            event_handle_->signal();
        }
        return 1;
    }

    [[nodiscard]] std::int16_t receive(CANFrame& out_frame, std::uint64_t& out_ts_monotonic,
                                       CanIOFlags& out_flags) {
        if (rx_.empty()) {
            return 0;
        }
        CanRxItem item;
        rx_.pop(item);
        out_frame = item.frame;
        out_ts_monotonic = item.timestamp_us;
        out_flags = item.flags;
        return 1;
    }

    [[nodiscard]] std::size_t tx_queued() const { return tx_.count(); }
    [[nodiscard]] std::size_t rx_queued() const { return rx_.count(); }

    void flush_tx() { tx_.clear(); }
    void clear_rx() { rx_.clear(); }

private:
    template <typename T, std::size_t Capacity>
    class FrameQueue {
    public:
        [[nodiscard]] bool empty() const { return count_ == 0; }
        [[nodiscard]] bool full() const { return count_ == Capacity; }
        [[nodiscard]] std::size_t count() const { return count_; }

        bool push(const T& item) {
            if (full()) {
                return false;
            }
            buf_[tail_] = item;
            tail_ = (tail_ + 1) % Capacity;
            ++count_;
            return true;
        }

        bool pop(T& out) {
            if (empty()) {
                return false;
            }
            out = buf_[head_];
            head_ = (head_ + 1) % Capacity;
            --count_;
            return true;
        }

        void clear() {
            head_ = 0;
            tail_ = 0;
            count_ = 0;
        }

    private:
        std::array<T, Capacity> buf_{};
        std::size_t head_ = 0;
        std::size_t tail_ = 0;
        std::size_t count_ = 0;
    };

    bool initialized_ = false;
    std::uint32_t bitrate_ = 0;
    std::uint64_t now_us_ = 0;
    BinarySemaphore* event_handle_ = nullptr;
    FrameQueue<CanTxItem, kCanQueueCapacity> tx_{};
    FrameQueue<CanRxItem, kCanQueueCapacity> rx_{};
};

}  // namespace fwcpp::hal
