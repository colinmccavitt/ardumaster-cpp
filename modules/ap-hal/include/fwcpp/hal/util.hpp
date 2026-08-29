#pragma once

// Port of AP_HAL::Util's SITL-subset (AP_HAL/Util.h + Util.cpp
// set_soft_armed ~73-82, AP_HAL_SITL/Util.h + Util.cpp). CPP-088 slice 3.
//
// Concrete caller-owned type: no virtual AP_HAL::Util base, no
// `extern const AP_HAL::HAL& hal`, no singleton `hal.util`.
//
// SITL defaults reproduced:
//   available_memory()           = 512 * 1024
//   safety_switch_state()        = RCOutput's safety (DISARMED at rest)
//   was_watchdog_reset()         = injected bool (not getenv)
//   get_system_id()              = caller-injected bytes (not host files)
//
// Host-file / getenv disclosures (deliberately not reproduced):
//   HALSITL::Util::get_system_id_unformatted opens /etc/machine-id then
//   /var/lib/dbus/machine-id, falls back to gethostname(), then
//   snprintf "sitl-unknown-%d". This port never opens those paths, never
//   calls gethostname, never reads /dev/urandom.
//   HALSITL::Util::was_watchdog_reset is
//   `getenv("SITL_WATCHDOG_RESET") != nullptr`. This port takes an
//   explicit bool from the caller.
//   HALSITL::Util::trap() raises SIGTRAP; get_hw_rtc uses clock_gettime /
//   gettimeofday. Neither is ported.
//
// SAFETY_NONE: AP_HAL::Util::safety_switch_state() defaults to
// SAFETY_NONE. SITL overrides that and returns
// RCOutput::_safety_switch_state(), which is SAFETY_DISARMED at
// construction and only ever toggled to SAFETY_ARMED / SAFETY_DISARMED
// by force_safety_off / force_safety_on. The HAL_USE_PWM==0 branch that
// would return SAFETY_NONE is compiled out on SITL (`#define HAL_USE_PWM 1`).
// RcOutput in this port has a two-state enum (kDisarmed=0, kArmed=1);
// Util has the three-state AP_HAL enum (kNone=0, kDisarmed=1, kArmed=2).
// Those numeric values must not be static_cast across. When an RcOutput
// is bound, NONE is unused — only the two RcOutput states are mapped.
// Tests can store kNone on an unbound Util.
//
// Not ported (this slice): snprintf/vsnprintf BufferPrinter, toneAlarm,
// trap()/SIGTRAP, get_hw_rtc, commandline_arguments, get_random_vals,
// uart_info, flash_bootloader, set_cmdline_parameters, stack overflow
// hooks, malloc_type.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <fwcpp/hal/rc_output.hpp>

namespace fwcpp::hal {

inline constexpr std::uint32_t kSitlAvailableMemoryBytes = 512U * 1024U;
inline constexpr std::uint8_t kSystemIdBufSize = 50;
inline constexpr std::uint8_t kSystemIdPrintableLen = 40;

class Util {
public:
    // Matches AP_HAL::Util::safety_state (NONE=0, DISARMED=1, ARMED=2).
    // Distinct from fwcpp::hal::SafetyState on RcOutput (DISARMED=0, ARMED=1).
    enum class SafetyState : std::uint8_t {
        kNone = 0,
        kDisarmed = 1,
        kArmed = 2,
    };

    // Fields from AP_HAL::Util::PersistentData that are POD / no heap.
    // STM32 limits this struct to 76 bytes; we keep the same members.
    struct PersistentData {
        float roll_rad = 0.0f;
        float pitch_rad = 0.0f;
        float yaw_rad = 0.0f;
        std::int32_t home_lat = 0;
        std::int32_t home_lon = 0;
        std::int32_t home_alt_cm = 0;
        std::uint32_t fault_addr = 0;
        std::uint32_t fault_icsr = 0;
        std::uint32_t fault_lr = 0;
        std::uint32_t internal_errors = 0;
        std::uint16_t internal_error_count = 0;
        std::uint16_t internal_error_last_line = 0;
        std::uint32_t spi_count = 0;
        std::uint32_t i2c_count = 0;
        std::uint32_t i2c_isr_count = 0;
        std::uint16_t waypoint_num = 0;
        std::uint16_t last_mavlink_msgid = 0;
        std::uint16_t last_mavlink_cmd = 0;
        std::uint16_t semaphore_line = 0;
        std::uint16_t fault_line = 0;
        std::uint8_t fault_type = 0;
        std::uint8_t fault_thd_prio = 0;
        char thread_name4[4] = {};
        std::int8_t scheduler_task = 0;
        bool armed = false;
        SafetyState safety_state = SafetyState::kNone;
        bool boot_to_dfu = false;
    };

    PersistentData persistent_data;
    // Upstream: last_persistent_data is only filled on a watchdog reset.
    // There is no backup-SRAM restore here; the caller sets the watchdog
    // bool, and we snapshot persistent_data into this field at that edge.
    PersistentData last_persistent_data;

    Util() = default;

    // Non-owning: SITL Util::safety_switch_state casts hal.rcout.
    explicit Util(RcOutput& rcout) : rc_output_(&rcout) {}

    void bind_rc_output(RcOutput& rcout) { rc_output_ = &rcout; }
    void unbind_rc_output() { rc_output_ = nullptr; }

    // Stored state used only when no RcOutput is bound. Default DISARMED
    // matches SITL RCOutput's construction default (not Util's SAFETY_NONE).
    void set_safety_switch_state(SafetyState state) { stored_safety_ = state; }

    // now_ms is injected: no AP_HAL::millis().
    // If b != soft_armed: set the flag, record now_ms, and write
    // persistent_data.armed only when !was_watchdog_reset() (Util.cpp:73-82).
    void set_soft_armed(bool b, std::uint32_t now_ms) {
        if (b != soft_armed_) {
            soft_armed_ = b;
            last_armed_change_ms_ = now_ms;
            if (!was_watchdog_reset()) {
                persistent_data.armed = b;
            }
        }
    }

    [[nodiscard]] bool get_soft_armed() const { return soft_armed_; }
    [[nodiscard]] std::uint32_t get_last_armed_change() const {
        return last_armed_change_ms_;
    }

    // Explicit bool. Upstream SITL: getenv("SITL_WATCHDOG_RESET") != nullptr.
    void set_was_watchdog_reset(bool reset) {
        if (reset && !was_watchdog_reset_) {
            last_persistent_data = persistent_data;
        }
        was_watchdog_reset_ = reset;
    }

    [[nodiscard]] bool was_watchdog_reset() const { return was_watchdog_reset_; }

    // Util.h one-liners.
    [[nodiscard]] bool was_watchdog_safety_off() const {
        return was_watchdog_reset() && persistent_data.safety_state == SafetyState::kArmed;
    }

    [[nodiscard]] bool was_watchdog_armed() const {
        return was_watchdog_reset() && persistent_data.armed;
    }

    // Bound RcOutput wins (SITL). Unbound: stored state, default DISARMED.
    [[nodiscard]] SafetyState safety_switch_state() const {
        if (rc_output_ != nullptr) {
            return from_rc_output(rc_output_->safety_state());
        }
        return stored_safety_;
    }

    [[nodiscard]] std::uint32_t available_memory() const {
        return kSitlAvailableMemoryBytes;
    }

    // Caller-injected identity. SITL adds get_instance() to buf[0].
    void set_instance(std::uint8_t instance) { instance_ = instance; }
    [[nodiscard]] std::uint8_t instance() const { return instance_; }

    void set_system_id(const std::uint8_t* data, std::uint8_t len) {
        if (data == nullptr) {
            system_id_len_ = 0;
            return;
        }
        const std::uint8_t n = std::min(len, static_cast<std::uint8_t>(system_id_.size()));
        std::memcpy(system_id_.data(), data, n);
        system_id_len_ = n;
    }

    // buf must be 50 bytes (AP_HAL::Util::get_system_id). SITL uses
    // unformatted with len=40; the result is already ASCII.
    [[nodiscard]] bool get_system_id(char buf[kSystemIdBufSize]) const {
        if (buf == nullptr) {
            return false;
        }
        std::uint8_t len = kSystemIdPrintableLen;
        return get_system_id_unformatted(reinterpret_cast<std::uint8_t*>(buf), len);
    }

    [[nodiscard]] bool get_system_id_unformatted(std::uint8_t* buf, std::uint8_t& len) const {
        if (buf == nullptr || len == 0 || system_id_len_ == 0) {
            return false;
        }
        const std::uint8_t n = system_id_len_ < len ? system_id_len_ : len;
        std::memcpy(buf, system_id_.data(), n);
        if (n == len) {
            buf[len - 1] = '\0';
        } else {
            buf[n] = '\0';
        }
        for (std::uint8_t i = 0; i < n; ++i) {
            if (buf[i] == '\n') {
                buf[i] = '\0';
                break;
            }
        }
        std::uint8_t slen = 0;
        while (slen < n && buf[slen] != '\0') {
            ++slen;
        }
        len = slen;
        buf[0] = static_cast<std::uint8_t>(buf[0] + instance_);
        return true;
    }

private:
    // Map RcOutput's two-state enum onto Util's three-state enum by name,
    // never by numeric value (DISARMED is 0 on RcOutput and 1 here).
    [[nodiscard]] static SafetyState from_rc_output(::fwcpp::hal::SafetyState s) {
        return s == ::fwcpp::hal::SafetyState::kArmed ? SafetyState::kArmed
                                                      : SafetyState::kDisarmed;
    }

    bool soft_armed_ = false;
    std::uint32_t last_armed_change_ms_ = 0;
    bool was_watchdog_reset_ = false;
    SafetyState stored_safety_ = SafetyState::kDisarmed;
    RcOutput* rc_output_ = nullptr;  // non-owning
    std::uint8_t instance_ = 0;
    std::array<std::uint8_t, kSystemIdBufSize> system_id_{};
    std::uint8_t system_id_len_ = 0;
};

}  // namespace fwcpp::hal
