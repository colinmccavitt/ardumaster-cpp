#pragma once

// Port of AP_Logger_RateLimiter::should_log_streaming. CPP-090, slice 3.
//
// Upstream: AP_Logger_Backend.cpp ~738-751. now is millis16 (uint16);
// callers inject now_ms16 — no HAL clock, no AP::scheduler().ticks().
// last_send_ms is a 256-entry table keyed by msgid (uint8).
//
// Remaining inside should_log() (not this slice):
//   - AP::scheduler().ticks() same-tick cache (last_sched_count / last_return)
//   - get_soft_armed / disarm_rate_limit_hz / in_log_persistance
//   - not_streaming bitmask + structure_for_msg_type streaming flag
// WriteV / msg_fmt_for_name / va_list WriteStreaming are also unported.

#include <array>
#include <cfloat>
#include <cstdint>

namespace fwcpp::logger {

class RateLimiter {
public:
    RateLimiter() = default;

    // AP_Logger_RateLimiter::should_log_streaming(msgid, rate_hz).
    // log_pause is injected (upstream front._log_pause). now_ms16 is
    // injected (upstream AP_HAL::millis16()).
    [[nodiscard]] bool should_log_streaming(std::uint8_t msgid,
                                            std::uint16_t now_ms16,
                                            float rate_hz,
                                            bool log_pause) {
        if (log_pause) {
            return false;
        }
        const std::uint16_t now = now_ms16;
        std::uint16_t delta_ms = now - last_send_ms_[msgid];
        // is_positive(rate_hz) is AP_Math: rate_hz >= FLT_EPSILON.
        // 1000.0f matches upstream `1000.0 / rate_hz` under
        // -fsingle-precision-constant (header-only tests lack that flag).
        if ((rate_hz >= FLT_EPSILON) && (delta_ms < 1000.0f / rate_hz)) {
            return false;
        }
        last_send_ms_[msgid] = now;
        return true;
    }

    [[nodiscard]] std::uint16_t last_send_ms(std::uint8_t msgid) const {
        return last_send_ms_[msgid];
    }

private:
    std::array<std::uint16_t, 256> last_send_ms_{};
};

}  // namespace fwcpp::logger
