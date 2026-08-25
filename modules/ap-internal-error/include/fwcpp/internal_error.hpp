#pragma once

// Port of AP_InternalError. CPP-005.
//
// Upstream is a singleton reached via AP::internalerror(), accumulating a
// bitmask of "should never happen" error categories plus a total count and
// the line of the most recent one. ADR-0012 decision 6 forbids reproducing
// the singleton; this is the same data held in an explicit, caller-owned
// object instead, passed by pointer to whatever needs to report into it
// (nullable - a null pointer is the same as a build with
// AP_INTERNALERROR_ENABLED off upstream: reporting becomes a no-op, and the
// operation that would have reported still completes its own contract,
// exactly as scalar.hpp's constrain_value and quaternion.hpp's normalize
// already do).
//
// error_t's flag list is reproduced from upstream in full, not trimmed to
// what this port currently uses - it is a stable identifier space shared
// with any future log/telemetry consumer, and most of the bits (logger_*,
// iomcu_*, spi_fail, ...) name subsystems this port has not built yet.
// Reporting an error this port has no path to trigger is harmless; the
// enum values still need to match if this port and upstream are ever
// compared error-code-for-error-code.

#include <cstdint>

namespace fwcpp {

enum class InternalErrorCode : std::uint32_t {
    logger_mapfailure            = 1U << 0,
    logger_missing_logstructure  = 1U << 1,
    logger_logwrite_missingfmt   = 1U << 2,
    logger_too_many_deletions    = 1U << 3,
    logger_bad_getfilename       = 1U << 4,
    panic                        = 1U << 5,
    logger_flushing_without_sem  = 1U << 6,
    logger_bad_current_block     = 1U << 7,
    logger_blockcount_mismatch   = 1U << 8,
    logger_dequeue_failure       = 1U << 9,
    constraining_nan             = 1U << 10,
    watchdog_reset               = 1U << 11,
    iomcu_reset                  = 1U << 12,
    iomcu_fail                   = 1U << 13,
    spi_fail                     = 1U << 14,
    main_loop_stuck              = 1U << 15,
    gcs_bad_missionprotocol_link = 1U << 16,
    bitmask_range                = 1U << 17,
    gcs_offset                   = 1U << 18,
    i2c_isr                      = 1U << 19,
    flow_of_control              = 1U << 20, // generic we-should-never-get-here
    switch_full_sector_recursion = 1U << 21,
    bad_rotation                 = 1U << 22,
    stack_overflow               = 1U << 23,
    imu_reset                    = 1U << 24,
    gpio_isr                     = 1U << 25,
    mem_guard                    = 1U << 26,
    dma_fail                     = 1U << 27,
    params_restored              = 1U << 28,
    invalid_arg_or_result        = 1U << 29,
};

// Explicit, non-singleton replacement for AP_InternalError. One instance
// per vehicle context (ADR-0012 decision 6's context-struct pattern);
// nothing in this port reaches for a global one.
class InternalError {
public:
    // Record one occurrence. Matches upstream's AP_InternalError::error():
    // ORs the bit into the running mask (a code that already fired stays
    // set - this is presence tracking, not a queue), increments the total
    // count unconditionally (so a rapid-fire single error still shows a
    // large count even though the mask has just one bit), and remembers
    // the most recent call site.
    void record(InternalErrorCode code, std::uint16_t line) {
        errors_ |= static_cast<std::uint32_t>(code);
        ++total_count_;
        last_line_ = line;
    }

    [[nodiscard]] std::uint32_t errors() const { return errors_; }
    [[nodiscard]] std::uint32_t count() const { return total_count_; }
    [[nodiscard]] std::uint16_t last_error_line() const { return last_line_; }

    [[nodiscard]] bool has_error(InternalErrorCode code) const {
        return (errors_ & static_cast<std::uint32_t>(code)) != 0;
    }

private:
    std::uint32_t errors_ = 0;
    std::uint32_t total_count_ = 0;
    std::uint16_t last_line_ = 0;
};

} // namespace fwcpp
