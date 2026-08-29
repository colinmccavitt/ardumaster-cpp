#pragma once

// Port of AP_Logger File backend max-files rotation. CPP-090, slice 6.
//
// Upstream: AP_Logger.cpp MIN_LOG_FILES=2 / MAX_LOG_FILES=500 and
// get_max_num_logs (~859) constrain_uint16 then AP_Param save.
// AP_Logger_File.cpp find_last_log (~493-513), start_new_log wrap
// (~798-805), EraseAll erase.log_num=1 (~414), io_timer erase_next
// while erase.log_num!=0 (~920-923), erase_next (~1090-1114).
//
// This is an in-memory slot table, not a POSIX directory of logNN.BIN
// files. No heap, no opendir, no AP::FS filenames, no disk_space_avail
// min-space deletion loop (File.cpp ~330). LASTLOG is a stored number
// plus optional discard 'D' flag (strtol + *endptr=='D'), not a file.
//
// next_log_number is the wrap arithmetic only; it does not open a
// logNN.BIN or write LASTLOG (start_new_log does both after this).
// erase_next clears a slot (unlink equivalent) then increments; when
// erase_log_num > max_logs it clears LASTLOG and sets erase_log_num=0.
//
// Divergence vs File.cpp: no stop_logging close of _write_fd (that
// lives on FileBackend, which keeps the caller-owned handle open).
// No periodic io_timer unlink of a real logNN.BIN path — slots are
// cleared in this table. _cached_oldest_log is not stored.

#include <array>
#include <cstdint>
#include <cstdlib>

namespace fwcpp::logger {

inline constexpr std::uint16_t kMinLogFiles = 2;
inline constexpr std::uint16_t kMaxLogFiles = 500;

// AP_Logger::get_max_num_logs constrain_uint16(requested, 2, 500).
// Does not persist AP_Param (no set_and_save_ifchanged).
[[nodiscard]] constexpr std::uint16_t get_max_num_logs(std::uint16_t requested) {
    if (requested < kMinLogFiles) {
        return kMinLogFiles;
    }
    if (requested > kMaxLogFiles) {
        return kMaxLogFiles;
    }
    return requested;
}

class LogDirectory {
public:
    explicit LogDirectory(std::uint16_t requested_max = kMaxLogFiles)
        : max_logs_(get_max_num_logs(requested_max)) {}

    void set_max_num_logs(std::uint16_t requested) {
        max_logs_ = get_max_num_logs(requested);
    }

    [[nodiscard]] std::uint16_t max_num_logs() const { return max_logs_; }

    // LASTLOG stored number. 0 if none / empty (File.cpp: FileData
    // missing -> ret=0). Optional discard flag is *endptr=='D'.
    [[nodiscard]] std::uint16_t find_last_log() const { return lastlog_; }

    [[nodiscard]] bool last_log_is_marked_discard() const { return lastlog_discard_; }

    // write_lastlog_file equivalent: store the number and optional 'D'.
    void write_lastlog(std::uint16_t log_num, bool discard = false) {
        lastlog_ = log_num;
        lastlog_discard_ = discard;
    }

    // find_last_log parse of LASTLOG.TXT bytes (strtol, then 'D').
    // Empty / null is 0 and not discarded.
    void load_lastlog_text(const char* data) {
        lastlog_ = 0;
        lastlog_discard_ = false;
        if (data == nullptr || data[0] == '\0') {
            return;
        }
        char* endptr = nullptr;
        const long parsed = std::strtol(data, &endptr, 10);
        lastlog_ = parsed < 0 ? 0 : static_cast<std::uint16_t>(parsed);
        if (endptr != nullptr) {
            lastlog_discard_ = (*endptr == 'D');
        }
    }

    // _get_log_size: slot size, 0 if missing / out of range / log 0.
    [[nodiscard]] std::uint32_t log_size(std::uint16_t log_num) const {
        if (log_num == 0 || log_num > kMaxLogFiles) {
            return 0;
        }
        return slot_size_[log_num];
    }

    // Test / writer seam: occupy slot n (no POSIX create).
    void occupy(std::uint16_t log_num, std::uint32_t size) {
        if (log_num == 0 || log_num > kMaxLogFiles) {
            return;
        }
        slot_size_[log_num] = size;
    }

    // start_new_log wrap, File.cpp ~798-805.
    [[nodiscard]] std::uint16_t next_log_number() const {
        std::uint16_t log_num = find_last_log();
        if (log_size(log_num) > 0 || log_num == 0) {
            ++log_num;
        }
        if (log_num > max_logs_) {
            log_num = 1;
        }
        return log_num;
    }

    // File.cpp EraseAll ~414: erase.log_num = 1. Armed-gate lives on
    // FileBackend. No stop_logging close here.
    void EraseAll() { erase_log_num_ = 1; }

    // File.cpp erase_next ~1090-1114. Clears slot erase_log_num
    // (unlink equivalent), increments, and when past max_logs clears
    // LASTLOG and sets erase_log_num=0. io_timer calls this while
    // erase.log_num != 0. No-op if erase is idle.
    void erase_next() {
        if (erase_log_num_ == 0) {
            return;
        }
        if (erase_log_num_ <= kMaxLogFiles) {
            slot_size_[erase_log_num_] = 0;
        }
        ++erase_log_num_;
        if (erase_log_num_ <= max_logs_) {
            return;
        }
        lastlog_ = 0;
        lastlog_discard_ = false;
        erase_log_num_ = 0;
    }

    [[nodiscard]] std::uint16_t erase_log_num() const { return erase_log_num_; }

private:
    // 1-indexed like logNN; slot 0 unused. Capacity is MAX_LOG_FILES.
    std::array<std::uint32_t, kMaxLogFiles + 1> slot_size_{};
    std::uint16_t max_logs_;
    std::uint16_t lastlog_{0};
    bool lastlog_discard_{false};
    std::uint16_t erase_log_num_{0};
};

}  // namespace fwcpp::logger
