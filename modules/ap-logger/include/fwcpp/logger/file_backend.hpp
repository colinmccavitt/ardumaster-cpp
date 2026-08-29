#pragma once

// Port of AP_Logger_File's POSIX write seam. CPP-090, slice 4.
//
// FileBackend is the fwrite / write path, not the SITL/Linux ByteBuffer
// ringbuffer + io_timer. Caller owns the FILE* or POSIX fd (opened
// before attach). No heap filename, no logNN.dat rotation, no directory
// scan. logging_started() matches File.h: `_write_fd != -1`.
//
// WriteBlock writes the whole span. A short write returns false (do not
// panic — Replay's AP_HAL::panic("Short write") is not reproduced). The
// local short-write counter is not MemoryBackend::num_dropped.
//
// WritesOK is false if not started. StartNewLogOK does not consult
// in_main_thread (no scheduler singleton); File.cpp rejects the main
// thread except Replay/UNKNOWN. CardInserted is true iff a FILE*/fd was
// provided (no SD detect).
//
// EraseAll (slice 5): armed-gate + rewind/ftruncate of the open stream.
// soft_armed is injected (no Util singleton). !_initialised is
// logging_started() — this backend has no separate Init() flag.
//
// Divergence from File.cpp stop_logging(): upstream closes _write_fd.
// We rewind+ftruncate the caller-owned FILE*/fd so WriteBlock still
// has a handle ("erase then can log again" without reopen). No
// directory walk, no erase.log_num / logNN.BIN rotation (remaining).
//
// Remaining vs File.cpp (later slices): _writebuf / io_timer chunked
// write+fsync, find_last_log, get_log_data, MAVLink LOG_REQUEST_*
// transfer, max-files rotation.

#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <span>
#include <unistd.h>

namespace fwcpp::logger {

class FileBackend {
public:
    FileBackend() = default;

    // Non-owning. Caller opens and closes the stream / fd.
    explicit FileBackend(std::FILE* file) : file_(file) {}
    explicit FileBackend(int fd) : fd_(fd) {}

    void attach(std::FILE* file) {
        file_ = file;
        fd_ = -1;
    }

    void attach(int fd) {
        fd_ = fd;
        file_ = nullptr;
    }

    [[nodiscard]] bool logging_started() const {
        return file_ != nullptr || fd_ >= 0;
    }

    [[nodiscard]] bool WritesOK() const { return logging_started(); }

    // File.cpp also rejects recent_open_error and (except Replay) the
    // main thread. No open-error clock and no scheduler singleton here.
    [[nodiscard]] bool StartNewLogOK() const { return WritesOK(); }

    // Upstream: _initialised && !recent_open_error(). No SD probe.
    [[nodiscard]] bool CardInserted() const { return logging_started(); }

    void StartWrite(std::uint32_t page_adr) {
        page_adr_ = page_adr;
        writing_ = true;
    }

    // fwrite / write the whole span. Short write -> false, count locally.
    [[nodiscard]] bool WriteBlock(std::span<const std::uint8_t> bytes) {
        const std::size_t n = bytes.size();
        std::size_t written = 0;
        if (file_ != nullptr) {
            written = std::fwrite(bytes.data(), 1, n, file_);
            // fflush so a buffered fwrite that later hits ENOSPC is a
            // short write (Replay uses unbuffered AP::FS().write).
            if (written == n && std::fflush(file_) != 0) {
                count_short_write();
                return false;
            }
        } else if (fd_ >= 0) {
            const ssize_t r = ::write(fd_, bytes.data(), n);
            written = r < 0 ? 0 : static_cast<std::size_t>(r);
        } else {
            count_short_write();
            return false;
        }
        if (written != n) {
            count_short_write();
            return false;
        }
        return true;
    }

    void EndWrite() {
        writing_ = false;
        if (ended_ < std::numeric_limits<std::uint32_t>::max()) {
            ++ended_;
        }
    }

    // AP_Logger_File::EraseAll (File.cpp ~401-415). Armed or
    // !logging_started: no-op, file untouched. Else record
    // was_logging and truncate the open stream to 0.
    void EraseAll(bool soft_armed) {
        if (soft_armed) {
            return;
        }
        if (!logging_started()) {
            return;
        }
        was_logging_ = true;
        if (file_ != nullptr) {
            std::fflush(file_);
            const int raw = ::fileno(file_);
            if (raw >= 0) {
                ::ftruncate(raw, 0);
            }
            std::rewind(file_);
            std::clearerr(file_);
        } else if (fd_ >= 0) {
            ::lseek(fd_, 0, SEEK_SET);
            ::ftruncate(fd_, 0);
        }
    }

    [[nodiscard]] bool was_logging() const { return was_logging_; }

    [[nodiscard]] std::uint32_t page_adr() const { return page_adr_; }
    [[nodiscard]] bool is_writing() const { return writing_; }
    [[nodiscard]] std::uint32_t ended_writes() const { return ended_; }
    [[nodiscard]] std::uint32_t num_short_writes() const { return short_writes_; }

private:
    void count_short_write() {
        if (short_writes_ < std::numeric_limits<std::uint32_t>::max()) {
            ++short_writes_;
        }
    }

    std::FILE* file_{nullptr};
    int fd_{-1};
    std::uint32_t page_adr_{0};
    bool writing_{false};
    std::uint32_t ended_{0};
    std::uint32_t short_writes_{0};
    bool was_logging_{false};
};

}  // namespace fwcpp::logger
