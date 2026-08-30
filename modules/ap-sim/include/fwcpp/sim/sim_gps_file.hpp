#pragma once

// Port of libraries/SITL/SIM_GPS_FILE.h/.cpp. Replays gpsN_NNN.log files
// written by AP_GPS_DEBUG_LOGGING_ENABLED (magic 0x7fe53b04).

#include <cstdint>
#include <cstdio>
#include <fcntl.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <fwcpp/sim/sim_gps.hpp>

namespace fwcpp::sim {

class GPS_FILE : public GPS_Backend {
public:
    using GPS_Backend::GPS_Backend;
    GPS_FILE(const GPS_FILE&) = delete;
    GPS_FILE& operator=(const GPS_FILE&) = delete;
    ~GPS_FILE() override {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }
    void publish(const GPS_Data* d) override;

private:
    int fd_{-1};
    std::uint32_t base_time_{0};
};

inline void GPS_FILE::publish(const GPS_Data*) {
    const std::uint16_t lognum = front.world().gps_log_num;
    if (instance > 1) {
        return;
    }
    if (fd_ == -1) {
        char fname[32];
        std::snprintf(fname, sizeof(fname), "gps%u_%03u.log", instance + 1, lognum);
        fd_ = ::open(fname, O_RDONLY | O_CLOEXEC);
        if (fd_ == -1) {
            return;
        }
    }
    const std::uint32_t magic = 0x7fe53b04;
    struct {
        std::uint32_t magic;
        std::uint32_t time_ms;
        std::uint32_t n;
    } header {};
    while (true) {
        if (::read(fd_, &header, sizeof(header)) != sizeof(header) || header.magic != magic) {
            break;
        }
        if (header.time_ms + base_time_ > front.now_ms()) {
            ::lseek(fd_, -static_cast<off_t>(sizeof(header)), SEEK_CUR);
            return;
        }
        std::vector<std::uint8_t> buf(header.n);
        if (::read(fd_, buf.data(), header.n) == static_cast<ssize_t>(header.n)) {
            write_to_autopilot(reinterpret_cast<const char*>(buf.data()), header.n);
            continue;
        }
        break;
    }
    base_time_ = front.now_ms();
    ::lseek(fd_, 0, SEEK_SET);
}

}  // namespace fwcpp::sim
