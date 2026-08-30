#pragma once

// Port of libraries/SITL/SIM_SerialRangeFinder.h/.cpp plus SIM_RF_MaxsonarSerialLV packet.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace fwcpp::sim {

class SerialRangeFinder {
public:
    virtual ~SerialRangeFinder() = default;

    virtual void update(float range, std::uint32_t now_ms) {
        if (now_ms - last_sent_ms_ < reading_interval_ms()) {
            return;
        }
        last_sent_ms_ = now_ms;
        last_packet_.assign(255, 0);
        const std::uint32_t packetlen = packet_for_alt(range, last_packet_.data(),
                                                       static_cast<std::uint8_t>(last_packet_.size()));
        last_packet_.resize(packetlen);
        last_range_m_ = range;
    }

    virtual std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) = 0;
    [[nodiscard]] virtual std::uint16_t reading_interval_ms() const { return 200; }
    [[nodiscard]] virtual bool has_temperature() const { return false; }
    virtual std::uint32_t packet_for_temperature(float, std::uint8_t*, std::uint8_t) { return 0; }

    [[nodiscard]] const std::vector<std::uint8_t>& last_packet() const { return last_packet_; }
    [[nodiscard]] float last_range_m() const { return last_range_m_; }

private:
    std::uint32_t last_sent_ms_{0};
    std::vector<std::uint8_t> last_packet_{};
    float last_range_m_{0.0f};
};

class MaxsonarSerialLV : public SerialRangeFinder {
public:
    std::uint32_t packet_for_alt(float alt_m, std::uint8_t* buffer, std::uint8_t buflen) override {
        if (buflen < 8) {
            return 0;
        }
        const float inches = alt_m * 100 / 2.54f;
        const int n = std::snprintf(reinterpret_cast<char*>(buffer), buflen, "%u\r", static_cast<unsigned>(inches));
        return n > 0 ? static_cast<std::uint32_t>(n) : 0;
    }
};

}  // namespace fwcpp::sim
