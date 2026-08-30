#pragma once

// Port of libraries/SITL/SIM_AIS.h/.cpp. AP_Param/GCS dropped; vessel_count
// and radius_m are plain fields. NMEA AIVDM packing is original-source.
// AIS_Replay reads a caller-supplied path (default SIM_AIS_data.txt).

#include <algorithm>
#include <cstdarg>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_adsb.hpp>
#include <fwcpp/sim/sim_gps_nmea.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

#ifndef KNOTS_TO_M_PER_SEC
#define KNOTS_TO_M_PER_SEC (1 / 1.94384449f)
#endif

namespace fwcpp::sim {

enum AisFlags : std::uint16_t {
    AIS_FLAGS_POSITION_ACCURACY = 1,
    AIS_FLAGS_VALID_COG = 2,
    AIS_FLAGS_VALID_VELOCITY = 4,
    AIS_FLAGS_HIGH_VELOCITY = 8,
    AIS_FLAGS_VALID_TURN_RATE = 16,
    AIS_FLAGS_TURN_RATE_SIGN_ONLY = 32,
    AIS_FLAGS_VALID_DIMENSIONS = 64,
    AIS_FLAGS_LARGE_BOW_DIMENSION = 128,
    AIS_FLAGS_LARGE_STERN_DIMENSION = 256,
    AIS_FLAGS_LARGE_PORT_DIMENSION = 512,
    AIS_FLAGS_LARGE_STARBOARD_DIMENSION = 1024,
};

struct AisVesselInfo {
    std::uint32_t MMSI = 0;
    std::uint16_t flags = 0;
    std::int32_t lat = 0;
    std::int32_t lon = 0;
    std::uint16_t velocity = 0;
    std::int16_t turn_rate = 0;
    std::uint16_t heading = 0;
    std::uint16_t COG = 0;
    std::uint8_t navigational_status = 0;
    std::uint8_t type = 0;
    std::uint16_t dimension_bow = 0;
    std::uint16_t dimension_stern = 0;
    std::uint8_t dimension_port = 0;
    std::uint8_t dimension_starboard = 0;
};

class AIS_Replay : public SerialDevice {
public:
    explicit AIS_Replay(const char* path = nullptr) {
        if (path != nullptr) {
            file_ = std::fopen(path, "r");
        }
        if (file_ != nullptr) {
            char line[100];
            IGNORE_RETURN_FGETS(line, sizeof(line), file_);
        }
    }
    ~AIS_Replay() {
        if (file_ != nullptr) {
            std::fclose(file_);
        }
    }

    void update(std::uint32_t now_ms) {
        if (file_ == nullptr) {
            return;
        }
        if (now_ms - last_sent_ms_ < 1000) {
            return;
        }
        last_sent_ms_ = now_ms;
        char line[100];
        if (!std::fgets(line, sizeof(line), file_)) {
            std::fseek(file_, 0, SEEK_SET);
            if (!std::fgets(line, sizeof(line), file_)) {
                return;
            }
            return;
        }
        write_to_autopilot(line, std::strlen(line));
    }

    [[nodiscard]] bool has_file() const { return file_ != nullptr; }

private:
    static void IGNORE_RETURN_FGETS(char* line, int n, FILE* f) { (void)std::fgets(line, n, f); }
    FILE* file_ = nullptr;
    std::uint32_t last_sent_ms_ = 0;
};

class AIS : public SerialDevice {
public:
    std::int8_t vessel_count = -1;
    float radius_m = 10000;

    void update(const Location& aircraft_loc, std::uint32_t now_ms) {
        if (vessel_count <= 0) {
            return;
        }
        const float dt = (now_ms - last_sim_update_ms_) * 0.001f;
        last_sim_update_ms_ = now_ms;

        for (std::uint8_t i = 0; i < 50; i++) {
            update_simulated_vessel(vessels_[i], dt, aircraft_loc, radius_m, now_ms);
        }
        for (std::uint8_t i = 0; i < 50; i++) {
            if (i >= vessel_count) {
                vessels_[i].active = false;
                continue;
            }
            if (vessels_[i].active) {
                continue;
            }
            init_vessel(vessels_[i], aircraft_loc, radius_m);
        }
    }

    [[nodiscard]] std::string last_nmea() const { return last_nmea_; }

    void send_position_report(const AisVesselInfo& info) {
        int8_t rot = -128;
        if ((info.flags & AIS_FLAGS_VALID_TURN_RATE) != 0) {
            if ((info.flags & AIS_FLAGS_TURN_RATE_SIGN_ONLY) != 0) {
                rot = info.turn_rate >= 0 ? 127 : -127;
            } else {
                const float turn_rate_deg_per_min = std::abs(info.turn_rate) * 60.0f * 0.01f;
                rot = 4.733 * std::sqrt(turn_rate_deg_per_min) * info.turn_rate >= 0 ? 1.0 : -1.0;
            }
        }

        std::uint16_t cog = 3600;
        if ((info.flags & AIS_FLAGS_VALID_COG) != 0) {
            cog = static_cast<std::uint16_t>(info.COG * 0.1);
        }
        const bool accuracy = (info.flags & AIS_FLAGS_POSITION_ACCURACY) != 0;
        std::uint16_t sog = 1023;
        if ((info.flags & AIS_FLAGS_VALID_VELOCITY) != 0) {
            if ((info.flags & AIS_FLAGS_HIGH_VELOCITY) != 0) {
                sog = 1022;
            } else {
                sog = static_cast<std::uint16_t>(
                    std::min((float(info.velocity) * 0.01f * (1.0f / KNOTS_TO_M_PER_SEC) * 10), 1022.0f));
            }
        }

        std::uint8_t payload[28]{};
        set_bits(payload, 0, 5, 1);
        set_bits(payload, 6, 7, 3);
        set_bits(payload, 8, 37, info.MMSI);
        set_bits(payload, 38, 41, info.navigational_status);
        set_bits_signed(payload, 42, 49, rot);
        set_bits(payload, 50, 59, sog);
        set_bits(payload, 60, 60, accuracy);
        set_bits_signed(payload, 61, 88, static_cast<std::int32_t>(double(info.lon) * 0.06));
        set_bits_signed(payload, 89, 115, static_cast<std::int32_t>(double(info.lat) * 0.06));
        set_bits(payload, 116, 127, cog);
        set_bits(payload, 128, 136, static_cast<std::uint32_t>(info.heading * 0.01));
        set_bits(payload, 137, 142, 60);
        set_bits(payload, 143, 144, 0);
        set_bits(payload, 148, 148, 0);
        set_bits(payload, 149, 167, 0);

        std::uint8_t encoded[sizeof(payload) + 1]{};
        for (std::uint8_t i = 0; i < sizeof(payload); i++) {
            encoded[i] = encode_char(payload[i]);
        }
        const std::uint8_t total_fragments = 1;
        const std::uint8_t fragment = 1;
        const char channel_code = char(0x41);
        const std::uint8_t fill_bits = 0;
        nmea_printf("!AIVDM,%u,%u,,%c,%s,%u", total_fragments, fragment, channel_code, encoded, fill_bits);
    }

    void send_static_and_voyage(const AisVesselInfo& info) {
        std::uint16_t bow_dim = 0;
        std::uint16_t stern_dim = 0;
        std::uint8_t port_dim = 0;
        std::uint8_t star_dim = 0;
        if ((info.flags & AIS_FLAGS_VALID_DIMENSIONS) != 0) {
            bow_dim = (info.flags & AIS_FLAGS_LARGE_BOW_DIMENSION) ? 511 : std::min(info.dimension_bow, static_cast<std::uint16_t>(511));
            stern_dim = (info.flags & AIS_FLAGS_LARGE_STERN_DIMENSION) ? 511 : std::min(info.dimension_stern, static_cast<std::uint16_t>(511));
            port_dim = (info.flags & AIS_FLAGS_LARGE_PORT_DIMENSION) ? 63 : std::min(info.dimension_port, static_cast<std::uint8_t>(63));
            star_dim = (info.flags & AIS_FLAGS_LARGE_STARBOARD_DIMENSION) ? 63
                                                                        : std::min(info.dimension_starboard, static_cast<std::uint8_t>(63));
        }
        std::uint8_t payload[71]{};
        set_bits(payload, 0, 5, 5);
        set_bits(payload, 6, 7, 3);
        set_bits(payload, 8, 37, info.MMSI);
        set_bits(payload, 38, 39, 1);
        set_bits(payload, 40, 69, 0);
        set_bits(payload, 70, 111, static_cast<std::uint32_t>(0x40));
        set_bits(payload, 112, 231, static_cast<std::uint32_t>(0x40));
        set_bits(payload, 232, 239, info.type);
        set_bits(payload, 240, 248, bow_dim);
        set_bits(payload, 249, 257, stern_dim);
        set_bits(payload, 258, 263, port_dim);
        set_bits(payload, 264, 269, star_dim);
        set_bits(payload, 270, 273, 1);
        set_bits(payload, 274, 277, 0);
        set_bits(payload, 278, 282, 0);
        set_bits(payload, 283, 287, 0);
        set_bits(payload, 288, 293, 0);
        set_bits(payload, 294, 301, 0);
        set_bits(payload, 302, 421, static_cast<std::uint32_t>(0x40));
        set_bits(payload, 422, 422, 1);

        const std::uint8_t msg1_len = 50;
        const std::uint8_t msg2_len = static_cast<std::uint8_t>(sizeof(payload) - msg1_len);
        std::uint8_t msg1_encoded[51]{};
        for (std::uint8_t i = 0; i < msg1_len; i++) {
            msg1_encoded[i] = encode_char(payload[i]);
        }
        std::uint8_t msg2_encoded[32]{};
        for (std::uint8_t i = 0; i < msg2_len; i++) {
            msg2_encoded[i] = encode_char(payload[msg1_len + i]);
        }
        const char channel_code = char(0x41);
        nmea_printf("!AIVDM,%u,%u,%u,%c,%s,%u", 2, 1, sequence_ID, channel_code, msg1_encoded, 0);
        nmea_printf("!AIVDM,%u,%u,%u,%c,%s,%u", 2, 2, sequence_ID, channel_code, msg2_encoded, 0);
        sequence_ID++;
    }

    void set_bits(std::uint8_t* payload, const std::uint16_t low, const std::uint16_t high, std::uint32_t value) {
        const std::uint8_t bit_len = static_cast<std::uint8_t>(high - low + 1);
        const std::uint32_t value_mask = 0xFFFFFFFFU >> (32 - bit_len);
        if ((value & ~value_mask) != 0) {
            return;
        }
        const std::uint8_t char_low = static_cast<std::uint8_t>(low / 6);
        const std::uint8_t bit_low = static_cast<std::uint8_t>(low % 6);
        const std::uint8_t char_high = static_cast<std::uint8_t>(high / 6);
        const std::uint8_t bit_high = static_cast<std::uint8_t>((high % 6) + 1);
        const std::uint8_t char_range = static_cast<std::uint8_t>(char_high - char_low);
        std::uint8_t bits = bit_len;
        for (std::uint8_t index = 0; index <= char_range; index++) {
            if (bits == 0) {
                return;
            }
            if (index == char_range) {
                const std::uint8_t bits_needed = std::min(bit_high, bit_len);
                bits = static_cast<std::uint8_t>(bits - bits_needed);
                payload[char_low + index] |= static_cast<std::uint8_t>(get_bits(value, bits, bits_needed) << (6 - bit_high));
            } else if (index == 0) {
                const std::uint8_t bits_needed = std::min(static_cast<std::uint8_t>(6 - bit_low), bit_len);
                bits = static_cast<std::uint8_t>(bits - bits_needed);
                payload[char_low + index] |= get_bits(value, bits, bits_needed);
            } else {
                bits = static_cast<std::uint8_t>(bits - 6);
                payload[char_low + index] |= get_bits(value, bits, 6);
            }
        }
    }

    void set_bits_signed(std::uint8_t* payload, const std::uint16_t low, const std::uint16_t high, std::int32_t value) {
        if (value >= 0) {
            set_bits(payload, low, high, static_cast<std::uint32_t>(value));
            return;
        }
        std::uint32_t val = static_cast<std::uint32_t>(value);
        const std::uint8_t bit_len = static_cast<std::uint8_t>(high - low + 1);
        val &= 0xFFFFFFFFU >> (32 - bit_len);
        val |= 1U << (bit_len - 1);
        set_bits(payload, low, high, val);
    }

    [[nodiscard]] std::uint8_t encode_char(std::uint8_t payload) const {
        if (payload + 8 > 40) {
            return static_cast<std::uint8_t>(payload + 8 + 48);
        }
        return static_cast<std::uint8_t>(payload + 48);
    }

    [[nodiscard]] std::uint8_t get_bits(std::uint32_t value, const std::uint8_t start, const std::uint8_t len) const {
        value = value >> start;
        const std::uint8_t mask = static_cast<std::uint8_t>(0b111111 >> (6 - len));
        return static_cast<std::uint8_t>(value & mask);
    }

private:
    struct ais_vessel {
        AisVesselInfo info{};
        std::uint32_t last_position_report_ms = 0;
        std::uint32_t last_static_and_voyage_ms = 0;
        bool active = false;
    };
    ais_vessel vessels_[50]{};
    std::uint32_t last_sim_update_ms_ = 0;
    std::uint8_t sequence_ID = 0;
    std::string last_nmea_{};

    void nmea_printf(const char* fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        const std::string s = nmea_vaprintf(fmt, ap);
        va_end(ap);
        if (!s.empty()) {
            write_to_autopilot(s.c_str(), s.size());
            last_nmea_ = s;
        }
    }

    void update_simulated_vessel(ais_vessel& vessel, const float dt, const Location& vehicle_loc, const float radius,
                                 const std::uint32_t now_ms) {
        if (!vessel.active) {
            return;
        }
        vessel.info.heading += static_cast<std::uint16_t>(vessel.info.turn_rate * dt);
        Location loc{vessel.info.lat, vessel.info.lon, 0, Location::AltFrame::ABOVE_ORIGIN};
        loc.offset_bearing(vessel.info.heading * 0.01f, vessel.info.velocity * 0.01f * dt);
        vessel.info.lat = loc.lat;
        vessel.info.lon = loc.lng;
        if (vehicle_loc.get_distance(loc) > radius) {
            vessel.active = false;
            return;
        }
        std::uint32_t position_interval;
        if (vessel.info.velocity > 23.0 * 0.1 * KNOTS_TO_M_PER_SEC * 100.0) {
            position_interval = 2000;
        } else if (vessel.info.velocity > 14.0 * 0.1 * KNOTS_TO_M_PER_SEC * 100.0) {
            position_interval = 6000;
        } else if (vessel.info.velocity > 0.0) {
            position_interval = 10000;
        } else {
            position_interval = 3 * 60 * 1000;
        }
        if ((vessel.last_position_report_ms == 0) || (now_ms - vessel.last_position_report_ms > position_interval)) {
            send_position_report(vessel.info);
            vessel.last_position_report_ms = now_ms;
        }
        const std::uint32_t static_and_voyage_interval = 6 * 60 * 1000;
        if ((vessel.last_static_and_voyage_ms == 0) || (now_ms - vessel.last_static_and_voyage_ms > static_and_voyage_interval)) {
            send_static_and_voyage(vessel.info);
            vessel.last_static_and_voyage_ms = now_ms;
        }
    }

    void init_vessel(ais_vessel& vessel, const Location& vehicle_loc, const float radius) {
        vessel = {};
        vessel.info.flags |= AIS_FLAGS_VALID_VELOCITY | AIS_FLAGS_VALID_TURN_RATE;
        vessel.info.MMSI = static_cast<std::uint32_t>(std::rand()) & 0x3FFFFFFF;
        if (std::rand() > RAND_MAX * 0.75) {
            vessel.info.turn_rate = static_cast<std::int16_t>(aircraft_rand_normal(0, 100.0));
        }
        if (std::rand() > RAND_MAX * 0.05) {
            vessel.info.velocity = static_cast<std::uint16_t>(std::abs(aircraft_rand_normal(0, 500.0)));
            const std::uint16_t high_velocity = static_cast<std::uint16_t>(1022 * 0.1 * KNOTS_TO_M_PER_SEC * 100.0);
            if (vessel.info.velocity >= high_velocity) {
                vessel.info.velocity = high_velocity;
                vessel.info.flags |= AIS_FLAGS_HIGH_VELOCITY;
            }
        }
        vessel.info.heading = static_cast<std::uint16_t>(math::wrap_360_cd(static_cast<int>(std::rand())));
        Location loc = vehicle_loc;
        loc.offset(static_cast<float>(aircraft_rand_normal(0, radius)), static_cast<float>(aircraft_rand_normal(0, radius)));
        vessel.info.lat = loc.lat;
        vessel.info.lon = loc.lng;
        vessel.active = true;
    }
};

}  // namespace fwcpp::sim
