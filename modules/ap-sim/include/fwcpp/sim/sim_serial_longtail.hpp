#pragma once

// Original-source SIM serial parsers live in dedicated headers (VectorNav,
// MicroStrain, InertialLabs, SensAItion, gimbals, FETtec/IE/Richen/Loweheiser,
// LD06/RPLidar/SF45B). This file keeps Frsky_D / CRSF / ELRS / Volz from the
// existing ArduPilot SITL ports and re-exports the rest.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_gimbals.hpp>
#include <fwcpp/sim/sim_inertiallabs.hpp>
#include <fwcpp/sim/sim_microstrain.hpp>
#include <fwcpp/sim/sim_power_serial.hpp>
#include <fwcpp/sim/sim_proximity.hpp>
#include <fwcpp/sim/sim_sensation.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>
#include <fwcpp/sim/sim_vectornav.hpp>

namespace fwcpp::sim {

class Frsky_D : public SerialDevice {
public:
    std::uint32_t last_update_ms = 0;
    void update(std::uint32_t now, float alt_m, float voltage) {
        if (now - last_update_ms < 200) {
            return;
        }
        last_update_ms = now;
        auto send = [&](std::uint8_t id, std::uint16_t v) {
            std::uint8_t pkt[4]{0x5E, id, static_cast<std::uint8_t>(v & 0xff), static_cast<std::uint8_t>(v >> 8)};
            write_to_autopilot(reinterpret_cast<const char*>(pkt), 4);
        };
        send(0x10, static_cast<std::uint16_t>(alt_m * 100));
        send(0x3A, static_cast<std::uint16_t>(voltage * 100));
    }
};

class CRSF : public SerialDevice {
public:
    std::uint32_t last_update_ms = 0;
    int id = 0;
    void update(std::uint32_t now) {
        char tmp[64];
        read_from_autopilot(tmp, sizeof(tmp));
        if (now - last_update_ms < 400) {
            return;
        }
        last_update_ms = now;
        static const std::uint8_t vtx_frame[] = {0xC8, 0x8, 0xF, 0xCE, 0x30, 0x8, 0x16, 0xE9, 0x0, 0x5F};
        static const std::uint8_t vtx_telem[] = {0xC8, 0x7, 0x10, 0xCE, 0xE, 0x16, 0x65, 0x0, 0x1B};
        static const std::uint8_t batt[] = {0xC8, 0x9, 0x8, 0x0, 0x9E, 0x0, 0x0, 0x0, 0x0, 0x0, 0x95};
        const std::uint8_t* bytes = id == 0 ? vtx_frame : (id == 1 ? vtx_telem : batt);
        const std::size_t len = id == 0 ? sizeof(vtx_frame) : (id == 1 ? sizeof(vtx_telem) : sizeof(batt));
        write_to_autopilot(reinterpret_cast<const char*>(bytes), len);
        id = (id + 1) % 3;
    }
};

class ELRS : public SerialDevice {
public:
    std::uint32_t last_update_ms = 0;
    void update(std::uint32_t now, const SitlInput& input) {
        if (now - last_update_ms < 20) {
            return;
        }
        last_update_ms = now;
        std::uint8_t pkt[26]{0xC8, 24, 0x16};
        for (int i = 0; i < 16; i++) {
            const std::uint16_t ch = input.servos[i] ? input.servos[i] : 1500;
            pkt[3 + i] = static_cast<std::uint8_t>((ch - 1000) * 255 / 1000);
        }
        pkt[25] = crc8_dvb_s2_update(0, pkt, 25);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 26);
    }
};

class Volz : public SerialDevice {
public:
    bool _enabled = true;
    std::uint32_t _output_mask = 0xFFFFFFFF;
    std::uint16_t position[32]{};
    void update(const SitlInput& input) {
        if (!_enabled) {
            return;
        }
        std::uint8_t cmd[6];
        if (read_from_autopilot(reinterpret_cast<char*>(cmd), 6) != 6) {
            return;
        }
        const std::uint8_t id = cmd[1];
        if (cmd[0] == 0xDC && id < 32) {
            position[id] = static_cast<std::uint16_t>((cmd[2] << 8) | cmd[3]);
            std::uint8_t resp[6]{0x2C, id, cmd[2], cmd[3], 0, 0};
            const std::uint16_t crc = crc16_ccitt(resp, 4, 0);
            resp[4] = static_cast<std::uint8_t>(crc >> 8);
            resp[5] = static_cast<std::uint8_t>(crc & 0xff);
            write_to_autopilot(reinterpret_cast<const char*>(resp), 6);
        }
        (void)input;
    }
};

}  // namespace fwcpp::sim
