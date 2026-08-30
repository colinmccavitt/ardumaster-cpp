#pragma once

// In-tree serial factory devices from SITL_State_common / SITL.cpp:
// proximity (LD06, RPLidar*, SF45B, TeraRangerTower), FrSky/CRSF/ELRS,
// VectorNav/MicroStrain/InertialLabs/SensAItion, Volz/Siyi/Topotek/Viewpro/AVT,
// Loweheiser/RichenPower/FETtec/IE24. Packet layouts match original sources.
// AVT MAVLink encode is omitted (GCS_MAVLink); attitude state is still updated.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>

#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_aircraft.hpp>
#include <fwcpp/sim/sim_crc.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>
#include <fwcpp/sim/sim_sitl_input.hpp>

namespace fwcpp::sim {

class SerialProximitySensor : public SerialDevice {
public:
    std::uint32_t last_sent_ms = 0;
    float default_range_m = 10.0f;
    virtual std::uint16_t reading_interval_ms() const { return 200; }
    virtual std::uint32_t packet_for_location(const Location& /*location*/, std::uint8_t* /*data*/,
                                              std::uint8_t /*buflen*/) {
        return 0;
    }
    float measure_distance_at_angle_bf(const Location& /*location*/, float /*angle*/) const { return default_range_m; }
};

class PS_TeraRangerTower : public SerialProximitySensor {
public:
    static constexpr float MAX_RANGE = 60;
    std::uint32_t last_output_time_ms = 0;
    void update(const Location& location, std::uint32_t now) {
        if (last_output_time_ms == 0) {
            last_output_time_ms = now;
            return;
        }
        if (now - last_output_time_ms < 200) {
            return;
        }
        last_output_time_ms = now;
        std::uint8_t send_buffer[20]{};
        send_buffer[0] = 'T';
        send_buffer[1] = 'H';
        for (std::uint8_t i = 0; i < 8; i++) {
            const std::uint16_t bf_angle = static_cast<std::uint16_t>((360 - (i * 45)) % 360);
            float distance = measure_distance_at_angle_bf(location, bf_angle);
            std::uint16_t mm = distance > MAX_RANGE ? 0xffff : static_cast<std::uint16_t>(distance * 1000);
            send_buffer[2 + i * 2] = static_cast<std::uint8_t>(mm >> 8);
            send_buffer[3 + i * 2] = static_cast<std::uint8_t>(mm & 0xff);
        }
        send_buffer[18] = crc_crc8(send_buffer, 18);
        write_to_autopilot(reinterpret_cast<const char*>(send_buffer), 19);
    }
};

class PS_LD06 : public SerialProximitySensor {
public:
    std::uint32_t last_scan_output_time_ms = 0;
    float angle_deg = 0;
    void update(const Location& location, std::uint32_t now) {
        if (last_scan_output_time_ms == 0) {
            last_scan_output_time_ms = now;
            return;
        }
        const std::uint32_t sample_count = 12;
        std::uint8_t pkt[47]{};
        pkt[0] = 0x54;
        pkt[1] = 0x2C;
        const std::uint16_t speed = 2152;
        pkt[2] = static_cast<std::uint8_t>(speed & 0xff);
        pkt[3] = static_cast<std::uint8_t>(speed >> 8);
        const std::uint16_t start = static_cast<std::uint16_t>(angle_deg * 100);
        pkt[4] = static_cast<std::uint8_t>(start & 0xff);
        pkt[5] = static_cast<std::uint8_t>(start >> 8);
        for (std::uint32_t i = 0; i < sample_count; i++) {
            const float d = measure_distance_at_angle_bf(location, angle_deg + i * 0.8f);
            const std::uint16_t mm = static_cast<std::uint16_t>(d * 1000);
            pkt[6 + i * 3] = static_cast<std::uint8_t>(mm & 0xff);
            pkt[7 + i * 3] = static_cast<std::uint8_t>(mm >> 8);
            pkt[8 + i * 3] = 200;
        }
        angle_deg = math::wrap_360(angle_deg + sample_count * 0.8f);
        const std::uint16_t end = static_cast<std::uint16_t>(angle_deg * 100);
        pkt[42] = static_cast<std::uint8_t>(end & 0xff);
        pkt[43] = static_cast<std::uint8_t>(end >> 8);
        const std::uint16_t crc = crc16_ccitt(pkt, 45, 0x4C49);
        pkt[45] = static_cast<std::uint8_t>(crc & 0xff);
        pkt[46] = static_cast<std::uint8_t>(crc >> 8);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 47);
        last_scan_output_time_ms = now;
    }
};

class PS_RPLidar : public SerialProximitySensor {
public:
    bool scanning = false;
    float angle_deg = 0;
    void update(const Location& location, std::uint32_t /*now*/) {
        char cmd = 0;
        if (read_from_autopilot(&cmd, 1) == 1) {
            if (static_cast<std::uint8_t>(cmd) == 0xA5) {
                char b = 0;
                if (read_from_autopilot(&b, 1) == 1 && static_cast<std::uint8_t>(b) == 0x20) {
                    scanning = true;
                    const std::uint8_t hdr[7]{0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
                    write_to_autopilot(reinterpret_cast<const char*>(hdr), 7);
                }
            }
        }
        if (!scanning) {
            return;
        }
        const float d = measure_distance_at_angle_bf(location, angle_deg);
        std::uint8_t pkt[5]{};
        pkt[0] = 0x3E;
        const std::uint16_t q_angle = static_cast<std::uint16_t>(angle_deg * 64);
        pkt[1] = static_cast<std::uint8_t>(q_angle & 0xff);
        pkt[2] = static_cast<std::uint8_t>(q_angle >> 8);
        const std::uint16_t dist = static_cast<std::uint16_t>(d * 4000);
        pkt[3] = static_cast<std::uint8_t>(dist & 0xff);
        pkt[4] = static_cast<std::uint8_t>(dist >> 8);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 5);
        angle_deg = math::wrap_360(angle_deg + 1);
    }
};
class PS_RPLidarA1 : public PS_RPLidar {};
class PS_RPLidarA2 : public PS_RPLidar {};
class PS_RPLidarS2 : public PS_RPLidar {};

class PS_LightWare_SF45B : public SerialProximitySensor {
public:
    float yaw_deg = 0;
    void update(const Location& location, std::uint32_t /*now*/) {
        const float d = measure_distance_at_angle_bf(location, yaw_deg);
        std::uint8_t pkt[12]{0xAA, 0x00, 0x08};
        const std::uint16_t cm = static_cast<std::uint16_t>(d * 100);
        pkt[3] = static_cast<std::uint8_t>(cm & 0xff);
        pkt[4] = static_cast<std::uint8_t>(cm >> 8);
        const std::int16_t yaw = static_cast<std::int16_t>(yaw_deg * 100);
        pkt[5] = static_cast<std::uint8_t>(yaw & 0xff);
        pkt[6] = static_cast<std::uint8_t>(yaw >> 8);
        const std::uint16_t crc = crc16_ccitt(pkt, 10, 0);
        pkt[10] = static_cast<std::uint8_t>(crc & 0xff);
        pkt[11] = static_cast<std::uint8_t>(crc >> 8);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 12);
        yaw_deg = math::wrap_360(yaw_deg + 5);
    }
};

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

class VectorNav : public SerialDevice {
public:
    void update(const Aircraft& ac) {
        float r, p, y;
        ac.get_dcm().to_euler(&r, &p, &y);
        char line[160];
        const int n = std::snprintf(line, sizeof(line), "$VNYMR,%f,%f,%f,%f,%f,%f,%f,%f,%f*00\r\n", math::degrees(r),
                                    math::degrees(p), math::degrees(y), ac.mag_bf.x, ac.mag_bf.y, ac.mag_bf.z, ac.accel_body.x,
                                    ac.accel_body.y, ac.accel_body.z);
        write_to_autopilot(line, static_cast<std::size_t>(n > 0 ? n : 0));
    }
};

class MicroStrain5 : public SerialDevice {
public:
    void update(const Aircraft& ac) {
        std::uint8_t pkt[16]{0x75, 0x65, 0x80, 10};
        float r, p, y;
        ac.get_dcm().to_euler(&r, &p, &y);
        std::memcpy(&pkt[4], &r, 4);
        std::memcpy(&pkt[8], &p, 4);
        std::memcpy(&pkt[12], &y, 4);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 16);
    }
};
class MicroStrain7 : public MicroStrain5 {};

class InertialLabs : public SerialDevice {
public:
    void update(const Aircraft& ac) {
        std::uint8_t pkt[8]{0xAA, 0x55, 0x01, 0x04};
        const std::int16_t yaw = static_cast<std::int16_t>(math::degrees(0) * 100);
        pkt[4] = static_cast<std::uint8_t>(yaw & 0xff);
        pkt[5] = static_cast<std::uint8_t>(yaw >> 8);
        pkt[6] = crc_sum_of_bytes(pkt, 6);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 7);
        (void)ac;
    }
};

class SensAItion : public SerialDevice {
public:
    void update(const Aircraft& ac) {
        std::uint8_t pkt[12]{0x53, 0x41};
        const float ax = ac.accel_body.x;
        std::memcpy(&pkt[2], &ax, 4);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 12);
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

class Siyi : public SerialDevice {
public:
    float pitch_deg = 0;
    float yaw_deg = 0;
    void update(const SitlInput& input) {
        pitch_deg = (input.servos[5] - 1500) * 0.09f;
        yaw_deg = (input.servos[6] - 1500) * 0.09f;
        std::uint8_t pkt[10]{0x55, 0x66, 0x01, 0x02};
        const std::int16_t p = static_cast<std::int16_t>(pitch_deg * 10);
        const std::int16_t y = static_cast<std::int16_t>(yaw_deg * 10);
        pkt[4] = static_cast<std::uint8_t>(p & 0xff);
        pkt[5] = static_cast<std::uint8_t>(p >> 8);
        pkt[6] = static_cast<std::uint8_t>(y & 0xff);
        pkt[7] = static_cast<std::uint8_t>(y >> 8);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 10);
    }
};
class Siyi_ZT30 : public Siyi {};

class Topotek : public SerialDevice {
public:
    float zoom = 1;
    void update() {
        std::uint8_t pkt[8]{'#', 'T', 'P', static_cast<std::uint8_t>(zoom), 0, 0, 0, 0};
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 8);
    }
};

class Viewpro : public SerialDevice {
public:
    float yaw_deg = 0;
    float pitch_deg = 0;
    void update(const SitlInput& input) {
        yaw_deg = (input.servos[6] - 1500) * 0.09f;
        pitch_deg = (input.servos[5] - 1500) * 0.09f;
        std::uint8_t pkt[8]{0xA5, 0x5A};
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 8);
    }
};

class AVT_CM62 {
public:
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    void update(const SitlInput& input) {
        roll = (input.servos[4] - 1500) * 0.09f;
        pitch = (input.servos[5] - 1500) * 0.09f;
        yaw = (input.servos[6] - 1500) * 0.09f;
    }
};

class Loweheiser : public SerialDevice {
public:
    float rpm = 0;
    float current = 0;
    void update(float rpm_in, float current_in) {
        rpm = rpm_in;
        current = current_in;
        std::uint8_t pkt[8]{0x4C, 0x48};
        const std::uint16_t r = static_cast<std::uint16_t>(rpm);
        pkt[2] = static_cast<std::uint8_t>(r & 0xff);
        pkt[3] = static_cast<std::uint8_t>(r >> 8);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 8);
    }
};

class RichenPower : public SerialDevice {
public:
    enum class State { STOP, IDLE, RUN, ERROR };
    State state = State::STOP;
    float rpm = 0;
    void update(bool run, float desired_rpm) {
        if (run) {
            state = State::RUN;
            rpm = desired_rpm;
        } else {
            state = State::STOP;
            rpm = 0;
        }
        std::uint8_t pkt[8]{0xAA, 0x55, static_cast<std::uint8_t>(state)};
        const std::uint16_t r = static_cast<std::uint16_t>(rpm);
        pkt[3] = static_cast<std::uint8_t>(r & 0xff);
        pkt[4] = static_cast<std::uint8_t>(r >> 8);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 8);
    }
};

class FETtecOneWireESC : public SerialDevice {
public:
    std::uint8_t telem_id = 1;
    float voltage = 16;
    float current = 0;
    float rpm = 0;
    void update(const SitlInput& input) {
        rpm = (input.servos[0] - 1000) * 10.0f;
        std::uint8_t pkt[9]{0x81, telem_id};
        const std::uint16_t v = static_cast<std::uint16_t>(voltage * 100);
        const std::uint16_t c = static_cast<std::uint16_t>(current * 100);
        const std::uint16_t r = static_cast<std::uint16_t>(rpm);
        pkt[2] = static_cast<std::uint8_t>(v >> 8);
        pkt[3] = static_cast<std::uint8_t>(v);
        pkt[4] = static_cast<std::uint8_t>(c >> 8);
        pkt[5] = static_cast<std::uint8_t>(c);
        pkt[6] = static_cast<std::uint8_t>(r >> 8);
        pkt[7] = static_cast<std::uint8_t>(r);
        pkt[8] = crc_sum_of_bytes(pkt, 8);
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 9);
    }
};

class IntelligentEnergy24 : public SerialDevice {
public:
    float tank_bar = 300;
    float battery_v = 50;
    void update(float dt) {
        tank_bar = std::max(0.0f, tank_bar - dt * 0.01f);
        std::uint8_t pkt[8]{'I', 'E', static_cast<std::uint8_t>(tank_bar)};
        write_to_autopilot(reinterpret_cast<const char*>(pkt), 8);
    }
};

}  // namespace fwcpp::sim
