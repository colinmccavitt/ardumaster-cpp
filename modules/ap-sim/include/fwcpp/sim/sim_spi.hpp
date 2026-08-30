#pragma once

// Port of libraries/SITL/SIM_SPI.cpp construction: RAMTRON_FM25V02 on
// bus 0 cs 0 and JEDEC_MX25L3206E on bus 1 cs 0. Storage is in-memory
// (original used a file descriptor). Invensense_v3 is I2C (ICM40609),
// not constructed on SIM_SPI.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

#include <fwcpp/sim/sim_aircraft.hpp>

namespace fwcpp::sim {

struct SpiIocTransfer {
    const std::uint8_t* tx_buf = nullptr;
    std::uint8_t* rx_buf = nullptr;
    std::uint32_t len = 0;
};

class SPIDevice {
public:
    virtual ~SPIDevice() = default;
    virtual void init() {}
    virtual void update(const Aircraft&) {}
    virtual int rdwr(std::uint8_t count, SpiIocTransfer* tfrs) = 0;
};

class RAMTRON : public SPIDevice {
public:
    enum class State { WAITING, READING_RDID, READING, WRITING };
    State state = State::WAITING;
    bool write_enabled = false;
    std::uint32_t xfr_addr = 0;
    std::vector<std::uint8_t> storage;
    virtual void fill_rdid(std::uint8_t* buf, std::uint8_t len) = 0;
    virtual std::uint32_t storage_size() const = 0;
    void ensure() {
        if (storage.size() != storage_size()) {
            storage.assign(storage_size(), 0xFF);
        }
    }
    int rdwr(std::uint8_t count, SpiIocTransfer* tfrs) override {
        ensure();
        static const std::uint8_t RAMTRON_RDID = 0x9f;
        static const std::uint8_t RAMTRON_READ = 0x03;
        static const std::uint8_t RAMTRON_WREN = 0x06;
        static const std::uint8_t RAMTRON_WRITE = 0x02;
        for (std::uint8_t i = 0; i < count; i++) {
            auto& tfr = tfrs[i];
            const std::uint8_t* tx_buf = tfr.tx_buf;
            std::uint8_t* rx_buf = tfr.rx_buf;
            switch (state) {
            case State::WAITING: {
                const std::uint8_t command = tx_buf[0];
                if (command == RAMTRON_RDID) {
                    state = State::READING_RDID;
                } else if (command == RAMTRON_READ) {
                    xfr_addr = (tx_buf[1] << 8) | tx_buf[2];
                    state = State::READING;
                } else if (command == RAMTRON_WRITE) {
                    xfr_addr = (tx_buf[1] << 8) | tx_buf[2];
                    state = State::WRITING;
                } else if (command == RAMTRON_WREN) {
                    write_enabled = true;
                } else {
                    return -1;
                }
                break;
            }
            case State::READING_RDID:
                fill_rdid(rx_buf, static_cast<std::uint8_t>(tfr.len));
                state = State::WAITING;
                break;
            case State::READING:
                if (xfr_addr + tfr.len > storage.size()) {
                    return -1;
                }
                std::memcpy(rx_buf, storage.data() + xfr_addr, tfr.len);
                state = State::WAITING;
                break;
            case State::WRITING:
                if (!write_enabled || xfr_addr + tfr.len > storage.size()) {
                    return -1;
                }
                std::memcpy(storage.data() + xfr_addr, tx_buf, tfr.len);
                state = State::WAITING;
                write_enabled = false;
                break;
            }
        }
        return 0;
    }
};

class RAMTRON_FM25V02 : public RAMTRON {
public:
    const std::uint8_t manufacturer[7]{0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2};
    static constexpr std::uint8_t family = 1;
    static constexpr std::uint8_t density = 2;
    void fill_rdid(std::uint8_t* buffer, std::uint8_t len) override {
        std::memcpy(buffer, manufacturer, std::min<std::uint8_t>(len, 7));
        if (len > 7) {
            buffer[7] = static_cast<std::uint8_t>(family << 5 | density);
        }
        if (len > 8) {
            buffer[8] = 0;
        }
    }
    std::uint32_t storage_size() const override { return 32768; }
};

class JEDEC : public SPIDevice {
public:
    enum class State { WAITING, READING_RDID, READING, WRITING, READING_RDSR };
    State state = State::WAITING;
    bool write_enabled = false;
    std::uint32_t xfr_addr = 0;
    std::vector<std::uint8_t> storage;
    virtual void fill_rdid(std::uint8_t* buf, std::uint8_t len) = 0;
    virtual void fill_rdsr(std::uint8_t* buf, std::uint8_t len) = 0;
    virtual std::uint8_t get_num_blocks() const = 0;
    virtual std::uint16_t get_page_per_block() const = 0;
    virtual std::uint8_t get_page_per_sector() const = 0;
    virtual std::uint16_t get_page_size() const = 0;
    std::uint32_t get_num_pages() const { return get_num_blocks() * get_page_per_block(); }
    std::uint32_t get_storage_size() const { return get_num_pages() * get_page_size(); }
    void ensure() {
        if (storage.size() != get_storage_size()) {
            storage.assign(get_storage_size(), 0xFF);
        }
    }
    std::uint32_t parse_addr(const std::uint8_t* buffer, std::uint32_t len) {
        if (len < 4) {
            return 0;
        }
        return (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
    }
    void page_erase(std::uint32_t addr) {
        const std::uint32_t n = get_page_size();
        if (addr + n <= storage.size()) {
            std::memset(storage.data() + addr, 0xFF, n);
        }
    }
    void sector4k_erase(std::uint32_t addr) {
        for (std::uint8_t i = 0; i < get_page_per_sector(); i++) {
            page_erase(addr + i * get_page_size());
        }
    }
    void block64k_erase(std::uint32_t addr) {
        for (std::uint16_t i = 0; i < 16; i++) {
            sector4k_erase(addr + i * get_page_per_sector() * get_page_size());
        }
    }
    void bulk_erase() {
        std::memset(storage.data(), 0xFF, storage.size());
    }
    int rdwr(std::uint8_t count, SpiIocTransfer* tfrs) override {
        ensure();
        static const std::uint8_t CMD_RDID = 0x9f;
        static const std::uint8_t CMD_READ = 0x03;
        static const std::uint8_t CMD_WREN = 0x06;
        static const std::uint8_t CMD_WRITE = 0x02;
        static const std::uint8_t CMD_RDSR = 0x05;
        static const std::uint8_t CMD_SE = 0x20;
        static const std::uint8_t CMD_BE = 0xD8;
        static const std::uint8_t CMD_CE = 0xC7;
        for (std::uint8_t i = 0; i < count; i++) {
            auto& tfr = tfrs[i];
            const std::uint8_t* tx = tfr.tx_buf;
            std::uint8_t* rx = tfr.rx_buf;
            switch (state) {
            case State::WAITING: {
                const std::uint8_t cmd = tx[0];
                if (cmd == CMD_RDID) {
                    state = State::READING_RDID;
                } else if (cmd == CMD_RDSR) {
                    state = State::READING_RDSR;
                } else if (cmd == CMD_READ) {
                    xfr_addr = parse_addr(tx, tfr.len);
                    state = State::READING;
                } else if (cmd == CMD_WRITE) {
                    xfr_addr = parse_addr(tx, tfr.len);
                    state = State::WRITING;
                } else if (cmd == CMD_WREN) {
                    write_enabled = true;
                } else if (cmd == CMD_SE) {
                    sector4k_erase(parse_addr(tx, tfr.len));
                } else if (cmd == CMD_BE) {
                    block64k_erase(parse_addr(tx, tfr.len));
                } else if (cmd == CMD_CE) {
                    bulk_erase();
                }
                break;
            }
            case State::READING_RDID:
                fill_rdid(rx, static_cast<std::uint8_t>(tfr.len));
                state = State::WAITING;
                break;
            case State::READING_RDSR:
                fill_rdsr(rx, static_cast<std::uint8_t>(tfr.len));
                state = State::WAITING;
                break;
            case State::READING:
                if (xfr_addr + tfr.len <= storage.size()) {
                    std::memcpy(rx, storage.data() + xfr_addr, tfr.len);
                }
                state = State::WAITING;
                break;
            case State::WRITING:
                if (write_enabled && xfr_addr + tfr.len <= storage.size()) {
                    for (std::uint32_t j = 0; j < tfr.len; j++) {
                        storage[xfr_addr + j] &= tx[j];
                    }
                }
                write_enabled = false;
                state = State::WAITING;
                break;
            }
        }
        return 0;
    }
};

class JEDEC_MX25L3206E : public JEDEC {
public:
    void fill_rdid(std::uint8_t* buf, std::uint8_t len) override {
        if (len >= 3) {
            buf[0] = 0xC2;
            buf[1] = 0x20;
            buf[2] = 0x16;
        }
    }
    void fill_rdsr(std::uint8_t* buf, std::uint8_t len) override {
        if (len >= 1) {
            buf[0] = write_enabled ? 0x02 : 0x00;
        }
    }
    std::uint8_t get_num_blocks() const override { return 64; }
    std::uint16_t get_page_per_block() const override { return 256; }
    std::uint8_t get_page_per_sector() const override { return 16; }
    std::uint16_t get_page_size() const override { return 256; }
};

struct SpiDeviceAtCs {
    std::uint8_t bus;
    std::uint8_t cs_pin;
    SPIDevice* device;
};

class SPI {
public:
    std::vector<SpiDeviceAtCs> devices;
    void init() {
        for (auto& d : devices) {
            if (d.device) {
                d.device->init();
            }
        }
    }
    int ioctl_transaction(std::uint8_t bus, std::uint8_t cs_pin, std::uint8_t count, SpiIocTransfer* data) {
        for (auto& d : devices) {
            if (d.bus == bus && d.cs_pin == cs_pin && d.device) {
                return d.device->rdwr(count, data);
            }
        }
        return -1;
    }
};

}  // namespace fwcpp::sim
