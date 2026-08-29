#pragma once

// Port of AP_HAL/Device.h register wrappers (read_registers,
// write_register, set_read_flag, set_speed) plus the in-memory
// slave/register bank that stands in for SITL's Linux i2c/spi ioctl
// path. CPP-088 slice 2.
//
// Upstream SITL (AP_HAL_SITL/I2CDevice.cpp _transfer ~146-196,
// SPIDevice.cpp transfer ~209+) builds i2c_msg / spi_ioc_transfer
// arrays and calls ioctl via sitl->i2c_ioctl / spi_ioctl. That ioctl
// is NOT ported: this bank is the SITL-subset. Empty nmsgs (no send
// and no recv) returns false, matching both backends.
//
// Typical register map used by Device::write_register / read_registers:
// first send byte is the start register; further send bytes write
// sequentially from that register; recv reads sequentially from that
// same start register (after the optional address-byte write).
// read_registers ORs _read_flag onto first_reg, then
// transfer(&first_reg, 1, recv, recv_len). write_register sends
// {reg, val} with no recv.
//
// setup_checked_registers is left remaining/OOS: upstream heap-
// allocates NEW_NOTHROW checkreg[nregs] (Device.cpp:48). write_register's
// checked flag is accepted and ignored.
//
// Concrete, no virtual Device base, no OwnPtr, no flight-path alloc.

#include <array>
#include <cstdint>

namespace fwcpp::hal {

// AP_HAL::Device::BusType.
enum class BusType : std::uint8_t {
    kUnknown = 0,
    kI2c = 1,
    kSpi = 2,
    kUavcan = 3,
    kSitl = 4,
    kMsp = 5,
    kSerial = 6,
    kWspi = 7,
};

// AP_HAL::Device::Speed. Stored as a bus hint; transfer does not
// change bank behavior (SITL set_speed is a no-op that returns true).
enum class Speed : std::uint8_t {
    kHigh = 0,
    kLow = 1,
};

// Token returned by register_periodic_callback. SITL heap-allocates a
// callback_info and returns the pointer; we record period+token on the
// bus and never fire (no OS threads / no per-bus timer tick).
using PeriodicHandle = std::uint32_t;

inline constexpr std::size_t kDeviceBankMaxSlaves = 16;
inline constexpr std::uint16_t kDeviceRegisterCount = 256;

// Shared in-memory slave/register bank keyed by bus + address (I2C)
// or bus + cs_pin (SPI). Caller-owned; devices hold a reference.
class DeviceRegisterBank {
public:
    // Combined send-then-recv, matching SITL I2CDevice::_transfer /
    // SPIDevice::transfer message assembly. Empty nmsgs -> false.
    // Does not call Linux ioctl.
    [[nodiscard]] bool transfer(std::uint8_t bus, std::uint8_t address,
                                const std::uint8_t* send, std::uint32_t send_len,
                                std::uint8_t* recv, std::uint32_t recv_len) {
        const bool have_send = send != nullptr && send_len != 0;
        const bool have_recv = recv != nullptr && recv_len != 0;
        if (!have_send && !have_recv) {
            return false;
        }
        Slave* slave = find_or_create(bus, address);
        if (slave == nullptr) {
            return false;
        }
        std::uint8_t start = 0;
        if (have_send) {
            start = send[0];
            for (std::uint32_t i = 1; i < send_len; ++i) {
                slave->regs[static_cast<std::uint8_t>(start + (i - 1))] = send[i];
            }
        }
        if (have_recv) {
            for (std::uint32_t i = 0; i < recv_len; ++i) {
                recv[i] = slave->regs[static_cast<std::uint8_t>(start + i)];
            }
        }
        return true;
    }

    [[nodiscard]] std::uint8_t slave_count() const { return occupied_; }

    [[nodiscard]] std::uint8_t peek(std::uint8_t bus, std::uint8_t address,
                                    std::uint8_t reg) const {
        const Slave* slave = find(bus, address);
        return slave != nullptr ? slave->regs[reg] : 0;
    }

private:
    struct Slave {
        bool occupied = false;
        std::uint8_t bus = 0;
        std::uint8_t address = 0;
        std::array<std::uint8_t, kDeviceRegisterCount> regs{};
    };

    [[nodiscard]] const Slave* find(std::uint8_t bus, std::uint8_t address) const {
        for (std::uint8_t i = 0; i < occupied_; ++i) {
            if (slaves_[i].occupied && slaves_[i].bus == bus &&
                slaves_[i].address == address) {
                return &slaves_[i];
            }
        }
        return nullptr;
    }

    [[nodiscard]] Slave* find(std::uint8_t bus, std::uint8_t address) {
        return const_cast<Slave*>(
            static_cast<const DeviceRegisterBank*>(this)->find(bus, address));
    }

    [[nodiscard]] Slave* find_or_create(std::uint8_t bus, std::uint8_t address) {
        if (Slave* existing = find(bus, address)) {
            return existing;
        }
        if (occupied_ >= kDeviceBankMaxSlaves) {
            return nullptr;
        }
        Slave& slot = slaves_[occupied_];
        slot.occupied = true;
        slot.bus = bus;
        slot.address = address;
        slot.regs.fill(0);
        ++occupied_;
        return &slot;
    }

    std::array<Slave, kDeviceBankMaxSlaves> slaves_{};
    std::uint8_t occupied_ = 0;
};

// Non-virtual Device: bus id, speed, read_flag, and the register
// wrappers from AP_HAL::Device.cpp. transfer() talks to the bank.
// I2cDevice / SpiDevice add bus-specific seams on top.
class Device {
public:
    Device(DeviceRegisterBank& bank, BusType type, std::uint8_t bus,
           std::uint8_t address)
        : bank_(bank), bus_type_(type), bus_(bus), address_(address) {}

    [[nodiscard]] BusType bus_type() const { return bus_type_; }
    [[nodiscard]] std::uint8_t bus_num() const { return bus_; }
    [[nodiscard]] std::uint8_t get_bus_address() const { return address_; }

    void set_address(std::uint8_t address) { address_ = address; }
    void set_device_type(std::uint8_t devtype) { devtype_ = devtype; }
    void set_retries(std::uint8_t retries) { retries_ = retries; }

    [[nodiscard]] std::uint8_t retries() const { return retries_; }
    [[nodiscard]] std::uint8_t device_type() const { return devtype_; }

    // Stored; transfer still uses the bank. SITL's set_speed returns
    // true and does not keep the value — we keep it as a test seam.
    bool set_speed(Speed speed) {
        speed_ = speed;
        return true;
    }

    [[nodiscard]] Speed speed() const { return speed_; }

    void set_read_flag(std::uint8_t flag) { read_flag_ = flag; }
    [[nodiscard]] std::uint8_t read_flag() const { return read_flag_; }

    [[nodiscard]] bool transfer(const std::uint8_t* send, std::uint32_t send_len,
                                std::uint8_t* recv, std::uint32_t recv_len) {
        return bank_.transfer(bus_, address_, send, send_len, recv, recv_len);
    }

    // Device.cpp:183 — OR _read_flag onto first_reg, then
    // transfer(&first_reg, 1, recv, recv_len).
    [[nodiscard]] bool read_registers(std::uint8_t first_reg, std::uint8_t* recv,
                                      std::uint32_t recv_len) {
        first_reg |= read_flag_;
        return transfer(&first_reg, 1, recv, recv_len);
    }

    // Device.cpp:169 — buf={reg,val}, transfer send-only.
    // checked-register recording is OOS (heap NEW_NOTHROW array).
    [[nodiscard]] bool write_register(std::uint8_t reg, std::uint8_t val,
                                      bool checked = false) {
        const std::uint8_t buf[2] = {reg, val};
        (void)checked;
        return transfer(buf, 2, nullptr, 0);
    }

    // Device.h:248 — transfer with no send, no read_flag.
    [[nodiscard]] bool read(std::uint8_t* recv, std::uint32_t recv_len) {
        return transfer(nullptr, 0, recv, recv_len);
    }

    // Upstream heap-allocates checkreg[nregs]. Refused here (no
    // flight-path alloc). Callers see the same failure as a failed NEW.
    [[nodiscard]] bool setup_checked_registers(std::uint8_t nregs,
                                               std::uint8_t frequency = 10) {
        (void)nregs;
        (void)frequency;
        return false;
    }

protected:
    DeviceRegisterBank& bank_;
    BusType bus_type_;
    std::uint8_t bus_;
    std::uint8_t address_;
    std::uint8_t devtype_ = 0;
    std::uint8_t retries_ = 0;
    std::uint8_t read_flag_ = 0;
    Speed speed_ = Speed::kHigh;
};

}  // namespace fwcpp::hal
