#pragma once

// Port of AP_HAL::I2CDevice / HALSITL::I2CDevice as a concrete
// caller-owned type. CPP-088 slice 2.
//
// SITL I2CDevice::transfer builds i2c_msg[] and ioctl(I2C_RDWR) into
// sitl->i2c_ioctl (I2CDevice.cpp ~146-196). That Linux ioctl is NOT
// ported; DeviceRegisterBank is the SITL-subset. Empty nmsgs -> false.
// Combined send then recv against the bank keyed by bus+address.
//
// get_semaphore returns the caller-owned Semaphore already on I2cBus
// (slice-1 recursive counter, no OS mutex).
//
// register_periodic_callback records period+token on the bus and does
// not fire (SITL heap-allocates callback_info and ticks from
// I2CBus::_timer_tick; we have no threads). adjust_periodic_callback
// returns false, matching HALSITL::I2CDevice.
//
// read_registers_multiple: SITL prints and returns false — we return
// false (no stderr).
//
// No I2CDeviceManager / get_device() / OwnPtr.

#include <cstdint>

#include <fwcpp/hal/device.hpp>
#include <fwcpp/hal/semaphore.hpp>

namespace fwcpp::hal {

class I2cBus {
public:
    explicit I2cBus(std::uint8_t bus = 0) : bus_(bus) {}

    [[nodiscard]] std::uint8_t bus() const { return bus_; }

    [[nodiscard]] Semaphore& semaphore() { return sem_; }
    [[nodiscard]] const Semaphore& semaphore() const { return sem_; }

    // Record only; never invoked. Token is a non-zero counter.
    PeriodicHandle register_periodic_callback(std::uint32_t period_usec) {
        period_usec_ = period_usec;
        ++token_;
        if (token_ == 0) {
            token_ = 1;
        }
        has_callback_ = true;
        return token_;
    }

    // HALSITL::I2CDevice::adjust_periodic_callback always returns false.
    bool adjust_periodic_callback(PeriodicHandle h, std::uint32_t period_usec) {
        (void)h;
        (void)period_usec;
        return false;
    }

    [[nodiscard]] bool has_periodic_callback() const { return has_callback_; }
    [[nodiscard]] std::uint32_t periodic_period_usec() const { return period_usec_; }
    [[nodiscard]] PeriodicHandle periodic_token() const { return token_; }

private:
    std::uint8_t bus_ = 0;
    Semaphore sem_{};
    bool has_callback_ = false;
    std::uint32_t period_usec_ = 0;
    PeriodicHandle token_ = 0;
};

class I2cDevice : public Device {
public:
    I2cDevice(I2cBus& bus, DeviceRegisterBank& bank, std::uint8_t address)
        : Device(bank, BusType::kI2c, bus.bus(), address), bus_(bus) {}

    [[nodiscard]] Semaphore& get_semaphore() { return bus_.semaphore(); }
    [[nodiscard]] const Semaphore& get_semaphore() const { return bus_.semaphore(); }

    [[nodiscard]] I2cBus& bus() { return bus_; }
    [[nodiscard]] const I2cBus& bus() const { return bus_; }

    PeriodicHandle register_periodic_callback(std::uint32_t period_usec) {
        return bus_.register_periodic_callback(period_usec);
    }

    bool adjust_periodic_callback(PeriodicHandle h, std::uint32_t period_usec) {
        return bus_.adjust_periodic_callback(h, period_usec);
    }

    // HALSITL::I2CDevice::read_registers_multiple: fprintf + return false.
    [[nodiscard]] bool read_registers_multiple(std::uint8_t first_reg, std::uint8_t* recv,
                                               std::uint32_t recv_len, std::uint8_t times) {
        (void)first_reg;
        (void)recv;
        (void)recv_len;
        (void)times;
        return false;
    }

    void set_split_transfers(bool set) { split_transfers_ = set; }
    [[nodiscard]] bool split_transfers() const { return split_transfers_; }

private:
    I2cBus& bus_;
    bool split_transfers_ = false;
};

}  // namespace fwcpp::hal
