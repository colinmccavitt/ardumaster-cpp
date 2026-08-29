#pragma once

// Port of AP_HAL::SPIDevice / HALSITL::SPIDevice as a concrete
// caller-owned type. CPP-088 slice 2.
//
// SITL SPIDevice::transfer builds spi_ioc_transfer[] and ioctl via
// sitl->spi_ioctl (SPIDevice.cpp ~209+). That Linux ioctl is NOT
// ported; DeviceRegisterBank is the SITL-subset, keyed by bus+cs_pin.
// Empty nmsgs -> false. Combined send then recv.
//
// set_speed stores SPEED_HIGH/LOW; transfer still uses the bank
// (SITL has "no concept of speed" and returns true without storing).
//
// get_semaphore returns the caller-owned Semaphore on SpiBus.
// register_periodic_callback records period+token without firing.
// transfer_fullduplex is not implemented (SITL abort()s); returns false.
//
// No SPIDeviceManager / get_device() / OwnPtr.

#include <cstdint>

#include <fwcpp/hal/device.hpp>
#include <fwcpp/hal/semaphore.hpp>

namespace fwcpp::hal {

class SpiBus {
public:
    explicit SpiBus(std::uint8_t bus = 0) : bus_(bus) {}

    [[nodiscard]] std::uint8_t bus() const { return bus_; }

    [[nodiscard]] Semaphore& semaphore() { return sem_; }
    [[nodiscard]] const Semaphore& semaphore() const { return sem_; }

    PeriodicHandle register_periodic_callback(std::uint32_t period_usec) {
        period_usec_ = period_usec;
        ++token_;
        if (token_ == 0) {
            token_ = 1;
        }
        has_callback_ = true;
        return token_;
    }

    // Base AP_HAL::SPIDevice::adjust_periodic_callback returns false.
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

class SpiDevice : public Device {
public:
    SpiDevice(SpiBus& bus, DeviceRegisterBank& bank, std::uint8_t cs_pin)
        : Device(bank, BusType::kSpi, bus.bus(), cs_pin), bus_(bus) {}

    [[nodiscard]] std::uint8_t cs_pin() const { return get_bus_address(); }

    [[nodiscard]] Semaphore& get_semaphore() { return bus_.semaphore(); }
    [[nodiscard]] const Semaphore& get_semaphore() const { return bus_.semaphore(); }

    [[nodiscard]] SpiBus& bus() { return bus_; }
    [[nodiscard]] const SpiBus& bus() const { return bus_; }

    PeriodicHandle register_periodic_callback(std::uint32_t period_usec) {
        return bus_.register_periodic_callback(period_usec);
    }

    bool adjust_periodic_callback(PeriodicHandle h, std::uint32_t period_usec) {
        return bus_.adjust_periodic_callback(h, period_usec);
    }

    // SITL SPIDevice::transfer_fullduplex abort()s. Return false rather
    // than abort; not used by the register-wrapper path.
    [[nodiscard]] bool transfer_fullduplex(const std::uint8_t* send, std::uint8_t* recv,
                                           std::uint32_t len) {
        (void)send;
        (void)recv;
        (void)len;
        return false;
    }

    [[nodiscard]] bool clock_pulse(std::uint32_t len) {
        (void)len;
        return false;
    }

private:
    SpiBus& bus_;
};

}  // namespace fwcpp::hal
