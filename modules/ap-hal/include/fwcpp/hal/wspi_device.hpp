#pragma once

// Port of AP_HAL::WSPIDevice as a concrete caller-owned type.
// CPP-088 slice 5. In-memory wrap only — no ChibiOS HAL
// (wspiStart / wspiCommand / bouncebuffer / STM32 QSPI/OSPI).
// Those backends are CPP-089.
//
// Upstream ChibiOS WSPIDevice::transfer (WSPIDevice.cpp ~97-125):
//   acquire_bus (check_owner + wspiAcquireBus / wspiStart);
//   send_len==0 && recv_len==0 -> wspiCommand (command-only SUCCESS);
//   send_len>0 && recv==nullptr -> wspiSend;
//   send_len==0 && recv_len>=1 -> wspiReceive;
//   both send and recv -> false ("Can't handle this transaction type").
//
// This SITL-subset reuses DeviceRegisterBank for send-only / recv-only
// data. Command-only does NOT go through the bank: DeviceRegisterBank
// treats empty nmsgs as false (I2C/SPI rule), which is wrong for WSPI
// command-only.
//
// is_busy is an injected bool (default false), not QUADSPI_SR_BUSY /
// OCTOSPI_SR_BUSY.
//
// acquire_bus check_owner is NOT enforced: Semaphore has no OS owner
// thread. Transfer is allowed when the bus taken-count is 0. Callers
// may still take get_semaphore() in tests. Disclosed vs ChibiOS
// WSPIDevice::acquire_bus ~137-162.
//
// No WSPIDeviceManager / get_device / OwnPtr. No QuadSPI CFG_* line-
// mode constants (optional leftover, OOS).

#include <cstdint>

#include <fwcpp/hal/device.hpp>
#include <fwcpp/hal/semaphore.hpp>

namespace fwcpp::hal {

// AP_HAL::Device::CommandHeader (Device.h ~53-59).
struct CommandHeader {
    std::uint32_t cmd = 0;
    std::uint32_t cfg = 0;
    std::uint32_t addr = 0;
    std::uint32_t alt = 0;
    std::uint32_t dummy = 0;
};

class WspiBus {
public:
    explicit WspiBus(std::uint8_t bus = 0) : bus_(bus) {}

    [[nodiscard]] std::uint8_t bus() const { return bus_; }

    [[nodiscard]] Semaphore& semaphore() { return sem_; }
    [[nodiscard]] const Semaphore& semaphore() const { return sem_; }

private:
    std::uint8_t bus_ = 0;
    Semaphore sem_{};
};

class WspiDevice : public Device {
public:
    WspiDevice(WspiBus& bus, DeviceRegisterBank& bank, std::uint8_t address = 0)
        : Device(bank, BusType::kWspi, bus.bus(), address), bus_(bus) {}

    void set_cmd_header(const CommandHeader& cmd_hdr) { header_ = cmd_hdr; }

    [[nodiscard]] const CommandHeader& cmd_header() const { return header_; }

    void set_busy(bool busy) { busy_ = busy; }

    [[nodiscard]] bool is_busy() const { return busy_; }

    // Shadows Device::transfer: command-only is success, combined
    // send+recv is refused. Bank empty-nmsgs false is not reused.
    [[nodiscard]] bool transfer(const std::uint8_t* send, std::uint32_t send_len,
                                std::uint8_t* recv, std::uint32_t recv_len) {
        if (busy_) {
            return false;
        }
        if (send_len == 0 && recv_len == 0) {
            return true;
        }
        if (send_len > 0 && recv == nullptr) {
            return Device::transfer(send, send_len, recv, recv_len);
        }
        if (send_len == 0 && recv_len >= 1) {
            return Device::transfer(send, send_len, recv, recv_len);
        }
        return false;
    }

    [[nodiscard]] Semaphore& get_semaphore() { return bus_.semaphore(); }
    [[nodiscard]] const Semaphore& get_semaphore() const { return bus_.semaphore(); }

    [[nodiscard]] WspiBus& bus() { return bus_; }
    [[nodiscard]] const WspiBus& bus() const { return bus_; }

private:
    WspiBus& bus_;
    CommandHeader header_{};
    bool busy_ = false;
};

}  // namespace fwcpp::hal
