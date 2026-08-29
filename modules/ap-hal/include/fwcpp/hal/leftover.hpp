#pragma once

// CPP-088 completeness: slice 5 (in-memory WSPIDevice). GPIO, I2C/SPI,
// Util, CAN are on main. remaining_count() == 0.

#include <cstddef>
#include <cstdint>

namespace fwcpp::hal {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct HalPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr HalPortItem kHalCompleteness[] = {
    {"GPIO", PortStatus::kOnMain,
     "AP_HAL::GPIO pin mode, digital read/write; SITL 16-pin table"},
    {"Semaphore", PortStatus::kOnMain,
     "AP_HAL::Semaphore take/give, take_nonblocking (recursive counter)"},
    {"BinarySemaphore", PortStatus::kOnMain,
     "AP_HAL::BinarySemaphore wait/signal (pending flag, no OS thread)"},
    {"completeness catalog", PortStatus::kOnMain, "this table"},
    {"I2CDevice", PortStatus::kOnMain,
     "AP_HAL::I2CDevice in-memory bank transfers (no Linux ioctl)"},
    {"SPIDevice", PortStatus::kOnMain,
     "AP_HAL::SPIDevice in-memory bank transfers (no Linux ioctl)"},
    {"Device register access", PortStatus::kOnMain,
     "AP_HAL::Device read_registers/write_register, set_speed, periodic token"},
    {"Util", PortStatus::kOnMain,
     "AP_HAL::Util SITL-subset: soft_armed, safety_switch, system_id, persistent_data"},
    {"CAN", PortStatus::kOnMain,
     "AP_HAL::CANIface in-memory send/receive (no SocketCAN)"},
    {"WSPI", PortStatus::kThisSlice,
     "AP_HAL::WSPIDevice in-memory command header and transfer (no ChibiOS)"},
};

[[nodiscard]] inline constexpr std::size_t hal_completeness_size() {
    return sizeof(kHalCompleteness) / sizeof(kHalCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kHalCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kHalCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}
[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}

}  // namespace fwcpp::hal
