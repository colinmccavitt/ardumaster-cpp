#pragma once

// CPP-088 completeness: slice 2 (SITL I2C/SPI Device register access)
// vs remaining AP_HAL Util / WSPI / CAN. remaining_count() == 3 until
// later slices land those surfaces. Slice 1 GPIO + Semaphore is on main.

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
    {"I2CDevice", PortStatus::kThisSlice,
     "AP_HAL::I2CDevice in-memory bank transfers (no Linux ioctl)"},
    {"SPIDevice", PortStatus::kThisSlice,
     "AP_HAL::SPIDevice in-memory bank transfers (no Linux ioctl)"},
    {"Device register access", PortStatus::kThisSlice,
     "AP_HAL::Device read_registers/write_register, set_speed, periodic token"},
    {"Util", PortStatus::kRemaining,
     "AP_HAL::Util safety_switch, system_id, persistent_data"},
    {"WSPI", PortStatus::kRemaining, "AP_HAL::WSPIDevice wrap/quad SPI"},
    {"CAN", PortStatus::kRemaining, "AP_HAL::CANIface send/receive, bitrate, filter"},
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
