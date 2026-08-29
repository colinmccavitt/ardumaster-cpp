#pragma once

// CPP-088 completeness: this slice (SITL GPIO + Semaphore) vs remaining
// AP_HAL device-bus / Util surfaces. remaining_count() > 0 until later
// slices land I2C/SPI/Device register access, Util, WSPI, and CAN.

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
    {"GPIO", PortStatus::kThisSlice,
     "AP_HAL::GPIO pin mode, digital read/write; SITL 16-pin table"},
    {"Semaphore", PortStatus::kThisSlice,
     "AP_HAL::Semaphore take/give, take_nonblocking (recursive counter)"},
    {"BinarySemaphore", PortStatus::kThisSlice,
     "AP_HAL::BinarySemaphore wait/signal (pending flag, no OS thread)"},
    {"completeness catalog", PortStatus::kThisSlice, "this table"},
    {"I2CDevice", PortStatus::kRemaining, "AP_HAL::I2CDevice bus transfers"},
    {"SPIDevice", PortStatus::kRemaining, "AP_HAL::SPIDevice bus transfers"},
    {"Device register access", PortStatus::kRemaining,
     "AP_HAL::Device register r/w, set_speed, periodic callback"},
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
