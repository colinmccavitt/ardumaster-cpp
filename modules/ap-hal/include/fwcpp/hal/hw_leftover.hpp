#pragma once

// CPP-089 slice 1 completeness catalog: BoardKind + compile-only
// Watchdog stub. SITL HalContext surfaces from CPP-025/088 stay on
// main. remaining_count() is intentionally > 0 (ChibiOS peripherals,
// Linux backend, ESP32 backend). Do not edit leftover.hpp — that
// catalog's remaining_count() stays 0 after CPP-088 WSPI.

#include <cstddef>
#include <cstdint>

namespace fwcpp::hal::hw {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct HwPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr HwPortItem kHwCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"BoardKind", PortStatus::kThisSlice,
     "kSitl / kChibiOS / kLinux / kEsp32; default kSitl"},
    {"Watchdog stub", PortStatus::kThisSlice,
     "init/pat/was_reset + save/load word span; no IWDG/WWDG/syscall"},
    {"SITL time", PortStatus::kOnMain,
     "injected now_ms/now_us; no wall-clock HAL source (CPP-025)"},
    {"SITL UART", PortStatus::kOnMain, "UartDriver in-memory ring (CPP-025)"},
    {"SITL RC", PortStatus::kOnMain, "RcInput / RcOutput (CPP-025)"},
    {"SITL GPIO", PortStatus::kOnMain, "16-pin in-memory table (CPP-088)"},
    {"SITL storage", PortStatus::kOnMain, "param RawStorage (CPP-025)"},
    {"SITL Util", PortStatus::kOnMain,
     "soft_armed, safety_switch, system_id, persistent_data (CPP-088)"},
    {"ChibiOS peripherals", PortStatus::kRemaining,
     "ChibiOS time/UART/RC/GPIO/storage on a bring-up class (not watchdog stub)"},
    {"Linux backend", PortStatus::kRemaining,
     "Linux HAL surface: time, UART, RC, GPIO, storage"},
    {"ESP32 backend", PortStatus::kRemaining,
     "ESP32 HAL surface: time, UART, RC, GPIO, storage"},
    {"ChibiOS peripheral drivers", PortStatus::kOutOfScope,
     "every ChibiOS driver beyond the HAL surface this port calls"},
    {"hwdef.dat", PortStatus::kOutOfScope, "ChibiOS hwdef pin/peripheral tables"},
    {"DMA", PortStatus::kOutOfScope, "ChibiOS DMA engine"},
    {"FATFS", PortStatus::kOutOfScope, "ChibiOS FATFS / SDMMC"},
};

[[nodiscard]] inline constexpr std::size_t hw_completeness_size() {
    return sizeof(kHwCompleteness) / sizeof(kHwCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kHwCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kHwCompleteness) {
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

}  // namespace fwcpp::hal::hw
