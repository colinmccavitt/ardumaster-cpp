#pragma once

// BoardKind selects which HAL backend a bring-up class targets.
// CPP-089: enum + SITL default (slice 1). LinuxHalContext (linux_hal.hpp)
// is the compile-only Linux bundle (slice 2). ChibiOS / ESP32 backends
// remain (see hw_leftover.hpp).
//
// Upstream CONFIG_HAL_BOARD (AP_HAL_Boards.h) numeric IDs are SITL=3,
// LINUX=7, CHIBIOS=10, ESP32=12. This port is not a preprocessor board
// switch, so those values are not reproduced.

#include <cstdint>

namespace fwcpp::hal {

enum class BoardKind : std::uint8_t {
    kSitl = 0,
    kChibiOS = 1,
    kLinux = 2,
    kEsp32 = 3,
};

inline constexpr BoardKind kDefaultBoardKind = BoardKind::kSitl;

}  // namespace fwcpp::hal
