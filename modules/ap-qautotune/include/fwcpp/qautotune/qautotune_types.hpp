#pragma once

#include <cstdint>

namespace fwcpp::qautotune {

enum class HalBoard : std::uint8_t {
    kUnknown = 0,
    kSitl = 3,
};

struct QAutotuneEnableInputs {
    bool hal_quadplane_enabled{true};
    HalBoard board{HalBoard::kSitl};
};

}  // namespace fwcpp::qautotune
