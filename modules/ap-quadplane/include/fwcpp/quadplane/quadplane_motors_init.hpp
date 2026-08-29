#pragma once

#include <cstdint>

namespace fwcpp::quadplane {

/// Upstream AP_Motors::init(frame_class, frame_type) arguments recorded at setup.
struct MotorsInitParams {
    std::uint8_t frame_class{0};
    std::uint8_t frame_type{0};
};

/// Stub for motors->init after allocation — no ap-motors mixing in this slice.
[[nodiscard]] inline constexpr MotorsInitParams make_motors_init_params(std::uint8_t frame_class,
                                                                          std::uint8_t frame_type) {
    return MotorsInitParams{frame_class, frame_type};
}

}  // namespace fwcpp::quadplane
