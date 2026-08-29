#pragma once

#include <cstdint>

namespace fwcpp::tailsitter {

enum class InputType {
    kVectoredYaw,
    kControlSurfaces,
};

enum class TiltrotorType : std::uint8_t {
    kNone = 0,
    kBicopter = 1,
};

struct SurfaceAssign {
    bool elevator{false};
    bool aileron{false};
    bool rudder{false};
    bool elevon{false};
    bool v_tail{false};
};

}  // namespace fwcpp::tailsitter
