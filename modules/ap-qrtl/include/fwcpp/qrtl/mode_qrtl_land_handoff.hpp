#pragma once

#include <fwcpp/quadplane/quadplane_poscontrol_stub.hpp>

#include <cstdint>

namespace fwcpp::qrtl {

using fwcpp::quadplane::PositionControlState;

[[nodiscard]] inline bool qrtl_copy_home_alt(PositionControlState state) {
    return static_cast<std::uint8_t>(state) >
           static_cast<std::uint8_t>(PositionControlState::kPosition2);
}

[[nodiscard]] inline bool qrtl_should_verify_land(PositionControlState state) {
    return static_cast<std::uint8_t>(state) >=
           static_cast<std::uint8_t>(PositionControlState::kPosition2);
}

[[nodiscard]] inline bool qrtl_stick_mixing_fbw(PositionControlState state) {
    return state == PositionControlState::kAirbrake ||
           state == PositionControlState::kApproach;
}

struct QrtlLandHandoff {
    bool copy_home_alt{false};
    bool verify_vtol_land{false};
    bool stick_mixing_fbw{false};
};

[[nodiscard]] inline QrtlLandHandoff qrtl_land_handoff(PositionControlState state) {
    QrtlLandHandoff out{};
    out.copy_home_alt = qrtl_copy_home_alt(state);
    out.verify_vtol_land = qrtl_should_verify_land(state);
    out.stick_mixing_fbw = qrtl_stick_mixing_fbw(state);
    return out;
}

}  // namespace fwcpp::qrtl
