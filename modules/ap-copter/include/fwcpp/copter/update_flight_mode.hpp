#pragma once

// Copter::update_flight_mode leftover. Upstream ArduCopter/mode.cpp
// ~497-508. No AHRS / attitude_control / pos_control / GCS objects —
// inject land_complete, move_vehicle_on_ekf_reset, and Mode*.
//
// surface_tracking.invalidate_for_logging is always recorded this
// slice (AP_RANGEFINDER_ENABLED compile-time remaining). Gain tables
// stay skipped. Mode::move_vehicle_on_ekf_reset virtual is CCP-037;
// inject the bool (default false matching Mode base).

#include <cstdint>

#include <fwcpp/copter/mode.hpp>

namespace fwcpp::copter {

// AC_PosControl::EKFResetMethod names (AC_PosControl.h ~584-587).
enum class EKFResetMethod : std::uint8_t {
    MoveTarget = 0,
    MoveVehicle = 1,
};

struct UpdateFlightModeInputs {
    bool land_complete{false};
    bool move_vehicle_on_ekf_reset{false};
    Mode* current{nullptr};
};

struct UpdateFlightModeEffects {
    bool invalidate_surface_tracking{false};
    bool landed_gain_reduction{false};
    EKFResetMethod ekf_reset_method{EKFResetMethod::MoveTarget};
    bool run_called{false};
};

[[nodiscard]] inline UpdateFlightModeEffects update_flight_mode(const UpdateFlightModeInputs& in) {
    UpdateFlightModeEffects fx{};
    fx.invalidate_surface_tracking = true;
    fx.landed_gain_reduction = in.land_complete;
    fx.ekf_reset_method = in.move_vehicle_on_ekf_reset ? EKFResetMethod::MoveVehicle
                                                       : EKFResetMethod::MoveTarget;
    if (in.current != nullptr) {
        in.current->run();
        fx.run_called = true;
    }
    return fx;
}

}  // namespace fwcpp::copter
