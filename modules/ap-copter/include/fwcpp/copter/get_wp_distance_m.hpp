#pragma once

// Copter::get_wp_distance_m leftover. Upstream ArduCopter/Copter.cpp
// ~944-949. Inject flightmode->wp_distance_m() as a float (no Mode*
// object). Always write the injected distance to the out-param and
// return true. No GCS object.
//
// Do not port Copter::get_wp_bearing_deg, get_wp_crosstrack_error_m,
// or Copter::update_auto_armed.

namespace fwcpp::copter {

struct GetWpDistanceMInputs {
    float wp_distance_m{0.0f};
};

[[nodiscard]] inline bool get_wp_distance_m(float& distance,
                                            const GetWpDistanceMInputs& in = {}) {
    distance = in.wp_distance_m;
    return true;
}

}  // namespace fwcpp::copter
