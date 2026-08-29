#pragma once

#include <fwcpp/math/scalar.hpp>

#include <cmath>
#include <cstdint>

namespace fwcpp::qrtl {

[[nodiscard]] inline float qrtl_vtol_return_radius_m(float loiter_radius_m, float rtl_radius_m) {
    const float loiter = std::fabs(loiter_radius_m);
    const float rtl = std::fabs(rtl_radius_m);
    const float larger = loiter > rtl ? loiter : rtl;
    return larger * 1.5F;
}

[[nodiscard]] inline float qrtl_min_climb_m(float qrtl_alt_min_m, float land_final_alt_m,
                                              float qrtl_alt_m) {
    return fwcpp::math::constrain_value(qrtl_alt_min_m, land_final_alt_m, qrtl_alt_m);
}

[[nodiscard]] inline float qrtl_climb_cone_target_alt_m(float qrtl_alt_m, float dist_m,
                                                        float radius_m, float min_climb_m) {
    const float denom = radius_m > dist_m ? radius_m : dist_m;
    const float cone = denom > 0.0F ? qrtl_alt_m * (dist_m / denom) : 0.0F;
    return cone > min_climb_m ? cone : min_climb_m;
}

enum class QrtlDestination : std::uint8_t {
    kHome = 0,
    kRally = 1,
};

[[nodiscard]] inline QrtlDestination calc_best_rally_or_home(float home_dist_m,
                                                             float rally_dist_m,
                                                             bool rally_incl_home,
                                                             bool has_rally) {
    if (has_rally && (!rally_incl_home || rally_dist_m < home_dist_m)) {
        return QrtlDestination::kRally;
    }
    return QrtlDestination::kHome;
}

[[nodiscard]] inline float qrtl_destination_dist_m(QrtlDestination dest, float home_dist_m,
                                                 float rally_dist_m) {
    return dest == QrtlDestination::kRally ? rally_dist_m : home_dist_m;
}

[[nodiscard]] inline bool is_positive(float value) { return value > 0.0F; }

}  // namespace fwcpp::qrtl
