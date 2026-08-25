// Location::get_bearing - the one Location method with a bare M_PI
// literal. Compiled under fwcpp_upstream_flags, same pattern as
// scalar.cpp's wrap_* family and vector2.cpp's angle().

#include <fwcpp/location.hpp>

namespace fwcpp {

float Location::get_bearing(const Location& loc2) const {
    const std::int32_t off_x = diff_longitude(loc2.lng, lng);
    const std::int32_t off_y = static_cast<std::int32_t>(
        (loc2.lat - lat) / loc2.longitude_scale((lat + loc2.lat) / 2));
    const double pi = math::pi_constant();
    float bearing = static_cast<float>(pi * 0.5) + std::atan2(static_cast<float>(-off_y), static_cast<float>(off_x));
    if (bearing < 0.0f) {
        bearing += static_cast<float>(2.0 * pi);
    }
    return bearing;
}

} // namespace fwcpp
