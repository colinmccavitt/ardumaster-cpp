#pragma once

// Port of libraries/SITL/SIM_Precland.h/.cpp. AP_Param/GCS dropped; fields
// are plain members. Ship-follow (SIM_SHIP) is wired when SIM_Ship is
// constructed by the caller. Cylinder/cone/sphere radiance is original.

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <fwcpp/location.hpp>
#include <fwcpp/math/rotations.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::sim {

class SIM_Precland {
public:
    enum class Option : std::uint8_t {
        ENABLE_TARGET_DISTANCE = (1U << 0),
    };
    enum PreclandType {
        PRECLAND_TYPE_CYLINDER = 0,
        PRECLAND_TYPE_CONE = 1,
        PRECLAND_TYPE_SPHERE = 2,
    };

    std::int8_t _enable = 0;
    float _device_lat = 0;
    float _device_lon = 0;
    float _device_height = 0;
    std::int16_t _orient_yaw = 0;
    std::int8_t _type = PRECLAND_TYPE_CYLINDER;
    std::int32_t _rate = 100;
    float _alt_limit = 15;
    float _dist_limit = 10;
    std::int8_t _orient = static_cast<std::int8_t>(math::Rotation::PITCH_90);
    std::int8_t _ship = 0;
    std::uint8_t _options = 0;
    bool _over_precland_base = false;

    void update(const Location& loc, std::uint32_t now_ms, const Location* ship_loc = nullptr) {
        if (!_enable) {
            _healthy = false;
            return;
        }
        if (math::is_zero(_alt_limit) || _dist_limit < 1.0f) {
            _healthy = false;
            return;
        }

        Location device_center(static_cast<std::int32_t>(_device_lat * 1.0e7f),
                               static_cast<std::int32_t>(_device_lon * 1.0e7f),
                               static_cast<std::int32_t>(_device_height * 100), Location::AltFrame::ABOVE_ORIGIN);

        if (device_center.lat == 0 && device_center.lng == 0 && device_center.alt == 0) {
            _healthy = false;
            return;
        }

        if (_ship == 1 && ship_loc != nullptr && !ship_loc->is_zero()) {
            device_center = *ship_loc;
        }

        math::Vector3d axis{1, 0, 0};
        axis.rotate(static_cast<math::Rotation>(_orient));

        const math::Vector3p ned = device_center.get_distance_NED_postype(loc);
        math::Vector3d position_wrt_device{ned.x, ned.y, ned.z};

        math::Vector3d projection_on_axis = position_wrt_device.projected(axis);
        const float longitudinal_dist = static_cast<float>(projection_on_axis.length());
        const float lateral_distance =
            math::safe_sqrt(std::max(0.0f, static_cast<float>(position_wrt_device.length_squared() -
                                                              static_cast<double>(longitudinal_dist) * longitudinal_dist)));

        if (projection_on_axis.dot(axis) <= 0 || longitudinal_dist > _alt_limit) {
            _healthy = false;
            return;
        }

        if (now_ms - _last_update_ms < 1000.0f * (1.0f / _rate)) {
            return;
        }
        _last_update_ms = now_ms;

        switch (_type) {
            case PRECLAND_TYPE_CONE: {
                const float lateral_limit = longitudinal_dist * _dist_limit / _alt_limit;
                if (lateral_distance > lateral_limit) {
                    _healthy = false;
                    return;
                }
                break;
            }
            case PRECLAND_TYPE_SPHERE: {
                if (position_wrt_device.length() > _dist_limit) {
                    _healthy = false;
                    return;
                }
                break;
            }
            default:
            case PRECLAND_TYPE_CYLINDER: {
                if (lateral_distance > _dist_limit) {
                    _healthy = false;
                    return;
                }
                break;
            }
        }
        _target_pos = position_wrt_device;
        _healthy = true;
    }

    [[nodiscard]] bool healthy() const { return _healthy; }
    [[nodiscard]] std::uint32_t last_update_ms() const { return _last_update_ms; }
    [[nodiscard]] const math::Vector3d& get_target_position() const { return _target_pos; }
    [[nodiscard]] bool is_enabled() const { return static_cast<bool>(_enable); }
    [[nodiscard]] bool option_enabled(Option option) const { return (_options & static_cast<std::uint8_t>(option)) != 0; }

    void set_default_location(float lat, float lon, std::int16_t yaw) {
        if (math::is_zero(_device_lat) && math::is_zero(_device_lon)) {
            _device_lat = lat;
            _device_lon = lon;
            _orient_yaw = yaw;
        }
    }

private:
    std::uint32_t _last_update_ms = 0;
    bool _healthy = false;
    math::Vector3d _target_pos{};
};

}  // namespace fwcpp::sim
