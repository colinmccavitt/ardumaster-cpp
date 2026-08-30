#pragma once

// Port of libraries/SITL/SIM_Ship.h/.cpp. Circular-path kinematics,
// get_location and get_ground_speed_adjustment. MAVLink socket report
// (send_report) is omitted — same GCS_MAVLink skip as SIM_RF_MAVLink.

#include <cmath>
#include <cstdint>

#include <fwcpp/location.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::sim {

class ShipSim;

class Ship {
public:
    friend class ShipSim;
    math::Vector2f position{};
    float heading_deg = 0;
    float yaw_rate = 0;
    float speed = 0;
    ShipSim* sim = nullptr;
    void update(float delta_t);
};

class ShipSim {
public:
    friend class Ship;
    std::int8_t enable = 0;
    float speed = 3;
    float path_size = 1000;
    float deck_size = 10;
    std::int8_t sys_id = 17;
    math::Vector3f offset{};
    Location home{};
    bool initialised = false;
    Ship ship{};
    std::uint32_t last_update_us = 0;

    void update(float delta_t, std::uint32_t now_us, const Location& sitl_home) {
        if (!enable) {
            return;
        }
        if (!initialised) {
            home = sitl_home;
            if (home.lat == 0 && home.lng == 0) {
                return;
            }
            home.offset(offset.x, offset.y);
            home.alt -= static_cast<std::int32_t>(offset.z * 100);
            initialised = true;
            ship.sim = this;
            last_update_us = now_us;
        }
        last_update_us = now_us;
        ship.update(delta_t);
    }

    bool get_location(Location& loc) const {
        if (!enable) {
            return false;
        }
        loc = home;
        loc.offset(ship.position.x, ship.position.y);
        return true;
    }

    math::Vector2f get_ground_speed_adjustment(const Location& loc, float& yaw_rate) {
        Location shiploc;
        if (!get_location(shiploc)) {
            yaw_rate = 0;
            return math::Vector2f(0, 0);
        }
        if (loc.get_distance(shiploc) > deck_size) {
            yaw_rate = 0;
            return math::Vector2f(0, 0);
        }
        Location center = shiploc;
        const float path_radius = path_size * 0.5f;
        center.offset_bearing(ship.heading_deg + (ship.yaw_rate > 0 ? 90.0f : -90.0f), path_radius);
        const float p = center.get_distance(loc) / path_radius;
        const float scaled_speed = ship.speed * p;
        const float bearing1 = center.get_bearing(loc);
        const float bearing2 = center.get_bearing(shiploc);
        const float heading = ship.heading_deg + math::degrees(bearing1 - bearing2);
        math::Vector2f vel(scaled_speed, 0);
        vel.rotate(math::radians(heading));
        yaw_rate = ship.yaw_rate;
        return vel;
    }
};

inline void Ship::update(float delta_t) {
    const float max_accel = 3.0f;
    const float dspeed_max = max_accel * delta_t;
    speed = math::constrain_value(sim->speed, speed - dspeed_max, speed + dspeed_max);
    float circumference = static_cast<float>(M_PI) * sim->path_size;
    float dist = delta_t * speed;
    float dangle = (dist / circumference) * 360.0f;
    if (delta_t > 0) {
        yaw_rate = math::radians(dangle) / delta_t;
    }
    heading_deg += dangle;
    heading_deg = math::wrap_360(heading_deg);
    math::Vector2f dpos(dist, 0);
    dpos.rotate(math::radians(heading_deg));
    position += dpos;
}

}  // namespace fwcpp::sim
