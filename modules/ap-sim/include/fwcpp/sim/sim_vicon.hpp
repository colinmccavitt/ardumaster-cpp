#pragma once

// Port of libraries/SITL/SIM_Vicon.h/.cpp observation math. MAVLink
// VISION_POSITION_ESTIMATE / ODOMETRY encode is omitted (no GCS_MAVLink);
// glitch, yaw, offset, gaussian noise, delay, and position-delta are original.

#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <fwcpp/location.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/quaternion.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/sim/sim_adsb.hpp>
#include <fwcpp/sim/sim_serial_device.hpp>

namespace fwcpp::sim {

struct ViconParms {
    math::Vector3f pos_offset{};
    math::Vector3f glitch{};
    std::int8_t fail = 0;
    std::int16_t yaw = 0;
    float yaw_error = 0;
    std::uint8_t type_mask = 3;
    math::Vector3f vel_glitch{};
    float pos_stddev = 0;
    float vel_stddev = 0;
    std::uint16_t rate_hz = 50;
    std::int8_t quality = 50;
};

struct ViconVehicleState {
    float rollDeg = 0;
    float pitchDeg = 0;
    float yawDeg = 0;
    float rollRate = 0;
    float pitchRate = 0;
    float yawRate = 0;
};

enum class ViconTypeMask : std::uint8_t {
    VISION_POSITION_ESTIMATE = (1 << 0),
    VISION_SPEED_ESTIMATE = (1 << 1),
    VICON_POSITION_ESTIMATE = (1 << 2),
    VISION_POSITION_DELTA = (1 << 3),
    ODOMETRY = (1 << 4),
};

struct ViconObservation {
    bool valid = false;
    std::uint64_t usec = 0;
    std::uint64_t time_delta_usec = 0;
    math::Vector3d pos{};
    math::Vector3f vel{};
    math::Vector3f vel_frd{};
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    math::Vector3f angle_delta{};
    math::Vector3f position_delta{};
    float pose_cov[21]{};
    float vel_cov[9]{};
    std::int8_t quality = 0;
};

class Vicon : public SerialDevice {
public:
    ViconParms parms{};
    ViconVehicleState vehicle_state{};
    ViconObservation last_obs{};

    bool should_send(ViconTypeMask type_mask) const {
        return ((static_cast<std::uint8_t>(type_mask) & parms.type_mask) > 0);
    }

    void update(const Location& loc, const math::Vector3d& position, const math::Vector3f& velocity,
                const math::Quaternion& attitude, std::uint64_t now_us) {
        (void)loc;
        update_vicon_position_estimate(position, velocity, attitude, now_us);
    }

private:
    std::uint64_t last_observation_usec = 0;
    std::uint64_t time_offset_us = 0;
    math::Quaternion _attitude_prev{};
    math::Vector3d _position_prev{};

    void update_vicon_position_estimate(const math::Vector3d& position, const math::Vector3f& velocity,
                                        const math::Quaternion& attitude, std::uint64_t now_us) {
        last_obs = {};
        if (time_offset_us == 0) {
            time_offset_us = (static_cast<unsigned>(std::rand()) % 7000) * 1000000ULL;
        }
        if (parms.rate_hz == 0) {
            return;
        }
        const std::uint64_t vicon_interval_us = 1000000UL / parms.rate_hz;
        if (last_observation_usec != 0 && now_us - last_observation_usec < vicon_interval_us) {
            return;
        }
        if (parms.fail != 0) {
            return;
        }

        float roll;
        float pitch;
        float yaw;
        attitude.to_euler(roll, pitch, yaw);

        const math::Vector3f& pos_offset = parms.pos_offset;
        math::Matrix3f rot;
        rot.from_euler(math::radians(vehicle_state.rollDeg), math::radians(vehicle_state.pitchDeg),
                       math::radians(vehicle_state.yawDeg));
        math::Vector3f pos_offset_ef = rot * pos_offset;

        math::Vector3d pos_corrected = position + (pos_offset_ef + parms.glitch).todouble();
        pos_corrected += math::Vector3d(aircraft_rand_normal(0, parms.pos_stddev), aircraft_rand_normal(0, parms.pos_stddev),
                                        aircraft_rand_normal(0, parms.pos_stddev));

        math::Vector3f gyro(math::radians(vehicle_state.rollRate), math::radians(vehicle_state.pitchRate),
                            math::radians(vehicle_state.yawRate));
        math::Vector3f vel_rel_offset_bf = gyro % pos_offset;
        math::Vector3f vel_rel_offset_ef = rot * vel_rel_offset_bf;
        math::Vector3f vel_corrected = velocity + vel_rel_offset_ef + parms.vel_glitch;

        const std::int16_t vicon_yaw_deg = parms.yaw;
        if (vicon_yaw_deg != 0) {
            const float vicon_yaw_rad = math::radians(static_cast<float>(vicon_yaw_deg));
            yaw = math::wrap_PI(yaw - vicon_yaw_rad);
            math::Matrix3d vicon_yaw_rot;
            vicon_yaw_rot.from_euler(0.0, 0.0, static_cast<double>(-vicon_yaw_rad));
            pos_corrected = vicon_yaw_rot * pos_corrected;
            vel_corrected = vicon_yaw_rot.tofloat() * vel_corrected;
        }

        vel_corrected += math::Vector3f(static_cast<float>(aircraft_rand_normal(0, parms.vel_stddev)),
                                        static_cast<float>(aircraft_rand_normal(0, parms.vel_stddev)),
                                        static_cast<float>(aircraft_rand_normal(0, parms.vel_stddev)));

        yaw = math::wrap_PI(yaw + math::radians(parms.yaw_error));

        float pose_cov[21];
        std::memset(pose_cov, 0, sizeof(pose_cov));
        const float pos_variance = parms.pos_stddev * parms.pos_stddev;
        pose_cov[0] = pos_variance;
        pose_cov[6] = pos_variance;
        pose_cov[11] = pos_variance;

        const std::uint64_t time_delta = (last_observation_usec == 0) ? 0 : now_us - last_observation_usec;

        math::Quaternion attitude_curr;
        attitude_curr.from_euler(roll, pitch, yaw);
        attitude_curr.invert();

        math::Quaternion attitude_curr_prev = attitude_curr * _attitude_prev.inverse();

        math::Matrix3f body_ned_m;
        attitude_curr.rotation_matrix(body_ned_m);
        math::Vector3f pos_delta = body_ned_m * (pos_corrected - _position_prev).tofloat();

        last_obs.valid = true;
        last_obs.usec = now_us + time_offset_us;
        last_obs.time_delta_usec = time_delta;
        last_obs.pos = pos_corrected;
        last_obs.vel = vel_corrected;
        last_obs.vel_frd = attitude.inverse() * vel_corrected;
        last_obs.roll = roll;
        last_obs.pitch = pitch;
        last_obs.yaw = yaw;
        last_obs.angle_delta = math::Vector3f(attitude_curr_prev.get_euler_roll(), attitude_curr_prev.get_euler_pitch(),
                                              attitude_curr_prev.get_euler_yaw());
        last_obs.position_delta = pos_delta;
        std::memcpy(last_obs.pose_cov, pose_cov, sizeof(pose_cov));
        const float vel_variance = parms.vel_stddev * parms.vel_stddev;
        last_obs.vel_cov[0] = vel_variance;
        last_obs.vel_cov[4] = vel_variance;
        last_obs.vel_cov[8] = vel_variance;
        last_obs.quality = parms.quality;
        if (last_obs.quality < -1) {
            last_obs.quality = -1;
        }
        if (last_obs.quality > 100) {
            last_obs.quality = 100;
        }

        last_observation_usec = now_us;
        _position_prev = pos_corrected;
        _attitude_prev = attitude_curr;
    }
};

}  // namespace fwcpp::sim
