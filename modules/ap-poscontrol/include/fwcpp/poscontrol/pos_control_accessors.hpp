#pragma once

// CCP-027 slice 6: terrain init, stopping-point getters, offset targets, update_estimates.
// ADR-0012: AHRS NED data is injected per tick; no AP::ahrs() singleton.

#include <cstdint>

#include <fwcpp/math/postype.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/pid/ac_p_1d.hpp>
#include <fwcpp/pid/ac_p_2d.hpp>
#include <fwcpp/poscontrol/pos_control_d.hpp>
#include <fwcpp/poscontrol/pos_control_ne.hpp>

namespace fwcpp::poscontrol {

struct NedEstimates {
    math::Vector3<math::postype_t> pos_m{};
    math::Vector3f vel_ms{};
};

/// Injected AHRS readings for update_estimates (upstream AP::ahrs() calls).
struct AhrsPosControlEstimateInputs {
    bool pos_ned_valid = false;
    math::Vector3<math::postype_t> pos_ned_m{};
    bool pos_d_valid = false;
    float pos_d_m = 0.0f;
    bool vel_ned_valid = false;
    math::Vector3f vel_ned_ms{};
    bool vert_rate_d_valid = false;
    float vert_rate_d_ms = 0.0f;
    bool high_vibes = false;
};

/// Refresh NED position/velocity estimates, upstream AC_PosControl::update_estimates.
[[nodiscard]] inline NedEstimates update_estimates(const AhrsPosControlEstimateInputs& ahrs,
                                                   NedEstimates prior = {}) {
    NedEstimates out = prior;
    if (ahrs.pos_ned_valid) {
        out.pos_m = ahrs.pos_ned_m;
    } else if (ahrs.pos_d_valid) {
        out.pos_m.z = math::postype_t{ahrs.pos_d_m};
    }

    if (ahrs.vel_ned_valid && !ahrs.high_vibes) {
        out.vel_ms = ahrs.vel_ned_ms;
    } else if (ahrs.vert_rate_d_valid) {
        out.vel_ms.z = ahrs.vert_rate_d_ms;
    }
    return out;
}

[[nodiscard]] inline DTerrain init_terrain() { return DTerrain{}; }

/// Set terrain height and align desired altitude, upstream init_pos_terrain_D_m.
inline void init_pos_terrain_d_m(PosControlD& d, DTerrain& terrain, float pos_terrain_d_m) {
    d.pos_desired_m -= (math::postype_t{pos_terrain_d_m} - terrain.pos_m);
    terrain.pos_m = math::postype_t{pos_terrain_d_m};
    terrain.vel_ms = 0.0f;
    terrain.accel_mss = 0.0f;
}

inline void init_pos_terrain_u_cm(PosControlD& d, DTerrain& terrain, float pos_terrain_u_cm) {
    init_pos_terrain_d_m(d, terrain, -pos_terrain_u_cm * 0.01f);
}

[[nodiscard]] inline math::Vector2<math::postype_t> get_stopping_point_ne_m(
    const NedEstimates& estimates, const NeOffsets& offsets, float kp, const NeLimits& limits) {
    return stopping_point_ne(
        math::Vector2<math::postype_t>{estimates.pos_m.x, estimates.pos_m.y},
        offsets.pos_m,
        math::Vector2f{estimates.vel_ms.x, estimates.vel_ms.y}, offsets.vel_ms, kp, limits);
}

[[nodiscard]] inline math::postype_t get_stopping_point_d_m(const NedEstimates& estimates,
                                                            const DOffsets& offsets, float kp,
                                                            const DLimits& limits) {
    return stopping_point_d(estimates.pos_m.z, offsets.pos_m, estimates.vel_ms.z, offsets.vel_ms,
                            kp, limits.accel_max_d_mss);
}

inline void set_posvelaccel_offset_target_ne_m(NeOffsetState& offsets,
                                               math::Vector2<math::postype_t> pos_target_ne_m,
                                               math::Vector2f vel_target_ne_ms,
                                               math::Vector2f accel_target_ne_mss,
                                               std::uint32_t now_ms) {
    offsets.target.pos_m = pos_target_ne_m;
    offsets.target.vel_ms = vel_target_ne_ms;
    offsets.target.accel_mss = accel_target_ne_mss;
    offsets.target_ms = now_ms;
}

inline void set_posvelaccel_offset_target_d_m(DOffsetState& offsets, float pos_target_d_m,
                                              float vel_target_d_ms, float accel_target_d_mss,
                                              std::uint32_t now_ms) {
    offsets.target.pos_m = math::postype_t{pos_target_d_m};
    offsets.target.vel_ms = vel_target_d_ms;
    offsets.target.accel_mss = accel_target_d_mss;
    offsets.target_ms = now_ms;
}

[[nodiscard]] inline math::Vector3<math::postype_t> get_pos_offset_ned_m(const NeOffsetState& ne,
                                                                         const DOffsetState& d) {
    return math::Vector3<math::postype_t>{ne.current.pos_m.x, ne.current.pos_m.y, d.current.pos_m};
}

[[nodiscard]] inline math::Vector3f get_vel_offset_ned_ms(const NeOffsetState& ne,
                                                            const DOffsetState& d) {
    return math::Vector3f{ne.current.vel_ms.x, ne.current.vel_ms.y, d.current.vel_ms};
}

[[nodiscard]] inline math::Vector3f get_accel_offset_ned_mss(const NeOffsetState& ne,
                                                               const DOffsetState& d) {
    return math::Vector3f{ne.current.accel_mss.x, ne.current.accel_mss.y, d.current.accel_mss};
}

inline void set_pos_offset_d_m(DOffsetState& d, float pos_offset_d_m) {
    d.current.pos_m = math::postype_t{pos_offset_d_m};
}

[[nodiscard]] inline float get_pos_offset_u_m(const DOffsetState& d) {
    return -static_cast<float>(d.current.pos_m);
}

[[nodiscard]] inline float get_vel_offset_d_ms(const DOffsetState& d) { return d.current.vel_ms; }

[[nodiscard]] inline float get_accel_offset_d_mss(const DOffsetState& d) {
    return d.current.accel_mss;
}

}  // namespace fwcpp::poscontrol
