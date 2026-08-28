#pragma once

// CPP-028 leftover-complete catalog + shared production helpers for the
// disclosed AhrsDcm leftovers (status/arming, GPS groundspeed, relative
// position, airspeed-sensor accessors).
//
// DCM slices 1-3 (attitude + yaw/roll/pitch drift correction) and the
// NavEKF3 + AhrsBackend arc (CPP-052, CPP-056-081) are already on main.
// This header does not redo them. It names every leftover surface from
// the 2026-08-28 ticket note and records whether this slice stubbed it
// or formally parked it as out-of-scope.
//
// Remaining is empty of in-scope work: the only unfinished upstream
// surfaces are formally out-of-scope (no DCM wind estimator in this
// port; no GCS/MAVLink/logging). SitlHarness deliberately leaves
// wind_estimate at zero - do NOT feed SimPlane truth as an oracle.
//
// Helpers below are the real math both AhrsDcm and EkfCoreBackend call
// (ADR-0012 explicit inputs, no singletons, no exceptions).

#include <cstddef>
#include <cstdint>

#include <fwcpp/location.hpp>
#include <fwcpp/math/vector2.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::ahrs {

enum class PortStatus {
    OnMain,      // landed before this leftover closer; do not redo
    ThisSlice,   // production stub added by this CPP-028 closer
    Remaining,   // in-scope leftover, not yet stubbed
    OutOfScope,  // formally excluded (no subsystem in this port)
};

struct AhrsPortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr AhrsPortItem kAhrsCompleteness[] = {
    {"DCM slice 1 gyro integration", PortStatus::OnMain,
     "matrix_update/normalize/check_matrix/renorm/reset - commit b0e2e6d"},
    {"DCM slice 2 yaw drift correction", PortStatus::OnMain,
     "drift_correction_yaw/yaw_error_compass/use_compass - commit 49960ca"},
    {"DCM slice 3 roll/pitch drift correction", PortStatus::OnMain,
     "drift_correction_accel accel-vs-gravity/GPS-velocity - commit 5557df8"},
    {"NavEKF3 EkfCore", PortStatus::OnMain,
     "CPP-052 + CPP-056-077: strapdown, fusion, delay buffers, tick()"},
    {"AhrsBackend + EkfCoreBackend + Plane polymorphic ahrs", PortStatus::OnMain,
     "CPP-078-081: update_full_cycle, six getters, EkfCore adapter"},
    {"healthy / last_failure_ms", PortStatus::ThisSlice,
     "AP_AHRS_DCM::healthy: no failure in the last 5s; 4 upstream stamp sites; ga_b.is_inf() continue-only"},
    {"pre_arm_check", PortStatus::ThisSlice,
     "AP_AHRS_DCM::pre_arm_check: healthy() only; no GCS failure_msg"},
    {"groundspeed / groundspeed_vector from GPS", PortStatus::ThisSlice,
     "GPS-only path of AP_AHRS_DCM::groundspeed_vector; no ADS+wind complementary filter"},
    {"airspeed_EAS / using_airspeed_sensor", PortStatus::ThisSlice,
     "sensor EAS when healthy (CPP-082/083); else last GPS-derived TAS cache, returning false"},
    {"set_home / relative position NE+D home", PortStatus::ThisSlice,
     "Location::get_distance_NE + alt-cm NED-down; no baro fallback"},
    {"leftover-complete catalog", PortStatus::ThisSlice,
     "this table"},
    {"DCM wind estimator (estimate_wind)", PortStatus::OutOfScope,
     "no DCM wind estimator; SitlHarness leaves wind_estimate at zero; do not use SimPlane truth"},
    {"GCS / MAVLink / logging", PortStatus::OutOfScope,
     "no GCS/MAVLink in this port; send_ekf_status_report / AP_AHRS_Logging excluded"},
};

[[nodiscard]] inline constexpr std::size_t ahrs_completeness_count() {
    return sizeof(kAhrsCompleteness) / sizeof(kAhrsCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t ahrs_completeness_count(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kAhrsCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

// upstream: AP_AHRS_DCM::healthy - consider healthy if no failure for 5s.
inline constexpr std::uint32_t kHealthyFailureWindowMs = 5000;

[[nodiscard]] inline bool healthy_from_last_failure(std::uint32_t last_failure_ms, std::uint32_t now_ms) {
    return last_failure_ms == 0 || (now_ms - last_failure_ms) > kHealthyFailureWindowMs;
}

[[nodiscard]] inline bool pre_arm_check_from_healthy(bool ahrs_healthy, bool /*requires_position*/) {
    // upstream: requires_position is unused; only healthy() is checked.
    // failure_msg is GCS and is out of scope for this port.
    return ahrs_healthy;
}

[[nodiscard]] inline float groundspeed_from_gps(bool has_fix, float ground_speed_ms) {
    return has_fix ? ground_speed_ms : 0.0f;
}

[[nodiscard]] inline math::Vector2f groundspeed_vector_from_gps(bool has_fix, const math::Vector3f& velocity_ned) {
    return has_fix ? velocity_ned.xy() : math::Vector2f{};
}

// Sensor-first unconstrained EAS. Returns true when the value comes from
// a healthy airspeed sensor; false when falling back to the last
// GPS-derived TAS cache (treated as EAS at eas2tas=1, the SITL default).
inline bool airspeed_eas_from_sensor(float sensor_eas, bool sensor_healthy, float last_airspeed_tas,
                                     float& airspeed_ret) {
    if (sensor_healthy) {
        airspeed_ret = sensor_eas;
        return true;
    }
    airspeed_ret = last_airspeed_tas;
    return false;
}

[[nodiscard]] inline bool using_airspeed_from_sensor(bool sensor_healthy) { return sensor_healthy; }

inline bool relative_position_ne_home(const Location& home, bool home_is_set, const Location& current,
                                      bool have_position, math::Vector2f& pos_ne) {
    if (!home_is_set || !have_position) {
        return false;
    }
    pos_ne = home.get_distance_NE(current);
    return true;
}

inline bool relative_position_d_home(const Location& home, bool home_is_set, const Location& current,
                                     bool have_position, float& pos_d) {
    if (!home_is_set || !have_position) {
        return false;
    }
    // NED-down metres: current above home => negative. alt is centimetres.
    pos_d = static_cast<float>(home.alt - current.alt) * 0.01f;
    return true;
}

} // namespace fwcpp::ahrs
