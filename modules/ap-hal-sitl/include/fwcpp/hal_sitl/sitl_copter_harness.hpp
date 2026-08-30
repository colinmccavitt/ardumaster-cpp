#pragma once

// CCP-043: SitlCopterHarness — Copter analogue of SitlHarness (CPP-084).
// CCP-045: motor PWM → SimMulticopter Frame/Motor plant (the previously
// OOS motor→aero path). No longer uses SimPlane as the quad model.
//
// Upstream ROLE: AP_HAL_SITL SITL_State sensor synthesis for ArduCopter.
// Not a port of AP_HAL_SITL source (ADR-0012). Mirrors SitlHarness:
// sensors from sim truth, vehicle tick, then servo/PWM feedback into the
// plant. Copter plant is SimMulticopter (SIM_Multicopter Frame/Motor),
// not SimPlane.

#include <cstddef>
#include <cstdint>

#include <fwcpp/compass/compass.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/mode_stabilize.hpp>
#include <fwcpp/location.hpp>
#include <fwcpp/sim/sim_baro.hpp>
#include <fwcpp/sim/sim_gps.hpp>
#include <fwcpp/sim/sim_motor.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

namespace fwcpp::hal_sitl {

class SitlCopterHarness {
public:
    SitlCopterHarness(copter::LeftoverCopter& copter, sim::SimMulticopter& sim)
        : copter_(copter), sim_(sim) {}

    // Synthesize gyro/accel/baro/GPS/compass from sim_ into leftover
    // sensor buffers, inject arm/spool/attitude-hold smoke flags, tick
    // leftover Copter, then feed motor_pwm into SimMulticopter::update
    // (Frame/Motor mixing + Aircraft dynamics). Matches SitlHarness
    // sensors-then-plant order.
    void step(float dt) {
        copter_.loop_dt = dt;
        copter_.gyro_buffer = sim_.gyro;
        copter_.accel_buffer = sim_.accel_body;
        copter_.gyro_injected = true;
        copter_.accel_injected = true;

        sim_.update_position();
        sim_.update_mag_field_bf();
        const auto baro = sim::sitl_baro_from_aircraft(sim_);
        copter_.baro_altitude_m = baro.altitude_amsl_m - sim_.home.alt * 0.01f;
        copter_.baro_injected = true;

        const auto gps = sim::sitl_gps_from_aircraft(sim_);
        copter_.gps_lat = gps.lat;
        copter_.gps_lng = gps.lng;
        copter_.gps_injected = true;

        copter_.compass_field_bf = sim_.get_mag_field_bf();
        copter_.compass_injected = true;

        copter_.motors_armed_injected = true;
        if (copter_.motors_armed) {
            copter_.spool_state = copter::SpoolState::THROTTLE_UNLIMITED;
            copter_.attitude_hold = true;
        } else {
            copter_.spool_state = copter::SpoolState::SHUT_DOWN;
            copter_.attitude_hold = false;
        }
        copter_.spool_injected = true;
        copter_.attitude_hold_injected = true;

        copter::leftover_copter_tick(copter_);

        sim::SitlInput input;
        for (std::uint8_t i = 0; i < sim::kSitlServoChannels; ++i) {
            input.servos[i] = copter_.motor_pwm[i];
        }
        sim_.update(input, dt);
    }

    [[nodiscard]] copter::LeftoverCopter& copter() { return copter_; }
    [[nodiscard]] sim::SimMulticopter& sim() { return sim_; }
    // Alias kept so CCP-043 tests that called sim_plane() still compile
    // if updated to sim(); new name is sim().
    [[nodiscard]] sim::SimMulticopter& sim_plane() { return sim_; }
    [[nodiscard]] const compass::Compass& compass() const { return compass_; }
    [[nodiscard]] std::uint32_t tick_count() const { return copter_.tick_count; }

private:
    copter::LeftoverCopter& copter_;
    sim::SimMulticopter& sim_;
    compass::Compass compass_{};
};

namespace sitl_copter {

enum class PortStatus : std::uint8_t {
    kOnMain = 0,
    kThisSlice = 1,
    kRemaining = 2,
    kOutOfScope = 3,
};

struct PortItem {
    const char* name;
    PortStatus status;
    const char* note;
};

inline constexpr PortItem kCompleteness[] = {
    {"leftover catalog", PortStatus::kThisSlice, "this table"},
    {"SitlCopterHarness scaffold", PortStatus::kThisSlice,
     "refs LeftoverCopter + SimMulticopter; step sensor inject + leftover_copter_tick"},
    {"leftover_copter_tick", PortStatus::kThisSlice,
     "CCP-064: leftover_copter_loop = Copter::loop leftover scheduler + update_flight_mode"},
    {"leftover_copter_loop", PortStatus::kThisSlice,
     "FAST_TASK rate/motors/AHRS/inertia/ekf/mode + SCHED_TASK rc/throttle/nav"},
    {"gyro/accel synthesis", PortStatus::kThisSlice,
     "SimMulticopter::gyro / accel_body → leftover buffers + inject flags"},
    {"baro synthesis", PortStatus::kThisSlice,
     "SimMulticopter altitude (-position.z) → leftover baro_altitude_m + flag"},
    {"GPS synthesis", PortStatus::kThisSlice,
     "home lat/lng + SimMulticopter NED north/east → leftover gps_lat/gps_lng + flag"},
    {"compass synthesis", PortStatus::kThisSlice,
     "Compass earth field via SimMulticopter::dcm → compass_field_bf + flag"},
    {"closed-loop arm/spool/hold", PortStatus::kThisSlice,
     "step injects motors_armed + spool + attitude_hold smoke"},
    {"motor PWM to SimMulticopter", PortStatus::kThisSlice,
     "CCP-045: leftover motor_pwm[] → SitlInput.servos → Frame/Motor plant"},
    {"SitlHarness Plane path (CPP-084)", PortStatus::kOnMain,
     "sitl_harness.hpp; Plane+SimPlane closed loop"},
    {"CCP-035 update_flight_mode", PortStatus::kOnMain,
     "update_flight_mode.hpp; harness wires via leftover_copter_tick"},
    {"SIM_Multicopter Frame/Motor plant", PortStatus::kOnMain,
     "sim_multicopter.hpp / sim_frame.hpp / sim_motor.hpp (CCP-045)"},
    {"AP:: / HAL SITL singletons", PortStatus::kOutOfScope, "ADR-0012 explicit refs"},
    {"Rust copter-sitl", PortStatus::kOutOfScope, "Do not copy Rust"},
};

[[nodiscard]] inline constexpr std::size_t completeness_size() {
    return sizeof(kCompleteness) / sizeof(kCompleteness[0]);
}

[[nodiscard]] inline constexpr std::size_t count_status(PortStatus status) {
    std::size_t n = 0;
    for (const auto& item : kCompleteness) {
        if (item.status == status) {
            ++n;
        }
    }
    return n;
}

[[nodiscard]] inline constexpr bool completeness_has(const char* name, PortStatus status) {
    for (const auto& item : kCompleteness) {
        const char* a = item.name;
        const char* b = name;
        while (*a != '\0' && *b != '\0') {
            if (*a != *b) {
                break;
            }
            ++a;
            ++b;
        }
        if (*a == '\0' && *b == '\0' && item.status == status) {
            return true;
        }
    }
    return false;
}

[[nodiscard]] inline constexpr std::size_t on_main_count() {
    return count_status(PortStatus::kOnMain);
}
[[nodiscard]] inline constexpr std::size_t this_slice_count() {
    return count_status(PortStatus::kThisSlice);
}
[[nodiscard]] inline constexpr std::size_t remaining_count() {
    return count_status(PortStatus::kRemaining);
}
[[nodiscard]] inline constexpr std::size_t out_of_scope_count() {
    return count_status(PortStatus::kOutOfScope);
}

}  // namespace sitl_copter

}  // namespace fwcpp::hal_sitl
