#pragma once

// CCP-044 leftover mission (arm / takeoff / hold / land) driving CCP-045
// SimMulticopter Frame/Motor plant via SitlCopterHarness::step.
//
// The leftover body-z shortcut (leftover_multirotor_aero shoving a single
// collective force through SimPlane::update_dynamics) is gone. Collective
// leftover throttle becomes four (or N) motor PWM values mixed by
// SIM_Frame / SIM_Motor. leftover_hold_command is leftover mission
// altitude-rate damping (not AC_PosControl, not the plant).

#include <cstddef>
#include <cstdint>

#include <fwcpp/copter/land_detector.hpp>
#include <fwcpp/copter/leftover_copter.hpp>
#include <fwcpp/copter/takeoff.hpp>
#include <fwcpp/hal_sitl/sitl_copter_harness.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>
#include <fwcpp/motors/motors_matrix.hpp>
#include <fwcpp/pid/ac_p_1d.hpp>
#include <fwcpp/pid/ac_pid.hpp>
#include <fwcpp/pid/ac_pid_basic.hpp>
#include <fwcpp/poscontrol/pos_control_d.hpp>
#include <fwcpp/poscontrol/pos_control_defaults.hpp>
#include <fwcpp/sim/sim_multicopter.hpp>

namespace fwcpp::hal_sitl::copter_sitl_run {

enum class MissionPhase : std::uint8_t {
    kDisarmed = 0,
    kTakeoff = 1,
    kHold = 2,
    kLand = 3,
    kLanded = 4,
};

struct LeftoverMission {
    MissionPhase phase{MissionPhase::kDisarmed};
    float takeoff_alt_m{10.0f};
    float climb_command{0.70f};
    float land_command{0.20f};
    float hold_s{2.0f};
    float hold_elapsed_s{0.0f};
    float command{0.0f};
    copter::TakeOffState takeoff{};
};

[[nodiscard]] inline const char* mission_phase_name(MissionPhase phase) {
    switch (phase) {
    case MissionPhase::kDisarmed:
        return "DISARMED";
    case MissionPhase::kTakeoff:
        return "TAKEOFF";
    case MissionPhase::kHold:
        return "HOLD";
    case MissionPhase::kLand:
        return "LAND";
    case MissionPhase::kLanded:
        return "LANDED";
    }
    return "?";
}

// CCP-064: AC_PosControl D cascade (pos -> vel -> accel -> throttle).
// Replaces the leftover 1-line vz damper. NED +z down. throttle_hover is
// Frame::hover_command() so leftover_apply_collective PWM matches expo.
inline void leftover_init_poscontrol(copter::LeftoverCopter& copter) {
    if (copter.pos_d_inited) {
        return;
    }
    copter.p_pos_d = pid::AcP1d::with_kp(1.0f);
    copter.pid_vel_d = pid::AcPidBasic::with_gains(5.0f, 0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
    copter.pid_accel_d.set_kP(0.5f);
    copter.pid_accel_d.set_kI(1.0f);
    copter.pid_accel_d.set_imax(1.0f);
    copter.d_limits = poscontrol::d_set_max_speed_accel_m(
        copter.d_limits, poscontrol::kPoscontrolSpeedDownMs, poscontrol::kPoscontrolSpeedUpMs,
        poscontrol::kPoscontrolAccelDMss, poscontrol::kPoscontrolJerkDMsss, copter.pid_accel_d);
    copter.pos_d_inited = true;
}

[[nodiscard]] inline float leftover_poscontrol_throttle(copter::LeftoverCopter& copter,
                                                        const sim::SimMulticopter& sim,
                                                        float pos_d_target_m,
                                                        float vel_d_desired_ms) {
    leftover_init_poscontrol(copter);
    copter.pos_d.pos_desired_m = pos_d_target_m;
    copter.pos_d.vel_desired_ms = vel_d_desired_ms;
    poscontrol::DUpdateInputs inp{};
    inp.dt = copter.loop_dt > 0.0f ? copter.loop_dt : 0.0025f;
    inp.now_ms = copter.now_ms;
    inp.estimates.pos_m = sim.position.z;
    inp.estimates.vel_ms = sim.velocity_ef.z;
    inp.estimated_accel_d_mss = 0.0f;
    inp.throttle_hover = sim.hover_command();
    inp.vel_max_down_ms = copter.d_limits.vel_max_down_ms;
    if (inp.vel_max_down_ms <= 0.0f) {
        inp.vel_max_down_ms = poscontrol::kPoscontrolSpeedDownMs;
    }
    const auto out = copter.pos_d.update_controller(copter.p_pos_d, copter.pid_vel_d, copter.pid_accel_d, inp);
    copter.throttle_out = math::constrain_value(out.throttle_out, 0.0f, 1.0f);
    return copter.throttle_out;
}

// Hold: AC_PosControl D to the leftover mission altitude (not vz damper).
[[nodiscard]] inline float leftover_hold_command(copter::LeftoverCopter& copter,
                                                 const sim::SimMulticopter& sim,
                                                 float hold_alt_m) {
    return leftover_poscontrol_throttle(copter, sim, -hold_alt_m, 0.0f);
}

inline void leftover_apply_collective(copter::LeftoverCopter& copter, const sim::SimMulticopter& sim, float command,
                                      float dt = 0.0025f) {
    static motors::MotorsMatrix mixer;
    static bool inited = false;
    if (!inited) {
        mixer.setup_motors(motors::MotorsMatrix::FrameClass::Quad, motors::MotorsMatrix::FrameType::X);
        mixer.normalise_rpy_factors();
        mixer.set_throttle_thrust_max(1.0f);
        inited = true;
    }
    for (std::uint8_t i = 0; i < sim::kSitlServoChannels; ++i) {
        copter.motor_pwm[i] = 0;
    }
    if (!copter.motors_armed) {
        return;
    }
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    sim.dcm.to_euler(&roll, &pitch, &yaw);
    const float roll_in = math::constrain_value(-0.5f * roll, -1.0f, 1.0f);
    const float pitch_in = math::constrain_value(-0.5f * pitch, -1.0f, 1.0f);
    const float yaw_in = math::constrain_value(-0.2f * sim.gyro.z, -1.0f, 1.0f);
    bool lr = false, lp = false, ly = false, ll = false, lu = false;
    mixer.output_armed_stabilizing(roll_in, 0.0f, pitch_in, 0.0f, yaw_in, 0.0f, command, command, 0.0f, 1.0f, dt, lr,
                                   lp, ly, ll, lu);
    mixer.set_spool_state(motors::MotorsMatrix::SpoolState::ThrottleUnlimited);
    motors::ThrustLinParams params;
    params.curve_expo = 0.0f;
    params.spin_min = 0.0f;
    params.spin_max = 1.0f;
    mixer.output_to_motors(true, false, 0.0f, 0.0f, 0.0f, params, dt, 1000, 2000);
    const auto& frame = sim.frame();
    for (std::uint8_t i = 0; i < frame.num_motors; ++i) {
        copter.motor_pwm[frame.motor_offset + frame.motors[i].servo] = static_cast<std::uint16_t>(mixer.pwm_out(i));
    }
}

inline void leftover_mission_begin_takeoff(LeftoverMission& mission) {
    mission.phase = MissionPhase::kTakeoff;
    mission.hold_elapsed_s = 0.0f;
}

inline void leftover_mission_advance(copter::LeftoverCopter& copter, sim::SimMulticopter& sim, LeftoverMission& mission,
                                     float dt) {
    const float alt_m = -sim.position.z;

    switch (mission.phase) {
    case MissionPhase::kDisarmed:
        copter.motors_armed = false;
        copter.land_complete = true;
        mission.command = 0.0f;
        break;

    case MissionPhase::kTakeoff: {
        copter.motors_armed = true;
        if (copter.land_complete) {
            copter::UserTakeoffInputs in;
            in.motors_armed = true;
            in.land_complete = true;
            in.has_user_takeoff = true;
            in.takeoff_alt_m = mission.takeoff_alt_m;
            in.current_alt_m = alt_m;
            copter::UserTakeoffEffects fx;
            if (leftover_do_user_takeoff_U_m(in, fx, &mission.takeoff, alt_m) && fx.leftover_takeoff_start_m) {
                copter.land_complete = false;
            }
        }
        mission.command = leftover_poscontrol_throttle(copter, sim, -mission.takeoff_alt_m, -2.5f);
        if (alt_m >= mission.takeoff_alt_m) {
            mission.takeoff._running = false;
            mission.phase = MissionPhase::kHold;
            mission.hold_elapsed_s = 0.0f;
            mission.command = leftover_hold_command(copter, sim, mission.takeoff_alt_m);
        }
        break;
    }

    case MissionPhase::kHold:
        copter.motors_armed = true;
        mission.command = leftover_hold_command(copter, sim, mission.takeoff_alt_m);
        mission.hold_elapsed_s += dt;
        if (mission.hold_elapsed_s >= mission.hold_s) {
            mission.phase = MissionPhase::kLand;
        }
        break;

    case MissionPhase::kLand: {
        copter.motors_armed = true;
        mission.command = leftover_poscontrol_throttle(copter, sim, 0.25f, 1.5f);
        if (sim.on_ground()) {
            copter::LandDetectorInputs lin;
            lin.motors_armed = true;
            lin.land_complete = copter.land_complete;
            lin.descent_rate_low = true;
            lin.throttle_at_lower_limit = true;
            lin.motors_throttle_low = true;
            lin.throttle_mix_min = true;
            lin.accel_stationary = true;
            lin.rangefinder_check = true;
            lin.wow_check = true;
            copter::LandDetectorEffects lfx;
            leftover_update_land_and_crash_detectors(lin, lfx);
            if (lfx.land_complete) {
                copter.land_complete = true;
                copter.motors_armed = false;
                mission.command = 0.0f;
                mission.phase = MissionPhase::kLanded;
            }
        }
        break;
    }

    case MissionPhase::kLanded:
        copter.motors_armed = false;
        copter.land_complete = true;
        mission.command = 0.0f;
        break;
    }

    leftover_apply_collective(copter, sim, mission.command, copter.loop_dt);
}

// Mission leftover then CCP-043/045 harness (sensors + Frame/Motor plant).
inline void leftover_copter_sitl_step(SitlCopterHarness& harness, LeftoverMission& mission, float dt) {
    leftover_mission_advance(harness.copter(), harness.sim(), mission, dt);
    harness.step(dt);
}

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
    {"leftover catalog", PortStatus::kThisSlice, "this table; CCP-044 mission leftover + CCP-045 plant"},
    {"copter_sitl_run scaffold", PortStatus::kThisSlice,
     "sitl/copter_main.cpp + CMake copter_sitl_run target"},
    {"leftover_mission_advance", PortStatus::kThisSlice, "arm / takeoff / hold / land leftover state machine"},
    {"leftover_hold_command", PortStatus::kThisSlice,
     "CCP-064: HOLD via AC_PosControl D cascade (not leftover vz damper)"},
    {"leftover_poscontrol_throttle", PortStatus::kThisSlice,
     "CCP-064: pos_desired/vel_desired -> PosControlD::update_controller throttle_out"},
    {"leftover_copter_loop", PortStatus::kOnMain,
     "CCP-064: leftover_copter_tick walks Copter scheduler leftover free functions"},
    {"leftover_apply_collective", PortStatus::kThisSlice,
     "leftover collective command → per-motor PWM (equal mix into Frame)"},
    {"leftover_copter_sitl_step", PortStatus::kThisSlice,
     "mission then SitlCopterHarness::step (sensors + SimMulticopter update)"},
    {"copter_sitl_run arm/takeoff/hold/land", PortStatus::kThisSlice,
     "main() drives leftover mission on the real Frame/Motor plant"},
    {"SitlCopterHarness sensor synth (CCP-043)", PortStatus::kOnMain,
     "sitl_copter_harness.hpp"},
    {"leftover takeoff / land_detector (CCP-041)", PortStatus::kOnMain,
     "takeoff.hpp + land_detector.hpp remaining_count()==0"},
    {"SIM_Multicopter Frame/Motor mixing", PortStatus::kOnMain,
     "CCP-045: sim_multicopter.hpp / sim_frame.hpp / sim_motor.hpp — not leftover body-z"},
    {"GCS / MAVLink / interactive run", PortStatus::kOutOfScope,
     "no GCS in this port; bounded duration like CPP-085"},
    {"AP:: / HAL SITL singletons", PortStatus::kOutOfScope, "ADR-0012 explicit refs"},
    {"Rust copter-sitl", PortStatus::kOutOfScope, "Do not copy Rust"},
    {"JSON custom frame models / battery drain / shove-twist-clamp", PortStatus::kOutOfScope,
     "optional original extras; default_model + constant voltage"},
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

}  // namespace fwcpp::hal_sitl::copter_sitl_run
