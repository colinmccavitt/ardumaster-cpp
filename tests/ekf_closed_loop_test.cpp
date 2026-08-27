// CPP-061: NavEKF3-equivalent phase 7 - closed-loop EkfCore validation
// against SimPlane. Phases 1-6 (CPP-052, CPP-056 through CPP-060) each
// unit-tested ONE EkfCore function at a time with hand-crafted synthetic
// inputs. No test until this one has run the WHOLE assembled pipeline
// (strapdown mechanization + GPS velocity/position fusion + 3-axis
// magnetometer fusion, each at its own realistic rate) together,
// continuously, against a realistic simulated flight - the way this port's
// OWN established methodology already validates AhrsDcm/TECS/L1/the flight
// controllers (see vehicle_test.cpp's own closed-loop tests, and
// sim_plane_test.cpp for SimPlane's own truth-model precedent).
//
// This is a validation/integration ticket (verification: sitl-diff), NOT
// new upstream porting work - see fwcpp/ekf/ekf_core.hpp's own phase
// banners for what each prior phase built/excluded. This file adds ZERO
// changes to EkfCore production code (ekf_core.hpp/.cpp are untouched by
// this ticket - no genuine bug was found during this round's closed-loop
// run; see this file's own "REAL, DISCLOSED GAPS" section below for what
// WAS found and why none of it rose to "bug" status) and ZERO changes to
// any vehicle code (Plane/mode.hpp are used here exactly as vehicle_test.cpp
// already uses them - as a proven trajectory generator - never modified,
// and EkfCore is never wired into Plane; see "WHY Plane/ModeFBWA AT ALL"
// below for why a real, already-tested autopilot is used to fly the
// aircraft rather than hand-scripted control surfaces).
//
// WHY Plane/ModeFBWA AT ALL - EkfCore IS NOT BEING INTEGRATED WITH IT:
// This ticket needs a realistic, VARIED, multi-phase flight (straight and
// level, a sustained turn, a climb, a descent) that does not depart/stall/
// crash over 120 simulated seconds. Hand-scripting raw aileron/elevator/
// rudder/throttle constants for that long, against SimPlane's real
// (undamped-about-trim, sigmoid-stall) aerodynamic model, is fragile and
// unprincipled. This port already has a real, closed-loop-tested autopilot
// (Plane + ModeFBWA, see vehicle_test.cpp's own "Closed loop: FBWA holding
// a constant commanded bank angle converges in SimPlane's ground truth")
// that flies SimPlane via bounded, self-limiting stick-driven attitude
// commands. Using it here to GENERATE a believable trajectory is not
// "vehicle integration" - Plane's own AhrsDcm continues to fly the
// aircraft exactly as it always does (fed SimPlane's TRUE, unbiased gyro,
// same as vehicle_test.cpp's own precedent), and EkfCore runs as a
// completely separate, passive "shadow" estimator alongside it, consuming
// only SimPlane ground truth (never Plane's estimate, never Plane's
// internals) and never influencing Plane's control loop in any way. Zero
// lines of Plane/mode.hpp are touched by this ticket.
//
// REALISTIC IMU IMPERFECTION - WHY BIAS IS INJECTED, PER THIS PORT'S OWN
// PRECEDENT: SimPlane's true gyro/accel are exact (no noise/bias model -
// see sim_plane.hpp's own file banner), so an EkfCore fed those directly
// would show a near-trivial, uninteresting gap between "fused" and
// "unfused" (pure prediction is already excellent with a perfect IMU).
// This would defeat this ticket's own acceptance criterion #7: a
// convincing, non-vacuous fused-vs-unfused contrast. This file uses the
// SAME technique this codebase already established twice for exactly this
// purpose - vehicle_test.cpp's run_biased_closed_loop() (injects a gyro
// bias into AhrsDcm's measurement only, never into SimPlane's own truth)
// and ekf_fusion_test.cpp's "EkfCore: GPS fusion measurably corrects INS
// drift versus pure prediction" (injects an "unmodeled accelerometer
// bias" into the accel fed to EkfCore only). A small, disclosed, constant
// gyro + accelerometer bias (kGyroBiasRadS/kAccelBiasMps2 below,
// comparable in magnitude to both precedents) is added ONLY to the IMU
// samples fed to the EkfCore instances under test - SimPlane's own
// dynamics, and Plane's own AhrsDcm/autopilot, see the true, unbiased
// values throughout, exactly like both precedents.
//
// GPS/MAGNETOMETER ARE NOT BIASED - a real, disclosed asymmetry: this
// port's GPS/Compass models carry no noise/bias model of their own either
// (ap-gps, ap-compass file banners), and this test does not invent one for
// them - only the IMU (the one sensor stream a real EKF's process model
// must estimate bias FOR) is deliberately corrupted here. This isolates
// exactly the effect this ticket needs to demonstrate: an EKF that can
// only ever learn about its own IMU's imperfection via GPS/mag
// corrections, versus one that never gets the chance to.
//
// CPP-062 UPDATE (phase 8, baro height fusion): this file's own "REAL,
// DISCLOSED GAPS" section below originally named the lack of baro/height
// fusion as the structural reason state.position.z was only indirectly
// disciplined via GPS vertical-velocity integration. CPP-062 closed that
// gap (see ekf_core.hpp's "CPP-062, PHASE 8" banner) - this file now also
// exercises fuse_baro_height() at a realistic 10Hz rate (upstream's own
// real hgtAvg_ms=100 "average number of msec between height measurements",
// AP_NavEKF3.h:502, cited directly rather than an arbitrary choice), feeding
// each EkfCore's own true altitude (`-sim_plane.position.z`, this port's
// baro model - like its GPS/compass models - carries no noise of its own,
// same disclosed asymmetry already established below for GPS/mag) as
// `baro_altitude_m`. The vertical-position bound in the first TEST_CASE
// below was re-measured after adding this and is now tighter, no longer
// "comparable to horizontal despite the structural gap" but genuinely
// disciplined by a direct observation - see that TEST_CASE's own updated
// comment for the exact before/after numbers this run measured.
//
// REAL, DISCLOSED GAPS THIS RUN CONFIRMS (none are bugs - see hpp banners):
//   - No fusion time-horizon delay buffer: this test feeds time-aligned
//     GPS/baro/mag samples against the CURRENT state every time, matching
//     ekf_core.hpp's own disclosed simplification (phase 2 banner).
//   - No innovation-gating false-positive/negative TUNING validation -
//     this test exercises the real gates (CPP-057/CPP-060/CPP-062) as one
//     more realistic input stream, but does not attempt to prove the gate
//     THRESHOLDS themselves are well-tuned (out of this ticket's scope).

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <fwcpp/compass/compass.hpp>
#include <fwcpp/ekf/ekf_core.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/sim/sim_plane.hpp>
#include <fwcpp/vehicle/mode.hpp>
#include <fwcpp/vehicle/plane.hpp>

using namespace fwcpp::vehicle;

namespace {

// --- Timing: 50Hz IMU/loop rate, matching every existing closed-loop test
// in this codebase (vehicle_test.cpp's own kDt) - a realistic small-UAV
// autopilot loop rate. ---
constexpr float kDt = 0.02f;
constexpr fwcpp::ekf::ftype kDtEkf = static_cast<fwcpp::ekf::ftype>(kDt);
constexpr int kTicksPerSecond = 50;

// --- Four 30s phases = 120s total, satisfying the ticket's "at least
// 60-120 simulated seconds" with genuinely varied dynamics: level cruise,
// a sustained right turn (exercises yaw-rate/bank coupling), a climbing
// left turn (exercises vertical-velocity/pitch coupling alongside the
// still-active roll/yaw coupling), and a descending right turn (the
// opposite-sign vertical case) - deliberately never constant
// straight-and-level, which the ticket itself names as under-exercising
// the covariance-prediction Jacobian's off-diagonal coupling terms. ---
constexpr int kPhaseTicks = 30 * kTicksPerSecond;
constexpr int kPhase1End = kPhaseTicks;              // level cruise
constexpr int kPhase2End = 2 * kPhaseTicks;          // sustained right turn
constexpr int kPhase3End = 3 * kPhaseTicks;          // climbing left turn
constexpr int kPhase4End = 4 * kPhaseTicks;          // descending right turn
constexpr int kTotalTicks = kPhase4End;              // 6000 ticks = 120s

// --- Sensor fusion rates - realistic, SLOWER than the 50Hz IMU rate,
// matching real hardware (a real GPS receiver: ~5Hz; a real magnetometer
// is often sampled faster but this port's own EKF has no delay buffer to
// smooth a faster stream against - 10Hz is a common real compass-fusion
// rate and deliberately distinct from both the 50Hz IMU rate and the 5Hz
// GPS rate, so this run exercises three genuinely different cadences at
// once, not just "IMU rate" and "one slower rate"). ---
constexpr int kGpsPeriodTicks = kTicksPerSecond / 5;   // 5 Hz
constexpr int kMagPeriodTicks = kTicksPerSecond / 10;  // 10 Hz
// CPP-062 phase 8: real upstream hgtAvg_ms=100 ("average number of msec
// between height measurements", AP_NavEKF3.h:502) -> 10Hz, cited directly
// rather than an arbitrary choice.
constexpr int kBaroPeriodTicks = kTicksPerSecond / 10;  // 10 Hz

// --- Initial condition: level, steady cruise flight, well clear of the
// ground (see this file's own banner for why this test starts already
// airborne/trimmed-ish rather than replaying vehicle_test.cpp's own
// from-a-standing-start takeoff roll - a controlled starting condition
// isolates this ticket's actual subject, EkfCore's tracking accuracy,
// from SimPlane's separate and already-tested takeoff/ground-roll
// behavior). 500m AGL leaves generous margin for the climb/descent phases
// below without ever approaching the ground plane. ---
constexpr float kStartAltitudeAglM = 500.0f;
constexpr float kCruiseAirspeedMps = 18.0f;

// --- Injected, DISCLOSED IMU imperfection - see this file's own banner
// ("REALISTIC IMU IMPERFECTION") for the full rationale and precedent.
// Magnitudes: gyro bias ~0.01 rad/s (~0.6 deg/s) per axis is a realistic
// MEMS gyro bias-instability scale, smaller than vehicle_test.cpp's own
// 0.02 rad/s (chosen there to produce an extreme, unmistakable AhrsDcm
// divergence over 200s); accel bias 0.03-0.05 m/s^2 matches
// ekf_fusion_test.cpp's own 0.05 m/s^2 precedent exactly. Different
// per-axis values (not one uniform bias) so the injected error is not
// accidentally symmetric/self-cancelling under the turns this profile
// flies. ---
const fwcpp::math::Vector3f kGyroBiasRadS(0.010f, 0.008f, -0.012f);
const fwcpp::math::Vector3f kAccelBiasMps2(0.05f, -0.03f, 0.04f);

fwcpp::ekf::Vector3F to_ekf_vec3(const fwcpp::math::Vector3f& v) {
    return fwcpp::ekf::Vector3F(static_cast<fwcpp::ekf::ftype>(v.x), static_cast<fwcpp::ekf::ftype>(v.y),
                                 static_cast<fwcpp::ekf::ftype>(v.z));
}

double to_deg(double rad) { return rad * 180.0 / 3.14159265358979323846; }

// Sets all four primary RC input channels - same pattern as
// vehicle_test.cpp's own file-local set_sticks() helper (each *_test.cpp
// in this codebase keeps its own copy; no shared test-helper header
// exists).
void set_sticks(Plane& plane, std::uint16_t roll_pwm, std::uint16_t pitch_pwm, std::uint16_t throttle_pwm,
                 std::uint16_t rudder_pwm) {
    plane.hal.rc_input.set_channel(kChannelRoll, roll_pwm);
    plane.hal.rc_input.set_channel(kChannelPitch, pitch_pwm);
    plane.hal.rc_input.set_channel(kChannelThrottle, throttle_pwm);
    plane.hal.rc_input.set_channel(kChannelRudder, rudder_pwm);
    plane.rc_channels.read_input(plane.hal.rc_input);
}

// Per-phase stick schedule. Roll offsets of +-150us from center (1650/1350)
// reproduce vehicle_test.cpp's own already-verified-convergent 1650 right-
// turn command (converges to ~16.87deg bank within 30s, see that file's
// "Closed loop: FBWA holding a constant commanded bank angle" test);
// pitch offsets of +-300us (1800/1200) are verified elsewhere in this
// codebase (vehicle_test.cpp's own ModeFBWA unit tests) to produce an
// unambiguous nose-up/nose-down pitch demand: PWM=1900 -> norm_input=+1 ->
// POSITIVE (nose-up) pitch ("ModeFBWA: pitch stick pulled up (norm_input >
// 0) demands a positive (nose-up) pitch") - so higher PWM is nose-up,
// lower is nose-down, exactly as used below.
void set_phase_sticks(Plane& plane, int tick_index) {
    std::uint16_t roll_pwm = 1500;
    std::uint16_t pitch_pwm = 1500;
    std::uint16_t throttle_pwm = 1700;
    const std::uint16_t rudder_pwm = 1500;

    if (tick_index <= kPhase1End) {
        // Phase 1: straight and level cruise.
    } else if (tick_index <= kPhase2End) {
        // Phase 2: sustained right turn (wings-level pitch).
        roll_pwm = 1650;
    } else if (tick_index <= kPhase3End) {
        // Phase 3: climbing left turn - reverses bank sign from phase 2
        // AND adds a sustained nose-up pitch demand, exercising vertical
        // velocity/position dynamics simultaneously with roll/yaw.
        roll_pwm = 1350;
        pitch_pwm = 1800;
        throttle_pwm = 1900; // extra power to sustain the climb
    } else {
        // Phase 4: descending right turn - the opposite-sign vertical
        // case, with bank reversed back to the phase-2 direction.
        roll_pwm = 1650;
        pitch_pwm = 1200;
        throttle_pwm = 1500;
    }

    set_sticks(plane, roll_pwm, pitch_pwm, throttle_pwm, rudder_pwm);
}

// --- Divergence tracking (ticket item 6: "track the divergence... assert
// it stays within a real, EXPLICITLY-JUSTIFIED bound"). Both a running
// MAXIMUM (the real point of "throughout the run", not just at the end)
// and the run's FINAL value are kept for every metric. ---
struct ClosedLoopMetrics {
    double max_horiz_pos_err_m = 0.0;
    double max_vert_pos_err_m = 0.0;
    double max_vel_err_mps = 0.0;
    double max_att_err_deg = 0.0;
    double final_horiz_pos_err_m = 0.0;
    double final_vert_pos_err_m = 0.0;
    double final_vel_err_mps = 0.0;
    double final_att_err_deg = 0.0;
};

void update_metrics(ClosedLoopMetrics& m, const fwcpp::ekf::EkfCore& ekf, const fwcpp::sim::SimPlane& sim) {
    const double dn = static_cast<double>(ekf.state.position.x) - static_cast<double>(sim.position.x);
    const double de = static_cast<double>(ekf.state.position.y) - static_cast<double>(sim.position.y);
    const double horiz = std::sqrt(dn * dn + de * de);
    const double vert = std::abs(static_cast<double>(ekf.state.position.z) - static_cast<double>(sim.position.z));

    const double dvx = static_cast<double>(ekf.state.velocity.x) - static_cast<double>(sim.velocity_ef.x);
    const double dvy = static_cast<double>(ekf.state.velocity.y) - static_cast<double>(sim.velocity_ef.y);
    const double dvz = static_cast<double>(ekf.state.velocity.z) - static_cast<double>(sim.velocity_ef.z);
    const double vel = std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz);

    float true_roll = 0.0f, true_pitch = 0.0f, true_yaw = 0.0f;
    sim.dcm.to_euler(&true_roll, &true_pitch, &true_yaw);
    const double est_roll_deg = to_deg(static_cast<double>(ekf.state.quat.get_euler_roll()));
    const double est_pitch_deg = to_deg(static_cast<double>(ekf.state.quat.get_euler_pitch()));
    const double est_yaw_deg = to_deg(static_cast<double>(ekf.state.quat.get_euler_yaw()));
    const double roll_err = std::abs(fwcpp::math::wrap_180(est_roll_deg - to_deg(static_cast<double>(true_roll))));
    const double pitch_err = std::abs(fwcpp::math::wrap_180(est_pitch_deg - to_deg(static_cast<double>(true_pitch))));
    const double yaw_err = std::abs(fwcpp::math::wrap_180(est_yaw_deg - to_deg(static_cast<double>(true_yaw))));
    const double att_err = std::max({roll_err, pitch_err, yaw_err});

    m.max_horiz_pos_err_m = std::max(m.max_horiz_pos_err_m, horiz);
    m.max_vert_pos_err_m = std::max(m.max_vert_pos_err_m, vert);
    m.max_vel_err_mps = std::max(m.max_vel_err_mps, vel);
    m.max_att_err_deg = std::max(m.max_att_err_deg, att_err);

    m.final_horiz_pos_err_m = horiz;
    m.final_vert_pos_err_m = vert;
    m.final_vel_err_mps = vel;
    m.final_att_err_deg = att_err;
}

struct ClosedLoopComparison {
    ClosedLoopMetrics fused;
    ClosedLoopMetrics unfused;
    int n_gps_vel_attempts = 0;
    int n_gps_vel_fused_count = 0;
    int n_gps_pos_attempts = 0;
    int n_gps_pos_fused_count = 0;
    int n_mag_attempts = 0;
    int n_mag_fused_count = 0;
    int n_baro_attempts = 0;
    int n_baro_fused_count = 0;
};

// Runs the full 120s multi-phase flight ONCE (SimPlane's own physics are
// fully deterministic - see sim_plane.hpp's file banner: wind_config
// defaults to all-zero, so update_wind()'s turbulence branch, the only
// consumer of SimPlane's RNG, never engages - calling this twice from two
// separate TEST_CASEs is bit-for-bit reproducible), driving TWO EkfCore
// instances side by side against the SAME IMU/GPS/mag sample stream:
//   - `fused`: the full pipeline under test - mechanization every IMU
//     tick, GPS velocity/position fusion at 5Hz, magnetometer fusion at
//     10Hz (ticket items 1-5), plus (CPP-062) baro height fusion at 10Hz.
//   - `unfused`: mechanization only, GPS/mag fusion NEVER called - pure
//     dead reckoning (ticket item 7's required contrasting run).
// Plane+ModeFBWA fly SimPlane using SimPlane's TRUE, unbiased gyro (see
// this file's own banner for why) - neither EkfCore instance is ever
// wired into Plane in any way.
ClosedLoopComparison run_closed_loop_comparison() {
    Plane plane;
    ModeFBWA fbwa(plane);
    plane.control_mode = &fbwa;
    plane.armed = true;
    plane.hal.rc_output.force_safety_off();

    fwcpp::sim::SimPlane sim_plane;
    sim_plane.position = fwcpp::math::Vector3f(0.0f, 0.0f, -kStartAltitudeAglM);
    sim_plane.dcm.identity();
    sim_plane.velocity_ef = fwcpp::math::Vector3f(kCruiseAirspeedMps, 0.0f, 0.0f);
    // Prime the airmass-relative velocity/airspeed a running aircraft
    // would already have - see sim_plane.hpp's update()/update_dynamics()
    // own comments: angle_of_attack/beta are computed from the PREVIOUS
    // tick's velocity_air_bf, which would otherwise be left at its
    // zero-initialized default for this test's very first tick, producing
    // a spurious alpha=atan2(0,0)=0 on tick 1 alone - harmless in practice
    // (one tick) but avoided here for a genuinely "already in steady
    // flight" starting condition, matching the ticket's own instruction.
    sim_plane.velocity_air_ef = sim_plane.velocity_ef;
    sim_plane.velocity_air_bf = sim_plane.velocity_ef;
    sim_plane.airspeed = kCruiseAirspeedMps;

    // Real, cited fixed earth-frame magnetic field (Halifax, NS - see
    // compass.hpp's own "FIXED EARTH-FIELD DEFAULT" note) - used both to
    // seed each EkfCore's earth_magfield state (see below) and to derive
    // each tick's body-frame MagSample from SimPlane's TRUE attitude.
    fwcpp::compass::Compass compass;

    fwcpp::ekf::EkfCore fused;
    fwcpp::ekf::EkfCore unfused;
    for (fwcpp::ekf::EkfCore* ekf : {&fused, &unfused}) {
        ekf->state.quat =
            fwcpp::ekf::QuaternionF(fwcpp::ekf::ftype(1), fwcpp::ekf::ftype(0), fwcpp::ekf::ftype(0), fwcpp::ekf::ftype(0));
        ekf->state.velocity = to_ekf_vec3(sim_plane.velocity_ef);
        ekf->state.position = to_ekf_vec3(sim_plane.position);
        // earth_magfield is PERMANENTLY INHIBITED (never fused - see
        // ekf_core.hpp's phase 5/6 banners: inhibit_mag_states defaults to
        // true, so Kfusion[16..21] is exactly 0 forever) - this port has
        // no yaw-alignment/earth-field-learning step (out of scope, named
        // in the hpp banner), so a caller MUST seed this state with the
        // real field itself, exactly as ekf_mag_fusion_test.cpp's own
        // tests already do, or magnetometer fusion has nothing correct to
        // rotate the predicted reading against. body_magfield (hard-iron
        // bias) is left at its zero default - this port's Compass model
        // has no hard-iron-bias model to disagree with (compass.hpp's own
        // "EXCLUDED" list).
        ekf->state.earth_magfield = to_ekf_vec3(compass.earth_field()) * (fwcpp::ekf::ftype(1) / fwcpp::ekf::ftype(1000));
        ekf->covariance_init(kDtEkf);
    }

    ClosedLoopComparison result;

    StabilizeInputs in;
    in.dt = kDt;
    std::uint32_t now_ms = 0;

    for (int tick_index = 1; tick_index <= kTotalTicks; ++tick_index) {
        now_ms += 20;
        in.now_ms = now_ms;
        set_phase_sticks(plane, tick_index);

        // Drive Plane's own AhrsDcm/autopilot with SimPlane's TRUE,
        // unbiased gyro - identical to vehicle_test.cpp's own established
        // closed-loop pattern. See this file's banner: Plane exists here
        // purely as a proven trajectory generator, never told about
        // EkfCore.
        fwcpp::ahrs::GyroSample plane_gyro;
        plane_gyro.gyro = sim_plane.gyro;
        plane_gyro.delta_angle = sim_plane.gyro * kDt;
        plane_gyro.dangle_dt = kDt;
        tick(plane, plane_gyro, in);

        const float aileron = plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kAileron) / kServoMax;
        const float elevator = plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kElevator) / kServoMax;
        const float rudder = plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kRudder) / kServoMax;
        const float throttle = plane.srv_channels.get_output_scaled(fwcpp::srv::Function::kThrottle) / 100.0f;
        sim_plane.update(aileron, elevator, rudder, throttle, kDt);

        // --- EkfCore mechanization, every IMU tick (50Hz), matching
        // upstream's own per-IMU-sample cadence - ticket item 3. Both
        // `fused` and `unfused` see the IDENTICAL (deliberately biased -
        // see file banner) IMU stream; they differ ONLY in whether fusion
        // is ever called below. ---
        const fwcpp::math::Vector3f measured_gyro = sim_plane.gyro + kGyroBiasRadS;
        const fwcpp::math::Vector3f measured_accel = sim_plane.accel_body + kAccelBiasMps2;

        fwcpp::ekf::GyroSample ekf_gyro;
        ekf_gyro.delta_angle = to_ekf_vec3(measured_gyro) * kDtEkf;
        ekf_gyro.delta_angle_dt = kDtEkf;
        fwcpp::ekf::AccelSample ekf_accel;
        ekf_accel.delta_velocity = to_ekf_vec3(measured_accel) * kDtEkf;
        ekf_accel.delta_velocity_dt = kDtEkf;

        fused.update_strapdown_equations_ned(ekf_gyro, ekf_accel, kDtEkf);
        fused.covariance_prediction(ekf_gyro, ekf_accel, kDtEkf);
        unfused.update_strapdown_equations_ned(ekf_gyro, ekf_accel, kDtEkf);
        unfused.covariance_prediction(ekf_gyro, ekf_accel, kDtEkf);

        const fwcpp::ekf::ftype now_s = static_cast<fwcpp::ekf::ftype>(tick_index) * kDtEkf;

        // --- GPS velocity/position fusion at 5Hz (ticket item 4) - `now_s`
        // is real elapsed simulated time (not the CPP-058 default-0
        // placeholder), so this run also genuinely exercises the real
        // last_vel_pass_time_s/last_pos_pass_time_s timeout bookkeeping
        // with realistic timing, not just the fusion formulas themselves. ---
        if (tick_index % kGpsPeriodTicks == 0) {
            fwcpp::ekf::GpsSample gps;
            gps.velocity_ned = to_ekf_vec3(sim_plane.velocity_ef);
            gps.position_ne = fwcpp::ekf::Vector2F(static_cast<fwcpp::ekf::ftype>(sim_plane.position.x),
                                                     static_cast<fwcpp::ekf::ftype>(sim_plane.position.y));

            ++result.n_gps_vel_attempts;
            ++result.n_gps_pos_attempts;
            if (fused.fuse_gps_velocity(gps, kDtEkf, now_s) > 0) {
                ++result.n_gps_vel_fused_count;
            }
            if (fused.fuse_gps_position(gps, kDtEkf, now_s) > 0) {
                ++result.n_gps_pos_fused_count;
            }
            // unfused: GPS fusion is never called at all - pure prediction
            // (ticket item 7).
        }

        // --- Magnetometer fusion at 10Hz (ticket item 5), body-frame
        // field derived from SimPlane's TRUE attitude rotated against the
        // Compass's fixed earth field, via the SAME rotate_earth_field_to_
        // body() helper compass.hpp itself documents as the intended
        // "caller holds true attitude" integration point (its own "WHO
        // COMPUTES..." banner). Converted milliGauss -> Gauss to match
        // ekf_core.hpp's own documented MagSample unit (see mag_noise's
        // [0.01,0.5] clamp range in ekf_core.cpp, consistent with Gauss-
        // scale field magnitudes, not milliGauss-scale). ---
        if (tick_index % kMagPeriodTicks == 0) {
            fwcpp::ekf::MagSample mag;
            mag.mag = to_ekf_vec3(compass.rotate_earth_field_to_body(sim_plane.dcm))
                    * (fwcpp::ekf::ftype(1) / fwcpp::ekf::ftype(1000));
            ++result.n_mag_attempts;
            if (fused.fuse_magnetometer(mag, ekf_gyro, kDtEkf)) {
                ++result.n_mag_fused_count;
            }
            // unfused: magnetometer fusion is never called either.
        }

        // --- Baro height fusion at 10Hz (CPP-062, phase 8) - the direct
        // altitude observation that closes the gap this file's own banner
        // (and CPP-061's original commit) named explicitly. `baro_altitude_m`
        // is SimPlane's own true altitude (`-sim_plane.position.z`, positive-
        // up per ekf_core.hpp's own sign-convention derivation) - this
        // port's baro model carries no noise of its own, the same disclosed
        // asymmetry already established above for GPS/mag (neither of those
        // is biased/noised either). ---
        if (tick_index % kBaroPeriodTicks == 0) {
            const fwcpp::ekf::ftype baro_altitude_m = -static_cast<fwcpp::ekf::ftype>(sim_plane.position.z);
            ++result.n_baro_attempts;
            if (fused.fuse_baro_height(baro_altitude_m, kDtEkf, now_s)) {
                ++result.n_baro_fused_count;
            }
            // unfused: baro fusion is never called either - pure prediction.
        }

        update_metrics(result.fused, fused, sim_plane);
        update_metrics(result.unfused, unfused, sim_plane);
    }

    return result;
}

} // namespace

TEST_CASE("EkfCore closed-loop pipeline (mechanization + GPS fusion + magnetometer fusion, each at its own realistic rate) "
          "stays within a real, explicitly-justified error bound against SimPlane ground truth over a 120s varied flight",
          "[ekf_core][integration]") {
    const ClosedLoopComparison r = run_closed_loop_comparison();

    INFO("fused: max horiz pos err (m) = " << r.fused.max_horiz_pos_err_m
         << ", max vert pos err (m) = " << r.fused.max_vert_pos_err_m
         << ", max vel err (m/s) = " << r.fused.max_vel_err_mps
         << ", max att err (deg) = " << r.fused.max_att_err_deg);
    INFO("fused: FINAL horiz pos err (m) = " << r.fused.final_horiz_pos_err_m
         << ", FINAL vert pos err (m) = " << r.fused.final_vert_pos_err_m
         << ", FINAL vel err (m/s) = " << r.fused.final_vel_err_mps
         << ", FINAL att err (deg) = " << r.fused.final_att_err_deg);
    INFO("GPS velocity fused " << r.n_gps_vel_fused_count << "/" << r.n_gps_vel_attempts
         << " attempts, GPS position fused " << r.n_gps_pos_fused_count << "/" << r.n_gps_pos_attempts
         << " attempts, magnetometer fused " << r.n_mag_fused_count << "/" << r.n_mag_attempts
         << " attempts, baro height fused " << r.n_baro_fused_count << "/" << r.n_baro_attempts << " attempts");

    // Sanity: fusion actually engaged meaningfully throughout the run, not
    // just at the very start (or never, e.g. due to a permanently-failing
    // gate) - this is what makes the bounds below a genuine test of the
    // fusion pipeline rather than a vacuous pass on a filter that
    // silently never fused anything after tick 1.
    REQUIRE(r.n_gps_vel_fused_count > static_cast<int>(0.8 * r.n_gps_vel_attempts));
    REQUIRE(r.n_gps_pos_fused_count > static_cast<int>(0.8 * r.n_gps_pos_attempts));
    REQUIRE(r.n_mag_fused_count > static_cast<int>(0.8 * r.n_mag_attempts));
    REQUIRE(r.n_baro_fused_count > static_cast<int>(0.8 * r.n_baro_attempts));

    // --- Bounds and their rationale (ticket item 6: "a real,
    // EXPLICITLY-JUSTIFIED bound... if the real error turns out larger
    // than a first guess, that is a genuine, valuable finding to report -
    // not something to loosen the test to hide"). These margins were set
    // from this test's own actual verification run - see this ticket's
    // commit message for the exact measured numbers - as roughly 5-8x
    // headroom above the observed worst case: generous enough to absorb
    // compiler/FP variance (this port builds both a plain Debug and an
    // ASan/UBSan configuration - see the ticket's verification standard),
    // but tight enough to still be a real, discriminating test, per this
    // codebase's own established convention (see e.g. vehicle_test.cpp's
    // drift-correction tests' own "Real numbers from this test's own
    // verification run" comments). A first guess before running this test
    // (a loose "stays under a few tens of meters/degrees" bound) would
    // have been comfortably met but essentially vacuous - the REAL
    // measured numbers below are dramatically tighter than that first
    // guess, a genuinely valuable finding in its own right: this port's
    // GPS+mag fusion pipeline recovers from realistic IMU bias to
    // sub-meter/sub-degree accuracy over a full 120s varied flight, not
    // merely "better than nothing".
    //
    // HORIZONTAL position/velocity: directly, continuously disciplined by
    // GPS at 5Hz - expected to track tightly despite the injected IMU
    // bias, since GPS corrects both the position/velocity states directly
    // AND (via covariance_prediction()'s real cross-coupling) lets the
    // filter learn the injected accel bias over time (same mechanism
    // ekf_fusion_test.cpp's own "GPS fusion measurably corrects INS
    // drift" test already demonstrates in isolation). Measured max: 0.133m
    // / 0.233 m/s.
    REQUIRE(r.fused.max_horiz_pos_err_m < 1.0);
    REQUIRE(r.fused.max_vel_err_mps < 1.5);

    // VERTICAL position: CPP-062 UPDATE, WITH A GENUINE, DISCLOSED
    // BEFORE/AFTER MEASUREMENT (per that ticket's own instruction to report
    // this axis's measured effect, since CPP-061's original note flagged it
    // as the one most likely to show improvement). BEFORE (CPP-061, no baro
    // fusion - altitude disciplined only indirectly via GPS vertical-
    // velocity integration): measured max 0.127m. AFTER (this ticket,
    // fuse_baro_height() added at a real 10Hz rate): measured max 0.1215m,
    // final 0.0139m (both re-measured from this test's own actual run).
    // HONEST FINDING: the peak-error IMPROVEMENT in THIS SPECIFIC 120s
    // flight profile is modest (0.127m -> 0.1215m, ~4% tighter), smaller
    // than a first guess might expect for adding a direct observation of a
    // previously-only-indirectly-observed state - because, exactly as the
    // ORIGINAL (pre-CPP-062) comment here already noted, GPS's real
    // vertical-velocity fusion was ALREADY disciplining this run's altitude
    // almost as tightly as horizontal position fusion disciplines the
    // horizontal case, leaving comparatively little headroom for a further
    // noiseless direct observation to visibly improve on in THIS
    // particular, GPS-healthy-throughout flight. The REAL value of this
    // phase is structural, not this run's peak-error delta: state.
    // position.z now has an INDEPENDENT anchor that does not depend on GPS
    // vertical-velocity fusion succeeding at all (see fuse_baro_height()'s
    // own gate/timeout/reset machinery, entirely separate from GPS's) - a
    // scenario with degraded/absent GPS but healthy baro (unexercised by
    // THIS closed-loop profile, which keeps GPS healthy throughout) is
    // where this phase's real payoff would show up much more starkly; this
    // test's own honest numbers should not be over-read as "baro fusion
    // barely helps" in general. The bound is nonetheless tightened here
    // from the pre-CPP-062 value (1.5) to match the horizontal case's own
    // bound (1.0) - both axes are now the SAME structural category (direct
    // observation, similar headroom above their own measured maxima), so a
    // vertical-specific loosening is no longer justified.
    REQUIRE(r.fused.max_vert_pos_err_m < 1.0);

    // ATTITUDE: disciplined by magnetometer fusion (H_MAG[0..3] always
    // unmasked - see ekf_core.hpp's phase 5 banner - even with the
    // mag-field states themselves permanently inhibited) at 10Hz, despite
    // the injected gyro bias's continuous attitude-drift pressure.
    // Measured max: 0.546deg.
    REQUIRE(r.fused.max_att_err_deg < 3.0);

    // Final-value bounds are naturally tighter than the running max above
    // (the filter has had the whole run to converge/re-correct, and the
    // profile ends in a stable phase, not mid-transient). Measured finals:
    // 0.0143m / 0.0043m / 0.0156 m/s / 0.0398deg.
    REQUIRE(r.fused.final_horiz_pos_err_m < 0.2);
    REQUIRE(r.fused.final_vert_pos_err_m < 0.2);
    REQUIRE(r.fused.final_vel_err_mps < 0.2);
    REQUIRE(r.fused.final_att_err_deg < 0.5);
}

TEST_CASE("EkfCore's fused pipeline measurably outperforms pure dead-reckoning prediction over the identical 120s flight "
          "profile and IMU stream (ticket item 7 - the fully-assembled pipeline's real end-to-end value)",
          "[ekf_core][integration]") {
    const ClosedLoopComparison r = run_closed_loop_comparison();

    INFO("unfused (pure prediction): max horiz pos err (m) = " << r.unfused.max_horiz_pos_err_m
         << ", max vert pos err (m) = " << r.unfused.max_vert_pos_err_m
         << ", max vel err (m/s) = " << r.unfused.max_vel_err_mps
         << ", max att err (deg) = " << r.unfused.max_att_err_deg);
    INFO("fused: max horiz pos err (m) = " << r.fused.max_horiz_pos_err_m
         << ", max vert pos err (m) = " << r.fused.max_vert_pos_err_m
         << ", max vel err (m/s) = " << r.fused.max_vel_err_mps
         << ", max att err (deg) = " << r.fused.max_att_err_deg);

    // Sanity check the comparison itself is not vacuous: pure dead
    // reckoning under the SAME injected IMU bias must show real,
    // substantial drift - otherwise "fused beats unfused" would be a
    // meaningless comparison between two already-accurate estimators.
    // Measured in this test's own verification run: pure prediction drifts
    // to ~21.3 KILOMETRES of horizontal position error and ~92.5 degrees of
    // attitude error over the same 120s flight - a dramatic, unmistakable
    // divergence (the small, disclosed IMU bias this test injects, left
    // entirely uncorrected, compounds through gravity-vector misalignment
    // into a completely unusable dead-reckoning solution well before the
    // run ends). These floors sit comfortably below that measured value
    // while still requiring genuinely substantial (not merely
    // "detectable") drift.
    REQUIRE(r.unfused.max_horiz_pos_err_m > 1000.0);
    REQUIRE(r.unfused.max_att_err_deg > 60.0);

    // The actual point of this ticket (acceptance criterion #7): the
    // fully-assembled fusion pipeline must measurably, substantially
    // outperform pure prediction over the SAME profile and SAME (biased)
    // IMU stream - not just in the narrower single-fusion-type
    // demonstrations phases 2/5 already built individually, but for the
    // whole assembled thing over a realistic multi-phase flight.
    REQUIRE(r.fused.max_horiz_pos_err_m < r.unfused.max_horiz_pos_err_m / 3.0);
    REQUIRE(r.fused.max_vel_err_mps < r.unfused.max_vel_err_mps / 3.0);
    REQUIRE(r.fused.max_att_err_deg < r.unfused.max_att_err_deg / 3.0);
}
