#pragma once

// Port of upstream SITL's "Plane" ground-truth flight-dynamics model:
// STANDARD fixed-wing configuration only. CPP-030.
//
// Upstream sources (Plane-4.7.0, read directly from the pinned worktree):
//   - libraries/SITL/SIM_Plane.h (141 lines, read in full) - the
//     Coefficients struct and its default_coefficients values (a real,
//     named airframe: last_letter's skywalker_2013/aerodynamics.yaml,
//     credited there to Georacer), mass=2.0f, hover_throttle=0.7f.
//   - libraries/SITL/SIM_Plane.cpp:
//       Plane::liftCoeff (line 235), Plane::dragCoeff (line 257),
//       Plane::getTorque (line 273), Plane::getForce (line 344),
//       Plane::calculate_forces (line 398), Plane::update (line 522).
//   - libraries/SITL/SIM_Aircraft.cpp:
//       Aircraft::hagl (line 145), Aircraft::on_ground (line 153),
//       Aircraft::update_dynamics (line 709).
//
// THIS IS NOT A PORT OF FLIGHT-CONTROL CODE. It is upstream's own PHYSICS
// ORACLE - the thing that stands in for a real aircraft during SITL - so
// this port's control loops (L1Control, AC_PID, AhrsDcm) can be tested
// end-to-end against a known-correct trajectory. SimPlane propagates truth
// directly and deliberately SHARES NO CODE with the estimator it is used to
// check: its own attitude integration (dcm.rotate + dcm.normalize) reuses
// ap-math's Matrix3, the same primitive AhrsDcm is built on, but never
// AhrsDcm itself - using the same integrator class for both the truth model
// and the thing being tested against it would make the check circular.
//
// NO SINGLETONS, EXPLICIT INPUTS INSTEAD (ADR-0012), matching this port's
// standing pattern (AcPid's Gains, L1Control's L1Inputs, AhrsDcm's
// GyroSample):
//   - Upstream's Aircraft/Plane reach into AP::ahrs(), AP_HAL::millis(), a
//     global SITL::SIM* singleton, AP_JSON model-file loading, and
//     battery/RPM/GCS simulation hooks. None of that exists here. SimPlane
//     owns all its own state directly (coefficients, mass, attitude,
//     gyro, velocity, position) and every update() call takes control
//     surface deflections, throttle, and dt explicitly.
//   - `struct sitl_input` (16-channel PWM array + per-vehicle decode via
//     filtered_servo_angle/filtered_servo_range) is replaced by plain
//     aileron/elevator/rudder/throttle float parameters, already in the
//     -1..1 (surfaces) / 0..1 (throttle) ranges filtered_servo_angle/
//     filtered_servo_range would have produced - servo PWM decoding is
//     ap-srv-channel's job, not this class's.
//
// SCOPE: STANDARD FIXED-WING CONFIGURATION ONLY. The following upstream
// Plane/Aircraft members, branches, and constructor frame-string flags are
// DELIBERATELY NOT PORTED in this slice:
//   - All airframe config variants and their "-suffix" frame-string
//     parsing: elevons, vtail, dspoilers, redundant, reverse_thrust,
//     reverse_elevator_rudder, tailsitter, aerobatic, copter_tailsitter,
//     have_launcher, have_steering, ice_engine (and ICEngine itself),
//     "-heavy"/"-jet" mass overrides. calculate_forces' surface-mixing
//     if/else-if chain (elevons/vtail/dspoilers/redundant) and its
//     reverse_elevator_rudder negation are skipped entirely - aileron/
//     elevator/rudder/throttle are used exactly as given. getTorque's
//     `if (tailsitter || aerobatic)` effective-airspeed/alpha adjustment
//     (SIM_Plane.cpp lines ~281-289) is excluded too - alpha and airspeed
//     are used as passed in, unmodified.
//   - load_coeffs() / AP_JSON model-file loading - Coefficients is a plain
//     aggregate a caller can hand-edit instead (see step 1 of the ticket).
//   - Wind modeling (Aircraft::update_wind, wind_ef). Genuinely out of
//     scope for a first slice: velocity_air_bf is simply the body-frame
//     velocity with wind_ef treated as always zero (velocity_air_ef ==
//     velocity_ef), and airspeed is the resulting true airspeed with
//     eas2tas == 1 (no barometer/EAS-TAS model exists in this port either -
//     see air_density below). A real wind model is slice 2 work (see the
//     bottom-of-file note).
//   - Atmosphere/air density model (AP_Baro::get_air_density_for_alt_amsl,
//     eas2tas from altitude). air_density defaults to SSL_AIR_DENSITY
//     (1.225f kg/m^3, AP_Math/definitions.h) and is held constant - a
//     caller may override it, but there is no altitude-driven model here
//     since this port has no AP_Baro yet.
//   - Sensor/GCS simulation: fill_fdm, smooth_sensors, add_noise,
//     update_mag_field_bf, battery/RPM simulation (battery_voltage,
//     battery_current, rpm[]), extrapolate_sensors - no sensor subsystems
//     (beyond AnalogIn) exist in this port to feed.
//   - The launcher (have_launcher / launch_accel / launch_time /
//     launch_start_ms) and ground-steering (have_steering, the
//     gyro.z += steering * ... nose-wheel hack in Plane::update) hooks -
//     test/debug airframe features, not core physics.
//   - Frame-rate/timing bookkeeping: frame_time_us, time_advance,
//     setup_frame_time, adjust_frame_time, sync_frame_time,
//     lock_step_scheduled. This class takes an explicit dt per update()
//     call, driven by whatever test/harness calls it, not a real-time
//     scheduler matching a target loop rate.
//   - set_pose / external pose injection, precision-landing hooks, Clamp,
//     shove/twist/external forces, ship/tether/slung-payload hooks
//     (AP_SIM_SHIP_ENABLED / AP_SIM_SLUNGPAYLOAD_ENABLED / AP_SIM_TETHER_
//     ENABLED) - test/debug/companion-computer features, not core physics.
//   - Aircraft::update_dynamics' entire `switch (ground_behavior)` block
//     (GROUND_BEHAVIOR_NO_MOVEMENT / FWD_ONLY / TAILSITTER) - upstream's
//     taxi/takeoff-roll simulation for various airframe configs (zeroing
//     roll/pitch, forcing forward-only body velocity, rotating the DCM for
//     tailsitter ground attitude). This slice's on_ground() handling is
//     the flat "don't sink through the floor" clamp described below,
//     nothing more.
//   - update_eas_airspeed's airspeed_pitot (120 m/s pitot-tube clamp) -
//     airspeed_pitot has no consumer in this port (no airspeed sensor
//     model yet); the plain `airspeed` member upstream itself uses for all
//     physics (as opposed to sensor simulation) is kept.
//
// GROUND MODEL - A DELIBERATE FLAT-EARTH SIMPLIFICATION, not upstream's:
// upstream's on_ground() is `hagl() <= 0.001f`, where hagl() itself is
// `(-position.z) + home.alt*0.01f - ground_level - frame_height -
// ground_height_difference()` - i.e. height above a possibly-sloped,
// possibly-offset-from-origin terrain model this port has none of (no
// Location/home/terrain subsystem wired to this class). This slice defines
// on_ground() as simply `position.z >= 0.0f`: a flat earth in NED
// coordinates, ground at z=0, with the caller responsible for initializing
// position.z to -initial_altitude. Likewise, upstream unconditionally snaps
// `position.z = -(ground_level + frame_height - home.alt*0.01f +
// ground_height_difference())` whenever on_ground() (SIM_Aircraft.cpp,
// just before the ground_behavior switch) - this class has no such terrain
// value to snap to, so ground contact is instead limited to exactly the two
// clamps upstream's OWN core update_dynamics already does independently of
// ground_behavior: (1) accel_earth.z clamped to <=0 while on the ground
// (upstream's real clamp, ported as-is), and (2) velocity_ef.z clamped to
// <=0 once integration lands on/through the ground plane. No position snap,
// no taxi/roll simulation - "don't sink through the floor", not a landing-
// gear model.
//
// LITERAL SAFETY / double-precision transcription: this header has no
// compiled .cpp of its own (no fwcpp_upstream_flags target to link), so
// nothing here needs the compiled-.cpp treatment scalar.cpp's wrap_*
// family needed - see scalar.hpp's own file banner for why that split
// exists at all. liftCoeff/dragCoeff/getForce/getTorque deliberately keep
// upstream's OWN `double` locals (sigmoid, linear, flatPlate, AR, c_drag_a,
// qbar, ax/ay/az, la/ma/na, p/q/r) exactly as upstream typed them, even
// though the surrounding Coefficients fields and parameters are float -
// ADR-0012's stance is against AMBIGUOUS bare literals under
// -fsingle-precision-constant, not against double itself, and upstream
// chose double precision for these specific intermediates on purpose.
// M_PI is not used directly (unlike upstream's dragCoeff, which multiplies
// by upstream's bare M_PI macro); fwcpp::math::pi_constant() is used
// instead - the same double-precision PI this port's own scalar.cpp
// already exposes for exactly this "rare caller needs the bare double
// constant" case, rather than adding a second hardcoded copy of M_PI here.

#include <cmath>

#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::sim {

// Upstream: AP_Math/definitions.h's GRAVITY_MSS (9.80665f) - reproduced as
// a local named constant rather than a shared ap-math export, matching
// this port's existing precedent (ap-nav's l1_control.hpp kGravityMss).
inline constexpr float kGravityMss = 9.80665f;

// Upstream: AP_Math/definitions.h's SSL_AIR_DENSITY (1.225f kg/m^3) - see
// file banner's "atmosphere/air density model" exclusion.
inline constexpr float kSslAirDensity = 1.225f;

// Upstream: SIM_Plane.h's nested `struct Coefficients` and its
// `default_coefficients` member - reproduced here as a plain, caller-
// overridable aggregate (no AP_JSON load_coeffs() in this port; a caller
// wanting a different airframe just constructs one and assigns fields).
// Comment and every numeric value transcribed exactly from upstream.
struct Coefficients {
    // from last_letter skywalker_2013/aerodynamics.yaml
    // thanks to Georacer!
    float s = 0.45f;
    float b = 1.88f;
    float c = 0.24f;
    float c_lift_0 = 0.56f;
    float c_lift_deltae = 0.0f;
    float c_lift_a = 6.9f;
    float c_lift_q = 0.0f;
    float mcoeff = 50.0f;
    float oswald = 0.9f;
    float alpha_stall = 0.4712f;
    float c_drag_q = 0.0f;
    float c_drag_deltae = 0.0f;
    float c_drag_p = 0.1f;
    float c_y_0 = 0.0f;
    float c_y_b = -0.98f;
    float c_y_p = 0.0f;
    float c_y_r = 0.0f;
    float c_y_deltaa = 0.0f;
    float c_y_deltar = -0.2f;
    float c_l_0 = 0.0f;
    float c_l_p = -1.0f;
    float c_l_b = -0.12f;
    float c_l_r = 0.14f;
    float c_l_deltaa = 0.25f;
    float c_l_deltar = -0.037f;
    float c_m_0 = 0.045f;
    float c_m_a = -0.7f;
    float c_m_q = -20.0f;
    float c_m_deltae = 1.0f;
    float c_n_0 = 0.0f;
    float c_n_b = 0.25f;
    float c_n_p = 0.022f;
    float c_n_r = -1.0f;
    float c_n_deltaa = 0.0f;
    float c_n_deltar = 0.1f;
    float deltaa_max = 0.3491f;
    float deltae_max = 0.3491f;
    float deltar_max = 0.3491f;
    // the X CoG offset should be -0.02, but that makes the plane too tail heavy
    // in manual flight. Adjusted to -0.15 gives reasonable flight
    math::Vector3f cg_offset{-0.15f, 0.0f, -0.05f};
};

// Ground-truth fixed-wing flight dynamics model - upstream: SITL::Plane
// (STANDARD configuration only; see file banner for every excluded
// variant). Owns the aircraft's true attitude/velocity/position state and
// advances it one `update()` call at a time given control-surface
// deflections and throttle.
class SimPlane {
public:
    // Upstream: Plane::Plane() sets coefficient = default_coefficients,
    // mass = 2.0f, hover_throttle is a const 0.7f member. All three are
    // constructor-overridable here since nothing in this port's JSON-model
    // sense applies, but the defaults reproduce upstream's own standard
    // "skywalker_2013"-equivalent plane with zero configuration.
    explicit SimPlane(const Coefficients& coeffs = Coefficients{}, float mass_kg = 2.0f, float hover_throttle = 0.7f)
        : coefficient(coeffs), mass(mass_kg), hover_throttle(hover_throttle) {
        dcm.identity();
    }

    // Upstream: Plane::liftCoeff (SIM_Plane.cpp:235) - from last_letter,
    // https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp,
    // thanks to Georacer! Sigmoid-blended stall model: a linear small-alpha
    // lift term smoothly blended into a flat-plate beyond-stall term as
    // alpha approaches/exceeds alpha_stall. alpha is clamped to
    // alpha_stall +/- 0.8 before use, to avoid exp() overflow in the
    // sigmoid - ported exactly, including the clamp.
    [[nodiscard]] float liftCoeff(float alpha) const {
        const float alpha0 = coefficient.alpha_stall;
        const float M = coefficient.mcoeff;
        const float c_lift_0 = coefficient.c_lift_0;
        const float c_lift_a0 = coefficient.c_lift_a;

        // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
        const float max_alpha_delta = 0.8f;
        if (alpha - alpha0 > max_alpha_delta) {
            alpha = alpha0 + max_alpha_delta;
        } else if (alpha0 - alpha > max_alpha_delta) {
            alpha = alpha0 - max_alpha_delta;
        }

        const double sigmoid = (1 + std::exp(-M * (alpha - alpha0)) + std::exp(M * (alpha + alpha0)))
                              / (1 + std::exp(-M * (alpha - alpha0))) / (1 + std::exp(M * (alpha + alpha0)));
        const double linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a0 * alpha); // Lift at small AoA
        const double flatPlate = sigmoid * (2 * std::copysign(1.0, alpha) * std::pow(std::sin(alpha), 2) * std::cos(alpha)); // Lift beyond stall

        const float result = static_cast<float>(linear + flatPlate);
        return result;
    }

    // Upstream: Plane::dragCoeff (SIM_Plane.cpp:257) - simple
    // aspect-ratio-based induced drag model, same last_letter source.
    [[nodiscard]] float dragCoeff(float alpha) const {
        const float b = coefficient.b;
        const float s = coefficient.s;
        const float c_drag_p = coefficient.c_drag_p;
        const float c_lift_0 = coefficient.c_lift_0;
        const float c_lift_a0 = coefficient.c_lift_a;
        const float oswald = coefficient.oswald;

        const double AR = std::pow(b, 2) / s;
        const double c_drag_a = c_drag_p + std::pow(c_lift_0 + c_lift_a0 * alpha, 2) / (math::pi_constant() * oswald * AR);

        return static_cast<float>(c_drag_a);
    }

    // Upstream: Plane::getForce (SIM_Plane.cpp:344). alpha/beta/airspeed/
    // gyro/air_density are upstream member state (angle_of_attack, beta,
    // airspeed, gyro, air_density) taken here as explicit parameters
    // instead - see file banner's "no singletons" note. Zero-force guard
    // (is_zero(airspeed)) ported exactly.
    [[nodiscard]] math::Vector3f getForce(float inputAileron, float inputElevator, float inputRudder,
                                           float alpha, float beta, float airspeed,
                                           const math::Vector3f& gyro, float air_density) const {
        const float c_drag_q = coefficient.c_drag_q;
        const float c_lift_q = coefficient.c_lift_q;
        const float s = coefficient.s;
        const float c = coefficient.c;
        const float b = coefficient.b;
        const float c_drag_deltae = coefficient.c_drag_deltae;
        const float c_lift_deltae = coefficient.c_lift_deltae;
        const float c_y_0 = coefficient.c_y_0;
        const float c_y_b = coefficient.c_y_b;
        const float c_y_p = coefficient.c_y_p;
        const float c_y_r = coefficient.c_y_r;
        const float c_y_deltaa = coefficient.c_y_deltaa;
        const float c_y_deltar = coefficient.c_y_deltar;

        const float rho = air_density;

        // request lift and drag alpha-coefficients from the corresponding functions
        const double c_lift_a = liftCoeff(alpha);
        const double c_drag_a = dragCoeff(alpha);

        // convert coefficients to the body frame
        const double c_x_a = -c_drag_a * std::cos(alpha) + c_lift_a * std::sin(alpha);
        const double c_x_q = -c_drag_q * std::cos(alpha) + c_lift_q * std::sin(alpha);
        const double c_z_a = -c_drag_a * std::sin(alpha) - c_lift_a * std::cos(alpha);
        const double c_z_q = -c_drag_q * std::sin(alpha) - c_lift_q * std::cos(alpha);

        // read angular rates
        const double p = gyro.x;
        const double q = gyro.y;
        const double r = gyro.z;

        // calculate aerodynamic force
        const double qbar = 1.0 / 2.0 * rho * std::pow(airspeed, 2) * s; // Calculate dynamic pressure
        double ax, ay, az;
        if (math::is_zero(airspeed)) {
            ax = 0;
            ay = 0;
            az = 0;
        } else {
            ax = qbar * (c_x_a + c_x_q * c * q / (2 * airspeed) - c_drag_deltae * std::cos(alpha) * std::fabs(inputElevator)
                         + c_lift_deltae * std::sin(alpha) * inputElevator);
            // split c_x_deltae to include "abs" term
            ay = qbar * (c_y_0 + c_y_b * beta + c_y_p * b * p / (2 * airspeed) + c_y_r * b * r / (2 * airspeed)
                         + c_y_deltaa * inputAileron + c_y_deltar * inputRudder);
            az = qbar * (c_z_a + c_z_q * c * q / (2 * airspeed) - c_drag_deltae * std::sin(alpha) * std::fabs(inputElevator)
                         - c_lift_deltae * std::cos(alpha) * inputElevator);
            // split c_z_deltae to include "abs" term
        }
        return math::Vector3f(static_cast<float>(ax), static_cast<float>(ay), static_cast<float>(az));
    }

    // Upstream: Plane::getTorque (SIM_Plane.cpp:273). The
    // `if (tailsitter || aerobatic)` effective-airspeed/alpha adjustment at
    // the top of upstream's version is EXCLUDED - see file banner - so
    // alpha/airspeed are used exactly as passed in. inputThrust is kept in
    // the signature to match upstream's own parameter list even though,
    // with that branch excluded, this slice never reads it - matches
    // upstream's exact interface shape for a future slice that re-adds
    // tailsitter/aerobatic support.
    [[nodiscard]] math::Vector3f getTorque(float inputAileron, float inputElevator, float inputRudder,
                                            [[maybe_unused]] float inputThrust, const math::Vector3f& force,
                                            float alpha, float airspeed, float beta,
                                            const math::Vector3f& gyro, float air_density) const {
        // calculate aerodynamic torque
        const float effective_airspeed = airspeed;

        const float s = coefficient.s;
        const float c = coefficient.c;
        const float b = coefficient.b;
        const float c_l_0 = coefficient.c_l_0;
        const float c_l_b = coefficient.c_l_b;
        const float c_l_p = coefficient.c_l_p;
        const float c_l_r = coefficient.c_l_r;
        const float c_l_deltaa = coefficient.c_l_deltaa;
        const float c_l_deltar = coefficient.c_l_deltar;
        const float c_m_0 = coefficient.c_m_0;
        const float c_m_a = coefficient.c_m_a;
        const float c_m_q = coefficient.c_m_q;
        const float c_m_deltae = coefficient.c_m_deltae;
        const float c_n_0 = coefficient.c_n_0;
        const float c_n_b = coefficient.c_n_b;
        const float c_n_p = coefficient.c_n_p;
        const float c_n_r = coefficient.c_n_r;
        const float c_n_deltaa = coefficient.c_n_deltaa;
        const float c_n_deltar = coefficient.c_n_deltar;
        const math::Vector3f& cg_offset = coefficient.cg_offset;

        const float rho = air_density;

        // read angular rates
        const double p = gyro.x;
        const double q = gyro.y;
        const double r = gyro.z;

        const double qbar = 1.0 / 2.0 * rho * std::pow(effective_airspeed, 2) * s; // Calculate dynamic pressure
        double la, na, ma;
        if (math::is_zero(effective_airspeed)) {
            la = 0;
            ma = 0;
            na = 0;
        } else {
            la = qbar * b
                 * (c_l_0 + c_l_b * beta + c_l_p * b * p / (2 * effective_airspeed) + c_l_r * b * r / (2 * effective_airspeed)
                    + c_l_deltaa * inputAileron + c_l_deltar * inputRudder);
            ma = qbar * c * (c_m_0 + c_m_a * alpha + c_m_q * c * q / (2 * effective_airspeed) + c_m_deltae * inputElevator);
            na = qbar * b
                 * (c_n_0 + c_n_b * beta + c_n_p * b * p / (2 * effective_airspeed) + c_n_r * b * r / (2 * effective_airspeed)
                    + c_n_deltaa * inputAileron + c_n_deltar * inputRudder);
        }

        // Add torque to force misalignment with CG
        // r x F, where r is the distance from CoG to CoL
        la += cg_offset.y * force.z - cg_offset.z * force.y;
        ma += -cg_offset.x * force.z + cg_offset.z * force.x;
        na += -cg_offset.y * force.x + cg_offset.x * force.y;

        return math::Vector3f(static_cast<float>(la), static_cast<float>(ma), static_cast<float>(na));
    }

    // Upstream: Aircraft::hagl() (SIM_Aircraft.cpp:145) / on_ground()
    // (SIM_Aircraft.cpp:153) - replaced by a flat-earth simplification, see
    // file banner's "GROUND MODEL" note. position.z follows NED convention
    // (down positive); the caller initializes position.z = -initial_altitude,
    // so position.z >= 0 means "at or below the starting ground plane".
    [[nodiscard]] bool on_ground() const { return position.z >= 0.0f; }

    // Upstream: Plane::calculate_forces (SIM_Plane.cpp:398) + the thrust-
    // scaling/ground-friction tail of it, folded together with
    // Aircraft::update_dynamics (SIM_Aircraft.cpp:709) into one per-tick
    // entry point - upstream's own Plane::update (SIM_Plane.cpp:522) does
    // the same two-call sequence (calculate_forces then update_dynamics),
    // just via a `struct sitl_input` this port has no equivalent of. Every
    // STANDARD-config branch is reproduced; every config-variant branch
    // (elevon/vtail/dspoilers/redundant/reverse_thrust/
    // reverse_elevator_rudder/tailsitter/aerobatic/launcher) is skipped -
    // see file banner.
    void update(float aileron, float elevator, float rudder, float throttle, float dt) {
        // calculate angle of attack (upstream: Plane::calculate_forces,
        // reading the PREVIOUS tick's velocity_air_bf - exactly reproduced:
        // velocity_air_bf here is only ever written by update_dynamics(),
        // at the end of the previous update() call, or left at its
        // zero-initialized default on the very first call).
        angle_of_attack = std::atan2(velocity_air_bf.z, velocity_air_bf.x);
        beta = std::atan2(velocity_air_bf.y, velocity_air_bf.x);

        const math::Vector3f force = getForce(aileron, elevator, rudder, angle_of_attack, beta, airspeed, gyro, air_density);
        math::Vector3f rot_accel = getTorque(aileron, elevator, rudder, throttle, force, angle_of_attack, airspeed, beta, gyro, air_density);

        // scale thrust to newtons - upstream: thrust_scale = (mass *
        // GRAVITY_MSS) / hover_throttle, computed once in Plane::Plane();
        // computed per-call here since mass/hover_throttle are plain public
        // fields a caller may change between calls.
        const float thrust_scale = (mass * kGravityMss) / hover_throttle;
        const float thrust_newtons = throttle * thrust_scale;

        accel_body = math::Vector3f(thrust_newtons, 0.0f, 0.0f) + force;
        accel_body = accel_body / mass;

        if (on_ground()) {
            // add some ground friction
            const math::Vector3f vel_body = dcm.transposed() * velocity_ef;
            accel_body.x -= vel_body.x * 0.3f;
        }

        update_dynamics(rot_accel, dt);
    }

    // Upstream: Aircraft::update_dynamics (SIM_Aircraft.cpp:709) - the
    // rigid-body integrator. Ported in full for the STANDARD config: gyro
    // integration + +-2000 deg/s clamp, body-accel +-64G clamp, DCM
    // rotate+normalize, body->earth accel rotation plus gravity, the
    // on-ground accel_earth.z clamp, accelerometer-equivalent accel_body
    // re-derivation, velocity/position integration, and
    // velocity_air_bf recomputation (wind=0, see file banner). The
    // eas2tas/air_density-from-altitude recompute, the entire
    // `switch (ground_behavior)` block, slung-payload/tether hooks, and
    // adjust_frame_time are excluded - see file banner.
    void update_dynamics(const math::Vector3f& rot_accel, float dt) {
        // update rotational rates in body frame
        gyro += rot_accel * dt;

        gyro.x = math::constrain_value(gyro.x, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.y = math::constrain_value(gyro.y, -math::radians(2000.0f), math::radians(2000.0f));
        gyro.z = math::constrain_value(gyro.z, -math::radians(2000.0f), math::radians(2000.0f));

        // limit body accel to 64G
        const float accel_limit = 64.0f * kGravityMss;
        accel_body.x = math::constrain_value(accel_body.x, -accel_limit, accel_limit);
        accel_body.y = math::constrain_value(accel_body.y, -accel_limit, accel_limit);
        accel_body.z = math::constrain_value(accel_body.z, -accel_limit, accel_limit);

        // update attitude
        dcm.rotate(gyro * dt);
        dcm.normalize();

        math::Vector3f accel_earth = dcm * accel_body;
        accel_earth += math::Vector3f(0.0f, 0.0f, kGravityMss);

        // if we're on the ground, then our vertical acceleration is limited
        // to zero. This effectively adds the force of the ground on the aircraft
        if (on_ground() && accel_earth.z > 0.0f) {
            accel_earth.z = 0.0f;
        }

        // work out acceleration as seen by the accelerometers. It sees the kinematic
        // acceleration (ie. real movement), plus gravity
        accel_body = dcm.transposed() * (accel_earth + math::Vector3f(0.0f, 0.0f, -kGravityMss));

        // new velocity vector
        velocity_ef += accel_earth * dt;

        // new position vector
        position += velocity_ef * dt;

        // velocity relative to airmass, body frame - wind_ef treated as
        // always zero this slice (velocity_air_ef == velocity_ef), see file
        // banner.
        velocity_air_bf = dcm.transposed() * velocity_ef;

        // Upstream: Aircraft::update_eas_airspeed() (SIM_Aircraft.cpp:1377),
        // airspeed = velocity_air_ef.length() / eas2tas with eas2tas held at
        // 1.0 this slice (see file banner's atmosphere-model exclusion).
        // BUGFIX during review: this assignment was missing from the initial
        // port - without it, airspeed stayed at its zero-initialized default
        // forever, so getForce/getTorque's is_zero(airspeed) guard was always
        // true and the aircraft never generated any aerodynamic force at all
        // (free-fall under gravity+thrust only). The 1-second sanity test's
        // -490m bound was too loose to catch this (free fall alone only
        // drops ~4.9m in 1s, well inside the old 10m margin).
        airspeed = velocity_air_bf.length();

        // constrain height to the ground - simplified flat-earth clamp
        // (upstream's real position.z snap-to-terrain and ground_behavior
        // switch are excluded, see file banner): don't let velocity carry
        // the aircraft further down once it's reached the ground plane.
        if (on_ground() && velocity_ef.z > 0.0f) {
            velocity_ef.z = 0.0f;
        }
    }

    // Aerodynamic/mass model - upstream: Plane::coefficient (assigned from
    // default_coefficients, or JSON-loaded - see file banner), Plane::mass
    // (Aircraft::mass, 2.0f), Plane::hover_throttle (const 0.7f).
    Coefficients coefficient;
    float mass;
    float hover_throttle;

    // True attitude - upstream: Aircraft::dcm (_dcm_matrix's SITL-truth
    // counterpart; SITL's own dcm, not AhrsDcm's dcm_matrix - see file
    // banner's "shares no code with the estimator" note).
    math::Matrix3f dcm;

    // True body-frame angular rate, rad/s - upstream: Aircraft::gyro.
    math::Vector3f gyro;

    // True body-frame acceleration, m/s^2 - upstream: Aircraft::accel_body.
    // Doubles as what an ideal accelerometer would read (kinematic +
    // gravity), matching upstream's own reuse of this one field for both
    // purposes.
    math::Vector3f accel_body;

    // True earth-frame (NED) velocity, m/s - upstream: Aircraft::velocity_ef.
    math::Vector3f velocity_ef;

    // True earth-frame (NED) position relative to the start point, m -
    // upstream: Aircraft::position (a Vector3p/postype_t upstream, for
    // long-duration precision; kept as plain Vector3f here - this slice has
    // no long-duration-precision requirement driving that choice, and
    // introducing FWCPP_POSTYPE_DOUBLE's postype_t here would coupled this
    // module to a build option it doesn't otherwise need).
    math::Vector3f position;

    // True body-frame airmass-relative velocity, m/s - upstream:
    // Aircraft::velocity_air_bf, with wind_ef treated as always zero this
    // slice (see file banner), so this equals dcm.transposed() *
    // velocity_ef exactly.
    math::Vector3f velocity_air_bf;

    // True angle of attack / sideslip, rad - upstream: Plane::angle_of_attack,
    // Plane::beta.
    float angle_of_attack = 0.0f;
    float beta = 0.0f;

    // True airspeed, m/s - upstream: Aircraft::airspeed, computed by
    // update_eas_airspeed() as velocity_air_ef.length() / eas2tas. eas2tas
    // is held at upstream's own pre-barometer default of 1.0 (no AP_Baro in
    // this port - see file banner), so this is simply
    // velocity_air_bf.length() and is recomputed as such by the caller (or
    // left at its zero-initialized default before the first update()).
    float airspeed = 0.0f;

    // Air density, kg/m^3 - upstream: Aircraft::air_density, normally
    // recomputed per-tick from altitude via AP_Baro. Fixed at sea-level
    // standard density this slice - see file banner.
    float air_density = kSslAirDensity;
};

} // namespace fwcpp::sim

// SLICE 2 NOTE: a higher-fidelity ap-sim would add (a) wind modeling -
// Aircraft::update_wind's turbulence + steady wind-vector model
// (SIM_Aircraft.cpp:888), feeding a real wind_ef into velocity_air_bf/
// airspeed/angle_of_attack/beta instead of this slice's wind_ef==0; (b) the
// ground_behavior variants (GROUND_BEHAVIOR_NO_MOVEMENT/FWD_ONLY/
// TAILSITTER) for realistic taxi/takeoff-roll behavior, once this port
// wants to simulate ground operations rather than just "don't sink through
// the floor"; and (c) the airframe config variants (elevons, vtail,
// dspoilers, tailsitter, aerobatic, reverse_thrust, ICEngine, launcher) -
// each is a self-contained addition to calculate_forces'/getTorque's
// existing branch points, not a redesign of what's here. None of (a)-(c)
// change update_dynamics' core rigid-body integration, which this slice
// already ports in full.
