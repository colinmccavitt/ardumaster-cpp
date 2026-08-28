#pragma once

// EkfCoreBackend: makes EkfCore (this port's NavEKF3-equivalent estimator,
// ekf_core.hpp) implement fwcpp::ahrs::AhrsBackend (CPP-079). CPP-080,
// phase 3 of vehicle-integration prep: CPP-078 collapsed Plane's four
// separate per-tick AhrsDcm calls into one update_full_cycle(); CPP-079
// defined AhrsBackend - the real, verified interface surface (one method
// plus six getters) - and made AhrsDcm implement it with ZERO behavior
// change (every override either an existing method reused verbatim, or a
// one-line getter returning an existing public field).
//
// THIS IS NOT THAT. AhrsDcm's own state (dcm_matrix/omega/accel_ef/
// roll/pitch/yaw) is exactly the state AhrsBackend's getters want, already
// sitting there as public fields - "implement the interface" meant "add a
// thin vtable in front of fields that already exist in the right shape".
// EkfCore's own StateVector (ekf_core.hpp, ~line 2108: quat/velocity/
// position/gyro_bias/accel_bias/earth_magfield/body_magfield/wind_vel) has
// NONE of dcm_matrix/omega/accel_ef - these three concepts do not exist
// anywhere in EkfCore today and are freshly derived here, every tick, from
// state.quat plus the raw IMU sample update_full_cycle() itself receives.
// That derivation - not a vtable - is this file's real content.
//
// CACHING, NOT ON-DEMAND COMPUTATION: AhrsBackend::get_omega()/
// get_dcm_matrix()/get_accel_ef() return `const math::Vector3f&`/
// `const math::Matrix3f&` - references, which a getter cannot manufacture
// by computing a fresh temporary on every call (it would dangle). So, like
// AhrsDcm's own dcm_matrix/omega/accel_ef, these three are real persistent
// member fields (dcm_matrix_/omega_/accel_ef_ below), recomputed exactly
// once per update_full_cycle() call and simply returned by reference
// thereafter. get_roll()/get_pitch()/get_yaw() return `float` BY VALUE, so
// they COULD be computed fresh in the getter body instead - this
// implementation caches them anyway (roll_/pitch_/yaw_, recomputed
// alongside the other three in the same recompute_cached_outputs() call),
// for one simple reason: AhrsDcm's own three Euler fields are also
// written once per tick, not derived lazily on read, so a caller reading
// get_roll() twice between two update_full_cycle() calls sees the same
// cost/behavior shape from either backend - no surprise "getters are O(1)
// field reads on AhrsDcm but O(trig) on EkfCoreBackend" asymmetry for code
// that might one day be written against `AhrsBackend&` generically. The
// trig cost of to_euler() is trivial either way; consistency with the
// reference-returning trio is the actual reason for caching these too.
//
// WHICH SAMPLE FEEDS get_omega()/get_accel_ef() - A REAL, DISCLOSED
// APPROXIMATION: EkfCore::tick() (ekf_core.hpp/.cpp, CPP-071) pushes the
// gyro/accel sample it's handed into an internal delay buffer
// (imu_buffer) and mechanizes the OLDEST buffered sample, not the one
// just pushed - see tick()'s own body (ekf_core.cpp) and its
// "CPP-071/073" banners for the full delayed-horizon design. EkfCore
// exposes no public accessor for that internally-delayed sample. Rather
// than reach into EkfCore's private buffer state (there is nothing public
// to reach into) or leave get_omega()/get_accel_ef() stale between ticks,
// this class derives them from THIS TICK'S OWN raw gyro_sample.gyro /
// accel_sample.accel (the direct filtered-rate/filtered-accel fields
// update_full_cycle() itself receives - the same fields AhrsDcm's own
// omega/accel_ef derivation reads, see ahrs_dcm.hpp's matrix_update()/
// accumulate_accel()), bias-corrected using the state's CURRENT (i.e.
// post-tick, reflecting whatever the delayed-sample mechanization just
// produced) gyro_bias/accel_bias estimate. This is a one-tick-ish
// approximation relative to "the exact sample tick() actually
// mechanized" - disclosed here, not hidden - and is the same kind of
// per-tick-current-sample convention AhrsBackend's interface already
// establishes for AhrsDcm (which has no delay buffer at all, so has no
// analogous tension).
//
// BIAS-CORRECTION CONVENTION - VERIFIED, NOT GUESSED: EkfCore's own
// gyro_bias/accel_bias are PER-SAMPLE DELTA biases (rad / m/s over
// whatever dt_ekf_avg they were estimated against), not rates - confirmed
// directly in update_strapdown_equations_ned() (ekf_core.cpp):
//   del_ang_corrected = gyro.delta_angle - state.gyro_bias * (gyro.delta_angle_dt / dt_ekf_avg);
//   del_vel_corrected = accel.delta_velocity - state.accel_bias * (accel.delta_velocity_dt / dt_ekf_avg);
// Dividing both sides by the sample's own dt gives the bias-corrected RATE
// upstream's mechanization actually integrates:
//   rate_corrected = raw_delta/raw_dt - state.bias/dt_ekf_avg
// which is exactly the "subtract state.bias/dt_ekf_avg from a raw rate"
// convention applied below to gyro_sample.gyro/accel_sample.accel (the
// port's own directly-sampled rate fields, used in preference to
// re-deriving a rate from delta_angle/delta_angle_dt a second time).
//
// math::Vector3f/Matrix3f (float, AhrsBackend's own fixed getter types)
// vs. this port's Vector3F/Matrix3F (Vector3<ftype>/Matrix3<ftype>,
// ftype = float unless the FWCPP_EKF_DOUBLE build option is on, see
// ekf_core.hpp): QuaternionF::rotation_matrix(Matrix3f&)/to_euler(float&,
// float&,float&) already have float-output overloads regardless of T
// (quaternion.hpp: "output precision is a caller choice, not tied to this
// quaternion's own T") so dcm_matrix_/roll_/pitch_/yaw_ need no manual
// conversion. omega_/accel_ef_ are computed in ftype precision (matching
// the bias-correction arithmetic's own natural type) and explicitly
// narrowed to float component-wise via the to_math_vec3f() helper below -
// a real, deliberate conversion, not an oversight, needed precisely
// because a double-build's Vector3F is a genuinely different type from
// math::Vector3f.
//
// ============================================================================
// THE FUSION-GATING DECISION (this ticket's own required, explicit design
// call): update_full_cycle() calls ONLY EkfCore::tick() (strapdown
// mechanization). It does NOT call fuse_gps_velocity()/fuse_gps_position()/
// fuse_magnetometer()/fuse_baro_height()/fuse_airspeed() at all. Two
// independent, each-sufficient-on-its-own reasons, both disclosed:
//
//   1. THE ALREADY-KNOWN GAP: EkfCore has no aiding-mode/health/GPS-quality
//      gating machinery (CPP-077 built tilt_align_complete/
//      del_ang_bias_learned but explicitly left readyToUseGPS()/
//      setAidingMode()/gpsGoodToAlign()'s ~482-line quality-check subsystem
//      unbuilt). Upstream's real AP_NavEKF3_core::UpdateFilter() only fuses
//      when that state machine says it's safe to - EkfCoreBackend has no
//      such gate to consult.
//
//   2. A NEWLY DISCOVERED, STRUCTURAL GAP (found while designing this
//      adapter, not assumed from the ticket): even IGNORING gap 1 and
//      fusing "unconditionally", AhrsBackend::update_full_cycle()'s own
//      parameter list cannot fully feed EkfCore's fusion surface:
//        - fuse_gps_position() needs a local-NE position (GpsSample::
//          position_ne, ekf_core.hpp). AhrsBackend's own ahrs::GpsSample
//          (ahrs_dcm.hpp) carries ground_speed_ms/ground_course_deg/
//          last_fix_time_ms/has_fix/velocity_ned/num_sats/has_3d_fix -
//          NO position field of any kind. There is no local-NE value to
//          hand fuse_gps_position() without inventing one from nothing.
//        - fuse_baro_height() needs a bare altitude (baro_altitude_m).
//          update_full_cycle()'s entire parameter list has no baro sample
//          or altitude value anywhere in it.
//      fuse_gps_velocity() (from ahrs::GpsSample::velocity_ned),
//      fuse_magnetometer() (from ahrs::CompassSample::field), and
//      fuse_airspeed() (from the existing airspeed_tas parameter) COULD
//      all be genuinely fed today - only 3 of the port's 4 real fusion
//      mechanisms. Wiring exactly those three and silently omitting the
//      other two would look like a deliberate, principled "fuse what's
//      healthy" gate to a future reader, when it is really just "fuse
//      whatever the interface happens to carry data for" - a materially
//      different, and materially less honest, thing to leave undisclosed.
//
//   Given both, this ticket keeps update_full_cycle() to mechanization
//   only. It exercises the genuinely new, substantial work this ticket is
//   actually about (state-representation conversion: quaternion state ->
//   dcm_matrix/omega/accel_ef/roll/pitch/yaw, plus a real
//   ahrs::GyroSample/AccelSample -> ekf::GyroSample/AccelSample sample-type
//   conversion) without also taking on an asymmetric, partially-fed fusion
//   wiring job that this ticket did not ask for and that would need its
//   own new data plumbing (a real baro sample, a real GPS
//   position/local-NE source) to do properly. Real fusion wiring - once
//   EkfCore's aiding-mode gating exists AND AhrsBackend's own interface
//   (or a wider replacement) can actually carry baro/GPS-position data -
//   is explicitly deferred to a future ticket, not attempted here.
// ============================================================================
//
// NOT ATTEMPTED HERE, named explicitly:
//   - Any fuse_*() wiring (see the fusion-gating decision above).
//   - Widening AhrsBackend's own interface to carry baro/GPS-position data
//     - a change to ahrs_backend.hpp, explicitly out of this ticket's
//     touch scope ("Do NOT touch AhrsBackend").
//   - Reaching into EkfCore's internal IMU delay buffer for the exact
//     sample tick() mechanized (see "WHICH SAMPLE" note above) - no public
//     accessor exists, and adding one would touch ekf_core.hpp for a
//     one-tick-ish precision gain this ticket doesn't need.
//   - Wiring EkfCoreBackend into Plane or any factory/selection logic -
//     purely additive, per the ticket's own scope.

#include <cstdint>

#include <fwcpp/ahrs/ahrs_dcm.hpp> // fwcpp::ahrs::AhrsBackend + GyroSample/AccelSample/CompassSample/GpsSample
#include <fwcpp/ekf/ekf_core.hpp>
#include <fwcpp/math/matrix3.hpp>
#include <fwcpp/math/scalar.hpp>
#include <fwcpp/math/vector3.hpp>

namespace fwcpp::ekf {

namespace detail {

// Component-wise precision conversion between this port's ftype-templated
// Vector3F (float unless FWCPP_EKF_DOUBLE is on) and AhrsBackend's fixed
// float math::Vector3f - see file banner. Trivial, but named/shared rather
// than inlined at each call site so the conversion is visibly deliberate
// everywhere it happens.
[[nodiscard]] inline math::Vector3f to_math_vec3f(const Vector3F& v) {
    return math::Vector3f(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}
[[nodiscard]] inline Vector3F to_ekf_vec3(const math::Vector3f& v) {
    return Vector3F(static_cast<ftype>(v.x), static_cast<ftype>(v.y), static_cast<ftype>(v.z));
}

} // namespace detail

// See file banner for the full design writeup (caching rationale, sample
// provenance for omega/accel_ef, bias-correction convention, precision
// conversion, and the fusion-gating decision). Matches this port's
// existing polymorphic-base-class idiom (AhrsBackend itself, ahrs_dcm.hpp:
// deleted copy ctor/assignment, `override` on every interface method).
class EkfCoreBackend : public fwcpp::ahrs::AhrsBackend {
public:
    // Default-constructs an owned EkfCore (EkfCore has no user-declared
    // constructor - StateVector{}/Matrix24{}/every other member already
    // has an in-class default, see ekf_core.hpp - so this is exactly
    // EkfCore's own "freshly initialized, identity attitude, zero
    // everything else" state) and seeds the cached getters from it, so a
    // caller reading get_dcm_matrix()/get_roll()/etc. before ever calling
    // update_full_cycle() sees a sane identity/zero state, not
    // uninitialized garbage - matching AhrsDcm's own constructor calling
    // dcm_matrix.identity() up front.
    EkfCoreBackend() { recompute_cached_outputs(ftype(0), math::Vector3f{}, math::Vector3f{}); }

    EkfCoreBackend(const EkfCoreBackend&) = delete;
    EkfCoreBackend& operator=(const EkfCoreBackend&) = delete;

    // Direct access to the owned EkfCore - not part of AhrsBackend, an
    // EkfCoreBackend-specific escape hatch so a caller (or a test) can
    // read/seed state (state.gyro_bias, P, health-adjacent fields as they
    // arrive in later tickets, etc.) that AhrsBackend's own six getters
    // deliberately don't expose. Mutable overload exists so tests can
    // seed state.gyro_bias/accel_bias directly before calling
    // update_full_cycle(), the same way ekf_core_test.cpp's existing
    // tests construct a bare EkfCore and poke its public `state` field.
    [[nodiscard]] EkfCore& ekf_core() { return ekf_core_; }
    [[nodiscard]] const EkfCore& ekf_core() const { return ekf_core_; }

    // Converts AhrsBackend's own sample shapes into EkfCore's (a genuinely
    // different set of structs, see ekf_backend.hpp's ticket/file banner -
    // ahrs::GyroSample carries delta_angle/dangle_dt/gyro, ekf::GyroSample
    // carries only delta_angle/delta_angle_dt), calls EkfCore::tick(), and
    // recomputes every cached getter output. compass/gps/fly_forward/
    // armed_and_safety_off/gps_use_enabled/wind_speed_ms/wind_estimate/
    // accel_healthy/ins_healthy/now_ms are intentionally unused parameters
    // (unnamed below) - see the file banner's "FUSION-GATING DECISION":
    // this ticket wires mechanization only, not fusion, so none of
    // AhrsDcm's drift-correction-only inputs are consumed here.
    // airspeed_tas is likewise unused today (fuse_airspeed() is not called
    // this ticket either) but is deliberately still named/visible in the
    // signature - only omitted from use, not from the interface it must
    // satisfy verbatim (AhrsBackend::update_full_cycle() is pure virtual;
    // this override's parameter list is fixed by the base class).
    void update_full_cycle(const fwcpp::ahrs::GyroSample& gyro_sample, const fwcpp::ahrs::AccelSample& accel_sample,
                            float dt, const fwcpp::ahrs::CompassSample& /*compass*/,
                            const fwcpp::ahrs::GpsSample& /*gps*/, bool /*fly_forward*/,
                            bool /*armed_and_safety_off*/, bool /*gps_use_enabled*/, float /*wind_speed_ms*/,
                            const math::Vector3f& /*wind_estimate*/, float /*airspeed_tas*/, bool /*accel_healthy*/,
                            bool /*ins_healthy*/, std::uint32_t /*now_ms*/) override {
        const ftype dt_ekf_avg = static_cast<ftype>(dt);

        // A REAL BUG FOUND WHILE WRITING THIS TICKET'S OWN TESTS, fixed
        // here: EkfCore::covariance_init(dt_ekf_avg) MUST run before the
        // first covariance_prediction() (every existing caller - e.g.
        // ekf_closed_loop_test.cpp - calls it exactly once, immediately
        // after construction, before any tick()). Without it, P starts
        // at all-zero (EkfCore's own Matrix24 P{} default), and
        // constrain_variances()'s real, CORRECT upstream fault-recovery
        // logic (ekf_core.cpp ~line 1177: P[13..15][13..15] underflowing
        // kMinSafeStateVar triggers `state.accel_bias.zero()`) sees that
        // all-zero variance as an underflow EVERY tick and silently
        // resets state.accel_bias to zero forever - not a bug in that
        // logic (it is upstream's real, intentional numerical safety
        // net), but a real bug in an EARLIER version of this adapter,
        // which never called covariance_init() at all and so had this
        // safety net permanently discard any accel_bias this class (or a
        // caller via ekf_core()) ever set. Lazily initialized on the
        // first update_full_cycle() call - dt_ekf_avg isn't known until
        // then - mirroring EkfCore::tick()'s own `imu_buffer_seeded`
        // lazy-first-tick pattern (ekf_core.cpp) rather than requiring a
        // guessed dt in this class's constructor.
        if (!covariance_initialized_) {
            ekf_core_.covariance_init(dt_ekf_avg);
            covariance_initialized_ = true;
        }

        GyroSample ekf_gyro;
        ekf_gyro.delta_angle = detail::to_ekf_vec3(gyro_sample.delta_angle);
        ekf_gyro.delta_angle_dt = static_cast<ftype>(gyro_sample.dangle_dt);

        AccelSample ekf_accel;
        ekf_accel.delta_velocity = detail::to_ekf_vec3(accel_sample.delta_velocity);
        ekf_accel.delta_velocity_dt = static_cast<ftype>(accel_sample.delta_velocity_dt);

        ekf_core_.tick(ekf_gyro, ekf_accel, dt_ekf_avg);

        recompute_cached_outputs(dt_ekf_avg, gyro_sample.gyro, accel_sample.accel);
    }

    [[nodiscard]] float get_roll() const override { return roll_; }
    [[nodiscard]] float get_pitch() const override { return pitch_; }
    [[nodiscard]] float get_yaw() const override { return yaw_; }
    [[nodiscard]] const math::Vector3f& get_omega() const override { return omega_; }
    [[nodiscard]] const math::Matrix3f& get_dcm_matrix() const override { return dcm_matrix_; }
    [[nodiscard]] const math::Vector3f& get_accel_ef() const override { return accel_ef_; }

private:
    // Recomputes every cached getter output from the CURRENT ekf_core_
    // state, plus this tick's raw gyro-rate/accel sample for the
    // bias-corrected omega_/accel_ef_ derivation - see file banner's
    // "WHICH SAMPLE" and "BIAS-CORRECTION CONVENTION" notes. Called from
    // both the constructor (dt_ekf_avg=0, zero samples - see ctor
    // comment) and update_full_cycle() (the real per-tick path).
    void recompute_cached_outputs(ftype dt_ekf_avg, const math::Vector3f& raw_gyro_rate,
                                   const math::Vector3f& raw_accel) {
        ekf_core_.state.quat.rotation_matrix(dcm_matrix_);
        ekf_core_.state.quat.to_euler(roll_, pitch_, yaw_);

        // dt_ekf_avg == 0 only on the constructor's seed call (state is
        // freshly default-constructed, gyro_bias/accel_bias already zero
        // then anyway) - guarded regardless, matching this port's own
        // is_zero()-before-divide convention used throughout ekf_core.cpp,
        // rather than relying on "the bias happens to be zero too".
        const bool have_dt = !fwcpp::math::is_zero(dt_ekf_avg);

        const Vector3F gyro_bias_rate = have_dt ? ekf_core_.state.gyro_bias / dt_ekf_avg : Vector3F{};
        const Vector3F omega_ekf = detail::to_ekf_vec3(raw_gyro_rate) - gyro_bias_rate;
        omega_ = detail::to_math_vec3f(omega_ekf);

        const Vector3F accel_bias_rate = have_dt ? ekf_core_.state.accel_bias / dt_ekf_avg : Vector3F{};
        const Vector3F accel_body_corrected = detail::to_ekf_vec3(raw_accel) - accel_bias_rate;
        // Rotated by dcm_matrix_ (just recomputed above from the CURRENT
        // quat) exactly like AhrsDcm's own `accel_ef = dcm_matrix *
        // sample.accel` (ahrs_dcm.hpp, accumulate_accel()) - gravity is
        // NOT removed, matching AhrsDcm's own documented convention.
        accel_ef_ = dcm_matrix_ * detail::to_math_vec3f(accel_body_corrected);
    }

    EkfCore ekf_core_;
    bool covariance_initialized_ = false; // see update_full_cycle()'s own comment on this field's purpose.

    math::Matrix3f dcm_matrix_;
    math::Vector3f omega_;
    math::Vector3f accel_ef_;
    float roll_ = 0.0f;
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
};

} // namespace fwcpp::ekf
